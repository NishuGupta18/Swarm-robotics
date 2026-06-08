#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D
from std_srvs.srv import SetBool
import numpy as np
import math
import paho.mqtt.client as mqtt
import json
import time

class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0: dt = 1e-3
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_out, self.max_out)
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return np.clip(output, -self.max_out, self.max_out)
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class MultiHolonomicController(Node):
    def __init__(self):
        super().__init__('multi_holonomic_controller')

        # --- MQTT Setup ---
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.connect("10.16.211.124", 1883, 8)
        self.mqtt_client.loop_start()
        self.pid={}
        # for robot_id in [0,2,4]:
        #     self.pid[robot_id]= {
        #         'linear': PID()
        #     }
        # --- Robot Configuration ---
        self.robot_ids = [4,0,2]
        self.pid_linear = {rid: PID(kp=0.4, ki=0.01, kd=0.05, max_out=10.0) for rid in self.robot_ids}
        self.pid_theta = {rid: PID(kp=0.24, ki=0.002, kd=0.06, max_out=10.0) for rid in self.robot_ids}
        self.robot_ids = [4,0,2]
        # Inside __init__
        self.active_drop_goals = {'red': None, 'green': None, 'blue': None}
        self.delivered_crate_ids = {'red': [], 'green': [], 'blue': []}
        self.robots = {
            rid: {
                'state': 'idle',
                'substate': 'idle',
                'current_pose': {'x': 0.0, 'y': 0.0, 'w': 0.0},
                'home_pose': self.get_home_pose(rid),
                'assigned_crate': None,
                'mag_status': 0,
                'current_base': 100.0, # For smooth interpolation
                'current_elbow': 45.0,
                'target_base': 100.0,
                'target_elbow': 45.0,
                'servo_step_size': 2.5,
                'timer_start': 0.0,
                'align_stable_counter': 0,
                'mag_service_called': False,
                'pick_mag_triggered': False
            } for rid in self.robot_ids
        }

        # self.pid = {}
        # for robot_id in [4, 0, 2]:
        #     self.pid[robot_id] = {
        #         'linear': PID(**self.pid_params['linear']),
        #         'theta': PID(**self.pid_params['theta'])
        #     }

        self.approach_distance = 170.0
        self.drop_zones = {'red': (1215, 1000, 0.0), 'green': (820, 1900, 0.0), 'blue': (1616, 1900, 0.0)}
        self.avoidance_radius = 250.0
        self.mag_servers = {}
        for rid in self.robot_ids:
            # We use a lambda to "bake" the robot_id into the callback
            self.mag_servers[rid] = self.create_service(
                SetBool,
                f'/bot_{rid}/attach_crate',
                lambda req, res, rid=rid: self.mag_service_cb(req, res, rid)
        )

        self.mag_clients = {rid: self.create_client(SetBool, f'/bot_{rid}/attach_crate') for rid in self.robot_ids}
        self.pose_sub = self.create_subscription(Poses2D, '/bot_pose', self.pose_cb, 10)
        self.crate_sub = self.create_subscription(Poses2D, '/crate_pose', self.crate_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.all_crates = {}
        self.last_time = time.time()
        self.get_logger().info("Multi-Bot Hardware Controller Started.")
    def mag_service_cb(self, request, response, robot_id):
        """Callback for the magnet service of a specific robot."""
        try:
            # Update the local status so the next MQTT message sends the correct state
            self.robots[robot_id]['mag_status'] = 1 if request.data else 0
            
            response.success = True
            self.get_logger().info(f"Magnet for Robot {robot_id} set to: {request.data}")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Service failed for Robot {robot_id}: {str(e)}")
            
        return response
    
    
    def get_home_pose(self, rid):
        if rid == 0: return (1218.0, 205.0, 0.0)
        if rid == 2: return (1568.0, 205.0, 0.0)
        return (846.0, 205.0, 0.0)

    def trigger_mag_service(self, rid, state_bool):
        if not self.mag_clients[rid].service_is_ready(): return
        req = SetBool.Request()
        req.data = state_bool
        self.mag_clients[rid].call_async(req)
        # Update local status immediately for MQTT sync
        self.robots[rid]['mag_status'] = 1 if state_bool else 0

    def pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id in self.robots:
                self.robots[pose.id]['current_pose'] = {'x': pose.x, 'y': pose.y, 'w': pose.w}

    def crate_cb(self, msg):
        for pose in msg.poses:
            if pose.id > 10 and pose.id != 17:
                # 1. Always update the global coordinate map first
                if pose.id not in self.all_crates:
                    color = self.get_crate_color(pose.id)
                    self.all_crates[pose.id] = {'x': pose.x, 'y': pose.y, 'color': color, 'assigned': False, 'completed': False}
                else:
                    self.all_crates[pose.id]['x'] = pose.x
                    self.all_crates[pose.id]['y'] = pose.y

                # 2. Check Proximity to the assigned Drop Zone
                color = self.all_crates[pose.id]['color']
                zx, zy, _ = self.drop_zones[color]
                dist_to_zone = math.hypot(pose.x - zx, pose.y - zy)

                # 3. Logic Trigger: If crate is in zone and NO robot is currently holding it
                if dist_to_zone < 300.0 : #and not self.all_crates[pose.id]['assigned']:
                    
                    # Register the crate if it's new to the zone
                    if pose.id not in self.delivered_crate_ids[color]:
                        # self.delivered_crate_ids[color].append(pose.id)
                        # self.all_crates[pose.id]['completed'] = True
                        self.get_logger().info(f"Crate {pose.id} confirmed in {color} zone.")

                    placed_list = self.delivered_crate_ids[color]
                    count = len(placed_list)

                    if count == 1:
                        # Target for 2nd crate = 1st crate position + 50mm
                        c1 = self.all_crates[placed_list[0]]
                        self.active_drop_goals[color] = (c1['x'] - 80.0, zy)
                        self.get_logger().info(f"[DROP GOAL] color={color} crate#2 goal → {self.active_drop_goals[color]}")
                        
                    elif count == 2:
                        # Target for 3rd crate = Midpoint of 1st and 2nd
                        c1 = self.all_crates[placed_list[0]]
                        c2 = self.all_crates[placed_list[1]]
                        self.active_drop_goals[color] = (((c1['x'] + c2['x']) / 2.0)-20, zy)
                        self.get_logger().info(
                           f"[DROP GOAL] Color={color} crate#3 goal → {self.active_drop_goals[color]}"
                        )
            self.assign_tasks()
                    # ------------------------------

    def get_crate_color(self, crate_id):
        if crate_id % 3 == 0: return 'red'
        elif crate_id % 3 == 1: return 'green'
        else: return 'blue'

    def assign_tasks(self, rid=None):
        # Determine which robots need a task
        rids_to_assign = [rid] if rid is not None else self.robot_ids
        
        # Filter for crates that are not assigned AND not completed
        available_crates = [cid for cid, c in self.all_crates.items() 
                            if not c['assigned'] and not c['completed']]

        for r_id in rids_to_assign:
            bot = self.robots[r_id]
            if bot['state'] != 'idle': continue # Only assign to idle robots
            if not available_crates: break
            
            # Find nearest available crate
            min_dist = float('inf')
            best_cid = None
            for cid in available_crates:
                crate = self.all_crates[cid]
                dist = math.sqrt((crate['x'] - bot['current_pose']['x'])**2 + 
                                (crate['y'] - bot['current_pose']['y'])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_cid = cid
            
            if best_cid is not None:
                bot['assigned_crate'] = best_cid
                bot['state'] = 'NAVIGATING'
                self.all_crates[best_cid]['assigned'] = True
                available_crates.remove(best_cid)
                self.get_logger().info(f"Reassigned Robot {r_id} -> Crate {best_cid}")
    def control_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        for rid, bot in self.robots.items():
            self.process_robot(rid, bot, dt, now)

    def process_robot(self, rid, bot, dt, now):
        state = bot['state']
        if state == 'idle': return

        pose = bot['current_pose']
        cx, cy, cw = pose['x'], pose['y'], pose['w']

        if state == 'NAVIGATING':
            if bot['assigned_crate'] not in self.all_crates:
                self.get_logger().warn(
                    f"Robot {rid}: Assigned crate lost, waiting..."
                )
                self.send_kinematics(rid, 0, 0, 0)
                return
            crate = self.all_crates[bot['assigned_crate']]
            # self.pid[rid]['theta'].reset()
            dx, dy = crate['x'] - cx, crate['y'] - cy
            dist = math.sqrt(dx**2 + dy**2)

            if dist < self.approach_distance:
                bot['state'] = 'ALIGNING'
                bot['align_stable_counter'] = 0
                self.send_kinematics(rid, 0, 0, 0)
                return

            velocity = self.pid_linear[rid].compute(dist, dt)
            vx_w = (dx/dist)*velocity
            vy_w = (dy/dist)*velocity
            
            bot_theta = self.normalize_angle(cw)
            t_rad = math.radians(cw)
            # Simple Avoidance Logic
            vx,vy = self.get_avoidance_vector(rid)
            vx_final_w = vx_w + vx
            vy_final_w = vy_w + vy
            vx_r = vx_final_w * math.cos(t_rad) + vy_final_w * math.sin(t_rad)
            vy_r = -vx_final_w * math.sin(t_rad) + vy_final_w * math.cos(t_rad)
            w_c = self.pid_theta[rid].compute(bot_theta, dt)
            # w_c = ((0 - cw) %360-180)* 0.2
            self.send_kinematics(rid,vx_r,vy_r,w_c)

        elif state == 'ALIGNING':
            crate = self.all_crates[bot['assigned_crate']]
            dx,dy = cx-crate['x'] , cy-crate['y']
            angle_to_crate = math.degrees(math.atan2(-dy, dx)) - 8.0
            bot_theta=self.normalize_angle(cw)
            arm_angle= bot_theta + 90.0
            angle_error = self.normalize_angle(arm_angle-angle_to_crate)

            align_tol_deg = 5.0
            w_out = self.pid_theta[rid].compute(angle_error, dt)
            self.send_kinematics(rid, 0, 0, w_out)

            if abs(angle_error) < align_tol_deg: bot['align_stable_counter'] += 1
            else: bot['align_stable_counter'] = 0

            if bot['align_stable_counter'] >= 3 and abs(angle_error) < 12.0:
                self.pid_theta[rid].reset()
                self.trigger_mag_service(rid, True)  # Ensure magnet is off before picking
                bot['state'] = 'PICKING'; bot['substate'] = 'idle'
                
        elif state == 'PICKING':
            self.handle_pick_sequence(rid, bot, now)

        elif state == 'MOVE_TO_ZONE':
            pose = bot['current_pose']
            cx, cy, cw = pose['x'], pose['y'], pose['w']
            crate = self.all_crates[bot['assigned_crate']]
            zx, zy, _ = self.drop_zones[crate['color']]
            color = crate['color']
            if self.active_drop_goals[color] is not None:
                target_zx,target_zy=self.active_drop_goals[color]
            else:
                target_zx, target_zy = zx, zy
            dx, dy = target_zx - cx, target_zy - cy
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 40.0:
                self.get_logger().info(f"Robot {rid} has reached the drop zone...")
                bot['state'] = 'PLACING'; bot['substate'] = 'idle'
                self.pid_theta[rid].reset()
                self.pid_linear[rid].reset()
                bot['place_start'] = now
                return

            v_mag = self.pid_linear[rid].compute(dist, dt) # Speed to zone
            vx_w = (dx/dist)*v_mag; vy_w = (dy/dist)*v_mag
            vx,vy = self.get_avoidance_vector(rid)
            bot_theta = self.normalize_angle(cw)
            # w_c = ((0 - cw) % 360 - 180) * 0.05
            w_c = self.pid_theta[rid].compute(bot_theta, dt)
            self.send_kinematics(rid, vx_w + vx, vy_w +vy, w_c)

        elif state == 'PLACING':
            self.send_speeds(rid, 0, 0, 0)
            self.handle_place_sequence(rid, bot, now)

        elif state == 'RETURN_HOME':
            hx, hy, _ = bot['home_pose']
            dx, dy = hx - cx, hy - cy
            dist = math.sqrt(dx**2 + dy**2)
            if dist < 70.0: 
                bot['state'] = 'idle'
                self.send_speeds(rid, 0, 0, 0)
                self.trigger_mag_service(rid, False)
                self.send_speeds(rid, 0, 0,0) # Reset arm position
                self.assign_tasks(rid)
            else:
                t_rad = math.radians(cw)
                # vx,vy = self.get_avoidance_vector(rid)
                # v_mag = self.pid_linear[rid].compute(dist, dt)
                # vx_w = (dx/dist)*v_mag; vy_w = (dy/dist)*v_mag
                # w_c = (((0 - cw)%360) -180)* 0.05
                # self.send_kinematics(rid, vx_w + vx, vy_w + vy, w_c)
                velocity = self.pid_linear[rid].compute(dist, dt)
                vx_w = (dx/dist)*velocity
                vy_w = (dy/dist)*velocity
                vx, vy = self.get_avoidance_vector(rid)
                vx_final_w = vx_w + vx
                vy_final_w = vy_w + vy
                vx_r = vx_final_w * math.cos(t_rad) + vy_final_w * math.sin(t_rad)
                vy_r = -vx_final_w * math.sin(t_rad) + vy_final_w * math.cos(t_rad)
                bot_theta = self.normalize_angle(cw)
                # w_c = ((0 - cw) %360-180)* 0.05
                w_c = self.pid_theta[rid].compute(bot_theta, dt)
                self.send_kinematics(rid,vx_r,vy_r,w_c)

    def handle_pick_sequence(self, rid, bot, now):
        if bot['substate'] == 'idle':
            bot['substate'] = 'lowering'; bot['timer_start'] = now
            bot['pick_mag_triggered'] = False
            return
        
        elapsed = now - bot['timer_start']
        
        if bot['substate'] == 'lowering':
            self.send_speeds(rid, 0, 0, 0, base=175.0, elbow=120.0)
            if not bot['pick_mag_triggered'] and elapsed > 0.5:
                self.get_logger().info(f"Robot {rid} is triggering the magnet...")
                self.trigger_mag_service(rid, True)
                bot['pick_mag_triggered'] = True
            
            if elapsed > 3.0: # Slow movement time
                self.get_logger().info(f"Robot {rid} is lowering...")
                bot['substate'] = 'attracting'; bot['timer_start'] = now
                bot['mag_service_called'] = True
                
        elif bot['substate'] == 'attracting':
            # if not bot['mag_service_called']:
            #     self.trigger_mag_service(rid, True)
            #     bot['mag_service_called'] = False
            
            if elapsed > 2.0:
                bot['substate'] = 'lifting'; bot['timer_start'] = now
                
        elif bot['substate'] == 'lifting':
            self.get_logger().info(f"Robot {rid} is lifting..")
            self.send_speeds(rid, 0, 0, 0, base=120.0, elbow=70.0)
            if elapsed > 3.0:
                self.get_logger().info(f"Robot {rid}'s pickup complete.")
                bot['state'] = 'MOVE_TO_ZONE'; bot['substate'] = 'idle'

    def handle_place_sequence(self, rid, bot, now):
        if bot['substate'] == 'idle':
            bot['substate'] = 'lowering'; bot['timer_start'] = now
            bot['mag_service_called'] = False
            return
        
        elapsed = now - bot['timer_start']
        crate_id = bot['assigned_crate']
        color = self.all_crates[crate_id]['color']
        stack_height = len(self.delivered_crate_ids[color])
        self.get_logger().info(f"Robot {rid} is placing {color} crate...")

        # Logic: If 2 crates are already there, this is the 3rd (the topper)
        if stack_height == 2:
            target_base = 140.0   # Stacking height
            target_elbow = 80.0
        else:
            target_base = 170.0   # Ground level
            target_elbow = 120.0

        if bot['substate'] == 'lowering':
            # Keep sending the target angles
            self.send_speeds(rid, 0, 0, 0, base=target_base, elbow=target_elbow)
            # ONLY move to release if time has passed
            if elapsed > 3.0:
                bot['substate'] = 'release'; bot['timer_start'] = now

        elif bot['substate'] == 'release':
            if not bot['mag_service_called']:
                self.trigger_mag_service(rid, False)
                bot['mag_service_called'] = True
                crate_id = bot['assigned_crate']
                color = self.all_crates[crate_id]['color']

                if crate_id not in self.delivered_crate_ids[color]:
                    self.delivered_crate_ids[color].append(crate_id)
                    self.all_crates[crate_id]['completed'] = True

                # count = len(self.delivered_crate_ids[color])

                # if count == 1:
                #     c1 = self.all_crates[self.delivered_crate_ids[color][0]]
                #     self.active_drop_goals[color] = (c1['x'] + 50, c1['y'])

                # elif count == 2:
                #     c1 = self.all_crates[self.delivered_crate_ids[color][0]]
                #     c2 = self.all_crates[self.delivered_crate_ids[color][1]]
                #     self.active_drop_goals[color] = ((c1['x']+c2['x'])/2, ((c1['y']+c2['y'])/2)-20)
            if elapsed > 1.5: # Give it a bit more time to settle
                bot['substate'] = 'lifting'; bot['timer_start'] = now
                
        elif bot['substate'] == 'lifting':
            # Lift high enough to clear the new stack height!
            self.send_speeds(rid, 0, 0, 0, base=120.0, elbow=70.0)
            if elapsed > 2.5:
                bot['state'] = 'RETURN_HOME'; bot['substate'] = 'idle'
                

    def get_avoidance_vector(self, robot_id):
        if self.robots[robot_id]['state'] in ['PLACING', 'PICKING']:
            return 0.0,0.0
            

        robot = self.robots[robot_id]
        cx, cy = robot['current_pose']['x'], robot['current_pose']['y']
        
        # Initialize avoidance forces as a vector (X, Y)
        avoid_vx, avoid_vy = 0.0, 0.0
        
        # --- 1. Bot-to-Bot Avoidance ---
        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue
            ox, oy = other['current_pose']['x'], other['current_pose']['y']
            dx, dy = ox - cx, oy - cy
            dist = math.sqrt(dx**2 + dy**2)
            
            if 0 < dist < 300.0: # Detection radius
                force = (300.0 - dist) * 0.2 # Gain
                angle = math.atan2(dy, dx)
                # Push in the opposite direction
                avoid_vx -= math.cos(angle) * force
                avoid_vy -= math.sin(angle) * force

        # --- Simple Crate Repulsion ---
        for cid, crate in self.all_crates.items():
            
            # 1. Ignore the crate this robot is supposed to pick up
            if cid == robot.get('assigned_crate'):
                continue

            # 2. Calculate distance from Robot to Crate
            dx = cx - crate['x']  # Positive if robot is to the right of crate
            dy = cy - crate['y']  # Positive if robot is above crate
            dist = math.sqrt(dx**2 + dy**2)

            # 3. If the robot is closer than 300mm, push away
            repel_threshold = 190.0 
            if 0 < dist < repel_threshold:
                # The closer the robot is, the stronger it pushes away
                push_strength = (repel_threshold - dist) * 0.15
                # angle = math.atan2(dy, dx)
                # Push in the opposite direction
                # avoid_vx -= math.cos(angle) * push_strength
                # avoid_vy -= math.sin(angle) * push_strength
                
                # Add the "Push" to the avoidance velocity
                avoid_vx += (dx / dist) * push_strength
                avoid_vy += (dy / dist) * push_strength

        return avoid_vx, avoid_vy
 

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

    def send_kinematics(self, rid, vx_r, vy_r, w_c):
        s1 = (-(vx_r/3.0)) + (math.sqrt(3)/3.0 * vy_r) + (w_c/3.0)
        s2 = (-(vx_r/3.0)) - (math.sqrt(3)/3.0 * vy_r) + (w_c/3.0)
        s3 = (2.0*vx_r/3.0) + (w_c/3.0)
        self.send_speeds(rid, s1, s2, s3)

    def send_speeds(self, rid, s1, s2, s3, base=None, elbow=None):
        bot = self.robots[rid]
        if base is not None: bot['target_base'] = base
        if elbow is not None: bot['target_elbow'] = elbow

        # Smooth Interpolation logic per robot
        if bot['current_base'] < bot['target_base']:
            bot['current_base'] = min(bot['current_base'] + bot['servo_step_size'], bot['target_base'])
        elif bot['current_base'] > bot['target_base']:
            bot['current_base'] = max(bot['current_base'] - bot['servo_step_size'], bot['target_base'])

        if bot['current_elbow'] < bot['target_elbow']:
            bot['current_elbow'] = min(bot['current_elbow'] + bot['servo_step_size'], bot['target_elbow'])
        elif bot['current_elbow'] > bot['target_elbow']:
            bot['current_elbow'] = max(bot['current_elbow'] - bot['servo_step_size'], bot['target_elbow'])

        payload = {
            "s1": round(s1, 2), "s2": round(s2, 2), "s3": round(s3, 2),
            "mag": int(bot['mag_status']),
            "base": round(bot['current_base'], 1), 
            "elbow": round(bot['current_elbow'], 1)
        }
        self.mqtt_client.publish(f"bot_{rid}/cmd_vel", json.dumps(payload))

def main():
    rclpy.init()
    node = MultiHolonomicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all robots on exit
        for rid in node.robot_ids:
            node.send_speeds(rid, 0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()