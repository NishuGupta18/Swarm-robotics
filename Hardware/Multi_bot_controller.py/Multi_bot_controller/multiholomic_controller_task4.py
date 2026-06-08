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
        self.mqtt_client.connect("10.216.105.124", 1883, 10)
        self.mqtt_client.loop_start()
        self.pid={}
        # for robot_id in [0,2,4]:
        #     self.pid[robot_id]= {
        #         'linear': PID()
        #     }
        # --- Robot Configuration ---
        self.robot_ids = [0, 2,4]
        self.pid_linear = {rid: PID(kp=0.4, ki=0.01, kd=0.05, max_out=10.0) for rid in self.robot_ids}
        self.pid_theta = {rid: PID(kp=0.24, ki=0.002, kd=0.06, max_out=6.0) for rid in self.robot_ids}
        self.robot_ids = [0, 2,4]
            
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

        self.approach_distance = 185.0
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
        """Update crate positions, mark invisible ones as potentially picked up"""
        
        # Initialize all_crates only once
        if not hasattr(self, 'all_crates'):
            self.all_crates = {}
        
        # Get currently visible crate IDs
        visible_crate_ids = set()
        
        for pose in msg.poses:
            if pose.id >= 5:  # Crate IDs
                visible_crate_ids.add(pose.id)
                color = self.get_crate_color(pose.id)
                
                if pose.id in self.all_crates:
                    # Update position and mark as visible
                    self.all_crates[pose.id]['x'] = pose.x
                    self.all_crates[pose.id]['y'] = pose.y
                    self.all_crates[pose.id]['w'] = pose.w
                    self.all_crates[pose.id]['visible'] = True
                else:
                    # Add new crate
                    self.all_crates[pose.id] = {
                        'x': pose.x,
                        'y': pose.y,
                        'w': pose.w,
                        'color': color,
                        'assigned': False,
                        'visible': True,
                        'picked_up': False
                    }
        
        # Mark crates not seen as potentially picked up (if assigned)
        for crate_id, crate in self.all_crates.items():
            if crate_id not in visible_crate_ids:
                if crate['assigned']:
                    crate['visible'] = False
                    crate['picked_up'] = True  # Assumed picked up if assigned and not visible
        
        # Trigger task assignment once on first detection
        if not hasattr(self, 'tasks_assigned') and len(self.all_crates) >= 1:
            self.get_logger().info(f" Detected {len(self.all_crates)} crates - starting allocation...")
            self.assign_tasks()
            self.tasks_assigned = True

    def get_crate_color(self, crate_id):
        if crate_id % 3 == 0: return 'red'
        elif crate_id % 3 == 1: return 'green'
        else: return 'blue'

    def assign_tasks(self,rid=None):
        """
        Greedy algorithm: Assign each crate to nearest available bot
        """
        self.get_logger().info(" Starting task allocation...")
        
        available_crates = list(self.all_crates.keys())
        
        for crate_id in available_crates:
            crate = self.all_crates[crate_id]
            
            # Find nearest idle robot
            min_distance = float('inf')
            nearest_robot = None
            
            for robot_id, robot in self.robots.items():
                if robot['assigned_crate'] is not None:
                    continue  # Skip if already assigned
                
                # Calculate distance
                dx = crate['x'] - robot['current_pose']['x']
                dy = crate['y'] - robot['current_pose']['y']
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < min_distance:
                    min_distance = distance
                    nearest_robot = robot_id   
            
            if nearest_robot is not None:
                # Assign crate to this robot
                self.robots[nearest_robot]['assigned_crate'] = crate_id
                self.robots[nearest_robot]['state'] = 'NAVIGATING'
                # self.robots[nearest_robot]['substate'] = 'approach'
                self.all_crates[crate_id]['assigned'] = True
                
                
                self.get_logger().info(
                    f"Robot {nearest_robot} assigned crate {crate_id} "
                    f"({crate['color']}) at distance {min_distance:.1f}mm"
                )
        unassigned = [cid for cid, c in self.all_crates.items() if not c['assigned']]
        if unassigned:
            self.get_logger().warn(f" {len(unassigned)} crates remain unassigned: {unassigned}")
            self.remaining_crates = unassigned
        else:
            self.remaining_crates = []
            self.get_logger().info(" All crates assigned!")
        
        self.get_logger().info(" Task allocation complete!")
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
            
            t_rad = math.radians(cw)
            # Simple Avoidance Logic
            vx, vy = self.get_avoidance_vector(rid)
            vx_final_w = vx_w + vx
            vy_final_w = vy_w + vy
            vx_r = vx_final_w * math.cos(t_rad) + vy_final_w * math.sin(t_rad)
            vy_r = -vx_final_w * math.sin(t_rad) + vy_final_w * math.cos(t_rad)
            w_c = ((0 - cw) %360-180)* 0.05
            self.send_kinematics(rid,vx_r,vy_r,w_c)

        elif state == 'ALIGNING':
            crate = self.all_crates[bot['assigned_crate']]
            dx,dy = cx-crate['x'] , cy-crate['y']
            angle_to_crate = math.degrees(math.atan2(-dy, dx))
            bot_theta=self.normalize_angle(cw)
            arm_angle= bot_theta + 90.0
            angle_error = self.normalize_angle(arm_angle-angle_to_crate)

            align_tol_deg = 5.0
            w_out = self.pid_theta[rid].compute(angle_error, dt)
            self.send_kinematics(rid, 0, 0, w_out)

            if abs(angle_error) < align_tol_deg: bot['align_stable_counter'] += 1
            else: bot['align_stable_counter'] = 0

            if bot['align_stable_counter'] >= 3 or abs(angle_error) < 8.0:
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
            dx, dy = zx - cx, zy - cy
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 70.0:
                self.get_logger().info(f"Robot {rid} has reached the drop zone...")
                bot['state'] = 'PLACING'; bot['substate'] = 'idle'
                self.pid_theta[rid].reset()
                self.pid_linear[rid].reset()
                bot['place_start'] = now
                return

            v_mag = self.pid_linear[rid].compute(dist, dt) # Speed to zone
            vx_w = (dx/dist)*v_mag; vy_w = (dy/dist)*v_mag
            vx, vy = self.get_avoidance_vector(rid)
            w_c = ((0 - cw) % 360 - 180) * 0.05
            self.send_kinematics(rid, vx_w + vx, vy_w + vy, w_c)

        elif state == 'PLACING':
            self.handle_place_sequence(rid, bot, now)

        elif state == 'RETURN_HOME':
            hx, hy, _ = bot['home_pose']
            dx, dy = hx - cx, hy - cy
            dist = math.sqrt(dx**2 + dy**2)
            if dist < 50.0: 
                bot['state'] = 'idle'
                self.send_speeds(rid, 0, 0, 0)
                self.assign_tasks(rid)
            else:
                vx,vy = self.get_avoidance_vector(rid)
                w_c = (((0 - cw)%360) -180)* 0.05
                self.send_kinematics(rid, (dx/dist)*8.0+vx, (dy/dist)*8.0+vy, w_c)

    def handle_pick_sequence(self, rid, bot, now):
        if bot['substate'] == 'idle':
            bot['substate'] = 'lowering'; bot['timer_start'] = now
            bot['pick_mag_triggered'] = False
            return
        
        elapsed = now - bot['timer_start']
        
        if bot['substate'] == 'lowering':
            self.send_speeds(rid, 0, 0, 0, base=150.0, elbow=45.0)
            if not bot['pick_mag_triggered'] and elapsed > 0.5:
                self.get_logger().info(f"Robot {rid} is triggering the magnet...")
                self.trigger_mag_service(rid, True)
                bot['pick_mag_triggered'] = True
            
            if elapsed > 4.0: # Slow movement time
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
            self.get_logger().info(f"Robot {rid} is lifting...")
            self.send_speeds(rid, 0, 0, 0, base=100.0, elbow=80.0)
            if elapsed > 3.0:
                self.get_logger().info(f"Robot {rid}'s pickup complete.")
                bot['state'] = 'MOVE_TO_ZONE'; bot['substate'] = 'idle'

    def handle_place_sequence(self, rid, bot, now):
        if bot['substate'] == 'idle':
            bot['substate'] = 'lowering'; bot['timer_start'] = now
            bot['mag_service_called'] = False
            return
        
        elapsed = now - bot['timer_start']

        if bot['substate'] == 'lowering':
            self.send_speeds(rid, 0, 0, 0, base=180.0, elbow=100.0)
            if elapsed > 3.0:
                bot['substate'] = 'release'; bot['timer_start'] = now

        elif bot['substate'] == 'release':
            if not bot['mag_service_called']:
                self.trigger_mag_service(rid, False)
                bot['mag_service_called'] = True
            if elapsed > 1.0:
                bot['substate'] = 'lifting'; bot['timer_start'] = now
                
        elif bot['substate'] == 'lifting':
            self.send_speeds(rid, 0, 0, 0, base=80.0, elbow=90.0)
            if elapsed > 2.0:
                bot['state'] = 'RETURN_HOME'; bot['substate'] = 'idle'

    def get_avoidance_vector(self, robot_id):
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
            
            if 0 < dist < 350.0: # Detection radius
                force = (350.0 - dist) * 0.2  # Gain
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
            dist = math.hypot(dx, dy)

            # 3. If the robot is closer than 300mm, push away
            repel_threshold = 250.0 
            if 0 < dist < repel_threshold:
                # The closer the robot is, the stronger it pushes away
                push_strength = (repel_threshold - dist) * 0.5
                
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