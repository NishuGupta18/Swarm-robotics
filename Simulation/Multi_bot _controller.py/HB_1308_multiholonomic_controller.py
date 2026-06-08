#!/usr/bin/env python3

'''
This Python file runs a ROS 2 node of name holonomic_pid_controller which holds the position of a holonomic robot
and drives it through a series of predefined goals using PID controllers on [x, y, θ].

This node publishes and subscribes to the following topics:

        PUBLICATIONS                               SUBSCRIPTIONS
        /forward_velocity_controller/commands      /bot_pose

Instead of defining separate variables for each PID axis, lists/dictionaries are used.
For example: pid_params['x'], pid_params['y'], pid_params['theta'], etc.

Code modularity and clarity are maintained to make tuning and extension easier.
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hb_interfaces.msg import Pose2D, Poses2D
import numpy as np
import math
from hb_interfaces.msg import BotCmd, BotCmdArray
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.task import Future
import time



# ---------------------- PID Controller Class --------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0
        


    def compute(self, error, dt):
#-----------------------------PID Compute Steps--------------------------------------------------------------
        if dt <= 0.0 or math.isnan(dt) or math.isinf(dt):
            dt = 1e-3
        
        # 1. Accumulate the error over time for the Integral term
        self.integral += error * dt
        max_integral = self.max_out / max(self.ki, 1e-6)
        self.integral = max(min(self.integral, max_integral), -max_integral)

        # 2. Compute the change in error for the Derivative term
        derivative = (error - self.prev_error) / dt
        # 3. Calculate the PID output:
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # 4. Store the current error for use in the next iteration
        self.prev_error = error
        # 5. Limit (clip) the output between [-max_out, +max_out] to avoid unsafe velocities
        output = max(min(output, self.max_out), -self.max_out)
        
        return output
#------------------------------------------------------------------------------------------------------------
       
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        # ---------------- Robot Parameters ----------------
        # 1. Robot ID(s)
        # Track all three robots
        self.robots = {
            4: {  # hb_glacio
                'name': 'hb_glacio',
                'current_pose': {'x': 846.0, 'y': 141.0, 'w': 0.0},
                'home_pose': (846.0, 141.0, 0.0),
                'state': 'idle',
                'substate': None,
                'assigned_crate': None,
                'goal_reached_counter': 0,
                'align_stable_counter': 0,
                'align_timeout_counter': 0
            },
            0: {  # hb_crystal
                'name': 'hb_crystal',
                'current_pose': {'x': 1219.0, 'y': 142.5, 'w': 0.0},
                'home_pose': (1218.0, 205.0, 0.0),
                'state': 'idle',
                'substate': None,
                'assigned_crate': None,
                'goal_reached_counter': 0,
                'align_stable_counter': 0,
                'align_timeout_counter': 0
                
            },
            2: {  # hb_frostbite
                'name': 'hb_frostbite',
                'current_pose': {'x': 1590.0, 'y': 141.5, 'w': 0.0},
                'home_pose': (1568.0, 202.0, 0.0),
                'state': 'idle',
                'substate': None,
                'assigned_crate': None,
                'goal_reached_counter': 0,
                'align_stable_counter': 0,
                'align_timeout_counter': 0
            }
            }
        
        # 3. Goal tracking index
        self.goal_idx = 0
        # ---------------- Simple Collision Avoidance ----------------
        self.avoidance_distance = 200.0  # mm - distance to trigger avoidance
        self.avoidance_strength = 100.0  # mm/s - how strong to push away
        
        # 4. Timing information:
        #    - Used to calculate the time difference (dt) between control loop iterations.
        # 5. Threshold for goal completion:
        self.goal_tolerance = {'x': 40.0, 'y': 40.0, 'theta': 10.0}
        #    - Defines the acceptable error tolerance for x, y, and θ.
        #    - Example: if error < 5 units → goal considered reached.
        self.last_time = self.get_clock().now()
        self.crate_pose = None
        self.crate_id = None
        self.align_stable_counter = 0
        self.align_stable_threshold = 30  # must hold steady for 10 cycles (~0.3s)
        # Add stability counter for goal reached
        self.goal_reached_counter = 0
        self.goal_reached_threshold = 40 

        # ---------------- Goal Definitions ----------------

        # List of waypoints [(x, y, yaw_deg)]
        self.drop_zones = {
            'red': {  # D1
                'x': 1215.0,  # center
                'y': 1080.0,
                'xmin': 1020, 'xmax': 1410,
                'ymin': 1075, 'ymax': 1355
            },
            'green': {  # D2
                'x': 820.0,
                'y': 1950.0,
                'xmin': 675, 'xmax': 965,
                'ymin': 1920, 'ymax': 2115
            },
            'blue': {  # D3
                'x': 1616.0,
                'y': 1950.0,
                'xmin': 1470, 'xmax': 1762,
                'ymin': 1920, 'ymax': 2115
            }
        }
        self.pose_timestamp = self.get_clock().now()


        #----------------DO NOT CHANGE----------------------

        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 3.5, 'ki': 0.015, 'kd': 0.15, 'max_out': 100},
            'y': {'kp': 3.5, 'ki': 0.015, 'kd': 0.15, 'max_out': 100},
            'theta': {'kp': 0.08, 'ki': 0.0004, 'kd': 0.004, 'max_out': 8.0}
        }
        self.pid = {}
        for robot_id in [4, 0, 2]:
            self.pid[robot_id] = {
                'x': PID(**self.pid_params['x']),
                'y': PID(**self.pid_params['y']),
                'theta': PID(**self.pid_params['theta'])
            }

        # Initialize PIDs
        
        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /bot_pose
        self.pose_sub = self.create_subscription(
            Poses2D, '/bot_pose', self.pose_cb, 10
        )
        # Subscribe to crate pose
        self.crate_sub = self.create_subscription(
            Poses2D, '/crate_pose', self.crate_cb, 10
        )

        # Publisher for motion + arm control
        self.bot_cmd_pub = self.create_publisher(
            BotCmdArray, '/bot_cmd', 10
        )

        # Create service clients for attach/detach - DON'T WAIT FOR THEM
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')
        self.get_logger().info("Waiting for attach/detach services...")


        self.publisher = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz

        self.get_logger().info(f'Holonomic PID Controller started. ')
        self.approach_distance = 128.0  # mm
        self.align_tolerance = 5.0  
        self.crate_color = None
        self.substate = "approach"      # for move_to_crate



    # ---------------- Subscriber Callback ----------------
    def pose_cb(self, msg):
        """
        Callback function for /bot_pose topic.
        This function is executed each time a message is received.

        Steps:
        1. Iterate through all poses in the incoming message.
        2.  Update self.current_pose with this robot's pose.
        """
        for pose in msg.poses:
            if pose.id in self.robots:
                
                self.robots[pose.id]['current_pose'] = {
                    'x': pose.x,
                    'y': pose.y,
                    'w': pose.w
                }
                self.pose_timestamp = self.get_clock().now()            
                
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
        if not hasattr(self, 'tasks_assigned') and len(self.all_crates) >= 4:
            self.get_logger().info(f" Detected {len(self.all_crates)} crates - starting allocation...")
            self.assign_tasks()
            self.tasks_assigned = True

    def get_crate_color(self, crate_id):
        """Determine crate color from ID"""
        if crate_id % 3 == 0:
            return 'red'
        elif crate_id % 3 == 1:
            return 'green'
        else:
            return 'blue'
    def assign_tasks(self):
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
                self.robots[nearest_robot]['state'] = 'move_to_crate'
                self.robots[nearest_robot]['substate'] = 'approach'
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

    # ---------------- Control Loop ----------------
    def control_cb(self):
        now=self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        for robot_id, robot in self.robots.items():
            self.control__single_robot(robot_id, robot, dt)
    
    def control__single_robot(self,robot_id,robot,dt):
        state=robot['state']
        current_pose=robot['current_pose']
        if state=='idle':
            if hasattr(self, 'remaining_crates') and self.remaining_crates:
                new_crate_id = self.remaining_crates.pop(0)
                crate = self.all_crates[new_crate_id]
                robot['assigned_crate'] = new_crate_id
                robot['state'] = 'move_to_crate'
                robot['substate'] = 'approach'
                crate['assigned'] = True
                self.get_logger().info(
                    f"Robot {robot_id} reassigned crate {new_crate_id} ({crate['color']})"
                )
            return
        elif state=='move_to_crate':
            self.handle_move_to_crate(robot_id, robot, dt)
        elif state=='pick_crate':
            self.handle_pick_crate(robot_id, robot)
        elif state=='move_to_zone':
            self.handle_move_to_zone(robot_id, robot, dt)
        elif state=='place_crate':
            self.handle_place_crate(robot_id, robot)
        elif state=='return_home':
            self.handle_return_home(robot_id, robot, dt)
        elif state=='done':
            self.stop_robot(robot_id)
            robot['state']='idle'
            self.get_logger().info(f"Robot {robot_id}: DONE state - awaiting further instructions.")
            
            
            
    


    def handle_move_to_crate(self, robot_id, robot, dt):
        # Ensure arm is up
        self.move_arm(robot_id, 0.0, 0.0)
        
        # Get assigned crate info
        crate_id = robot['assigned_crate']
        if crate_id is None or crate_id not in self.all_crates:
            self.get_logger().error(f"Robot {robot_id}: No valid crate assigned!")
            robot['state'] = 'idle'
            return
        
        crate = self.all_crates[crate_id]
        crate_x = float(crate['x'])
        crate_y = float(crate['y'])
        crate_theta = float(crate['w'])

        
        bot_pose = robot['current_pose']
        bot_x = float(bot_pose['x'])
        bot_y = float(bot_pose['y'])
        bot_theta = float(bot_pose['w'])
        
        # Calculate angle to crate
        angle_to_crate = math.degrees(math.atan2(crate_y - bot_y, crate_x - bot_x))
        
        # Get or initialize substate
        if robot['substate'] is None:
            robot['substate'] = 'approach'
            robot['align_stable_counter'] = 0
        
        # ---------- SUBSTATE: APPROACH ----------
        if robot['substate'] == 'approach':
            # Compute target position 130mm away from crate center
            goal_x = crate_x - self.approach_distance * math.cos(math.radians(angle_to_crate))
            goal_y = crate_y - self.approach_distance * math.sin(math.radians(angle_to_crate))
            # print.__get__('')(f"Robot {robot_id}: Approaching crate {crate_id} at ({goal_x:.1f}, {goal_y:.1f})")
            goal_theta = bot_theta  # maintain current orientation
            
            # Navigate to approach position
            self.move_to_goal(robot_id, goal_x, goal_y, goal_theta, dt)
            
            # Check if close enough to approach position
            dist = math.sqrt((goal_x - bot_x)**2 + (goal_y - bot_y)**2)
            
            if dist < 30.0:  # Within 40mm of approach position
                self.stop_robot(robot_id)
                robot['substate'] = 'align'
                self.pid[robot_id]['theta'].reset()
                self.get_logger().info(f"Robot {robot_id}: Approach complete → aligning with crate.")
        
        # ---------- SUBSTATE: ALIGN ----------
        elif robot['substate'] == 'align':
            
            # Compute angle error to face crate directly
            # angle_error = (angle_to_crate - bot_theta + 180.0) % 360.0 - 180.0 + 45.0
            bot_theta=-((bot_theta+180)%360-180) #convert to standard angle
            arm_angle = bot_theta+90
            angle_error=arm_angle - angle_to_crate
            if angle_error > 180:
                angle_error -= 360
            elif angle_error < -180:
                angle_error += 360
            
            align_tol_deg = 10.0
            if abs(angle_error) < 3.0:
                w = 0.0
            else:
                w = self.pid[robot_id]['theta'].compute(angle_error, dt)
                if abs(angle_error) < 15.0:
                    w *= 0.5  # Slow down when close
                # w=max(min(w,5.0),-5.0)

                # generate angular velocity from PID, no translational motion
                # w = self.pid_theta.compute(angle_error, dt)
            vx, vy = 0.0, 0.0

            # inverse kinematics (unchanged)
            s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
            s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
            s3 = (2.0*vx/3.0) + (w/3.0)
            self.publish_bot_cmd(robot_id,s1, s2, s3)

            # stable check: require small angle for several cycles
            if abs(angle_error) < align_tol_deg:
                robot['align_stable_counter'] += 1
            else:
                robot['align_stable_counter'] = 0

            if robot['align_stable_counter'] >= self.align_stable_threshold:
                self.stop_robot(robot_id)
                self.publish_bot_cmd(robot_id,0.0, 0.0, 0.0)
            
                robot['align_stable_counter'] = 0
                robot['substate'] = "done"
                robot['state'] = "pick_crate"
                self.get_logger().info(f"Aligned with crate, angle_error={angle_error:.2f}°")
            # Backup: if very close but never stable long enough
            elif abs(angle_error) < 8.0:
                self.pid[robot_id]['theta'].reset()
                self.pid[robot_id]['x'].reset()
                self.pid[robot_id]['y'].reset()   
                self.stop_robot(robot_id)
                self.publish_bot_cmd(robot_id,0.0, 0.0, 0.0)  # Extra stop command
                self.publish_bot_cmd(robot_id,0.0, 0.0, 0.0)
                robot['align_stable_counter'] = 0
                # robot['substate'] = "done"
                robot['state'] = "pick_crate"
                self.get_logger().warn("Forced pick: near-perfect align but counter not stable.")
            # self.align_heading = self.robots[robot_id]['current_pose']['w']
            
            elif robot['align_timeout_counter'] >= 100:  # ~4.5 seconds at 30Hz
                if abs(angle_error) < 30.0:  # Within 30 degrees
                    self.stop_robot(robot_id)
                    time.sleep(0.3)
                    self.publish_bot_cmd(robot_id, 0.0, 0.0, 0.0)
                    
                    robot['align_stable_counter'] = 0
                    robot['align_timeout_counter'] = 0
                    robot['substate'] = None
                    robot['state'] = "pick_crate"
                    self.get_logger().error(
                        f"Robot {robot_id}:  TIMEOUT! Forcing pickup. Error={angle_error:.2f}°"
                    )
                else:
                    # Too far off - retry approach
                    self.get_logger().error(
                        f"Robot {robot_id}: ALIGNMENT FAILED! Error={angle_error:.2f}° → Retry approach"
                    )
                    robot['substate'] = 'approach'
                    robot['align_stable_counter'] = 0
                    robot['align_timeout_counter'] = 0
                    self.pid[robot_id]['theta'].reset()
            
    def handle_pick_crate(self, robot_id, robot):
        """
        Handle the pick_crate state - lower arm, attach, lift
        """
        self.get_logger().info(f"Robot {robot_id}: Picking crate...")
        
        # Reset theta PID
        self.pid[robot_id]['theta'].reset()
        
        # Step 1: Small forward nudge to make contact with crate
        self.move_cartesian(robot_id, vx=0.0, vy=25.0, w=0.0, duration=0.2)
        time.sleep(0.2)
        self.stop_robot(robot_id)
        self.publish_bot_cmd(robot_id,0.0, 0.0, 0.0)  # Extra stop command
        
        # Step 2: Lower the arm fully to grab crate
        self.move_arm(robot_id, 100.0, 75.0)
        time.sleep(1.5)
        
        # Step 3: Attach the crate
        self.attach_crate(robot_id)
        time.sleep(0.3)
        self.stop_robot(robot_id)
        self.publish_bot_cmd(robot_id,0.0, 0.0, 0.0)  # Extra stop command
        
        # Step 4: Lift the arm slightly
        self.move_arm(robot_id, 90.0, 45.0)
        time.sleep(0.8)
        
        # Step 5: Move back slightly
        self.move_cartesian(robot_id, vx=0.0, vy=-25.0, w=0.0, duration=0.2)
        time.sleep(0.2)
        self.stop_robot(robot_id)
        
        # Reset PIDs
        self.pid[robot_id]['x'].reset()
        self.pid[robot_id]['y'].reset()
        self.pid[robot_id]['theta'].reset()
        
        self.get_logger().info(f"Robot {robot_id}: Crate picked up and secured.")
        
        # Wait for pose update
        time.sleep(1.5)
        
        current_pose = robot['current_pose']
        self.get_logger().warn(
            f"Robot {robot_id}: Current Pose after pickup: "
            f"x={current_pose['x']:.1f}, y={current_pose['y']:.1f}, w={current_pose['w']:.1f}°"
        )
        
        
        # Transition to move_to_zone
        robot['substate'] = None
        robot['zone_substate'] = None
        robot['realign_stable_counter'] = 0
        robot['state'] = 'move_to_zone'
        
    
    

    def handle_move_to_zone(self, robot_id, robot, dt):
        """
        Handle the move_to_zone state - first realign to w=0, then navigate to drop zone
        """
        # Initialize substate if needed
        if robot.get('zone_substate') is None:
            robot['zone_substate'] = 'realign'
            robot['realign_stable_counter'] = 0
            self.get_logger().info(f"Robot {robot_id}: Starting move_to_zone - first realigning to w=0")
        
        current_pose = robot['current_pose']
        x = current_pose['x']
        y = current_pose['y']
        theta = current_pose['w']
        
        # ---------- SUBSTATE: REALIGN TO W=0 ----------
        if robot['zone_substate'] == 'realign':
            # Calculate angle error to reach w=0
            # Normalize theta to [-180, 180]
            normalized_theta = ((theta + 180) % 360) - 180
            angle_error = 0.0 - normalized_theta  # Target is 0 degrees
            
            realign_tolerance = 6.0  # degrees
            
            # Log progress (throttled)
            self.get_logger().info(
                f"Robot {robot_id}: REALIGNING | Current θ: {theta:.1f}° | "
                f"Normalized: {normalized_theta:.1f}° | Error: {angle_error:.1f}° | "
                f"Counter: {robot['realign_stable_counter']}"
             )
                
            # Apply PID control for rotation only
            if abs(angle_error) < 2.0:
                w = 0.0
            else:
                w = self.pid[robot_id]['theta'].compute(angle_error, dt)
            
            # No translational motion during realignment
            vx, vy = 0.0, 0.0
            
            # Inverse kinematics
            s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
            s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
            s3 = (2.0*vx/3.0) + (w/3.0)
            self.publish_bot_cmd(robot_id, s1, s2, s3)
            
            # Check if realignment is stable
            if abs(angle_error) < realign_tolerance:
                robot['realign_stable_counter'] += 1
            else:
                robot['realign_stable_counter'] = 0
            
            # Transition to navigation when stable
            if robot['realign_stable_counter'] >= 20:  # ~0.6 seconds at 30Hz
                self.stop_robot(robot_id)
                time.sleep(0.3)
                robot['zone_substate'] = 'navigate'
                robot['realign_stable_counter'] = 0
                self.get_logger().info(f"Robot {robot_id}: Realignment complete! Now navigating to drop zone.")
            
            return  # Exit early during realignment
        
        # ---------- SUBSTATE: NAVIGATE TO DROP ZONE ----------
        elif robot['zone_substate'] == 'navigate':
            # Get crate color and corresponding drop zone
            crate_id = robot['assigned_crate']
            if crate_id not in self.all_crates:
                self.get_logger().error(
                    f"Robot {robot_id}: Crate {crate_id} not in all_crates! Skipping to place_crate."
                )
                robot['state'] = 'place_crate'
                return
            
            crate = self.all_crates[crate_id]
            crate_color = crate['color']
            
            # Get drop zone center coordinates
            drop_zone = self.drop_zones[crate_color]
            goal_x = drop_zone['x']
            goal_y = drop_zone['y']
            goal_theta = 0  # Face forward
            
            # Calculate errors
            dx = goal_x - x
            dy = goal_y - y
            dist = math.sqrt(dx**2 + dy**2)
            
            # Throttled logging
            if int(self.get_clock().now().nanoseconds / 1e8) % 10 == 0:
                self.get_logger().info(f"Robot {robot_id}: Moving to {crate_color.upper()} zone | Dist: {dist:.1f}mm")
            
            # Navigate if far from goal
            if dist > 40.0:
                self.move_to_goal(robot_id, goal_x, goal_y, goal_theta, dt)
            else:
                # Reached drop zone
                self.stop_robot(robot_id)
                time.sleep(0.5)
                self.stop_robot(robot_id)
                
                # Clean up substate
                robot['zone_substate'] = None
                robot['state'] = 'place_crate'
                self.get_logger().info(f"Robot {robot_id}: REACHED {crate_color.upper()} DROP ZONE!")
            
    def handle_place_crate(self, robot_id, robot):
        """
        Handle the place_crate state - lower arm, detach crate
        """
        self.get_logger().info(f"Robot {robot_id}: Placing crate...")
        
        # Lower arm to place crate
        self.move_arm(robot_id, 90.0, 60.0)
        time.sleep(1.5)
        
        # Detach crate
        self.detach_crate(robot_id)
        time.sleep(0.8)
        
        # Raise arm back to stowed position
        self.move_arm(robot_id, 0.0, 0.0)
        time.sleep(0.8)
        
        self.get_logger().info(f"Robot {robot_id}: Crate placed successfully.")
        
        # Transition to return_home
        robot['state'] = 'return_home'


    def handle_return_home(self, robot_id, robot, dt):
        """
        Handle the return_home state - navigate back to docking position
        """
        # Get home position for this robot
        home_pose = robot['home_pose']
        gx, gy, gt = home_pose
        
        # Current position
        current_pose = robot['current_pose']
        x = current_pose['x']
        y = current_pose['y']
        theta = current_pose['w']
        theta_rad = math.radians(theta)
        gt = 0.0
        
        # Calculate distance to home
        dx = gx - x
        dy = gy - y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Throttled logging
        if int(self.get_clock().now().nanoseconds / 1e8) % 10 == 0:
            self.get_logger().info(f"Robot {robot_id}: Returning home | Dist: {dist:.1f}mm")
        
        # Navigate to home
        if dist > 40.0:
            self.move_to_goal(robot_id, gx, gy, gt, dt)
        else:
            # Check if goal is reached with stability
            if self.is_goal_reached(robot_id, gx, gy, gt):
                self.stop_robot(robot_id)
                time.sleep(0.3)
                self.stop_robot(robot_id)
                
                robot['goal_reached_counter'] = 0
                robot['state'] = 'done'
                
                self.get_logger().info(f"Robot {robot_id}: Mission Complete! 🎉")
        


    def move_cartesian(self, robot_id, vx, vy, w=0.0, duration=0.3):
        """
        Move the robot using Cartesian velocity (vx, vy, w) in mm/s for a specified duration
        """
        # Calculate wheel speeds using inverse kinematics
        s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
        s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
        s3 = (2.0*vx/3.0) + (w/3.0)
        
        self.publish_bot_cmd(robot_id, -s1, -s2, -s3)
        time.sleep(duration)
        self.stop_robot(robot_id)
    def move_to_goal(self, robot_id, gx, gy, gt, dt):
        """
        Move robot towards goal (gx, gy, gt) using PID controllers
        """
        robot = self.robots[robot_id]
        current_pose = robot['current_pose']
        
        x = current_pose['x']
        y = current_pose['y']
        theta = current_pose['w']
        theta = ((theta + 180) % 360) - 180  # Normalize to [-180, 180]
        theta_rad = math.radians(theta)
        
        # Calculate errors
        ex = gx - x
        ey = gy - y
        etheta = (gt - theta + 180) % 360 - 180
        
        
        # Transform to robot frame
        ex_r = math.cos(theta_rad)*ex + math.sin(theta_rad)*ey
        ey_r = -math.sin(theta_rad)*ex + math.cos(theta_rad)*ey
        
        # Compute velocities using PID
        vx = self.pid[robot_id]['x'].compute(ex_r, dt)
        vy = self.pid[robot_id]['y'].compute(ey_r, dt)
        w = self.pid[robot_id]['theta'].compute(etheta, dt)
        
        v = self.get_avoidance_velocity(robot_id)
        vx= vx-v
        vy=vy-v
        # Calculate wheel speeds using inverse kinematics
        s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
        s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
        s3 = (2.0*vx/3.0) + (w/3.0)
        
        self.publish_bot_cmd(robot_id, s1, s2, s3)
        
    def attach_crate(self, robot_id):
        """Attach crate for specific robot"""
        robot = self.robots[robot_id]
        crate_id = robot['assigned_crate']
        
        if crate_id is None or crate_id not in self.all_crates:
            self.get_logger().error(f"Robot {robot_id}: Invalid crate ID!")
            return
        
        crate = self.all_crates[crate_id]
        crate_color = crate['color']
        
        req = AttachLink.Request()
        robot_name = robot['name']
        crate_name = f"crate_{crate_color}_{crate_id}"
        link_name = f"box_link_{crate_id}"
        
        # The service expects a JSON string in 'data' field
        req.data = f'{{"model1_name":"{robot_name}","link1_name":"arm_link_2","model2_name":"{crate_name}","link2_name":"{link_name}"}}'
        
        future = self.attach_cli.call_async(req)
        self.get_logger().info(f"Robot {robot_id}: Attaching {crate_name}")


    def detach_crate(self, robot_id):
        """Detach crate for specific robot"""
        robot = self.robots[robot_id]
        crate_id = robot['assigned_crate']
        
        if crate_id is None or crate_id not in self.all_crates:
            self.get_logger().error(f"Robot {robot_id}: Invalid crate ID!")
            return
        
        crate = self.all_crates[crate_id]
        crate_color = crate['color']
        
        req = DetachLink.Request()
        robot_name = robot['name']
        crate_name = f"crate_{crate_color}_{crate_id}"
        link_name = f"box_link_{crate_id}"
        
        req.data = f'{{"model1_name":"{robot_name}","link1_name":"arm_link_2","model2_name":"{crate_name}","link2_name":"{link_name}"}}'
        
        future = self.detach_cli.call_async(req)
        self.get_logger().info(f"Robot {robot_id}: Detaching {crate_name}")




    def is_goal_reached(self, robot_id, gx, gy, gt):
        """
        Check if robot has reached goal position with stability check
        """
        robot = self.robots[robot_id]
        current_pose = robot['current_pose']
        
        x = current_pose['x']
        y = current_pose['y']
        t = current_pose['w']
        
        # Normalize angle difference
        angle_diff = abs(gt - t)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # Check if within tolerance
        if (abs(gx - x) < self.goal_tolerance['x'] and
            abs(gy - y) < self.goal_tolerance['y'] and
            angle_diff < self.goal_tolerance['theta']):
            
            robot['goal_reached_counter'] += 1
        else:
            robot['goal_reached_counter'] = 0
        
        # Goal is reached if stable for threshold iterations
        return robot['goal_reached_counter'] >= self.goal_reached_threshold
    
    def get_avoidance_velocity(self, robot_id):
        robot = self.robots[robot_id]
        current_pose = robot['current_pose']
        x = current_pose['x']
        y = current_pose['y']   
        e=0.0
        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue
            other_pose = other['current_pose']
            ox = other_pose['x']
            oy = other_pose['y']
            dx = ox - x
            dy = oy - y
            dist = math.sqrt(dx**2 + dy**2)
            if dist >300.0 :
                e=0.0
            else:
                e = (300.0 - dist)
            m=dy/dx   
            theta =(math.atan(m))
            cos_theta=math.cos(theta)
            v = e*cos_theta
        return v
        
        
    def publish_bot_cmd(self, robot_id, m1, m2, m3, base=0.0, elbow=0.0):
        """Publish command for specific robot"""
        cmd = BotCmd(
            id=robot_id, 
            m1=m1, 
            m2=m2, 
            m3=m3, 
            base=base, 
            elbow=elbow
        )
        array_msg = BotCmdArray(cmds=[cmd])
        self.bot_cmd_pub.publish(array_msg)
    def stop_robot(self, robot_id):
        """Stop the robot by publishing zero velocities"""
        self.publish_bot_cmd(robot_id, 0.0, 0.0, 0.0)   
        self.pid[robot_id]['x'].reset()
        self.pid[robot_id]['y'].reset()
        self.pid[robot_id]['theta'].reset()

    def move_arm(self, robot_id, base, elbow):
        cmd = BotCmd(id=robot_id, m1=0.0, m2=0.0, m3=0.0, 
                    base=base, elbow=elbow)
        self.bot_cmd_pub.publish(BotCmdArray(cmds=[cmd]))


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':   
    main()