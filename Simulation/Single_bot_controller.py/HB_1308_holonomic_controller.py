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
        self.robot_id = 0 
        # 2. Current pose of the robot:
        #    - Updated from the /bot_pose topic in the callback function.
        #    - Stores [x, y, θ] information for the active robot.
        self.current_pose = {'x': 1219.0, 'y': 142, 'w': 0.0}
        # 3. Goal tracking index
        self.goal_idx = 0
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
        self.pickup_offset = 130.0  # mm distance between bot and crate center for pickup
        self.pickup_nudge_duration = 0.3


        # Add stability counter for goal reached
        self.goal_reached_counter = 0
        self.goal_reached_threshold = 30 # Must be stable for 15 iterations

        # ---------------- Goal Definitions ----------------

        # List of waypoints [(x, y, yaw_deg)]
        self.state = "move_to_crate"
        self.drop_zone = {'xmin': 1020, 'xmax': 1410, 'ymin': 1075, 'ymax': 1355}
        self.home_pose = (1219.0, 142.0, 0.0)
        self.pose_timestamp = self.get_clock().now()


        #----------------DO NOT CHANGE----------------------

        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 3.0, 'ki': 0.01, 'kd': 0.1, 'max_out': 100},
            'y': {'kp': 3.0, 'ki': 0.01, 'kd': 0.2, 'max_out': 100},
            'theta': {'kp': 0.05, 'ki': 0.0001, 'kd': 0.002, 'max_out': 8.0}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

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
        self.approach_distance = 133.0  # mm
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
            if pose.id == self.robot_id:
                self.current_pose['x'] = pose.x
                self.current_pose['y'] = pose.y
                self.current_pose['w'] = pose.w
                self.pose_timestamp = self.get_clock().now()
                break
            
    def crate_cb(self, msg):
            for pose in msg.poses:
                if pose.id >=10:   # Example crate ID
                    self.crate_pose = {'x': pose.x, 'y': pose.y, 'w': pose.w}
                    self.crate_id = pose.id
                    
                    if pose.id%3==0:
                        self.crate_color='red'
                    elif pose.id%3==1:
                        self.crate_color='green'
                    else:
                        self.crate_color='blue'
                    self.get_logger().info(f"Detected crate ID: {self.crate_id}, Color: {self.crate_color}")
                    break
    def move_cartesian(self, vx, vy, w=0.0, duration=0.5):
        """
        Move the robot using Cartesian velocity (vx, vy, w) in mm/s.
        Uses correct inverse kinematics instead of raw wheel values.
        """
        s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
        s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
        s3 = (2.0*vx/3.0) + (w/3.0)

        self.publish_bot_cmd(s1, s2, s3)
        time.sleep(duration)
        self.stop_robot()


    # ---------------- Control Loop ----------------
    def control_cb(self):
        if self.crate_pose is None or self.current_pose is None:
            return
        
        else:
            angle_to_crate = 0.0  # default safe angle


        # dt computation
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Current pose
        x, y, theta = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']
        theta_rad = math.radians(theta)

        if self.state == "move_to_crate":
            self.move_arm(0.0, 0.0)  # ensure arm is up
        
            crate_x = float(self.crate_pose['x'])
            crate_y = float(self.crate_pose['y'])
            crate_theta = float(self.crate_pose['w'])

            bot_x = float(self.current_pose['x'])
            bot_y = float(self.current_pose['y'])
            bot_theta = float(self.current_pose['w'])

            angle_to_crate = math.degrees(math.atan2(crate_y - bot_y, crate_x - bot_x))

            # ---------- SUBSTATE: APPROACH ----------
            if self.substate == "approach":
                # Compute target 130 mm away from crate center
                goal_x = crate_x - self.approach_distance * math.cos(math.radians(angle_to_crate))
                goal_y = crate_y - self.approach_distance * math.sin(math.radians(angle_to_crate))
                goal_theta = bot_theta  # don't change orientation yet

                self.move_to_goal(goal_x, goal_y, goal_theta, dt)
                dist = math.sqrt((goal_x - bot_x)**2 + (goal_y - bot_y)**2)

                if dist < 40.0:  # close enough
                    self.stop_robot()
                    self.substate = "align"
                    self.pid_theta.reset()
                    self.get_logger().info("Approach complete → aligning with crate.")

            # ---------- SUBSTATE: ALIGN ----------
            elif self.substate == "align":
                # compute shortest angle to face crate (degrees)
                angle_error = (angle_to_crate - bot_theta + 180.0) % 360.0 - 180.0 + 60
                if angle_error > 180:
                    angle_error -= 360
                elif angle_error < -180:
                    angle_error += 360
              
                # want small alignment (e.g. within 10 degrees)
                align_tol_deg = 15.0
                if abs(angle_error) < 3.0:
                    w = 0.0
                else:
                    w = self.pid_theta.compute(angle_error, dt)

                # generate angular velocity from PID, no translational motion
                # w = self.pid_theta.compute(angle_error, dt)
                vx, vy = 0.0, 0.0

                # inverse kinematics (unchanged)
                s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
                s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
                s3 = (2.0*vx/3.0) + (w/3.0)
                self.publish_bot_cmd(s1, s2, s3)

                # stable check: require small angle for several cycles
                if abs(angle_error) < align_tol_deg:
                    self.align_stable_counter += 1
                else:
                    self.align_stable_counter = 0

                if self.align_stable_counter >= self.align_stable_threshold:
                    self.stop_robot()
                    self.publish_bot_cmd(0.0, 0.0, 0.0)
                   
                    self.align_stable_counter = 0
                    self.substate = "done"
                    self.state = "pick_crate"
                    self.get_logger().info(f"Aligned with crate, angle_error={angle_error:.2f}°")
                # Backup: if very close but never stable long enough
                elif abs(angle_error) < 8.0:
                    self.pid_theta.reset() 
                    self.pid_x.reset()      # ← ADD THIS LINE
                    self.pid_y.reset()   
                    self.stop_robot()
                    self.publish_bot_cmd(0.0, 0.0, 0.0)  # Extra stop command
                    self.publish_bot_cmd(0.0, 0.0, 0.0)
                    self.pid_theta.reset() 
                    self.pid_x.reset()      # ← ADD THIS LINE
                    self.pid_y.reset()      # ← ADD THIS LINE
                    self.align_stable_counter = 0
                    self.substate = "done"
                    self.state = "pick_crate"
                    self.get_logger().warn("Forced pick: near-perfect align but counter not stable.")
                self.align_heading = self.current_pose['w']



        # -------------------- PICK CRATE PHASE --------------------
        elif self.state == "pick_crate":
            self.get_logger().info("Picking crate...")
            self.pid_theta.reset()
            # self.current_pose['w'] = self.align_heading

            # --- Step 1: small forward nudge to make contact ---
            self.move_cartesian(vx=0.0, vy=25.0, w=0.0, duration=0.6)  # precise small forward nudge
 
            time.sleep(0.5)
            self.stop_robot()

            # --- Step 2: lower the arm fully to grab crate ---
            self.move_arm(100.0, 75.0)
            time.sleep(2)


            # --- Step 3: try attaching once in contact ---
            self.attach_crate()
            time.sleep(4)
            self.stop_robot()
            # --- Step 4: lift the arm slightly and move back ---
            self.move_arm(90.0, 45.0)
            time.sleep(1.5)
            self.move_cartesian(vx=0.0, vy=-18.0, w=0.0, duration=0.3)  # back away from crate
            time.sleep(0.5)
            self.stop_robot()
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()
            self.get_logger().info("Crate picked up and secured.")
            self.get_logger().info("waiting for pose update")
            time.sleep(1.5)
            self.get_logger().warn(f" Current Pose after pickup: x={self.current_pose['x']:.1f}, y={self.current_pose['y']:.1f}, w={self.current_pose['w']:.1f}°")
            self.state = "move_to_zone"

        elif self.state == "move_to_zone":
            goal_x = 1215.0
            goal_y = 1215.0
            goal_theta = self.current_pose['w']  # maintain current orientation
            x= self.current_pose['x']
            y= self.current_pose['y']
            theta_rad = math.radians(self.current_pose['w'])
            # Calculate errors (distance from goal)
            dx = goal_x - x
            dy = goal_y - y
            etheta= goal_theta
            dist = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle to goal (in world frame)
            angle_to_goal_deg = math.degrees(math.atan2(dy, dx))
            
            # Logging
            self.get_logger().info("="*60)
            self.get_logger().info(f" Current Pos: ({x:.1f}, {y:.1f}) facing {theta:.1f}°")
            self.get_logger().info(f" Target: ({goal_x:.1f}, {goal_y:.1f})")
            self.get_logger().info(f" Distance: {dist:.1f}mm")
            
            
            
            # Navigate if far from goal
            if dist > 30.0:
                ex_r = math.cos(theta_rad)*dx + math.sin(theta_rad)*dy
                ey_r = -math.sin(theta_rad)*dx + math.cos(theta_rad)*dy
                
                # Speed proportional to distance (max 50mm/s)
                vx=self.pid_x.compute(ex_r, dt)
                vy=self.pid_y.compute(ey_r, dt) 
                w= 0.0
                vx = max(min(vx, 60.0), -60.0)
                vy = max(min(vy, 60.0), -60.0)
                # Calculate wheel speeds (no rotation)
                s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy)+(w/3.0)
                s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy)+(w/3.0)
                s3 = (2.0*vx/3.0)+(w/3.0)
                
                self.publish_bot_cmd(-s1, -s2, -s3)
                self.get_logger().info(f" Wheel speeds: s1={s1:.1f}, s2={s2:.1f}, s3={s3:.1f}")
            else:
                # Reached goal
                self.stop_robot()
                time.sleep(0.5)
                self.stop_robot()
                self.goal_idx = 0
                self.state = "place_crate"
                self.get_logger().info(" REACHED DROP ZONE! ")
                    
            #     self.get_logger().info("Reached drop zone.")
        elif self.state == "place_crate":
            self.get_logger().info("Placing crate...")
            self.move_arm(90.0, 60.0)
            time.sleep(2)
            self.detach_crate()
            time.sleep(1)
            self.move_arm(0.0, 0.0)
            self.state = "return_home"
            self.get_logger().info("Crate placed.")

        elif self.state == "return_home":
            gx, gy, gt = self.home_pose
            x,y,theta = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']
            theta_rad = math.radians(theta)
            dx = gx - x
            dy = gy - y
            dist = math.sqrt(dx**2 + dy**2)

            # FIXED: Proper angle normalization
            angle_error = -theta  # Target is 0°, current is theta
            while angle_error > 180:
                angle_error -= 360
            while angle_error < -180:
                angle_error += 360

            # Phase 1: Move to home position (ignore orientation)
            if dist > 20.0:
                ex_r = math.cos(theta_rad)*dx + math.sin(theta_rad)*dy
                ey_r = -math.sin(theta_rad)*dx + math.cos(theta_rad)*dy
                
                vx = self.pid_x.compute(ex_r, dt)
                vy = self.pid_y.compute(ey_r, dt)
                w = 0.0  # No rotation while moving
                
                vx = max(min(vx, 60.0), -60.0)
                vy = max(min(vy, 60.0), -60.0)
                
                s1 = (-(vx/3)) + (math.sqrt(3)/3 * vy) + (w/3)
                s2 = (-(vx/3)) + (-math.sqrt(3)/3 * vy) + (w/3)
                s3 = (2*vx/3) + (w/3)
                
                self.publish_bot_cmd(-s1, -s2, -s3)
                
                if int(now.nanoseconds / 1e8) % 10 == 0:
                    self.get_logger().info(f"Returning home | Dist: {dist:.1f}mm")
                return

            # Phase 2: Align to 0° once at home position
            if abs(angle_error) > 1.0:
                w = self.pid_theta.compute(angle_error, dt)
                vx, vy = 0.0, 0.0

                s1 = (-(vx/3.0)) + (math.sqrt(3)/3.0 * vy) + (w/3.0)
                s2 = (-(vx/3.0)) + (-math.sqrt(3)/3.0 * vy) + (w/3.0)
                s3 = (2.0*vx/3.0) + (w/3.0)
                self.publish_bot_cmd(s1, s2, s3)

                self.get_logger().info(f"Aligning: θ={theta:.1f}° → 0°, error={angle_error:.1f}°")
                return

            # Phase 3: Done - properly aligned at home
            self.stop_robot()
            self.stop_robot()  # Double stop for safety
            self.state = "done"
            self.get_logger().info("✓ Returned home and aligned to θ=0°")

        elif self.state == "done":
            self.stop_robot()
            return

        
        elif self.state == "done":
            self.stop_robot()
            self.publish_bot_cmd(0.0, 0.0, 0.0)
            self.get_logger().info("Task complete. Robot is idle.")
            pass  # Idle state

            
    def move_to_goal(self, gx, gy, gt, dt):
        x, y, theta = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']
        theta_rad = math.radians(theta)
        ex, ey, etheta = gx - x, gy - y, gt - theta
        
        
        ex_r = math.cos(theta_rad)*ex + math.sin(theta_rad)*ey
        ey_r = -math.sin(theta_rad)*ex + math.cos(theta_rad)*ey
        vx = self.pid_x.compute(ex_r, dt)
        vy = self.pid_y.compute(ey_r, dt)
        w = self.pid_theta.compute(etheta, dt)
        # if self.state in [ "return_home"]:
        #     w *= 0
        #     vx = max(min(vx, 60.0), -60.0)
        #     vy = max(min(vy, 60.0), -60.0)
        s1 = (-(vx/3)) + (math.sqrt(3)/3 * vy) + (w/3)
        s2 = (-(vx/3)) + (-math.sqrt(3)/3 * vy) + (w/3)
        s3 = (2*vx/3) + (w/3)
        # if self.state in ["return_home"]:
        #     self.publish_bot_cmd(-s1, -s2, -s3)
                
        
        
        self.publish_bot_cmd(s1, s2, s3)
        

    def stop_robot(self):
        self.publish_bot_cmd(0.0, 0.0, 0.0)
        # Reset PIDs
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

    def publish_bot_cmd(self, m1, m2, m3, base=0.0, elbow=0.0):
        cmd = BotCmd(id=self.robot_id, m1=m1, m2=m2, m3=m3, base=base, elbow=elbow)
        array_msg = BotCmdArray(cmds=[cmd])
        self.bot_cmd_pub.publish(array_msg)

    def move_arm(self, base_angle, elbow_angle):
        cmd = BotCmd(id=self.robot_id, m1=0.0, m2=0.0, m3=0.0, base=base_angle, elbow=elbow_angle)
        self.bot_cmd_pub.publish(BotCmdArray(cmds=[cmd]))

    def attach_crate(self):
        if self.crate_id is None:
            self.get_logger().error("Crate ID not detected!")
            return
        
        req = AttachLink.Request()
        crate_name = f"crate_{self.crate_color}_{self.crate_id}"
        link_name = f"box_link_{self.crate_id}"
        req.data = f'{{"model1_name":"hb_crystal","link1_name":"arm_link_2","model2_name":"{crate_name}","link2_name":"{link_name}"}}'
        
        # Call service asynchronously
        future = self.attach_cli.call_async(req)
        self.get_logger().info(f"Attaching {crate_name}")

    def detach_crate(self):
        if self.crate_id is None:
            self.get_logger().error("Crate ID not detected!")
            return
            
        req = DetachLink.Request()
        crate_name = f"crate_{self.crate_color}_{self.crate_id}"
        link_name = f"box_link_{self.crate_id}"
        req.data = f'{{"model1_name":"hb_crystal","link1_name":"arm_link_2","model2_name":"{crate_name}","link2_name":"{link_name}"}}'
        
        # Call service asynchronously
        future = self.detach_cli.call_async(req)
        self.get_logger().info(f"Detaching {crate_name}")

    def is_goal_reached(self, gx, gy, gt):
        x, y, t = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']
        
        # Normalize angle difference
        angle_diff = abs(gt - t)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # Check if within tolerance
        if (abs(gx-x) < self.goal_tolerance['x'] and
            abs(gy-y) < self.goal_tolerance['y'] and
            angle_diff < self.goal_tolerance['theta']):
            self.goal_reached_counter += 1
        else:
            self.goal_reached_counter = 0
        
        # Goal is reached if stable for threshold iterations
        return self.goal_reached_counter >= self.goal_reached_threshold



# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':   
    main()