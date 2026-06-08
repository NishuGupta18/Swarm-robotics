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

class HolonomicVectorController(Node):
    def __init__(self):
        super().__init__('holonomic_vector_controller')

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.connect("10.216.105.124", 1883, 60)
        self.mqtt_client.loop_start()

        self.current_base = 100.0   # Where the arm actually is
        self.current_elbow = 45.0
        self.servo_step_size = 2.0
        self.robot_id = 0
        self.crate_id =12
        self.pick_mag_triggered = False
        self.current_pose = {'x': 0.0, 'y': 0.0, 'w': 0.0}
        self.crate_pose = {'x': None, 'y': None}
        self.goals = [] 
        self.pid_linear = PID(kp=0.4, ki=0.01, kd=0.05, max_out=10.0)
        self.target_base = 100.0  # Safe travel/initial height
        self.target_elbow = 45.0
        self.pick_substate = 'idle' # Add this to track pickup steps
        self.state = 'NAVIGATING'
        self.place_substate = 'idle' # Sub-state for the placing sequence
        self.timer_start = 0.0
        self.align_stable_counter = 0
        self.align_stable_threshold = 5
        self.pickup_start_time = 0.0
        self.place_start_time = 0.0
        self.base_speed = 8.0 
        # Add these to your __init__
        self.mag_service_called = False
        self.place_mag_unlatched = False
        # Try increasing Kp slightly
        self.pid_theta = PID(kp=0.24, ki=0.002, kd=0.06, max_out=6.0)
        self.last_time = time.time()
        self.mag_status = 0
        
        self.approach_distance = 180.0 # Distance to switch to alignment
        self.drop_zone = (1215.0, 1200.0, 0.0) 
        self.home_pose = (1219.0, 142.0, 0.0)
        self.srv = self.create_service(SetBool, '/bot_0/attach_crate', self.mag_service_cb)
        
        # 2. Create the Service Client (To trigger automatically)
        self.mag_client = self.create_client(SetBool, '/bot_0/attach_crate')

        self.pose_sub = self.create_subscription(Poses2D, '/bot_pose', self.pose_cb, 10)
        self.crate_sub = self.create_subscription(Poses2D, '/crate_pose', self.crate_pose_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Controller Started - Waiting for crate...")

    def mag_service_cb(self, request, response):
        self.mag_status = 1 if request.data else 0
        response.success = True
        return response
    
    # --- Client Trigger (The "Automation" logic) ---
    def trigger_mag_service(self, state_bool):
        # if not self.mag_client.service_is_ready():
        #     return
        # req = SetBool.Request()
        # req.data = state_bool
        # self.mag_client.call_async(req)
        if not self.mag_client.service_is_ready():
            self.get_logger().warn("Mag service not ready!")
            return
        req = SetBool.Request()
        req.data = state_bool
        future = self.mag_client.call_async(req)
        self.mag_status = 1 if state_bool else 0  
            
    def pose_cb(self, msg):
        for pose in msg.poses:
            if int(pose.id) == self.robot_id:
                self.current_pose['x'] = pose.x
                self.current_pose['y'] = pose.y
                self.current_pose['w'] = pose.w
                
    def crate_pose_cb(self, msg):
        for pose in msg.poses:
            if int(pose.id) == self.crate_id:
                self.crate_pose['x'] = pose.x
                self.crate_pose['y'] = pose.y
                self.crate_pose['w'] = pose.w
                self.goals = [(pose.x, pose.y, 0.0)]

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def control_loop(self):
        if not self.goals and self.state == 'NAVIGATING':
            self.get_logger().info("Waiting for crate detection...", throttle_duration_sec=2.0)
            return

        curr_time = time.time()
        dt = curr_time - self.last_time
        self.last_time = curr_time

        cx, cy, cw = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']

        if self.state == 'NAVIGATING':
            target_x, target_y, _ = self.goals[0]
            dx, dy = target_x - cx, target_y - cy
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.approach_distance:
                self.get_logger().info(f"Reached approach distance ({distance:.1f}mm). Switching to ALIGNING.")
                self.state = 'ALIGNING'
                self.pid_theta.reset()  # Reset PID for clean alignment
                self.send_speeds(0, 0, 0)
                return

            # Calculate velocity using PID on distance
            velocity = self.pid_linear.compute(distance, dt)

            # Direction vector (normalized)
            vx_target = (dx / distance) * velocity
            vy_target = (dy / distance) * velocity
            
            # Transform to robot frame
            theta_rad = math.radians(cw)
            vx_robot = vx_target * math.cos(theta_rad) + vy_target * math.sin(theta_rad)
            vy_robot = -vx_target * math.sin(theta_rad) + vy_target * math.cos(theta_rad)
            
            # *** FIX: NO ORIENTATION CONTROL DURING NAVIGATION ***
            # Robot maintains its current heading while moving to crate
            w_correction = (0-cw)%360-180
            w_correction*=0.05
            # heading_error = self.normalize_angle(0 - cw)
            # w_correction = self.pid_theta.compute(heading_error, dt)
            
            self.send_kinematics(vx_robot, vy_robot, w_correction)
            
            # Debug output
            if int(curr_time * 4) % 10 == 0:  # Every ~2.5 seconds
                self.get_logger().info(f"NAVIGATING: dist={distance:.1f}px, heading={cw:.1f}°")

        elif self.state == 'ALIGNING':
            # Calculate angle from robot to crate
            dx, dy = cx - self.crate_pose['x'], cy - self.crate_pose['y']
            angle_to_crate = math.degrees(math.atan2(-dy, dx))
            
            # Robot's heading normalized to [-180, 180]
            bot_theta = self.normalize_angle(cw)
            
            # Arm is at 90° from robot's front (adjust based on your robot)
            arm_angle = bot_theta + 90
            
            # Calculate alignment error (with offset correction)
            angle_error = self.normalize_angle(arm_angle - angle_to_crate)
            
            # if cx>1219.0:
            #     angle_error= - angle_error

            align_tol_deg = 5.0
            
          
            w_out = self.pid_theta.compute(angle_error, dt)
            
            # Pure rotation - no translation
            self.send_kinematics(0.0, 0.0, w_out)
            
            if abs(angle_error) < align_tol_deg:
                self.align_stable_counter += 1
            else:
                self.align_stable_counter = 0

            if self.align_stable_counter >= self.align_stable_threshold:
                self.get_logger().info(f'align_stable_counter=align_stable_counter')
                self.pid_theta.reset()
                self.trigger_mag_service(True)
                self.align_stable_counter = 0
                self.pickup_start_time = time.time()
                self.state = 'PICKING'
            # Backup: if very close but never stable long enough
            elif abs(angle_error) < 8.0:
                self.pid_theta.reset() 
                self.align_stable_counter = 0
                self.trigger_mag_service(True)
                self.pickup_start_time = time.time()
                self.state = 'PICKING'

        elif self.state == 'PICKING':
            curr_time = time.time()
    
            # Initialization of Pickup Sequence
            if self.pick_substate == 'idle':
                self.get_logger().info("Lowering Arm...")
                self.pick_substate = 'lowering'
                self.pickup_start_time = curr_time
                self.pick_mag_triggered = False
                return

            elapsed = curr_time - self.pickup_start_time

            # Step 1: Lower the arm and wait
            if self.pick_substate == 'lowering':
                # Send lowered position (adjust values for your bot)
                self.send_speeds(0, 0, 0, base=150.0, elbow=45.0)
                if not self.pick_mag_triggered:
                    self.get_logger().info("Triggering Magnet ON...")
                    self.trigger_mag_service(True)
                    self.pick_mag_triggered = True # Latch it

                if elapsed > 4.0: # WAIT 4 SECONDS for arm to settle
                    self.get_logger().info("Activating Magnet...")
                    self.pick_substate = 'attracting'
                    self.pickup_start_time = curr_time # Reset timer for next step

            # Step 2: Stay down while magnet grabs the crate
            elif self.pick_substate == 'attracting':
                if elapsed > 2.0: # WAIT 1 SECOND for magnetic attraction
                    self.get_logger().info("Lifting Crate...")
                    self.pick_substate = 'lifting'
                    # self.current_base = 150
                    # self.current_elbow = 45
                    # self.target_base = 100
                    # self.target_elbow = 45
                    self.pickup_start_time = curr_time

            # Step 3: Lift the crate to travel height
            elif self.pick_substate == 'lifting':
                self.send_speeds(0, 0, 0, base=100.0, elbow=80.0)
                
                if elapsed > 3.0:
                    self.get_logger().info("Pickup Complete. Moving to Zone.")
                    self.pick_substate = 'idle' # Reset for next use
                    self.state = 'MOVE_TO_ZONE'
        elif self.state == 'MOVE_TO_ZONE':
            zx, zy, _ = self.drop_zone
            dx, dy = zx - cx, zy - cy
            dist_to_zone = math.sqrt(dx**2 + dy**2)

            if dist_to_zone < 50.0:
                self.get_logger().info("Reached drop zone. Placing crate...")
                self.state = 'PLACE_CRATE'
                self.pid_linear.reset()
                self.pid_theta.reset()
                self.place_start_time = time.time()
                return

            # Move to zone with moderate speed
            v_mag = self.pid_linear.compute(dist_to_zone,dt)
            vx_z = (dx / dist_to_zone) *v_mag
            vy_z = (dy / dist_to_zone) *v_mag
            
            # Transform to robot frame
            t_rad = math.radians(cw)
            vx_r = vx_z * math.cos(t_rad) + vy_z * math.sin(t_rad)
            vy_r = -vx_z * math.sin(t_rad) + vy_z * math.cos(t_rad)
            w_correction = (0 - cw) % 360 - 180
            w_correction *= 0.05

            # NO rotation during movement to zone
            self.send_kinematics(vx_r, vy_r,w_correction )
            
            if int(curr_time * 4) % 10 == 0:
                self.get_logger().info(f"MOVE_TO_ZONE: dist={dist_to_zone:.1f}px")

        elif self.state == 'PLACE_CRATE':
            self.handle_place_sequence(curr_time)

        elif self.state == 'RETURN_HOME':
            hx, hy, _ = self.home_pose
            dx, dy = hx - cx, hy - cy
            dist_to_home = math.sqrt(dx**2 + dy**2)

            if dist_to_home < 150.0:
                self.get_logger().info(" Mission Complete! ")
                self.pid_linear.reset()
                self.pid_theta.reset()
                self.state = 'DONE'
                return

            # Move home
            v_mag = self.pid_linear.compute(dist_to_home, dt)
            vx_h = (dx / dist_to_home) * v_mag
            vy_h = (dy / dist_to_home) * v_mag
            
            t_rad = math.radians(cw)
            vx_r = vx_h * math.cos(t_rad) + vy_h * math.sin(t_rad)
            vy_r = -vx_h * math.sin(t_rad) + vy_h * math.cos(t_rad)

            w_correction = (0 - cw) % 360 - 180
            w_correction *= 0.05

            self.send_kinematics(vx_r, vy_r, w_correction)
            
            if int(curr_time * 4) % 10 == 0:
                self.get_logger().info(f"RETURN_HOME: dist={dist_to_home:.1f}px")

        elif self.state == 'DONE':
            self.send_speeds(0, 0, 0)

    def handle_place_sequence(self, curr_time):
        """Automated sequence: Stop -> Lower Arm -> Release Mag -> Raise Arm"""
        
        if self.place_substate == 'idle':
            self.get_logger().info("Starting automated placement sequence...")
            self.place_substate = 'lowering_arm'
            self.timer_start = curr_time
            self.mag_service_called = False
            return

        elapsed = curr_time - self.timer_start

        if self.place_substate == 'lowering_arm':
            # Stop wheels and lower arm (Adjust angles for your hardware)
            self.send_speeds(0, 0, 0, base=150.0, elbow=45.0) 
            if elapsed > 4.5: # Wait for arm to reach bottom
                self.place_substate = 'release_magnet'
                self.timer_start = curr_time

        # elif self.place_substate == 'release_magnet':
        #     # Trigger the service to turn off magnet
        #     if not hasattr(self, 'mag_service_called') or not self.mag_service_called:
        #         self.trigger_mag_service(False)
        #         self.place_mag_unlatched = True
        #     if elapsed > 2.0:
        #         self.place_substate = 'raising_arm'
        #         self.mag_service_called = False

        elif self.place_substate == 'release_magnet':
            if not self.mag_service_called:
                self.get_logger().info("Turning magnet OFF")
                self.trigger_mag_service(False)
                self.mag_status = 0   # force update
                self.mag_service_called = True

            if elapsed > 2.0:
                self.place_substate = 'raising_arm'
                self.timer_start = curr_time


        elif self.place_substate == 'raising_arm':
            # Lift arm back to travel/rest position
            self.send_speeds(0, 0, 0, base=100.0, elbow=45.0)
            if elapsed > 3.0:
                self.get_logger().info("Placement sequence complete.")
                self.pid_linear.reset()
                self.pid_theta.reset()
                self.place_substate = 'idle'
                self.state = 'RETURN_HOME'

    def send_kinematics(self, vx_r, vy_r, w_c):
        """Convert robot-frame velocities to wheel speeds"""
        sqrt3_over_3 = math.sqrt(3.0) / 3.0
        s1 = (-(vx_r/3.0)) + (sqrt3_over_3 * vy_r) + (w_c/3.0)
        s2 = (-(vx_r/3.0)) - (sqrt3_over_3 * vy_r) + (w_c/3.0)
        s3 = (2.0*vx_r/3.0) + (w_c/3.0)
        self.send_speeds(s1, s2, s3)

    def send_speeds(self, s1, s2, s3, base=None, elbow=None):
        # Update target positions if provided
        if base is not None: self.target_base = base
        if elbow is not None: self.target_elbow = elbow

        # Smoothly move current_base toward target_base
        if self.current_base < self.target_base:
            self.current_base = min(self.current_base + self.servo_step_size, self.target_base)
        elif self.current_base > self.target_base:
            self.current_base = max(self.current_base - self.servo_step_size, self.target_base)

        # Smoothly move current_elbow toward target_elbow
        if self.current_elbow < self.target_elbow:
            self.current_elbow = min(self.current_elbow + self.servo_step_size, self.target_elbow)
        elif self.current_elbow > self.target_elbow:
            self.current_elbow = max(self.current_elbow - self.servo_step_size, self.target_elbow)

        payload = {
            "s1": round(s1, 2), 
            "s2": round(s2, 2), 
            "s3": round(s3, 2),
            "mag": int(self.mag_status),
            "base": round(self.current_base, 1), 
            "elbow": round(self.current_elbow, 1)
        }
        self.mqtt_client.publish("bot_0/cmd_vel", json.dumps(payload))
def main():
    rclpy.init()
    node = HolonomicVectorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_speeds(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()