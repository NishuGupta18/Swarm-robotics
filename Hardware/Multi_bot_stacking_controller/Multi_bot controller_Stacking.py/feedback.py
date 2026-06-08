#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D

class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.bridge = CvBridge()
        
        # --- CONFIGURATION (PHYSICAL MEASUREMENTS) ---
        self.camera_height_mm = 2980.4      
        self.crate_height_mm = 50.0         
        self.bot_height_mm = 67.0          
        self.arena_width_mm = 2438.4
        self.arena_height_mm = 2438.4
        
        # SIZES
        self.corner_marker_size = 150.0  # Corner markers (1,3,5,7)
        self.payload_marker_size = 50.0  # Bot/Crate markers
        
        # NADIR POINT (Camera center)
        self.camera_center_world_x = self.arena_width_mm / 2.0
        self.camera_center_world_y = self.arena_height_mm / 2.0

        # --- PERSISTENCE ---
        self.saved_marker_corners = {} # id: 4x2 array of corners
        self.calibration_locked = False
        self.H_ground = None

        # --- ROS SETUP ---
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)

        # --- ARUCO SETUP ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.bot_ids = [0, 2, 4]       
        self.corner_ids = [1, 3, 5, 7] 

    def get_world_corners(self, marker_id):
        """Returns the world coordinates for the 4 corners of the 150mm markers."""
        s = self.corner_marker_size
        W = self.arena_width_mm
        H = self.arena_height_mm
        
        # Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        refs = {
            1: np.array([[0, 0], [s, 0], [s, s], [0, s]]),
            3: np.array([[W-s, 0], [W, 0], [W, s], [W-s, s]]),
            5: np.array([[0, H-s], [s, H-s], [s, H], [0, H]]),
            7: np.array([[W-s, H-s], [W, H-s], [W, H], [W-s, H]])
        }
        return refs.get(marker_id)



    def calibrate_16_points(self):
        """Uses all 16 corner points from the 4 corner markers for stability."""
        img_pts = []
        world_pts = []
        for cid in self.corner_ids:
            if cid in self.saved_marker_corners:
                img_pts.extend(self.saved_marker_corners[cid])
                world_pts.extend(self.get_world_corners(cid))
        
        if len(img_pts) >= 12: # Need at least 3 markers for stable homography
            self.H_ground, _ = cv2.findHomography(np.array(img_pts), np.array(world_pts), cv2.RANSAC, 5.0)
            if len(img_pts) == 16:
                self.calibration_locked = True
                self.get_logger().info("16-Point Calibration LOCKED.")

    def get_real_coordinate(self, gx, gy, h_obj):
        cx, cy = self.camera_center_world_x, self.camera_center_world_y
        scale = (self.camera_height_mm - h_obj) / self.camera_height_mm
        rx = cx + (gx - cx) * scale
        ry = cy + (gy - cy) * scale
        return rx, ry

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except: return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is not None:
            ids = ids.flatten()
            for i, mid in enumerate(ids):
                if mid in self.corner_ids:
                    self.saved_marker_corners[mid] = corners[i][0]

        if not self.calibration_locked:
            self.calibrate_16_points()
            cv2.putText(cv_img, f"Calibrating... {len(self.saved_marker_corners)}/4", (50, 50), 1, 2, (0,0,255), 2)
        else:
            if ids is not None:
                self.process_payloads(cv_img, corners, ids)

        cv2.imshow("Localization", cv2.resize(cv_img, None, fx=0.5, fy=0.5))
        cv2.waitKey(1)

    def process_payloads(self, img, corners, ids):
        b_poses, c_poses = [], []
        for i, mid in enumerate(ids):
            if mid in self.corner_ids: continue
            
            c = corners[i][0]
            px, py = np.mean(c[:, 0]), np.mean(c[:, 1])
            
            # Ground Plane Transform
            src = np.array([[[px, py]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(src, self.H_ground)
            gx, gy = dst[0,0,0], dst[0,0,1]
            
            # Parallax Correction
            h = self.bot_height_mm if mid in self.bot_ids else self.crate_height_mm
            rx, ry = self.get_real_coordinate(gx, gy, h)
            
            # Yaw
            yaw = (-math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))) % 360
            
            pose = {'id': int(mid), 'x': rx, 'y': ry, 'w': yaw}
            
            # Visualization
            label = f"ID:{mid} ({int(rx)},{int(ry)}) W:{int(yaw)}deg"
            cv2.putText(img, label, (int(px)-20, int(py)-20), 1, 0.8, (255,0,255), 1)
            
            if mid in self.bot_ids: b_poses.append(pose)
            else: c_poses.append(pose)

        self.publish_poses(self.bot_poses_pub, b_poses)
        self.publish_poses(self.crate_poses_pub, c_poses)

    def publish_poses(self, pub, poses):
        if not poses: return
        msg = Poses2D()
        for p in poses:
            p2d = Pose2D()
            p2d.id, p2d.x, p2d.y, p2d.w = p['id'], float(p['x']), float(p['y']), float(p['w'])
            msg.poses.append(p2d)
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()