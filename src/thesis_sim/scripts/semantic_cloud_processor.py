#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge

class SemanticCloudProcessor(Node):
    def __init__(self):
        super().__init__('semantic_cloud_processor')
        self.bridge = CvBridge()
        self.intrinsics = None

        # --- 1. SETUP SUBSCRIBERS (Synced RGB + Depth) ---
        # We sync RGB and Depth directly so we can process them together
        self.sub_rgb = message_filters.Subscriber(
            self, Image, '/camera/image', qos_profile=qos_profile_sensor_data)
        
        self.sub_depth = message_filters.Subscriber(
            self, Image, '/camera/depth_image', qos_profile=qos_profile_sensor_data)

        # Approximate Time Synchronizer (allows for slight time differences)
        # Queue size 10, slop 0.1s
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.process_data)

        # --- 2. CAMERA INFO ---
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, qos_profile_sensor_data)

        # --- 3. PUBLISHERS ---
        # Main output: The weighted point cloud
        self.pub_cloud = self.create_publisher(PointCloud2, '/semantic_pcl', 10)
        # Debug output: To see what the computer "sees" as classes (Optional)
        self.pub_debug_mask = self.create_publisher(Image, '/camera/semantic_debug_mask', 10)

        self.get_logger().info("Unified Semantic Processor Started. Waiting for Camera Info & Frames...")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics = True
            self.get_logger().info(f"Intrinsics Lock: fx={self.fx:.2f}, cx={self.cx:.2f}")

    def process_data(self, rgb_msg, depth_msg):
        # Prevent processing until we know camera Intrinsics
        if not self.intrinsics:
            return

        try:
            # =========================================
            # PART A: PREPARE IMAGES
            # =========================================
            # Convert RGB
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            
            # Convert Depth & Normalize to Meters
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_raw.dtype == np.uint16:
                depth_img = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_img = depth_raw.astype(np.float32)

            # =========================================
            # PART B: SEMANTIC SEGMENTATION (From Node 1)
            # =========================================
            hsv = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2HSV)

            # --- RED (Dynamic) ---
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = mask_r1 + mask_r2

            # --- GREEN (Static) ---
            lower_green = np.array([35, 70, 50])
            upper_green = np.array([85, 255, 255])
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            # --- CREATE SEMANTIC MAP ---
            semantic_map = np.zeros(mask_red.shape, dtype=np.uint8)
            semantic_map[mask_red > 0] = 100   # Class ID 100
            semantic_map[mask_green > 0] = 200 # Class ID 200

            # Cleanup noise
            kernel = np.ones((5,5), np.uint8)
            semantic_map = cv2.morphologyEx(semantic_map, cv2.MORPH_OPEN, kernel)

            # (Optional) Publish the debug mask so you can see it in Rviz
            debug_msg = self.bridge.cv2_to_imgmsg(semantic_map, encoding="mono8")
            debug_msg.header = rgb_msg.header
            self.pub_debug_mask.publish(debug_msg)

            # =========================================
            # PART C: POINT CLOUD GENERATION (From Node 2)
            # =========================================
            
            # Filter: Valid depth AND reasonable range
            valid_mask = np.isfinite(depth_img) & (depth_img > 0.1) & (depth_img < 8.0)
            
            # Extract Z and pixel coordinates (u, v)
            z = depth_img[valid_mask]
            
            # If no valid points, exit
            if len(z) == 0:
                return

            h, w = depth_img.shape
            v, u = np.mgrid[0:h, 0:w]
            u_valid = u[valid_mask]
            v_valid = v[valid_mask]

            # Project to 3D (Pinhole Model)
            x = (u_valid - self.cx) * z / self.fx
            y = (v_valid - self.cy) * z / self.fy

            # =========================================
            # PART D: FUSION & WEIGHTING
            # =========================================
            # We take the semantic_map we created in Part B and sample it 
            # at the exact same pixels where we found valid depth
            semantic_values = semantic_map[valid_mask]
            
            # Assign weights based on Class ID
            weights = np.ones_like(z) * 0.1  # Default / Background weight
            
            # 100 = Dynamic (Red) -> High Weight/Intensity
            weights[semantic_values == 100] = 100.0 
            # 200 = Static (Green) -> Medium Weight/Intensity
            weights[semantic_values == 200] = 1.0   

            # =========================================
            # PART E: PUBLISH CLOUD
            # =========================================
            # Stack [x, y, z, intensity]
            points = np.stack([x, y, z, weights], axis=-1).astype(np.float32)

            cloud_msg = PointCloud2()
            cloud_msg.header = depth_msg.header
            # Use the frame ID consistent with your depth camera
            cloud_msg.header.frame_id = "camera_optical_frame" 
            
            cloud_msg.height = 1
            cloud_msg.width = len(z)
            cloud_msg.is_dense = False
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            cloud_msg.point_step = 16
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_bigendian = False
            cloud_msg.data = points.tobytes()

            self.pub_cloud.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticCloudProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()