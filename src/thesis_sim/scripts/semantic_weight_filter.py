#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge

class SemanticAwarePointCloud(Node):
    def __init__(self):
        super().__init__('semantic_aware_pc_node')
        self.bridge = CvBridge()
        self.intrinsics = None

        # 1. SETUP SYNCHRONIZED SUBSCRIBERS
        # We use qos_profile_sensor_data because simulation/real cameras usually publish Best Effort
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth_image', qos_profile=qos_profile_sensor_data)
        self.mask_sub = message_filters.Subscriber(
            self, Image, '/camera/semantic_mask', qos_profile=qos_profile_sensor_data)

        # 10ms tolerance for synchronization
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.mask_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)

        # 2. CAMERA INFO (Only needs to be received once)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, qos_profile_sensor_data)
        
        # self.pub_cloud = self.create_publisher(PointCloud2, '/semantic_pcl', 10)
        self.get_logger().info("Semantic Point Cloud Node Started. Waiting for data...")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics = True
            self.get_logger().info(f"Intrinsics Lock: fx={self.fx:.2f}, cx={self.cx:.2f}")

    def sync_callback(self, depth_msg, mask_msg):
        if not self.intrinsics:
            return

        try:
            # --- IMAGE CONVERSION ---
            # Depth: Use passthrough to detect if it's 16-bit mm or 32-bit meters
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            mask_img = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")

            # Standardize Depth to meters
            if depth_raw.dtype == np.uint16:
                depth_img = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_img = depth_raw.astype(np.float32)

            # --- MASKING & FILTERING ---
            # Only process points that are within range AND have a mask value
            # (Filters out noise and "background" if background is 0)
            valid_mask = np.isfinite(depth_img) & (depth_img > 0.1) & (depth_img < 8.0)
            
            # Apply depth filter
            z = depth_img[valid_mask]
            h, w = depth_img.shape
            v, u = np.mgrid[0:h, 0:w]
            u_valid = u[valid_mask]
            v_valid = v[valid_mask]

            if len(z) == 0:
                return

            # --- 3D PROJECTION ---
            x = (u_valid - self.cx) * z / self.fx
            y = (v_valid - self.cy) * z / self.fy

            # --- SEMANTIC WEIGHTING ---
            # Extract mask values for the 3D points we just created
            semantic_values = mask_img[valid_mask]
            weights = np.ones_like(z) * 0.1  # Default low weight
            
            # Custom logic: Dynamic objects high weight, static medium
            weights[semantic_values == 100] = 100.0  # e.g., Red Panda
            weights[semantic_values == 200] = 1.0    # e.g., Static Obstacle

            # --- MESSAGE PACKING ---
            # Stack [x, y, z, intensity]
            points = np.stack([x, y, z, weights], axis=-1).astype(np.float32)

            cloud_msg = PointCloud2()
            cloud_msg.header = depth_msg.header
            # CRITICAL: Use the rotated frame from your launch file so it's right-side up
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
            cloud_msg.point_step = 16  # 4 floats * 4 bytes
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_bigendian = False
            cloud_msg.data = points.tobytes()

            self.pub_cloud.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f"Sync Callback Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticAwarePointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    