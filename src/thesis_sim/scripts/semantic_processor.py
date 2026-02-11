#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SemanticProcessor(Node):
    def __init__(self):
        super().__init__('semantic_processor')
        
        # 1. Subscribers
        self.sub_rgb = self.create_subscription(
            Image, '/camera/image', self.rgb_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10)

        # 2. Publisher
        # This publishes a single image where pixel value = Class ID
        # self.pub_mask = self.create_publisher(Image, '/camera/semantic_mask', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Semantic Processor Active: Classifying Red (Dynamic) vs Green (Static)...")

    def rgb_callback(self, msg):
        try:
            # A. Convert ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # --- CLASS 1: RED (Dynamic) ---
            # Red wraps around 0/180
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            
            mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = mask_r1 + mask_r2

            # --- CLASS 2: GREEN (Static) ---
            # Green is roughly centered at Hue 60
            lower_green = np.array([35, 70, 50])
            upper_green = np.array([85, 255, 255])
            
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            # --- COMBINE INTO SEMANTIC MAP ---
            # Create an empty black image (single channel)
            semantic_map = np.zeros(mask_red.shape, dtype=np.uint8)

            # Assign Values (Weights)
            # 100 = Dynamic (Red)
            # 200 = Static (Green)
            # You can change these numbers to whatever cost/weight you want later
            semantic_map[mask_red > 0] = 100   
            semantic_map[mask_green > 0] = 200 

            # C. Cleanup (Remove noise)
            kernel = np.ones((5,5), np.uint8)
            semantic_map = cv2.morphologyEx(semantic_map, cv2.MORPH_OPEN, kernel)
            
            # D. Publish
            mask_msg = self.bridge.cv2_to_imgmsg(semantic_map, encoding="mono8")
            mask_msg.header = msg.header
            self.pub_mask.publish(mask_msg)

            

        except Exception as e:
            self.get_logger().error(f'Error processing RGB: {str(e)}')

    def depth_callback(self, msg):
        pass # We focus on the mask for now

def main(args=None):
    rclpy.init(args=args)
    node = SemanticProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()