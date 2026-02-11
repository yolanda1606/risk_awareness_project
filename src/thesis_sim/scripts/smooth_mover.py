#!/usr/bin/env python3
# Smoothly move the dynamic obstacle back and forth in Ignition Gazebo
# First attempt 23/01 - Check report for details

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import math
import time

class SmoothMover(Node):
    def __init__(self):
        super().__init__('smooth_mover')
        self.publisher_ = self.create_publisher(PoseStamped, '/box_ground_truth', 10)
        
        # CONFIGURATION
        self.world_name = "obstacle_world"  # Must match your SDF
        self.model_name = "dynamic_box"
        self.update_rate = 5.0              # Hz (Updates per second)
        
        # MOVEMENT PARAMETERS
        self.center_x = 0.5   # 0.5m in front of robot
        self.center_y = 0.0   # Center
        self.center_z = 0.1   # Low on the floor
        self.amplitude = 0.7  # Move 0.4m left and right
        self.speed = 0.5      # Speed of oscillation
        
        self.timer = self.create_timer(1.0/self.update_rate, self.move_cycle)
        self.start_time = time.time()
        self.get_logger().info("Smooth Mover Started: Publishing to /box_ground_truth")

    def move_cycle(self):
        # 1. Calculate new position (Sine wave for smooth back-and-forth)
        elapsed = time.time() - self.start_time
        new_y = self.center_y + self.amplitude * math.sin(self.speed * elapsed)
        
        # 2. Publish Ground Truth to ROS (For the Logger)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.center_x
        msg.pose.position.y = new_y
        msg.pose.position.z = self.center_z
        self.publisher_.publish(msg)

        # 3. Move the Box in Simulation (Ignition Service)
        self.send_gz_command(self.center_x, new_y, self.center_z)

    def send_gz_command(self, x, y, z):
        # Formatted string for Ignition
        req_content = f"name: '{self.model_name}', position: {{ x: {x}, y: {y}, z: {z} }}"
        
        cmd = [
            "ign", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "100", # Short timeout to prevent lag
            "--req", req_content
        ]
        # Run in background so we don't block the loop
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()