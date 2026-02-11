#!/usr/bin/env python3
# Step Response Mover for Dynamic Obstacle in Ignition Gazebo
# Moves the box between two positions with a fixed hold time
# First attempt 26/01 - Check report for details


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import time

class StepResponseMover(Node):
    def __init__(self):
        super().__init__('step_response_mover')
        
        # Publisher for Ground Truth
        self.publisher_ = self.create_publisher(PoseStamped, '/box_ground_truth', 10)
        
        # CONFIGURATION
        self.world_name = "obstacle_world"  # Check your SDF name!
        self.model_name = "dynamic_box"
        self.interval = 10.0 # Seconds to wait in each state
        
        # POSITIONS
        # Position A: The "Test Zone" (In front of robot)
        self.pos_A_str = "0.5 0.0 0.3"
        self.pos_A_list = [0.5, 0.0, 0.3]
        
        # Position B: The "Void" (Far away to clear the map)
        self.pos_B_str = "0.5 0.25 0.3" 
        self.pos_B_list = [0.5, 0.5, 0.3]
        
        self.state = "AT_A" # Start at A
        self.timer = self.create_timer(self.interval, self.cycle_step)
        self.publish_timer = self.create_timer(0.1, self.publish_truth) # 10Hz Truth
        
        self.get_logger().info("Step Response Mover Started. 10s Hold Time.")
        # Move to A initially
        self.teleport_box(self.pos_A_str)

    def cycle_step(self):
        if self.state == "AT_A":
            # Move to B (Clear the zone)
            self.get_logger().info("--- STEP: CLEARING (Moving to Void) ---")
            self.teleport_box(self.pos_B_str)
            self.state = "AT_B"
        else:
            # Move to A (Discovery)
            self.get_logger().info("--- STEP: DISCOVERY (Moving to Test Zone) ---")
            self.teleport_box(self.pos_A_str)
            self.state = "AT_A"

    def publish_truth(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        if self.state == "AT_A":
            msg.pose.position.x = self.pos_A_list[0]
            msg.pose.position.y = self.pos_A_list[1]
            msg.pose.position.z = self.pos_A_list[2]
        else:
            msg.pose.position.x = self.pos_B_list[0]
            msg.pose.position.y = self.pos_B_list[1]
            msg.pose.position.z = self.pos_B_list[2]
            
        self.publisher_.publish(msg)

    def teleport_box(self, position_str):
        x, y, z = position_str.split()

        # Correct ignition.msgs.Pose request format (no commas, braces)
        req_content = (
            f'name: "{self.model_name}" '
            f'position {{ x: {x} y: {y} z: {z} }} '
            f'orientation {{ x: 0 y: 0 z: 0 w: 1 }}'
        )

        cmd = [
            "ign", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000",
            "--req", req_content
        ]

        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            self.get_logger().error(f"Teleport failed: {res.stderr}")
        else:
            # Print stdout too because ign service often reports details there
            self.get_logger().info(f"Teleport response: {res.stdout.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = StepResponseMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()