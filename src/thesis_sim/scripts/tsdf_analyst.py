#!/usr/bin/env python3
# Analyze TSDF map data to extract voxel weights and distances at the dynamic obstacle's location
# First attempt 23/01 - Check report for details


import rclpy
from rclpy.node import Node
from voxblox_msgs.msg import Layer
from geometry_msgs.msg import PoseStamped
import struct
import time
import csv
import math

class TSDFAnalyst(Node):
    def __init__(self):
        super().__init__('tsdf_analyst')
        
        # Subscribe to the raw map data (Big data!)
        self.create_subscription(Layer, '/tsdf_map_out', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/box_ground_truth', self.truth_callback, 10)
        
        self.latest_box_pos = None
        self.last_map_time = time.time()
        
        # CSV Logging
        self.csv_file = open('thesis_tsdf_weights.csv', 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["Timestamp", "Update_Delay_Sec", "Box_Weight", "Box_Distance", "Status"])
        
        self.voxel_size = 0.05 # Must match your launch file!
        self.get_logger().info("TSDF Analyst Started. Waiting for map updates...")

    def truth_callback(self, msg):
        self.latest_box_pos = msg.pose.position

    def map_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_map_time
        self.last_map_time = current_time
        
        if self.latest_box_pos is None:
            return

        # 1. Find which BLOCK contains our Red Box
        # Voxblox divides the world into "Blocks". Each block has N voxels.
        # We need to iterate through the blocks to find the one near our robot/box.
        
        found_weight = 0.0
        found_dist = 0.0
        status = "BOX_NOT_IN_UPDATE"

        for block in msg.blocks:
            # Check if this block is anywhere near the box (Optimization)
            # block_index * block_size roughly gives position
            # This is a rough check. For precise analysis, we look at the data payload.
            
            # 2. DECODE THE DATA
            # The 'data' field is a list of uint32 integers.
            # In Voxblox, they come in pairs: [Distance, Weight, Distance, Weight...]
            # We unpack them into floats.
            
            data_ints = block.data
            num_voxels = len(data_ints) // 2 
            
            # Iterate through the voxels in this block
            for i in range(0, len(data_ints), 2):
                # Convert Int -> Bytes -> Float
                raw_dist = data_ints[i]
                raw_weight = data_ints[i+1]
                
                # struct 'f' denotes float (4 bytes)
                dist = struct.unpack('f', struct.pack('I', raw_dist))[0]
                weight = struct.unpack('f', struct.pack('I', raw_weight))[0]
                
                # Filter: We only care about voxels that are "occupied" (dist close to 0)
                # and have some confidence (weight > 0)
                if abs(dist) < 0.1 and weight > 1.0:
                    # This voxel is effectively "inside" an object.
                    # Since the box is the only moving thing, this is likely our box data.
                    found_weight = weight
                    found_dist = dist
                    status = "FOUND_BOX_DATA"
                    break # Found it, stop searching this block
            
            if status == "FOUND_BOX_DATA":
                break

        # 3. Log the Data
        self.writer.writerow([current_time, dt, found_weight, found_dist, status])
        print(f"Update Time: {dt:.3f}s | Weight: {found_weight:.1f} | Dist: {found_dist:.3f} | {status}")

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = TSDFAnalyst()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()