#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from voxblox_msgs.msg import Layer
import numpy as np


class TsdfWeightProbe(Node):
    def __init__(self):
        super().__init__("tsdf_weight_logger")

        # Hard-coded probe point (world coordinates)
        self.probe = np.array([0.5, 0.0, 0.4], dtype=np.float32)

        self.create_subscription(Layer, "/tsdf_map_out", self.tsdf_callback, 10)

        # TSDF meta
        self.voxel_size = None
        self.voxels_per_side = None
        self.block_size = None
        self.words_per_voxel = None

        # Cache blocks across updates
        self.block_cache = {}

        self.get_logger().info(
            "TSDF weight probe started at x=0.5 y=0.0 z=0.4"
        )

    def tsdf_callback(self, msg: Layer):
        # Read geometry
        self.voxel_size = float(msg.voxel_size)
        self.voxels_per_side = int(msg.voxels_per_side)
        self.block_size = self.voxel_size * self.voxels_per_side

        # Handle reset
        if msg.action == Layer.ACTION_RESET:
            self.block_cache.clear()

        # Update cache
        for b in msg.blocks:
            key = (int(b.x_index), int(b.y_index), int(b.z_index))
            self.block_cache[key] = b.data

        if not self.block_cache:
            return

        # Infer words per voxel once
        if self.words_per_voxel is None:
            any_block = next(iter(self.block_cache.values()))
            nvox = self.voxels_per_side ** 3
            self.words_per_voxel = int(round(len(any_block) / nvox))
            self.get_logger().info(
                f"Detected TSDF layout: words_per_voxel={self.words_per_voxel}"
            )

        w = self.get_weight_at_point(self.probe)
        self.get_logger().info(f"TSDF weight at probe = {w:.3f}")

    def get_weight_at_point(self, point: np.ndarray) -> float:
        if self.words_per_voxel is None or self.words_per_voxel < 2:
            return 0.0

        bx = math.floor(point[0] / self.block_size)
        by = math.floor(point[1] / self.block_size)
        bz = math.floor(point[2] / self.block_size)

        key = (int(bx), int(by), int(bz))
        data = self.block_cache.get(key, None)
        if data is None:
            return 0.0

        local = point - np.array([bx, by, bz], dtype=np.float32) * self.block_size
        vx = int(local[0] / self.voxel_size)
        vy = int(local[1] / self.voxel_size)
        vz = int(local[2] / self.voxel_size)

        if vx < 0 or vy < 0 or vz < 0:
            return 0.0
        if vx >= self.voxels_per_side or vy >= self.voxels_per_side or vz >= self.voxels_per_side:
            return 0.0

        idx = vx + vy * self.voxels_per_side + vz * self.voxels_per_side ** 2
        stride = self.words_per_voxel

        start = idx * stride
        end = start + stride
        if end > len(data):
            return 0.0

        # Pack uint32 words into bytes
        raw = b"".join(struct.pack("<I", int(w) & 0xFFFFFFFF) for w in data[start:end])

        # TSDF layout: [distance(float32), weight(float32), ...]
        weight = struct.unpack_from("<f", raw, 4)[0]
        if not math.isfinite(weight):
            return 0.0
        return float(weight)


def main():
    rclpy.init()
    node = TsdfWeightProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
