#!/usr/bin/env python3
"""
Step-response logger for Voxblox TSDF/ESDF dynamics.

Logs:
  - GT_State from /box_ground_truth vs a configurable "zone" sphere
  - ESDF values near probe points from /esdf_pointcloud (median within radius, with nearest fallback)
  - TRUE TSDF weights at probe voxels from /tsdf_map_out (voxblox_msgs/Layer), decoded from uint32[] data
    IMPORTANT: supports incremental Layer updates by caching blocks across messages (ACTION_UPDATE / ACTION_MERGE).

Run:
  source /opt/ros/humble/setup.bash
  source ~/Projects/joel/install/setup.bash
  python3 src/thesis_sim/scripts/step_response_logger.py
"""

import csv
import math
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from voxblox_msgs.msg import Layer

import sensor_msgs_py.point_cloud2 as pc2


class StepResponseLogger(Node):
    def __init__(self):
        super().__init__("step_response_logger")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("csv_path", "thesis_step_response.csv")
        self.declare_parameter("log_period_s", 0.1)

        # Zone for GT (center of "present" location)
        self.declare_parameter("zone_x", 0.5)
        self.declare_parameter("zone_y", 0.0)
        self.declare_parameter("zone_z", 0.1)          # cube on floor (0.2m cube => center z=0.1)
        self.declare_parameter("zone_radius_m", 0.25)  # make this forgiving unless you need strictness

        # ESDF probe neighborhood
        self.declare_parameter("probe_radius_m", 0.15)     # 0.05 is often too tight for voxel grids
        self.declare_parameter("nearest_fallback_m", 0.35) # if no points in radius, allow nearest if close enough

        # Probe points (Option 2A: around the cube on the floor)
        self.declare_parameter("probe_center_x", 0.5)
        self.declare_parameter("probe_center_y", 0.0)
        self.declare_parameter("probe_center_z", 0.1)

        self.declare_parameter("probe_front_dx", -0.12)   # in front along x
        self.declare_parameter("probe_side_dy",  0.12)    # to the side along y

        self.declare_parameter("flush_every_n_rows", 20)

        # Topics
        self.declare_parameter("topic_ground_truth", "/box_ground_truth")
        self.declare_parameter("topic_esdf_cloud", "/esdf_pointcloud")
        self.declare_parameter("topic_tsdf_layer", "/tsdf_map_out")

        self.csv_path = str(self.get_parameter("csv_path").value)
        self.log_period_s = float(self.get_parameter("log_period_s").value)

        self.zone = np.array([
            float(self.get_parameter("zone_x").value),
            float(self.get_parameter("zone_y").value),
            float(self.get_parameter("zone_z").value),
        ], dtype=np.float32)
        self.zone_radius = float(self.get_parameter("zone_radius_m").value)

        self.probe_radius = float(self.get_parameter("probe_radius_m").value)
        self.nearest_fallback = float(self.get_parameter("nearest_fallback_m").value)
        self.flush_every = int(self.get_parameter("flush_every_n_rows").value)

        cx = float(self.get_parameter("probe_center_x").value)
        cy = float(self.get_parameter("probe_center_y").value)
        cz = float(self.get_parameter("probe_center_z").value)
        front_dx = float(self.get_parameter("probe_front_dx").value)
        side_dy = float(self.get_parameter("probe_side_dy").value)

        self.probe_center = np.array([cx, cy, cz], dtype=np.float32)
        self.probe_front  = np.array([cx + front_dx, cy, cz], dtype=np.float32)
        self.probe_side   = np.array([cx, cy + side_dy, cz], dtype=np.float32)

        self.topic_truth = str(self.get_parameter("topic_ground_truth").value)
        self.topic_esdf  = str(self.get_parameter("topic_esdf_cloud").value)
        self.topic_tsdf  = str(self.get_parameter("topic_tsdf_layer").value)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(PoseStamped, self.topic_truth, self.truth_callback, 10)
        self.create_subscription(PointCloud2, self.topic_esdf, self.esdf_callback, 10)
        self.create_subscription(Layer, self.topic_tsdf, self.tsdf_callback, 10)

        # -----------------------------
        # State (GT + ESDF)
        # -----------------------------
        self.truth_pos = None

        self.esdf_points = None         # Nx4 float32: x,y,z,val
        self.esdf_value_field = None    # 'intensity' or 'distance'
        self._warned_no_esdf_field = False

        self.last_esdf_rx_time = None
        self.last_tsdf_rx_time = None

        # -----------------------------
        # State (TSDF layer cache)
        # -----------------------------
        self.tsdf_voxel_size = None
        self.tsdf_voxels_per_side = None
        self.tsdf_block_size = None
        self.tsdf_words_per_voxel = None
        self.tsdf_bytes_per_voxel = None
        self.tsdf_layer_type = None

        # Cache blocks across incremental updates:
        # key: (x_index, y_index, z_index) -> uint32[] data
        self.tsdf_block_cache = {}
        self._tsdf_printed_meta = False

        # latest decoded weights
        self.tsdf_data = {"center_w": 0.0, "front_w": 0.0, "side_w": 0.0}

        # -----------------------------
        # CSV setup
        # -----------------------------
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # Keep original columns + add coverage/debug columns at the end
        self.writer.writerow([
            "t_sim", "GT_State",
            "ESDF_center", "ESDF_front", "ESDF_side",
            "Weight_center", "Weight_front", "Weight_side",
            "esdf_age_s", "tsdf_age_s",
            "logger_cycle_ms",
            "ESDF_count_center", "ESDF_count_front", "ESDF_count_side",
            "ESDF_dmin_center", "ESDF_dmin_front", "ESDF_dmin_side",
        ])
        self._row_count = 0

        # Timer
        self.create_timer(self.log_period_s, self.log_cycle)

        self.get_logger().info(
            f"StepResponseLogger started. CSV: {self.csv_path} | "
            f"probes: center={self.probe_center.tolist()} front={self.probe_front.tolist()} side={self.probe_side.tolist()} | "
            f"ESDF radius={self.probe_radius} m | zone={self.zone.tolist()} r={self.zone_radius}"
        )

    # -----------------------------
    # Callbacks
    # -----------------------------
    def truth_callback(self, msg: PoseStamped):
        self.truth_pos = msg.pose.position

    def esdf_callback(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]

        if "intensity" in field_names:
            val_field = "intensity"
        elif "distance" in field_names:
            val_field = "distance"
        else:
            if not self._warned_no_esdf_field:
                self.get_logger().warning(
                    f"{self.topic_esdf} has no 'intensity' or 'distance'. Fields: {field_names}. ESDF logging disabled."
                )
                self._warned_no_esdf_field = True
            self.esdf_points = None
            return

        self.esdf_value_field = val_field

        gen = pc2.read_points(msg, field_names=("x", "y", "z", val_field), skip_nans=True)

        # Convert structured points -> plain floats
        data_list = [[float(p[0]), float(p[1]), float(p[2]), float(p[3])] for p in gen]
        if not data_list:
            self.esdf_points = None
            return

        self.esdf_points = np.asarray(data_list, dtype=np.float32)
        self.last_esdf_rx_time = self.get_clock().now()

    def tsdf_callback(self, msg: Layer):
        # Read geometry from message (this is authoritative)
        self.tsdf_voxel_size = float(msg.voxel_size)
        self.tsdf_voxels_per_side = int(msg.voxels_per_side)
        self.tsdf_block_size = self.tsdf_voxel_size * self.tsdf_voxels_per_side
        self.tsdf_layer_type = str(msg.layer_type)

        # ACTION_RESET means "set layer to this state": clear cache
        if msg.action == Layer.ACTION_RESET:
            self.tsdf_block_cache.clear()

        # Update cache with received blocks (ACTION_UPDATE / ACTION_MERGE / ACTION_RESET all replace provided blocks)
        for b in msg.blocks:
            key = (int(b.x_index), int(b.y_index), int(b.z_index))
            self.tsdf_block_cache[key] = b.data

        # Infer data layout once (words_per_voxel)
        if not self.tsdf_block_cache:
            return

        if self.tsdf_words_per_voxel is None:
            # Use any cached block
            any_key = next(iter(self.tsdf_block_cache.keys()))
            any_data = self.tsdf_block_cache[any_key]
            nvox = self.tsdf_voxels_per_side ** 3
            if nvox > 0:
                # data is uint32[] where each entry is a 4-byte chunk
                self.tsdf_words_per_voxel = len(any_data) / float(nvox)
                # should be integer-ish
                wv = int(round(self.tsdf_words_per_voxel))
                self.tsdf_words_per_voxel = wv
                self.tsdf_bytes_per_voxel = wv * 4

        if not self._tsdf_printed_meta:
            self.get_logger().info(
                f"TSDF meta: voxel_size={self.tsdf_voxel_size} voxels_per_side={self.tsdf_voxels_per_side} "
                f"block_size={self.tsdf_block_size} layer_type='{self.tsdf_layer_type}' "
                f"words_per_voxel={self.tsdf_words_per_voxel} bytes_per_voxel={self.tsdf_bytes_per_voxel} "
                f"cache_blocks={len(self.tsdf_block_cache)}"
            )
            self._tsdf_printed_meta = True

        # Decode weights at probes from the cached layer state
        self.tsdf_data["center_w"] = self.get_tsdf_weight_at_point(self.probe_center)
        self.tsdf_data["front_w"]  = self.get_tsdf_weight_at_point(self.probe_front)
        self.tsdf_data["side_w"]   = self.get_tsdf_weight_at_point(self.probe_side)

        self.last_tsdf_rx_time = self.get_clock().now()

    # -----------------------------
    # Main loop
    # -----------------------------
    def log_cycle(self):
        t0 = time.perf_counter()
        now_clock = self.get_clock().now()

        if self.truth_pos is None or self.esdf_points is None:
            return

        # GT State: inside a sphere around zone center
        truth_xyz = np.array([self.truth_pos.x, self.truth_pos.y, self.truth_pos.z], dtype=np.float32)
        dist_to_zone = float(np.linalg.norm(truth_xyz - self.zone))
        gt_state = 1 if dist_to_zone < self.zone_radius else 0

        # ESDF values (median within radius, fallback to nearest if close enough)
        esdf_center, cnt_c, dmin_c = self.get_esdf_stats(self.probe_center)
        esdf_front,  cnt_f, dmin_f = self.get_esdf_stats(self.probe_front)
        esdf_side,   cnt_s, dmin_s = self.get_esdf_stats(self.probe_side)

        # TSDF weights (true, from layer cache)
        w_center = float(self.tsdf_data["center_w"])
        w_front  = float(self.tsdf_data["front_w"])
        w_side   = float(self.tsdf_data["side_w"])

        # Ages
        esdf_age = (now_clock - self.last_esdf_rx_time).nanoseconds * 1e-9 if self.last_esdf_rx_time else float("nan")
        tsdf_age = (now_clock - self.last_tsdf_rx_time).nanoseconds * 1e-9 if self.last_tsdf_rx_time else float("nan")

        # Timestamp (sim time seconds)
        t_sim = now_clock.nanoseconds * 1e-9

        cycle_ms = (time.perf_counter() - t0) * 1000.0

        self.writer.writerow([
            t_sim, gt_state,
            esdf_center, esdf_front, esdf_side,
            w_center, w_front, w_side,
            esdf_age, tsdf_age,
            cycle_ms,
            cnt_c, cnt_f, cnt_s,
            dmin_c, dmin_f, dmin_s,
        ])
        self._row_count += 1
        if self._row_count % self.flush_every == 0:
            self.csv_file.flush()

        status = "PRESENT" if gt_state == 1 else "CLEARED"
        print(
            f"[{status}] ESDF(c,f,s)=({self._fmt(esdf_center)},{self._fmt(esdf_front)},{self._fmt(esdf_side)}) "
            f"W(c,f,s)=({w_center:.2f},{w_front:.2f},{w_side:.2f}) "
            f"esdf_cnt(c,f,s)=({cnt_c},{cnt_f},{cnt_s}) dmin(c,f,s)=({dmin_c:.2f},{dmin_f:.2f},{dmin_s:.2f}) "
            f"ages(e,t)=({esdf_age:.2f}s,{tsdf_age:.2f}s) {cycle_ms:.2f}ms"
        )

    # -----------------------------
    # ESDF helpers
    # -----------------------------
    def get_esdf_stats(self, target_xyz: np.ndarray):
        """
        Returns:
          value: median of ESDF values within probe_radius; if none, nearest value if within nearest_fallback; else nan
          count: number of ESDF points within probe_radius
          dmin : minimum distance from target to any ESDF point (diagnostic)
        """
        pts = self.esdf_points
        if pts is None or len(pts) == 0:
            return float("nan"), 0, float("nan")

        xyz = pts[:, 0:3]
        val = pts[:, 3]
        d = np.linalg.norm(xyz - target_xyz, axis=1)

        dmin = float(np.min(d))
        mask = d <= self.probe_radius
        count = int(np.sum(mask))

        if count > 0:
            return float(np.nanmedian(val[mask])), count, dmin

        # nearest fallback (handles voxel-center grid offsets / sparse pointcloud)
        if dmin <= self.nearest_fallback:
            i = int(np.argmin(d))
            return float(val[i]), 0, dmin

        return float("nan"), 0, dmin

    # -----------------------------
    # TSDF helpers (TRUE weight from Layer cache)
    # -----------------------------
    def get_tsdf_weight_at_point(self, point: np.ndarray) -> float:
        """
        Decodes TSDF voxel weight at a world point from cached blocks.

        Layer message guarantees:
          - voxel_size
          - voxels_per_side
          - blocks with indices
          - block.data packed in 4-byte chunks (uint32[])

        For TSDF voxels, voxblox commonly packs:
          - float distance (4 bytes)
          - float weight   (4 bytes)
        => 8 bytes per voxel => 2 uint32 per voxel.

        Some builds may include extra fields; we always read weight from bytes [4..7] of each voxel.
        """
        if self.tsdf_voxel_size is None or self.tsdf_voxels_per_side is None or self.tsdf_block_size is None:
            return 0.0
        if self.tsdf_words_per_voxel is None or self.tsdf_words_per_voxel < 2:
            return 0.0

        bx = math.floor(point[0] / self.tsdf_block_size)
        by = math.floor(point[1] / self.tsdf_block_size)
        bz = math.floor(point[2] / self.tsdf_block_size)

        key = (int(bx), int(by), int(bz))
        data = self.tsdf_block_cache.get(key, None)
        if data is None:
            return 0.0

        local = point - np.array([bx, by, bz], dtype=np.float32) * self.tsdf_block_size
        vx = int(local[0] / self.tsdf_voxel_size)
        vy = int(local[1] / self.tsdf_voxel_size)
        vz = int(local[2] / self.tsdf_voxel_size)

        if vx < 0 or vy < 0 or vz < 0 or vx >= self.tsdf_voxels_per_side or vy >= self.tsdf_voxels_per_side or vz >= self.tsdf_voxels_per_side:
            return 0.0

        idx = vx + (vy * self.tsdf_voxels_per_side) + (vz * self.tsdf_voxels_per_side * self.tsdf_voxels_per_side)
        stride = int(self.tsdf_words_per_voxel)

        # Convert voxel's data to bytes and read weight from byte offset 4
        # We avoid assuming "weight is word+1" in case of extra fields.
        voxel_word_start = idx * stride
        voxel_word_end = voxel_word_start + stride
        if voxel_word_end > len(data):
            return 0.0

        # Pack this voxel's uint32 words into bytes (little endian)
        # Weight is bytes [4:8] in the voxel payload for TSDFVoxel (distance float then weight float).
        b = b"".join(struct.pack("<I", int(w) & 0xFFFFFFFF) for w in data[voxel_word_start:voxel_word_end])

        if len(b) < 8:
            return 0.0

        w = struct.unpack_from("<f", b, 4)[0]
        if not math.isfinite(w):
            return 0.0
        return float(w)

    # -----------------------------
    # Utils
    # -----------------------------
    @staticmethod
    def _fmt(x):
        try:
            if math.isnan(x):
                return "nan"
        except Exception:
            pass
        return f"{x:.3f}"

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepResponseLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
