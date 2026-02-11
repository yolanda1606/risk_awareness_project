#!/usr/bin/env python3
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
    """
    Logs step-response signals at 3 probe points around a floor cube.

    Probes (world frame):
      TOP:    (0.50, 0.00, 0.21)  # slightly above top surface (z=0.20)
      CENTER: (0.50, 0.00, 0.10)  # geometric cube center (for 0.20m cube on floor)
      SIDE:   (0.50, 0.11, 0.10)  # slightly outside side face (face at y=0.10)

    What voxel are we analyzing?
      For each probe point p, we read the TSDF voxel cell that CONTAINS p.
      We log:
        - Block indices (bx,by,bz)
        - Voxel indices inside block (vx,vy,vz)
        - Voxel center in world (vcx,vcy,vcz)

    TSDF:
      Reads /tsdf_map_out (voxblox_msgs/Layer) and decodes TRUE weight float from packed uint32[].

    ESDF:
      Reads /esdf_pointcloud (PointCloud2) and samples intensity/distance near each probe.

    End-effector:
      Fixed position (world frame) as parameters, logs distance to TOP probe.
    """

    def __init__(self):
        super().__init__("step_response_logger")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("csv_path", "thesis_step_response.csv")
        self.declare_parameter("log_period_s", 0.1)
        self.declare_parameter("flush_every_n_rows", 20)

        # Fixed end-effector position (world)
        self.declare_parameter("ee_x", 0.0)
        self.declare_parameter("ee_y", 0.0)
        self.declare_parameter("ee_z", 0.0)

        # Ground truth "present zone" (optional)
        self.declare_parameter("zone_x", 0.5)
        self.declare_parameter("zone_y", 0.0)
        self.declare_parameter("zone_z", 0.1)
        self.declare_parameter("zone_radius_m", 0.25)

        # ESDF sampling
        self.declare_parameter("probe_radius_m", 0.15)
        self.declare_parameter("nearest_fallback_m", 0.35)

        # Probes (your requested coordinates)
        self.declare_parameter("probe_top_x", 0.50)
        self.declare_parameter("probe_top_y", 0.00)
        self.declare_parameter("probe_top_z", 0.21)

        self.declare_parameter("probe_center_x", 0.50)
        self.declare_parameter("probe_center_y", 0.00)
        self.declare_parameter("probe_center_z", 0.10)

        self.declare_parameter("probe_side_x", 0.50)
        self.declare_parameter("probe_side_y", 0.11)
        self.declare_parameter("probe_side_z", 0.10)

        # Topics
        self.declare_parameter("topic_box_ground_truth", "/box_ground_truth")
        self.declare_parameter("topic_esdf_cloud", "/esdf_pointcloud")
        self.declare_parameter("topic_tsdf_layer", "/tsdf_map_out")

        # Read params
        self.csv_path = str(self.get_parameter("csv_path").value)
        self.log_period_s = float(self.get_parameter("log_period_s").value)
        self.flush_every = int(self.get_parameter("flush_every_n_rows").value)

        self.ee = np.array([
            float(self.get_parameter("ee_x").value),
            float(self.get_parameter("ee_y").value),
            float(self.get_parameter("ee_z").value),
        ], dtype=np.float32)

        self.zone = np.array([
            float(self.get_parameter("zone_x").value),
            float(self.get_parameter("zone_y").value),
            float(self.get_parameter("zone_z").value),
        ], dtype=np.float32)
        self.zone_radius = float(self.get_parameter("zone_radius_m").value)

        self.probe_radius = float(self.get_parameter("probe_radius_m").value)
        self.nearest_fallback = float(self.get_parameter("nearest_fallback_m").value)

        self.probe_top = np.array([
            float(self.get_parameter("probe_top_x").value),
            float(self.get_parameter("probe_top_y").value),
            float(self.get_parameter("probe_top_z").value),
        ], dtype=np.float32)

        self.probe_center = np.array([
            float(self.get_parameter("probe_center_x").value),
            float(self.get_parameter("probe_center_y").value),
            float(self.get_parameter("probe_center_z").value),
        ], dtype=np.float32)

        self.probe_side = np.array([
            float(self.get_parameter("probe_side_x").value),
            float(self.get_parameter("probe_side_y").value),
            float(self.get_parameter("probe_side_z").value),
        ], dtype=np.float32)

        self.topic_truth = str(self.get_parameter("topic_box_ground_truth").value)
        self.topic_esdf = str(self.get_parameter("topic_esdf_cloud").value)
        self.topic_tsdf = str(self.get_parameter("topic_tsdf_layer").value)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(PoseStamped, self.topic_truth, self.truth_callback, 10)
        self.create_subscription(PointCloud2, self.topic_esdf, self.esdf_callback, 10)
        self.create_subscription(Layer, self.topic_tsdf, self.tsdf_callback, 10)

        # -----------------------------
        # State
        # -----------------------------
        self.box_truth_pos = None

        # ESDF
        self.esdf_points = None  # Nx4 float32 [x,y,z,val]
        self.esdf_value_field = None
        self._warned_no_esdf_field = False
        self.last_esdf_rx_time = None

        # TSDF
        self.tsdf_voxel_size = None
        self.tsdf_voxels_per_side = None
        self.tsdf_block_size = None
        self.tsdf_words_per_voxel = None
        self.tsdf_layer_type = None
        self.last_tsdf_rx_time = None

        self.tsdf_block_cache = {}  # (bx,by,bz) -> uint32[] data
        self._tsdf_printed_meta = False
        self._tsdf_printed_probe_map = False

        self.tsdf_data = {
            "top_w": 0.0, "center_w": 0.0, "side_w": 0.0,
            "top_idx": None, "center_idx": None, "side_idx": None
        }

        # -----------------------------
        # CSV
        # -----------------------------
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "t_sim",
            "GT_State",
            "EE_to_top_dist_m",

            "ESDF_top", "ESDF_center", "ESDF_side",
            "Weight_top", "Weight_center", "Weight_side",

            "esdf_age_s", "tsdf_age_s",
            "logger_cycle_ms",

            "ESDF_count_top", "ESDF_count_center", "ESDF_count_side",
            "ESDF_dmin_top", "ESDF_dmin_center", "ESDF_dmin_side",

            # TSDF indices (what voxel are we reading?)
            "TSDF_top_bx", "TSDF_top_by", "TSDF_top_bz", "TSDF_top_vx", "TSDF_top_vy", "TSDF_top_vz",
            "TSDF_center_bx", "TSDF_center_by", "TSDF_center_bz", "TSDF_center_vx", "TSDF_center_vy", "TSDF_center_vz",
            "TSDF_side_bx", "TSDF_side_by", "TSDF_side_bz", "TSDF_side_vx", "TSDF_side_vy", "TSDF_side_vz",

            # voxel centers in world
            "TSDF_top_vcx", "TSDF_top_vcy", "TSDF_top_vcz",
            "TSDF_center_vcx", "TSDF_center_vcy", "TSDF_center_vcz",
            "TSDF_side_vcx", "TSDF_side_vcy", "TSDF_side_vcz",
        ])
        self._row_count = 0

        # Timer
        self.create_timer(self.log_period_s, self.log_cycle)

        self.get_logger().info(
            f"Logger started. CSV: {self.csv_path}\n"
            f"Fixed EE: {self.ee.tolist()} (world)\n"
            f"Probes:\n"
            f"  TOP    {self.probe_top.tolist()}\n"
            f"  CENTER {self.probe_center.tolist()}\n"
            f"  SIDE   {self.probe_side.tolist()}\n"
            f"ESDF radius={self.probe_radius} m"
        )

    # -----------------------------
    # Callbacks
    # -----------------------------
    def truth_callback(self, msg: PoseStamped):
        self.box_truth_pos = msg.pose.position

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
        data_list = [[float(p[0]), float(p[1]), float(p[2]), float(p[3])] for p in gen]
        if not data_list:
            self.esdf_points = None
            return

        self.esdf_points = np.asarray(data_list, dtype=np.float32)
        self.last_esdf_rx_time = self.get_clock().now()

    def tsdf_callback(self, msg: Layer):
        self.tsdf_voxel_size = float(msg.voxel_size)
        self.tsdf_voxels_per_side = int(msg.voxels_per_side)
        self.tsdf_block_size = self.tsdf_voxel_size * self.tsdf_voxels_per_side
        self.tsdf_layer_type = str(msg.layer_type)

        # Reset cache if requested
        if msg.action == Layer.ACTION_RESET:
            self.tsdf_block_cache.clear()

        # Cache/update blocks
        for b in msg.blocks:
            key = (int(b.x_index), int(b.y_index), int(b.z_index))
            self.tsdf_block_cache[key] = b.data

        # Infer words_per_voxel once
        if self.tsdf_words_per_voxel is None and self.tsdf_block_cache:
            any_key = next(iter(self.tsdf_block_cache.keys()))
            any_data = self.tsdf_block_cache[any_key]
            nvox = self.tsdf_voxels_per_side ** 3
            if nvox > 0:
                self.tsdf_words_per_voxel = int(round(len(any_data) / float(nvox)))

        if (not self._tsdf_printed_meta) and (self.tsdf_words_per_voxel is not None):
            self.get_logger().info(
                f"TSDF meta: voxel_size={self.tsdf_voxel_size} voxels_per_side={self.tsdf_voxels_per_side} "
                f"block_size={self.tsdf_block_size} layer_type='{self.tsdf_layer_type}' "
                f"words_per_voxel={self.tsdf_words_per_voxel} cache_blocks={len(self.tsdf_block_cache)}"
            )
            self._tsdf_printed_meta = True

        # Decode TSDF weights at probes
        w_top, idx_top = self.get_tsdf_weight_and_indices(self.probe_top)
        w_cen, idx_cen = self.get_tsdf_weight_and_indices(self.probe_center)
        w_sid, idx_sid = self.get_tsdf_weight_and_indices(self.probe_side)

        self.tsdf_data["top_w"], self.tsdf_data["top_idx"] = w_top, idx_top
        self.tsdf_data["center_w"], self.tsdf_data["center_idx"] = w_cen, idx_cen
        self.tsdf_data["side_w"], self.tsdf_data["side_idx"] = w_sid, idx_sid

        self.last_tsdf_rx_time = self.get_clock().now()

        if (not self._tsdf_printed_probe_map) and idx_top and idx_cen and idx_sid:
            self._print_probe_voxel_mapping_once()
            self._tsdf_printed_probe_map = True

    # -----------------------------
    # Main loop
    # -----------------------------
    def log_cycle(self):
        t0 = time.perf_counter()
        now_clock = self.get_clock().now()

        if self.esdf_points is None:
            return

        # GT state (optional)
        gt_state = 0
        if self.box_truth_pos is not None:
            truth_xyz = np.array([self.box_truth_pos.x, self.box_truth_pos.y, self.box_truth_pos.z], dtype=np.float32)
            gt_state = 1 if float(np.linalg.norm(truth_xyz - self.zone)) < self.zone_radius else 0

        # Fixed EE -> TOP probe distance
        ee_to_top = float(np.linalg.norm(self.ee - self.probe_top))

        # ESDF stats at probes
        esdf_top,    cnt_t, dmin_t = self.get_esdf_stats(self.probe_top)
        esdf_center, cnt_c, dmin_c = self.get_esdf_stats(self.probe_center)
        esdf_side,   cnt_s, dmin_s = self.get_esdf_stats(self.probe_side)

        # TSDF weights
        w_top = float(self.tsdf_data["top_w"])
        w_center = float(self.tsdf_data["center_w"])
        w_side = float(self.tsdf_data["side_w"])

        # Ages
        esdf_age = (now_clock - self.last_esdf_rx_time).nanoseconds * 1e-9 if self.last_esdf_rx_time else float("nan")
        tsdf_age = (now_clock - self.last_tsdf_rx_time).nanoseconds * 1e-9 if self.last_tsdf_rx_time else float("nan")

        t_sim = now_clock.nanoseconds * 1e-9
        cycle_ms = (time.perf_counter() - t0) * 1000.0

        # Indices and voxel centers
        it = self.tsdf_data["top_idx"]
        ic = self.tsdf_data["center_idx"]
        isd = self.tsdf_data["side_idx"]

        def idx_or_nan(idx, k):
            return idx[k] if idx is not None else float("nan")

        # CSV row
        self.writer.writerow([
            t_sim,
            gt_state,
            ee_to_top,

            esdf_top, esdf_center, esdf_side,
            w_top, w_center, w_side,

            esdf_age, tsdf_age,
            cycle_ms,

            cnt_t, cnt_c, cnt_s,
            dmin_t, dmin_c, dmin_s,

            idx_or_nan(it, "bx"), idx_or_nan(it, "by"), idx_or_nan(it, "bz"),
            idx_or_nan(it, "vx"), idx_or_nan(it, "vy"), idx_or_nan(it, "vz"),

            idx_or_nan(ic, "bx"), idx_or_nan(ic, "by"), idx_or_nan(ic, "bz"),
            idx_or_nan(ic, "vx"), idx_or_nan(ic, "vy"), idx_or_nan(ic, "vz"),

            idx_or_nan(isd, "bx"), idx_or_nan(isd, "by"), idx_or_nan(isd, "bz"),
            idx_or_nan(isd, "vx"), idx_or_nan(isd, "vy"), idx_or_nan(isd, "vz"),

            idx_or_nan(it, "vcx"), idx_or_nan(it, "vcy"), idx_or_nan(it, "vcz"),
            idx_or_nan(ic, "vcx"), idx_or_nan(ic, "vcy"), idx_or_nan(ic, "vcz"),
            idx_or_nan(isd, "vcx"), idx_or_nan(isd, "vcy"), idx_or_nan(isd, "vcz"),
        ])
        self._row_count += 1
        if self._row_count % self.flush_every == 0:
            self.csv_file.flush()

        status = "PRESENT" if gt_state == 1 else "CLEARED"
        print(
            f"[{status}] EE->TOP={ee_to_top:.3f}m | "
            f"ESDF(top,cen,side)=({self._fmt(esdf_top)},{self._fmt(esdf_center)},{self._fmt(esdf_side)}) | "
            f"W(top,cen,side)=({w_top:.2f},{w_center:.2f},{w_side:.2f}) | "
            f"ages(e,t)=({esdf_age:.2f}s,{tsdf_age:.2f}s) {cycle_ms:.2f}ms"
        )

    # -----------------------------
    # ESDF helper
    # -----------------------------
    def get_esdf_stats(self, target_xyz: np.ndarray):
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

        if dmin <= self.nearest_fallback:
            i = int(np.argmin(d))
            return float(val[i]), 0, dmin

        return float("nan"), 0, dmin

    # -----------------------------
    # TSDF decoding: TRUE weight + indices
    # -----------------------------
    def get_tsdf_weight_and_indices(self, point: np.ndarray):
        if self.tsdf_voxel_size is None or self.tsdf_voxels_per_side is None or self.tsdf_block_size is None:
            return 0.0, None
        if self.tsdf_words_per_voxel is None or self.tsdf_words_per_voxel < 2:
            return 0.0, None

        idx = self.world_to_tsdf_indices(point)
        key = (idx["bx"], idx["by"], idx["bz"])
        data = self.tsdf_block_cache.get(key, None)
        if data is None:
            return 0.0, idx

        vps = self.tsdf_voxels_per_side
        lin = idx["vx"] + idx["vy"] * vps + idx["vz"] * vps * vps

        stride = int(self.tsdf_words_per_voxel)

        # TSDFVoxel layout: [distance(float), weight(float)] -> weight is second float
        wi = lin * stride + 1
        if wi < 0 or wi >= len(data):
            return 0.0, idx

        raw = int(data[wi]) & 0xFFFFFFFF
        w = struct.unpack("<f", struct.pack("<I", raw))[0]
        if not math.isfinite(w):
            w = 0.0
        return float(w), idx

    def world_to_tsdf_indices(self, point: np.ndarray):
        bs = self.tsdf_block_size
        vs = self.tsdf_voxel_size
        vps = self.tsdf_voxels_per_side

        bx = int(math.floor(float(point[0]) / bs))
        by = int(math.floor(float(point[1]) / bs))
        bz = int(math.floor(float(point[2]) / bs))

        lx = float(point[0]) - bx * bs
        ly = float(point[1]) - by * bs
        lz = float(point[2]) - bz * bs

        vx = int(math.floor(lx / vs))
        vy = int(math.floor(ly / vs))
        vz = int(math.floor(lz / vs))

        vx = max(0, min(vps - 1, vx))
        vy = max(0, min(vps - 1, vy))
        vz = max(0, min(vps - 1, vz))

        vcx = bx * bs + (vx + 0.5) * vs
        vcy = by * bs + (vy + 0.5) * vs
        vcz = bz * bs + (vz + 0.5) * vs

        return {"bx": bx, "by": by, "bz": bz,
                "vx": vx, "vy": vy, "vz": vz,
                "vcx": vcx, "vcy": vcy, "vcz": vcz}

    def _print_probe_voxel_mapping_once(self):
        it = self.tsdf_data["top_idx"]
        ic = self.tsdf_data["center_idx"]
        isd = self.tsdf_data["side_idx"]

        self.get_logger().info("=== TSDF Probe -> Voxel Mapping (what voxel are we analyzing?) ===")
        self.get_logger().info(
            f"TSDF voxel_size={self.tsdf_voxel_size} voxels_per_side={self.tsdf_voxels_per_side} "
            f"block_size={self.tsdf_block_size} words_per_voxel={self.tsdf_words_per_voxel}"
        )
        self.get_logger().info(
            f"PROBE_TOP    world={self.probe_top.tolist()}    block=({it['bx']},{it['by']},{it['bz']}) voxel=({it['vx']},{it['vy']},{it['vz']}) "
            f"voxel_center=({it['vcx']:.3f},{it['vcy']:.3f},{it['vcz']:.3f})"
        )
        self.get_logger().info(
            f"PROBE_CENTER world={self.probe_center.tolist()} block=({ic['bx']},{ic['by']},{ic['bz']}) voxel=({ic['vx']},{ic['vy']},{ic['vz']}) "
            f"voxel_center=({ic['vcx']:.3f},{ic['vcy']:.3f},{ic['vcz']:.3f})"
        )
        self.get_logger().info(
            f"PROBE_SIDE   world={self.probe_side.tolist()}   block=({isd['bx']},{isd['by']},{isd['bz']}) voxel=({isd['vx']},{isd['vy']},{isd['vz']}) "
            f"voxel_center=({isd['vcx']:.3f},{isd['vcy']:.3f},{isd['vcz']:.3f})"
        )
        same_tc = (it["bx"], it["by"], it["bz"], it["vx"], it["vy"], it["vz"]) == (ic["bx"], ic["by"], ic["bz"], ic["vx"], ic["vy"], ic["vz"])
        same_ts = (it["bx"], it["by"], it["bz"], it["vx"], it["vy"], it["vz"]) == (isd["bx"], isd["by"], isd["bz"], isd["vx"], isd["vy"], isd["vz"])
        self.get_logger().info(f"top==center voxel? {same_tc} | top==side voxel? {same_ts}")
        self.get_logger().info("===============================================================")

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
