#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


class POccMapperStage1(Node):
    """
    Stage 1: Endpoint-only occupancy fusion (NO raycasting yet).

    Input:  /camera/points  (PointCloud2) in camera frame
    Output: /p_occ/occupied_voxels (PointCloud2) in 'world' frame
            intensity = p_occ in [0,1]
    """

    def __init__(self):
        super().__init__('p_occ_mapper_stage1')

        # ---- Params ----
        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('voxel_size', 0.10)       # start 0.10m for speed
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('w_meas', 1.0)
        self.declare_parameter('w_max', 30.0)
        self.declare_parameter('p_meas_occ', 0.90)       # endpoint evidence
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('publish_threshold', 0.60)
        self.declare_parameter('max_publish_points', 200000)
        self.declare_parameter('point_stride', 4)        # process every Nth point

        self.input_topic = self.get_parameter('input_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.w_meas = float(self.get_parameter('w_meas').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.p_meas_occ = float(self.get_parameter('p_meas_occ').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_threshold = float(self.get_parameter('publish_threshold').value)
        self.max_publish_points = int(self.get_parameter('max_publish_points').value)
        self.point_stride = int(self.get_parameter('point_stride').value)

        # ---- TF ----
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Map: key=(ix,iy,iz) -> (p_occ, W) ----
        self.map = {}

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, 10)
        self.pub = self.create_publisher(PointCloud2, '/p_occ/occupied_voxels', 10)
        self.timer = self.create_timer(1.0 / max(0.5, self.publish_rate_hz), self.publish)

        self.get_logger().info(
            f"POccMapperStage1 | input={self.input_topic} | voxel={self.voxel_size:.3f}m | stride={self.point_stride}"
        )

    def _lookup_transform(self, src_frame: str, stamp):
        # Try at message time; if that fails, try "latest" (Time()).
        try:
            return self.tf_buffer.lookup_transform(
                self.world_frame, src_frame, stamp, timeout=Duration(seconds=0.2)
            )
        except Exception:
            try:
                return self.tf_buffer.lookup_transform(
                    self.world_frame, src_frame, Time(), timeout=Duration(seconds=0.2)
                )
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed {self.world_frame} <- {src_frame}: {e}")
                return None

    @staticmethod
    def _tf_to_matrix(tf) -> np.ndarray:
        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ], dtype=np.float32)
        M = np.eye(4, dtype=np.float32)
        M[:3, :3] = R
        M[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float32)
        return M

    def _voxel_index(self, x: float, y: float, z: float):
        vs = self.voxel_size
        return (int(math.floor(x / vs)),
                int(math.floor(y / vs)),
                int(math.floor(z / vs)))

    def _voxel_center(self, ix: int, iy: int, iz: int):
        vs = self.voxel_size
        return ((ix + 0.5) * vs, (iy + 0.5) * vs, (iz + 0.5) * vs)

    def on_cloud(self, msg: PointCloud2):
        tf = self._lookup_transform(msg.header.frame_id, msg.header.stamp)
        if tf is None:
            return
        M = self._tf_to_matrix(tf)

        stride = max(1, self.point_stride)
        i = 0

        # IMPORTANT: skip_nans=True does NOT remove +inf.
        for (x, y, z) in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if (i % stride) != 0:
                i += 1
                continue
            i += 1

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            r = math.sqrt(x*x + y*y + z*z)
            if r <= 0.05 or r > self.max_range:
                continue

            pw = M @ np.array([x, y, z, 1.0], dtype=np.float32)
            wx, wy, wz = float(pw[0]), float(pw[1]), float(pw[2])

            key = self._voxel_index(wx, wy, wz)

            # Weighted average fusion
            p_old, w_old = self.map.get(key, (0.5, 0.0))
            w_meas = self.w_meas
            p_meas = self.p_meas_occ

            w_sum = w_old + w_meas
            p_new = (w_old * p_old + w_meas * p_meas) / w_sum
            w_new = min(self.w_max, w_sum)

            self.map[key] = (clamp01(p_new), w_new)

    def publish(self):
        if not self.map:
            return

        pts = []
        count = 0
        for (ix, iy, iz), (p, w) in self.map.items():
            if p < self.publish_threshold:
                continue
            x, y, z = self._voxel_center(ix, iy, iz)
            pts.append((x, y, z, float(p)))
            count += 1
            if count >= self.max_publish_points:
                break

        if not pts:
            return

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.world_frame

        cloud = pc2.create_cloud(h, fields, pts)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = POccMapperStage1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

