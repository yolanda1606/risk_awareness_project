#!/usr/bin/env python3
import math
import numpy as np
import itertools # <--- REQUIRED for efficient skipping

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def voxel_index(x: float, y: float, z: float, vs: float):
    return (int(math.floor(x / vs)),
            int(math.floor(y / vs)),
            int(math.floor(z / vs)))

class POccMapperStage2(Node):
    """
    Stage 2: Raycasting Fusion (Optimized).
    - Uses downsampling to run in real-time on Python.
    - Parameters tuned for Dynamic Obstacles.
    """

    def __init__(self):
        super().__init__('p_occ_mapper_stage2')

        # ---- Params ----
        self.declare_parameter('input_topic', '/camera/points') 
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'static_camera_link') 
        self.declare_parameter('voxel_size', 0.12)       
        self.declare_parameter('max_range', 4.0)
        
        # Probabilistic Logic (Dynamic Tuning)
        self.declare_parameter('w_meas', 2.0)      # High learning rate
        self.declare_parameter('w_max', 10.0)      # Low inertia (fast forgetting)
        self.declare_parameter('p_meas_occ', 0.95)  # Strong occupancy
        self.declare_parameter('p_meas_free', 0.1) # Strong clearing

        # Output settings
        self.declare_parameter('publish_rate', 5.0) 
        self.declare_parameter('publish_threshold', 0.1) 

        # Fetch params
        self.input_topic = self.get_parameter('input_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.vs = self.get_parameter('voxel_size').value
        self.max_range = self.get_parameter('max_range').value
        
        self.w_meas = self.get_parameter('w_meas').value
        self.w_max = self.get_parameter('w_max').value
        self.p_meas_occ = self.get_parameter('p_meas_occ').value
        self.p_meas_free = self.get_parameter('p_meas_free').value
        self.pub_thresh = self.get_parameter('publish_threshold').value

        self.map = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/p_occ/occupied_voxels', 10)
        
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_map)
        
        self.get_logger().info(f"Stage 2 Mapper Initialized (Optimized). Listening to: {self.input_topic}")

    def get_rotation_matrix(self, q):
        """Helper to convert Quaternion to 3x3 Rotation Matrix (Numpy)"""
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx**2 - 2*qy**2]
        ])

    def cloud_cb(self, msg: PointCloud2):
        if msg.width * msg.height == 0:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                rclpy.time.Time())
        except TransformException as ex:
            return

        # 1. Prepare Matrices
        t_vec = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ])
        r_mat = self.get_rotation_matrix(trans.transform.rotation)
        cam_x, cam_y, cam_z = t_vec

        # 2. Read Points
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        occupied_updates = set()
        free_updates = set()

        # --- OPTIMIZATION: Process every 50th point ---
        # This keeps the geometry but drastically reduces CPU load.
        # 1% of 300k points = 3000 rays (More than enough for 15cm voxels)
        step_size = 50 
        
        for p in itertools.islice(gen, 0, None, step_size):
            p_cam = np.array([p[0], p[1], p[2]])

            if not np.isfinite(p_cam).all():
                continue

            # Transform to World
            p_world = np.dot(r_mat, p_cam) + t_vec
            px, py, pz = p_world[0], p_world[1], p_world[2]
            
            # --- Raycasting ---
            
            # 1. Hit (Endpoint)
            occ_idx = voxel_index(px, py, pz, self.vs)
            occupied_updates.add(occ_idx)
            
            # 2. Miss (Free Space)
            dist = math.sqrt((px-cam_x)**2 + (py-cam_y)**2 + (pz-cam_z)**2)
            
            if dist > self.max_range or dist < 0.1:
                continue
                
            steps = int(dist / (self.vs)) 
            
            for s in range(1, steps): 
                ratio = s / steps
                rx = cam_x + (px - cam_x) * ratio
                ry = cam_y + (py - cam_y) * ratio
                rz = cam_z + (pz - cam_z) * ratio
                
                free_idx = voxel_index(rx, ry, rz, self.vs)
                if free_idx != occ_idx:
                    free_updates.add(free_idx)

        # --- Batch Update ---
        for idx in free_updates:
            if idx in occupied_updates: continue
            self.update_voxel(idx, self.p_meas_free) 

        for idx in occupied_updates:
            self.update_voxel(idx, self.p_meas_occ) 

    def update_voxel(self, key, p_meas):
        if key in self.map:
            p_old, w_old = self.map[key]
        else:
            p_old, w_old = 0.5, 0.0

        w_sum = w_old + self.w_meas
        p_new = (w_old * p_old + self.w_meas * p_meas) / w_sum
        w_new = min(self.w_max, w_sum)
        
        self.map[key] = (clamp01(p_new), w_new)

    def _voxel_center(self, ix, iy, iz):
        return ((ix + 0.5) * self.vs,
                (iy + 0.5) * self.vs,
                (iz + 0.5) * self.vs)

    def publish_map(self):
        if not self.map:
            return

        pts = []
        for (ix, iy, iz), (p, w) in self.map.items():
            if p > self.pub_thresh:
                x, y, z = self._voxel_center(ix, iy, iz)
                pts.append((x, y, z, float(p), float(w)))

        if not pts:
            return

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='weight', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.world_frame

        cloud = pc2.create_cloud(h, fields, pts)
        self.pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = POccMapperStage2()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()