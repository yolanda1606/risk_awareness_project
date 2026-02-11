#!/usr/bin/env python3
import math
import numpy as np
import itertools 
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

# --- HELPER FUNCTIONS ---
def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def voxel_index(x: float, y: float, z: float, vs: float):
    return (int(math.floor(x / vs)),
            int(math.floor(y / vs)),
            int(math.floor(z / vs)))

class POccMapperStage3(Node):
    def __init__(self):
        super().__init__('p_occ_mapper_stage3')

        # ---- Parameters ----
        self.declare_parameter('input_topic', '/semantic_pcl') 
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('voxel_size', 0.10)       
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('pub_thresh', -0.1) 
        
        # Risk Zone Thresholds (Meters from Initial Dynamic Workspace)
        self.declare_parameter('zone_active_dist', 0.40)
        self.declare_parameter('zone_rc_dist', 0.70)
        self.declare_parameter('zone_close_dist', 0.90)

        self.vs = self.get_parameter('voxel_size').value
        self.max_range = self.get_parameter('max_range').value
        self.pub_thresh = self.get_parameter('pub_thresh').value
        self.target_frame = self.get_parameter('world_frame').value

        self.z_active = self.get_parameter('zone_active_dist').value
        self.z_rc = self.get_parameter('zone_rc_dist').value
        self.z_close = self.get_parameter('zone_close_dist').value

        # ---- State & Memory ----
        self.map = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- THE SPATIO-SEMANTIC UPGRADE ----
        self.calibration_frames = 0
        self.calib_points = []
        self.kdtree = None
        self.r_cache = {} 
        self.zone_center = None # To easily draw the RViz spheres

        # Lookup Table: { R_Zone: { S(t)_Class: Gamma } }
        # R Zones: 0=Active, 1=Really Close, 2=Close, 3=Safe
        # S(t) Classes: 100=Dynamic, 0=Air, 200=Static
        self.gamma_table = {
            0: { 100: 0.05, 0: 0.45, 200: 0.65 }, # Active
            1: { 100: 0.15, 0: 0.50, 200: 0.75 }, # Really Close
            2: { 100: 0.25, 0: 0.50, 200: 0.85 }, # Close
            3: { 100: 0.35, 0: 0.55, 200: 0.95 }  # Safe
        }

        # ---- ROS Interface ----
        self.sub = self.create_subscription(PointCloud2, self.get_parameter('input_topic').value, self.cloud_cb, 10)
        
        self.pub = self.create_publisher(PointCloud2, '/p_occ/map', 1)
        self.pub_zones = self.create_publisher(MarkerArray, '/p_occ/r_zones', 1)
        
        self.timer = self.create_timer(0.1, self.publish_data)

        self.get_logger().info(f"Stage 3 Mapper Started. Calibrating Workspace for 50 frames...")

    def get_r_zone(self, vx, vy, vz):
        """Finds which R zone a voxel belongs to based on the initial KD-Tree."""
        idx = voxel_index(vx, vy, vz, self.vs)
        if idx in self.r_cache:
            return self.r_cache[idx]

        dist, _ = self.kdtree.query([vx, vy, vz])
        
        if dist <= self.z_active: zone = 0      # Active
        elif dist <= self.z_rc: zone = 1        # Really Close
        elif dist <= self.z_close: zone = 2     # Close
        else: zone = 3                          # Safe

        self.r_cache[idx] = zone
        return zone

    def cloud_cb(self, msg: PointCloud2):
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, Time())
        except TransformException:
            return 

        tx, ty, tz = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
        qx, qy, qz, qw = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
        cam_x, cam_y, cam_z = tx, ty, tz

        gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        
        # ==========================================================
        # PHASE 1: CALIBRATION (Learn the Workspace)
        # ==========================================================
        if self.calibration_frames < 50:
            for point in itertools.islice(gen, 0, None, 10):
                lx, ly, lz, intensity = point 
                if int(intensity) == 100: # Only save Dynamic points
                    rx = (1 - 2*qy*qy - 2*qz*qz)*lx + (2*qx*qy - 2*qz*qw)*ly + (2*qx*qz + 2*qy*qw)*lz
                    ry = (2*qx*qy + 2*qz*qw)*lx + (1 - 2*qx*qx - 2*qz*qz)*ly + (2*qy*qz - 2*qx*qw)*lz
                    rz = (2*qx*qz - 2*qy*qw)*lx + (2*qy*qz + 2*qx*qw)*ly + (1 - 2*qx*qx - 2*qy*qy)*lz
                    self.calib_points.append([rx + tx, ry + ty, rz + tz])
            
            self.calibration_frames += 1
            if self.calibration_frames == 50:
                if not self.calib_points:
                    self.get_logger().error("NO DYNAMIC POINTS SEEN IN CALIBRATION! Defaulting to origin.")
                    self.calib_points.append([0.0, 0.0, 0.0])
                    
                self.kdtree = KDTree(self.calib_points)
                
                # Calculate center for RViz visualization
                cx = sum(p[0] for p in self.calib_points) / len(self.calib_points)
                cy = sum(p[1] for p in self.calib_points) / len(self.calib_points)
                cz = sum(p[2] for p in self.calib_points) / len(self.calib_points)
                self.zone_center = (cx, cy, cz)
                
                self.get_logger().info("Calibration Complete! Workspace locked. Running Map...")
            return

        # ==========================================================
        # PHASE 2: SPATIO-SEMANTIC MAPPING
        # ==========================================================
        hit_updates = {}
        miss_updates = set()

        for i, point in enumerate(itertools.islice(gen, 0, None, 10)):
            lx, ly, lz, intensity = point 

            rx = (1 - 2*qy*qy - 2*qz*qz)*lx + (2*qx*qy - 2*qz*qw)*ly + (2*qx*qz + 2*qy*qw)*lz
            ry = (2*qx*qy + 2*qz*qw)*lx + (1 - 2*qx*qx - 2*qz*qz)*ly + (2*qy*qz - 2*qx*qw)*lz
            rz = (2*qx*qz - 2*qy*qw)*lx + (2*qy*qz + 2*qx*qw)*ly + (1 - 2*qx*qx - 2*qy*qy)*lz
            
            wx, wy, wz = rx + tx, ry + ty, rz + tz
            dist = math.sqrt((wx-cam_x)**2 + (wy-cam_y)**2 + (wz-cam_z)**2)
            
            is_hit = True
            
            # Floor Fix
            if wz < 0.05: is_hit = False
            
            if dist > self.max_range:
                dist = self.max_range
                is_hit = False
                
            if dist < 0.20: continue # 20cm Blindspot

            class_id = int(intensity)
            if class_id not in [100, 200]: class_id = 0 # Air/Unknown

            hit_idx = voxel_index(wx, wy, wz, self.vs)
            
            # LOOKUP GAMMA FOR HIT
            if is_hit:
                r_zone = self.get_r_zone(wx, wy, wz)
                g_factor = self.gamma_table[r_zone][class_id]

                if hit_idx in hit_updates:
                    current_g, _ = hit_updates[hit_idx]
                    if g_factor < current_g: hit_updates[hit_idx] = (g_factor, 1.0)
                else:
                    hit_updates[hit_idx] = (g_factor, 1.0)

            # --- RAYCASTING (Air/Misses) ---
            penetration_dist = dist + 0.15 # Ghost Shadow Fix
            steps = int(penetration_dist / self.vs)
            
            if steps > 1:
                for s in range(1, steps): 
                    ratio = s / steps
                    if (ratio * penetration_dist) < 0.20: continue
                        
                    mx, my, mz = cam_x + (wx - cam_x)*ratio, cam_y + (wy - cam_y)*ratio, cam_z + (wz - cam_z)*ratio
                    if mz < 0.05: continue
                    
                    midx = voxel_index(mx, my, mz, self.vs)
                    if midx != hit_idx or not is_hit:
                        miss_updates.add((midx, mx, my, mz))

        # --- APPLY UPDATES ---
        
        # 1. Process MISSES (Air)
        for (idx, mx, my, mz) in miss_updates:
            if idx in hit_updates: continue
                
            if idx in self.map:
                l_old = self.map[idx]['p']
                
                # LOOKUP GAMMA FOR AIR
                r_zone = self.get_r_zone(mx, my, mz)
                g_air = self.gamma_table[r_zone][0] # 0 is the S(t) code for Air
                
                l_new = g_air * l_old 
                if l_new < 0.01: del self.map[idx]
                else: self.map[idx] = {'p': l_new, 'g': g_air}

        # 2. Process HITS (Occupancy)
        for idx, (g, z_meas) in hit_updates.items():
            l_old = self.map[idx]['p'] if idx in self.map else 0.5
            l_new = (g * l_old) + ((1.0 - g) * z_meas)
            self.map[idx] = {'p': clamp01(l_new), 'g': g}

    def publish_data(self):
        if self.calibration_frames < 50: return
        
        # Publish PointCloud
        if self.map:
            pts = []
            for (ix, iy, iz), data in self.map.items():
                p = data['p'] 
                if p > self.pub_thresh:
                    x, y, z = (ix + 0.5)*self.vs, (iy + 0.5)*self.vs, (iz + 0.5)*self.vs
                    pts.append((x, y, z, float(p)))

            if pts:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = self.target_frame

                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                self.pub.publish(pc2.create_cloud(header, fields, pts))

        # Publish RViz Zones
        if self.zone_center:
            cx, cy, cz = self.zone_center
            marker_array = MarkerArray()
            
            zones = [
                (self.z_active, 1.0, 0.0, 0.0, 0.3),  # Active: Red
                (self.z_rc,     1.0, 0.5, 0.0, 0.2),  # Really Close: Orange
                (self.z_close,  0.0, 1.0, 0.0, 0.1)   # Close: Green
            ]

            for i, (radius, r, g, b, a) in enumerate(zones):
                marker = Marker()
                marker.header.frame_id = self.target_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "r_zones"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                marker.pose.position.x = cx
                marker.pose.position.y = cy
                marker.pose.position.z = cz
                
                marker.scale.x = radius * 2.0
                marker.scale.y = radius * 2.0
                marker.scale.z = radius * 2.0
                
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = a
                
                marker_array.markers.append(marker)

            self.pub_zones.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = POccMapperStage3()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()