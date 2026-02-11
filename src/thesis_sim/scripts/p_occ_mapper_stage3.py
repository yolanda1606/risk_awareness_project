#!/usr/bin/env python3
import math
import numpy as np
import itertools 

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException

# --- HELPER FUNCTIONS ---
def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def voxel_index(x: float, y: float, z: float, vs: float):
    return (int(math.floor(x / vs)),
            int(math.floor(y / vs)),
            int(math.floor(z / vs)))

class POccMapperStage3(Node):
    """
    Stage 3: Semantic-Adaptive Mapper (The "Supervisor's Logic")
    - Input: /semantic_pcl (x, y, z, intensity)
    - Logic: Complementary Filter based on Semantic Class
        * Dynamic (100) -> Fast Update (g=0.1)
        * Static (200)  -> Slow Update (g=0.9)
    - Output: Occupancy Map
    """

    def __init__(self):
        super().__init__('p_occ_mapper_stage3')

        # ---- Parameters ----
        # Subscribe to Yolanda's processed cloud
        self.declare_parameter('input_topic', '/semantic_pcl') 
        self.declare_parameter('world_frame', 'world')
        
        # Voxel Size: 0.10 is the sweet spot (Safe against "Swiss Cheese", precise enough for MPC)
        self.declare_parameter('voxel_size', 0.10)       
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('pub_thresh', -0.1) # Only show solid objects

        # ---- Constants ----
        self.vs = self.get_parameter('voxel_size').value
        self.max_range = self.get_parameter('max_range').value
        self.pub_thresh = self.get_parameter('pub_thresh').value
        self.target_frame = self.get_parameter('world_frame').value

        # ---- State ----
        # Map Structure: { (ix,iy,iz): probability_float } 
        self.map = {}

        # ---- TF Buffer ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- ROS Interface ----
        self.sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_topic').value,
            self.cloud_cb,
            10
        )
        
        self.pub = self.create_publisher(PointCloud2, '/p_occ/map', 1)
        
        # Timer (Publish at 10Hz)
        self.timer = self.create_timer(0.1, self.publish_map)

        self.get_logger().info(f"Stage 3 Mapper (Semantic) Started. VS={self.vs}m")

    def cloud_cb(self, msg: PointCloud2):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, Time())
        except TransformException as ex:
            return 

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        
        cam_x, cam_y, cam_z = tx, ty, tz

        gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        
        hit_updates = {}
        miss_updates = set()

        for i, point in enumerate(itertools.islice(gen, 0, None, 10)):
            lx, ly, lz, intensity = point 

            rx = (1 - 2*qy*qy - 2*qz*qz)*lx + (2*qx*qy - 2*qz*qw)*ly + (2*qx*qz + 2*qy*qw)*lz
            ry = (2*qx*qy + 2*qz*qw)*lx + (1 - 2*qx*qx - 2*qz*qz)*ly + (2*qy*qz - 2*qx*qw)*lz
            rz = (2*qx*qz - 2*qy*qw)*lx + (2*qy*qz + 2*qx*qw)*ly + (1 - 2*qx*qx - 2*qy*qy)*lz
            
            wx = rx + tx
            wy = ry + ty
            wz = rz + tz

            dist = math.sqrt((wx-cam_x)**2 + (wy-cam_y)**2 + (wz-cam_z)**2)
            
            is_hit = True
            
            # --- THE GHOST FIX: THE FLOOR ---
            # If the ray hits the floor, DO NOT draw a solid block.
            # But DO NOT 'continue' either! We need this ray to clear the ghosts above it!
            if wz < 0.05: 
                is_hit = False
            
            if dist > self.max_range:
                dist = self.max_range
                is_hit = False
                
            if dist < 0.20:
                continue

            class_id = int(intensity)
            if class_id == 100:     # DYNAMIC
                g_factor = 0.1      
            elif class_id == 200:   # STATIC
                g_factor = 0.9      
            else:                   # UNKNOWN 
                g_factor = 0.5      

            hit_idx = voxel_index(wx, wy, wz, self.vs)
            
            if is_hit:
                if hit_idx in hit_updates:
                    current_g, _ = hit_updates[hit_idx]
                    if g_factor < current_g:
                        hit_updates[hit_idx] = (g_factor, 1.0)
                else:
                    hit_updates[hit_idx] = (g_factor, 1.0)

            steps = int(dist / self.vs)
            if steps > 1:
                for s in range(1, steps): 
                    ratio = s / steps
                    
                    if (ratio * dist) < 0.20:
                        continue
                        
                    mx = cam_x + (wx - cam_x) * ratio
                    my = cam_y + (wy - cam_y) * ratio
                    mz = cam_z + (wz - cam_z) * ratio
                    
                    # Ignore underground misses
                    if mz < 0.05: continue
                    
                    midx = voxel_index(mx, my, mz, self.vs)
                    
                    if midx != hit_idx or not is_hit:
                        miss_updates.add(midx)

        # 1. Process MISSES first (Clearing)
        for idx in miss_updates:
            if idx in hit_updates:
                continue
                
            if idx in self.map:
                l_old = self.map[idx]['p']
                voxel_g = self.map[idx]['g'] 
                
                l_new = voxel_g * l_old 
                
                if l_new < 0.01:
                    del self.map[idx]
                else:
                    self.map[idx]['p'] = l_new

        # 2. Process HITS (Occupancy)
        for idx, (g, z_meas) in hit_updates.items():
            if idx in self.map:
                l_old = self.map[idx]['p']
            else:
                l_old = 0.5
                
            l_new = (g * l_old) + ((1.0 - g) * z_meas)
            self.map[idx] = {'p': clamp01(l_new), 'g': g}

    def publish_map(self):
        if not self.map:
            return

        # Prepare Cloud
        pts = []
        for (ix, iy, iz), voxel_data in self.map.items():
            # EXTRACT the probability from the dictionary
            p = voxel_data['p'] 
            
            if p > self.pub_thresh:
                # Calculate center
                x = (ix + 0.5) * self.vs
                y = (iy + 0.5) * self.vs
                z = (iz + 0.5) * self.vs
                
                # Visual Trick: Map Probability to Intensity for coloring in RViz
                pts.append((x, y, z, float(p)))

        if not pts:
            return

        # Create Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame

        # Define Fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack and Publish
        pc_data = pc2.create_cloud(header, fields, pts)
        self.pub.publish(pc_data)

def main(args=None):
    rclpy.init(args=args)
    node = POccMapperStage3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()