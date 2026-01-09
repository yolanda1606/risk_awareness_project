from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', 
        default_value = "0.05", # Adjusted from 0.02 to 0.05
        description='Voxblox voxel size'
    )

    voxblox_node = Node(
        package='voxblox_ros',
        executable='esdf_server',
        name='voxblox_node',
        output='screen',
        parameters=[{
            # 1. FRAMES
            'world_frame': 'world',
            #'truncation_distance': 0.1,
            'sensor_frame': '', # <--- Leave EMPTY for multi-camera setups
            'allow_clear': True,  # Ensures the static camera can delete the "ghosts" created by the moving arm
            'use_tf_transforms': True,
            'use_sim_time': True,  # CRITICAL FIX

            # 2. MAPPING SETTINGS
            'tsdf_voxel_size': LaunchConfiguration('voxel_size'), 
            'tsdf_voxels_per_side': 16,
            'voxel_carving_enabled': True,
            'color_mode': 'normals',
            'method': 'simple', # 'fast' is good for laptops /better CPU usage. 'merged' optimal or 'simple' more accurate but need more CPU
            'verbose': True,
            
            # 3. PERFORMANCE OPTIMIZATION (CRITICAL FIXES HERE)
            'update_mesh_every_n_sec': 0.5,    # <--- CHANGE 5.0 to 0.5 (Fast feedback)
            'update_esdf_every_n_sec': 0.5,    # <--- Match the mesh rate
            'publish_map_every_n_sec': 0.5,    # <--- Ensure map is sent frequently
            'min_time_between_msgs_sec': 0.0,  # <--- Let all data through for now
            'max_ray_length_m': 2.5,

            # 4. PUBLISHING
            'publish_tsdf_map': True,
            'publish_esdf_map': True,
            'publish_pointclouds': True,
            'publish_slices': False,
            'slice_level': 0.75,


            # --- HOLE FIXING (Sparsity Compensation) ---
            'use_sparsity_compensation_factor': True,
            'sparsity_compensation_factor': 5.0, # Strong protection against holes
            'enable_anti_grazing': True,          # Prevents rays from cutting corners
        }],
        remappings=[
            # 1. Input Data
            ('pointcloud', '/camera/combined_points'), 
            
            # 2. ESDF Output
            ('voxblox_node/esdf_map_out', '/esdf_map'),

            # 3. TSDF Output (THIS WAS MISSING)
            # Without this, the inspector node hears silence.
            ('tsdf_map_out', '/tsdf_map_out') 
        ]
    )

    return LaunchDescription([
        voxel_size_arg,
        voxblox_node
    ])