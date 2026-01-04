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
            'sensor_frame': '', #'panda_robot/panda_hand/camera'
            'use_tf_transforms': True,
            'use_sim_time': True,  # CRITICAL FIX

            # 2. MAPPING SETTINGS
            'tsdf_voxel_size': LaunchConfiguration('voxel_size'), 
            'tsdf_voxels_per_side': 16,
            'voxel_carving_enabled': True,
            'color_mode': 'normals',
            'method': 'merged', # 'fast' merged or 'simple' more accurate but need more CPU
            'verbose': True,
            
            # 3. PERFORMANCE OPTIMIZATION (Laptop Friendly)
            'update_mesh_every_n_sec': 2.0,   # Slow down visual updates (was 0.1)
            'update_esdf_every_n_sec': 0.1,   # Slow down map updates (was 0.1)
            'min_time_between_msgs_sec': 0.2, # Process max 5 frames per second
            'max_ray_length_m': 2.5,          # Don't map things too far away

            # 4. PUBLISHING
            'publish_esdf_map': True,      # We NEED this
            'publish_pointclouds': True,   # Useful for debug
            'publish_slices': False,       # DISABLE to save CPU
            'slice_level': 0.6,
            # --- HOLE FIXING (Sparsity Compensation) ---
            'use_sparsity_compensation_factor': True,
            'sparsity_compensation_factor': 5.0, # Strong protection against holes
            'enable_anti_grazing': True,          # Prevents rays from cutting corners
        }],
        remappings=[
            ('pointcloud', '/camera/depth/points'), 
            ('voxblox_node/esdf_map_out', '/esdf_map')
        ]
    )

    return LaunchDescription([
        voxel_size_arg,
        voxblox_node
    ])