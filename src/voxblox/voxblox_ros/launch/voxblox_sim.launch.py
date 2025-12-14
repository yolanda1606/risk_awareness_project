from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Optional: Allow changing voxel size from command line
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', 
        default_value = "0.05",
        description='Voxblox voxel size'
    )

    voxblox_node = Node(
        package='voxblox_ros',
        executable='esdf_server',  # Using ESDF server for maps + collision
        name='voxblox_node',
        output='screen',
        parameters=[{
            # 1. FRAMES (Critical for Simulation)
            'world_frame': 'world',
            'sensor_frame': 'panda_robot/panda_link7/camera', # Matches your static_tf node
            'use_tf_transforms': True,  # ENABLED: Sim provides real TF

            # 2. MAPPING SETTINGS
            'tsdf_voxel_size': LaunchConfiguration('voxel_size'), 
            'tsdf_voxels_per_side': 16,
            'voxel_carving_enabled': True,
            'color_mode': 'normals',
            'method': 'fast',
            'verbose': True,
            
            # 3. UPDATE RATES (Adjust if sim is slow)
            'update_mesh_every_n_sec': 0.5,
            'update_esdf_every_n_sec': 0.5,
            'min_time_between_msgs_sec': 0.0,

            # 4. PUBLISHING
            'publish_esdf_map': True,
            'publish_pointclouds': True,
            'publish_slices': True,
            'slice_level': 1.0,
        }],
        remappings=[
            # CONNECT SIMULATION DATA HERE
            ('pointcloud', '/camera/depth/points'), 
            ('voxblox_node/esdf_map_out', '/esdf_map')
        ]
    )

    return LaunchDescription([
        voxel_size_arg,
        voxblox_node
    ])