from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # ARGUMENTS
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', default_value="0.05", description='Voxblox voxel size'
    )

    # 1. REALSENSE CAMERA NODE
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'enable_pointcloud': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'ordered_pc': True,
            
            # --- BANDWIDTH FIX START ---
            # Lower resolution to prevent USB timeout in WSL2
            'depth_module.profile': '424x240x15', 
            'rgb_camera.profile': '424x240x15',
            # Disable extra streams you don't need
            'enable_infra1': False,
            'enable_infra2': False,
            'global_time_enabled': False,
            # --- BANDWIDTH FIX END ---
        }]
    )

    # 2. STATIC TRANSFORM (Handheld Mode)
    # If the camera is just sitting on your desk or you are holding it,
    # we need to fake a "world" frame so Voxblox knows where the camera is.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ["0", "0", "1", "0", "0", "0", "world", "camera_link"]
    )

    # 3. VOXBLOX NODE (Configured for Real Data)
    voxblox_node = Node(
        package='voxblox_ros',
        executable='esdf_server',
        name='voxblox_node',
        output='screen',
        parameters=[{
            # FRAMES
            'world_frame': 'world',
            'sensor_frame': 'camera_depth_optical_frame',
            'use_tf_transforms': True,
            
            # TIMING (CRITICAL CHANGE)
            'use_sim_time': False, # <--- MUST be False for real hardware

            # MAPPING
            'tsdf_voxel_size': LaunchConfiguration('voxel_size'), 
            'tsdf_voxels_per_side': 16,
            'voxel_carving_enabled': True,
            'color_mode': 'normals', # RealSense provides color!
            'method': 'fast',
            'verbose': True,
            
            # PERFORMANCE
            'update_mesh_every_n_sec': 1.0, 
            'max_ray_length_m': 3.0,
            
            # OUTPUT
            'publish_esdf_map': True,
            'publish_pointclouds': True,
            'publish_slices': True,
            'slice_level': 0.9  ,
        }],
        remappings=[
            # Remap Voxblox input to RealSense output
            ('pointcloud', '/camera/realsense_camera/depth/color/points'),
            ('voxblox_node/esdf_map_out', '/esdf_map')
        ]
    )

    return LaunchDescription([
        voxel_size_arg,
        realsense_node,
        static_tf,
        voxblox_node
    ])