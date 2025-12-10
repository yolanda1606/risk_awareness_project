
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    voxblox_rviz_plugin_share_directory = get_package_share_directory('voxblox_rviz_plugin')
    voxblox_ros_share_directory = get_package_share_directory("voxblox_ros")
    parameters_file_path = Path(voxblox_ros_share_directory) / "config" / "cow_and_lady.yaml"

    bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value = "./data/cow_and_lady_dataset",
        description='Filepath to bag file to playback.'
    )
    
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file', 
        default_value = f"{voxblox_rviz_plugin_share_directory}/voxblox.rviz",
        description='File path to rviz config file for voxblox.'
    )
    
    tsdf_server_node = Node(
        package='voxblox_ros',
        executable='esdf_server',
        name='voxblox_node',
        output='screen',
        arguments=['-alsologtostderr'],
        parameters=[
            str(parameters_file_path),
            {'generate_esdf': True},      
            {'publish_esdf_map': True},
            {'publish_pointclouds': True},  # <--- ADD THIS LINE
            {'publish_slices': True},       # <--- ADD THIS LINE (Default is False)
            {'slice_level': 0.75},           # <--- Optional: Height in meters to cut the slice
            {'update_esdf_every_n_sec': 10.0},  # <--- 1.0(Origin) to 5.0
            {'tsdf_voxel_size': 0.10}, # <--- ADJUSTED LINE 0.05(Origin) to 0.15 
            {'tsdf_voxels_per_side': 16},
            {'voxel_carving_enabled': True},
            {'color_mode': 'color'},
            {'use_tf_transforms': False},
            {'update_mesh_every_n_sec': 10.0}, # <--- 1.0(Origin) to 5.0
            {'min_time_between_msgs_sec': 0.0},
            {'method': 'fast'},  # Simple / Merged / Fast
            {'use_const_weight': False},
            {'allow_clear': True},
            {'verbose': True},
            {'mesh_filename': launch.substitutions.LaunchConfiguration('bag_file')},
        ],
        remappings=[
            ('pointcloud', '/camera/depth_registered/points'), 
            ('transform', '/kinect/vrpn_client/estimated_transform')
        ],
    )
        
    play_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', launch.substitutions.LaunchConfiguration('bag_file'), '--loop', '-r', '0.5'],
    output='screen'
)
    
    start_rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration("rviz_config_file")],
        output='screen',
    )
    
    launch_list = [
        bag_file_arg,
        rviz_config_file_arg,

        tsdf_server_node,
        play_bag_process,
        start_rviz_process,
    ]
    
    return LaunchDescription(launch_list)

