import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # 1. SETUP: Define paths
    pkg_panda = get_package_share_directory('panda_description')
    xacro_file = os.path.join(pkg_panda, 'urdf', 'panda.urdf.xacro')
    gz_resource_path = os.path.dirname(pkg_panda)
    world_path = '/home/dummy/Projects/joel/src/panda_ign_moveit2/scenarios/lab_visibility.sdf'

    # 2. ENVIRONMENT VARIABLES
    set_gz_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )

    set_gz_plugin_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib' 
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. NODES
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'panda_robot', '-z', '0.0', '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # BRIDGE CONFIGURATION
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # 2. Camera Info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            
            # 3. DYNAMIC CAMERA (End-Effector)
            '/world/obstacle_world/model/panda_robot/link/panda_hand/sensor/camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            
            # 4. STATIC CAMERA (Corrected)
            '/static_camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            # FUSION TRICK: Remap BOTH to the SAME topic
            ('/world/obstacle_world/model/panda_robot/link/panda_hand/sensor/camera/depth_image/points', '/camera/combined_points'),
            ('/static_camera/depth_image/points', '/camera/combined_points'),
        ],
        output='screen'
    )
    

    # DYNAMIC CAMERA TF (Existing - Connects URDF to Optical Frame)
    # Note: Ensure this child frame matches what Gazebo publishes in the header!
    static_tf_dynamic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_dynamic',
        arguments=[
            "--x", "0.04", "--y", "0", "--z", "0.04",
            "--qx", "0", "--qy", "-0.707", "--qz", "0", "--qw", "0.707",
            "--frame-id", "panda_hand", 
            "--child-frame-id", "panda_robot/panda_hand/camera" # Verify this matches Gazebo header
        ]
    )

    # STATIC CAMERA TF
    # Matches the NEW SDF pose: x=2.0, z=1.3, pitch=0.65
    static_tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_static',
        arguments=[
            "--x", "2.0", "--y", "0", "--z", "1.3",
            "--roll", "0", "--pitch", "0.65", "--yaw", "3.14159", 
            "--frame-id", "world", 
            "--child-frame-id", "static_camera/link/static_depth_camera"
        ]
    )
    # 1. Create the spawner node for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],        output='screen'
    )


    # 2. (Optional) Create the spawner for your arm controller (e.g., joint_trajectory_controller)
    arm_controller_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],    
         output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_gz_plugin_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        static_tf_dynamic,
        static_tf_static,
        joint_state_broadcaster_spawner, # <--- ADD THIS
        arm_controller_spawner
    ])