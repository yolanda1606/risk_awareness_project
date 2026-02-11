import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. SETUP: Define paths
    pkg_panda = get_package_share_directory('panda_description')
    xacro_file = os.path.join(pkg_panda, 'urdf', 'panda.urdf.xacro')
    gz_resource_path = os.path.dirname(pkg_panda)
    
    # --- FIX STARTS HERE ---
    # Find the installed location of your thesis_sim package
    pkg_thesis_sim = get_package_share_directory('thesis_sim')

    # Construct the path dynamically to the NEW 'worlds' folder
    world_path = os.path.join(pkg_thesis_sim, 'worlds', 'clean_world.sdf')
    # --- FIX ENDS HERE ---

    print(f"[INFO] Loading World: {world_path}")

    # 2. ENVIRONMENT VARIABLES
    set_gz_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )

    # Note: You might need to add pkg_thesis_sim to the path if your world uses models from there
    # set_gz_resource_path_sim = AppendEnvironmentVariable(
    #     name='IGN_GAZEBO_RESOURCE_PATH',
    #     value=os.path.dirname(pkg_thesis_sim)
    # )

    set_gz_plugin_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib' 
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. NODES
    
    # A. Robot State Publisher
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

    # B. Ignition Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    # C. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'panda_robot', 
            '-z', '0.0', 
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # D. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # 2. Camera Info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            
            # 3. DYNAMIC CAMERA (Hand)
            '/world/obstacle_world/model/panda_robot/link/panda_hand/sensor/camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            
            # 4. STATIC CAMERA (Box)
            '/static_camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/world/obstacle_world/model/panda_robot/link/panda_hand/sensor/camera/depth_image/points', '/camera/combined_points'),
            ('/static_camera/depth_image/points', '/camera/combined_points'),
            ('esdf_pointcloud', '/esdf_pointcloud'),
        ],
        output='screen'
    )

    # E. TF Dynamic (Hand Camera)
    static_tf_dynamic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_dynamic',
        arguments=[
            "--x", "0.04", "--y", "0", "--z", "0.04",
            "--qx", "0", "--qy", "-0.707", "--qz", "0", "--qw", "0.707",
            "--frame-id", "panda_hand", 
            "--child-frame-id", "panda_robot/panda_hand/camera"
        ]
    )

    # F. TF Static (World Camera)
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

    # G. Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],        
        output='screen'
    )

    arm_controller_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],    
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
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])