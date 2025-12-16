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
    #    cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
    #   output='screen'
    
    cmd=['ign', 'gazebo', '-r', '/home/josa/thesis/obstacles.sdf'],
    output='screen'
)

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'panda_robot', '-z', '0.0', '-allow_renaming', 'true'],
        output='screen'
    )

    # BRIDGE CONFIGURATION
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. Clock (Critical for RViz Sync)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # 2. Joint States
            #'/world/empty/model/panda_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # 3. Point Cloud (With Explicit Types)
            #'/world/empty/model/panda_robot/link/panda_link7/sensor/camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/world/obstacle_world/model/panda_robot/link/panda_link7/sensor/camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # 4. Camera Info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        remappings=[
            #('/world/empty/model/panda_robot/joint_state', '/joint_states'),
            #('/world/empty/model/panda_robot/link/panda_link7/sensor/camera/depth_image/points', '/camera/depth/points'),
            ('/world/obstacle_world/model/panda_robot/link/panda_link7/sensor/camera/depth_image/points', '/camera/depth/points'),
        ],
        output='screen'
    )

    # STATIC TRANSFORM (Connects Gazebo camera frame to Robot URDF link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'panda_hand', 'panda_robot/panda_link7/camera']
    )

    # 1. Create the spawner node for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )


    # 2. (Optional) Create the spawner for your arm controller (e.g., joint_trajectory_controller)
    arm_controller_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
         output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_gz_plugin_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        static_tf,
        joint_state_broadcaster_spawner, # <--- ADD THIS
        arm_controller_spawner
    ])