import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    world_path_arg = DeclareLaunchArgument(
        'world_path',
        default_value='/home/dummy/Projects/Yolanda/thesis/src/thesis_sim/worlds/clean_world.sdf',
        description='Absolute path to the SDF world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = LaunchConfiguration('world_path')

    pkg_panda = get_package_share_directory('panda_description')
    xacro_file = os.path.join(pkg_panda, 'urdf', 'panda.urdf.xacro')
    gz_resource_path = os.path.dirname(pkg_panda)

    set_gz_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )
    set_gz_plugin_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

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

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'panda_robot',
            '-z', '0.0',
            '-allow_renaming', 'true'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge ONLY static camera point cloud to ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/static_camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[('/static_camera/depth_image/points', '/camera/points')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # World -> camera TF (static camera)
    static_tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_static',
        arguments=[
            '--x', '2.0', '--y', '0', '--z', '1.3',
            '--roll', '0', '--pitch', '0.65', '--yaw', '3.14159',
            '--frame-id', 'world',
            '--child-frame-id', 'static_camera/link/static_depth_camera'
        ]
    )

    # These restore /joint_states so RViz RobotModel works again.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    # Give Gazebo + ros2_control a moment to come up before spawning controllers
    delayed_spawners = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner]
    )

    return LaunchDescription([
        world_path_arg,
        set_gz_resource_path,
        set_gz_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_spawners,
        bridge,
        static_tf_static,
    ])

