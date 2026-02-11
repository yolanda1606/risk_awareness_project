import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    
    # 1. SETUP: Define paths
    pkg_panda = get_package_share_directory('panda_description')
    pkg_thesis_sim = get_package_share_directory('thesis_sim')
    gz_resource_path = os.path.dirname(pkg_panda)
    
    # The new world and the new wrapper xacros
    world_path = os.path.join(pkg_thesis_sim, 'worlds', 'high_low_scenario.sdf')
    white_xacro_file = os.path.join(pkg_thesis_sim, 'urdf', 'white_panda.urdf.xacro')
    red_xacro_file = os.path.join(pkg_thesis_sim, 'urdf', 'red_panda.urdf.xacro')

    print(f"[INFO] Loading World: {world_path}")

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

    # 3. XACRO COMMANDS (Passing the arm_id parameter dynamically)
    # This turns panda_link0 into r1_link0 and r2_link0
    r1_desc_cmd = Command(['xacro ', white_xacro_file, ' arm_id:=r1'])
    r2_desc_cmd = Command(['xacro ', red_xacro_file, ' arm_id:=r2'])


    # 4. NODES
    
    # A. Ignition Gazebo (Using the official ROS wrapper)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r {world_path}"}.items(),
    )

    # B. Robot State Publishers (Namespaced)
    rsp_r1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='r1',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': r1_desc_cmd}]
    )

    rsp_r2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='r2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': r2_desc_cmd}]
    )

    # C. Spawn Entities in Gazebo
    spawn_r1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', '/r1/robot_description', 
            '-name', 'R1', 
            '-x', '2.6', '-y', '0.5', '-z', '0.0', '-Y', '3.14',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    spawn_r2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', '/r2/robot_description', 
            '-name', 'R2', 
            '-x', '1.8', '-y', '-0.5', '-z', '0.7', '-Y', '1.57',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # D. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            
            # Joint States for both robots
            '/world/semantic_world/model/R1/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/world/semantic_world/model/R2/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/semantic_world/model/R1/joint_state', '/r1/joint_states'),
            ('/world/semantic_world/model/R2/joint_state', '/r2/joint_states'),
        ],
        output='screen'
    )

    # E. Static Transforms for Robot Bases (Using old panda convention: arm_id_link0)
    static_tf_r1 = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_r1',
        arguments=['2.6', '0.5', '0.0', '3.14', '0.0', '0.0', 'world', 'r1_link0']
    )

    static_tf_r2 = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_r2',
        arguments=['1.8', '-0.5', '0.7', '1.57', '0.0', '0.0', 'world', 'r2_link0']
    )

    # F. Static Transforms for the Camera (From your high_low_scenario SDF pose)
    static_tf_cam = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_cam',
        arguments=['1.2', '1.5', '0.8', '-0.99', '0.3', '0.0', 'world', 'camera_link']
    )

    static_tf_optical = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_optical',
        arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link', 'camera_optical_frame']
    )

    # G. Semantic Node
    semantic_processor = Node(
        package='thesis_sim',            
        executable='semantic_node',      
        name='semantic_cloud_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_gz_plugin_path,
        gazebo,
        rsp_r1,
        rsp_r2,
        spawn_r1,
        spawn_r2,
        bridge,
        static_tf_r1,
        static_tf_r2,
        static_tf_cam,
        static_tf_optical,
        semantic_processor
    ])