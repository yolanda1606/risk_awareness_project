import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    
    # 1. DEFINE PATHS
    panda_desc_pkg = get_package_share_directory('panda_description')
    panda_moveit_pkg = get_package_share_directory('panda_moveit_config')
    
    xacro_file = os.path.join(panda_desc_pkg, 'urdf', 'panda.urdf.xacro')
    srdf_file = os.path.join(panda_moveit_pkg, 'srdf', 'panda.srdf')

    # 2. LOAD DESCRIPTIONS
    # Robot Description (URDF)
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
    )
    robot_description = {"robot_description": robot_description_content}

    # Semantic Robot Description (SRDF)
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": semantic_content}

    # 3. HARDCODED KINEMATICS (The Fix)
    # We define this manually to ensure 'arm' is used and 0.05 is a Number.
    kinematics_config = {
        "arm": {   # <--- CORRECT GROUP NAME (Matches your screenshot)
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05,  # <--- NO QUOTES (This fixes the crash)
            "kinematics_solver_attempts": 3
        }
    }

    # 4. MOVE GROUP NODE
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {"use_sim_time": True},
            # Allow trajectory execution
            {"moveit_manage_controllers": True},
            {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
            {"trajectory_execution.allowed_goal_duration_margin": 0.5},
            {"trajectory_execution.allowed_start_tolerance": 0.01},
        ],
    )

    # 5. RVIZ NODE
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])