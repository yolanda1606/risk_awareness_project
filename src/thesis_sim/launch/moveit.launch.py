import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

# Helper function to load OMPL config
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

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

    # 3. KINEMATICS
    kinematics_config = {
        "arm": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05,
            "kinematics_solver_attempts": 3
        }
    }

    # 4. OMPL CONFIGURATION
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("panda_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 5. CONTROLLERS CONFIGURATION (THE FIX)
    # We manually tell MoveIt where to send the commands.
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["joint_trajectory_controller"],
            "joint_trajectory_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": [
                    "panda_joint1", "panda_joint2", "panda_joint3", 
                    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
                ]
            }
        }
    }

    # 6. MOVE GROUP NODE
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            moveit_controllers, # <--- Passing the controller config here
            {"use_sim_time": True},
            {"moveit_manage_controllers": True},
        ],
    )

    # 7. RVIZ NODE
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])