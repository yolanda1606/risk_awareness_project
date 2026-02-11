import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define a simple red box SDF model string
    # We spawn it at (0.5, 0.0, 0.5) - Right in front of the robot
    box_sdf = """
    <?xml version='1.0'?>
    <sdf version='1.6'>
        <model name='dynamic_box'>
            <pose>0.5 0.0 0.5 0 0 0</pose>
            <link name='link'>
                <inertial>
                    <mass>1.0</mass>
                    <inertia> <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.01</iyy><iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
                    <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
                </visual>
                <collision name='collision'>
                    <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
                </collision>
            </link>
        </model>
    </sdf>
    """

    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', box_sdf, '-name', 'dynamic_box', '-allow_renaming', 'false'],
            output='screen'
        )
    ])