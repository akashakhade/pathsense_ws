"""Show the robot model in RViz with sliders for every moving joint (no physics).

    ros2 launch pathsense_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pathsense_description')
    xacro_file = os.path.join(pkg, 'urdf', 'pathsense.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' use_gazebo:=false']), value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen',
             parameters=[{'zeros': {'arm_left_joint': 0.40, 'arm_right_joint': 0.40}}]),
        # RViz config uses 'odom' as fixed frame, so pin the robot at the origin
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint']),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'pathsense.rviz')]),
    ])
