"""Drive the robot in RViz only (no Gazebo): a light kinematic simulator moves the model.

    ros2 launch pathsense_description rviz_teleop.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard        # drive
    ros2 run pathsense_description arm_teleop_keyboard.py       # arms
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
        Node(package='pathsense_description', executable='rviz_teleop_sim.py', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'pathsense.rviz')]),
    ])
