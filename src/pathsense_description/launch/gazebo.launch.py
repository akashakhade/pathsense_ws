"""Spawn the PathSense robot in Gazebo Harmonic, bridge topics to ROS 2 and open RViz.

    ros2 launch pathsense_description gazebo.launch.py
    ros2 launch pathsense_description gazebo.launch.py rviz:=false world:=/path/to/world.sdf

Then drive it from another terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
and move the climbing arms from a third terminal:
    ros2 run pathsense_description arm_teleop_keyboard.py
"""
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pathsense_description')
    xacro_file = os.path.join(pkg, 'urdf', 'pathsense.urdf.xacro')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    use_tof = LaunchConfiguration('use_tof')

    color_split = os.path.join(get_package_prefix('pathsense_description'),
                               'lib', 'pathsense_description', 'urdf_color_split.py')
    robot_description = ParameterValue(
        Command([color_split, ' ', xacro_file, ' use_gazebo:=true use_tof:=', use_tof]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -v 3 ', world], 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'pathsense', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.03', '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml'),
                     'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg, 'rviz', 'pathsense.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'stair_test.sdf')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_tof', default_value='true',
                              description='Simulate the 6 ToF edge sensors (set false on slow laptops)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # lets Gazebo find this package if meshes are added later
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.dirname(pkg) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
    ])
