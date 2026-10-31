"""Live SLAM + Nav2 (no pre-made map) for topological route following.

Start Gazebo first (without its RViz):
    ros2 launch pathsense_description gazebo.launch.py rviz:=false use_tof:=false world:=<...>/college_first_floor.sdf
Then:
    ros2 launch pathsense_description slam_nav.launch.py
Then the route follower:
    ros2 run pathsense_description graph_navigator.py
Save the map built by SLAM at any time:
    ros2 run nav2_map_server map_saver_cli -f ~/college_slam_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pathsense_description')
    params = LaunchConfiguration('params_file')
    sim = {'use_sim_time': True}
    nav_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

    # Behaviour tree without the Spin recovery: when blocked -> clear costmaps, back up 0.3 m, replan
    bt_xml = os.path.join(pkg, 'config', 'through_poses_no_spin.xml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': LaunchConfiguration('slam_params_file'),
                          'use_sim_time': 'true'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_slam_params.yaml')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(pkg, 'config', 'slam_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),

        # SLAM Toolbox: builds /map live and publishes map -> odom
        slam,

        # Nav2 (same planner / controller as the room tour, but no map_server / AMCL)
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params, sim],  remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params, sim]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params, sim],  remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, sim, {'default_nav_through_poses_bt_xml': bt_xml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': nav_nodes}]),

        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[os.path.join(pkg, 'config', 'twist_mux.yaml')],
             remappings=[('cmd_vel_out', '/cmd_vel')]),

        Node(package='rviz2', executable='rviz2', output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(pkg, 'rviz', 'slam_graph.rviz')], parameters=[sim]),
    ])