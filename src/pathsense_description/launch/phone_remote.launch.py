"""Phone remote control: web page (port 8000) + rosbridge (9090) + camera stream (8080).

Start after Gazebo and slam_nav.launch.py:
    ros2 launch pathsense_description phone_remote.launch.py
Then open the printed address (http://<laptop-ip>:8000) in the phone's browser.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket',
             output='screen', parameters=[{'port': 9090, 'address': '0.0.0.0'}]),
        Node(package='web_video_server', executable='web_video_server', name='web_video_server',
             output='screen', parameters=[{'port': 8080, 'address': '0.0.0.0'}]),
        Node(package='pathsense_description', executable='phone_server.py', name='phone_server',
             output='screen'),
    ])
