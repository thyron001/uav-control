from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tello_control', executable='drone_connector'),
        Node(package='tello_control', executable='telemetry_monitor'),
        Node(package='tello_control', executable='object_detector'),
        Node(package='tello_control', executable='mission_planner'),
    ])
