from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unilidar2_ros2',
            executable='unilidar2_node',
            parameters=[]
        ),
    ])
