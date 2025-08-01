from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myturtlesim',
            namespace='myturtlesim',
            executable='turtlesim_node',
            name='sim'
        )
    ])
