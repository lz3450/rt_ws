from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    csv_logger = Node(
        package="utils",
        executable="csv_logger",
        name="CSVLoggerNode",
    )

    simple_joint_command_publisher = Node(
        package="myisaacsim",
        executable="simple_joint_command_publisher",
        name="SimpleJointCommandPublisher",
    )

    delayed_joint_command_publisher = TimerAction(
        period=3.0,
        actions=[simple_joint_command_publisher],
    )

    shutdown = TimerAction(
        period=15.0,
        actions=[Shutdown()],
    )

    return LaunchDescription(
        [
            csv_logger,
            delayed_joint_command_publisher,
            shutdown,
        ]
    )
