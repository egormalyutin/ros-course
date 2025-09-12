from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                namespace="turtlesim1",
            ),
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                namespace="turtlesim2",
                remappings=[
                    ("turtle1/cmd_vel", "/turtlesim1/turtle1/cmd_vel"),
                ],
            ),
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                namespace="turtlesim3",
                remappings=[
                    ("turtle1/cmd_vel", "/turtlesim1/turtle1/cmd_vel"),
                ],
            ),
        ]
    )
