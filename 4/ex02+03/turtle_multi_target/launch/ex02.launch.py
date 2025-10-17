from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                executable="turtlesim_node",
            ),
            Node(
                package="turtle_multi_target",
                executable="carrot",
                parameters=[
                    dict(
                        frame="turtle1",
                        carrot="carrot",
                        speed=0.2,
                        radius=1.0,
                    )
                ],
            ),
            Node(
                package="turtle_multi_target",
                executable="spawn",
                parameters=[
                    dict(
                        name="turtle2",
                        x=2.0,
                        y=2.0,
                        theta=0.0,
                    ),
                ],
            ),
            Node(
                package="turtle_multi_target",
                executable="turtle_tf",
                remappings=[("pose", "/turtle1/pose")],
                parameters=[{"frame": "turtle1"}],
            ),
            Node(
                package="turtle_multi_target",
                executable="turtle_tf",
                remappings=[("pose", "/turtle2/pose")],
                parameters=[{"frame": "turtle2"}],
            ),
            Node(
                package="turtle_multi_target",
                executable="controller",
                parameters=[
                    dict(
                        turtle_frame="turtle2",
                        target_frames=["carrot"],
                        switch_threshold=0.05,
                    )
                ],
                remappings=[("cmd_vel", "/turtle2/cmd_vel")],
            ),
        ]
    )
