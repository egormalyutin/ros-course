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
                executable="spawn",
                parameters=[
                    dict(
                        name="turtle2",
                        x=2.0,
                        y=2.0,
                        theta=0.0,
                    )
                ],
            ),
            *[
                Node(
                    package="turtle_multi_target",
                    executable="turtle_tf",
                    remappings=[("pose", f"/turtle{i}/pose")],
                    parameters=[{"frame": f"turtle{i}"}],
                )
                for i in [1, 2]
            ],
            Node(
                package="turtle_multi_target",
                executable="controller",
                parameters=[
                    dict(
                        turtle_frame="turtle2",
                        delay=3.0,
                        target_frames=["turtle1"],
                    )
                ],
                remappings=[("cmd_vel", "/turtle2/cmd_vel")],
            ),
        ]
    )
