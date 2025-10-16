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
            Node(
                package="turtle_multi_target",
                executable="spawn",
                parameters=[
                    dict(
                        name="turtle3",
                        x=8.0,
                        y=8.0,
                        theta=0.0,
                    )
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["8", "2", "0", "0", "0", "0", "world", "static_target"],
            ),
            *[
                Node(
                    package="turtle_multi_target",
                    executable="turtle_tf",
                    remappings=[("pose", f"/turtle{i}/pose")],
                    parameters=[{"frame": f"turtle{i}"}],
                )
                for i in [1, 2, 3]
            ],
            *[
                Node(
                    package="turtle_multi_target",
                    executable="carrot",
                    parameters=[
                        dict(
                            frame=f"turtle{i}",
                            carrot=f"carrot{i}",
                            speed=0.2,
                            radius=1.0,
                        )
                    ],
                )
                for i in [1, 3]
            ],
            Node(
                package="turtle_multi_target",
                executable="controller",
                parameters=[
                    dict(
                        turtle_frame="turtle2",
                        target_frames=["carrot1", "static_target", "carrot3"],
                    )
                ],
                remappings=[("cmd_vel", "/turtle2/cmd_vel")],
            ),
        ]
    )
