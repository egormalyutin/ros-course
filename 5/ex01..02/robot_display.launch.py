import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_URDF = """
<?xml version="1.0"?>
<robot name="robot">
  <link name="base_link" />
</robot>
"""


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    dir = os.path.dirname(__file__)
    urdf = os.path.join(dir, "model.urdf.xacro")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_description": DEFAULT_URDF}
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="watch_urdf",
                executable="watch_urdf",
                parameters=[{"path": urdf}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(dir, "config.rviz"),
                ],
            ),
        ]
    )
