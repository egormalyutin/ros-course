from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def process_xacro(path):
    doc = xacro.parse(open(path, "r"))
    xacro.process_doc(doc)
    return doc.toxml()


def generate_launch_description():
    urdf_path = join(
        get_package_share_directory("minigun_resources"),
        "models/model.urdf.xacro",
    )
    urdf = process_xacro(urdf_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "watch",
                default_value="false",
                description="Watch",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": urdf,
                    }
                ],
            ),
            Node(
                package="watch_urdf",
                executable="watch_urdf",
                parameters=[{"path": urdf_path}],
                condition=IfCondition(LaunchConfiguration("watch")),
            ),
        ]
    )
