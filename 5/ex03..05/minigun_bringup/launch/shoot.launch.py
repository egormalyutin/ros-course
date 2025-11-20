from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("minigun_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="false", description="Open RViz."
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_bringup, "launch", "gz.launch.py"),
                ),
                launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
            ),
            Node(
                package="minigun_controller",
                executable="shoot",
                output="screen",
            ),
        ]
    )
