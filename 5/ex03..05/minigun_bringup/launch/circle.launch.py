from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("minigun_bringup")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_bringup, "launch", "gz.launch.py"),
                ),
                launch_arguments={"rviz": "false"}.items(),
            ),
            Node(
                package="minigun_controller",
                executable="circle",
                output="screen",
            ),
        ]
    )
