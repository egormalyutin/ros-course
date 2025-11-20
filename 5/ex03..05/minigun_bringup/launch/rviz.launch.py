from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("minigun_bringup")
    pkg_resources = get_package_share_directory("minigun_resources")

    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_resources, "launch", "publisher.launch.py")
        ),
        launch_arguments={"watch": "true"}.items(),
    )

    return LaunchDescription(
        [
            publisher,
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    join(pkg_bringup, "config", "diff_drive.rviz"),
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
        ]
    )
