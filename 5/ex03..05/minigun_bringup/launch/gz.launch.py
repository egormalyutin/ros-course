from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("minigun_bringup")
    pkg_resources = get_package_share_directory("minigun_resources")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world = join(pkg_resources, "worlds", "playground.sdf")
    print(world)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        # launch_arguments={"gz_args": f"-r gpu_lidar_sensor.sdf"}.items(),
        # launch_arguments={"gz_args": f"-r empty.sdf"}.items(),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
        # launch_arguments={"gz_args": f"-r empty.sdf"}.items(),
    )

    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_resources, "launch", "publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            join(pkg_bringup, "config", "diff_drive.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": join(pkg_bringup, "config", "ros_gz_bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "1.0",
            "-z",
            "0.5",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim,
            bridge,
            publisher,
            rviz,
            spawn,
        ]
    )
