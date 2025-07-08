import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("thruster_driver"),
        "config",
        "thruster_driver.param.yaml",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    return LaunchDescription(
        [
            log_level_arg,
            Node(
                package="thruster_driver",
                namespace="driver",
                executable="thruster_driver",
                parameters=[config],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
