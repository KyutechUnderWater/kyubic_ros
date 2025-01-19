import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("dvl_driver"), "config", "dvl_driver.param.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="dvl_driver",
                namespace="driver",
                executable="dvl_driver",
                parameters=[config],
                output="screen",
            )
        ]
    )
