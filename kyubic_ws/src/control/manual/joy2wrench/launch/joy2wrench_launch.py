import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("joy2wrench"), "config", "joy2wrench.param.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                namespace="driver",
                executable="joy_node",
                output="screen",
            ),
            Node(
                package="joy2wrench",
                namespace="driver",
                executable="joy2wrench",
                parameters=[config],
                output="screen",
            ),
        ]
    )
