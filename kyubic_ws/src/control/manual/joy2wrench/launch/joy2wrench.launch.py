import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory(
            "joy2wrench"), "config", "joy2wrench.param.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                namespace="joy2wrench",
                executable="joy_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="joy2wrench",
                namespace="joy2wrench",
                executable="joy2wrench",
                remappings=[
                    ("/joy2wrench/robot_force", "/driver/robot_force"),
                ],
                parameters=[config],
                output="screen",
            ),
        ]
    )
