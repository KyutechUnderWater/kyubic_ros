import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("educational_pid"), "config", "pid_gains.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="educational_pid",
                executable="pid_node",
                name="educational_pid_node",
                parameters=[config],
            )
        ]
    )
