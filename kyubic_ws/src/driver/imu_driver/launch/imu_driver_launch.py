import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("imu_driver"), "config", "imu_driver.param.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="imu_driver",
                namespace="driver",
                executable="imu_driver",
                parameters=[config],
                output="screen",
            )
        ]
    )
