from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("dvl_driver"),
            "config",
            "dvl_setting.param.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                namespace="driver/dvl_driver",
                package="dvl_driver",
                executable="dvl_setting",
                name="dvl_setting",
                output="screen",
                parameters=[config],
            )
        ]
    )
