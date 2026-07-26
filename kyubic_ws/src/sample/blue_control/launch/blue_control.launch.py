"""Launch the receive-only BlueROV topic listener sample node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the blue_control listener launch description."""
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            Node(
                package="blue_control",
                executable="blue_control_listener",
                name="blue_control_listener",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
