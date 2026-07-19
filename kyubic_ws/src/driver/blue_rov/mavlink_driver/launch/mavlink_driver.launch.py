"""Launch the BlueROV MAVLink driver."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    """Create the MAVLink driver launch description.

    Returns:
        LaunchDescription: The configured BlueROV driver node.
    """
    config = PathJoinSubstitution(
        [FindPackageShare("mavlink_driver"), "config", "mavlink_driver.param.yaml"]
    )
    log_level = DeclareLaunchArgument("log_level", default_value="info")

    return LaunchDescription(
        [
            log_level,
            Node(
                package="mavlink_driver",
                executable="mavlink_driver",
                name="mavlink_driver",
                namespace="driver/blue_rov/mavlink_driver",
                parameters=[config],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
