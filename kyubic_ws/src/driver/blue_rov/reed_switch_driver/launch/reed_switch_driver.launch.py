"""Launch the reed switch mission-start trigger driver."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Create the reed switch driver launch description.

    Returns:
        LaunchDescription: The configured reed switch driver node.
    """
    config = PathJoinSubstitution(
        [FindPackageShare("reed_switch_driver"), "config", "reed_switch_driver.param.yaml"]
    )
    log_level = DeclareLaunchArgument("log_level", default_value="info")

    return LaunchDescription(
        [
            log_level,
            Node(
                package="reed_switch_driver",
                executable="reed_switch_driver",
                name="reed_switch_driver",
                namespace="driver/blue_rov/reed_switch_driver",
                parameters=[config],
                remappings=[
                    ("mission_start_trigger", "/driver/blue_rov/mission_start_trigger"),
                    ("led", "/driver/blue_rov/mavlink_driver/led"),
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
