"""Launch the drivers used by BlueROV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the BlueROV driver launch description."""
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("mavlink_driver"),
                        "launch",
                        "mavlink_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("dvl75_driver"),
                        "launch",
                        "dvl75_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("camera_driver"),
                        "launch",
                        "camera_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("reed_switch_driver"),
                        "launch",
                        "reed_switch_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
        ]
    )
