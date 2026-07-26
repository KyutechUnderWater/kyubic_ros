"""Launch the drivers used by Kyubic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the Kyubic driver launch description."""
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            # IncludeLaunchDescription(
            #     PathJoinSubstitution(
            #         [
            #             FindPackageShare("gnss_driver"),
            #             "launch",
            #             "gnss_driver.launch.py",
            #         ]
            #     ),
            #     launch_arguments={"log_level": log_level}.items(),
            # ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("logic_distro_rp2040_driver"),
                        "launch",
                        "logic_distro_rp2040_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("sensors_esp32_driver"),
                        "launch",
                        "sensors_esp32_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("imu_driver"),
                        "launch",
                        "imu_driver.launch.py",
                    ]
                ),
                launch_arguments={"log_level": log_level}.items(),
            ),
        ]
    )
