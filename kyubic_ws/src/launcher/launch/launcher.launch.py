from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_launcher_dir = PathJoinSubstitution(
        [FindPackageShare("driver_launcher"), "launch"]
    )
    localization_dir = PathJoinSubstitution(
        [FindPackageShare("localization"), "launch"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [driver_launcher_dir, "driver_launcher.launch.py"]
                ),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([localization_dir, "localization.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
        ]
    )
