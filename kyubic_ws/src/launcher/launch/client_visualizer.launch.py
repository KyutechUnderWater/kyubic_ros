from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_time_plotter_dir = PathJoinSubstitution(
        [FindPackageShare("real_time_plotter"), "launch"]
    )
    trajectory_viewer_dir = PathJoinSubstitution(
        [FindPackageShare("trajectory_viewer"), "launch"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [real_time_plotter_dir, "real_time_plotter.launch.py"]
                ),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [trajectory_viewer_dir, "trajectory_viewer.launch.py"]
                ),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
        ]
    )
