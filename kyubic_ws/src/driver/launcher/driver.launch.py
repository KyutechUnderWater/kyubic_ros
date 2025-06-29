from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth_driver_dir = PathJoinSubstitution(
        [FindPackageShare("depth_driver"), "launch"]
    )
    imu_driver_dir = PathJoinSubstitution([FindPackageShare("imu_driver"), "launch"])
    dvl_driver_dir = PathJoinSubstitution([FindPackageShare("dvl_driver"), "launch"])
    led_driver_dir = PathJoinSubstitution([FindPackageShare("led_driver"), "launch"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([depth_driver_dir, "depth_driver_launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([imu_driver_dir, "imu_driver_launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([dvl_driver_dir, "dvl_driver_launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([led_driver_dir, "led_driver_launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
        ]
    )
