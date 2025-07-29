from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth_driver_dir = PathJoinSubstitution(
        [FindPackageShare("depth_driver"), "launch"]
    )
    dvl_driver_dir = PathJoinSubstitution([FindPackageShare("dvl_driver"), "launch"])
    led_driver_dir = PathJoinSubstitution([FindPackageShare("led_driver"), "launch"])
    thruster_driver_dir = PathJoinSubstitution(
        [FindPackageShare("thruster_driver"), "launch"]
    )
    gnss_driver_dir = PathJoinSubstitution([FindPackageShare("gnss_driver"), "launch"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([depth_driver_dir, "depth_driver.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([dvl_driver_dir, "dvl_driver.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([led_driver_dir, "led_driver.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([gnss_driver_dir, "gnss_driver.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            # TODO: ソフトウェア非常停止が搭載されれば，開放
            # IncludeLaunchDescription(
            #     PathJoinSubstitution(
            #         [thruster_driver_dir, "thruster_driver.launch.py"]
            #     ),
            #     launch_arguments={
            #         "log_level": "warn",
            #     }.items(),
            # ),
        ]
    )
