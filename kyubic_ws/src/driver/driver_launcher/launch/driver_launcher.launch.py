from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    actuator_rp2040_driver_dir = PathJoinSubstitution(
        [FindPackageShare("actuator_rp2040_driver"), "launch"]
    )
    dvl_driver_dir = PathJoinSubstitution([FindPackageShare("dvl_driver"), "launch"])
    gnss_driver_dir = PathJoinSubstitution([FindPackageShare("gnss_driver"), "launch"])
    logic_distro_rp2040_driver_dir = PathJoinSubstitution(
        [FindPackageShare("logic_distro_rp2040_driver"), "launch"]
    )
    sensors_esp32_driver_dir = PathJoinSubstitution(
        [FindPackageShare("sensors_esp32_driver"), "launch"]
    )

    return LaunchDescription(
        [
            log_level_arg,
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [actuator_rp2040_driver_dir, "actuator_rp2040_driver.launch.py"]
                ),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            # IncludeLaunchDescription(
            #     PathJoinSubstitution([dvl_driver_dir, "dvl_driver.launch.py"]),
            #     launch_arguments={
            #         "log_level": LaunchConfiguration("log_level"),
            #     }.items(),
            # ),
            # IncludeLaunchDescription(
            #     PathJoinSubstitution([gnss_driver_dir, "gnss_driver.launch.py"]),
            #     launch_arguments={
            #         "log_level": LaunchConfiguration("log_level"),
            #     }.items(),
            # ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        logic_distro_rp2040_driver_dir,
                        "logic_distro_rp2040_driver.launch.py",
                    ]
                ),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([sensors_esp32_driver_dir, "sensors_esp32_driver.launch.py"]),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
        ]
    )
