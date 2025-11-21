from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("sensor_board_driver"),
            "config",
            "sensor_board_driver.param.yaml",
        ]
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    executable_list = [
        "button_battery_state",
        "buzzer_switch",
        "depth",
        "environment",
        "imu_reset",
        "leak",
        "rgb",
        "rtc_gnss",
        "rtc_time",
        "tilt_servo",
        "zed_power",
    ]

    nodes_to_launch = []
    for exe in executable_list:
        node = Node(
            package="sensor_board_driver",
            namespace="sensor_board_driver",
            executable=exe,
            name=exe,
            parameters=[config],
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
        nodes_to_launch.append(node)

    return LaunchDescription(
        [
            log_level_arg,
            *nodes_to_launch,
        ]
    )
