from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("sensors_esp32_driver"),
            "config",
            "sensors_esp32_driver.param.yaml",
        ]
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    executable_list = [
        "button_battery_state_component_node",
        "buzzer_switch_component_node",
        "depth_component_node",
        "environment_component_node",
        "imu_reset_component_node",
        "leak_component_node",
        "rgb_component_node",
        "rtc_gnss_component_node",
        "rtc_time_component_node",
        "system_status_component_node",
        "tilt_servo_component_node",
        "zed_power_component_node",
    ]

    nodes = []
    for exe in executable_list:
        node_remappings = []
        if exe == "depth_component_node":
            node_remappings.append(("depth", "/driver/depth"))

        # Nodeの定義
        node = Node(
            package="sensors_esp32_driver",
            executable=exe,
            namespace="sensors_esp32_driver",
            name=exe,
            parameters=[config],
            remappings=node_remappings,
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            output="screen",
        )
        nodes.append(node)

    return LaunchDescription(
        [
            log_level_arg,
        ]
        + nodes
    )
