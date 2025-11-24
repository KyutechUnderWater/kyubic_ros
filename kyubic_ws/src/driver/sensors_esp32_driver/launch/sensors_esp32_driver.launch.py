from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def snake_to_pascal(text):
    """
    snake_case -> PascalCase
    (ex: button_battery_state_component -> ButtonBatteryState)
    """
    if text == "rgb_component":
        return "RGB"

    if text.endswith("_component"):
        text = text.replace("_component", "")
    return "".join(x.title() for x in text.split("_"))


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
        default_value=["info"],
        description="Logging level",
    )

    executable_list = [
        "button_battery_state_component",
        "buzzer_switch_component",
        "depth_component",
        "environment_component",
        "imu_reset_component",
        "leak_component",
        "rgb_component",
        "rtc_gnss_component",
        "rtc_time_component",
        "tilt_servo_component",
        "zed_power_component",
    ]

    composable_nodes = []
    for exe in executable_list:
        node_remappings = []
        if exe == "depth_component":
            node_remappings.append(("depth", "/driver/depth"))

        composable_nodes.append(
            ComposableNode(
                package="sensors_esp32_driver",
                plugin=f"sensors_esp32_driver::{snake_to_pascal(exe)}",
                namespace="sensors_esp32_driver",
                name=exe,
                parameters=[config],
                remappings=node_remappings,
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    container = ComposableNodeContainer(
        name="sensors_esp32_driver_container",
        namespace="sensors_esp32_driver",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )

    return LaunchDescription(
        [
            log_level_arg,
            container,
        ]
    )
