from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("actuator_rp2040_driver"),
            "config",
            "actuator_rp2040_driver.param.yaml",
        ]
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    driver_component = ComposableNode(
        package="actuator_rp2040_driver",
        plugin="driver::actuator_rp2040_driver::ActuatorRP2040",
        namespace="driver/actuator_rp2040_driver",
        name="actuator_rp2040_driver",
        parameters=[config],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="logic_distro_container",
        namespace="driver/actuator_rp2040_driver",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[driver_component],
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            log_level_arg,
            container,
        ]
    )
