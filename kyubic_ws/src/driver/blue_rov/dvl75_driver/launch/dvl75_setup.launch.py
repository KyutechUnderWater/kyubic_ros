"""Apply the persistent DVL-75 settings during initial provisioning."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Start the C++ driver once with persistent device configuration enabled."""
    config = PathJoinSubstitution(
        [FindPackageShare("dvl75_driver"), "config", "dvl75_driver.param.yaml"]
    )
    log_level = DeclareLaunchArgument("log_level", default_value="info")

    return LaunchDescription(
        [
            log_level,
            ComposableNodeContainer(
                name="dvl75_setup_container",
                namespace="driver/blue_rov/dvl75_driver",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    ComposableNode(
                        package="dvl75_driver",
                        plugin="dvl75_driver::Dvl75Driver",
                        name="dvl75_setup",
                        namespace="driver/blue_rov/dvl75_driver",
                        parameters=[config, {"apply_device_config": True}],
                        remappings=[("dvl", "/driver/dvl")],
                    ),
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
