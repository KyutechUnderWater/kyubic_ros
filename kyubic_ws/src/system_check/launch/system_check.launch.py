from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("system_check"), "config", "system_check.param.yaml"]
    )

    checker_component = ComposableNode(
        package="system_check",
        plugin="system_check::SystemCheck",
        name="checker_node",
        parameters=[config],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="system_check_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[checker_component],
        output="screen",
    )

    return LaunchDescription([container])
