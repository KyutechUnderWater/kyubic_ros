import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory(
            "test_pid"), "config", "test_pid.param.yaml"
    )

    action_component_container = ComposableNodeContainer(
        name="test_pid_component_container",
        namespace="test",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="test_pid_component",
                namespace="test_pid",
                package="test_pid",
                plugin="test::TestPID",
                # remappings=[("/localization/depth", "/driver/depth")],
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
        output="screen",
    )

    return LaunchDescription([action_component_container])
