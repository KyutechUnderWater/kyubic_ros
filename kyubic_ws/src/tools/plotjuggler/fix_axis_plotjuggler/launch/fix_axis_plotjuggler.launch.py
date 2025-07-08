from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

import os
import yaml


def check_num(config: dict) -> int:
    assert len(config["topic_name"]) == len(config["min"]) == len(config["max"]), (
        "Not match num of parameters (topic_name, min, max)"
    )

    return len(config["topic_name"])


def generate_launch_description():
    # Get config path
    config_path = os.path.join(
        get_package_share_directory("fix_axis_plotjuggler"),
        "config",
        "fix_axis_plotjuggler.param.yaml",
    )

    # Load config
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)["/**"]["ros__parameters"]

    # Create ComposableNodes
    num: int = check_num(config)
    nodes: list[ComposableNode] = []
    for i in range(num):
        nodes.append(
            ComposableNode(
                name=f"fix_axis_plotjuggler_{i}",
                namespace="tools/plotjuggler",
                package="fix_axis_plotjuggler",
                plugin="tools::plotjuggler::FixAxisPlotjuggler",
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
                parameters=[
                    {
                        "topic_name": config["topic_name"][i],
                        "min": float(config["min"][i]),
                        "max": float(config["max"][i]),
                    }
                ],
            )
        )

    action_component_container = ComposableNodeContainer(
        name="fix_axis_plotjuggler_component_container",
        namespace="tools/plotjuggler",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return LaunchDescription([action_component_container])
