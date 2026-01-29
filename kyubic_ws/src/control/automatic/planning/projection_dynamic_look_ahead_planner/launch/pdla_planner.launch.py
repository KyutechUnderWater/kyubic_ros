import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("projection_dynamic_look_ahead_planner"),
            "config",
            "pdla_planner.param.yaml",
        ]
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    composable_nodes = [
        ComposableNode(
            package="projection_dynamic_look_ahead_planner",
            plugin="planner::pdla_planner::PDLAPlanner",
            namespace="planner/pdla_planner",
            name="pdla_planner_component",
            remappings=[
                ("odom", "/localization/odom"),
                ("goal_current_odom", "/planner/wrench_planner/goal_current_odom"),
            ],
            parameters=[config],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="projection_dynamic_look_ahead_planner",
            plugin="planner::pdla_planner::PDLAFeedbackRepub",
            namespace="planner/pdla_planner",
            name="pdla_feedback_repub_component",
            remappings=[
                ("feedback", "/planner/pdla_planner/pdla_plan/_action/feedback"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]

    container = ComposableNodeContainer(
        name="pdla_planner_container",
        namespace="pdla_planner",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )

    return LaunchDescription([log_level_arg, container])
