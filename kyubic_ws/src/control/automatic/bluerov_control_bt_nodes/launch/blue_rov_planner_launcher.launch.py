"""BlueROV 向け PDLA プランナのみ起動する launch。

Kyubic 本家の planner_launcher.launch.py は ZOH/WrenchPlanner を
actuator_rp2040 向けに含むため、BlueROV では bluerov_bt.launch.py 内の
bluerov_wrench_planner_container (mavlink 向け) と重複する。
本 launch は PDLA + PDLAFeedbackRepub だけを起動し、出力は
/planner/wrench_planner/zoh_wrench_plan へ接続する (Kyubic planner_launcher と同じ)。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cfg_pdla_planner = PathJoinSubstitution(
        [
            FindPackageShare("projection_dynamic_look_ahead_planner"),
            "config",
            "pdla_planner.param.yaml",
        ]
    )
    path_planner_path = FindPackageShare("path_planner")
    log_level = LaunchConfiguration("log_level")

    pdla_container = ComposableNodeContainer(
        name="planner_launcher_component_container",
        namespace="planner",
        package="rclcpp_components",
        executable="component_container_isolated",
        output="screen",
        ros_arguments=["--log-level", log_level],
        arguments=["--use_multi_threaded_executor"],
        composable_node_descriptions=[
            ComposableNode(
                name="pdla_planner_component",
                namespace="planner/pdla_planner",
                package="projection_dynamic_look_ahead_planner",
                plugin="planner::pdla_planner::PDLAPlanner",
                remappings=[
                    ("odom", "/localization/odom"),
                    ("goal_current_odom", "/planner/wrench_planner/zoh_wrench_plan"),
                ],
                parameters=[
                    {"path_planner_path": path_planner_path},
                    cfg_pdla_planner,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                name="pdla_feedback_repub_component",
                namespace="planner/pdla_planner",
                package="projection_dynamic_look_ahead_planner",
                plugin="planner::pdla_planner::PDLAFeedbackRepub",
                remappings=[
                    ("feedback", "/planner/pdla_planner/pdla_plan/_action/feedback"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            pdla_container,
        ]
    )
