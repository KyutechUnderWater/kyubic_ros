from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg_pdla_planner = PathJoinSubstitution(
        [
            FindPackageShare("projection_dynamic_look_ahead_planner"),
            "config",
            "pdla_planner.param.yaml",
        ]
    )
    path_planner_path = FindPackageShare("path_planner")

    cfg_qr_planner = PathJoinSubstitution(
        [FindPackageShare("qr_planner"), "config", "qr_planner.param.yaml"]
    )

    cfg_wrench_planner = PathJoinSubstitution(
        [FindPackageShare("wrench_planner"), "config", "wrench_planner.param.yaml"]
    )

    p_pid_controller_path = FindPackageShare("p_pid_controller")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="planner_launcher_component_container",
        namespace="planner",
        package="rclcpp_components",
        executable="component_container_isolated",
        output="screen",
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        arguments=[
            "--use_multi_threaded_executor",
        ],
        composable_node_descriptions=[
            ComposableNode(
                name="pdla_planner_component",
                namespace="planner/pdla_planner",
                package="projection_dynamic_look_ahead_planner",
                plugin="planner::PDLAPlanner",
                remappings=[
                    ("odom", "/localization/odom"),
                ],
                parameters=[
                    {"path_planner_path": path_planner_path},
                    cfg_pdla_planner,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="qr_planner_component",
                namespace="planner/qr_planner",
                package="qr_planner",
                plugin="planner::QRPlanner",
                remappings=[
                    ("odom", "/localization/odom"),
                    ("zed_power", "/sensors_esp32_driver/zed_power"),
                    ("goal_current_odom", "/planner/goal_current_odom"),
                ],
                parameters=[
                    cfg_qr_planner,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="wrench_planner_component",
                namespace="planner/wrench_planner",
                package="wrench_planner",
                plugin="planner::WrenchPlanner",
                remappings=[
                    ("goal_current_odom", "/planner/pdla_planner/goal_current_odom"),
                    ("robot_force", "/driver/robot_force"),
                    ("targets", "/rt_pose_plotter/targets"),
                ],
                parameters=[
                    {"p_pid_controller_path": p_pid_controller_path},
                    cfg_wrench_planner,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
    )

    return LaunchDescription([log_level_arg, action_component_container])
