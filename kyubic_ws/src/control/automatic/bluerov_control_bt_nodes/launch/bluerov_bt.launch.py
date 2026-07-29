"""BlueROV Phase 1: Behavior Tree(bt_executor) + wrench_planner + zero_order_hold + emergency
+ heartbeat_publisher を起動する。

Kyubicの behavior_tree/launch/behavior_tree.launch.py と
control/automatic/planning/wrench_planner/launch/zoh_wrench_planner.launch.py を
BlueROV向けに合成したもの。wrench_planner/zero_order_hold/emergency/bt_executorの
ソースコードは一切変更していない(remapping・パラメータ・新規BT葉ノード登録のみ)。
新規のBT葉ノードはbluerov_control_bt_nodesパッケージのSetDepthTargetのみ
(kyubic_ws/src/behavior_tree/src/bt_executor.cppに登録行を追加済み)。

前提: driver_launcher の blue_rov_driver.launch.py と
localization の localization_components.launch.py が別途起動済みであること。

旧 bluerov_control/launch/bluerov_control.launch.py (bluerov_control_node、自前FSM+PID)は
このPhase 1構成とは独立に残っている。移行が完了するまでは切り戻しできるよう並存させる。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")

    wrench_planner_config = PathJoinSubstitution(
        [FindPackageShare("bluerov_control_bt_nodes"), "config", "bluerov_wrench_planner.param.yaml"]
    )
    p_pid_controller_path = PathJoinSubstitution(
        [FindPackageShare("bluerov_control_bt_nodes"), "config"]
    )
    emergency_config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )
    bt_xml_path = PathJoinSubstitution(
        [FindPackageShare("bluerov_control_bt_nodes"), "bt_xml", "bluerov_phase1.xml"]
    )

    # zero_order_hold + wrench_planner (Kyubicのzoh_wrench_planner.launch.pyと同じ構成)
    wrench_planner_container = ComposableNodeContainer(
        name="bluerov_wrench_planner_container",
        namespace="planner/wrench_planner",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="wrench_planner",
                plugin="planner::wrench_planner::ZeroOrderHold",
                namespace="planner/wrench_planner",
                name="zoh_wrench_planner_component",
                remappings=[("odom", "/localization/odom")],
                parameters=[wrench_planner_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="wrench_planner",
                plugin="planner::wrench_planner::WrenchPlanner",
                namespace="planner/wrench_planner",
                name="wrench_planner_component",
                remappings=[
                    ("odom", "/localization/odom"),
                    ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
                    ("targets", "/planner/wrench_planner/targets"),
                ],
                parameters=[
                    {"p_pid_controller_path": p_pid_controller_path},
                    wrench_planner_config,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    # emergency(既存emergencyパッケージをBlueROV向けremapで再利用。ソース変更なし)
    emergency_node = LifecycleNode(
        package="emergency",
        executable="emergency_surfacing_component_node",
        name="emergencyNode",
        namespace="",
        remappings=[("robot_force", "/driver/blue_rov/mavlink_driver/robot_force")],
        parameters=[emergency_config],
        output="screen",
    )

    # BT Manager(bluerov_phase1.xmlを実行。behavior_treeパッケージのソースは変更なし)
    # SetDepthTargetは相対トピック名(odom, zoh_wrench_plan)しか知らないため、
    # btExecutorNode(namespace無し)側でwrench_planner/zero_order_holdの実トピック名にremapする
    # (Kyubic本家のbehavior_tree.launch.pyがimu/dvl/depth等をremapしているのと同じ流儀)。
    bt_manager_node = Node(
        package="behavior_tree",
        executable="behavior_tree",
        name="btExecutorNode",
        parameters=[{"bt_xml_file": bt_xml_path}],
        remappings=[
            ("odom", "/localization/odom"),
            ("zoh_wrench_plan", "/planner/wrench_planner/zoh_wrench_plan"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    heartbeat_publisher_node = Node(
        package="bluerov_control",
        executable="bluerov_heartbeat_publisher",
        name="bluerov_heartbeat_publisher",
        remappings=[("heartbeat", "/driver/blue_rov/mavlink_driver/heartbeat")],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            wrench_planner_container,
            emergency_node,
            bt_manager_node,
            heartbeat_publisher_node,
        ]
    )
