"""BlueROV Phase 2: Behavior Tree(bt_executor) + wrench_planner + zero_order_hold + emergency
+ heartbeat_publisher を起動する。

Kyubicの behavior_tree/launch/behavior_tree.launch.py と
control/automatic/planning/wrench_planner/launch/zoh_wrench_planner.launch.py を
BlueROV向けに合成したもの。wrench_planner/zero_order_hold/emergency/bt_executorの
ソースコードは一切変更していない(remapping・パラメータ・新規BT葉ノード登録のみ)。
新規のBT葉ノードはbluerov_control_bt_nodesパッケージの5種類
(GoToDepth/GoToBlackboardTarget/CheckPingerFound/CheckBuoyDetected/CheckRosBoolParam。
kyubic_ws/src/behavior_tree/src/bt_executor.cppに登録行を追加済み)。

前提: driver_launcher の blue_rov_driver.launch.py と
localization の localization_components.launch.py が別途起動済みであること。

旧 bluerov_control/launch/bluerov_control.launch.py (bluerov_control_node、自前FSM+PID)は
このPhase 2構成とは独立に残っている。移行が完了するまでは切り戻しできるよう並存させる。

launch引数:
  - depth_target_m(既定1.0m): DESCEND_TO_7M相当のGoToDepthが使う潜航目標深度
  - main_tree(既定""): 個別テスト用。bluerov_phase2.xml内の特定のBehaviorTree ID
    (MainTree_VisualPath等)を直接rootとして起動する。空なら通常通りMainTreeを実行する
    (例: ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_VisualPath)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")
    depth_target_m = LaunchConfiguration("depth_target_m")
    main_tree = LaunchConfiguration("main_tree")

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
        [FindPackageShare("bluerov_control_bt_nodes"), "bt_xml", "bluerov_phase2.xml"]
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

    # BT Manager(bluerov_phase2.xmlを実行。behavior_treeパッケージのソースは変更なし)
    # 新規BT葉ノードは相対トピック名(odom, zoh_wrench_plan, pinger_direction, buoy_detection)
    # しか知らないため、btExecutorNode(namespace無し)側で実トピック名にremapする
    # (Kyubic本家のbehavior_tree.launch.pyがimu/dvl/depth等をremapしているのと同じ流儀)。
    # pinger_direction/buoy_detectionの実際のトピック名は、以前bluerov_control側で
    # 確認したものと同じ(音響班/画像処理班のノードが決まり次第、要再確認)。
    bt_manager_node = Node(
        package="behavior_tree",
        executable="behavior_tree",
        name="btExecutorNode",
        parameters=[
            {
                "bt_xml_file": bt_xml_path,
                "default_depth_m": ParameterValue(depth_target_m, value_type=float),
                "main_tree_id": main_tree,
            }
        ],
        remappings=[
            ("odom", "/localization/odom"),
            ("zoh_wrench_plan", "/planner/wrench_planner/zoh_wrench_plan"),
            ("pinger_direction", "/pinger_direction"),
            ("buoy_detection", "/perception/buoy_detection"),
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
            DeclareLaunchArgument(
                "depth_target_m",
                default_value="1.0",
                description="GoToDepth(DESCEND_TO_7M相当)の潜航目標深度[m](NED、正=下方向)。"
                "プールの実測水深に合わせて指定すること(競技用水槽向けの既定7.0mは深すぎる場合がある)",
            ),
            DeclareLaunchArgument(
                "main_tree",
                default_value="",
                description="個別テスト用。bluerov_phase2.xml内のBehaviorTree ID"
                "(例: MainTree_VisualPath)を直接rootとして起動する。空なら通常のMainTreeを実行",
            ),
            wrench_planner_container,
            emergency_node,
            bt_manager_node,
            heartbeat_publisher_node,
        ]
    )
