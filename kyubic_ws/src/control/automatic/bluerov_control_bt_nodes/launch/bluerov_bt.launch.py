"""BlueROV: Behavior Tree(bt_executor) + wrench_planner + zero_order_hold + emergency
+ heartbeat_publisher を起動する。

Kyubicの behavior_tree/launch/behavior_tree.launch.py と
control/automatic/planning/wrench_planner/launch/zoh_wrench_planner.launch.py を
BlueROV向けに合成したもの。wrench_planner/zero_order_hold/emergency/bt_executorの
ソースコードは一切変更していない(remapping・パラメータ・新規BT葉ノード登録のみ)。
新規のBT葉ノードはbluerov_control_bt_nodesパッケージ側にある
(CheckBuoyDetected/CheckBuoyInFrame/CheckBuoyPosition/CheckPingerPitch/
WriteHydrophoneWaypointCSV/WriteAxisOverrideWaypointCSV/WriteVisionWaypointCSV。
kyubic_ws/src/behavior_tree/src/bt_executor.cppに登録行を追加済み)。実行するBT XMLは
behavior_tree パッケージ側の bt_xml/iwakuni2026.xml (旧 bluerov_phase1.xml/
bluerov_phase2.xml は iwakuni2026.xml へ統合され削除済み。経緯は
bluerov_control_bt_nodes/README.md §10.1 参照)。iwakuni2026.xmlは「独自に
WrenchPlanを出し続ける移動ノード(旧GoToDepth/GoToBlackboardTarget等)は使わず、
現在odom基準の1点CSVを書いて既存のWaypointAction(PDLA)へ処理を任せる」方式に
統一している(README §10.6/§10.7参照)。

前提: driver_launcher の blue_rov_driver.launch.py と
localization の localization_components.launch.py が別途起動済みであること。
音響(SBL)は bluerov_bt では起動しない。別途
``ros2 launch sbl_control sbl.launch.py`` で PingerDirection を配信すること
(pinger_direction_topic launch引数でトピック名を合わせる)。
bt_executor は reed_switch_driver が publish する
/driver/blue_rov/mission_start_trigger を待ってから MainTree を開始し、
自動的に BlueROV2Auto ミッションを実行する(手動モードへの分岐なし)。

旧 bluerov_control/launch/bluerov_control.launch.py (bluerov_control_node、自前FSM+PID)は
このBT構成とは独立に残っている。移行が完了するまでは切り戻しできるよう並存させる。

launch引数:
  - main_tree(既定""): 個別テスト用。iwakuni2026.xml内の特定のBehaviorTree ID
    を直接rootとして起動する。空なら通常通りMainTreeを実行する
  - pinger_direction_topic(既定"/pinger_direction"): sbl_control が発行する
    planner_msgs/PingerDirection のトピック名
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")
    main_tree = LaunchConfiguration("main_tree")
    pinger_direction_topic = LaunchConfiguration("pinger_direction_topic")

    wrench_planner_config = PathJoinSubstitution(
        [
            FindPackageShare("bluerov_control_bt_nodes"),
            "config",
            "bluerov_wrench_planner.param.yaml",
        ]
    )
    p_pid_controller_path = PathJoinSubstitution(
        [FindPackageShare("bluerov_control_bt_nodes"), "config"]
    )
    emergency_config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )
    # iwakuni2026.xmlはbluerov_control_bt_nodesではなくbehavior_treeパッケージ側にある
    # (KyubicのbtExecutorNodeが共通で実行するBT XML置き場のため)。
    bt_xml_path = PathJoinSubstitution(
        [FindPackageShare("behavior_tree"), "bt_xml", "iwakuni2026.xml"]
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

    # BT Manager(iwakuni2026.xmlを実行。behavior_treeパッケージのソースは変更なし)
    # 新規BT葉ノードは相対トピック名(odom, zoh_wrench_plan, pinger_direction, buoy_detection)
    # しか知らないため、btExecutorNode(namespace無し)側で実トピック名にremapする
    # (Kyubic本家のbehavior_tree.launch.pyがimu/dvl/depth等をremapしているのと同じ流儀)。
    # pinger_direction: sbl_control(sbl.launch.py)が発行。BT葉ノードは相対名
    # pinger_direction を購読し、ここで実トピックへ remap する。
    bt_manager_node = Node(
        package="behavior_tree",
        executable="behavior_tree",
        name="btExecutorNode",
        parameters=[
            {
                "bt_xml_file": bt_xml_path,
                "main_tree_id": main_tree,
                # リードスイッチ(/driver/blue_rov/mission_start_trigger)発火まで
                # MainTree開始を待つ。発火後はジョイスティック操作なしでBlueROV2Autoへ直行する
                # (iwakuni2026.xml MainTree参照)。
                "wait_for_trigger": True,
            },
        ],
        remappings=[
            ("odom", "/localization/odom"),
            ("imu", "/driver/imu"),
            ("depth", "/driver/depth"),
            ("dvl", "/driver/dvl"),
            ("power_state", "/driver/blue_rov/mavlink_driver/power_state"),
            # ("leak", "/driver/leak"),  # 後述の leak 追加後
            ("zoh_wrench_plan", "/planner/wrench_planner/zoh_wrench_plan"),
            ("pinger_direction", pinger_direction_topic),
            ("buoy_detection", "/perception/buoy_detection"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    # buoy_detector(perception側の検出ノードを直接起動。buoy_detector_driverは使用しない)。
    # 出力トピック/buoy/relative_positionは、bt_xml/iwakuni2026.xmlのCheckBuoyInFrame/
    # CheckBuoyPosition/WriteVisionWaypointCSVがbuoy_relative_position_topicポートで
    # 直接購読するトピック名と一致させている。
    buoy_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("buoy_detector"), "launch", "buoy_detector.launch.py"]
            )
        ),
        launch_arguments={
            "input_image_topic": "/driver/blue_rov/camera/image_raw",
            "output_topic": "/buoy/relative_position",
            "body_frame_id": "base_link",
            "confidence": "0.25",
            "device": "cpu",
            # iwakuni2026.xmlのCheckBuoyPosition(FinalApproach)はposition_valid==trueを
            # 要求するため、position_estimation_enabledはtrueが必須(falseだとposition_valid
            # が常にfalseになりFinalApproachが永久に成立しない)。
            "position_estimation_enabled": "true",
        }.items(),
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
                "main_tree",
                default_value="",
                description="個別テスト用。iwakuni2026.xml内のBehaviorTree IDを直接rootとして"
                "起動する。空なら通常のMainTreeを実行",
            ),
            DeclareLaunchArgument(
                "pinger_direction_topic",
                default_value="/pinger_direction",
                description="sbl_control(sbl.launch.py)が発行する planner_msgs/PingerDirection "
                "トピック名",
            ),
            wrench_planner_container,
            emergency_node,
            bt_manager_node,
            buoy_detector_launch,
            heartbeat_publisher_node,
        ]
    )
