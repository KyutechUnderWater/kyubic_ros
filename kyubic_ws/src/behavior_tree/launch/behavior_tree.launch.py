import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- 設定ファイルのパス定義 ---

    # PDLA (Action Server) の設定
    pdla_config = os.path.join(
        get_package_share_directory("projection_dynamic_look_ahead_planner"),
        "config",
        "pdla_planner.param.yaml",
    )

    # WrenchPlanner の設定ファイルパス
    wrench_planner_config = os.path.join(
        get_package_share_directory("wrench_planner"),
        "config",
        "wrench_planner.param.yaml",
    )
    # WrenchPlanner が必要とする p_pid_controller のパス
    p_pid_controller_path = get_package_share_directory("p_pid_controller")

    # Behavior Tree の設定
    behavior_tree_config = PathJoinSubstitution(
        [FindPackageShare("behavior_tree"), "config", "behavior_tree.param.yaml"]
    )

    joy_common_dir = PathJoinSubstitution([FindPackageShare("joy_common"), "launch"])
    joy2wrench_config = PathJoinSubstitution(
        [FindPackageShare("joy2wrench"), "config", "joy2wrench.param.yaml"]
    )

    emergency_surfacing_config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )

    # --- ノードの定義 ---

    # 1. Joy Common (コントローラ入力の変換)
    # /joy -> /joy_common/joy_common に変換します
    joy_common_launch = IncludeLaunchDescription(
        PathJoinSubstitution([joy_common_dir, "joy_common.launch.py"]),
        launch_arguments={
            "log_level": "warn",
        }.items(),
    )

    # 2. 制御対象のLifecycleノード
    manual_node = LifecycleNode(
        package="joy2wrench",
        executable="joy2wrench_component_node",
        name="manualNode",
        namespace="",
        remappings=[
            ("joy_common", "/joy_common/joy_common"),
            ("robot_force", "/driver/robot_force"),
        ],
        parameters=[joy2wrench_config],
        output="screen",
    )

    emergency_node = LifecycleNode(
        package="emergency",
        executable="emergency_surfacing_component_node",
        name="emergencyNode",
        namespace="",
        remappings=[
            ("robot_force", "/driver/robot_force"),
        ],
        parameters=[emergency_surfacing_config],
        output="screen",
    )

    # 3. BT Manager (ビヘイビアツリー実行ノード)
    bt_manager_node = Node(
        package="behavior_tree",
        executable="behavior_tree",
        name="btExecutorNode",
        parameters=[behavior_tree_config],
        output="screen",
    )

    pdla_server_node = Node(
        package="projection_dynamic_look_ahead_planner",
        executable="pdla_planner_component_node",
        name="pdla_planner", 
        namespace="",
        output="screen",
        parameters=[pdla_config],
        remappings=[
            ("odom", "/localization/odom"), 
        ],
    )

    wrench_planner_node = Node(
        package="wrench_planner",
        executable="wrench_planner_component_node",
        name="wrench_planner",
        namespace="", 
        output="screen",
        parameters=[
            wrench_planner_config,
            {"p_pid_controller_path": p_pid_controller_path},
        ],
        remappings=[
            ("goal_current_odom", "/goal_current_odom"),
            ("robot_force", "/driver/robot_force"),
        ],
    )

    qr_server_node = Node(
        package="qr_planner",
        executable="qr_planner_node",
        name="qr_planner", 
        namespace="",
        output="screen",
        # 必要に応じてパラメータを追加
        # parameters=[{"remote_ip": "192.168.x.x"}],
        remappings=[
            ("odom", "/localization/odom"), 
        ],
    )

    return LaunchDescription(
        [
            joy_common_launch,
            manual_node,
            emergency_node,
            bt_manager_node,
            pdla_server_node,    
            wrench_planner_node, 
            qr_server_node,  
        ]
    )