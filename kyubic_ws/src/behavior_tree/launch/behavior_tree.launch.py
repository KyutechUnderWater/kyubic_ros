import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # # PDLA (Action Server) の設定
    # pdla_config = os.path.join(
    #     get_package_share_directory("projection_dynamic_look_ahead_planner"),
    #     "config",
    #     "pdla_planner_action.param.yaml",
    # )
    #
    # # WaypointNode (Action Client) に渡すCSVパス
    # waypoint_csv_path = os.path.join(
    #     get_package_share_directory("projection_dynamic_look_ahead_planner"),
    #     "config",
    #     "assets/sample/mapping_alt_generated.csv",  # 仮のパス
    # )
    #
    # # WrenchPlanner の設定ファイルパス
    # wrench_planner_config = os.path.join(
    #     get_package_share_directory("wrench_planner"),
    #     "config",
    #     "wrench_planner.param.yaml",
    # )
    # # WrenchPlanner が必要とする p_pid_controller のパス（仮）
    # p_pid_controller_path = get_package_share_directory("p_pid_controller")

    joy_common_dir = PathJoinSubstitution([FindPackageShare("joy_common"), "launch"])
    joy2wrench_config = PathJoinSubstitution(
        [FindPackageShare("joy2wrench"), "config", "joy2wrench.param.yaml"]
    )

    emergency_surfacing_config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )

    joy_common_launch = IncludeLaunchDescription(
        PathJoinSubstitution([joy_common_dir, "joy_common.launch.py"]),
        launch_arguments={
            "log_level": "warn",
        }.items(),
    )

    wrench_action_demo = Node(
        package="wrench_action_sample",
        executable="wrench_action_server",
        name="wrench_action",
        namespace="",
        remappings=[
            ("robot_force", "/driver/robot_force"),
        ],
        output="screen",
    )

    fibonacci_action_demo = Node(
        package="action_tutorials_cpp",
        executable="fibonacci_action_server",
        name="fibonacci_action",
        namespace="",
        output="screen",
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

    # waypoint_node = LifecycleNode(
    #     package="behavior_tree",
    #     executable="waypointNode",
    #     name="waypointNode",
    #     namespace="",
    #     output="screen",
    #     parameters=[{"csv_path": waypoint_csv_path}],
    # )

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

    # 3. BT Manager
    bt_manager_node = Node(
        package="behavior_tree",
        executable="behavior_tree",
        name="bt_manager_node",
        output="screen",
    )

    # # 4. PDLA Action Server
    # pdla_server_node = Node(
    #     package="projection_dynamic_look_ahead_planner",
    #     executable="pdla_planner_component_node",
    #     name="pdla_planner_action",
    #     namespace="",
    #     output="screen",
    #     parameters=[pdla_config],
    #     remappings=[
    #         (
    #             "odom",
    #             "/localization/odom",
    #         )  # 仮(localization パッケージ）の構成と合っている?)
    #     ],
    # )
    #
    # # 5. Wrench Planner ノード を追加
    # wrench_planner_node = Node(
    #     package="wrench_planner",
    #     executable="wrench_planner_component_node",
    #     name="wrench_planner",
    #     namespace="",  # 必要に応じて 'planner/wrench_planner' などに変更
    #     output="screen",
    #     parameters=[
    #         wrench_planner_config,
    #         # wrench_planner が p_pid_controller_path を必要とする場合
    #         {"p_pid_controller_path": p_pid_controller_path},
    #     ],
    #     remappings=[
    #         # PDLAが出力する仮想目標点を購読する
    #         (
    #             "goal_current_odom",
    #             "/pdla_planner_action/goal_current_odom",
    #         ),  # (仮)pdla_server_node の出力トピック名（[ノード名]/goal_current_odom）と一致しているか
    #         # 必要に応じて他のリマッピング
    #         # ("robot_force", "/driver/robot_force"),
    #     ],
    # )

    return LaunchDescription(
        [
            fibonacci_action_demo,
            wrench_action_demo,
            joy_common_launch,
            manual_node,
            emergency_node,
            bt_manager_node,
            # waypoint_node,
            # pdla_server_node,
            # wrench_planner_node # [修正] wrench_planner の起動を追加
        ]
    )
