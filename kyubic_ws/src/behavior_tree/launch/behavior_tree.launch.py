from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 設定ファイルのパス
    behavior_tree_config = PathJoinSubstitution(
        [FindPackageShare("behavior_tree"), "config", "behavior_tree.param.yaml"]
    )

    joy2wrench_config = PathJoinSubstitution(
        [FindPackageShare("joy2wrench"), "config", "joy2wrench.param.yaml"]
    )

    emergency_surfacing_config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )

    # launchファイルのパス
    joy_common_dir = PathJoinSubstitution([FindPackageShare("joy_common"), "launch"])

    planner_launcher_dir = PathJoinSubstitution(
        [FindPackageShare("planner_launcher"), "launch"]
    )

    # ノードの定義
    plannler_launch = IncludeLaunchDescription(
        PathJoinSubstitution([planner_launcher_dir, "planner_launcher.launch.py"]),
        launch_arguments={
            "log_level": "warn",
        }.items(),
    )

    # 制御対象のLifecycleノード
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

    # BT Manager
    bt_manager_node = Node(
        package="behavior_tree",
        executable="behavior_tree",
        name="btExecutorNode",
        parameters=[behavior_tree_config],
        remappings=[
            ("power_state", "/logic_distro_rp2040_driver/power_state"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            plannler_launch,
            manual_node,
            emergency_node,
            bt_manager_node,
        ]
    )
