from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["debug"],
        description="Logging level",
    )

    emergency_surfacing_node = Node(
        package="emergency",
        executable="emergency_surfacing_component_node",
        name="emergency_surfacing",
        namespace="emergency",
        output="screen",
        parameters=[config],
        remappings=[
            ("robot_force", "/driver/robot_force"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_emergency",
        namespace="emergency",
        output="screen",
        parameters=[
            # configure と activate を両方実行
            {"autostart": True},
            {"node_names": ["/emergency/emergency_surfacing"]},
            {"bond_timeout": 0.0},
        ],
    )

    return LaunchDescription(
        [log_level_arg, emergency_surfacing_node, lifecycle_manager]
    )
