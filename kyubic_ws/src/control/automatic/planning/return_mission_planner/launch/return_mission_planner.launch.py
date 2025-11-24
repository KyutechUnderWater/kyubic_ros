from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return_planner_node = Node(
        package="return_mission_planner",
        executable="return_planner_node",
        name="return_planner",
        namespace="",
        output="screen",
        # 必要に応じてリマップ
        remappings=[
            ("odom", "/localization/odom"),
            ("goal_current_odom", "/planner/goal_current_odom"),
        ],
    )

    return LaunchDescription([
        return_planner_node
    ])