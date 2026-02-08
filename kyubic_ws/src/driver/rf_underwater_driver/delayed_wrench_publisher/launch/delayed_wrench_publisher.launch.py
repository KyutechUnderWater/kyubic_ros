from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "delayed_wrench_publisher"

    # Config file
    config_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "delayed_wrench_publisher.param.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package=package_name,
                executable="delayed_wrench_publisher",
                name="delayed_wrench_publisher",
                output="screen",
                parameters=[config_file_path],
                remappings=[
                    ("wrench_plan", "/planner/wrench_planner/wrench_plan"),
                    ("pdla_feedback", "/planner/pdla_planner/pdla_feedback"),
                ],
            )
        ]
    )
