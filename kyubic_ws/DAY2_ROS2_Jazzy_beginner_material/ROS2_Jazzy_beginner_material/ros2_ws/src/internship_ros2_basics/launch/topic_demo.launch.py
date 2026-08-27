from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="internship_ros2_basics",
                executable="string_publisher",
                name="string_publisher",
            ),
            Node(
                package="internship_ros2_basics",
                executable="string_subscriber",
                name="string_subscriber",
            ),
        ]
    )
