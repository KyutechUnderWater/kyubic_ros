from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="turtlesim", executable="turtlesim_node", name="turtlesim"),
            Node(
                package="internship_ros2_basics",
                executable="turtle_circle_publisher",
                name="turtle_circle_publisher",
                parameters=[{"linear_speed": 1.5, "angular_speed": 1.0}],
            ),
            Node(
                package="internship_ros2_basics",
                executable="turtle_pose_subscriber",
                name="turtle_pose_subscriber",
            ),
        ]
    )
