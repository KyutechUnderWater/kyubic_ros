"""Launch the BlueROV camera driver."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    """Create the camera driver launch description.

    Returns:
        LaunchDescription: The configured BlueROV camera driver node.
    """
    config = PathJoinSubstitution(
        [FindPackageShare("camera_driver"), "config", "camera_driver.param.yaml"]
    )
    log_level = DeclareLaunchArgument("log_level", default_value="info")

    return LaunchDescription(
        [
            log_level,
            Node(
                package="camera_driver",
                executable="camera_driver",
                name="camera_driver",
                namespace="driver/blue_rov/camera_driver",
                parameters=[config],
                remappings=[
                    ("image_raw", "/driver/blue_rov/camera/image_raw"),
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
