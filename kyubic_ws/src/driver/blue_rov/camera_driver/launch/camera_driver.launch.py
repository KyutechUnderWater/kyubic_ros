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
    forward_host = DeclareLaunchArgument(
        "forward_host",
        default_value="192.168.2.98",
        description="Client IP to forward the raw RTP stream to (empty to disable).",
    )
    forward_port = DeclareLaunchArgument(
        "forward_port",
        default_value="5601",
        description="UDP port on the client used for forwarded RTP.",
    )

    return LaunchDescription(
        [
            log_level,
            forward_host,
            forward_port,
            Node(
                package="camera_driver",
                executable="camera_driver",
                name="camera_driver",
                namespace="driver/blue_rov/camera_driver",
                parameters=[
                    config,
                    {
                        "forward_host": LaunchConfiguration("forward_host"),
                        "forward_port": LaunchConfiguration("forward_port"),
                    },
                ],
                remappings=[
                    ("image_raw", "/driver/blue_rov/camera/image_raw"),
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
