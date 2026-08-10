"""Launch the BlueROV web dashboard."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Create the BlueROV dashboard launch description.

    Returns:
        LaunchDescription: The configured dashboard node.
    """
    config = PathJoinSubstitution(
        [FindPackageShare("bluerov_dashboard"), "config", "bluerov_dashboard.param.yaml"]
    )
    log_level = DeclareLaunchArgument("log_level", default_value="info")

    return LaunchDescription(
        [
            log_level,
            Node(
                package="bluerov_dashboard",
                executable="bluerov_dashboard",
                name="bluerov_dashboard",
                parameters=[config],
                remappings=[
                    ("depth", "/driver/depth"),
                    ("imu", "/driver/imu"),
                    ("power_state", "/driver/blue_rov/mavlink_driver/power_state"),
                    ("gnss", "/driver/blue_rov/mavlink_driver/gnss"),
                    ("vehicle_state", "/driver/blue_rov/mavlink_driver/vehicle_state"),
                    ("dvl", "/driver/blue_rov/dvl75_driver/dvl75"),
                    ("image_raw", "/driver/blue_rov/camera/image_raw"),
                    ("set_armed", "/driver/blue_rov/mavlink_driver/set_armed"),
                    ("led", "/driver/blue_rov/mavlink_driver/led"),
                    ("camera_tilt", "/driver/blue_rov/mavlink_driver/camera_tilt"),
                    ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
                    ("heartbeat", "/driver/blue_rov/mavlink_driver/heartbeat"),
                    ("odom", "/localization/odom"),
                    ("mission_start_trigger", "/driver/blue_rov/mission_start_trigger"),
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
