from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("dashboard"), "config", "dashboard.param.yaml"])

    return LaunchDescription(
        [
            Node(
                package="dashboard",
                executable="dashboard",
                name="dashboard_node",
                parameters=[config],
                remappings=[
                    ("dvl", "/driver/dvl"),
                    ("depth", "/driver/depth"),
                    ("environment", "/sensors_esp32_driver/environment"),
                    ("power_state", "/logic_distro_rp2040_driver/power_state"),
                    ("system_status", "/sensors_esp32_driver/system_status"),
                    ("system_switch", "/logic_distro_rp2040_driver/system_switch"),
                    ("imu", "/driver/imu"),
                ],
                output="screen",
            )
        ]
    )
