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
                    ("dvl", "/driver/dvl_driver/dvl"),
                    ("depth", "/driver/sensors_esp32_driver/depth"),
                    ("environment", "/driver/sensors_esp32_driver/environment"),
                    ("power_state", "/driver/logic_distro_rp2040_driver/power_state"),
                    ("system_status", "/driver/sensors_esp32_driver/system_status"),
                    ("system_switch", "/driver/logic_distro_rp2040_driver/system_switch"),
                    ("imu", "/driver/imu_driver/imu"),
                    ("imu_reset", "/driver/sensors_esp32_driver/imu_reset"),
                    ("tilt_servo", "/driver/sensors_esp32_driver/tilt_servo"),
                    ("led", "/driver/actuator_rp2040_driver/led"),
                ],
                output="screen",
            )
        ]
    )
