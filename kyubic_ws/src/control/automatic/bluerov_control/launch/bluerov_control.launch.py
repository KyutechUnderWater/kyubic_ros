"""BlueROVミッション制御ノードと、BlueROV向けに配線したemergency_surfacingを起動する。

前提として driver_launcher の blue_rov_driver.launch.py (mavlink_driver, dvl75_driver) と
localization の localization_components.launch.py が別途起動済みであること
(README「起動方法」参照)。

emergency_surfacing は control/automatic/emergency パッケージの既存ノードをそのまま使うが、
Kyubic向けのemergency.launch.pyとは異なりrobot_forceをBlueROVのmavlink_driverへremapし、
かつnav2_lifecycle_managerでの常時activateは行わない(bluerov_control_nodeがEMERGENCY State
突入時にのみ change_state サービスでconfigure/activateを呼ぶ、README参照)。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")

    bluerov_control_config = PathJoinSubstitution(
        [FindPackageShare("bluerov_control"), "config", "bluerov_control.param.yaml"]
    )
    emergency_config = PathJoinSubstitution(
        [FindPackageShare("emergency"), "config", "emergency.param.yaml"]
    )

    bluerov_control_node = Node(
        package="bluerov_control",
        executable="bluerov_control_node",
        name="bluerov_control_node",
        namespace="control/automatic/bluerov_control",
        output="screen",
        parameters=[bluerov_control_config],
        remappings=[
            ("odom", "/localization/odom"),
            ("vehicle_state", "/driver/blue_rov/mavlink_driver/vehicle_state"),
            ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
            ("heartbeat", "/driver/blue_rov/mavlink_driver/heartbeat"),
            ("set_armed", "/driver/blue_rov/mavlink_driver/set_armed"),
            ("hydrophone_bearing", "/perception/hydrophone_bearing"),
            ("buoy_detection", "/perception/buoy_detection"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    emergency_surfacing_node = Node(
        package="emergency",
        executable="emergency_surfacing_component_node",
        name="emergency_surfacing",
        namespace="emergency",
        output="screen",
        parameters=[emergency_config],
        remappings=[
            ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
        ],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            bluerov_control_node,
            emergency_surfacing_node,
        ]
    )
