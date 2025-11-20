import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 引数定義: ターゲットPCのIPアドレス 
    remote_ip_arg = DeclareLaunchArgument(
        "remote_ip",
        default_value="192.168.9.245", 
        description="IP address of the PC running techno.py"
    )

    remote_port_arg = DeclareLaunchArgument(
        "remote_port",
        default_value="11111",
        description="Port number of the PC running techno.py"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    return LaunchDescription([
        remote_ip_arg,
        remote_port_arg,
        log_level_arg,
        
        Node(
            package="qr_planner",
            namespace="planner",
            executable="qr_planner_node",  # ★ CMakeで定義した正しい名前
            name="qr_planner",             # ★ パッケージ名と統一!!!
            remappings=[
                ("odom", "/localization/odom"), # ★ 推奨: 相対パスでremapした方が綺麗
            ],
            parameters=[{
                "remote_ip": LaunchConfiguration("remote_ip"),
                "remote_port": LaunchConfiguration("remote_port"),
                # 必要なら他のパラメータもここに追加
                "reach_tolerance.x": 0.1,
            }],
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level", LaunchConfiguration("log_level"),
            ],
        ),
    ])