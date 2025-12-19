from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    command_srv_arg = DeclareLaunchArgument(
        "command_srv",
        default_value="/driver/dvl_driver/command",
        description="Service name for the dvl command",
    )

    plotter_node = Node(
        package="trdi_toolz",
        executable="trdi_toolz",
        name="trdi_toolz",
        output="screen",
        remappings=[
            ("/dvl_command", LaunchConfiguration("command_srv")),
        ],
    )

    return LaunchDescription(
        [
            command_srv_arg,
            plotter_node,
        ]
    )
