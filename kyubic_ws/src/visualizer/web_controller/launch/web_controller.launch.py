from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Declear Launch Argument ---
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/localization/odom",
        description="Topic name for odometry messages",
    )
    targets_topic_arg = DeclareLaunchArgument(
        "targets_topic",
        default_value="/controller/p_pid_controller/targets",
        description="Topic name for the target Z position",
    )
    front_cam_topic_arg = DeclareLaunchArgument(
        "front_cam_topic",
        default_value="/camera/front",
        description="Topic name for the target Z position",
    )
    bottom_cam_topic_arg = DeclareLaunchArgument(
        "bottom_cam_topic",
        default_value="/camera/bottom",
        description="Topic name for the target Z position",
    )

    # --- Define Node ---
    viewer_node = Node(
        package="web_controller",
        executable="web_controller",
        name="web_controller",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
            ("targets", LaunchConfiguration("targets_topic")),
            ("camera/front", LaunchConfiguration("front_cam_topic")),
            ("camera/bottom", LaunchConfiguration("bottom_cam_topic")),
        ],
    )

    # --- Generate LaunchDescription ---
    return LaunchDescription(
        [
            odom_topic_arg,
            targets_topic_arg,
            front_cam_topic_arg,
            bottom_cam_topic_arg,
            viewer_node,
        ]
    )
