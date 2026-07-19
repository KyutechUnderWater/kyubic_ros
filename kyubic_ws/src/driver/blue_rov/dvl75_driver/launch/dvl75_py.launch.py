"""Launch the retained Python implementation of the DVL-75 driver."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the retained Python node.

    Returns:
        Launch description that starts ``dvl75_py`` with the standard configuration.
    """
    config = PathJoinSubstitution(
        [FindPackageShare("dvl75_driver"), "config", "dvl75_driver.param.yaml"]
    )
    log_level = DeclareLaunchArgument("log_level", default_value="info")

    return LaunchDescription(
        [
            log_level,
            Node(
                package="dvl75_driver",
                executable="dvl75_py",
                name="dvl75_py",
                namespace="driver/blue_rov/dvl75_driver",
                parameters=[config],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                output="screen",
            ),
        ]
    )
