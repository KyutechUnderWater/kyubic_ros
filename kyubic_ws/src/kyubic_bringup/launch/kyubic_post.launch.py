from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localization_dir = PathJoinSubstitution([FindPackageShare("localization"), "launch"])
    dvl_dir = PathJoinSubstitution([FindPackageShare("dvl_driver"), "launch"])
    actuator_dir = PathJoinSubstitution([FindPackageShare("actuator_rp2040_driver"), "launch"])
    # behavior_tree_dir = PathJoinSubstitution(
    #     [FindPackageShare("behavior_tree"), "launch"]
    # )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([dvl_dir, "dvl_driver.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([localization_dir, "localization_components.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([actuator_dir, "actuator_rp2040_driver.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            # IncludeLaunchDescription(
            #     PathJoinSubstitution([behavior_tree_dir, "behavior_tree.launch.py"]),
            #     launch_arguments={
            #         "log_level": "warn",
            #     }.items(),
            # ),
        ]
    )
