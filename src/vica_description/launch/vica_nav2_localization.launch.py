from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vica_description")
    nav2_share = FindPackageShare("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Absolute path to the occupancy-grid map YAML file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "vica_nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [nav2_share, "launch", "localization_launch.py"]
                    )
                ),
                launch_arguments={
                    "map": map_yaml,
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                }.items(),
            ),
        ]
    )
