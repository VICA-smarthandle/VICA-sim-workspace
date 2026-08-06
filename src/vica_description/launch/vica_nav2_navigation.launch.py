from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = FindPackageShare("vica_description")
    nav2_share = FindPackageShare("nav2_bringup")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # The behaviour tree is the robot's: nav2's default with BackUp removed,
    # because a person walks behind holding the handle and reversing drives the
    # robot into them.
    #
    # The path is substituted here rather than written into the YAML because it
    # is an install path that differs per machine. RewrittenYaml only replaces
    # keys that already exist, so the config carries a placeholder -- and if
    # this substitution is ever skipped, bt_navigator fails to find the file
    # rather than quietly falling back to the default tree with BackUp in it.
    rewritten = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [package_share, "behavior_trees",
                 "navigate_to_pose_no_backup.xml"]
            ),
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "vica_nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [nav2_share, "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "params_file": rewritten,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                }.items(),
            ),
        ]
    )
