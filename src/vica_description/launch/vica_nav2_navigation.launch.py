from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            # Same treatment, same reason. The robot's config carries this one
            # as /home/<someone>/VICA-smarthandle/vica_ros2_ws/install/... and
            # the planner simply does not come up anywhere else. Substituting
            # keeps the machine out of the YAML.
            "lattice_filepath": PathJoinSubstitution(
                [package_share, "config", "lattice", "output_r010_diff.json"]
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
            # Depth, pressed into one plane. Values copied from
            # vica_ros2_ws@dev's nav2_map_test.launch.py, which is where the
            # robot settled them.
            #
            # The costmap consumes /camera/depth_scan, not the raw cloud. The
            # robot's reason: a camera 1.045 m up cannot clear a cell below it,
            # because the ray grazing that cell has to reach floor 6.7 m out to
            # do it, and indoors a wall stops it 3 to 4 m away. A lidar clears
            # well because it looks at its own height. Flattening the depth
            # gives it that property; without it, marked cells never clear and
            # the costmap keeps an afterimage that closes a 1 m corridor.
            #
            # The band is 0.30 to 1.05 above the ground: below 0.30 the floor
            # comes in when the body rocks, above 1.05 is over the robot's own
            # head. target_frame is base_footprint so both are ground-relative
            # and match the numbers in nav2_params.yaml.
            #
            # If the camera is not publishing, this node waits quietly and the
            # depth_scan source simply contributes nothing.
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="depth_band_to_scan",
                output="screen",
                remappings=[
                    ("cloud_in", "/camera/camera/depth/color/points"),
                    ("scan", "/camera/depth_scan"),
                ],
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "target_frame": "base_footprint",
                    "transform_tolerance": 0.05,
                    "min_height": 0.30,
                    "max_height": 1.05,
                    "angle_min": -0.75,
                    "angle_max": 0.75,
                    "angle_increment": 0.0087,
                    "scan_time": 0.0667,
                    "range_min": 0.30,
                    "range_max": 4.0,
                    "use_inf": True,
                    "queue_size": 1,
                }],
                respawn=False,
            ),
        ]
    )
