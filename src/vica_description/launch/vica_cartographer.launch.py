"""Cartographer 2D mapping against a running Isaac stage.

Same stack as the physical robot (VICA-smarthandle: vica_cartographer), pointed
at Isaac's /scan and /odom instead of the lidar driver and the EKF.

    ros2 launch vica_description vica_cartographer.launch.py

Drive with teleop, then save:

    ros2 run nav2_map_server map_saver_cli -t /map -f <path> \\
        --ros-args -p save_map_timeout:=30.0 -p map_subscribe_transient_local:=true

Requires cartographer_ros, which is not a default Jazzy install:

    sudo apt install ros-jazzy-cartographer-ros

This takes the same instance lock as vica_nav2_bringup, because cartographer and
AMCL both publish map -> odom. Run them together and the two take turns winning,
which shows up as a robot that teleports rather than as an error.
"""

import fcntl
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_INSTANCE_LOCKS = []


def _acquire_instance_lock(_context):
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "/tmp")
    lock_path = os.path.join(runtime_dir, f"vica_nav2_bringup_{os.getuid()}.lock")
    lock_file = open(lock_path, "a+", encoding="utf-8")

    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_file.close()
        raise RuntimeError(
            "vica_nav2_bringup or another mapping launch is already running. "
            "Both own the map -> odom transform, so stop the other one first."
        ) from error

    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    _INSTANCE_LOCKS.append(lock_file)
    return []


def generate_launch_description():
    package_share = FindPackageShare("vica_description")

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_default_joint_states = LaunchConfiguration(
        "publish_default_joint_states"
    )
    configuration_directory = LaunchConfiguration("configuration_directory")
    configuration_basename = LaunchConfiguration("configuration_basename")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "vica_sim_display.launch.py"]
            )
        ),
        launch_arguments={
            "model": model,
            "rviz_config": rviz_config,
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "publish_default_joint_states": publish_default_joint_states,
        }.items(),
    )

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", configuration_directory,
            "-configuration_basename", configuration_basename,
        ],
        remappings=[
            ("scan", scan_topic),
            ("odom", odom_topic),
        ],
    )

    # Cartographer works in submaps; this node is what turns them into the
    # /map OccupancyGrid that RViz draws and map_saver_cli writes out.
    occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution", resolution,
            "-publish_period_sec", publish_period_sec,
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            OpaqueFunction(function=_acquire_instance_lock),
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [package_share, "urdf", "VICA.xacro"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "vica_sim.rviz"]
                ),
            ),
            DeclareLaunchArgument(
                "configuration_directory",
                default_value=PathJoinSubstitution([package_share, "config"]),
            ),
            DeclareLaunchArgument(
                "configuration_basename", default_value="vica_2d.lua"
            ),
            # Matches the resolution of the existing maps and of the global
            # costmap; changing it here alone would silently mismatch both.
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            # Isaac drives the clock; unlike on the robot, true is the norm here.
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "publish_default_joint_states", default_value="false"
            ),
            display,
            cartographer,
            occupancy_grid,
        ]
    )
