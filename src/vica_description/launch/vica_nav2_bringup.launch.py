import fcntl
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
            "Another vica_nav2_bringup launch is already running. "
            "Stop it with Ctrl-C before starting a different map."
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
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_default_joint_states = LaunchConfiguration(
        "publish_default_joint_states"
    )

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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "vica_nav2_localization.launch.py"]
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "vica_nav2_navigation.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            # The startup gate activates these nodes after localization TF is stable.
            "autostart": "false",
        }.items(),
    )

    startup_gate = Node(
        package="vica_description",
        executable="nav2_startup_gate",
        name="nav2_startup_gate",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"target_frame": "map"},
            {"source_frame": "base_footprint"},
            {"stable_duration": 3.0},
        ],
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "publish_default_joint_states", default_value="false"
            ),
            display,
            localization,
            navigation,
            startup_gate,
        ]
    )
