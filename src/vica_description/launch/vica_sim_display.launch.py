from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("vica_description")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_default_joint_states = LaunchConfiguration(
        "publish_default_joint_states"
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [package_share, "urdf", "VICA.xacro"]
                ),
                description="Absolute path to the VICA Xacro file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "vica_sim.rviz"]
                ),
                description="Absolute path to the RViz configuration.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the /clock published by Isaac Sim.",
            ),
            DeclareLaunchArgument(
                "publish_default_joint_states",
                default_value="false",
                description=(
                    "Publish zero positions for movable joints so wheel and caster "
                    "meshes are visible. Set false when Isaac Sim publishes /joint_states."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz2 with the VICA navigation configuration.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=IfCondition(publish_default_joint_states),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(launch_rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                on_exit=Shutdown(reason="RViz2 closed"),
                additional_env={
                    # Prevent Snap/IBus Qt plugins from injecting an
                    # incompatible core20 GLIBC into the ROS 2 RViz process.
                    "QT_IM_MODULE": "none",
                    "GTK_PATH": "",
                    "QT_PLUGIN_PATH": "/usr/lib/x86_64-linux-gnu/qt5/plugins",
                },
            ),
        ]
    )
