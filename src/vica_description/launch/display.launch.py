import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
# robot_description 값을 문자열(str)로 명시하지 않아서 파싱 실패.그래서 아래 import 추가
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("vica_description")
    default_model = os.path.join(pkg_share, "urdf", "VICA.xacro")
    default_rviz_config = os.path.join(pkg_share, "rviz", "urdf.rviz")

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")

    # robot_description = {
    #     "robot_description": Command([
    #         FindExecutable(name="xacro"),
    #         " ",
    #         model,
    #     ])
    # }
    # robot_description 값을 문자열(str)로 명시하지 않아서 파싱 실패.그래서 아래와 같이 수정
    robot_description = {
    "robot_description": ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            model,
        ]),
        value_type=str,
    )
}

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
