from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="aubo_description",
            description="Name of the support package",
        )
    )

    # 获取参数
    support_package = LaunchConfiguration("support_package")

    # 使用 ROS1 对齐的 e5 模型
    robot_xacro_file = "aubo_ros2.xacro"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf/xacro/inc/", robot_xacro_file]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher - 发布机器人状态
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint State Publisher GUI - 可以手动调整关节角度
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz2 - 可视化
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("aubo_moveit_config"), "config", "view_robot.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
