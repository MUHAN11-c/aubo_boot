from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # 直接读取使用 DAE 格式的 URDF 文件
    urdf_file = os.path.join(
        get_package_share_directory('aubo_description'),
        'urdf',
        'aubo_e5_10_dae.urdf'
    )
    
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
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
