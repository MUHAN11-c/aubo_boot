from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    """
    实际的启动设置函数，在 OpaqueFunction 中调用
    这样可以访问 context 来执行 Command
    """
    # 获取参数值
    support_package = LaunchConfiguration("support_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    aubo_type = LaunchConfiguration("aubo_type")
    
    # 生成 robot_description 参数（与 aubo_moveit_bridge_ros1.launch.py 相同）
    robot_xacro_file_to_use = "aubo_ros2.xacro"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf/xacro/inc/", robot_xacro_file_to_use]
            ),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # 生成 robot_description_semantic 参数（需要执行 Command）
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content.perform(context)
    }
    
    # 所有服务节点共享的参数
    common_parameters = [
        robot_description,
        robot_description_semantic,
    ]
    
    # Demo Driver 服务节点（延迟启动，等待 MoveIt2 就绪）
    # 使用 TimerAction 延迟 15 秒启动，确保 MoveIt2 已完全初始化
    # 参数通过 launch 文件传递，节点使用 automatically_declare_parameters_from_overrides(true) 自动声明
    move_to_pose_server_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="demo_driver",
                executable="move_to_pose_server_node",
                name="move_to_pose_server",
                output="screen",
                parameters=common_parameters,
            )
        ]
    )
    
    plan_trajectory_server_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="demo_driver",
                executable="plan_trajectory_server_node",
                name="plan_trajectory_server",
                output="screen",
                parameters=common_parameters,
            )
        ]
    )
    
    execute_trajectory_server_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="demo_driver",
                executable="execute_trajectory_server_node",
                name="execute_trajectory_server",
                output="screen",
                parameters=common_parameters,
            )
        ]
    )
    
    get_current_state_server_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="demo_driver",
                executable="get_current_state_server_node",
                name="get_current_state_server",
                output="screen",
                parameters=common_parameters,
            )
        ]
    )
    
    set_speed_factor_server_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="demo_driver",
                executable="set_speed_factor_server_node",
                name="set_speed_factor_server",
                output="screen",
                parameters=common_parameters,
            )
        ]
    )
    
    return [
        move_to_pose_server_node,
        plan_trajectory_server_node,
        execute_trajectory_server_node,
        get_current_state_server_node,
        set_speed_factor_server_node,
    ]


def generate_launch_description():
    """
    启动 demo_driver 服务节点
    
    这些服务节点需要等待 MoveIt2 和 robot_description 参数就绪后才能启动。
    使用 TimerAction 延迟 15 秒启动，确保 MoveIt2 已完全初始化。
    
    使用方法：
        ros2 launch aubo_moveit_config demo_driver_services.launch.py
    
    注意：此 launch 文件需要先启动 MoveIt2（例如通过 aubo_moveit_bridge_ros1.launch.py）
    """
    
    # 声明启动参数（与 aubo_moveit_bridge_ros1.launch.py 保持一致）
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="aubo_description",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="Name of the MoveIt config package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="aubo_i5.srdf",
            description="Name of the SRDF file for MoveIt",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_i5",
            description="Type of Aubo robot",
        )
    )
    
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup),
    ])

