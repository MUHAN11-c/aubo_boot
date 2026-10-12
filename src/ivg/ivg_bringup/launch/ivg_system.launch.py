"""Start IVG with a single deployment-selected URDF; no automatic motion or IO."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ivg_description.description import GRIPPERS
from ivg_moveit_config.configuration import build_moveit_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    mode = LaunchConfiguration('hardware_mode').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    config = build_moveit_config(gripper, mode, robot_ip)
    controllers = Path(get_package_share_directory('aubo_e5_bringup')) / 'config/controllers.yaml'
    selected = ('joint_trajectory_controller' if mode == 'mock'
                else 'aubo_passthrough_trajectory_controller')
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[config.robot_description, str(controllers)], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=[selected], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[config.to_dict()], output='screen'),
    ]
    if mode == 'real':
        nodes.append(Node(package='controller_manager', executable='spawner',
                          arguments=['aubo_io_controller'], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('gripper', default_value='none', choices=list(GRIPPERS)),
        DeclareLaunchArgument('robot_ip', default_value='169.254.10.98'),
        OpaqueFunction(function=launch_setup),
    ])
