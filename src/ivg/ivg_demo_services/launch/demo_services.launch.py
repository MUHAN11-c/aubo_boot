"""Use the same deployment-selected IVG model as ivg_bringup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ivg_description.description import GRIPPERS
from ivg_moveit_config.configuration import build_moveit_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    gripper = LaunchConfiguration('gripper').perform(context)
    mode = LaunchConfiguration('hardware_mode').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    config = build_moveit_config(gripper, mode, robot_ip)
    share = Path(get_package_share_directory('ivg_demo_services'))
    common = [config.robot_description, config.robot_description_semantic,
              config.robot_description_kinematics, str(share / 'config/demo_services.yaml'),
              {'io_simulated': mode == 'mock'}]
    return [
        Node(package='ivg_demo_services', executable='system_monitor_node', output='screen'),
        Node(package='ivg_demo_services', executable='move_service_node',
             name='move_service_node', parameters=common, output='screen'),
        Node(package='ivg_demo_services', executable='grasp_trigger_node',
             name='grasp_trigger_node', parameters=common + [
                 {'ee_link': 'grasp_tcp' if gripper == 'gripper2' else 'tool_tcp'}],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='none', choices=list(GRIPPERS)),
        DeclareLaunchArgument('hardware_mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('robot_ip', default_value='169.254.10.98'),

        OpaqueFunction(function=launch_setup),
    ])
