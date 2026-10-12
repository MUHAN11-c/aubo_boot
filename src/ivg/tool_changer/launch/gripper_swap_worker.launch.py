"""Use the same deployment-selected IVG model as ivg_bringup."""

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
    initial_tool = LaunchConfiguration('initial_tool_id').perform(context)
    if gripper != 'none':
        raise RuntimeError(
            'Physical tool changing requires gripper:=none; '
            'use deployment parameters for fixed geometry')
    return [
        Node(package='tool_changer', executable='gripper_swap_worker_node',
             name='gripper_swap_worker', output='screen', parameters=[
                 config.robot_description, config.robot_description_semantic,
                 config.robot_description_kinematics,
                 {'initial_tool_id': initial_tool, 'io_simulated': mode == 'mock'}]),
        Node(package='tool_changer', executable='scene_attach_worker_node',
             name='scene_attach_worker', output='screen',
             parameters=[{'attach_frame': 'kuaihuan_Link', 'modeled_tool': gripper}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='none', choices=list(GRIPPERS)),
        DeclareLaunchArgument('hardware_mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('robot_ip', default_value='169.254.10.98'),
        DeclareLaunchArgument('initial_tool_id', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
