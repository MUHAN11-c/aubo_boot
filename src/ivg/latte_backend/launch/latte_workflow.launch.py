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
    return [
        Node(package='latte_backend', executable='latte_workflow_node',
             name='latte_workflow_node', output='screen', parameters=[
                 config.robot_description, config.robot_description_semantic,
                 config.robot_description_kinematics,
                 {'io_simulated': mode == 'mock', 'lwf_execute_latte': True}]),
        Node(package='latte_backend', executable='latte_io_node',
             name='latte_io_node', parameters=[{'io_simulated': mode == 'mock'}],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='none', choices=list(GRIPPERS)),
        DeclareLaunchArgument('hardware_mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('robot_ip', default_value='169.254.10.98'),

        OpaqueFunction(function=launch_setup),
    ])
