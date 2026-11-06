"""互斥启动五点力串口源或虚拟平面源；均不启动 Web 或运动工具."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """按 source 选择唯一力源，并装载对应节点使用的参数."""
    share = FindPackageShare('serial_force')
    default_params = PathJoinSubstitution([share, 'config', 'serial_force.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'force_params_file', default_value=default_params,
            description='5 点力参数文件；默认包内 config/serial_force.yaml'),
        DeclareLaunchArgument(
            'source', default_value='serial', choices=['serial', 'sim'],
            description='serial=真实串口驱动；sim=虚拟平面力源'),
        Node(
            package='serial_force',
            executable='serial_force_node',
            name='serial_force',
            output='screen',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('source'), "' == 'serial'"])),
            parameters=[
                ParameterFile(
                    LaunchConfiguration('force_params_file'),
                    allow_substs=True),
            ],
        ),
        Node(
            package='serial_force',
            executable='force_sim_node',
            name='serial_force',
            output='screen',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('source'), "' == 'sim'"])),
            parameters=[
                ParameterFile(
                    LaunchConfiguration('force_params_file'),
                    allow_substs=True),
            ],
        ),
    ])
