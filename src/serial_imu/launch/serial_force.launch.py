"""USB 串口 5 点力传感器：驱动，逐帧打印 kgf（无 RViz）."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """装载参数并起 5 点力驱动."""
    share = FindPackageShare('serial_imu')
    default_params = PathJoinSubstitution(
        [share, 'config', 'serial_force.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'force_params_file', default_value=default_params,
            description='5 点力参数文件；默认包内 config/serial_force.yaml'),
        Node(
            package='serial_imu',
            executable='serial_force_node',
            name='serial_force',
            output='screen',
            emulate_tty=True,
            parameters=[
                ParameterFile(
                    LaunchConfiguration('force_params_file'),
                    allow_substs=True),
            ],
        ),
    ])
