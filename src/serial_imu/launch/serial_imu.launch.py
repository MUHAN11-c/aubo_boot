"""USB 串口 IMU：驱动 + 可选 RViz（TF/坐标轴；Imu 插件需本机已装）."""

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def _warn_missing_imu_plugin(context, *args, **kwargs):
    """缺 rviz_imu_plugin 时打印 apt 安装命令，不阻止 launch."""
    del context, args, kwargs
    try:
        get_package_share_directory('rviz_imu_plugin')
        return []
    except PackageNotFoundError:
        return [LogInfo(msg=(
            '未找到 rviz_imu_plugin。RViz 仍可用 TF/Axes 看 imu_link。 '
            '手动安装: sudo apt install ros-jazzy-imu-tools ；'
            '装完后 RViz Add → Imu，话题 /imu/data，Reliability=Best Effort'
        ))]


def generate_launch_description():
    """装载参数、起驱动；use_rviz 时开 RViz2."""
    share = FindPackageShare('serial_imu')
    default_params = PathJoinSubstitution([share, 'config', 'serial_imu.yaml'])
    default_rviz = PathJoinSubstitution([share, 'rviz', 'serial_imu.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_imu_params_file', default_value=default_params,
            description='IMU 参数文件；默认包内 config/serial_imu.yaml'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='true 时启动 RViz2（TF / imu_attitude / Imu 插件）'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz 配置；默认包内 rviz/serial_imu.rviz'),
        DeclareLaunchArgument(
            'tf_parent_frame', default_value='world',
            description='imu_link / imu_attitude 的父坐标系；接手臂时用 base_link'),
        OpaqueFunction(function=_warn_missing_imu_plugin),
        Node(
            package='serial_imu',
            executable='serial_imu_node',
            name='serial_imu',
            output='screen',
            parameters=[
                ParameterFile(
                    LaunchConfiguration('serial_imu_params_file'),
                    allow_substs=True),
                {'tf_parent_frame': LaunchConfiguration('tf_parent_frame')},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
