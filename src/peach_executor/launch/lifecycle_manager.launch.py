"""启动 peach 生命周期管理器（有序 configure/activate，不发 RunHarvest）."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造管理器节点；GPL 默认在 lifecycle_manager_parameters.yaml，本文件只加载运行覆盖."""
    params = PathJoinSubstitution([
        FindPackageShare('peach_executor'),
        'config', 'lifecycle_manager.yaml'])
    return LaunchDescription([
        Node(
            package='peach_executor',
            executable='peach_lifecycle_manager',
            name='peach_lifecycle_manager',
            output='screen',
            parameters=[ParameterFile(params, allow_substs=True)],
        ),
    ])
