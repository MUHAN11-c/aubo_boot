"""启动 peach 生命周期管理器（有序 configure/activate，不发 RunHarvest）."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造管理器节点；名单与超时在 config/lifecycle_manager.yaml."""
    params = PathJoinSubstitution([
        FindPackageShare('peach_task_executor'),
        'config', 'lifecycle_manager.yaml'])
    return LaunchDescription([
        Node(
            package='peach_task_executor',
            executable='peach_lifecycle_manager',
            name='peach_lifecycle_manager',
            output='screen',
            parameters=[ParameterFile(params, allow_substs=True)],
        ),
    ])
