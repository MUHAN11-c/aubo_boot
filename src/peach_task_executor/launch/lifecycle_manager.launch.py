"""启动 peach 生命周期管理器（有序 configure/activate，不发 RunHarvest）."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """构造管理器节点."""
    return LaunchDescription([
        Node(
            package='peach_task_executor',
            executable='peach_lifecycle_manager',
            name='peach_lifecycle_manager',
            output='screen',
        ),
    ])
