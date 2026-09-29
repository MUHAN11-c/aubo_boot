"""
独立起 GPU 枝/叶分割节点；不进 harvest_system / lifecycle 名单.

2026-09-29 观测性轮修复：launch 不再发 configure/activate 事件——节点
main() 的 ensure_active() 是唯一转换源。此前 launch EmitEvent 与 main 自激活
确定性互杀（对已 active 节点再发 activate 会打死进程，09-21 实测），导致
launch 路径不可用、只能 ros2 run 直起；现两条路径均可用。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """起 peach_vegetation；生命周期转换由节点 main() 自激活完成."""
    config = PathJoinSubstitution([
        FindPackageShare('peach_vegetation'),
        'config', 'vegetation.yaml'])
    node = LifecycleNode(
        package='peach_vegetation',
        executable='peach_vegetation',
        name='peach_vegetation',
        namespace='',
        parameters=[ParameterFile(config, allow_substs=True)],
        remappings=[('image', LaunchConfiguration('image'))],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'image', default_value='/camera/color/image_raw',
            description='彩色图话题（remap 到节点相对名 image）'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='兼容保留参数：生命周期转换恒由节点 main() 自激活'
                        '（ensure_active），launch 不发转换事件（09-21 互杀'
                        '修复，09-29 观测性轮）'),
        node,
    ])
