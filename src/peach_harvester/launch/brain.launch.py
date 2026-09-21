"""
peach_harvester 大脑进程 launch（3b）：一进程三节点，lifecycle_manager 驱动.

参数装配：三份节点 yaml（按各自根键即节点名生效）+ grasp_standoffs 注入
（scene/recon overlay 并集）+ 工具档案注入并集 + require_managed_stack。
并集注入对未声明键由各节点按 overrides 静默忽略（rcl 语义），无冲突键。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from peach_harvester.vision.grasp_standoffs import (
    reconstruction_overlay,
    scene_overlay,
)
from peach_harvester.vision.tool_profiles import (
    reconstruction_tool_params,
    scene_tool_params,
    tool_profile_id_params,
)


def generate_launch_description():
    """大脑单进程入口：三节点 + lifecycle_manager 托管."""
    share = FindPackageShare('peach_harvester')
    scene_params = PathJoinSubstitution(
        [share, 'config', 'scene_perception.yaml'])
    recon_params = PathJoinSubstitution(
        [share, 'config', 'target_reconstruction.yaml'])
    supervisor_params = PathJoinSubstitution(
        [share, 'config', 'peach_supervisor.yaml'])
    tool_profile = LaunchConfiguration('tool_profile')
    # 注入并集：未声明键按 overrides 被各节点忽略（rcl 语义）
    overlays = {
        **scene_overlay(),
        **reconstruction_overlay(),
        **scene_tool_params(tool_profile),
        **reconstruction_tool_params(tool_profile),
        **tool_profile_id_params(tool_profile),
    }
    brain = Node(
        package='peach_harvester',
        executable='peach_harvester',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(scene_params, allow_substs=True),
            ParameterFile(recon_params, allow_substs=True),
            ParameterFile(supervisor_params, allow_substs=True),
            overlays,
            {
                'peach_supervisor': {
                    'ros__parameters': {
                        'require_managed_stack': ParameterValue(
                            LaunchConfiguration('require_managed_stack'),
                            value_type=bool),
                        'skip_reconstruction': ParameterValue(
                            LaunchConfiguration('skip_reconstruction'),
                            value_type=bool),
                    }
                }
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'require_managed_stack', default_value='true',
            description='true 时须等生命周期管理器就绪旗标才接受 RunHarvest'),
        DeclareLaunchArgument(
            'skip_reconstruction', default_value='false',
            description='true 时 DISPATCH 跳过 Build/补视，用场景观测进接触'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_cylinder_v1',
            choices=['hollow_cylinder_v1', 'adaptive_cylinder_v1'],
            description='末端工具档案（URDF/许可内径/标签统一注入）'),
        brain,
    ])
