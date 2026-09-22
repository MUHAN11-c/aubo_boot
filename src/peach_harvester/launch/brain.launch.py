"""
peach_harvester 大脑进程 launch（3b）：一进程三节点，lifecycle_manager 驱动.

参数装配：三份节点 yaml（按各自根键即节点名生效）+ grasp_standoffs 注入
（scene/recon overlay 并集）+ 工具档案注入并集 + require_managed_stack /
skip_reconstruction 的 supervisor 键 ParameterFile overlay.

brain 是 unnamed Node（exec 名 peach_harvester，勿 name= 重映射——三节点
图名在进程内自报，argv 不含节点名）。嵌套 dict
``peach_supervisor.ros__parameters`` 打不进进程内的 peach_supervisor
（launch 把它绑到 Node 默认名）。独立入口 peach_executor.launch.py
节点名即 peach_supervisor，扁平键可以。
"""

import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
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


def launch_setup(context):
    """求值 skip/require 后写成 peach_supervisor 键 yaml，供进程内节点按名装入."""
    share = FindPackageShare('peach_harvester')
    scene_params = PathJoinSubstitution(
        [share, 'config', 'scene_perception.yaml'])
    recon_params = PathJoinSubstitution(
        [share, 'config', 'target_reconstruction.yaml'])
    supervisor_params = PathJoinSubstitution(
        [share, 'config', 'peach_supervisor.yaml'])
    tool_profile = LaunchConfiguration('tool_profile')
    overlays = {
        **scene_overlay(),
        **reconstruction_overlay(),
        **scene_tool_params(tool_profile),
        **reconstruction_tool_params(tool_profile),
        **tool_profile_id_params(tool_profile),
    }
    # 字符串 launch 参数勿直接 ParameterValue(bool)：非空 'false' 会被当成 True。
    skip = LaunchConfiguration('skip_reconstruction').perform(context).lower() in (
        'true', '1', 'yes')
    require = LaunchConfiguration(
        'require_managed_stack').perform(context).lower() in (
            'true', '1', 'yes')
    overlay_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='peach_supervisor_overlay_', suffix='.yaml',
        delete=False, encoding='utf-8')
    overlay_file.write(
        'peach_supervisor:\n'
        '  ros__parameters:\n'
        f'    require_managed_stack: {str(require).lower()}\n'
        f'    skip_reconstruction: {str(skip).lower()}\n')
    overlay_file.close()
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
            ParameterFile(overlay_file.name),
        ],
    )
    return [brain]


def generate_launch_description():
    """大脑单进程入口：三节点 + lifecycle_manager 托管."""
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
        OpaqueFunction(function=launch_setup),
    ])
