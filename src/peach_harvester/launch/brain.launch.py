"""
peach_harvester 大脑进程 launch（3b）：一进程三节点，lifecycle_manager 驱动.

参数装配：三份节点 yaml（按各自根键即节点名生效）+ grasp_standoffs 注入
（scene/recon overlay 并集）+ 工具档案注入并集 + require_managed_stack /
skip_reconstruction——全部按节点名分节写进 overlay ParameterFile。

brain 是 unnamed Node（exec 名 peach_harvester，勿 name= 重映射——三节点
图名在进程内自报，argv 不含节点名）。进程内三节点按各自名字装参：扁平
dict / 嵌套 ``peach_supervisor.ros__parameters`` 都打不进任何节点（2026-09-30
实测：bite 栈下重建/调度 tool.* 一直是 adaptive 基础值，P0 门红）——凡跨
节点注入一律写节点名分节的 yaml。独立入口 peach_executor.launch.py 节点名
即 peach_supervisor，扁平键可以。
"""

import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
import yaml


def _perform_param(value, context):
    """ParameterValue（档案惰性替换）/静态值 → 可写 yaml 的 python 值."""
    if isinstance(value, ParameterValue):
        return value.evaluate(context)
    return value


def launch_setup(context):
    """求值 skip/require 与档案注入，按节点名分节写 overlay yaml."""
    share = FindPackageShare('peach_harvester')
    scene_params = PathJoinSubstitution(
        [share, 'config', 'scene_perception.yaml'])
    recon_params = PathJoinSubstitution(
        [share, 'config', 'target_reconstruction.yaml'])
    supervisor_params = PathJoinSubstitution(
        [share, 'config', 'peach_supervisor.yaml'])
    tool_profile = LaunchConfiguration('tool_profile')
    # 字符串 launch 参数勿直接 ParameterValue(bool)：非空 'false' 会被当成 True。
    skip = LaunchConfiguration('skip_reconstruction').perform(context).lower() in (
        'true', '1', 'yes')
    require = LaunchConfiguration(
        'require_managed_stack').perform(context).lower() in (
            'true', '1', 'yes')
    # 三节点各自的覆盖：standoffs + 工具档案 + supervisor 开关/标签。
    # rcl 参数文件结构：<节点全名>: ros__parameters: {键: 值}。
    node_overrides = {
        'peach_scene_perception_node': {
            **scene_overlay(),
            **scene_tool_params(tool_profile),
        },
        'peach_target_reconstruction_node': {
            **reconstruction_overlay(),
            **reconstruction_tool_params(tool_profile),
        },
        'peach_supervisor': {
            **tool_profile_id_params(tool_profile),
            'require_managed_stack': require,
            'skip_reconstruction': skip,
        },
    }
    sections = {
        node: {'ros__parameters': {
            key: _perform_param(value, context)
            for key, value in overrides.items()}}
        for node, overrides in node_overrides.items()}
    overlay_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='peach_brain_overlay_', suffix='.yaml',
        delete=False, encoding='utf-8')
    yaml.safe_dump(sections, overlay_file, default_flow_style=False,
                   allow_unicode=True, sort_keys=True)
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
            'tool_profile', default_value='adaptive_shear_v1',
            choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1'],
            description='末端工具档案（URDF/许可内径/标签统一注入）'),
        OpaqueFunction(function=launch_setup),
    ])
