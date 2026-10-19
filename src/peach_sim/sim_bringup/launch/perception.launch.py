"""
仿真感知栈：现行 peach 视觉源码的仿真副本，跑在统一机体相机流上.

前置：``sim.launch.py`` 已起（统一机体 + 腕载 rgbd + 相机桥 + JSB /joint_states）。
参数装配与 ``peach_harvester`` brain.launch 的感知两节相同：全量 yaml，
再注入 ``grasp_standoffs`` 与 ``tool_profile`` 档案。不含 supervisor /
scene_obstacles。测完 Ctrl+C 停栈并 pgrep 清残留（MUST）。
"""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from sim_perception.vision.grasp_standoffs import (
    reconstruction_overlay,
    scene_overlay,
)
from sim_perception.vision.tool_profiles import (
    reconstruction_tool_params,
    scene_tool_params,
)
import yaml


def _perform_param(value, context):
    """ParameterValue 在 launch 上下文求值，静态值原样写入."""
    if isinstance(value, ParameterValue):
        return value.evaluate(context)
    return value


def _setup(context, *args, **kwargs):
    perc_share = Path(get_package_share_directory('sim_perception'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    tool_profile = LaunchConfiguration('tool_profile')
    node_overrides = {
        'peach_scene_perception_node': {
            **scene_overlay(),
            **scene_tool_params(tool_profile),
        },
        'peach_target_reconstruction_node': {
            **reconstruction_overlay(),
            **reconstruction_tool_params(tool_profile),
        },
    }
    sections = {
        node: {'ros__parameters': {
            key: _perform_param(value, context)
            for key, value in overrides.items()}}
        for node, overrides in node_overrides.items()}
    overlay = tempfile.NamedTemporaryFile(
        mode='w', prefix='sim_perception_overlay_', suffix='.yaml',
        delete=False, encoding='utf-8')
    yaml.safe_dump(sections, overlay, default_flow_style=False,
                   allow_unicode=True, sort_keys=True)
    overlay.close()

    perception = LifecycleNode(
        package='sim_perception',
        executable='sim_scene_perception_node',
        name='peach_scene_perception_node',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(
                str(perc_share / 'config' / 'scene_perception.yaml'),
                allow_substs=True),
            ParameterFile(overlay.name, allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )
    reconstruction = LifecycleNode(
        package='sim_perception',
        executable='sim_target_reconstruction_node',
        name='peach_target_reconstruction_node',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(
                str(perc_share / 'config' / 'target_reconstruction.yaml'),
                allow_substs=True),
            ParameterFile(overlay.name, allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='peach_lifecycle_manager',
        output='screen',
        parameters=[{
            'node_names': [
                'peach_scene_perception_node',
                'peach_target_reconstruction_node',
            ],
            'autostart': True,
            'bond_timeout': 0.0,
            'use_sim_time': use_sim_time,
        }],
    )

    return [
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        perception,
        reconstruction,
        lifecycle_manager,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真时钟（与 sim.launch.py 同源 /clock）'),
        DeclareLaunchArgument(
            'tool_profile', default_value='bite_shear_v1',
            description='工具档案，与 sim.launch 的 xacro 同一参数'),
        OpaqueFunction(function=_setup),
    ])
