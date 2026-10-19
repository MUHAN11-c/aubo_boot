"""
统一机体 MoveIt 前端（P4）：规划帧 = platform_base_link.

与 aubo_e5_moveit_config/moveit.launch.py 的唯一差别：robot_description 用
``sim_description/harvester_unified.urdf.xacro``（履带平台 + 臂，根
platform_base_link），使 move_group 规划帧落在 map/odom 主 TF 树内
（aubo 原版 URDF 根为 world，仿真里无 world 帧，规划场景无法投影到 map，
RViz PlanningScene/Trajectory 显示报红、笛卡尔目标帧变换失败）。

SRDF/IK/Pilz/控制器映射全部复用 aubo_e5_moveit_config（chain base_link→tcp
在统一机体中同名存在）。JTC spawner 不在此（sim.launch 统一延迟 spawn）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from moveit_configs_utils import MoveItConfigsBuilder

from sim_worlds.params import params_from_dict, xacro_mappings
import yaml

# 与 aubo moveit.launch 同口径：MTC/Pilz sequence 能力
_MOVE_GROUP_CAPABILITIES = (
    'pilz_industrial_motion_planner/MoveGroupSequenceAction '
    'pilz_industrial_motion_planner/MoveGroupSequenceService '
    'move_group/ExecuteTaskSolutionCapability')


def launch_setup(context):
    tool_profile = LaunchConfiguration('tool_profile').perform(context)
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    scene_path = Path(LaunchConfiguration('scene_params').perform(context))

    raw = yaml.safe_load(scene_path.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    if problems:
        raise RuntimeError(
            f'{scene_path} 场景参数不合法：' + '；'.join(problems))

    desc_share = Path(get_package_share_directory('sim_description'))
    unified_xacro = str(desc_share / 'urdf' / 'harvester_unified.urdf.xacro')
    # 映射单源：sim.launch 的 RSP 展开与这里共用 sim_worlds.params.xacro_mappings
    # （两处逐键同值，否则 move_group 模型与 TF 分叉）
    mappings = xacro_mappings(params, tool_profile)

    moveit_config = (
        MoveItConfigsBuilder(
            'aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(file_path=unified_xacro, mappings=mappings)
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp'],
            default_planning_pipeline='pilz_industrial_motion_planner')
        .trajectory_execution(
            file_path='config/' + controllers_file,
            moveit_manage_controllers=False)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True)
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs())

    nodes = [
        # 单独起本入口也统一 sim 时钟（include 时由 sim.launch 透传覆盖）
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[moveit_config.to_dict(),
                        {'capabilities': _MOVE_GROUP_CAPABILITIES}]),
    ]
    rviz_on = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1')
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if rviz_on:
        # 带 MoveIt 参数（robot_description/_semantic 等）起 RViz——
        # moveit_rviz_plugin 显示层需要 SRDF；裸起会报
        # "Could not parse the SRDF XML / did not receive robot_description_semantic"
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                # 空 rviz_config = 仅 MoveIt 参数裸起
                arguments=['-d', rviz_config] if rviz_config else [],
                parameters=[moveit_config.to_dict()]))
    return nodes


def generate_launch_description() -> LaunchDescription:
    worlds_share = Path(get_package_share_directory('sim_worlds'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真时钟（与 sim.launch 同源 /clock；单独起本入口勿关）'),
        DeclareLaunchArgument(
            'scene_params',
            default_value=str(worlds_share / 'config' / 'orchard.yaml'),
            description='场景参数 yaml（与 sim.launch 同源同值）'),
        DeclareLaunchArgument(
            'tool_profile', default_value='bite_shear_v1',
            description='末端工具档案（须与 sim.launch 同值）'),
        DeclareLaunchArgument(
            'controllers_file', default_value='controllers_mock.yaml',
            description='aubo_e5_moveit_config/config/ 下的控制器映射'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='随 move_group 起 RViz（带 MoveIt 参数，moveit 显示层需要）'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='RViz 配置路径（空 = 仅 MoveIt 参数裸起；'
                        'harvest 传 sim_harvest.rviz 全景视图）'),
        OpaqueFunction(function=launch_setup),
    ])
