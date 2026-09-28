"""套袋桃运动能力：拍照、视点、MTC、工具与撤退."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from moveit_configs_utils import MoveItConfigsBuilder

from peach_harvester.vision.grasp_standoffs import manipulation_overlay
from peach_harvester.vision.tool_profiles import (
    tool_profile_body_params,
    tool_profile_d_inner_params,
    tool_profile_id_params,
)


def _skills_moveit_params(tool_profile: str):
    """
    Load aubo_e5_moveit_config via the official Builder.

    Call trajectory_execution before to_moveit_configs() to avoid warnings.
    The skills node does not inject controller maps (execution uses move_group).
    robot_description 经 mappings 透传 tool_profile（与 RSP/move_group 同值），
    纯 str 映射构建期即展开，故本函数须在拿到已求值的 tool_profile 后调用。
    """
    moveit_config = (
        MoveItConfigsBuilder(
            'aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(mappings={'tool_profile': tool_profile})
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp'],
            default_planning_pipeline='ompl')
        .trajectory_execution(
            file_path='config/controllers.yaml',
            moveit_manage_controllers=False)
        .to_moveit_configs())
    planning = dict(
        moveit_config.joint_limits.get('robot_description_planning', {}))
    if moveit_config.pilz_cartesian_limits:
        planning.update(
            moveit_config.pilz_cartesian_limits.get(
                'robot_description_planning', {}))
    return [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        {'robot_description_planning': planning},
    ]


def launch_setup(context):
    """tool_profile 须 perform 后再拼 Builder（xacro 映射构建期展开）."""
    tool_profile = LaunchConfiguration('tool_profile').perform(context)
    params_file = LaunchConfiguration('params_file')
    # 字符串 launch 参数勿直接 ParameterValue(bool)：非空 'false' 会被当成 True。
    require_robot_status = LaunchConfiguration(
        'require_robot_status').perform(context).lower() in ('true', '1', 'yes')
    allow_unrefined_geometry = LaunchConfiguration(
        'allow_unrefined_geometry').perform(context).lower() in (
            'true', '1', 'yes')
    node = LifecycleNode(
        package='peach_arm',
        executable='peach_arm',
        name='peach_arm',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(params_file, allow_substs=True),
            manipulation_overlay(),
            tool_profile_id_params(LaunchConfiguration('tool_profile')),
            tool_profile_d_inner_params(LaunchConfiguration('tool_profile')),
            tool_profile_body_params(LaunchConfiguration('tool_profile')),
            *_skills_moveit_params(tool_profile),
            {'execution.require_robot_status': require_robot_status},
            {'quality.allow_unrefined_geometry': allow_unrefined_geometry},
        ],
    )
    autostart = LaunchConfiguration('autostart')
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(autostart))
    return [activate, node, configure]


def generate_launch_description():
    """配置并激活技能节点；MoveIt 模型参数来自 moveit_config 包."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('peach_arm'),
                'config', 'peach_arm.yaml']),
            description='技能运行参数文件',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='true 时本 launch 自行 configure/activate'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1'],
            description='末端工具档案（URDF/标签随档案切换）'),
        DeclareLaunchArgument(
            'require_robot_status', default_value='true',
            description='安全门是否要求 /aubo_io_controller/robot_status；'
                        'harvest_system mock 传 false（bringup mock 不起 '
                        'aubo_io_controller）；真机须 true'),
        DeclareLaunchArgument(
            'allow_unrefined_geometry', default_value='false',
            description='true 时用场景观测几何代替重建精化；'
                        'harvest_system skip_reconstruction:=true 时传入 true'),
        OpaqueFunction(function=launch_setup),
    ])
