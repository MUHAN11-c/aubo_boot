"""套袋桃运动能力：拍照、视点、MTC、工具与撤退."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
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

from peach_perception.grasp_standoffs import manipulation_overlay


def _skills_moveit_params():
    """
    Load aubo_e5_moveit_config via the official Builder.

    Call trajectory_execution before to_moveit_configs() to avoid warnings.
    The skills node does not inject controller maps (execution uses move_group).
    """
    moveit_config = (
        MoveItConfigsBuilder(
            'aubo_e5', package_name='aubo_e5_moveit_config')
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
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


def generate_launch_description():
    """配置并激活技能节点；MoveIt 模型参数来自 moveit_config 包."""
    default_params = PathJoinSubstitution([
        FindPackageShare('peach_manipulation'),
        'config', 'peach_manipulation.yaml'])
    params_file = LaunchConfiguration('params_file')
    node = LifecycleNode(
        package='peach_manipulation',
        executable='peach_manipulation_node',
        name='peach_manipulation_node',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(params_file, allow_substs=True),
            manipulation_overlay(),
            *_skills_moveit_params(),
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='技能运行参数文件',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='true 时本 launch 自行 configure/activate'),
        activate,
        node,
        configure,
    ])
