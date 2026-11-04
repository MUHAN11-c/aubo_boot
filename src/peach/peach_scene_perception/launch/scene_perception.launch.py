"""Scene perception lifecycle node: configure/activate then process RGB-D."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from peach_common.grasp_standoffs import scene_overlay
from peach_common.tool_profiles import scene_tool_params


def _setup(context):
    """Build parameters after launch substitutions are available."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    params_file = value('params_file') or PathJoinSubstitution([
        FindPackageShare('peach_scene_perception'),
        'config', 'scene_perception.yaml']).perform(context)
    parameters = [
        ParameterFile(params_file, allow_substs=True),
        scene_overlay(),
        scene_tool_params(LaunchConfiguration('tool_profile')),
        {'use_sim_time': value('use_sim_time').lower() in ('1', 'true', 'yes')},
    ]
    for name in ('params_overlay', 'task_params_file'):
        path = value(name).strip()
        if path:
            parameters.append(ParameterFile(path, allow_substs=True))
    # 官方 18 图案前端最后装入，避免 orin/user overlay 把 max_frame_age 打回 0.50。
    if value('camera_frontend').strip().lower() == 'percipio':
        parameters.append(ParameterFile(
            PathJoinSubstitution([
                FindPackageShare('peach_scene_perception'),
                'config', 'percipio_frontend.yaml']).perform(context),
            allow_substs=True))

    node = LifecycleNode(
        package='peach_scene_perception',
        executable='peach_scene_perception_node',
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'},
        name='peach_scene_perception_node',
        namespace=value('namespace'),
        parameters=parameters,
        output='screen',
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(LaunchConfiguration('autostart')))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(LaunchConfiguration('autostart')))
    return [activate, node, configure]


def generate_launch_description():
    """Declare standard launch arguments and construct the lifecycle node."""
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([
                FindPackageShare('peach_scene_perception'),
                'config', 'scene_perception.yaml'])),
        DeclareLaunchArgument('params_overlay', default_value=''),
        DeclareLaunchArgument('task_params_file', default_value=''),
        DeclareLaunchArgument(
            'camera_frontend', default_value='stereo',
            choices=['percipio', 'stereo', 'replay'],
            description='透传 harvest_system 相机前端；percipio 时装入 1 fps 年龄门 overlay'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1']),
        OpaqueFunction(function=_setup),
    ])
