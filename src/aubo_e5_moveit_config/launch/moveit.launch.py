import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load(package, path):
    with open(os.path.join(get_package_share_directory(package), path), encoding='utf-8') as f:
        return yaml.safe_load(f)

def text(package, path):
    with open(os.path.join(get_package_share_directory(package), path), encoding='utf-8') as f:
        return f.read()

def xacro_text(package, path):
    filename = os.path.join(get_package_share_directory(package), path)
    return xacro.process_file(filename).toxml()

def launch_setup(context):
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    desc = {'robot_description': xacro_text('aubo_description', 'urdf/aubo_e5.urdf.xacro')}
    semantic = {'robot_description_semantic': text('aubo_e5_moveit_config', 'config/aubo_e5.srdf')}
    kin = {'robot_description_kinematics': load('aubo_e5_moveit_config', 'config/kinematics.yaml')}
    limits = {'robot_description_planning': load('aubo_e5_moveit_config', 'config/joint_limits.yaml')}
    rviz_config = os.path.join(get_package_share_directory('aubo_e5_moveit_config'), 'rviz', 'moveit.rviz')
    # 双管线（move_group 2.12 在节点顶层读取 planning_pipelines 列表 +
    # default_planning_pipeline，每条管线的参数在 <pipeline_name>.* 命名空间下，
    # 与 MoveItConfigsBuilder 产物一致——注意不要再嵌套进 move_group 键，
    # 否则 move_group 读不到列表会回退到 legacy 单管线命名空间）：
    # - ompl（默认）：TOTG 之后接 Ruckig jerk 平滑（response_adapters 按列表顺序执行，
    #   Ruckig 要求输入已是时间参数化轨迹，必须排在 AddTimeOptimalParameterization 之后）；
    #   totg.resample_dt 0.1→0.01，输出路点更密，供硬件侧五次重采样取更平滑的段边界。
    # - pilz_industrial_motion_planner：确定性 PTP/LIN/CIRC，两点直达运动用。
    ompl_pipeline = {
        'planning_plugins': ['ompl_interface/OMPLPlanner'],
        'request_adapters': [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision'],
        'response_adapters': [
            'default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/AddRuckigTrajectorySmoothing',
            'default_planning_response_adapters/ValidateSolution',
            'default_planning_response_adapters/DisplayMotionPath'],
        'start_state_max_bounds_error': 0.1,
        # TOTG 适配器经 generate_parameter_library 读取 <管线命名空间>.totg.*
        # （default_response_adapter_parameters 只是 C++ 命名空间，不进参数名）：
        # resample_dt 0.1→0.01，输出路点更密，利于硬件侧重采样平滑。
        'totg': {'resample_dt': 0.01}}
    ompl_pipeline.update(load('aubo_e5_moveit_config', 'config/ompl_planning.yaml'))
    pipelines = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_pipeline,
        'pilz_industrial_motion_planner': load(
            'aubo_e5_moveit_config', 'config/pilz_industrial_motion_planner_planning.yaml'),
        'capabilities': ('pilz_industrial_motion_planner/MoveGroupSequenceAction '
                         'pilz_industrial_motion_planner/MoveGroupSequenceService')}
    # controllers_file 由 bringup 按 hardware_mode 选择：sim/real 用
    # controllers.yaml（passthrough 控制器），mock 用 controllers_mock.yaml（标准 JTC）。
    controllers = {'moveit_simple_controller_manager': load('aubo_e5_moveit_config', 'config/' + controllers_file),
                   'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    trajectory_execution = {
        'moveit_manage_controllers': False,
        # passthrough 蓝本值（aubo_boot）：整段轨迹一次下发、硬件侧自行插补执行，
        # 执行耗时与 RIB 流控相关，余量须比流式 JTC 宽。
        'trajectory_execution.allowed_execution_duration_scaling': 5.0,
        'trajectory_execution.allowed_goal_duration_margin': 10.0,
        'trajectory_execution.allowed_start_tolerance': 0.15}
    monitor = {'publish_planning_scene': True, 'publish_geometry_updates': True,
               'publish_state_updates': True, 'publish_transforms_updates': True}
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[desc], output='screen', condition=IfCondition(LaunchConfiguration('standalone_state_publishers'))),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen', condition=IfCondition(LaunchConfiguration('standalone_state_publishers'))),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=[desc, semantic, kin, limits, pipelines, controllers, trajectory_execution, monitor]),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config], parameters=[desc, semantic, kin, pipelines]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'standalone_state_publishers', default_value='true',
            description='Start robot_state_publisher and joint_state_publisher_gui here'),
        DeclareLaunchArgument(
            'controllers_file', default_value='controllers.yaml',
            description='config/ 下的 MoveIt 控制器映射文件（mock 模式用 controllers_mock.yaml）'),
        OpaqueFunction(function=launch_setup),
    ])
