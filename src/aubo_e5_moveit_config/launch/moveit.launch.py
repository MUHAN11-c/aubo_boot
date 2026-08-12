# Copyright 2026, aubo_e5_ros2_ws authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# moveit.launch.py —— MoveIt 整体启动入口（真机/仿真共用），本包唯一的 launch，
# aubo_e5_bringup 经本文件集成 MoveIt。
#
# 启动内容：
#   move_group               规划/执行服务（控制器映射由 controllers_file 选择：
#                            sim/real -> controllers.yaml（passthrough），
#                            mock    -> controllers_mock.yaml（标准 JTC），
#                            由 bringup 按 hardware_mode 透传）
#   rviz2                    带完整 MoveIt 参数（必须与 move_group 拿同一份
#                            robot_description*，含 robot_description_planning：
#                            MotionPlanning 面板 Velocity/Accel 滑条初值从
#                            robot_description_planning.default_*_scaling_factor
#                            读取，缺参数会回退硬编码 0.1）
#   robot_state_publisher + joint_state_publisher_gui
#                            仅 standalone_state_publishers:=true（脱离 bringup
#                            单跑）时启动；经 bringup 集成时传 false，rsp 由
#                            bringup 自带，不要重复起（TF 双发）
#   static_transform_publisher
#                            末端 TCP 静态 TF wrist3_Link→tcp，数值见
#                            config/tcp.yaml（URDF 未内建 tcp link，在此补发）
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


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
    # controllers_file 是 launch 参数，必须在 OpaqueFunction 里 perform 后才能
    # 拼路径读 yaml。
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    standalone = (
        LaunchConfiguration('standalone_state_publishers')
        .perform(context).lower() == 'true')

    desc = {'robot_description': xacro_text('aubo_description', 'urdf/aubo_e5.urdf.xacro')}
    semantic = {'robot_description_semantic': text('aubo_e5_moveit_config', 'config/aubo_e5.srdf')}
    kin = {'robot_description_kinematics': load('aubo_e5_moveit_config', 'config/kinematics.yaml')}
    # 末端 TCP 静态 TF（wrist3_Link→tcp，数值权威源 config/tcp.yaml）。
    # URDF 未内建 tcp link，故在此补发；若未来 URDF 内建 tcp，必须删除本节点避免双发。
    tcp = load('aubo_e5_moveit_config', 'config/tcp.yaml')['tcp']
    limits = {'robot_description_planning':
              load('aubo_e5_moveit_config', 'config/joint_limits.yaml')}
    rviz_config = os.path.join(
        get_package_share_directory('aubo_e5_moveit_config'), 'rviz', 'moveit.rviz')
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
        # MTC 的 Task::execute() 通过此 capability 将完整 solution 交给 move_group。
        'capabilities': ('pilz_industrial_motion_planner/MoveGroupSequenceAction '
                         'pilz_industrial_motion_planner/MoveGroupSequenceService '
                         'move_group/ExecuteTaskSolutionCapability')}
    controllers = {
        'moveit_simple_controller_manager': load(
            'aubo_e5_moveit_config', 'config/' + controllers_file),
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    trajectory_execution = {
        'moveit_manage_controllers': False,
        # passthrough 蓝本值（aubo_boot）：整段轨迹一次下发、硬件侧自行插补执行，
        # 执行耗时与 RIB 流控相关，余量须比流式 JTC 宽。
        'trajectory_execution.allowed_execution_duration_scaling': 5.0,
        'trajectory_execution.allowed_goal_duration_margin': 10.0,
        'trajectory_execution.allowed_start_tolerance': 0.15}
    monitor = {'publish_planning_scene': True, 'publish_geometry_updates': True,
               'publish_state_updates': True, 'publish_transforms_updates': True}
    nodes = [
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[desc, semantic, kin, limits, pipelines, controllers,
                         trajectory_execution, monitor]),
        # rviz 与 move_group 拿同一份 desc/semantic/kin/limits/pipelines（见文件头注释）
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_config], parameters=[desc, semantic, kin, limits, pipelines]),
        # 新版参数形式（Jazzy 已废弃位置参数，会刷 deprecated 告警）
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(tcp['xyz'][0]), '--y', str(tcp['xyz'][1]),
                        '--z', str(tcp['xyz'][2]),
                        '--yaw', str(tcp['rpy'][0]), '--pitch', str(tcp['rpy'][1]),
                        '--roll', str(tcp['rpy'][2]),
                        '--frame-id', tcp['parent_frame'],
                        '--child-frame-id', tcp['child_frame']]),
    ]
    if standalone:
        nodes += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[desc], output='screen'),
            Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                 output='screen'),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'standalone_state_publishers', default_value='true',
            description='true=脱离 bringup 单跑（自带 rsp + joint_state_publisher_gui）；'
                        'false=经 bringup 集成（rsp 由 bringup 提供）'),
        DeclareLaunchArgument(
            'controllers_file', default_value='controllers.yaml',
            description='config/ 下的 MoveIt 控制器映射文件（mock 模式用 controllers_mock.yaml）'),
        OpaqueFunction(function=launch_setup),
    ])
