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
# 参数装载：MoveItConfigsBuilder（与 MoveIt 2 官方 move_group launch 同链）。
# .setup_assistant 提供 URDF/SRDF 定位；ompl/pilz 走 config/*_planning.yaml。
#
# 启动内容：
#   move_group  规划/执行（controllers_file：real→controllers.yaml 透传，
#               mock→controllers_mock.yaml 标准 JTC，由 bringup 按 hardware_mode 透传）
#   rviz2       与 move_group 同一套 robot_description* / 管线（MotionPlanning
#               面板 default_*_scaling_factor 来自 robot_description_planning）
#   robot_state_publisher + joint_state_publisher_gui
#               仅 standalone_state_publishers:=true（脱离 bringup 单跑）；
#               经 bringup 集成时传 false，避免 TF 双发
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# MTC Task::execute() 经 move_group 这些 capability 下发完整 solution。
# 不是管线 yaml 字段，须作为 move_group 顶层 capabilities（空格分隔）。
_MOVE_GROUP_CAPABILITIES = (
    'pilz_industrial_motion_planner/MoveGroupSequenceAction '
    'pilz_industrial_motion_planner/MoveGroupSequenceService '
    'move_group/ExecuteTaskSolutionCapability')


def _moveit_configs(controllers_file: str):
    """按官方 Builder 装载 URDF/SRDF/IK/管线/控制器映射/Pilz 笛卡尔限."""
    return (
        MoveItConfigsBuilder(
            'aubo_e5', package_name='aubo_e5_moveit_config')
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
            default_planning_pipeline='ompl')
        .trajectory_execution(
            file_path='config/' + controllers_file,
            moveit_manage_controllers=False)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True)
        .to_moveit_configs())


def launch_setup(context):
    """controllers_file / standalone 须 perform 后再拼 Builder 与可选 RSP."""
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    standalone = (
        LaunchConfiguration('standalone_state_publishers')
        .perform(context).lower() == 'true')
    moveit_config = _moveit_configs(controllers_file)
    params = moveit_config.to_dict()
    rviz_config = str(moveit_config.package_path / 'rviz' / 'moveit.rviz')
    nodes = [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[params, {'capabilities': _MOVE_GROUP_CAPABILITIES}]),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[params]),
    ]
    if standalone:
        nodes += [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[moveit_config.robot_description],
                output='screen'),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
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
            description='config/ 下的 MoveIt 控制器映射（mock 用 controllers_mock.yaml）'),
        OpaqueFunction(function=launch_setup),
    ])
