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
        FindPackageShare('peach_manipulation_skills'),
        'config', 'peach_manipulation_skills.yaml'])
    behavior_tree = PathJoinSubstitution([
        FindPackageShare('peach_manipulation_skills'),
        'config', 'behavior_tree.xml'])
    params_file = LaunchConfiguration('params_file')
    node = LifecycleNode(
        package='peach_manipulation_skills',
        executable='peach_manipulation_skills_node',
        name='peach_manipulation_skills_node',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(params_file, allow_substs=True),
            *_skills_moveit_params(),
            {'behavior_tree.xml': behavior_tree},
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
