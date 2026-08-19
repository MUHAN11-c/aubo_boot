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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import xacro
import yaml


def generate_launch_description():
    default_params = Path(
        get_package_share_directory('peach_manipulation_skills')
    ) / 'config' / 'approach_grasp.yaml'
    approach_params_file = LaunchConfiguration('approach_params_file')
    description_path = Path(
        get_package_share_directory('aubo_description')
    ) / 'urdf' / 'aubo_e5.urdf.xacro'
    moveit_share = Path(get_package_share_directory('aubo_e5_moveit_config'))
    robot_description = {
        'robot_description': xacro.process_file(str(description_path)).toxml(),
    }
    robot_description_semantic = {
        'robot_description_semantic': (
            moveit_share / 'config' / 'aubo_e5.srdf'
        ).read_text(encoding='utf-8'),
    }
    with (moveit_share / 'config' / 'kinematics.yaml').open(
            encoding='utf-8') as stream:
        robot_description_kinematics = {
            'robot_description_kinematics': yaml.safe_load(stream),
        }
    with (moveit_share / 'config' / 'joint_limits.yaml').open(
            encoding='utf-8') as stream:
        robot_description_planning = {
            'robot_description_planning': yaml.safe_load(stream),
        }
    with (moveit_share / 'config' / 'ompl_planning.yaml').open(
            encoding='utf-8') as stream:
        ompl_pipeline = yaml.safe_load(stream)
    ompl_pipeline.update({
        'planning_plugins': ['ompl_interface/OMPLPlanner'],
        'request_adapters': [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision',
        ],
        'response_adapters': [
            'default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/AddRuckigTrajectorySmoothing',
            'default_planning_response_adapters/ValidateSolution',
        ],
        'start_state_max_bounds_error': 0.1,
        'totg': {'resample_dt': 0.01},
    })
    mtc_pipeline = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_pipeline,
    }
    behavior_tree = {
        'behavior_tree.xml': str(
            Path(get_package_share_directory('peach_manipulation_skills')) /
            'config' / 'harvest_tree.xml'),
    }
    node = LifecycleNode(
        package='peach_manipulation_skills',
        executable='approach_grasp_node',
        name='peach_manipulation_skills_node',
        namespace='',
        output='screen',
        parameters=[
            approach_params_file,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            mtc_pipeline,
            behavior_tree,
        ],
    )
    # 生命周期自动转换（A8）：进程就绪后 configure（参数验证+资源分配，
    # 失败则停在 Unconfigured 报错），到 Inactive 后 activate 开放运动输出权限。
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
    ))
    return LaunchDescription([
        DeclareLaunchArgument(
            'approach_params_file',
            default_value=str(default_params),
            description='主动视觉靠近与抓取参数文件',
        ),
        activate,
        node,
        configure,
    ])
