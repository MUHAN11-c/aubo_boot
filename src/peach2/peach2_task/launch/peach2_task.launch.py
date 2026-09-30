# Copyright 2026 wjz
# SPDX-License-Identifier: BSD-3-Clause
"""
Start the peach2_task lifecycle node (unconfigured; the stack lifecycle manager drives it).

Never sends RunBatch: a batch is always started by the operator.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hardware_mode = LaunchConfiguration('hardware_mode')
    tool_profile = LaunchConfiguration('tool_profile')
    runs_dir = LaunchConfiguration('runs_dir')
    params_file = PathJoinSubstitution(
        [FindPackageShare('peach2_task'), 'config', 'peach2_task.param.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_mode', default_value='mock', choices=['mock', 'real'],
            description='mock: no aubo_io_controller, robot_status not required'),
        DeclareLaunchArgument(
            'tool_profile', default_value='',
            description='mounted tool id (shear_v1 | bite_shear_v1 | adaptive_shear_v1)'),
        DeclareLaunchArgument(
            'runs_dir', default_value='',
            description='ledger root; empty = $PEACH_RUNS_DIR, else <cwd>/runs'),
        LifecycleNode(
            package='peach2_task',
            executable='peach2_task_node',
            name='peach2_task',
            namespace='',
            output='screen',
            parameters=[
                params_file,
                {
                    'default_tool_id': ParameterValue(tool_profile, value_type=str),
                    'runs_dir': ParameterValue(runs_dir, value_type=str),
                    'safety.require_robot_status': ParameterValue(
                        PythonExpression(["'", hardware_mode, "' == 'real'"]),
                        value_type=bool),
                },
            ],
        ),
    ])
