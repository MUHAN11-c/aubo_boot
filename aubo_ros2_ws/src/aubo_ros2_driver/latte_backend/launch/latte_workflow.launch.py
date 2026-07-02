"""
latte_workflow.launch.py — 启动咖啡拉花工作流编排节点

依赖 move_group 提供 MoveIt 运动能力，在 /joint_states 就绪后延迟启动。

用法:
  ros2 launch latte_backend latte_workflow.launch.py
"""

import time
import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _wait_for_readiness(timeout=30.0, poll_interval=1.0):
    start = time.time()
    while time.time() - start < timeout:
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True, text=True, timeout=3,
            )
            if "/joint_states" in result.stdout:
                return 0.0
        except Exception:
            pass
        time.sleep(poll_interval)
    return 10.0


def _launch_setup(context, *args, **kwargs):
    delay = _wait_for_readiness()

    moveit_config = (
        MoveItConfigsBuilder("aubo_e5", package_name="aubo_moveit_config")
        .robot_description(
            file_path="config/aubo_e5.urdf.xacro",
            mappings={"use_fake_hardware": "true"},
        )
        .robot_description_semantic(file_path="config/aubo_e5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    node = Node(
        package="latte_backend",
        executable="latte_workflow_node",
        name="latte_workflow_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"lwf_execute_latte": True},
        ],
    )

    if delay > 0:
        return [TimerAction(period=delay, actions=[node])]
    return [node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
