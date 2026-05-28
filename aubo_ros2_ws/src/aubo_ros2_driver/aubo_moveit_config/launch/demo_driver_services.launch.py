"""
demo_driver_services.launch.py — 启动 Demo Driver 服务节点

依赖 move_group 提供的 action/services，在 MoveIt2 就绪后自动启动。
使用轮询 /joint_states 替换固定 TimerAction，避免无效等待。

用法:
  ros2 launch aubo_moveit_config demo_driver_services.launch.py
"""

import time

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder



def _wait_for_readiness(context, timeout=30.0, poll_interval=1.0):
    """轮询 /joint_states 话题，就绪后启动 Demo Driver 节点。

    启动脚本通过主动轮询控制步骤推进，此处的 TimerAction 仅作为
    最终安全网——如果轮询超时，节点仍会启动（最坏情况延迟 10s）。
    """
    import subprocess
    start = time.time()
    ready = False
    while time.time() - start < timeout:
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True, text=True, timeout=3,
            )
            if "/joint_states" in result.stdout:
                ready = True
                break
        except Exception:
            pass
        time.sleep(poll_interval)

    delay = 0.0 if ready else 10.0
    return delay


def _launch_setup(context, *args, **kwargs):
    delay = _wait_for_readiness(context, timeout=30.0)

    moveit_config = (
        MoveItConfigsBuilder("aubo_e5", package_name="aubo_moveit_config")
        .robot_description(
            file_path="config/aubo_e5.urdf.xacro",
            mappings={"use_fake_hardware": "true"},
        )
        .robot_description_semantic(file_path="config/aubo_e5.srdf")
        .to_moveit_configs()
    )

    common_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        {"moveit_velocity_scaling_factor": 0.1},
    ]

    nodes_cfg = [
        ("plan_trajectory_server_node", "plan_trajectory_server"),
        ("execute_trajectory_server_node", "execute_trajectory_server"),
        ("move_to_pose_server_node", "move_to_pose_server"),
        ("get_current_state_server_node", "get_current_state_server"),
        ("set_speed_factor_server_node", "set_speed_factor_server"),
        ("set_robot_pose_server_node", "set_robot_pose_server"),
        ("set_robot_enable_server_node", "set_robot_enable_server"),
        ("read_robot_io_server_node", "read_robot_io_server"),
    ]

    actions = []
    for executable, name in nodes_cfg:
        if delay > 0:
            actions.append(
                TimerAction(
                    period=delay,
                    actions=[
                        Node(
                            package="demo_driver",
                            executable=executable,
                            name=name,
                            output="screen",
                            parameters=common_parameters,
                        )
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="demo_driver",
                    executable=executable,
                    name=name,
                    output="screen",
                    parameters=common_parameters,
                )
            )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
