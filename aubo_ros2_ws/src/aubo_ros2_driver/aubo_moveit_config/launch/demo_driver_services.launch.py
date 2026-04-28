"""
demo_driver_services.launch.py — 启动 Demo Driver 服务节点

这些节点依赖 move_group 提供的 action/services，需在 MoveIt2 启动后再运行。
使用 TimerAction 延迟 15 秒启动，确保 MoveIt2 完全初始化。

用法:
  ros2 launch aubo_moveit_config demo_driver_services.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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

    nodes = [
        ("plan_trajectory_server_node", "plan_trajectory_server"),
        ("execute_trajectory_server_node", "execute_trajectory_server"),
        ("get_current_state_server_node", "get_current_state_server"),
        ("set_speed_factor_server_node", "set_speed_factor_server"),
        ("set_robot_pose_server_node", "set_robot_pose_server"),
    ]

    actions = []
    for executable, name in nodes:
        actions.append(
            TimerAction(
                period=15.0,
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

    return LaunchDescription(actions)
