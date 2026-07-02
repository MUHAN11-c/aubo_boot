"""
tool_changer_test.launch.py — 末端快换与 PlanningScene 附着联调（仿真跳过 IO）

启动: aubo_new_driver + gripper_swap_worker + scene_attach_worker
（与生产环境 `gripper_swap_worker.launch.py` 一致包含 scene_attach，否则 /scene_attach 服务不存在喵~）

用法:
  ros2 launch tool_changer tool_changer_test.launch.py
  ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("aubo_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("aubo_e5", package_name="aubo_moveit_config")
        .robot_description(
            file_path="config/aubo_e5.urdf.xacro",
            mappings={"use_fake_hardware": "true"},
        )
        .robot_description_semantic(file_path="config/aubo_e5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    pure_ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, "launch", "aubo_new_driver.launch.py")
        ]),
    )

    # 仿真：跳过真实 IO；Collision 附着由 scene_attach_worker（非 URDF 动态切换）喵~
    swap_worker = Node(
        package="tool_changer",
        executable="gripper_swap_worker_node",
        name="gripper_swap_worker",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "simulation_skip_io": True,
                "joint_velocity_scaling": 1.0,
                "joint_acceleration_scaling": 1.0,
                "home_velocity_scaling": 1.0,
                "home_acceleration_scaling": 1.0,
                "initial_tool_id": "gripper0",
            },
        ],
    )

    scene_attach = Node(
        package="tool_changer",
        executable="scene_attach_worker_node",
        name="scene_attach_worker",
        output="screen",
    )

    return LaunchDescription([
        pure_ros2_launch,
        swap_worker,
        scene_attach,
    ])
