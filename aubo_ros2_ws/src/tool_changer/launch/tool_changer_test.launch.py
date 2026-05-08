"""
tool_changer_test.launch.py —— 末端快换测试

启动: aubo_new_driver + gripper_swap_worker (URDF 动态切换)

用法:
  ros2 launch tool_changer tool_changer_test.launch.py
  ros2 service call /scene_attach tool_changer_interface/srv/ChangeTool "{tool_id: gripper0}"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("aubo_moveit_config")

    # ── MoveIt 配置 ──
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

    # ── aubo_new_driver.launch.py ──
    pure_ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, "launch", "aubo_new_driver.launch.py")
        ]),
    )

    # ── gripper_swap_worker（仿真模式，URDF 切换）──
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
            },
        ],
    )

    return LaunchDescription([
        pure_ros2_launch,
        swap_worker,
    ])
