"""
AUBO 新框架驱动 launch — 纯新架构 (不依赖旧 driver/simulator/trajectory_action)

替代 aubo_moveit_pure_ros2.launch.py 中的机械臂部分:
  ❌ aubo_driver_ros2 (旧驱动)
  ❌ aubo_ros2_trajectory_action (旧 Action)
  ❌ aubo_robot_simulator_ros2 (Python 插值)
  ✅ joint_trajectory_controller (C++ 插值 + TCP2CAN)
  ✅ aubo_dashboard_node (SDK 全功能)
  ✅ aubo_state_broadcaster (回调驱动)

用法:
  ros2 launch aubo_moveit_config aubo_new_driver.launch.py
  ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_SERVER_HOST = "169.254.10.98"


def launch_setup(context, *args, **kwargs):
    try:
        motion_command_hz = float(LaunchConfiguration("motion_command_hz").perform(context))
    except ValueError:
        motion_command_hz = 200.0
    if motion_command_hz < 1.0:
        motion_command_hz = 200.0
    trajectory_sample_duration = 1.0 / motion_command_hz

    moveit_config = (
        MoveItConfigsBuilder("aubo_e5", package_name="aubo_moveit_config")
        .robot_description(file_path="config/aubo_e5.urdf.xacro",
                           mappings={"use_fake_hardware": "true"})
        .robot_description_semantic(file_path="config/aubo_e5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    pkg_share = get_package_share_directory("aubo_moveit_config")
    robot_description = moveit_config.robot_description

    ompl_config = moveit_config.planning_pipelines.get("ompl", {})
    ompl_planning = {"move_group": ompl_config.copy()}
    ompl_planning["move_group"].update({
        "sample_duration": trajectory_sample_duration,
        "planning_time": 15.0,
        "max_planning_attempts": 10,
        "resample_dt": 0.05,
    })
    trajectory_execution = moveit_config.trajectory_execution.copy()
    trajectory_execution.update({
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    })

    server_host = LaunchConfiguration("server_host")

    # ---- 新框架节点 ----

    dashboard_node = Node(
        package="aubo_driver_ros2",
        executable="aubo_dashboard_node",
        name="aubo_dashboard",
        output="screen",
        parameters=[{"server_host": server_host, "server_port": 8899, "collision_class": 6}],
    )

    state_bc_node = Node(
        package="aubo_driver_ros2",
        executable="aubo_state_broadcaster",
        name="aubo_state_broadcaster",
        output="screen",
        parameters=[{"server_host": server_host, "server_port": 8899}],
    )

    controller_node = Node(
        package="aubo_driver_ros2",
        executable="joint_trajectory_controller",
        name="joint_trajectory_controller",
        output="screen",
        parameters=[{"server_host": server_host, "server_port": 8899}],
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            ompl_planning,
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            trajectory_execution,
            {"publish_robot_description_semantic": True},
            {"publish_robot_description": True},
            {"planning_scene_monitor_options": {
                "joint_state_topic": "/joint_states",
                "wait_for_initial_state_timeout": 10.0,
            }},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(pkg_share, "config", "moveit.rviz")],
        parameters=[robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines, moveit_config.joint_limits],
    )
    delayed_rviz = TimerAction(period=12.0, actions=[rviz_node])

    return [
        dashboard_node,
        state_bc_node,
        controller_node,
        rsp_node,
        move_group_node,
        delayed_rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "server_host", default_value=DEFAULT_SERVER_HOST,
            description="机械臂控制器 IP"),
        DeclareLaunchArgument(
            "motion_command_hz", default_value="200.0",
            description="轨迹插值频率(Hz)"),
        OpaqueFunction(function=launch_setup),
    ])
