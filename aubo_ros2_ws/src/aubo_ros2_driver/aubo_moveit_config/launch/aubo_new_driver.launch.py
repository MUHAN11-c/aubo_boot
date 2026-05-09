"""
AUBO 新框架驱动 launch —— 纯新架构 (不依赖旧 driver/simulator/trajectory_action)

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

启动时自动 TCP 探测机械臂是否可达:
  - 可达 → 真实硬件模式 (AUBO 自定义节点)
  - 不可达 → 仿真模式 (ros2_control + mock_components/GenericSystem)
"""

import os
import socket
import yaml
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_SERVER_HOST = "169.254.10.98"


def _check_robot_reachable(host, port=8899, timeout=2.0):
    """TCP 端口探测: 机械臂可达则返回 True"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        r = s.connect_ex((host, port))
        s.close()
        return r == 0
    except Exception:
        return False


def launch_setup(context, *args, **kwargs):
    try:
        motion_command_hz = float(LaunchConfiguration("motion_command_hz").perform(context))
    except ValueError:
        motion_command_hz = 200.0
    if motion_command_hz < 1.0:
        motion_command_hz = 200.0
    trajectory_sample_duration = 1.0 / motion_command_hz

    server_host = LaunchConfiguration("server_host").perform(context)
    use_fake = not _check_robot_reachable(server_host)

    if use_fake:
        print(f"[WARN] 机械臂 {server_host}:8899 不可达 → 自动切换到仿真模式 "
              f"(ros2_control + mock_components/GenericSystem)", file=sys.stderr)
    else:
        print(f"[INFO] 机械臂 {server_host}:8899 已连接 → 真实硬件模式 "
              f"(AUBO 自定义驱动节点)", file=sys.stderr)

    aubo_driver_mode = "simulation" if use_fake else "real"

    # ---- MoveIt 配置 ----
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

    # ---- 公共节点 (两种模式共享) ----
    aubo_mode_node = Node(
        package="aubo_moveit_config",
        executable="aubo_mode.py",
        name="aubo_mode",
        output="screen",
        parameters=[{"aubo_driver_mode": aubo_driver_mode}],
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

    if use_fake:
        # ============================================================
        # 仿真模式: ros2_control + mock_components/GenericSystem
        # ============================================================

        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                robot_description,
                os.path.join(pkg_share, "config", "ros2_controllers.yaml"),
            ],
        )

        joint_state_broadcaster_spawner = TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c", "/controller_manager",
                ],
            )],
        )

        joint_trajectory_controller_spawner = TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "-c", "/controller_manager",
                ],
            )],
        )

        return [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            aubo_mode_node,
            rsp_node,
            move_group_node,
            delayed_rviz,
        ]
    else:
        # ============================================================
        # 真实硬件模式: AUBO 自定义驱动节点
        # ============================================================

        dashboard_node = Node(
            package="aubo_driver_ros2",
            executable="aubo_dashboard_node",
            name="aubo_dashboard",
            output="screen",
            parameters=[{"server_host": server_host, "server_port": 8899,
                         "collision_class": 6}],
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

        return [
            dashboard_node,
            state_bc_node,
            controller_node,
            aubo_mode_node,
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
