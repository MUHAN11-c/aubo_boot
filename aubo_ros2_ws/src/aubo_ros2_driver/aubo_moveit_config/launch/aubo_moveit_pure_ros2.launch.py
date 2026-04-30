"""
纯 ROS2 启动：真实机用 aubo_driver_ros2，轨迹插值在 ROS2。

全链路运动指令频率由 launch 参数 motion_command_hz（默认 200）统一：
  move_group.sample_duration = 1/hz ；aubo_robot_simulator motion_update_rate = hz ；aubo_driver motion_command_hz = hz 。

用法:
  ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
  ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98
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
        .robot_description(
            file_path="config/aubo_e5.urdf.xacro",
            mappings={"use_fake_hardware": "true"},
        )
        .robot_description_semantic(file_path="config/aubo_e5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    pkg_share = get_package_share_directory("aubo_moveit_config")

    robot_description = moveit_config.robot_description

    # OMPL 规划参数：从 planning_pipelines["ompl"] 提取并放入 move_group 命名空间，覆盖运行时参数
    ompl_config = moveit_config.planning_pipelines.get("ompl", {})
    ompl_planning = {"move_group": ompl_config.copy()}
    ompl_planning["move_group"].update({
        "sample_duration": trajectory_sample_duration,
        "planning_time": 15.0,
        "max_planning_attempts": 10,
        "resample_dt": 0.05,
    })

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
    }

    # 轨迹执行参数：真实机使用实时 driver，moveit_manage_controllers=False
    trajectory_execution = moveit_config.trajectory_execution.copy()
    trajectory_execution.update({
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    })

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
            move_group_capabilities,
            trajectory_execution,
            {"publish_robot_description_semantic": True},
            {"publish_robot_description": True},
            {
                "planning_scene_monitor_options": {
                    "joint_state_topic": "/joint_states",
                    "wait_for_initial_state_timeout": 10.0,
                }
            },
        ],
    )

    rviz_config_path = os.path.join(pkg_share, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # -------------------------------------------------------------------------
    # Aubo 硬件驱动节点
    # -------------------------------------------------------------------------
    with open(os.path.join(pkg_share, "config", "joint_names.yaml"), "r") as f:
        joint_names_data = yaml.safe_load(f)
    joint_names_yaml = {"joint_name": joint_names_data}
    joint_names_list = joint_names_data.get(
        "controller_joint_names",
        [
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint",
        ],
    )

    aubo_trajectory_action_node = Node(
        package="aubo_ros2_trajectory_action",
        executable="aubo_ros2_trajectory_action",
        output="screen",
        parameters=[joint_names_yaml],
    )

    aubo_driver_ros2_node = Node(
        package="aubo_driver_ros2",
        executable="aubo_driver_ros2",
        name="aubo_driver",
        output="screen",
        parameters=[
            {
                "server_host": LaunchConfiguration("aubo_driver_server_host"),
                "external_axis_number": 0,
                "motion_command_hz": motion_command_hz,
            }
        ],
    )

    move_to_pose_server_node = Node(
        package="demo_driver",
        executable="move_to_pose_server_node",
        name="move_to_pose_server_node",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    aubo_robot_simulator_ros2_node = Node(
        package="aubo_robot_simulator_ros2",
        executable="aubo_robot_simulator_node",
        name="aubo_robot_simulator",
        output="screen",
        parameters=[
            {
                "motion_update_rate": motion_command_hz,
                "minimum_buffer_size": 600,
                "joint_names": joint_names_list,
            }
        ],
    )

    # RViz 延迟 12s 启动，等 move_group 完全就绪后再加载 MotionPlanning 面板
    delayed_rviz_node = TimerAction(period=12.0, actions=[rviz_node])

    return [
        aubo_driver_ros2_node,
        aubo_robot_simulator_ros2_node,
        aubo_trajectory_action_node,
        move_to_pose_server_node,
        robot_state_publisher,
        move_group_node,
        delayed_rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "aubo_driver_server_host",
            default_value=DEFAULT_SERVER_HOST,
            description="机械臂控制器 IP（aubo_driver_ros2）",
        ),
        DeclareLaunchArgument(
            "motion_command_hz",
            default_value="200.0",
            description="MoveIt→simulator→driver 运动指令统一频率(Hz)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
