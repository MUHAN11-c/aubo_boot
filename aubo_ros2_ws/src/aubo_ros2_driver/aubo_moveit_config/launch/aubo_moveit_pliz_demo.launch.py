"""
Pilz Industrial Motion Planner 演示：纯 ROS2 启动，使用 Pilz 规划器（PTP/LIN/CIRC）。

用法:
  ros2 launch aubo_moveit_config aubo_moveit_pliz_demo.launch.py
  ros2 launch aubo_moveit_config aubo_moveit_pliz_demo.launch.py aubo_driver_server_host:=169.254.10.98

使用 Pilz 规划器替代 OMPL，支持：
  - PTP: 点对点关节空间运动
  - LIN: 笛卡尔空间直线运动
  - CIRC: 笛卡尔空间圆弧运动
  - MoveGroupSequenceAction/Service: 多段轨迹序列规划

参考：moveit2_tutorials/doc/how_to_guides/pilz_industrial_motion_planner

预期警告（可忽略）:
  - log4cplus "could not open file ./config/tracelog.properties"：aubo_driver 从当前工作目录读配置，不影响运动。
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_SERVER_HOST = "169.254.10.98"


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path), "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    try:
        motion_command_hz = float(LaunchConfiguration("motion_command_hz").perform(context))
    except ValueError:
        motion_command_hz = 200.0
    if motion_command_hz < 1.0:
        motion_command_hz = 200.0

    # MoveItConfigsBuilder：使用 Pilz 规划器（PTP/LIN/CIRC）
    moveit_config = (
        MoveItConfigsBuilder("aubo_robot", package_name="aubo_moveit_config")
        .robot_description_semantic(file_path="config/aubo_i5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])  # 加载 config/pilz_industrial_motion_planner_planning.yaml
        .to_moveit_configs()
    )
    pkg_share = get_package_share_directory("aubo_moveit_config")

    # robot_description 使用 aubo_ros2.xacro（不用 to_dict 里的默认 urdf）
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("aubo_description"),
                "urdf/xacro/inc/",
                "aubo_ros2.xacro",
            ]),
            " aubo_type:=aubo_e5_10 ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # move_group：使用 Pilz 规划器；robot_description_planning 需合并 joint_limits 与 pilz_cartesian_limits
    robot_description_planning = {}
    for config in [moveit_config.joint_limits, moveit_config.pilz_cartesian_limits]:
        if config and isinstance(config, dict):
            for v in config.values():
                if isinstance(v, dict):
                    robot_description_planning.update(v)
                    break
    planning_params = (
        {"robot_description_planning": robot_description_planning}
        if robot_description_planning
        else moveit_config.joint_limits
    )

    # Pilz MoveGroupSequenceAction 与 MoveGroupSequenceService 能力
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
    }
    with open(os.path.join(pkg_share, "config", "moveit_controllers.yaml"), "r") as f:
        moveit_controllers = {
            "moveit_simple_controller_manager": yaml.safe_load(f),
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }

    # 轨迹执行允许时间 = 规划时长×scaling + margin；在 launch 中设置合理值，避免过早超时又不过大
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            planning_params,
            moveit_config.planning_pipelines,
            move_group_capabilities,
            trajectory_execution,
            {'publish_robot_description_semantic': True},
            moveit_controllers,
            {"planning_scene_monitor_options": {"joint_state_topic": "/joint_states", "wait_for_initial_state_timeout": 10.0}},
        ],
    )

    # RViz：直接用 moveit_config 的语义、运动学、规划、关节限制
    # MoveGroupInterface 通过 move_group.planning_pipelines.<pipeline>.<group>.default_planner_config 查找默认规划器
    rviz_config_path = os.path.join(pkg_share, "config", "view_robot.rviz")
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
            # MoveGroupInterface 查找此参数以设置默认规划器；RViz 中若仍为 <unspecified> 请手动选 PTP
            {"move_group/planning_pipelines/pilz_industrial_motion_planner.manipulator.default_planner_config": "PTP"},
        ],
    )

    # -------------------------------------------------------------------------
    # Publishing Transforms to tf2
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 与 aubo_moveit_pure_ros2 一致：use_aubo_driver_ros2 时不启动 ros2_control_node，
    # joint_states 与执行由 aubo_driver_ros2 + aubo_robot_simulator_ros2 完成。

    # -------------------------------------------------------------------------
    # Aubo 专用节点：轨迹 action、真实机驱动、插值器、Move To Pose 服务
    # -------------------------------------------------------------------------
    with open(os.path.join(pkg_share, "config", "joint_names.yaml"), "r") as f:
        joint_names_data = yaml.safe_load(f)
    joint_names_yaml = {"joint_name": joint_names_data}
    joint_names_list = joint_names_data.get("controller_joint_names", [
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint",
    ])

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
        parameters=[{
            "server_host": LaunchConfiguration("aubo_driver_server_host"),
            "external_axis_number": 0,
            "motion_command_hz": motion_command_hz,
        }],
    )

    # 不传 planning_group_name/base_frame：节点已 declare_parameter，再传会报 already been declared
    # planner_id: PTP 供 Pilz 规划器使用（OMPL 可留空）
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
            {"planner_id": "PTP"},
        ],
    )

    aubo_robot_simulator_ros2_node = Node(
        package="aubo_robot_simulator_ros2",
        executable="aubo_robot_simulator_node",
        name="aubo_robot_simulator",
        output="screen",
        parameters=[{
            "motion_update_rate": motion_command_hz,
            "minimum_buffer_size": 600,  # 降低节流阈值，避免速度因子 0.1 时 rib>2000 导致成批发送卡顿
            "joint_names": joint_names_list,
        }],
    )

    # -------------------------------------------------------------------------
    # Launching all the nodes
    # -------------------------------------------------------------------------
    return [
        aubo_driver_ros2_node,
        aubo_robot_simulator_ros2_node,
        aubo_trajectory_action_node,
        move_to_pose_server_node,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
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
            description="simulator 插补与 driver motion_command_hz 统一频率(Hz)；与 Noetic jti 1/200 对齐时取 200",
        ),
        OpaqueFunction(function=launch_setup),
    ])
