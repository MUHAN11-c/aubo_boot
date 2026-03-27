"""
纯 ROS2 启动：真实机用 aubo_driver_ros2，轨迹插值在 ROS2。

全链路运动指令频率由 launch 参数 motion_command_hz（默认 200）统一：
  move_group.sample_duration = 1/hz ；aubo_robot_simulator motion_update_rate = hz ；aubo_driver motion_command_hz = hz 。
  笛卡尔路径（computeCartesianPath → execute）生成的关节轨迹同样会经规划管线里的 AddTimeOptimalParameterization 做时间参数化与重采样；
  若未设置 move_group.resample_dt，默认 0.1 s 比 sample_duration 粗很多，容易在笛卡尔执行阶段表现为顿挫；此处将 resample_dt 与 sample_duration 对齐。

用法:
  ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
  ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98

使用 moveit_configs_utils 供 RViz / move_to_pose_server；move_group 不使用 to_dict()，显式传参并指定 OMPL。

参考：https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html

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
    # 运动指令链统一频率：MoveIt 轨迹采样 ≡ simulator 插补 ≡ driver 取点周期（默认 200Hz，与 Noetic jti 1/200 一致）
    try:
        motion_command_hz = float(LaunchConfiguration("motion_command_hz").perform(context))
    except ValueError:
        motion_command_hz = 200.0
    if motion_command_hz < 1.0:
        motion_command_hz = 200.0
    trajectory_sample_duration = 1.0 / motion_command_hz

    # MoveItConfigsBuilder：显式指定各配置文件（路径相对于 aubo_moveit_config 包）
    moveit_config = (
        MoveItConfigsBuilder("aubo_robot", package_name="aubo_moveit_config")
        .robot_description_semantic(file_path="config/aubo_i5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])  # 加载 config/ompl_planning.yaml
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

    # move_group：不使用 to_dict()，显式传参；仅显式指定 OMPL，其余用 moveit_config 已加载的默认
    ompl_planning = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
            "sample_duration": trajectory_sample_duration,
            "planning_time": 15.0,
            "max_planning_attempts": 10,
        }
    }
    ompl_planning["move_group"].update(load_yaml("aubo_moveit_config", "config/ompl_planning.yaml"))
    # AddTimeOptimalParameterization：默认 resample_dt=0.1；笛卡尔与关节规划后的轨迹重采样均走此处，过稀时笛卡尔执行段尤其易顿挫
    ompl_planning["move_group"]["resample_dt"] = 0.05
    # Enable MTC execute action server: /execute_task_solution
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
    }
    with open(os.path.join(pkg_share, "config", "moveit_controllers.yaml"), "r") as f:
        moveit_controllers = {
            "moveit_simple_controller_manager": yaml.safe_load(f),
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }

    # 上界 ≈ 末点时间×scaling + margin；MTC 需在节点侧启用 AddTimeOptimalParameterization，勿仅靠放大 margin。
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
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
            moveit_config.joint_limits,
            ompl_planning,
            move_group_capabilities,
            moveit_controllers,
            trajectory_execution,  # 放最后以覆盖 yaml 中的 1.2/0.5
            {'publish_robot_description_semantic': True},
            {'publish_robot_description': True},
            {"planning_scene_monitor_options": {"joint_state_topic": "/joint_states", "wait_for_initial_state_timeout": 10.0}},
        ],
    )

    # RViz：直接用 moveit_config 的语义、运动学、规划、关节限制
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

    # use_aubo_driver_ros2 时不启动 ros2_control_node，
    # joint_states 与执行由 aubo_driver_ros2 + aubo_robot_simulator_ros2 完成。
    # robot_state_publisher 需要启动以发布 TF 树（基于 URDF 和 /joint_states）。

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
            description="MoveIt→simulator→driver 运动指令统一频率(Hz)，与 motion_update_rate、motion_command_hz 一致",
        ),
        OpaqueFunction(function=launch_setup),
    ])
