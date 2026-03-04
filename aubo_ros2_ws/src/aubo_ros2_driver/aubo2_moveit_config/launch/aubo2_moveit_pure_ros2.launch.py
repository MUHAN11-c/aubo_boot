"""
aubo2 纯 ROS2 启动（无 ROS1 桥接）：真实机用 aubo_driver_ros2，轨迹插值在 ROS2。

参考 MoveIt 官方教程编写：
  https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html
- Loading the MoveIt Configuration (MoveItConfigsBuilder)
- Launching Move Group
- Visualizing with RViz（DeclareLaunchArgument + PathJoinSubstitution）
- Publishing Transforms to tf2
- 真实机不启动 ros2_control，由 Aubo 专用节点替代。

用法:
  ros2 launch aubo2_moveit_config aubo2_moveit_pure_ros2.launch.py
  ros2 launch aubo2_moveit_config aubo2_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98
  ros2 launch aubo2_moveit_config aubo2_moveit_pure_ros2.launch.py rviz_config:=moveit.rviz

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
    # Loading the MoveIt Configuration（参考教程 MoveItConfigsBuilder）
    moveit_config = (
        MoveItConfigsBuilder("aubo_robot", package_name="aubo2_moveit_config")
        .robot_description_semantic(file_path="config/aubo_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",  # 默认规划器
            pipelines=["ompl", "pilz"],  # 要加载的规划器列表
            load_all=False,  # 不加载所有规划器
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    pkg_share = get_package_share_directory("aubo2_moveit_config")

    # robot_description 使用 aubo_description 的 aubo_robot.urdf.xacro（与 aubo2 模型一致，真实机不启动 ros2_control）
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("aubo_description"),
                "urdf/robots/",
                "aubo_robot.urdf.xacro",
            ]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Launching Move Group（教程：parameters=[moveit_config.to_dict()]）
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), robot_description],
    )

    # Visualizing with RViz（教程：DeclareLaunchArgument + PathJoinSubstitution）
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("aubo2_moveit_config"),
        "config",
        LaunchConfiguration("rviz_config"),
    ])
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

    # Publishing Transforms to tf2（教程：robot_state_publisher 发布 joint states 到 tf2）
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 教程中 "Setting up ros2_control"：真实机此处不启动 controller_manager，由 Aubo 专用节点替代。

    # Aubo 专用节点：轨迹 action、真实机驱动、插值器、Move To Pose 服务
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
            "motion_update_rate": 200.0,
            "minimum_buffer_size": 600,
            "joint_names": joint_names_list,
        }],
    )

    # Launching all the nodes（教程最后一节）
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
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz 配置文件（相对于本包 config/）",
        ),
        OpaqueFunction(function=launch_setup),
    ])
