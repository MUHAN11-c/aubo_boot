"""panda_moveit_web.launch.py — MoveIt 2 官方 Panda 示例 × ros_web_link 端到端演示.

组成（对齐官方 moveit_resources_panda_moveit_config/demo.launch.py，去 rviz）：
  MoveItConfigsBuilder(moveit_resources_panda, mock_components)
  + move_group（官方动作/服务端：/move_action、/compute_ik、/plan_kinematic_path）
  + robot_state_publisher（latched /robot_description，panda URDF）
  + ros2_control（mock 组件）+ joint_state_broadcaster/panda_arm_controller spawner
  + static_tf world→panda_link0
  + ivg_foxglove_bridge + ros_web_link 网关（console.html 的 RobotModel 经
    robot_mesh 路由解析 package://moveit_resources_panda_description 网格）

用法：
  source install/setup.bash   # 网关需能定位 panda_description share（mesh 路由）
  ros2 launch ros_web_link panda_moveit_web.launch.py
  浏览器 http://<host>:8095/console.html
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "web_port", default_value="8095",
        description="网关端口（8090 归 peach_observability）"))
    ld.add_action(DeclareLaunchArgument(
        "bridge_port", default_value="8765",
        description="foxglove_bridge 上游端口"))

    # ── 官方 MoveIt 配置（与 demo.launch.py 同一构建器/mock 语义） ──────────
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True)
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"])
        .to_moveit_configs()
    )

    ld.add_action(Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "warn"],
    ))
    ld.add_action(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[moveit_config.robot_description], output="log",
    ))
    ld.add_action(Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        output="log",
    ))

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config", "ros2_controllers.yaml")
    ld.add_action(Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    ))
    for ctrl in ("joint_state_broadcaster", "panda_arm_controller"):
        ld.add_action(Node(
            package="controller_manager", executable="spawner",
            arguments=[ctrl, "-c", "/controller_manager",
                       "--param-file", ros2_controllers_path],
            output="screen",
        ))

    # ── web 传输栈（桥 + 网关；python3 须指向 aubo_py3.12 venv） ────────────
    bridge_launch = PathJoinSubstitution([
        FindPackageShare("ivg_foxglove_bridge"),
        "launch", "ivg_foxglove_bridge.launch.xml",
    ])
    ld.add_action(IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(bridge_launch),
        launch_arguments={"port": LaunchConfiguration("bridge_port")}.items(),
    ))
    ld.add_action(ExecuteProcess(
        cmd=["python3", "-m", "ros_web_link.cli",
             LaunchConfiguration("web_port"), "--bind", "0.0.0.0",
             "--foxglove-bridge-port", LaunchConfiguration("bridge_port")],
        output="screen", respawn=True, respawn_delay=2,
    ))

    return ld
