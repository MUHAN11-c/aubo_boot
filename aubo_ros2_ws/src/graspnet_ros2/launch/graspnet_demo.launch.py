#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo Launch 文件。

功能：
  1. 启动 Aubo 机械臂模型与 ros2_control 仿真
  2. 启动 graspnet_demo_node（预测抓取，发布 MarkerArray、点云与 TF）
  3. 发布手眼标定静态 TF（末端 -> camera_frame），以及 camera_frame -> camera_link 单位变换（感知帧与 link 等价）
  4. 启动 MoveIt2 move_group 与 MoveToPose 服务
  5. 启动 RViz2

使用：
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch graspnet_ros2 graspnet_demo.launch.py

手动触发抓取发布：
  ros2 run graspnet_ros2 publish_grasps_client

TF 诊断（详见 TF_USAGE.md）：
  ros2 run tf2_ros tf2_echo base_link wrist3_Link
  ros2 run tf2_ros tf2_echo wrist3_Link camera_frame
  ros2 run tf2_ros tf2_echo camera_frame grasp_pose_0
  ros2 run tf2_tools view_frames   # 生成 TF 树 PDF
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 包名常量
PKG_GRASPNET = 'graspnet_ros2'
PKG_AUBO_DESC = 'aubo_description'
PKG_AUBO_MOVEIT = 'aubo_moveit_config'


def get_package_share_directory(package_name):
    """获取包的 share 目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory as _get
        return _get(package_name)
    except ImportError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_yaml(package_name, file_path):
    """从包 share 路径加载 YAML 文件。"""
    try:
        pkg_share = get_package_share_directory(package_name)
        with open(os.path.join(pkg_share, file_path), 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def _controller_load_script():
    """返回加载 ros2_control 控制器的 bash 命令。"""
    return (
        'ros2 service call /controller_manager/load_controller '
        'controller_manager_msgs/srv/LoadController "{name: joint_state_broadcaster}" && '
        'ros2 service call /controller_manager/configure_controller '
        'controller_manager_msgs/srv/ConfigureController "{name: joint_state_broadcaster}" && '
        'ros2 service call /controller_manager/load_controller '
        'controller_manager_msgs/srv/LoadController "{name: joint_trajectory_controller}" && '
        'ros2 service call /controller_manager/configure_controller '
        'controller_manager_msgs/srv/ConfigureController "{name: joint_trajectory_controller}" && '
        'ros2 service call /controller_manager/switch_controller '
        'controller_manager_msgs/srv/SwitchController "{activate_controllers: [\'joint_state_broadcaster\', '
        '\'joint_trajectory_controller\'], deactivate_controllers: [], strictness: 2, '
        'activate_asap: true, timeout: {sec: 0, nanosec: 0}}"'
    )


def launch_setup_robot_and_tf(context):
    """启动机械臂、ros2_control、手眼 TF、MoveIt2 与 RViz2。"""
    aubo_type = LaunchConfiguration('aubo_type').perform(context)
    hand_eye_yaml = LaunchConfiguration('hand_eye_yaml_path').perform(context)
    ee_frame_id = LaunchConfiguration('ee_frame_id').perform(context)

    sim_time_param = {}
    nodes = []

    # --- 机械臂描述 ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(PKG_AUBO_DESC), 'urdf/xacro/inc/', 'aubo_ros2.xacro'
        ]),
        ' ', 'aubo_type:=', aubo_type, ' ',
    ])
    robot_description = {'robot_description': robot_description_content}

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, sim_time_param],
    ))

    # --- ros2_control ---
    ros2_controllers_path = os.path.join(
        get_package_share_directory(PKG_AUBO_MOVEIT), 'config', 'ros2_controllers.yaml'
    )
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path, sim_time_param],
        output='screen',
    ))
    nodes.append(ExecuteProcess(
        cmd=['bash', '-c', _controller_load_script()],
        output='screen',
    ))

    # --- 手眼静态 TF ---
    nodes.append(Node(
        package=PKG_GRASPNET,
        executable='hand_eye_static_tf_node',
        name='hand_eye_static_tf_node',
        output='screen',
        parameters=[{
            'hand_eye_yaml_path': hand_eye_yaml,
            'ee_frame_id': ee_frame_id,
            'child_frame_id': 'camera_frame',
        }],
    ))
    # 感知帧与 link 同名等价：camera_frame 与 camera_link 视为同一坐标系（0 变换）
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_to_camera_link',
        arguments=['0', '0', '0', '1.5708', '-1.5708', '0', 'camera_frame', 'camera_link'],
    ))

    # --- MoveIt2 move_group ---
    srdf_path = os.path.join(
        get_package_share_directory(PKG_AUBO_MOVEIT), 'config', 'aubo_i5.srdf'
    )
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml(PKG_AUBO_MOVEIT, 'config/kinematics.yaml')
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(PKG_AUBO_MOVEIT, 'config/joint_limits.yaml')
    }
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
            'sample_duration': 0.005,
            'planning_time': 5.0,
            'max_planning_attempts': 10,
        }
    }
    ompl_yaml = load_yaml(PKG_AUBO_MOVEIT, 'config/ompl_planning.yaml')
    if ompl_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    moveit_controllers_yaml = load_yaml(PKG_AUBO_MOVEIT, 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 3.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/move_group/planning_scene_monitor',
            'publish_planning_scene_topic': '/move_group/publish_planning_scene',
            'monitored_planning_scene_topic': '/move_group/monitored_planning_scene',
            'wait_for_initial_state_timeout': 10.0,
        },
    }

    octomap_config = {'octomap_frame': 'camera_frame', 'octomap_resolution': 0.02}
    octomap_updater_config = load_yaml(PKG_GRASPNET, 'config/sensors_3d.yaml')

    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        {'publish_robot_description_semantic': True},
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        joint_limits_yaml,
        sim_time_param,
    ]
    if octomap_updater_config:
        move_group_params.extend([octomap_config, octomap_updater_config])

    nodes.append(Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    ))

    # --- MoveToPose 服务 ---
    nodes.append(Node(
        package='demo_driver',
        executable='move_to_pose_server_node',
        name='move_to_pose_server_node',
        output='screen',
        parameters=[robot_description, robot_description_semantic, sim_time_param],
    ))

    # --- RViz2 ---
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if rviz_config and os.path.exists(rviz_config):
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                robot_description,
                sim_time_param,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                kinematics_yaml,
                joint_limits_yaml,
            ],
        ))

    return nodes


def _declare_launch_arguments(package_path, baseline_dir):
    """声明所有 launch 参数，返回列表。"""
    try:
        hand_eye_default = os.path.join(
            get_package_share_directory('hand_eye_calibration'),
            'config', 'hand_eye_calibration_best.yaml'
        )
    except Exception:
        hand_eye_default = ''

    return [
        # GraspNet 核心
        DeclareLaunchArgument('model_path', default_value=os.path.join(
            baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'
        ), description='模型权重路径'),
        DeclareLaunchArgument('data_dir', default_value=os.path.join(
            baseline_dir, 'doc', 'pose_1'
        ), description='数据目录（含 color/depth/mask/meta）'),
        DeclareLaunchArgument('marker_topic', default_value='grasp_markers', description='MarkerArray 话题'),
        DeclareLaunchArgument('pointcloud_topic', default_value='graspnet_pointcloud', description='点云话题'),
        DeclareLaunchArgument('frame_id', default_value='camera_frame', description='抓取坐标系'),
        DeclareLaunchArgument('use_open3d', default_value='true', description='是否启用 Open3D'),
        DeclareLaunchArgument('auto_run', default_value='false', description='是否自动运行（否则需服务触发）'),
        DeclareLaunchArgument('baseline_dir', default_value=baseline_dir, description='graspnet-baseline 路径'),
        DeclareLaunchArgument('enable_graspnet_node', default_value='true', description='是否启动 graspnet_demo_node'),

        # 机械臂与手眼
        DeclareLaunchArgument('ee_frame_id', default_value='wrist3_Link', description='末端 link（手眼 TF 父坐标系）'),
        DeclareLaunchArgument('hand_eye_yaml_path', default_value=hand_eye_default, description='手眼标定 YAML'),
        DeclareLaunchArgument('aubo_type', default_value='aubo_e5_10', description='Aubo 型号'),

        # RViz
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            package_path, 'config', 'demo.rviz'
        ), description='RViz 配置'),

        # 相机与触发（可选覆盖）
        DeclareLaunchArgument('trigger_service', default_value='/software_trigger', description='软触发服务名'),
        DeclareLaunchArgument('camera_id', default_value='207000152740', description='相机 ID'),
        DeclareLaunchArgument('color_image_topic', default_value='/camera/color/image_raw', description='彩色图话题'),
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth/image_raw', description='深度图话题'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info', description='相机内参话题'),
        DeclareLaunchArgument('factor_depth', default_value='4000.0', description='深度缩放因子'),
        DeclareLaunchArgument('trigger_grasp_service', default_value='trigger_grasp', description='触发抓取服务名'),
    ]


def generate_launch_description():
    """生成 LaunchDescription。"""
    package_path = get_package_share_directory(PKG_GRASPNET)
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')

    declared_arguments = _declare_launch_arguments(package_path, baseline_dir)

    graspnet_demo_node = Node(
        package=PKG_GRASPNET,
        executable='graspnet_demo_node',
        name='graspnet_demo_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_graspnet_node')),
        parameters=[{
            'baseline_dir': LaunchConfiguration('baseline_dir'),
            'model_path': LaunchConfiguration('model_path'),
            'data_dir': LaunchConfiguration('data_dir'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'hand_eye_yaml_path': LaunchConfiguration('hand_eye_yaml_path'),
            'use_open3d': LaunchConfiguration('use_open3d'),
            'auto_run': LaunchConfiguration('auto_run'),
        }],
    )

    robot_and_tf = OpaqueFunction(function=launch_setup_robot_and_tf)

    return LaunchDescription(declared_arguments + [graspnet_demo_node, robot_and_tf])
