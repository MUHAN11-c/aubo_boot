#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo（点云版）Launch 文件。

功能：
  1. 启动 Aubo 机械臂模型与 ros2_control 仿真
  2. 启动 graspnet_demo_points_node（订阅 PointCloud2，预测抓取，发布 MarkerArray、点云与 TF）
  3. 发布手眼标定静态 TF（末端 -> camera_frame），以及 camera_frame -> camera_link 单位变换
  4. 启动 MoveIt2 move_group 与 MoveToPose 服务
  5. 启动 RViz2

使用：
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch graspnet_ros2 graspnet_demo_points.launch.py

点云话题默认：/camera/depth_registered/points（可在 launch 参数中覆盖）。
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
        # GraspNet 点云版核心
        DeclareLaunchArgument('model_path', default_value=os.path.join(
            baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'
        ), description='模型权重路径'),
        DeclareLaunchArgument('input_pointcloud_topic', default_value='/camera/depth_registered/points',
                              description='输入点云话题'),
        DeclareLaunchArgument('marker_topic', default_value='grasp_markers', description='MarkerArray 话题'),
        DeclareLaunchArgument('pointcloud_topic', default_value='graspnet_pointcloud', description='发布点云话题'),
        DeclareLaunchArgument('frame_id', default_value='camera_frame', description='抓取坐标系'),
        DeclareLaunchArgument('num_point', default_value='20000', description='点云采样点数'),
        DeclareLaunchArgument('num_view', default_value='300', description='视角数'),
        DeclareLaunchArgument('collision_thresh', default_value='0.01', description='碰撞检测阈值'),
        DeclareLaunchArgument('voxel_size', default_value='0.01', description='体素下采样尺寸'),
        DeclareLaunchArgument('max_grasps_num', default_value='20', description='最大抓取数量'),
        DeclareLaunchArgument('gpu', default_value='0', description='GPU 设备 ID'),
        DeclareLaunchArgument('use_open3d', default_value='false', description='是否启用 Open3D 可视化'),
        DeclareLaunchArgument('auto_run', default_value='false', description='是否自动运行（否则需服务触发）'),
        DeclareLaunchArgument('baseline_dir', default_value=baseline_dir, description='graspnet-baseline 路径'),

        # 机械臂与手眼
        DeclareLaunchArgument('ee_frame_id', default_value='wrist3_Link', description='末端 link（手眼 TF 父坐标系）'),
        DeclareLaunchArgument('hand_eye_yaml_path', default_value=hand_eye_default, description='手眼标定 YAML'),
        DeclareLaunchArgument('aubo_type', default_value='aubo_e5_10', description='Aubo 型号'),

        # RViz
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            package_path, 'config', 'demo.rviz'
        ), description='RViz 配置'),
    ]


def generate_launch_description():
    """生成 LaunchDescription。"""
    package_path = get_package_share_directory(PKG_GRASPNET)
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')

    declared_arguments = _declare_launch_arguments(package_path, baseline_dir)

    graspnet_demo_points_node = Node(
        package=PKG_GRASPNET,
        executable='graspnet_demo_points_node',
        name='graspnet_demo_points_node',
        output='screen',
        parameters=[{
            'baseline_dir': LaunchConfiguration('baseline_dir'),
            'model_path': LaunchConfiguration('model_path'),
            'input_pointcloud_topic': LaunchConfiguration('input_pointcloud_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'num_point': LaunchConfiguration('num_point'),
            'num_view': LaunchConfiguration('num_view'),
            'collision_thresh': LaunchConfiguration('collision_thresh'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'max_grasps_num': LaunchConfiguration('max_grasps_num'),
            'gpu': LaunchConfiguration('gpu'),
            'use_open3d': LaunchConfiguration('use_open3d'),
            'auto_run': LaunchConfiguration('auto_run'),
        }],
    )

    robot_and_tf = OpaqueFunction(function=launch_setup_robot_and_tf)

    return LaunchDescription(declared_arguments + [graspnet_demo_points_node, robot_and_tf])
