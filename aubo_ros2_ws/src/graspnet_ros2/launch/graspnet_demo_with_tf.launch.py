#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet Demo + TF 专用 Launch（与 aubo_moveit_bridge_ros1 配合使用）

功能：
  1. 发布手眼标定静态 TF：末端(ee_frame_id) -> camera_frame
  2. 启动 graspnet_demo_node（从文件读取数据并预测抓取，发布 MarkerArray、点云和 grasp_pose_X 的 TF）

使用方式：先启动机械臂与 MoveIt 桥接，再启动本 launch。
  终端1: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
  终端2: ros2 launch graspnet_ros2 graspnet_demo_with_tf.launch.py

手动触发发布抓取：
  ros2 run graspnet_ros2 publish_grasps_client

TF 链（必须连成一条，否则 publish_grasps_client 查 base_link->grasp_pose_0 会报两棵树）：
  [aubo_moveit_bridge_ros1] base_link -> ... -> wrist3_Link  （robot_state_publisher）
  [本 launch] wrist3_Link -> camera_frame                    （static_transform_publisher，手眼标定）
  [graspnet_demo_node] camera_frame -> grasp_pose_0,1,...     （调用 /publish_grasps 后发布）

若报错 "base_link and grasp_pose_0 not in the same tree"，请先确认：
  1) aubo_moveit_bridge_ros1 已启动（start_hand_eye_calibration.sh 的 [3/11] ROS2 MoveIt 桥接）
  2) 再启动本 launch，最后再运行 publish_grasps_client
  3) 诊断：ros2 run tf2_ros tf2_echo base_link wrist3_Link
           ros2 run tf2_ros tf2_echo wrist3_Link camera_frame
           ros2 run tf2_ros tf2_echo camera_frame grasp_pose_0

参考：graspnet_demo.launch.py（仅保留 GraspNet 节点与 末端->camera_frame TF，不启动机械臂/ros2_control/rviz）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation


def get_package_share_directory(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory as get_share_dir
        return get_share_dir(package_name)
    except ImportError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_hand_eye_static_tf_args(yaml_path):
    """从手眼标定 YAML 读取变换矩阵，返回 static_transform_publisher 所需的 (x, y, z, yaw, pitch, roll)。"""
    if not yaml_path or not os.path.exists(yaml_path):
        return None
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        he = data.get('hand_eye_calibration', {})
        T_list = he.get('transformation_matrix', None)
        if not T_list:
            return None
        T = np.array(T_list, dtype=np.float64)
        if T.shape != (4, 4):
            return None
        x, y, z = (T[0, 3] / 1000.0), (T[1, 3] / 1000.0), (T[2, 3] / 1000.0)
        R = T[:3, :3]
        euler = ScipyRotation.from_matrix(R).as_euler('zyx', degrees=False)
        yaw, pitch, roll = float(euler[0]), float(euler[1]), float(euler[2])
        return (x, y, z, yaw, pitch, roll)
    except Exception:
        return None


def launch_setup_graspnet_and_tf(context):
    """启动 末端->camera_frame 静态 TF 与 graspnet_demo_node。"""
    package_path = get_package_share_directory('graspnet_ros2')
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')

    launch_file_abs = os.path.abspath(__file__)
    if 'install' in launch_file_abs:
        ws_root = launch_file_abs.split('/install/')[0]
        src_package_path = os.path.join(ws_root, 'src', 'graspnet_ros2')
    else:
        src_package_path = os.path.dirname(os.path.dirname(launch_file_abs))

    nodes = []

    # 静态 TF：末端 -> camera_frame（与 graspnet_demo.launch.py 一致）
    hand_eye_yaml = LaunchConfiguration('hand_eye_yaml_path').perform(context)
    ee_frame_id = LaunchConfiguration('ee_frame_id').perform(context)
    tf_args = load_hand_eye_static_tf_args(hand_eye_yaml)
    if tf_args is not None:
        x, y, z, yaw, pitch, roll = tf_args
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_publisher',
            arguments=[str(x), str(y), str(z), str(yaw), str(pitch), str(roll), ee_frame_id, 'camera_frame'],
            output='screen',
        ))
    else:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_publisher',
            arguments=['0', '0', '0', '0', '0', '0', ee_frame_id, 'camera_frame'],
            output='screen',
        ))

    # GraspNet Demo 节点（与 graspnet_demo.launch.py 中 create_demo_node 逻辑一致）
    demo_script_paths = [
        os.path.join(src_package_path, 'graspnet_ros2', 'graspnet_demo_node.py'),
        os.path.join(package_path, 'graspnet_ros2', 'graspnet_demo_node.py'),
    ]
    demo_script_path = None
    for p in demo_script_paths:
        if os.path.exists(p):
            demo_script_path = p
            break
    if not demo_script_path:
        demo_script_path = demo_script_paths[0]

    python_exec = '/home/mu/miniconda3/envs/ros2_env/bin/python3'
    data_dir_val = LaunchConfiguration('data_dir').perform(context)
    source_data_dir = os.path.join(src_package_path, 'graspnet-baseline', 'doc', 'pose_1')
    required_files = ['color.png', 'depth.png', 'workspace_mask.png', 'meta.mat']
    source_ok = os.path.isdir(source_data_dir) and all(
        os.path.isfile(os.path.join(source_data_dir, f)) for f in required_files
    )
    default_ok = data_dir_val and os.path.isdir(data_dir_val) and all(
        os.path.isfile(os.path.join(data_dir_val, f)) for f in required_files
    )
    if source_ok and (not default_ok or '/install/' in data_dir_val.replace('\\', '/')):
        data_dir_val = source_data_dir

    demo_params = [
        '--ros-args',
        '-r', '__node:=graspnet_demo_node',
        '-p', f"baseline_dir:={LaunchConfiguration('baseline_dir').perform(context)}",
        '-p', f"model_path:={LaunchConfiguration('model_path').perform(context)}",
        '-p', f'data_dir:={data_dir_val}',
        '-p', f"marker_topic:={LaunchConfiguration('marker_topic').perform(context)}",
        '-p', f"pointcloud_topic:={LaunchConfiguration('pointcloud_topic').perform(context)}",
        '-p', f"frame_id:={LaunchConfiguration('frame_id').perform(context)}",
        '-p', f"hand_eye_yaml_path:={hand_eye_yaml}",
        '-p', f"use_open3d:={LaunchConfiguration('use_open3d').perform(context)}",
        '-p', f"auto_run:={LaunchConfiguration('auto_run').perform(context)}",
        '-p', f"num_point:={LaunchConfiguration('num_point').perform(context)}",
        '-p', f"num_view:={LaunchConfiguration('num_view').perform(context)}",
        '-p', f"collision_thresh:={LaunchConfiguration('collision_thresh').perform(context)}",
        '-p', f"voxel_size:={LaunchConfiguration('voxel_size').perform(context)}",
        '-p', f"max_grasps_num:={LaunchConfiguration('max_grasps_num').perform(context)}",
        '-p', f"gpu:={LaunchConfiguration('gpu').perform(context)}",
        '-p', f"trigger_service:={LaunchConfiguration('trigger_service').perform(context)}",
        '-p', f"camera_id:={LaunchConfiguration('camera_id').perform(context)}",
        '-p', f"color_image_topic:={LaunchConfiguration('color_image_topic').perform(context)}",
        '-p', f"depth_image_topic:={LaunchConfiguration('depth_image_topic').perform(context)}",
        '-p', f"camera_info_topic:={LaunchConfiguration('camera_info_topic').perform(context)}",
        '-p', f"factor_depth:={LaunchConfiguration('factor_depth').perform(context)}",
        '-p', f"trigger_grasp_service:={LaunchConfiguration('trigger_grasp_service').perform(context)}",
    ]
    env = dict(os.environ)
    ros2_env_prefix = '/home/mu/miniconda3/envs/ros2_env'
    env['CONDA_PREFIX'] = ros2_env_prefix
    env['CONDA_DEFAULT_ENV'] = 'ros2_env'
    ros_pythonpath = '/opt/ros/foxy/lib/python3.8/site-packages'
    if 'PYTHONPATH' in env:
        if ros_pythonpath not in env['PYTHONPATH']:
            env['PYTHONPATH'] = f"{ros_pythonpath}:{env['PYTHONPATH']}"
    else:
        env['PYTHONPATH'] = ros_pythonpath
    cwd = src_package_path if os.path.exists(src_package_path) else os.path.dirname(demo_script_path) if demo_script_path else package_path

    nodes.append(ExecuteProcess(
        cmd=[python_exec, demo_script_path] + demo_params,
        output='screen',
        cwd=cwd,
        env=env,
    ))
    return nodes


def generate_launch_description():
    package_path = get_package_share_directory('graspnet_ros2')
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')

    declared_arguments = []

    try:
        hand_eye_config_path = os.path.join(
            get_package_share_directory('hand_eye_calibration'),
            'config', 'hand_eye_calibration_best.yaml'
        )
    except Exception:
        hand_eye_config_path = ''

    declared_arguments.append(
        DeclareLaunchArgument(
            'hand_eye_yaml_path',
            default_value=hand_eye_config_path,
            description='手眼标定 YAML 路径（相机到机械臂末端的变换，用于 末端->camera_frame TF）',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ee_frame_id',
            default_value='wrist3_Link',
            description='机械臂末端 link，用于发布 末端->camera_frame 静态 TF（与 aubo_moveit_bridge 中机器人模型一致）',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'),
            description='GraspNet 模型权重路径',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'data_dir',
            default_value=os.path.join(baseline_dir, 'doc', 'pose_1'),
            description='数据目录（color.png, depth.png, workspace_mask.png, meta.mat）',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument('marker_topic', default_value='grasp_markers', description='MarkerArray 话题'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('pointcloud_topic', default_value='graspnet_pointcloud', description='点云话题'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('frame_id', default_value='camera_frame', description='抓取/Marker 坐标系'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_open3d', default_value='false', description='是否启用 Open3D 可视化'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('auto_run', default_value='false', description='是否自动运行（false 时需调用 /publish_grasps）'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('baseline_dir', default_value=baseline_dir, description='graspnet-baseline 目录'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('num_point', default_value='20000', description='点云采样数量'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('num_view', default_value='300', description='视角数量'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('collision_thresh', default_value='0.01', description='碰撞检测阈值'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('voxel_size', default_value='0.01', description='体素大小'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('max_grasps_num', default_value='20', description='发布的最大抓取数量'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('gpu', default_value='0', description='GPU 设备 ID'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('trigger_service', default_value='/software_trigger', description='相机软触发服务名'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('camera_id', default_value='207000152740', description='相机 ID'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('color_image_topic', default_value='/camera/color/image_raw', description='彩色图话题'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth/image_raw', description='深度图话题'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info', description='相机内参话题'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('factor_depth', default_value='4000.0', description='深度缩放因子'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('trigger_grasp_service', default_value='trigger_grasp', description='触发抓取服务名'),
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup_graspnet_and_tf)]
    )
