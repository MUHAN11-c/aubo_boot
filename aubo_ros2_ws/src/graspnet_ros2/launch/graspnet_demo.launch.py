#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo（仅仿真）：使用 aubo2_moveit_config 配置与模型，ros2_control 仿真 + MoveIt2 + graspnet_demo_node + 手眼 TF + RViz。
本 launch 明确用于仿真，不连接真实机械臂。规划使用 OMPL 最短路径（默认 RRTstar）。

用法:
  ros2 launch graspnet_ros2 graspnet_demo.launch.py
  # 同时启动 MoveIt C++ 教程节点（规划演示 + RViz 可视化）:
  ros2 launch graspnet_ros2 graspnet_demo.launch.py use_moveit_cpp_tutorial:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory as get_ament_package_share
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation


def get_package_share_directory(package_name):
    """获取包的共享目录（ROS2 标准方式）"""
    try:
        from ament_index_python.packages import get_package_share_directory as get_share_dir
        return get_share_dir(package_name)
    except ImportError:
        # 回退到源码目录（开发环境）
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_yaml(package_name, file_path):
    """从 ROS 包路径加载 YAML 文件（用于 MoveIt 配置）。"""
    try:
        pkg_share = get_package_share_directory(package_name)
        abs_path = os.path.join(pkg_share, file_path)
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def load_hand_eye_static_tf_args(yaml_path):
    """从手眼标定 YAML 读取变换矩阵，返回 static_transform_publisher 所需的 (x, y, z, yaw, pitch, roll)。
    矩阵含义：相机坐标系 → 机械臂末端坐标系（T_ee_camera），平移单位为毫米，返回的平移转换为米。
    """
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
        # 平移：毫米 -> 米
        x, y, z = (T[0, 3] / 1000.0), (T[1, 3] / 1000.0), (T[2, 3] / 1000.0)
        R = T[:3, :3]
        # ROS 常用 yaw-pitch-roll (ZYX 内旋)，单位弧度
        euler = ScipyRotation.from_matrix(R).as_euler('zyx', degrees=False)
        yaw, pitch, roll = float(euler[0]), float(euler[1]), float(euler[2])
        return (x, y, z, yaw, pitch, roll)
    except Exception:
        return None


def launch_setup_robot_and_tf(context):
    """MoveIt 与仿真部分完全参考 moveit2_tutorials doc/examples/moveit_cpp/launch/moveit_cpp_tutorial.launch.py。"""
    moveit_pkg = "aubo2_moveit_config"
    hand_eye_yaml = LaunchConfiguration("hand_eye_yaml_path").perform(context)
    ee_frame_id = LaunchConfiguration("ee_frame_id").perform(context)
    pkg_share = get_ament_package_share(moveit_pkg)

    # MoveIt configuration：与官方一致，robot_description_semantic/kinematics/joint_limits + trajectory_execution + moveit_cpp（规划从 moveit_cpp 的 ompl_planning.yaml 读）；planning_pipelines 为 move_group/move_to_pose/movel 所需
    moveit_config = (
        MoveItConfigsBuilder("aubo_robot", package_name=moveit_pkg)
        .robot_description_semantic(file_path="config/aubo_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"], load_all=False)
        .moveit_cpp(file_path="config/ompl_planning.yaml")
        .to_moveit_configs()
    )
    # robot_description 与官方一致：用 xacro 生成（aubo_description）
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
    # 用 ParameterValue(value_type=str) 避免 launch 将 robot_description 当 YAML 解析
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ----- 节点顺序与参数完全对照官方 -----
    # Static TF（官方：world->panda_link0；此处保留 末端->camera_frame 手眼标定）
    tf_args = load_hand_eye_static_tf_args(hand_eye_yaml)
    x, y, z = (tf_args[0], tf_args[1], tf_args[2]) if tf_args else (0, 0, 0)
    yaw, pitch, roll = (tf_args[3], tf_args[4], tf_args[5]) if tf_args else (0, 0, 0)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--x", str(x), "--y", str(y), "--z", str(z), "--yaw", str(yaw), "--pitch", str(pitch), "--roll", str(roll), "--frame-id", ee_frame_id, "--child-frame-id", "camera_frame"],
    )

    # Publish TF（官方：parameters=[moveit_config.robot_description]）
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz（官方：parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic]）
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    rviz_config_file = LaunchConfiguration("rviz_config").perform(context)
    rviz_config_path = rviz_config_file if os.path.isabs(rviz_config_file) else os.path.join(str(pkg_share), "config", rviz_config_file)
    rviz_node = None
    if use_rviz and os.path.exists(rviz_config_path):
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

    # move_group（官方无；graspnet 需要）parameters 与官方风格一致
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), robot_description],
    )

    move_to_pose_node = Node(
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

    movel_node = Node(
        package="demo_driver",
        executable="movel_server_node",
        name="movel_server_node",
        output="screen",
        parameters=[moveit_config.to_dict(), robot_description],
    )

    # MoveItCpp demo executable（官方：parameters=[moveit_config.to_dict()]）
    use_moveit_cpp_tutorial = LaunchConfiguration("use_moveit_cpp_tutorial", default="false").perform(context) == "true"
    moveit_cpp_node = None
    if use_moveit_cpp_tutorial:
        moveit_cpp_node = Node(
            name="moveit_cpp_tutorial",
            package="demo_driver",
            executable="moveit_cpp_tutorial_node",
            output="screen",
            parameters=[moveit_config.to_dict(), robot_description],
        )

    # ros2_control using FakeSystem as hardware（官方：parameters=[moveit_config.robot_description, ros2_controllers_path]）
    ros2_controllers_path = os.path.join(str(pkg_share), "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    # Load controllers（官方：ExecuteProcess shell 调用 spawner）
    load_controllers = []
    for i, controller in enumerate(["joint_state_broadcaster", "manipulator_controller"]):
        args = [controller, "--controller-manager", "/controller_manager"] if i == 0 else [controller, "-c", "/controller_manager"]
        load_controllers.append(
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner " + " ".join(args)],
                shell=True,
                output="screen",
            )
        )

    # 组装顺序与官方一致：static_tf, robot_state_publisher, rviz, moveit_cpp, ros2_control, load_controllers（官方无 move_group/move_to_pose/movel）
    nodes = [static_tf, robot_state_publisher]
    if rviz_node is not None:
        nodes.append(rviz_node)
    nodes.extend([run_move_group_node, move_to_pose_node, movel_node])
    if moveit_cpp_node is not None:
        nodes.append(moveit_cpp_node)
    nodes.append(ros2_control_node)
    nodes.extend(load_controllers)
    return nodes


def create_demo_node(context):
    """创建 demo 节点的函数"""
    package_path = get_package_share_directory('graspnet_ros2')
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')
    
    # 获取源码目录
    launch_file_abs = os.path.abspath(__file__)
    if 'install' in launch_file_abs:
        ws_root = launch_file_abs.split('/install/')[0]
        src_package_path = os.path.join(ws_root, 'src', 'graspnet_ros2')
    else:
        launch_file_dir = os.path.dirname(launch_file_abs)
        src_package_path = os.path.dirname(launch_file_dir)
    
    # 查找 demo 脚本
    demo_script_paths = [
        os.path.join(src_package_path, 'graspnet_ros2', 'graspnet_demo_node.py'),
        os.path.join(package_path, 'graspnet_ros2', 'graspnet_demo_node.py'),
    ]
    
    demo_script_path = None
    for path in demo_script_paths:
        if os.path.exists(path):
            demo_script_path = path
            break
    
    if not demo_script_path:
        demo_script_path = demo_script_paths[0]
    
    # 若设置了 graspnet_python_executable 则用其运行 demo 节点；否则优先尝试 conda graspnet 环境（含 pointnet2._ext）；再否则用当前解释器
    python_override = LaunchConfiguration('graspnet_python_executable').perform(context)
    if python_override and python_override.strip():
        python_exec = python_override.strip()
    else:
        graspnet_env_python = '/home/mu/miniconda3/envs/graspnet/bin/python3'
        python_exec = graspnet_env_python if os.path.isfile(graspnet_env_python) else sys.executable
    
    # 获取参数值
    model_path_val = LaunchConfiguration('model_path').perform(context)
    data_dir_val = LaunchConfiguration('data_dir').perform(context)
    # 当 data_dir 未设置、不存在或缺少必要文件时，优先使用源码 doc/pose_1，与直接运行 demo.py 一致
    source_data_dir = os.path.join(src_package_path, 'graspnet-baseline', 'doc', 'pose_1')
    required_files = ['color.png', 'depth.png', 'workspace_mask.png', 'meta.mat']
    source_ok = os.path.isdir(source_data_dir) and all(
        os.path.isfile(os.path.join(source_data_dir, f)) for f in required_files
    )
    default_ok = data_dir_val and os.path.isdir(data_dir_val) and all(
        os.path.isfile(os.path.join(data_dir_val, f)) for f in required_files
    )
    # 使用源码目录当：当前路径无效，或当前路径在 install 下（开发时与 demo.py 一致）
    if source_ok and (not default_ok or '/install/' in data_dir_val.replace('\\', '/')):
        data_dir_val = source_data_dir
    marker_topic_val = LaunchConfiguration('marker_topic').perform(context)
    pointcloud_topic_val = LaunchConfiguration('pointcloud_topic').perform(context)
    frame_id_val = LaunchConfiguration('frame_id').perform(context)
    hand_eye_yaml_val = LaunchConfiguration('hand_eye_yaml_path').perform(context)
    use_open3d_val = LaunchConfiguration('use_open3d').perform(context)
    auto_run_val = LaunchConfiguration('auto_run').perform(context)
    baseline_dir_val = LaunchConfiguration('baseline_dir').perform(context)
    num_point_val = LaunchConfiguration('num_point').perform(context)
    num_view_val = LaunchConfiguration('num_view').perform(context)
    collision_thresh_val = LaunchConfiguration('collision_thresh').perform(context)
    voxel_size_val = LaunchConfiguration('voxel_size').perform(context)
    max_grasps_num_val = LaunchConfiguration('max_grasps_num').perform(context)
    gpu_val = LaunchConfiguration('gpu').perform(context)
    trigger_service_val = LaunchConfiguration('trigger_service').perform(context)
    camera_id_val = LaunchConfiguration('camera_id').perform(context)
    color_image_topic_val = LaunchConfiguration('color_image_topic').perform(context)
    depth_image_topic_val = LaunchConfiguration('depth_image_topic').perform(context)
    camera_info_topic_val = LaunchConfiguration('camera_info_topic').perform(context)
    factor_depth_val = LaunchConfiguration('factor_depth').perform(context)
    trigger_grasp_service_val = LaunchConfiguration('trigger_grasp_service').perform(context)
    
    # 构建参数字符串（含手眼标定路径与目标坐标系，用于将抓取位姿和 MarkerArray 变换到机器人基座）
    demo_params = [
        '--ros-args',
        '-r', '__node:=graspnet_demo_node',
        '-p', f'baseline_dir:={baseline_dir_val}',
        '-p', f'model_path:={model_path_val}',
        '-p', f'data_dir:={data_dir_val}',
        '-p', f'marker_topic:={marker_topic_val}',
        '-p', f'pointcloud_topic:={pointcloud_topic_val}',
        '-p', f'frame_id:={frame_id_val}',
        '-p', f'hand_eye_yaml_path:={hand_eye_yaml_val}',
        '-p', f'use_open3d:={use_open3d_val}',
        '-p', f'auto_run:={auto_run_val}',
        '-p', f'num_point:={num_point_val}',
        '-p', f'num_view:={num_view_val}',
        '-p', f'collision_thresh:={collision_thresh_val}',
        '-p', f'voxel_size:={voxel_size_val}',
        '-p', f'max_grasps_num:={max_grasps_num_val}',
        '-p', f'gpu:={gpu_val}',
        '-p', f'trigger_service:={trigger_service_val}',
        '-p', f'camera_id:={camera_id_val}',
        '-p', f'color_image_topic:={color_image_topic_val}',
        '-p', f'depth_image_topic:={depth_image_topic_val}',
        '-p', f'camera_info_topic:={camera_info_topic_val}',
        '-p', f'factor_depth:={factor_depth_val}',
        '-p', f'trigger_grasp_service:={trigger_grasp_service_val}',
    ]
    
    cwd = src_package_path if os.path.exists(src_package_path) else os.path.dirname(demo_script_path) if demo_script_path else package_path
    
    # ExecuteProcess 默认继承当前环境（source setup.bash 后的 PATH/PYTHONPATH）
    return [ExecuteProcess(
        cmd=[python_exec, demo_script_path] + demo_params,
        output='screen',
        cwd=cwd,
    )]


def generate_launch_description():
    package_path = get_package_share_directory('graspnet_ros2')
    baseline_dir = os.path.join(package_path, 'graspnet-baseline')
    
    # 声明启动参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar'),
            description='模型权重文件路径',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'data_dir',
            default_value=os.path.join(baseline_dir, 'doc', 'pose_1'),
            description='数据目录路径（包含 color.png, depth.png, workspace_mask.png, meta.mat）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'marker_topic',
            default_value='grasp_markers',
            description='MarkerArray 话题名称',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='graspnet_pointcloud',
            description='点云话题名称',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_frame',
            description='抓取位姿与 MarkerArray 的坐标系（GraspNet 预测在相机系，保持 camera_frame 让 TF 树自动处理变换）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ee_frame_id',
            default_value='wrist3_Link',
            description='机械臂末端 link 名称，用于发布 末端->camera_frame 静态 TF（与手眼标定 相机→末端 一致）',
        )
    )
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
            description='手眼标定 YAML 路径（相机到机械臂末端的变换矩阵，用于抓取位姿与 TF）',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz2',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'graspnet_python_executable',
            default_value='',
            description='运行 graspnet_demo_node 的 Python 解释器路径。若使用 conda graspnet 环境（含 pointnet2._ext），可设为该环境的 python，例如 /home/mu/miniconda3/envs/graspnet/bin/python3；空则使用当前 ros2 launch 的 Python',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value='moveit.rviz',
            description='RViz2 配置文件名（置于 aubo2_moveit_config/config/ 下）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_open3d',
            default_value='true',  # 默认启用 Open3D 可视化
            description='是否启用 Open3D 可视化',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'auto_run',
            default_value='false',  # 默认手动模式，需要通过服务触发
            description='是否自动运行（false 时需要调用 /publish_grasps 服务手动触发发布）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'baseline_dir',
            default_value=baseline_dir,
            description='graspnet-baseline 目录路径（用于设置 Python 导入路径）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_point',
            default_value='20000',
            description='点云采样数量',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_view',
            default_value='300',
            description='视角数量',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'collision_thresh',
            default_value='0.01',
            description='碰撞检测阈值',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.01',
            description='体素大小',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'max_grasps_num',
            default_value='20',
            description='发布的最大抓取数量',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'gpu',
            default_value='0',
            description='GPU 设备 ID',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'trigger_service',
            default_value='/software_trigger',
            description='相机软触发服务名',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_id',
            default_value='207000152740',
            description='相机 ID（与 hand_eye 一致）',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'color_image_topic',
            default_value='/camera/color/image_raw',
            description='彩色图话题',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/depth/image_raw',
            description='深度图话题',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='相机内参话题',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'factor_depth',
            default_value='4000.0',
            description='深度缩放因子（Percipio 0.25mm 用 4000，毫米用 1000）',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'trigger_grasp_service',
            default_value='trigger_grasp',
            description='触发抓取服务名',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_cpp_tutorial',
            default_value='false',
            description='是否启动 MoveIt C++ 教程节点（需在 launch 内启动并传入 robot_description 等参数，因 ROS2 参数按节点隔离）',
        )
    )
    
    # GraspNet Demo 节点（使用 OpaqueFunction，内部传入 hand_eye_yaml_path 与 frame_id）
    demo_node = OpaqueFunction(function=create_demo_node)
    
    # 机械臂模型 + ros2_control 仿真 + base_link->camera_frame 静态 TF + RViz2
    robot_and_tf = OpaqueFunction(function=launch_setup_robot_and_tf)
    
    return LaunchDescription(
        declared_arguments + [
            demo_node,
            robot_and_tf,
        ]
    )
