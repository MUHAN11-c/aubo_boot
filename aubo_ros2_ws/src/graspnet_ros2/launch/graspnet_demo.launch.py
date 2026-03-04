#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo Launch 文件

功能：
  1. 启动 Aubo 机械臂模型（参考 aubo_moveit_bridge_ros1.launch.py）与 ros2_control 仿真
  2. 启动 graspnet_demo_node（从文件读取数据并预测抓取，发布 MarkerArray、点云和 TF）
  3. 发布手眼标定静态 TF（末端 ee_frame_id -> camera_frame），标定矩阵为 相机→末端，来自 hand_eye_calibration_best.yaml
  4. 启动 rviz2 可视化工具
  5. 默认同时启动 Gazebo（gazebo_ros 的 gazebo.launch.py，与参考一致只传 world）与 RViz2

使用方法（请先 source ROS2 与工作空间，使用系统 Python 以匹配 ROS Humble 的 rclpy）：
  cd /path/to/aubo_ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch graspnet_ros2 graspnet_demo.launch.py

  或使用脚本： ./launch/run_graspnet_demo.sh
  
手动触发发布抓取结果：
  ros2 run graspnet_ros2 publish_grasps_client

若 graspnet_demo_node 报错 No module named 'pointnet2._ext'，说明需在已编译 Pointnet2 的 conda 环境中运行，请使用：
  conda activate graspnet
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch graspnet_ros2 graspnet_demo.launch.py graspnet_python_executable:=/home/mu/miniconda3/envs/graspnet/bin/python3
（graspnet 环境中需已安装 rclpy，例如：pip install rclpy）
  
可选参数示例： 
  use_open3d:=true  data_dir:=/path/to/data  auto_run:=false
  use_gazebo:=false 仅启动 RViz2、不启动 Gazebo（默认 true 为 Gazebo + RViz2 联合仿真）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    """启动机械臂模型（robot_state_publisher）、ros2_control 仿真、可选 MoveIt2 move_group、以及 末端->camera_frame 静态 TF。"""
    support_package = 'aubo_description'
    moveit_config_package = 'aubo_moveit_config'
    robot_xacro_to_use = 'aubo_ros2.xacro'
    aubo_type = LaunchConfiguration('aubo_type').perform(context)
    hand_eye_yaml = LaunchConfiguration('hand_eye_yaml_path').perform(context)
    ee_frame_id = LaunchConfiguration('ee_frame_id').perform(context)
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context) == 'true'
    use_moveit = LaunchConfiguration('use_moveit').perform(context) == 'true'
    use_move_to_pose_server = LaunchConfiguration('use_move_to_pose_server').perform(context) == 'true'
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context) == 'true'

    # 与参考一致：启用 Gazebo 时使用 use_sim_time，与 /clock 同步
    sim_time_param = {'use_sim_time': True} if use_gazebo else {}

    nodes = []

    # 机械臂描述（与 aubo_moveit_bridge_ros1.launch.py 一致，使用 ros2_control 版 xacro）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(support_package), 'urdf/xacro/inc/', robot_xacro_to_use
        ]),
        ' ', 'aubo_type:=', aubo_type, ' ',
    ])
    robot_description = {'robot_description': robot_description_content}

    # robot_state_publisher（参考：先于 Gazebo 与 spawn 启动，供 -topic robot_description 使用）
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, sim_time_param],
    ))

    # ros2_control 仿真
    if use_ros2_control:
        ros2_controllers_path = os.path.join(
            get_package_share_directory(moveit_config_package), 'config', 'ros2_controllers.yaml'
        )
        nodes.append(Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ros2_controllers_path, sim_time_param],
            output='screen',
        ))
        # 延迟启动控制器（Load -> Configure -> Switch，控制器须先 configure 成 inactive 才能被 activate）
        nodes.append(TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                         'ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: joint_state_broadcaster}" && '
                         'ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: joint_state_broadcaster}" && '
                         'ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: joint_trajectory_controller}" && '
                         'ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: joint_trajectory_controller}" && '
                         'ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [\'joint_state_broadcaster\', \'joint_trajectory_controller\'], deactivate_controllers: [], strictness: 2, activate_asap: true, timeout: {sec: 0, nanosec: 0}}"'],
                    output='screen',
                ),
            ],
        ))

    # 静态 TF：末端(ee_frame_id) -> camera_frame（手眼标定矩阵为 相机→末端 T_ee_camera）
    tf_args = load_hand_eye_static_tf_args(hand_eye_yaml)
    if tf_args is not None:
        x, y, z, yaw, pitch, roll = tf_args
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_publisher',
            arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                       '--yaw', str(yaw), '--pitch', str(pitch), '--roll', str(roll),
                       '--frame-id', ee_frame_id, '--child-frame-id', 'camera_frame'],
            output='screen',
        ))
    else:
        # 无标定文件时使用单位变换，便于仅做可视化
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', ee_frame_id, '--child-frame-id', 'camera_frame'],
            output='screen',
        ))

    # MoveIt2 move_group（可选）：启用后 RViz 中可使用 MotionPlanning 规划与预览轨迹
    if use_moveit:
        srdf_path = os.path.join(
            get_package_share_directory(moveit_config_package), 'config', 'aubo_i5.srdf'
        )
        with open(srdf_path, 'r') as f:
            robot_description_semantic = {'robot_description_semantic': f.read()}
        kinematics_yaml = load_yaml(moveit_config_package, 'config/kinematics.yaml')
        joint_limits_yaml = {'robot_description_planning': load_yaml(moveit_config_package, 'config/joint_limits.yaml')}
        ompl_planning_pipeline_config = {
            'move_group': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1,
                'sample_duration': 0.005,
                'planning_time': 5.0,
                'max_planning_attempts': 10,
            }
        }
        ompl_yaml = load_yaml(moveit_config_package, 'config/ompl_planning.yaml')
        if ompl_yaml:
            ompl_planning_pipeline_config['move_group'].update(ompl_yaml)
        moveit_controllers_yaml = load_yaml(moveit_config_package, 'config/moveit_controllers.yaml')
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
        nodes.append(Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
                joint_limits_yaml,
                sim_time_param,
            ],
        ))

    # MoveIt2 MoveToPose 服务节点（可选）：用于控制机械臂运动到目标位姿
    if use_move_to_pose_server and use_moveit:
        nodes.append(TimerAction(
            period=10.0,  # 在 move_group 启动后延迟启动
            actions=[
                Node(
                    package='demo_driver',
                    executable='move_to_pose_server_node',
                    name='move_to_pose_server_node',
                    output='screen',
                    parameters=[
                        robot_description,
                        robot_description_semantic,
                        sim_time_param,
                    ],
                )
            ],
        ))

    # RViz2（demo.rviz 已含 MotionPlanning；启用 MoveIt2 时需传入 moveit 参数供插件使用）
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if use_rviz and rviz_config and os.path.exists(rviz_config):
        rviz_params = [robot_description, sim_time_param]
        if use_moveit:
            rviz_params.extend([
                robot_description_semantic,
                ompl_planning_pipeline_config,
                kinematics_yaml,
                joint_limits_yaml,
            ])
        nodes.append(TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config],
                    parameters=rviz_params,
                )
            ],
        ))

    # 与参考一致：Gazebo 与 spawn 放在最后；spawn 从 -topic robot_description 读 URDF
    # 延迟 spawn，等 gzserver 加载 GazeboRosFactory 并发布 /spawn_entity 后再执行，避免 "Service /spawn_entity unavailable"
    if use_gazebo:
        gazebo_share = FindPackageShare('gazebo_ros')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([gazebo_share, 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments=[
                ('world', PathJoinSubstitution([gazebo_share, 'worlds', 'empty.world'])),
            ],
        ))
        nodes.append(TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_aubo',
                    arguments=['-entity', 'aubo_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.5'],
                    output='screen',
                )
            ],
        ))

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
    
    # 继承当前环境（source /opt/ros/humble/setup.bash 后的 PATH/PYTHONPATH）
    env = dict(os.environ)
    
    cwd = src_package_path if os.path.exists(src_package_path) else os.path.dirname(demo_script_path) if demo_script_path else package_path
    
    return [ExecuteProcess(
        cmd=[python_exec, demo_script_path] + demo_params,
        output='screen',
        cwd=cwd,
        env=env,
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
            'aubo_type',
            default_value='aubo_e5_10',
            description='Aubo 机械臂型号',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='是否启动 ros2_control 仿真（joint_state_broadcaster + joint_trajectory_controller）',
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
            'use_moveit',
            default_value='true',
            description='是否启动 MoveIt2 move_group；为 true 时使用 demo_moveit.rviz（含 MotionPlanning 与抓取 Marker/点云）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_move_to_pose_server',
            default_value='true',
            description='是否启动 MoveToPose 服务节点（用于控制机械臂运动到抓取位姿）',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='是否同时启动 Gazebo 空世界，与 RViz2 联合仿真（启用后依赖 Gazebo Classic 与 gazebo_ros）',
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
            default_value=os.path.join(package_path, 'config', 'demo.rviz'),
            description='RViz2 配置文件路径',
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
