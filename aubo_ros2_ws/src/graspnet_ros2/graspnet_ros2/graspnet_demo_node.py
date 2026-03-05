#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo 节点：从文件读取数据，预测抓取并发布 MarkerArray 和 TF。

功能：
  1. 从指定目录读取 color.png, depth.png, workspace_mask.png；相机内参从 config/camera_meta.yaml 读取
  2. 使用预训练 GraspNet 模型预测抓取位姿
  3. 碰撞检测、NMS、排序
  4. 发布一次 MarkerArray、点云和 TF 到 RViz2
  5. 发布抓取位姿的 TF 变换（camera_frame -> grasp_pose_X）
  6. 可选：Open3D 可视化
  
说明：
  - 静态场景模式：拍照一次，计算一次，发布一次
  - MarkerArray、点云和 TF 不会随机械臂移动而跟随
  - TF 可用于运动规划和位姿查询
"""

# 导入系统和路径相关库
import os  # 文件和目录操作
import sys  # 系统相关功能,用于修改 Python 路径

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'graspnet-baseline'))
sys.path.insert(0, ROOT_DIR)                            # 用于 from models.xxx、graspnetAPI 等
sys.path.insert(0, os.path.join(ROOT_DIR, 'models'))   # 优先搜索 models 目录
sys.path.insert(0, os.path.join(ROOT_DIR, 'dataset'))  # 优先搜索 dataset 目录
sys.path.append(os.path.join(ROOT_DIR, 'utils'))       # 将 utils 目录加入搜索路径末尾
# ========== 导入 GraspNet 相关模块（路径由文件顶部 ROOT_DIR 注入 sys.path）==========
from models.graspnet import GraspNet, pred_decode  #graspnet模型和预测解码函数
from utils.collision_detector import ModelFreeCollisionDetector  #无模型碰撞检测器
from utils.data_utils import CameraInfo, create_point_cloud_from_depth_image  #相机信息封装与点云生成工具
from graspnetAPI import GraspGroup  # 抓取组数据结构

import rclpy  # ROS 2 Python 客户端库
from rclpy.node import Node  # ROS 2 节点基类

# 导入 ROS 2 消息类型
from visualization_msgs.msg import Marker, MarkerArray  # RViz 可视化标记
from geometry_msgs.msg import Pose, TransformStamped  # 位姿消息(位置+方向)和变换消息
from builtin_interfaces.msg import Duration, Time  # 时间和持续时间消息
from sensor_msgs.msg import PointCloud2, PointField  # 点云消息类型
from std_msgs.msg import Header  # 消息头
from std_srvs.srv import Trigger  # 触发服务类型
from tf2_ros import TransformBroadcaster  # TF2 变换广播器

# 导入科学计算和机器学习库
import numpy as np  # 数值计算库
import torch  # PyTorch 深度学习框架
import time  # 时间相关功能
import multiprocessing  # 多进程支持,用于 Open3D 可视化不阻塞 ROS2
from scipy.spatial.transform import Rotation  # 旋转变换工具
from PIL import Image  # 图像处理库
import open3d as o3d  # 3D 数据处理和可视化
import yaml  # YAML 配置文件解析


def _vis_grasps_open3d_process(gg, cloud):
    """在独立进程中运行 Open3D 可视化的函数
    
    功能：
        在独立进程中显示 Open3D 可视化窗口
    
    参数：
        gg: GraspGroup 对象,包含要可视化的抓取
        cloud: Open3D 点云对象
    
    说明：
        - 这个函数在独立进程中执行,不会影响 ROS2 主进程
        - 必须是模块级函数（不是类方法）以支持 multiprocessing
        - 可视化窗口关闭后进程自动结束
        - 点云与夹爪直接使用 GraspNet 输出,不做坐标变换(Open3D 与 GraspNet 约定一致)
    """
    try:
        # ===== 预处理抓取 =====
        gg = gg.nms()
        gg.sort_by_score()
        num_grasps_to_show = min(10, len(gg)) # 最多显示 10 个抓取
        gg_vis = gg[:num_grasps_to_show]
        
        # ===== 生成抓取几何体 =====
        grippers = gg_vis.to_open3d_geometry_list() 
        
        # ===== 组合并显示(不变换坐标系) =====
        geometries = [cloud] + grippers
        
        o3d.visualization.draw_geometries(  
            geometries,
            window_name='GraspNet visualization',
            width=640,
            height=480,
            point_show_normal=False 
        )
        
    except Exception as e:
        print(f'Open3D 可视化失败: {str(e)}')
        import traceback # 导入异常堆栈信息打印函数
        traceback.print_exc() # 打印异常堆栈信息


class GraspNetDemoNode(Node):
    """GraspNet ROS2 Demo 节点类
    
    功能：
        - 从文件读取 RGB-D 图像和相机参数
        - 使用 GraspNet 模型预测抓取位姿
        - 进行碰撞检测和非极大值抑制(NMS)
        - 将抓取结果以 MarkerArray 形式发布一次到 RViz2
        - 发布抓取位姿的 TF 变换（camera_frame -> grasp_pose_X）
        - 可选的 Open3D 三维可视化
    
    说明：
        - 静态场景模式：只计算和发布一次结果
        - MarkerArray、点云和 TF 不会定期重新发布
        - TF 变换可用于后续的运动规划
    
    继承自：
        rclpy.node.Node: ROS 2 节点基类
    """
    
    def __init__(self):
        """节点初始化函数
        
        功能：
            1. 声明和获取 ROS 2 参数
            2. 加载手眼标定数据
            3. 设置 Python 路径
            4. 导入 GraspNet 相关模块
            5. 加载预训练模型
            6. 创建 ROS 2 发布器和定时器
        """
        # 调用父类初始化,设置节点名称为 'graspnet_demo_node'
        super().__init__('graspnet_demo_node')
        
        # ========== 声明 ROS 2 参数 ==========
        # 这些参数将从 launch 文件传入
        
        # GraspNet 相关路径参数（baseline_dir 默认使用文件顶部固定的 ROOT_DIR）
        self.declare_parameter('baseline_dir', ROOT_DIR)
        self.declare_parameter('model_path', '')  # 模型权重文件路径
        self.declare_parameter('data_dir', '')  # 输入数据目录路径
        
        # 算法参数
        self.declare_parameter('num_point', 20000)  # 点云采样数量
        self.declare_parameter('num_view', 300)  # 抓取视角数量
        self.declare_parameter('collision_thresh', 0.01)  # 碰撞检测阈值
        self.declare_parameter('voxel_size', 0.01)  # 体素网格大小(米)
        self.declare_parameter('max_grasps_num', 20)  # 最大发布抓取数量
        self.declare_parameter('gpu', 0)  # GPU 设备 ID
        
        # ROS 2 发布相关参数
        self.declare_parameter('marker_topic', 'grasp_markers')  # MarkerArray 话题名
        self.declare_parameter('pointcloud_topic', 'graspnet_pointcloud')  # 点云话题名
        self.declare_parameter('frame_id', 'camera_frame')  # 坐标系 ID
        
        # 手眼标定参数
        self.declare_parameter('hand_eye_yaml_path', '')  # 手眼标定文件路径
        # 相机内参（替代 meta.mat）
        self.declare_parameter('camera_meta_yaml', '')  # 相机内参 YAML 路径，空则使用 config/camera_meta.yaml
        
        # 可视化参数
        self.declare_parameter('use_open3d', False)  # 是否启用 Open3D 可视化
        
        # 运行模式参数
        self.declare_parameter('auto_run', False)  # 是否自动运行（False时需要通过服务手动触发）
        
        # ========== 获取参数值 ==========
        # 从参数服务器读取所有参数的实际值
        
        baseline_dir = self.get_parameter('baseline_dir').get_parameter_value().string_value or ROOT_DIR
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        # data_dir 为空或目录不存在时，回退到 baseline_dir/doc/pose_1（与 demo.py 默认一致）
        if not self.data_dir or not os.path.isdir(self.data_dir):
            fallback = os.path.join(baseline_dir, 'doc', 'pose_1')
            if os.path.isdir(fallback):
                self.data_dir = fallback
                self.get_logger().info(f'data_dir 使用回退路径: {self.data_dir}')
        self.num_point = self.get_parameter('num_point').get_parameter_value().integer_value
        self.num_view = self.get_parameter('num_view').get_parameter_value().integer_value
        self.collision_thresh = self.get_parameter('collision_thresh').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.max_grasps_num = self.get_parameter('max_grasps_num').get_parameter_value().integer_value
        self.gpu = self.get_parameter('gpu').get_parameter_value().integer_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.hand_eye_yaml_path = self.get_parameter('hand_eye_yaml_path').get_parameter_value().string_value
        _camera_meta_yaml = self.get_parameter('camera_meta_yaml').get_parameter_value().string_value
        self.camera_meta_yaml_path = self._resolve_camera_meta_yaml_path(_camera_meta_yaml, baseline_dir)
        self.use_open3d = self.get_parameter('use_open3d').get_parameter_value().bool_value
        self.auto_run = self.get_parameter('auto_run').get_parameter_value().bool_value

        # ========== 加载手眼标定矩阵 ==========
        # T_ee_camera: 相机坐标系到机械臂末端坐标系的齐次变换矩阵 (4x4)
        self.T_ee_camera = None
        if self.hand_eye_yaml_path:  # 如果提供了手眼标定文件路径
            # 从 YAML 文件加载变换矩阵
            self.T_ee_camera = self._load_hand_eye_transform(self.hand_eye_yaml_path)
            if self.T_ee_camera is not None:  # 加载成功
                self.get_logger().info(
                    f'已加载手眼标定矩阵（相机→末端），TF 静态变换将通过 launch 文件发布'
                )
            else:  # 加载失败
                self.get_logger().warn('手眼标定矩阵加载失败')
        
        # 记录坐标系信息
        self.get_logger().info(f'MarkerArray 将在 {self.frame_id} 坐标系发布，坐标变换由 TF 树自动处理')
        
        # 将导入的类和函数保存为实例变量,供其他方法使用
        self.GraspNet = GraspNet  # 模型类
        self.pred_decode = pred_decode  # 预测解码函数
        self.ModelFreeCollisionDetector = ModelFreeCollisionDetector  # 碰撞检测器类
        self.CameraInfo = CameraInfo  # 相机参数类
        self.create_point_cloud_from_depth_image = create_point_cloud_from_depth_image  # 点云生成函数
        self.GraspGroup = GraspGroup  # 抓取组类
        
        # ========== 设置计算设备 ==========
        # 如果有可用的 CUDA GPU,则使用 GPU;否则使用 CPU
        self.device = torch.device(f"cuda:{self.gpu}" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"使用设备: {self.device}")
        
        # ========== 加载预训练模型 ==========
        self.net = self._load_model()  # 调用模型加载函数
        
        # ========== 创建 ROS 2 发布器 ==========
        # 创建 MarkerArray 发布器,队列大小为 10
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        # 创建 PointCloud2 发布器,用于发布场景点云
        self.pointcloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        # 创建 TF 广播器,用于发布抓取位姿的坐标变换
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ========== 初始化状态变量 ==========
        self.draw_grasp_id = 0  # Marker ID 计数器,用于给每个 Marker 分配唯一 ID
        self.processed_gg = None  # 缓存处理后的抓取结果
        self.cloud_o3d = None  # 缓存点云数据
        self.has_computed = False  # 是否已经计算过抓取
        
        # ========== 创建 ROS 2 服务 ==========
        # 创建发布服务，允许手动触发发布
        self.publish_srv = self.create_service(
            Trigger,
            'publish_grasps',
            self.publish_service_callback
        )
        
        # ========== 输出节点启动信息 ==========
        self.get_logger().info('GraspNet Demo 节点已启动')
        self.get_logger().info(f'数据目录: {self.data_dir}')
        self.get_logger().info(f'发布话题: {self.marker_topic}')
        self.get_logger().info(f'点云话题: {self.pointcloud_topic}')
        self.get_logger().info(f'坐标系: {self.frame_id}')
        self.get_logger().info(f'自动运行: {self.auto_run}')
        self.get_logger().info(f'发布服务: publish_grasps')
        self.get_logger().info(f'TF 发布: {self.frame_id} -> grasp_pose_X')
        self.get_logger().info(f'相机内参配置: {self.camera_meta_yaml_path}')
        
        # ========== 创建定时器（可选自动运行）==========
        if self.auto_run:
            # 自动模式：创建单次执行的定时器,1秒后执行 run_once 函数
            self.timer = self.create_timer(1.0, self.run_once)
            self.run_count = 0  # 运行计数器,确保只运行一次
            self.get_logger().info('自动模式：将在 1 秒后自动计算并发布抓取')
        else:
            # 手动模式：不自动运行，等待服务调用
            self.get_logger().info('手动模式：请调用 /publish_grasps 服务来计算并发布抓取')
    
    def _resolve_camera_meta_yaml_path(self, param_path, baseline_dir):
        """解析相机内参 YAML 路径：参数 > 包 share/config > 源码 config。"""
        if param_path and os.path.isfile(param_path):
            return param_path
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('graspnet_ros2')
            candidate = os.path.join(pkg_share, 'config', 'camera_meta.yaml')
            if os.path.isfile(candidate):
                return candidate
        except Exception:
            pass
        # 源码运行：graspnet_ros2/graspnet_ros2/xxx.py -> graspnet_ros2/config/camera_meta.yaml
        this_dir = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.normpath(os.path.join(os.path.dirname(this_dir), '..', 'config', 'camera_meta.yaml'))
        if os.path.isfile(candidate):
            return candidate
        raise FileNotFoundError(
            '未找到 camera_meta.yaml。请设置参数 camera_meta_yaml 或确保 config/camera_meta.yaml 存在于 graspnet_ros2 包中。'
        )
    
    def _load_model(self):
        """加载 GraspNet 预训练模型
        
        功能：
            1. 检查模型文件是否存在
            2. 创建 GraspNet 网络结构
            3. 加载预训练权重
            4. 设置为评估模式
        
        返回：
            torch.nn.Module: 加载完成的 GraspNet 模型
        
        异常：
            FileNotFoundError: 模型文件不存在时抛出
        """
        self.get_logger().info(f'正在加载模型: {self.model_path}')
        
        # 检查模型文件是否存在
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'模型文件不存在: {self.model_path}')
            raise FileNotFoundError(f'模型文件不存在: {self.model_path}')
        
        # 创建 GraspNet 模型实例
        net = self.GraspNet(
            input_feature_dim=0,  # 输入特征维度(0表示只使用XYZ坐标)
            num_view=self.num_view,  # 抓取视角数量
            num_angle=12,  # 抓取角度离散化数量
            num_depth=4,  # 抓取深度离散化数量
            cylinder_radius=0.05,  # 圆柱采样半径(米)
            hmin=-0.02,  # 最小抓取高度(米)
            hmax_list=[0.01, 0.02, 0.03, 0.04],  # 不同深度级别的最大抓取高度列表
            is_training=False,  # 设置为评估模式
        )
        net.to(self.device)  # 将模型移动到指定计算设备(CPU/GPU)
        
        # 加载预训练权重
        checkpoint = torch.load(self.model_path, map_location=self.device)
        net.load_state_dict(checkpoint['model_state_dict'])  # 加载模型参数
        net.eval()  # 设置为评估模式(关闭 dropout 和 batch normalization)
        
        self.get_logger().info('模型加载完成')
        return net

    def _load_hand_eye_transform(self, yaml_path):
        """从手眼标定 YAML 文件中加载变换矩阵
        
        功能：
            从 YAML 配置文件读取相机到机械臂末端的齐次变换矩阵
        
        参数：
            yaml_path: 手眼标定 YAML 文件路径
        
        返回：
            np.ndarray: 4x4 齐次变换矩阵 T_ee_camera,如果加载失败返回 None
        
        说明：
            - YAML 中的平移单位为毫米,这里会转换为米
            - 变换矩阵定义：相机坐标系 → 机械臂末端坐标系
            - 用于将相机坐标系下的抓取位姿变换到机械臂坐标系
        """
        try:
            # 检查文件是否存在
            if not os.path.exists(yaml_path):
                self.get_logger().error(f'手眼标定文件不存在: {yaml_path}')
                return None

            # 读取 YAML 文件
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)  # 安全加载 YAML 数据

            # 提取手眼标定部分
            he = data.get('hand_eye_calibration', {})
            T_list = he.get('transformation_matrix', None)
            
            # 检查是否找到变换矩阵
            if T_list is None:
                self.get_logger().error('hand_eye_calibration.transformation_matrix 字段不存在')
                return None

            # 将列表转换为 NumPy 数组
            T = np.array(T_list, dtype=np.float64)
            
            # 检查矩阵形状是否正确(应该是4x4)
            if T.shape != (4, 4):
                self.get_logger().error(f'手眼标定矩阵形状错误: {T.shape}, 期望 (4, 4)')
                return None

            # 将平移部分从毫米转换为米,以与点云/抓取位姿的单位一致
            # T[:3, 3] 是变换矩阵的平移向量(x, y, z)
            T[:3, 3] /= 1000.0

            return T  # 返回变换矩阵
            
        except Exception as e:
            # 捕获所有异常并记录
            self.get_logger().error(f'读取手眼标定文件失败: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())  # 输出详细错误堆栈
            return None
    
    def publish_service_callback(self, request, response):
        """发布服务回调函数
        
        功能：
            响应服务请求，计算并发布抓取结果
        
        参数：
            request: Trigger.Request (空请求)
            response: Trigger.Response (包含 success 和 message)
        
        返回：
            response: 服务响应
        """
        try:
            self.get_logger().info('收到发布请求，开始计算抓取...')
            
            # 如果还没有计算过，先计算
            if not self.has_computed:
                self._compute_grasps()
            
            # 发布结果
            if self.processed_gg is not None and self.cloud_o3d is not None:
                self.publish_marker_array(self.processed_gg)
                self.publish_pointcloud(self.cloud_o3d)
                
                # 构建返回的抓取数据（返回最佳抓取）
                import json
                grasp_data = []
                for i in range(len(self.processed_gg)):
                    grasp = self.processed_gg[i]
                    grasp_info = {
                        'index': i,
                        'score': float(grasp.score),
                        'width': float(grasp.width),
                        'height': float(grasp.height),
                        'depth': float(grasp.depth),
                        'translation': [float(grasp.translation[0]), 
                                       float(grasp.translation[1]), 
                                       float(grasp.translation[2])],
                        'rotation_matrix': grasp.rotation_matrix.flatten().tolist(),
                        'object_id': int(grasp.object_id)
                    }
                    grasp_data.append(grasp_info)
                
                response.success = True
                response.message = json.dumps({
                    'status': 'success',
                    'num_grasps': len(self.processed_gg),
                    'grasps': grasp_data
                })
                self.get_logger().info(f'成功发布 {len(self.processed_gg)} 个抓取和点云，已返回抓取数据')
            else:
                response.success = False
                response.message = '没有可发布的抓取数据'
                self.get_logger().warn(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f'发布失败: {str(e)}'
            self.get_logger().error(response.message)
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        return response
    
    def _compute_grasps(self):
        """计算抓取位姿（内部方法）
        
        功能：
            1. 读取和处理输入数据
            2. 预测抓取位姿
            3. 碰撞检测
            4. NMS 和排序
            5. 缓存结果
            6. 可选的 Open3D 可视化
        """
        try:
            self.get_logger().info('开始计算抓取...')
            
            # ===== 步骤1: 读取并处理数据 读取RGB图像、深度图像、工作空间掩码 =====
            end_points, cloud = self._get_and_process_data()
            
            # ===== 步骤2: 预测抓取 使用GraspNet模型预测抓取位姿 =====
            gg = self._get_grasps(end_points)
            
            # ===== 步骤3: 碰撞检测 =====
            if self.collision_thresh > 0:  # 如果设置了碰撞检测阈值
                gg = self._collision_detection(gg, np.array(cloud.points))
            
            # ===== 步骤4: 后处理 =====
            # NMS (非极大值抑制): 移除重复的抓取
            processed_gg = gg.nms()
            # 按分数排序: 将抓取按质量分数从高到低排序
            processed_gg.sort_by_score()
            # 限制数量: 只保留前 max_grasps_num 个最好的抓取
            processed_gg = processed_gg[:self.max_grasps_num]
            
            # ===== 缓存结果 =====
            self.processed_gg = processed_gg # 缓存抓取结果
            self.cloud_o3d = cloud # 缓存点云
            self.has_computed = True # 标记已计算
            
            self.get_logger().info(f'抓取计算完成，共 {len(processed_gg)} 个抓取')
            
            # ===== Open3D 可视化（可选）=====
            if self.use_open3d:  # 如果启用了 Open3D 可视化
                self.get_logger().info('正在启动 Open3D 可视化进程...')
                # 在单独的进程中运行 Open3D 可视化,完全独立于 ROS2 主进程
                vis_process = multiprocessing.Process(
                    target=_vis_grasps_open3d_process,
                    args=(processed_gg, cloud),
                    daemon=True  # 设置为守护进程,主程序退出时自动结束
                )
                vis_process.start()
                self.get_logger().info(f'Open3D 可视化进程已启动 (PID: {vis_process.pid})')
            else:  # 未启用 Open3D
                self.get_logger().info('Open3D 可视化已禁用（使用 use_open3d:=true 启用）')
            
        except Exception as e:
            # 捕获所有异常并记录
            self.get_logger().error(f'抓取计算失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise
    
    def run_once(self):
        """自动模式下执行一次抓取预测
        
        功能：
            在自动模式下，定时器调用此函数计算并发布抓取
        
        说明：
            - 使用 run_count 确保只运行一次
            - 运行后会取消定时器
        """
        # 如果已经运行过,直接返回
        if self.run_count > 0:
            return
        
        # 增加运行计数
        self.run_count += 1
        # 取消定时器,防止重复执行
        self.timer.cancel()
        
        try:
            # 计算抓取
            self._compute_grasps()
            
            # 发布结果
            self.publish_marker_array(self.processed_gg)
            self.publish_pointcloud(self.cloud_o3d)
            
            self.get_logger().info('自动模式：预测和发布完成')
            
        except Exception as e:
            # 捕获所有异常并记录
            self.get_logger().error(f'自动模式预测失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _get_and_process_data(self):
        """从文件读取数据并处理成模型输入格式
        
        功能：
            1. 读取 RGB 图像、深度图像、工作空间掩码
            2. 读取相机内参矩阵
            3. 生成点云
            4. 应用掩码筛选有效点
            5. 采样固定数量的点
        
        返回：
            tuple: (end_points, cloud_o3d)
                - end_points: 包含模型输入数据的字典
                - cloud_o3d: Open3D 点云对象(用于可视化)
        """
        self.get_logger().info(f'正在读取数据: {self.data_dir}')
        
        # ===== 读取图像和相机参数 =====
        # 读取 RGB 图像并归一化到 [0, 1]
        color = np.array(Image.open(os.path.join(self.data_dir, 'color.png')), dtype=np.float32) / 255.0
        # 读取深度图像(单位通常是毫米)
        depth = np.array(Image.open(os.path.join(self.data_dir, 'depth.png')))
        # 读取工作空间掩码(标记哪些区域是有效的工作空间)
        workspace_mask = np.array(Image.open(os.path.join(self.data_dir, 'workspace_mask.png')))
        # 从 config/camera_meta.yaml 读取相机内参（替代 meta.mat）
        with open(self.camera_meta_yaml_path, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        intrinsic = np.array(meta['intrinsic_matrix'], dtype=np.float64)
        factor_depth = float(meta['factor_depth'])
        
        # ===== 生成点云 =====
        # 获取图像尺寸
        height, width = depth.shape[0], depth.shape[1]
        # 创建相机参数对象
        camera = self.CameraInfo(
            width, height,  # 图像宽度和高度
            intrinsic[0][0], intrinsic[1][1],  # fx, fy (焦距)
            intrinsic[0][2], intrinsic[1][2],  # cx, cy (主点)
            factor_depth  # 深度缩放因子
        )
        # 从深度图生成点云(organized=True 保持图像结构)
        cloud = self.create_point_cloud_from_depth_image(depth, camera, organized=True)
        
        # ===== 有效点筛选 =====
        # 创建有效点掩码: 工作空间内 且 深度值大于0
        mask = (workspace_mask & (depth > 0))
        # 应用掩码提取有效点
        cloud_masked = cloud[mask]  # 有效点的 3D 坐标
        color_masked = color[mask]  # 有效点的颜色
        
        # ===== 点云采样 =====
        # 固定随机种子，使与 demo.py 使用同一数据时得到一致结果（否则每次采样不同导致预测/碰撞结果不同）
        np.random.seed(1)
        # 目标: 采样固定数量(num_point)的点
        if len(cloud_masked) >= self.num_point:
            # 情况1: 有效点数量 >= 目标数量,随机采样不重复
            idxs = np.random.choice(len(cloud_masked), self.num_point, replace=False)
        else:
            # 情况2: 有效点数量 < 目标数量,需要重复采样补齐
            idxs1 = np.arange(len(cloud_masked))  # 所有有效点的索引
            idxs2 = np.random.choice(
                len(cloud_masked),
                self.num_point - len(cloud_masked),  # 需要补充的点数
                replace=True  # 允许重复
            )
            idxs = np.concatenate([idxs1, idxs2], axis=0)  # 合并索引
        
        # 根据采样索引提取点云
        cloud_sampled = cloud_masked[idxs]
        
        # ===== 创建 Open3D 点云对象（用于可视化）=====
        cloud_o3d = o3d.geometry.PointCloud()
        # 设置点的坐标
        cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        # 设置点的颜色
        cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
        
        # ===== 准备模型输入 =====
        # 将点云转换为 PyTorch 张量并移动到计算设备
        cloud_sampled_torch = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(self.device)
        # 创建端点字典(end_points),包含模型需要的所有输入
        end_points = {
            'point_clouds': cloud_sampled_torch,  # 采样后的点云 (1, num_point, 3)
            'cloud_colors': color_masked[idxs]  # 对应的颜色信息
        }
        
        return end_points, cloud_o3d  # 返回模型输入和可视化点云
    
    def _get_grasps(self, end_points):
        """使用 GraspNet 模型预测抓取
        
        功能：
            1. 前向传播获取模型输出
            2. 解码模型输出为抓取位姿
        
        参数：
            end_points: 包含点云等输入数据的字典
        
        返回：
            GraspGroup: 预测的抓取组对象,包含多个抓取位姿
        """
        # 禁用梯度计算(推理模式,节省内存)
        with torch.no_grad():
            # 模型前向传播
            end_points = self.net(end_points)
            # 解码模型输出为抓取位姿
            # grasp_preds: (batch_size, num_grasps, 17)
            # 17维: [score, width, height, depth, rotation(9), translation(3), object_id]
            grasp_preds = self.pred_decode(end_points)
        
        # 将第一个 batch 的预测结果转换为 NumPy 数组
        gg_array = grasp_preds[0].detach().cpu().numpy()
        # 创建 GraspGroup 对象(graspnetAPI 提供的数据结构)
        gg = self.GraspGroup(gg_array)

        return gg
    
    def _collision_detection(self, gg, cloud):
        """碰撞检测：移除与场景碰撞的抓取
        
        功能：
            使用无模型碰撞检测器检测抓取是否会与场景点云碰撞
        
        参数：
            gg: GraspGroup 对象,包含待检测的抓取
            cloud: 场景点云 (N, 3) NumPy 数组
        
        返回：
            GraspGroup: 移除碰撞抓取后的 GraspGroup 对象
        """
        # 创建无模型碰撞检测器
        # voxel_size: 体素网格大小,用于加速碰撞检测
        mfcdetector = self.ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        
        # 检测碰撞
        # approach_dist: 接近距离(米),检测夹爪接近物体时的碰撞
        # collision_thresh: 碰撞阈值,判断是否碰撞的距离阈值
        # 返回: 布尔掩码,True 表示该抓取会碰撞
        collision_mask = mfcdetector.detect(
            gg, 
            approach_dist=0.05,  # 检测接近路径上5厘米范围内的碰撞
            collision_thresh=self.collision_thresh  # 碰撞判定阈值
        )
        
        # 移除碰撞的抓取(保留 ~collision_mask 为 True 的,即不碰撞的)
        gg = gg[~collision_mask]
        self.get_logger().info(f'碰撞检测后剩余 {len(gg)} 个抓取')
        if len(gg) == 0:
            self.get_logger().warn(
                '碰撞检测后抓取数为 0。若 demo.py 能出抓取，可尝试: collision_thresh:=0 关闭碰撞检测，或检查 data_dir 是否与 demo.py 使用同一目录'
            )
        return gg

    def _color_map(self, score):
        """将抓取分数映射为颜色

        参数：
            score: 归一化后的分数 [0, 1]

        返回：
            tuple: (r, g, b, a) 颜色值,范围 [0, 1]
            - 低分(0) -> 红色 (1, 0, 0, 1)
            - 高分(1) -> 绿色 (0, 1, 0, 1)
        """
        return tuple([float(1 - score), float(score), float(0), float(1)])

    def publish_marker_array(self, gg, clear_first=True):
        """发布抓取的 MarkerArray 到 RViz2 并发布 TF
        
        功能：
            将 GraspGroup 中的每个抓取转换为 RViz Marker 并发布
            同时发布每个抓取位姿的 TF 变换
        
        参数：
            gg: GraspGroup 对象(应该已经过 NMS 和排序处理)
            clear_first: 是否先清除之前的所有标记
        
        说明：
            - 每个抓取由4个圆柱体 + 1个位姿中心球表示
            - 颜色根据抓取质量分数映射(绿色=高分,红色=低分)
            - Marker 在 camera_frame 坐标系下发布
            - 同时发布 TF: camera_frame -> grasp_pose_0, grasp_pose_1, ...
            - 只发布一次
        """
        # ===== 清除旧标记 =====
        if clear_first:
            self.clear_markers()  # 发送 DELETEALL 消息清除所有旧标记
        
        # ===== 检查是否有抓取 =====
        if len(gg) == 0:
            self.get_logger().warn('没有可发布的抓取')
            return
        

        # ===== 初始化 =====
        vmin, vmax = 0, 1  # 分数归一化范围
        self.draw_grasp_id = 0  # 重置 Marker ID 计数器
        
        # ===== 发布每个抓取 =====
        for i in range(len(gg)):
            # 获取当前抓取
            grasp = gg[i]
            
            self.get_logger().info(f'抓取 #{i}:')
            self.get_logger().info(f'  分数: {grasp.score:.3f}')
            self.get_logger().info(f'  尺寸 (宽x高x深): {grasp.width:.3f} x {grasp.height:.3f} x {grasp.depth:.3f} m')
            self.get_logger().info(f'  位置: [{grasp.translation[0]:.3f}, {grasp.translation[1]:.3f}, {grasp.translation[2]:.3f}]')
            
            # 归一化抓取分数到 [0, 1]
            score_normalized = (grasp.score - vmin) / (vmax - vmin) if vmax > vmin else 0.5
            
            # 根据分数生成颜色
            color = self._color_map(score_normalized)
            
            # 创建该抓取的所有 Marker(左指、右指、手腕、手掌)
            markers = self.create_grasp_markers(grasp, color, self.draw_grasp_id)
            
            # 将 Marker 列表打包成 MarkerArray
            marker_array = MarkerArray()
            marker_array.markers = markers
            
            # 发布 MarkerArray
            self.marker_pub.publish(marker_array)
            
            # 发布抓取位姿的 TF
            self.publish_grasp_tf(grasp, i)
            
            # 更新 ID 计数器(每个抓取用5个ID: 左指、右指、手腕、手掌、中心点)
            self.draw_grasp_id += 5
            
            # 短暂延时,避免一次发布太多消息
            time.sleep(0.05)
        
        self.get_logger().info(f'已发布 {len(gg)} 个抓取和 TF 变换')
    
    def clear_markers(self):
        """清除 RViz2 中的所有标记
        
        功能：
            发送 DELETEALL 动作的 Marker,清除该 namespace 下的所有标记
        """
        # 创建删除标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # 设置动作为删除所有标记
        delete_marker.header.frame_id = self.frame_id  # 设置坐标系
        delete_marker.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        
        # 打包成 MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [delete_marker]
        
        # 发布删除消息
        self.marker_pub.publish(marker_array)
    
    def publish_grasp_tf(self, grasp, grasp_index):
        """发布抓取位姿的 TF 变换
        
        功能：
            将抓取位姿作为 TF 变换发布到 TF 树
        
        参数：
            grasp: 单个抓取对象，包含 translation 和 rotation_matrix
            grasp_index: 抓取索引，用于命名 TF frame
        
        说明：
            - 发布从 camera_frame 到 grasp_pose_X 的变换
            - 可用于后续的运动规划和可视化
            - 坐标系转换：GraspNet(X=approach,Y=width,Z=height) -> ROS标准(Z=approach,X=width,Y=height)
        """
        # 创建 TransformStamped 消息
        t = TransformStamped()
        
        # 设置时间戳和坐标系
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id  # 父坐标系：camera_frame
        t.child_frame_id = f'grasp_pose_{grasp_index}'  # 子坐标系：grasp_pose_0, grasp_pose_1, ...
        
        # 设置平移（位置不变）
        t.transform.translation.x = float(grasp.translation[0])
        t.transform.translation.y = float(grasp.translation[1])
        t.transform.translation.z = float(grasp.translation[2])
        
        # 坐标系转换：将 GraspNet 坐标系转换为 ROS 标准末端执行器坐标系
        # GraspNet: col0=approach(X轴), col1=width(Y轴), col2=height(Z轴)
        # ROS标准: Z轴=approach, X轴=width, Y轴=height
        # 与 Marker 可视化使用相同的转换，确保 TF 和 Marker 姿态一致
        R_graspnet = grasp.rotation_matrix
        R_ros = np.column_stack([
            R_graspnet[:, 1],  # ROS X轴 = GraspNet width (col1)
            R_graspnet[:, 2],  # ROS Y轴 = GraspNet height (col2)
            R_graspnet[:, 0]   # ROS Z轴 = GraspNet approach (col0) - 手指伸出方向
        ])
        
        # 转换为四元数
        quat = Rotation.from_matrix(R_ros).as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        # 发布 TF 变换
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().debug(f'发布 TF: {self.frame_id} -> {t.child_frame_id} (已转换为ROS标准坐标系)')
    
    def publish_pointcloud(self, cloud):
        """发布点云到 ROS2 话题
        
        功能：
            将 Open3D 点云对象转换为 ROS2 PointCloud2 消息并发布
        
        参数：
            cloud: Open3D 点云对象 (o3d.geometry.PointCloud)
        
        说明：
            - 将点云数据从 Open3D 格式转换为 ROS2 格式
            - 支持带颜色的点云 (XYZRGB)
            - 只发布一次
        """
        try:
            # 将 Open3D 点云转换为 ROS2 PointCloud2 消息
            pointcloud_msg = self._o3d_to_ros2_pointcloud(cloud)
            
            # 发布点云
            self.pointcloud_pub.publish(pointcloud_msg)
            
            self.get_logger().info(f'已发布点云: {len(cloud.points)} 个点')
            
        except Exception as e:
            self.get_logger().error(f'点云发布失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _o3d_to_ros2_pointcloud(self, o3d_cloud):
        """将 Open3D 点云转换为 ROS2 PointCloud2 消息
        
        功能：
            将 Open3D 格式的点云数据转换为 ROS2 PointCloud2 消息格式
        
        参数：
            o3d_cloud: Open3D 点云对象 (o3d.geometry.PointCloud)
        
        返回：
            PointCloud2: ROS2 点云消息
        
        说明：
            - 自动检测是否有颜色信息
            - 有颜色: 使用 XYZRGB 格式 (6个字段)
            - 无颜色: 使用 XYZ 格式 (3个字段)
            - 颜色范围从 [0,1] 转换为 [0,255]
        """
        # 创建消息头
        header = Header()
        header.frame_id = self.frame_id  # 设置坐标系
        header.stamp = Time(sec=0, nanosec=0)  # 使用时间戳 0(使用最新可用的 TF)
        
        # 获取点云数据
        points = np.asarray(o3d_cloud.points)  # 转换为 numpy 数组 (N, 3)
        has_colors = o3d_cloud.has_colors()  # 检查是否有颜色
        
        # 定义 PointCloud2 的字段
        if has_colors:
            # 有颜色: XYZRGB 格式
            colors = np.asarray(o3d_cloud.colors)  # 颜色数据 (N, 3), 范围 [0, 1]
            colors = (colors * 255).astype(np.uint8)  # 转换为 [0, 255]
            
            # 定义字段: X, Y, Z, RGB
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            
            # 将 RGB 打包为 uint32
            rgb = np.zeros(len(colors), dtype=np.uint32)
            rgb = (colors[:, 0].astype(np.uint32) << 16 | 
                   colors[:, 1].astype(np.uint32) << 8 | 
                   colors[:, 2].astype(np.uint32))
            
            # 组合数据: 将 XYZ 和 RGB 组合成一个结构化数组
            cloud_data = np.zeros(len(points), dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.uint32),
            ])
            cloud_data['x'] = points[:, 0]
            cloud_data['y'] = points[:, 1]
            cloud_data['z'] = points[:, 2]
            cloud_data['rgb'] = rgb
            
        else:
            # 无颜色: XYZ 格式
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            # 组合数据
            cloud_data = np.zeros(len(points), dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
            ])
            cloud_data['x'] = points[:, 0]
            cloud_data['y'] = points[:, 1]
            cloud_data['z'] = points[:, 2]
        
        # 创建 PointCloud2 消息
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1  # 无序点云高度为 1
        pointcloud_msg.width = len(points)  # 点的总数
        pointcloud_msg.fields = fields  # 字段定义
        pointcloud_msg.is_bigendian = False  # 小端序
        pointcloud_msg.point_step = cloud_data.dtype.itemsize  # 每个点的字节数
        pointcloud_msg.row_step = cloud_data.dtype.itemsize * len(points)  # 每行的字节数
        pointcloud_msg.is_dense = True  # 没有无效点
        pointcloud_msg.data = cloud_data.tobytes()  # 将数据转换为字节流
        
        return pointcloud_msg
    
    def create_grasp_markers(self, grasp, color, id_start=0):
        """为单个抓取创建可视化标记
        
        功能：
            将一个抓取位姿转换为4个圆柱体 Marker(左指、右指、手腕、手掌)
        
        参数：
            grasp: 单个抓取对象,包含位置、旋转、宽度、深度等信息
            color: RGBA 颜色元组 (r, g, b, a)
            id_start: 起始 Marker ID
        
        返回：
            list: 包含5个 Marker 对象的列表(4 个圆柱 + 1 个位姿中心球)
        
        说明：
            - 相机系与 ROS REP 103 / GraspNet 一致: X右 Y下 Z前
            - GraspNet rotation_matrix 列为 [approach, width, height], 与 plot_gripper_pro_max 一致
            - 局部偏移与圆柱轴按 (depth, width, height) 使用, 与 Open3D 可视化对齐
        """
        # ===== 构建抓取位姿矩阵 =====
        # GraspNet rotation_matrix 列: col0=approach(深度), col1=width(左右), col2=height(上下)
        # 与 plot_gripper_pro_max 一致: 手指沿 +X(approach) 伸出, 中心在 (d/2, ±w/2); 手掌/tail 在后方
        pose_mat = np.eye(4)
        pose_mat[:3, 3] = grasp.translation
        pose_mat[:3, :3] = grasp.rotation_matrix
        R = pose_mat[:3, :3]
        
        w = grasp.width
        d = grasp.depth
        radius = 0.005
        stamp = Time(sec=0, nanosec=0)
        markers = []
        
        # 与 plot_gripper_pro_max 完全一致的几何常数 (utils.py)
        finger_width = 0.004 # 手指/手掌厚度
        depth_base = 0.02 # 夹爪基座深度
        tail_length = 0.04 # 手腕(tail)长度
        # 手指在 plot_gripper 中 x 从 -(depth_base+finger_width) 到 depth，与手掌衔接
        finger_length = d + (depth_base + finger_width)
        finger_center_x = d / 2 - (depth_base + finger_width) / 2
        
        # ===== 左指: 局部中心与长度与 plot_gripper 一致，与手掌衔接 =====
        left_point = pose_mat @ np.array([finger_center_x, -w / 2 - finger_width / 2, 0, 1])
        left_pose = np.eye(4)
        left_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])  # Marker Z = col0 (approach)
        left_pose[:3, 3] = left_point[:3]
        markers.append(self._create_marker(Marker.CYLINDER, left_pose, [radius, radius, finger_length], color, id_start, stamp))
        
        # ===== 右指: 同上 =====
        right_point = pose_mat @ np.array([finger_center_x, w / 2 + finger_width / 2, 0, 1])
        right_pose = np.eye(4)
        right_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])
        right_pose[:3, 3] = right_point[:3]
        markers.append(self._create_marker(Marker.CYLINDER, right_pose, [radius, radius, finger_length], color, id_start + 1, stamp))
        
        # ===== 手腕(tail): plot_gripper 中 tail 中心 = -(tail_length/2 + finger_width + depth_base), 沿 approach, 长度 tail_length =====
        wrist_center_x = -(tail_length / 2 + finger_width + depth_base)
        wrist_point = pose_mat @ np.array([wrist_center_x, 0, 0, 1])
        wrist_pose = np.eye(4)
        wrist_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])
        wrist_pose[:3, 3] = wrist_point[:3]
        markers.append(self._create_marker(Marker.CYLINDER, wrist_pose, [radius, radius, tail_length], color, id_start + 2, stamp))
        
        # ===== 手掌(bottom): plot_gripper 中 bottom 中心 = (-depth_base - finger_width/2, 0, 0), 圆柱轴沿 width(col1), 长度 w =====
        # 右手系: Marker Z=col1, 取 X=col2 Y=col0 使 X×Y=col1
        palm_center_x = -depth_base - finger_width / 2
        palm_point = pose_mat @ np.array([palm_center_x, 0, 0, 1])
        palm_pose = np.eye(4)
        palm_pose[:3, :3] = np.column_stack([R[:, 2], R[:, 0], R[:, 1]])  # (X,Y,Z)=(col2,col0,col1), Z=col1
        palm_pose[:3, 3] = palm_point[:3]
        markers.append(self._create_marker(Marker.CYLINDER, palm_pose, [radius, radius, w], color, id_start + 3, stamp))
        
        # ===== 位姿中心点: pose_mat 原点，用球体显示 =====
        center_pose = np.eye(4)
        center_pose[:3, 3] = pose_mat[:3, 3]
        sphere_radius = 0.015  # 直径约 3cm，便于在 RViz 中辨认
        markers.append(self._create_marker(Marker.SPHERE, center_pose, [sphere_radius * 2] * 3, color, id_start + 4, stamp))
        
        return markers
    
    def _create_marker(self, marker_type, pose_mat, scale, color, id, stamp):
        """创建单个 RViz Marker
        
        功能：
            根据给定参数创建一个 visualization_msgs/Marker 消息
        
        参数：
            marker_type: Marker 类型(如 Marker.CYLINDER)
            pose_mat: 4x4 齐次变换矩阵,表示 Marker 的位姿
            scale: 尺寸,可以是标量或 [x, y, z] 列表
            color: RGBA 颜色元组 (r, g, b, a)
            id: Marker 的唯一 ID
            stamp: 时间戳
        
        返回：
            Marker: 完整配置的 Marker 消息对象
        """
        # ===== 处理尺寸参数 =====
        # 如果 scale 是标量,转换为 [scale, scale, scale]
        if np.isscalar(scale):
            scale = [float(scale), float(scale), float(scale)]  
        else:
            scale = [float(s) for s in scale]  
        
        # ===== 创建 Marker 对象 =====
        marker = Marker()
        
        # ===== 设置 Header =====
        marker.header.frame_id = self.frame_id  # 坐标系(camera_frame)
        marker.header.stamp = stamp  # 时间戳
        
        # ===== 设置 Marker 基本属性 =====
        marker.ns = "grasp"  # 命名空间,用于区分不同类型的 Marker
        marker.id = id  # 唯一 ID
        marker.type = marker_type  # 类型(圆柱体/球体/立方体等)
        marker.action = Marker.ADD  # 动作: 添加或修改 Marker
        
        # ===== 设置生命周期 =====
        # lifetime = 0 表示永久存在,不会自动消失
        marker.lifetime = Duration(sec=0, nanosec=0)
        
        # ===== 设置位姿 =====
        # 将4x4变换矩阵转换为 Pose 消息(位置+四元数)
        marker.pose = self._matrix_to_pose(pose_mat)
        
        # ===== 设置尺寸 =====
        marker.scale.x = scale[0]  # X 方向尺寸
        marker.scale.y = scale[1]  # Y 方向尺寸
        marker.scale.z = scale[2]  # Z 方向尺寸
        
        # ===== 设置颜色 =====
        marker.color.r = float(color[0])  # 红色分量 [0, 1]
        marker.color.g = float(color[1])  # 绿色分量 [0, 1]
        marker.color.b = float(color[2])  # 蓝色分量 [0, 1]
        marker.color.a = float(color[3])  # 透明度 [0, 1], 1=不透明
        
        return marker
    
    def _matrix_to_pose(self, pose_mat):
        """将4x4齐次变换矩阵转换为 geometry_msgs/Pose 消息
        
        功能：
            提取变换矩阵中的旋转和平移部分,转换为 ROS Pose 消息
        
        参数：
            pose_mat: 4x4 齐次变换矩阵
                [[R11, R12, R13, tx],
                 [R21, R22, R23, ty],
                 [R31, R32, R33, tz],
                 [0,   0,   0,   1 ]]
        
        返回：
            Pose: ROS Pose 消息,包含位置(x,y,z)和方向(四元数)
        """
        pose = Pose()
        
        # ===== 提取旋转矩阵并转换为四元数 =====
        # pose_mat[:3, :3] 是3x3旋转矩阵
        # Rotation.from_matrix() 将旋转矩阵转换为 Rotation 对象
        # as_quat() 获取四元数表示 [x, y, z, w]
        quat = Rotation.from_matrix(pose_mat[:3, :3]).as_quat()
        
        # 设置方向(四元数)
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        
        # ===== 提取平移向量 =====
        # pose_mat[:, 3] 是4x1平移向量的前3个元素(第4个是1)
        pose.position.x = float(pose_mat[0, 3])
        pose.position.y = float(pose_mat[1, 3])
        pose.position.z = float(pose_mat[2, 3])
        
        return pose


def main(args=None):
    """主函数：ROS 2 节点的入口点
    
    功能：
        1. 初始化 ROS 2
        2. 创建节点实例
        3. 启动节点循环(spin)
        4. 处理退出和清理
    
    参数：
        args: 命令行参数(可选)
    """
    # ===== 初始化 ROS 2 =====
    rclpy.init(args=args)
    
    # ===== 创建节点实例 =====
    node = GraspNetDemoNode()
    
    try:
        # ===== 启动节点循环 =====
        # spin() 会一直运行,处理回调函数(定时器、订阅等)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # ===== 处理键盘中断 (Ctrl+C) =====
        node.get_logger().info('节点关闭')
        
    finally:
        # ===== 清理资源 =====
        # 销毁节点,释放资源
        node.destroy_node()
        # 关闭 ROS 2
        rclpy.shutdown()


# Python 脚本入口点
if __name__ == '__main__':
    main()  # 调用主函数
