#!/usr/bin/env python3
"""
ROS2通信模块

功能：
1. 提供ROS2服务接口（EstimatePose, ListTemplates, StandardizeTemplate等）
2. 处理图像消息和服务请求
3. 协调各个模块完成姿态估计任务
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from interface.srv import EstimatePose, ListTemplates, StandardizeTemplate, UpdateParams
from interface.msg import CartesianPosition
from cv_bridge import CvBridge

# 可选导入 - 机器人接口
try:
    from demo_interface.msg import RobotStatus
    ROBOT_AVAILABLE = True
except ImportError:
    RobotStatus = None
    ROBOT_AVAILABLE = False
import cv2
import numpy as np
import logging
from pathlib import Path
from typing import Optional, Dict, List, Tuple
import json
import math
import time
import threading
import os
import traceback
try:
    import yaml  # 用于加载手眼标定YAML文件
except ImportError:
    yaml = None

from .preprocessor import Preprocessor
from .feature_extractor import FeatureExtractor, ComponentFeature
from .template_standardizer import TemplateStandardizer
from .config_reader import ConfigReader
from .pose_estimator import PoseEstimator, TemplateItem
from .debug_visualizer import DebugVisualizer
from .rembg_processor import RemBGProcessor
from .subprocess_rembg import SubprocessRemBGProcessor

# 导入参数管理器（从web_ui共享）
import sys
_ros2_comm_pkg_root = Path(__file__).resolve().parent.parent
web_ui_path = _ros2_comm_pkg_root / 'web_ui' / 'scripts'
if str(web_ui_path) not in sys.path:
    sys.path.insert(0, str(web_ui_path))
try:
    from params_manager import ParamsManager
except ImportError:
    ParamsManager = None

# 默认配置目录：web_ui/configs（标定等优先从此加载）
# 源码运行：包内 web_ui/configs；install 运行：configs 在 share/ 下，需用 ament 解析
_configs_from_pkg = _ros2_comm_pkg_root / 'web_ui' / 'configs'
if _configs_from_pkg.exists():
    WEB_UI_CONFIGS_DIR = _configs_from_pkg
else:
    try:
        from ament_index_python.packages import get_package_share_directory
        WEB_UI_CONFIGS_DIR = Path(get_package_share_directory('visual_pose_estimation_python')) / 'web_ui' / 'configs'
    except Exception:
        WEB_UI_CONFIGS_DIR = _configs_from_pkg


class ROS2Communication:
    """ROS2通信类"""
    
    def __init__(self, node: Node):
        """初始化ROS2通信
        
        Args:
            node: ROS2节点
        """
        self.node = node
        self.logger = node.get_logger()
        self.cv_bridge = CvBridge()
        
        # 组件
        self.preprocessor = None
        self.feature_extractor = None
        self.template_standardizer = None
        self.config_reader = None
        self.pose_estimator = None
        self.rembg_processor = None
        self._rembg_mask_cache: Dict[int, np.ndarray] = {}
        self._rembg_cutout_cache: Dict[int, np.ndarray] = {}
        
        # 配置
        self.template_root = ""
        self.calib_file = ""
        
        # 相机参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.T_E_C = None  # 末端到相机的变换
        self.actual_calib_file_path = None  # 实际使用的手眼标定文件路径
        
        # 服务
        self.estimate_pose_service = None
        self.list_templates_service = None
        self.standardize_template_service = None
        
        # 发布者
        self.status_publisher = None
        
        # 图像订阅和存储
        self.color_image_subscription = None
        self.depth_image_subscription = None
        self.current_color_image = None  # 当前彩色图
        self.current_depth_image = None  # 当前深度图
        self.color_image_timestamp = 0.0  # 彩色图接收时间戳
        self.depth_image_timestamp = 0.0  # 深度图接收时间戳
        self.image_lock = threading.Lock()  # 线程锁，保护图像访问
        
        # 机器人状态订阅和存储
        self.robot_status_subscription = None
        self.latest_robot_status = None  # 最新机器人状态
        self.robot_status_lock = threading.Lock()  # 线程锁，保护机器人状态访问
        

    def _log_kv(self, level: str, event: str, **data) -> None:
        """统一结构化日志输出（避免散落的print/写文件）。"""
        try:
            payload = {"event": event, **data}
            msg = json.dumps(payload, ensure_ascii=False, default=str)
        except Exception:
            msg = f"{event} {data}"
        log_fn = getattr(self.logger, level, None)
        if callable(log_fn):
            log_fn(msg)
        else:
            # 兜底：使用info
            self.logger.info(msg)
    
    def initialize(
        self,
        config_reader: ConfigReader,
        template_root: str,
        calib_file: str = ""
    ) -> bool:
        """初始化ROS2通信
        
        Args:
            config_reader: 配置读取器
            template_root: 模板根目录
            calib_file: 手眼标定文件路径
            
        Returns:
            是否初始化成功
        """
        try:
            self.config_reader = config_reader
            self.template_root = template_root
            self.calib_file = calib_file
            
            # 创建组件
            self.preprocessor = Preprocessor()
            self.feature_extractor = FeatureExtractor()
            self.template_standardizer = TemplateStandardizer()
            self.pose_estimator = PoseEstimator(self.preprocessor, self.feature_extractor)
            self.debug_visualizer = DebugVisualizer()
            
            # 加载参数：统一使用config_reader获取阈值参数
            debug_thresholds = self.config_reader.load_debug_thresholds()
            
            # 加载Preprocessor参数（阈值参数从config_reader获取，其他参数使用默认配置）
            preprocessor_params = self._merge_threshold_params(
                config_reader.get_section('preprocessor'),
                debug_thresholds,
                'preprocessor'
            )
            self.preprocessor.set_parameters(preprocessor_params)
            
            # 加载FeatureExtractor参数（阈值参数从config_reader获取，其他参数使用默认配置）
            feature_extractor_params = self._merge_threshold_params(
                config_reader.get_section('feature_extractor'),
                debug_thresholds,
                'feature_extractor'
            )
            self.feature_extractor.set_parameters(feature_extractor_params)
            
            # 设置姿态估计器参数（包括暴力匹配参数）
            pose_estimator_params = config_reader.get_section('pose_estimator')
            self.pose_estimator.set_parameters(pose_estimator_params)
            
            # 加载手眼标定（优先使用指定的calib_file，否则尝试从标准位置加载）
            calib_loaded = False
            if calib_file and Path(calib_file).exists():
                calib_loaded = self._load_hand_eye_calibration(calib_file)
                if calib_loaded:
                    self.actual_calib_file_path = calib_file
                    self.pose_estimator.calib_file_path = calib_file
                    self.logger.info(f'手眼标定文件: {calib_file}')
                else:
                    self.logger.warning(f'加载手眼标定失败: {calib_file}')
            
            # 如果未加载成功，尝试从标准位置加载
            if not calib_loaded:
                calib_loaded = self._load_hand_eye_calibration_from_standard_paths()
                if calib_loaded and self.actual_calib_file_path:
                    self.logger.info(f'手眼标定文件: {self.actual_calib_file_path}')
                    self.pose_estimator.calib_file_path = self.actual_calib_file_path
                else:
                    self.logger.warning('未找到手眼标定文件')
            
            # ========== 创建ROS2服务 ==========
            # 姿态估计服务：接收图像和工件ID，返回估计的姿态（抓取、准备、放置等位置）
            # 服务名称: /estimate_pose
            # 请求: 输入图像（可选，也可通过触发拍照获取）、工件ID
            # 响应: 姿态估计结果（位置、姿态、置信度等）
            self.estimate_pose_service = self.node.create_service(
                EstimatePose,
                'estimate_pose',
                self._handle_estimate_pose
            )
            
            # 模板列表服务：列出可用的模板库信息
            # 服务名称: /list_templates
            # 请求: 模板目录路径（可选，空字符串使用默认路径）
            # 响应: 模板ID列表、工件ID列表、姿态ID列表等
            self.list_templates_service = self.node.create_service(
                ListTemplates,
                'list_templates',
                self._handle_list_templates
            )
            
            # 模板标准化服务：标准化模板姿态（旋转到标准方向，生成标准化掩膜等）
            # 服务名称: /standardize_template
            # 请求: 工件ID
            # 响应: 标准化结果（处理数量、跳过数量、处理的姿态ID列表等）
            self.standardize_template_service = self.node.create_service(
                StandardizeTemplate,
                'standardize_template',
                self._handle_standardize_template
            )
            
            # 参数更新服务：动态更新算法参数（预处理、特征提取、模板匹配等参数）
            # 服务名称: /update_params
            # 请求: 参数段（section）和参数字典
            # 响应: 更新结果和当前参数值
            self.update_params_service = self.node.create_service(
                UpdateParams,
                'update_params',
                self._handle_update_params
            )
            
            # ========== 创建ROS2发布者 ==========
            # 系统状态发布者：发布系统运行状态和日志信息
            # 话题名称: /system_status
            # 消息类型: std_msgs/String
            # 队列深度: 10
            self.status_publisher = self.node.create_publisher(
                String,
                'system_status',
                10
            )
            
            # 不再持续订阅图像话题，改为在需要时临时订阅获取图像
            # 移除初始化时的订阅设置，改为触发拍照后临时获取
            
            # ========== 订阅机器人状态话题 ==========
            # 用于获取当前机器人位姿，计算 T_B_C
            if ROBOT_AVAILABLE:
                try:
                    self.robot_status_subscription = self.node.create_subscription(
                        RobotStatus,
                        '/demo_robot_status',
                        self._robot_status_callback,
                        10
                    )
                except Exception as e:
                    self.logger.warning(f'订阅机器人状态话题失败: {e}，将使用单位矩阵作为 T_B_C')
            else:
                self.logger.warning('RobotStatus 消息类型不可用，将使用单位矩阵作为 T_B_C')
            
            self.publish_system_status('ROS2通信初始化成功')
            
            return True
            
        except Exception as e:
            self.logger.error(f'ROS2通信初始化失败: {e}')
            return False


    def _merge_threshold_params(self, base_params: Dict, debug_thresholds: Dict, module: str = 'preprocessor') -> Dict:
        """合并阈值参数到基础参数字典
        
        Args:
            base_params: 基础参数字典（使用默认配置）
            debug_thresholds: 阈值参数字典（从debug_thresholds.json获取）
            module: 模块名称（'preprocessor' 或 'feature_extractor'）
            
        Returns:
            合并后的参数字典
        """
        merged = base_params.copy()
        
        if module == 'preprocessor':
            # Preprocessor需要的所有阈值参数
            threshold_keys = [
                'binary_threshold_min', 'binary_threshold_max',
                'component_min_area', 'component_max_area',
                'component_min_aspect_ratio', 'component_max_aspect_ratio',
                'component_min_width', 'component_min_height',
                'component_max_count', 'enable_zero_interp',
                'enable_smooth_edges', 'smooth_edges_blur_sigma'
            ]
        elif module == 'feature_extractor':
            # FeatureExtractor需要的阈值参数（不包含binary_threshold和enable_zero_interp）
            threshold_keys = [
                'component_min_area', 'component_max_area',
                'component_min_aspect_ratio', 'component_max_aspect_ratio',
                'component_min_width', 'component_min_height'
            ]
        else:
            return merged
        
        # 合并阈值参数
        for key in threshold_keys:
            if key in debug_thresholds:
                merged[key] = debug_thresholds[key]
        
        return merged
    
    def _apply_debug_params_to_preprocessor(self, debug_params: Dict) -> Tuple[int, int]:
        """把Debug阈值映射到Preprocessor和FeatureExtractor参数，
        并返回(binary_threshold_min, binary_threshold_max)。
        """
        binary_min = int(debug_params.get('binary_threshold_min', 0))
        binary_max = int(debug_params.get('binary_threshold_max', 65535))

        # Preprocessor参数映射（参考 http_bridge_server.py）
        preprocessor_params = {
            'binary_threshold_min': float(binary_min),
            'binary_threshold_max': float(binary_max),
            'component_min_area': float(debug_params.get('component_min_area', 10)),
            'component_max_area': float(debug_params.get('component_max_area', 100000)),
            'component_min_aspect_ratio': float(debug_params.get('component_min_aspect_ratio', 0.3)),
            'component_max_aspect_ratio': float(debug_params.get('component_max_aspect_ratio', 4.0)),
            'component_min_width': float(debug_params.get('component_min_width', 60)),
            'component_min_height': float(debug_params.get('component_min_height', 60)),
            'component_max_count': float(debug_params.get('component_max_count', 3)),
            'enable_zero_interp': float(debug_params.get('enable_zero_interp', 1.0)),
            'enable_smooth_edges': float(1 if debug_params.get('enable_smooth_edges', True) else 0),
            'smooth_edges_blur_sigma': float(debug_params.get('smooth_edges_blur_sigma', 0)),
        }

        # FeatureExtractor参数映射（参考 http_bridge_server.py）
        feature_extractor_params = {
            'component_min_area': float(debug_params.get('component_min_area', 10)),
            'component_max_area': float(debug_params.get('component_max_area', 100000)),
            'component_min_aspect_ratio': float(debug_params.get('component_min_aspect_ratio', 0.3)),
            'component_max_aspect_ratio': float(debug_params.get('component_max_aspect_ratio', 4.0)),
            'component_min_width': float(debug_params.get('component_min_width', 60)),
            'component_min_height': float(debug_params.get('component_min_height', 60)),
        }

        self.logger.info(
            f'应用Debug阈值到Preprocessor: '
            f'binary=[{binary_min}, {binary_max}], '
            f'contour_area=[{preprocessor_params["component_min_area"]}, {preprocessor_params["component_max_area"]}], '
            f'aspect=[{preprocessor_params["component_min_aspect_ratio"]}, {preprocessor_params["component_max_aspect_ratio"]}], '
            f'size=[{preprocessor_params["component_min_width"]}x{preprocessor_params["component_min_height"]}], '
            f'max_count={preprocessor_params["component_max_count"]}, '
            f'enable_zero_interp={preprocessor_params["enable_zero_interp"]}, '
            f'enable_smooth_edges={preprocessor_params["enable_smooth_edges"]}, '
            f'blur_sigma={preprocessor_params["smooth_edges_blur_sigma"]}'
        )

        try:
            if self.preprocessor:
                self.preprocessor.set_parameters(preprocessor_params)
        except Exception as e:
            self.logger.warning(f'应用Debug阈值到Preprocessor失败: {e}')

        self.logger.info(
            f'应用Debug阈值到FeatureExtractor: '
            f'contour_area=[{feature_extractor_params["component_min_area"]}, {feature_extractor_params["component_max_area"]}], '
            f'aspect=[{feature_extractor_params["component_min_aspect_ratio"]}, {feature_extractor_params["component_max_aspect_ratio"]}], '
            f'size=[{feature_extractor_params["component_min_width"]}x{feature_extractor_params["component_min_height"]}]'
        )

        try:
            if self.feature_extractor:
                self.feature_extractor.set_parameters(feature_extractor_params)
        except Exception as e:
            self.logger.warning(f'应用Debug阈值到FeatureExtractor失败: {e}')

        return binary_min, binary_max

    def _draw_debug_like_visuals(
        self,
        color_image: np.ndarray,
        component_mask: np.ndarray,
        feature: ComponentFeature,
        preprocessed_color: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """生成与Debug选项卡观感一致的两张可视化图：vis_original / vis_img。"""
        try:
            if color_image is None or component_mask is None:
                return None, None

            vis_original = color_image.copy()

            contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(max_contour)
                cx = x + w // 2
                cy = y + h // 2

                rw = int(w * 1.2)
                rh = int(h * 1.2)
                rx = cx - rw // 2
                ry = cy - rh // 2

                cv2.rectangle(vis_original, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.rectangle(vis_original, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
                cv2.circle(vis_original, (cx, cy), 5, (255, 0, 0), -1)

            base = preprocessed_color if preprocessed_color is not None else color_image
            try:
                vis_img = feature.draw_features(base)
            except Exception:
                vis_img = base.copy()

            return vis_original, vis_img
        except Exception as e:
            self.logger.warning(f'生成Debug风格可视化失败: {e}')
            return None, None
    
    def _capture_images_on_trigger(self, timeout=10.0):
        """触发拍照后临时订阅并获取深度图和彩色图
        
        注意：此方法会临时创建订阅，获取图像后立即销毁订阅，不持续接收视频流
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            (depth_image, color_image): 深度图和彩色图的元组，如果超时则为(None, None)
        """
        try:
            # QoS配置：使用较小的队列深度，只保留最新的图像
            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1  # 只保留最新的一帧
            )
            
            # 重置图像接收标志
            with self.image_lock:
                self.current_color_image = None
                self.current_depth_image = None
                self.color_image_timestamp = 0.0
                self.depth_image_timestamp = 0.0
            
            # 获取话题名称
            self.node.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
            depth_topic = self.node.get_parameter('depth_image_topic').value
            
            self.node.declare_parameter('color_image_topic', '/camera/color/image_raw')
            color_topic = self.node.get_parameter('color_image_topic').value
            
            # 临时创建订阅
            depth_sub = self.node.create_subscription(
                Image,
                depth_topic,
                self._depth_image_callback,
                image_qos
            )
            
            color_sub = self.node.create_subscription(
                Image,
                color_topic,
                self._color_image_callback,
                image_qos
            )
            
            self.logger.info(f'订阅图像话题: {depth_topic}, {color_topic}')
            
            # 等待一小段时间让订阅建立（ROS2订阅需要时间建立连接）
            time.sleep(0.2)
            
            # 等待图像到达
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.image_lock:
                    if (self.current_depth_image is not None and 
                        self.current_color_image is not None):
                        # 复制图像以便返回
                        depth_copy = self.current_depth_image.copy()
                        color_copy = self.current_color_image.copy()
                        
                        # 销毁临时订阅
                        self.node.destroy_subscription(depth_sub)
                        self.node.destroy_subscription(color_sub)
                        
                        self.logger.info(f'获取图像: 深度图={depth_copy.shape}, 彩色图={color_copy.shape}')
                        return depth_copy, color_copy
                
                # 短暂休眠，等待消息到达
                time.sleep(0.05)
            
            # 超时，销毁订阅
            self.node.destroy_subscription(depth_sub)
            self.node.destroy_subscription(color_sub)
            
            self.logger.warning(f'⚠️ 获取图像超时（{timeout}秒）')
            return None, None
                
        except Exception as e:
            self.logger.error(f'临时订阅获取图像失败: {e}')
            # 确保销毁订阅
            try:
                if 'depth_sub' in locals():
                    self.node.destroy_subscription(depth_sub)
                if 'color_sub' in locals():
                    self.node.destroy_subscription(color_sub)
            except:
                return None, None
            return None, None
    
    def _depth_image_callback(self, msg: Image):
        """深度图回调函数
        
        Args:
            msg: 深度图消息
        """
        try:
            # 将ROS图像消息转换为OpenCV格式（保持原始深度值）
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            with self.image_lock:
                self.current_depth_image = cv_image
                self.depth_image_timestamp = time.time()
                
        except Exception as e:
            self.logger.error(f'深度图回调失败: {e}')
    
    def _color_image_callback(self, msg: Image):
        """彩色图回调函数
        
        Args:
            msg: sensor_msgs/Image 消息
        """
        try:
            # 使用 cv_bridge 转换为 OpenCV 格式
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if image is not None:
                with self.image_lock:
                    self.current_color_image = image
                    self.color_image_timestamp = time.time()
            else:
                self.logger.warning('⚠️ 彩色图转换结果为None')
                
        except Exception as e:
            self.logger.error(f'彩色图回调失败: {e}')
    
    def _load_hand_eye_calibration(self, calibration_path: str) -> bool:
        """加载手眼标定参数（支持JSON和YAML格式）
        
        Args:
            calibration_path: 标定文件路径
            
        Returns:
            是否加载成功
        """
        try:
            calib_path = Path(calibration_path)
            if not calib_path.exists():
                self.logger.warning(f'标定文件不存在: {calibration_path}')
                return False
            
            # 根据文件扩展名选择解析方式
            ext = calib_path.suffix.lower()
            if ext in ['.yaml', '.yml']:
                if yaml is None:
                    self.logger.error('需要 yaml 模块来加载YAML格式的标定文件')
                    return False
                with open(calib_path, 'r', encoding='utf-8') as f:
                    calib_data = yaml.safe_load(f)
            elif ext == '.json':
                with open(calib_path, 'r', encoding='utf-8') as f:
                    calib_data = json.load(f)
            else:
                # 尝试作为JSON加载
                with open(calib_path, 'r', encoding='utf-8') as f:
                    calib_data = json.load(f)
            
            loaded_anything = False
            
            # 加载相机内参（如果手眼标定文件中包含）
            camera_matrix_data = calib_data.get('camera_matrix', [])
            if camera_matrix_data:
                # 处理不同格式：可能是列表的列表，或包含 data 字段的字典
                if isinstance(camera_matrix_data, dict) and 'data' in camera_matrix_data:
                    # 格式：{rows: 3, cols: 3, data: [...]}
                    flat_data = camera_matrix_data['data']
                    self.camera_matrix = np.array(flat_data, dtype=np.float64).reshape(3, 3)
                elif isinstance(camera_matrix_data, list) and len(camera_matrix_data) == 3:
                    # 格式：[[...], [...], [...]]
                    self.camera_matrix = np.array(camera_matrix_data, dtype=np.float64)
                else:
                    self.camera_matrix = np.array(camera_matrix_data, dtype=np.float64).reshape(3, 3)
                loaded_anything = True
            
            # 加载畸变系数（如果手眼标定文件中包含）
            dist_coeffs_data = calib_data.get('dist_coeffs', calib_data.get('distortion_coefficients', []))
            if dist_coeffs_data:
                # 处理不同格式：可能是列表，或包含 data 字段的字典
                if isinstance(dist_coeffs_data, dict) and 'data' in dist_coeffs_data:
                    self.dist_coeffs = np.array(dist_coeffs_data['data'], dtype=np.float64)
                else:
                    self.dist_coeffs = np.array(dist_coeffs_data, dtype=np.float64)
                loaded_anything = True
            
            # 加载手眼变换（手眼标定结果文件中的 transformation_matrix 是 T_C_E，需要取逆得到 T_E_C）
            # 尝试多种可能的键名
            T_C_E_data = None
            if 'hand_eye_calibration' in calib_data:
                hand_eye_data = calib_data['hand_eye_calibration']
                T_C_E_data = hand_eye_data.get('transformation_matrix', [])
            elif 'T_C_E' in calib_data:
                T_C_E_data = calib_data['T_C_E']
            elif 'T_E_C' in calib_data:
                # 如果直接是 T_E_C，直接使用
                T_E_C_data = calib_data['T_E_C']
                if T_E_C_data:
                    self.T_E_C = np.array(T_E_C_data, dtype=np.float64).reshape(4, 4)
                    loaded_anything = True
            
            # 如果找到的是 T_C_E（相机到末端），需要取逆得到 T_E_C（末端到相机）
            if T_C_E_data:
                T_C_E = np.array(T_C_E_data, dtype=np.float64).reshape(4, 4)
                
                # 统一单位为米：
                # 手眼标定文件中的translation_vector单位通常是毫米（mm）
                # 而机器人姿态JSON中的位置单位是米（m），需要统一为米
                # 检查平移向量的数值范围来判断单位
                translation_norm = np.linalg.norm(T_C_E[:3, 3])
                if translation_norm > 100:  # 如果平移向量模长>100，很可能是毫米单位
                    # 将平移向量从毫米转换为米（除以1000）
                    T_C_E[:3, 3] = T_C_E[:3, 3] / 1000.0
                
                self.T_E_C = np.linalg.inv(T_C_E)
                loaded_anything = True
            
            if loaded_anything:
                return True
            else:
                self.logger.warning(f'标定文件中未找到有效数据: {calibration_path}')
                return False
            
        except Exception as e:
            self.logger.error(f'加载手眼标定失败: {e}')
            return False
    
    def _print_hand_eye_calibration_info(self):
        """打印手眼标定文件内容（在姿态估计计算前调用）"""
        if self.actual_calib_file_path:
            self.logger.info(f'手眼标定文件: {self.actual_calib_file_path}')
        
        if self.T_E_C is None:
            self.logger.warning('T_E_C未加载')
        
        if self.camera_matrix is None:
            self.logger.warning('相机内参未加载')
    
    def _load_hand_eye_calibration_from_standard_paths(self) -> bool:
        """从标准路径加载手眼标定参数
        
        优先从 web_ui/configs 加载，若无则回退到 hand_eye_calibration 包路径：
        1. 相机内参：web_ui/configs/camera_intrinsics.yaml（标准名），或 ost.yaml 兼容，或 hand_eye_calibration/.../ost.yaml
        2. 手眼标定：web_ui/configs/hand_eye_calibration.yaml（标准名），或最新 hand_eye_calibration*.yaml，或 hand_eye_calibration/.../calibration_results/
        
        Returns:
            是否加载成功
        """
        try:
            # 1. 先加载相机内参（如果还没有加载）：优先 web_ui/configs
            if self.camera_matrix is None and yaml is not None:
                for candidate in (WEB_UI_CONFIGS_DIR / 'camera_intrinsics.yaml', WEB_UI_CONFIGS_DIR / 'ost.yaml'):
                    if candidate.exists():
                        try:
                            with open(candidate, 'r', encoding='utf-8') as f:
                                cam_data = yaml.safe_load(f)
                            camera_matrix_data = cam_data.get('camera_matrix', {})
                            if camera_matrix_data:
                                if isinstance(camera_matrix_data, dict) and 'data' in camera_matrix_data:
                                    flat_data = camera_matrix_data['data']
                                    self.camera_matrix = np.array(flat_data, dtype=np.float64).reshape(3, 3)
                                elif isinstance(camera_matrix_data, list):
                                    self.camera_matrix = np.array(camera_matrix_data, dtype=np.float64)
                                dist_coeffs_data = cam_data.get('distortion_coefficients', {})
                                if dist_coeffs_data:
                                    if isinstance(dist_coeffs_data, dict) and 'data' in dist_coeffs_data:
                                        self.dist_coeffs = np.array(dist_coeffs_data['data'], dtype=np.float64)
                                    elif isinstance(dist_coeffs_data, list):
                                        self.dist_coeffs = np.array(dist_coeffs_data, dtype=np.float64)
                                self.logger.info(f'相机内参文件: {candidate}')
                                break
                        except Exception as e:
                            self.logger.warning(f'加载相机内参失败: {candidate}, 错误: {e}')
                # 回退：hand_eye_calibration 包路径
                if self.camera_matrix is None:
                    fallback_camera = Path('/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/calibrationdata/ost.yaml')
                    if fallback_camera.exists():
                        try:
                            with open(fallback_camera, 'r', encoding='utf-8') as f:
                                cam_data = yaml.safe_load(f)
                            camera_matrix_data = cam_data.get('camera_matrix', {})
                            if camera_matrix_data:
                                if isinstance(camera_matrix_data, dict) and 'data' in camera_matrix_data:
                                    self.camera_matrix = np.array(camera_matrix_data['data'], dtype=np.float64).reshape(3, 3)
                                elif isinstance(camera_matrix_data, list):
                                    self.camera_matrix = np.array(camera_matrix_data, dtype=np.float64)
                                dist_coeffs_data = cam_data.get('distortion_coefficients', {})
                                if dist_coeffs_data and isinstance(dist_coeffs_data, dict) and 'data' in dist_coeffs_data:
                                    self.dist_coeffs = np.array(dist_coeffs_data['data'], dtype=np.float64)
                                self.logger.info(f'相机内参文件: {fallback_camera}')
                        except Exception as e:
                            self.logger.warning(f'加载相机内参失败: {fallback_camera}, 错误: {e}')
            
            # 2. 加载手眼标定结果：优先 web_ui/configs/hand_eye_calibration.yaml（标准名），否则最新 hand_eye_calibration*.yaml
            if WEB_UI_CONFIGS_DIR.exists():
                standard_he = WEB_UI_CONFIGS_DIR / 'hand_eye_calibration.yaml'
                if standard_he.exists() and self._load_hand_eye_calibration(str(standard_he)):
                    self.actual_calib_file_path = str(standard_he)
                    return True
                he_files = [p for p in WEB_UI_CONFIGS_DIR.glob('hand_eye_calibration*.yaml')
                            if p.name != 'hand_eye_calibration.yaml']
                if not he_files:
                    he_files = [p for p in WEB_UI_CONFIGS_DIR.glob('*.yaml')
                                if p.name not in ('ost.yaml', 'camera_intrinsics.yaml')]
                if he_files:
                    latest = max(he_files, key=lambda p: p.stat().st_mtime)
                    if self._load_hand_eye_calibration(str(latest)):
                        self.actual_calib_file_path = str(latest)
                        return True
            
            # 回退：hand_eye_calibration 包 calibration_results
            calib_results_dir = Path('/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/calibration_results')
            if calib_results_dir.exists():
                yaml_files = sorted(calib_results_dir.glob('*.yaml'), key=lambda p: p.stat().st_mtime, reverse=True)
                if yaml_files and self._load_hand_eye_calibration(str(yaml_files[0])):
                    self.actual_calib_file_path = str(yaml_files[0])
                    return True
            
            if self.camera_matrix is not None:
                self.logger.warning('相机内参已加载，但手眼标定参数未找到')
            return False
            
        except Exception as e:
            self.logger.warning(f'从标准路径加载手眼标定失败: {e}')
            return False
    
    def _load_pose_json_data(self, pose_data: Dict) -> Optional[np.ndarray]:
        """从JSON数据加载姿态变换矩阵 (4x4)
        
        支持多种格式：
        1. 机器人状态格式：{"cartesian_position": {"position": {...}, "orientation": {...}}}
        2. 直接格式：{"position": {...}, "orientation": {...}}
        3. 矩阵数组：直接是4x4数组
        
        Args:
            pose_data: JSON数据
            
        Returns:
            4x4变换矩阵 或 None
        """
        try:
            # 如果直接是矩阵数组
            if isinstance(pose_data, list) and len(pose_data) == 4:
                T = np.array(pose_data, dtype=np.float64)
                return T
            
            # 支持机器人状态格式（cartesian_position.position/orientation）
            cartesian_pos = pose_data.get('cartesian_position', {})
            if cartesian_pos:
                position = cartesian_pos.get('position', {})
                orientation = cartesian_pos.get('orientation', {})
            else:
                # 直接格式（position/orientation）
                position = pose_data.get('position', {})
                orientation = pose_data.get('orientation', {})
            
            T = np.eye(4, dtype=np.float64)
            
            # 从position和orientation构建
            if isinstance(position, dict):
                T[0, 3] = float(position.get('x', 0.0))
                T[1, 3] = float(position.get('y', 0.0))
                T[2, 3] = float(position.get('z', 0.0))
            elif isinstance(position, list) and len(position) >= 3:
                T[0, 3] = float(position[0])
                T[1, 3] = float(position[1])
                T[2, 3] = float(position[2])
            
            # 四元数转旋转矩阵
            if isinstance(orientation, dict):
                qx = float(orientation.get('x', 0.0))
                qy = float(orientation.get('y', 0.0))
                qz = float(orientation.get('z', 0.0))
                qw = float(orientation.get('w', 1.0))
            elif isinstance(orientation, list) and len(orientation) >= 4:
                qx = float(orientation[0])
                qy = float(orientation[1])
                qz = float(orientation[2])
                qw = float(orientation[3])
            else:
                # 如果没有orientation，使用单位矩阵
                self.logger.warning('未找到orientation数据，使用单位旋转矩阵')
                return T
            
            # 四元数转旋转矩阵
            R = np.array([
                [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
            ], dtype=np.float64)
            T[:3, :3] = R
            
            return T
            
        except Exception as e:
            self.logger.warning(f'从JSON加载姿态失败: {e}')
            return None
    
    def _robot_status_callback(self, msg: RobotStatus):
        """机器人状态回调函数
        
        Args:
            msg: 机器人状态消息
        """
        try:
            with self.robot_status_lock:
                self.latest_robot_status = msg
        except Exception as e:
            self.logger.warning(f'更新机器人状态失败: {e}')
    
    def _get_current_T_B_E(self) -> Optional[np.ndarray]:
        """从最新机器人状态获取当前 T_B_E（基座到末端执行器）
        
        Returns:
            4x4变换矩阵，如果无法获取则返回None
        """
        if not ROBOT_AVAILABLE or self.latest_robot_status is None:
            return None
        
        try:
            with self.robot_status_lock:
                if self.latest_robot_status is None:
                    return None
                
                cartesian = self.latest_robot_status.cartesian_position
                pos = cartesian.position
                ori = cartesian.orientation
                
                # 构建 T_B_E
                T_B_E = np.eye(4, dtype=np.float64)
                T_B_E[0, 3] = float(pos.x)
                T_B_E[1, 3] = float(pos.y)
                T_B_E[2, 3] = float(pos.z)
                
                # 四元数转旋转矩阵
                qx, qy, qz, qw = float(ori.x), float(ori.y), float(ori.z), float(ori.w)
                R = np.array([
                    [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                    [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
                    [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
                ], dtype=np.float64)
                T_B_E[:3, :3] = R
                
                return T_B_E
        except Exception as e:
            self.logger.warning(f'从机器人状态获取 T_B_E 失败: {e}')
            return None
    
    def publish_system_status(self, status: str):
        """发布系统状态
        
        Args:
            status: 状态信息
        """
        try:
            msg = String()
            msg.data = status
            self.status_publisher.publish(msg)
        except Exception as e:
            self.logger.warning(f'发布系统状态失败: {e}')
    
    def _handle_estimate_pose(
        self,
        request: EstimatePose.Request,
        response: EstimatePose.Response
    ) -> EstimatePose.Response:
        """处理姿态估计服务请求
        
        Args:
            request: 服务请求（包含深度图和可选的彩色图）
            response: 服务响应
            
        Returns:
            服务响应
        """
        start_time = time.time()
        
        try:
            self._rembg_mask_cache.clear()
            self._rembg_cutout_cache.clear()
            self.logger.info(f'姿态估计请求: 工件ID={request.object_id}')
            
            # 1. 获取深度图和彩色图
            depth_image, color_image = self._get_images_from_request(request)
            
            # 1.2 验证图像完整性
            if not self._validate_images(depth_image, color_image, response):
                return response
            
            # 2. 深度图预处理和工件提取
            components, preprocessed_color = self._preprocess_images(depth_image, color_image)
            if not components:
                self.logger.warning('未检测到工件')
                response.success_num = 0
                return response
            
            # 3. 特征提取
            features = self._extract_features(components, preprocessed_color)
            if not features:
                self.logger.warning('未提取到有效特征')
                response.success_num = 0
                return response
            
            # 4. 加载模板库
            if not self._ensure_template_library_loaded(request.object_id, response):
                return response
            
            # 5. 验证手眼标定（仅在未加载时打印警告）
            if not self.actual_calib_file_path or self.T_E_C is None or self.camera_matrix is None:
                self._print_hand_eye_calibration_info()
            
            # 6. 姿态估计
            results = []
            for idx, feature in enumerate(features):
                result = self._process_single_feature(
                    idx, feature, features, components, color_image,
                    depth_image, request.object_id
                )
                if result is not None:
                    results.append(result)
            
            # 6. 填充响应
            self._fill_estimate_pose_response(results, features, preprocessed_color, response)
            
            processing_time = time.time() - start_time
            response.processing_time_sec = processing_time
            
            if len(results) > 0:
                avg_confidence = np.mean([r.confidence for r in results])
                self.logger.info(f'姿态估计完成: 成功数={response.success_num}/{len(features)}, 耗时={processing_time:.3f}秒, 平均置信度={avg_confidence:.4f}')
            else:
                self.logger.info(f'姿态估计完成: 成功数={response.success_num}/{len(features)}, 耗时={processing_time:.3f}秒')
            
        except Exception as e:
            self.logger.error(f'姿态估计服务异常: {e}')
            response.success_num = 0
        
        return response
    
    def _convert_transform_to_cartesian_position(
        self, 
        T: np.ndarray
    ) -> CartesianPosition:
        """将变换矩阵转换为CartesianPosition消息
        
        Args:
            T: 4x4变换矩阵
            
        Returns:
            CartesianPosition消息
        """
        cart_pos = CartesianPosition()
        
        # 位置
        cart_pos.position = Point()
        cart_pos.position.x = float(T[0, 3])
        cart_pos.position.y = float(T[1, 3])
        cart_pos.position.z = float(T[2, 3])
        
        # 旋转（四元数）
        R = T[:3, :3]
        q = self._rotation_matrix_to_quaternion(R)
        cart_pos.orientation = Quaternion()
        # 四元数取反（与C++版本一致）
        q_out = np.array([-q[0], -q[1], -q[2], -q[3]])
        cart_pos.orientation.x = float(q_out[0])
        cart_pos.orientation.y = float(q_out[1])
        cart_pos.orientation.z = float(q_out[2])
        cart_pos.orientation.w = float(q_out[3])
        
        # 笛卡尔转换往返校验：输出四元数 -> 旋转矩阵，应与 R 一致
        R_from_quat = self._quaternion_to_rotation_matrix(q_out)
        if not np.allclose(R, R_from_quat, atol=1e-5):
            self.logger.warning(
                f'笛卡尔转换往返校验未通过: R 与 四元数->R 最大差 {np.abs(R - R_from_quat).max():.2e}'
            )
        
        # 欧拉角（RPY，ZYX 顺序）
        euler = self._rotation_matrix_to_euler_rpy(R)
        cart_pos.euler_orientation_rpy_rad = [float(e) for e in euler]
        cart_pos.euler_orientation_rpy_deg = [float(np.degrees(e)) for e in euler]
        
        # 关节角度：grab/prep 为估计位姿，无逆解故填 0；preplace/place 若来自模板，由调用方用模板关节覆盖
        cart_pos.joint_position_rad = [0.0] * 6
        cart_pos.joint_position_deg = [0.0] * 6
        
        return cart_pos
    
    def _validate_images(
        self,
        depth_image: Optional[np.ndarray],
        color_image: Optional[np.ndarray],
        response: EstimatePose.Response
    ) -> bool:
        """验证图像完整性
        
        Args:
            depth_image: 深度图
            color_image: 彩色图
            response: 服务响应
            
        Returns:
            是否验证通过
        """
        if depth_image is None or color_image is None:
            missing = []
            if depth_image is None:
                missing.append('深度图')
            if color_image is None:
                missing.append('彩色图')
            
            error_msg = f'缺少必要图像: {", ".join(missing)}'
            self.logger.error(error_msg)
            response.success_num = 0
            return False
        
        # 检查图像尺寸是否匹配
        if depth_image.shape[:2] != color_image.shape[:2]:
            self.logger.warning(f'深度图和彩色图尺寸不匹配: 深度图={depth_image.shape[:2]}, 彩色图={color_image.shape[:2]}')
        
        return True

    def _should_use_rembg(self) -> bool:
        # #region agent log
        import json
        import time
        try:
            with open("/home/mu/IVG/.cursor/debug.log", "a", encoding="utf-8") as f:
                params = self.config_reader.load_debug_thresholds() if self.config_reader else {}
                value = params.get("use_rembg", False)
                result = bool(value) and value != 0
                log_entry = {
                    "sessionId": "rembg-debug",
                    "runId": "run1",
                    "hypothesisId": "C",
                    "location": "ros2_communication.py:_should_use_rembg",
                    "message": "检查 RemBG 是否启用",
                    "data": {
                        "config_value": value,
                        "enabled": result,
                        "has_config_reader": self.config_reader is not None
                    },
                    "timestamp": int(time.time() * 1000)
                }
                f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
        except Exception:
            pass
        # #endregion
        params = self.config_reader.load_debug_thresholds() if self.config_reader else {}
        value = params.get("use_rembg", False)
        return bool(value) and value != 0

    def _get_rembg_processor(self):
        """获取 RemBG 处理器（支持直接/子进程两种模式，兼容旧程序）."""
        # #region agent log
        import json
        import time
        try:
            with open("/home/mu/IVG/.cursor/debug.log", "a", encoding="utf-8") as f:
                log_entry = {
                    "sessionId": "rembg-debug",
                    "runId": "run1",
                    "hypothesisId": "C",
                    "location": "ros2_communication.py:_get_rembg_processor",
                    "message": "获取 RemBG 处理器（direct/subprocess 自动选择）",
                    "data": {
                        "has_processor": self.rembg_processor is not None,
                    },
                    "timestamp": int(time.time() * 1000)
                }
                f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
        except Exception:
            pass
        # #endregion

        if self.rembg_processor is not None:
            return self.rembg_processor

        # 尝试检测系统 Python 环境是否可以直接使用 RemBGProcessor
        direct_available = False
        try:
            import onnxruntime  # type: ignore  # noqa: F401
            import rembg  # type: ignore  # noqa: F401
            direct_available = True
        except Exception as exc:
            # #region agent log
            import json
            import time
            try:
                with open("/home/mu/IVG/.cursor/debug.log", "a", encoding="utf-8") as f:
                    log_entry = {
                        "sessionId": "rembg-debug",
                        "runId": "run1",
                        "hypothesisId": "C",
                        "location": "ros2_communication.py:_get_rembg_processor",
                        "message": "系统环境 RemBG 不可用，将回退到子进程模式",
                        "data": {
                            "error": str(exc),
                            "error_type": type(exc).__name__
                        },
                        "timestamp": int(time.time() * 1000)
                    }
                    f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
            except Exception:
                pass
            # #endregion

        if direct_available:
            # 兼容模式：直接在 ROS2 Python 环境中使用 RemBGProcessor
            self.rembg_processor = RemBGProcessor(prefer_cuda=True)
            # #region agent log
            import json
            import time
            try:
                with open("/home/mu/IVG/.cursor/debug.log", "a", encoding="utf-8") as f:
                    log_entry = {
                        "sessionId": "rembg-debug",
                        "runId": "run1",
                        "hypothesisId": "C",
                        "location": "ros2_communication.py:_get_rembg_processor",
                        "message": "创建 RemBGProcessor（direct 模式）",
                        "data": {
                            "mode": "direct",
                            "providers": self.rembg_processor.providers if self.rembg_processor else None
                        },
                        "timestamp": int(time.time() * 1000)
                    }
                    f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
            except Exception:
                pass
            # #endregion
        else:
            # 默认模式：使用子进程 RemBG（通过 conda 环境）
            self.rembg_processor = SubprocessRemBGProcessor(prefer_cuda=True)
            # #region agent log
            import json
            import time
            try:
                with open("/home/mu/IVG/.cursor/debug.log", "a", encoding="utf-8") as f:
                    log_entry = {
                        "sessionId": "rembg-debug",
                        "runId": "run1",
                        "hypothesisId": "C",
                        "location": "ros2_communication.py:_get_rembg_processor",
                        "message": "创建 SubprocessRemBGProcessor（subprocess 模式）",
                        "data": {
                            "mode": "subprocess"
                        },
                        "timestamp": int(time.time() * 1000)
                    }
                    f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
            except Exception:
                pass
            # #endregion

        return self.rembg_processor

    def _resolve_rembg_bbox(
        self,
        feature: Optional[ComponentFeature],
        component_mask: Optional[np.ndarray],
        image_shape: Tuple[int, int]
    ) -> Optional[Tuple[int, int, int, int]]:
        if feature and feature.workpiece_center and feature.workpiece_radius > 0:
            cx, cy = feature.workpiece_center
            radius = feature.workpiece_radius
            x = int(round(cx - radius))
            y = int(round(cy - radius))
            w = int(round(radius * 2))
            h = int(round(radius * 2))
            return (x, y, w, h)

        if component_mask is None or component_mask.size == 0:
            return None

        ys, xs = np.where(component_mask > 0)
        if ys.size == 0 or xs.size == 0:
            return None

        x0 = int(xs.min())
        x1 = int(xs.max())
        y0 = int(ys.min())
        y1 = int(ys.max())
        return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)

    def _get_rembg_outputs(
        self,
        idx: int,
        color_image: np.ndarray,
        feature: Optional[ComponentFeature] = None,
        component_mask: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if idx in self._rembg_mask_cache and idx in self._rembg_cutout_cache:
            return self._rembg_mask_cache[idx], self._rembg_cutout_cache[idx]

        bbox = self._resolve_rembg_bbox(
            feature,
            component_mask,
            color_image.shape[:2]
        )
        if bbox is None:
            return None, None

        processor = self._get_rembg_processor()
        mask, cutout = processor.process_roi(color_image, bbox)
        if mask is None or cutout is None:
            return None, None

        self._rembg_mask_cache[idx] = mask
        self._rembg_cutout_cache[idx] = cutout
        return mask, cutout

    def _clear_rembg_cache(self) -> None:
        """清空 RemBG 结果缓存，避免跨模板 / 跨姿态复用错误的掩模。"""
        self._rembg_mask_cache.clear()
        self._rembg_cutout_cache.clear()
    
    def _preprocess_images(
        self,
        depth_image: np.ndarray,
        color_image: np.ndarray
    ) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
        """预处理图像：二值化和连通域提取
        
        Args:
            depth_image: 深度图
            color_image: 彩色图
            
        Returns:
            (components, preprocessed_color) 连通域列表和预处理后的彩色图
        """
        debug_params = self.config_reader.load_debug_thresholds()
        binary_threshold_min, binary_threshold_max = self._apply_debug_params_to_preprocessor(debug_params)
        
        components, preprocessed_color = self.preprocessor.preprocess(
            depth_image,
            color_image,
            int(binary_threshold_min),
            int(binary_threshold_max)
        )
        self.logger.info(f'预处理完成: 连通域数={len(components)}')
        
        return components, preprocessed_color
    
    def _extract_features(
        self,
        components: List[np.ndarray],
        preprocessed_color: Optional[np.ndarray]
    ) -> List[ComponentFeature]:
        """提取特征
        
        Args:
            components: 连通域列表
            preprocessed_color: 预处理后的彩色图
            
        Returns:
            特征列表
        """
        features = self.feature_extractor.extract_features(components, preprocessed_color)
        self.logger.info(f'特征提取完成: 特征数={len(features)}')
        
        return features
    
    def _ensure_template_library_loaded(
        self,
        object_id: str,
        response: EstimatePose.Response
    ) -> bool:
        """确保模板库已加载
        
        Args:
            object_id: 工件ID
            response: 服务响应
            
        Returns:
            是否加载成功
        """
        if not self.pose_estimator.templates:
            template_dir = Path(self.template_root) / object_id
            if not template_dir.exists():
                self.logger.error(f'模板目录不存在: {template_dir}')
                response.success_num = 0
                return False
            
            success = self.pose_estimator.load_template_library(
                str(template_dir),
                self.camera_matrix if self.camera_matrix is not None else np.eye(3),
                self.T_E_C if self.T_E_C is not None else np.eye(4)
            )
            
            if not success:
                self.logger.error(f'模板加载失败')
                response.success_num = 0
                return False
            
            self.logger.info(f'模板加载完成: {len(self.pose_estimator.templates)}个模板')
        
        return True
    
    def _process_single_feature(
        self,
        idx: int,
        feature: ComponentFeature,
        features: List[ComponentFeature],
        components: List[np.ndarray],
        color_image: np.ndarray,
        depth_image: np.ndarray,
        object_id: str
    ) -> Optional[object]:
        """处理单个特征：模板匹配和姿态估计
        
        这是姿态估计流程的核心方法，负责对单个检测到的特征进行完整的处理流程：
        1. 获取目标掩膜（用于模板匹配）
        2. 模板匹配（从模板库中找到最佳匹配模板）
        3. 加载相机位姿（当前和模板拍摄时的相机位姿）
        4. 计算角度差（目标与模板的角度差异）
        5. 姿态估计（计算3D抓取姿态和准备姿态）
        6. 加载附加姿态（预放置和放置姿态）
        7. 输出结果摘要
        
        Args:
            idx: 特征索引（在当前检测到的特征列表中的位置，从0开始）
            feature: 特征对象（ComponentFeature），包含：
                - workpiece_center: 工件中心坐标（像素）
                - workpiece_radius: 工件半径（像素）
                - valve_center: 阀体中心坐标（像素，可选）
                - valve_radius: 阀体半径（像素，可选）
                - standardized_angle: 标准化角度（弧度）
                - standardized_angle_deg: 标准化角度（度）
            features: 所有特征列表（用于日志输出，显示当前处理的是第几个特征）
            components: 连通域列表（二值掩膜数组），每个元素是一个连通域的掩膜
                - 用于提取目标掩膜（components[idx]）
                - 掩膜尺寸可能与彩色图不一致，需要调整
            color_image: 彩色图（BGR格式，numpy数组）
                - 用于提取目标掩膜时的尺寸参考
                - 用于模板匹配时的图像对齐
            depth_image: 深度图（uint16格式，numpy数组）
                - 用于计算3D位置（从像素坐标转换到相机坐标系）
                - 用于姿态估计时的深度值提取
            object_id: 工件ID（字符串，如 "3211242785"）
                - 用于确定模板目录路径（templates/{object_id}/）
                - 用于加载相机位姿和附加姿态
            
        Returns:
            Optional[PoseEstimationResult]: 姿态估计结果对象，包含：
                - template_id: 最佳匹配模板的ID（如 "pose_1"）
                - T_B_E_grasp: 抓取姿态变换矩阵（4x4），从机器人基座到末端执行器
                - T_B_E_prep: 准备姿态变换矩阵（4x4），从机器人基座到末端执行器
                - T_B_E_preplace: 预放置姿态变换矩阵（4x4，可选）
                - T_B_E_place: 放置姿态变换矩阵（4x4，可选）
                - confidence: 匹配置信度（0.0-1.0，暴力匹配时提供）
                - bounding_box: 边界框 [x, y, width, height]（像素坐标）
                - feature_distance: 特征距离（特征距离匹配时提供）
                如果任何步骤失败（模板匹配失败、姿态估计失败等），返回None
        """
        # 处理特征 {idx}/{len(features)-1}
        
        # 获取目标掩膜
        target_mask = self._get_target_mask(idx, feature, components, color_image)
        
        # 模板匹配
        match_result = self._match_template(idx, feature, target_mask, object_id)
        if match_result is None:
            return None
        
        best_idx, distance, confidence, best_angle_deg, best_aligned_mask = match_result
        best_template = self.pose_estimator.templates[best_idx]
        
        # 加载相机位姿
        T_B_C, T_B_C_template = self._load_camera_pose(object_id, best_template)
        
        # 计算角度差（用于姿态估计）
        dtheta_rad = self._calculate_angle_difference(best_angle_deg, confidence)
        
        # 姿态估计
        result = self._estimate_pose_for_feature(
            idx, feature, best_template, T_B_C, T_B_C_template,
            depth_image, distance, dtheta_rad
        )
        if result is None:
            return None
        
        # 加载附加姿态
        self._load_additional_poses(idx, object_id, best_template, result)
        
        # 输出结果摘要
        self._log_pose_result_summary(idx, result)
        
        return result
    
    def _get_target_mask(
        self,
        idx: int,
        feature: ComponentFeature,
        components: List[np.ndarray],
        color_image: np.ndarray
    ) -> Optional[np.ndarray]:
        """获取目标掩膜（用于模板匹配）
        
        Args:
            idx: 特征索引
            components: 连通域列表
            color_image: 彩色图
            
        Returns:
            目标掩膜，如果获取失败则返回None
        """
        target_mask = None
        use_rembg = self._should_use_rembg()
        if use_rembg:
            rembg_mask, rembg_cutout = self._get_rembg_outputs(
                idx,
                color_image,
                feature=feature,
                component_mask=components[idx] if idx < len(components) else None
            )
            if rembg_mask is not None:
                target_mask = rembg_mask
                feature.component_mask = rembg_mask
                if rembg_cutout is not None:
                    feature.color_image = rembg_cutout
        if target_mask is None and idx < len(components):
            target_mask = components[idx].copy()
            original_mask_shape = target_mask.shape[:2]
            self.logger.info(f'    [5.{idx+1}.0] 获取目标掩膜 - 原始尺寸: {original_mask_shape[1]}x{original_mask_shape[0]}')
            
            # 确保目标掩膜与输入图像尺寸一致
            if target_mask.shape[:2] != color_image.shape[:2]:
                target_mask = cv2.resize(target_mask, (color_image.shape[1], color_image.shape[0]))
                self.logger.info(f'      掩膜尺寸调整: {original_mask_shape[1]}x{original_mask_shape[0]} -> {color_image.shape[1]}x{color_image.shape[0]}')
            else:
                self.logger.info(f'      掩膜尺寸与图像一致，无需调整')
        else:
            self.logger.warning(f'    [5.{idx+1}.0] 目标掩膜获取失败 - 索引 {idx} 超出组件列表范围 (共 {len(components)} 个组件)')
        
        return target_mask
    
    def _match_template(
        self,
        idx: int,
        feature: ComponentFeature,
        target_mask: Optional[np.ndarray],
        object_id: str
    ) -> Optional[Tuple[int, float, Optional[float], Optional[float], Optional[np.ndarray]]]:
        """执行模板匹配
        
        Args:
            idx: 特征索引
            feature: 特征对象
            target_mask: 目标掩膜
            object_id: 工件ID
            
        Returns:
            (best_idx, distance, confidence, best_angle_deg, best_aligned_mask) 匹配结果，如果失败则返回None
        """
        self.logger.info(f'    [5.{idx+1}.1] 模板匹配')
        workpiece_template_dir = str(Path(self.template_root) / object_id)
        self.logger.info(f'      模板目录: {workpiece_template_dir}')
        self.logger.info(f'      目标掩膜: {"提供" if target_mask is not None else "未提供"}')
        self.logger.info(f'      暴力匹配: {"启用" if self.pose_estimator.brute_force_matching_enabled else "禁用"}')
        
        match_start = time.time()
        result_tuple = self.pose_estimator.select_best_template(
            feature,
            target_mask=target_mask,
            workpiece_template_dir=workpiece_template_dir
        )
        match_time = time.time() - match_start
        
        best_idx, distance, confidence, best_angle_deg, best_aligned_mask = result_tuple
        
        if best_idx < 0:
            self.logger.warning(f'    ✗ 特征 {idx} 未找到匹配模板')
            return None
        
        best_template = self.pose_estimator.templates[best_idx]
        self.logger.info(f'    ✓ 模板匹配完成，耗时: {match_time:.3f}秒')
        
        # 记录匹配信息
        if self.pose_estimator.brute_force_matching_enabled and confidence is not None:
            self.logger.info(f'模板匹配: 模板={best_template.id}, 置信度={confidence:.4f}, 角度={best_angle_deg:.2f}°')
        else:
            self.logger.info(f'模板匹配: 模板={best_template.id}, 距离={distance:.4f}')
        
        return result_tuple
    
    def _load_camera_pose(
        self,
        object_id: str,
        best_template: TemplateItem
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """加载相机位姿
        
        Args:
            object_id: 工件ID
            best_template: 最佳模板
            
        Returns:
            (T_B_C, T_B_C_template) 相机位姿和模板相机位姿
        """
        # 尝试从机器人获取当前 T_B_E，计算 T_B_C = T_B_E @ T_E_C
        T_B_C = None
        T_B_E_current = self._get_current_T_B_E()
        if T_B_E_current is not None and self.T_E_C is not None and not np.allclose(self.T_E_C, np.eye(4)):
            T_B_C = T_B_E_current @ self.T_E_C
            pos = T_B_C[:3, 3]
            self.logger.info(f'从机器人状态获取相机位姿T_B_C: 位置=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})m')
        else:
            # 回退：使用单位矩阵（警告）
            T_B_C = np.eye(4)
            if T_B_E_current is None:
                self.logger.warning(f'无法获取机器人状态，使用单位矩阵T_B_C')
            elif self.T_E_C is None or np.allclose(self.T_E_C, np.eye(4)):
                self.logger.warning(f'T_E_C未设置或为单位矩阵，使用单位矩阵T_B_C')
        
        # 尝试从模板目录加载相机位姿（如果存在）
        T_B_C_template = None
        try:
            template_dir = Path(self.template_root) / object_id / best_template.id
            camera_pose_file = template_dir / "camera_pose.json"
            if camera_pose_file.exists():
                with open(camera_pose_file, 'r') as f:
                    camera_pose_data = json.load(f)
                    if 'cartesian_position' in camera_pose_data:
                        pos = camera_pose_data['cartesian_position']['position']
                        ori = camera_pose_data['cartesian_position']['orientation']
                        # 构建T_B_E（基座到末端）
                        T_B_E_camera = np.eye(4)
                        T_B_E_camera[0, 3] = pos.get('x', 0.0)
                        T_B_E_camera[1, 3] = pos.get('y', 0.0)
                        T_B_E_camera[2, 3] = pos.get('z', 0.0)
                        # 四元数转旋转矩阵
                        qx, qy, qz, qw = ori.get('x', 0.0), ori.get('y', 0.0), ori.get('z', 0.0), ori.get('w', 1.0)
                        R = np.array([
                            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
                            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
                        ])
                        T_B_E_camera[:3, :3] = R
                        # T_B_C = T_B_E * T_E_C
                        T_E_C = self.T_E_C if self.T_E_C is not None else np.eye(4)
                        T_B_C_template = T_B_E_camera @ T_E_C
                        self.logger.info(f'从模板加载相机位姿: {best_template.id}')
        except Exception as e:
            self.logger.warning(f'加载模板相机位姿失败: {e}')
        
        return T_B_C, T_B_C_template
    
    def _calculate_angle_difference(
        self,
        best_angle_deg: Optional[float],
        confidence: Optional[float]
    ) -> Optional[float]:
        """计算角度差（用于姿态估计）
        
        Args:
            best_angle_deg: 最佳角度（度）
            confidence: 置信度
            
        Returns:
            角度差（弧度），如果不需要则返回None
        """
        dtheta_rad = None
        if self.pose_estimator.brute_force_matching_enabled and best_angle_deg is not None:
            # 将角度从度转换为弧度
            dtheta_rad = np.deg2rad(best_angle_deg)
            # 归一化到[-π, π]范围
            while dtheta_rad > np.pi:
                dtheta_rad -= 2 * np.pi
            while dtheta_rad < -np.pi:
                dtheta_rad += 2 * np.pi
            # 保存置信度供estimate_pose使用
            self.pose_estimator._last_match_confidence = confidence
        
        return dtheta_rad
    
    def _estimate_pose_for_feature(
        self,
        idx: int,
        feature: ComponentFeature,
        best_template: TemplateItem,
        T_B_C: np.ndarray,
        T_B_C_template: Optional[np.ndarray],
        depth_image: np.ndarray,
        distance: float,
        dtheta_rad: Optional[float]
    ) -> Optional[object]:
        """为单个特征估计姿态
        
        Args:
            idx: 特征索引
            feature: 特征对象
            best_template: 最佳模板
            T_B_C: 相机位姿
            T_B_C_template: 模板相机位姿
            depth_image: 深度图
            distance: 特征距离
            dtheta_rad: 角度差（弧度）
            
        Returns:
            姿态估计结果，如果失败则返回None
        """
        # 姿态计算（详细日志在 pose_estimator.py 中）
        
        pose_estimate_start = time.time()
        result = self.pose_estimator.estimate_pose(
            feature,
            best_template,
            self.camera_matrix if self.camera_matrix is not None else np.eye(3),
            T_B_C,
            self.T_E_C if self.T_E_C is not None else np.eye(4),
            depth_image=depth_image,
            T_B_C_template=T_B_C_template,
            feature_distance=distance,
            dtheta_rad=dtheta_rad
        )
        pose_estimate_time = time.time() - pose_estimate_start
        self.logger.info(f'    ✓ 姿态计算完成，耗时: {pose_estimate_time:.3f}秒')
        
        return result
    
    def _load_additional_poses(
        self,
        idx: int,
        object_id: str,
        best_template: TemplateItem,
        result: object
    ) -> None:
        """加载附加姿态（预放置和放置姿态）
        
        Args:
            idx: 特征索引
            object_id: 工件ID
            best_template: 最佳模板
            result: 姿态估计结果（会被修改）
        """
        template_dir = Path(self.template_root) / object_id / best_template.id
        if template_dir.exists():
            # 加载预放置姿态
            preplace_json = template_dir / "preplace_position.json"
            if preplace_json.exists():
                try:
                    self.logger.info(f'读取预放置姿态文件: {preplace_json}')
                    with open(preplace_json, 'r', encoding='utf-8') as f:
                        preplace_data = json.load(f)
                    T_B_E_preplace = self._load_pose_json_data(preplace_data)
                    if T_B_E_preplace is not None:
                        result.T_B_E_preplace = T_B_E_preplace
                        prep_pos = T_B_E_preplace[:3, 3]
                        self.logger.info(f'预放置姿态: 位置=({prep_pos[0]:.4f}, {prep_pos[1]:.4f}, {prep_pos[2]:.4f})m')
                    # 保留模板关节角（preplace/place 来自模板 JSON，有真实关节）
                    jd = preplace_data.get('joint_position_deg')
                    jr = preplace_data.get('joint_position_rad')
                    if isinstance(jd, (list, tuple)) and len(jd) >= 6:
                        result.joint_position_deg_preplace = [float(x) for x in jd[:6]]
                    if isinstance(jr, (list, tuple)) and len(jr) >= 6:
                        result.joint_position_rad_preplace = [float(x) for x in jr[:6]]
                except Exception as e:
                    self.logger.warning(f'加载预放置姿态失败: {e}')
            
            # 加载放置姿态
            place_json = template_dir / "place_position.json"
            if place_json.exists():
                try:
                    self.logger.info(f'读取放置姿态文件: {place_json}')
                    with open(place_json, 'r', encoding='utf-8') as f:
                        place_data = json.load(f)
                    T_B_E_place = self._load_pose_json_data(place_data)
                    if T_B_E_place is not None:
                        result.T_B_E_place = T_B_E_place
                        place_pos = T_B_E_place[:3, 3]
                        self.logger.info(f'放置姿态: 位置=({place_pos[0]:.4f}, {place_pos[1]:.4f}, {place_pos[2]:.4f})m')
                    # 保留模板关节角
                    jd = place_data.get('joint_position_deg')
                    jr = place_data.get('joint_position_rad')
                    if isinstance(jd, (list, tuple)) and len(jd) >= 6:
                        result.joint_position_deg_place = [float(x) for x in jd[:6]]
                    if isinstance(jr, (list, tuple)) and len(jr) >= 6:
                        result.joint_position_rad_place = [float(x) for x in jr[:6]]
                except Exception as e:
                    self.logger.warning(f'加载放置姿态失败: {e}')
    
    def _log_pose_result_summary(
        self,
        idx: int,
        result: object
    ) -> None:
        """输出姿态估计结果摘要
        
        Args:
            idx: 特征索引
            result: 姿态估计结果
        """
        self.logger.info(f'特征{idx}结果: 模板ID={result.template_id}, 置信度={result.confidence:.4f}')
    
    def _fill_estimate_pose_response(
        self,
        results: List[object],
        features: List[ComponentFeature],
        preprocessed_color: Optional[np.ndarray],
        response: EstimatePose.Response
    ) -> None:
        """填充姿态估计服务响应
        
        Args:
            results: 姿态估计结果列表
            features: 特征列表
            preprocessed_color: 预处理后的彩色图
            response: 服务响应（会被修改）
        """
        response.success_num = len(results)
        
        for result in results:
            # 位置（2D）
            point = Point()
            point.x = float(result.bounding_box[0] + result.bounding_box[2] / 2)
            point.y = float(result.bounding_box[1] + result.bounding_box[3] / 2)
            point.z = 0.0
            response.position.append(point)
            
            # 抓取姿态
            grab_pos = self._convert_transform_to_cartesian_position(result.T_B_E_grasp)
            response.grab_position.append(grab_pos)
            
            # 准备姿态
            if result.T_B_E_prep is not None and not np.allclose(result.T_B_E_prep, np.eye(4)):
                prep_pos = self._convert_transform_to_cartesian_position(result.T_B_E_prep)
                response.preparation_position.append(prep_pos)
            else:
                # 如果没有准备姿态，使用抓取姿态
                response.preparation_position.append(grab_pos)
            
            # 预放置姿态
            if result.T_B_E_preplace is not None and not np.allclose(result.T_B_E_preplace, np.eye(4)):
                preplace_pos = self._convert_transform_to_cartesian_position(result.T_B_E_preplace)
                response.preplace_position.append(preplace_pos)
                # 用模板关节角覆盖（preplace 来自模板 JSON）
                jd = getattr(result, 'joint_position_deg_preplace', None)
                jr = getattr(result, 'joint_position_rad_preplace', None)
                if jd is not None and len(jd) >= 6:
                    response.preplace_position[-1].joint_position_deg = list(jd[:6])
                if jr is not None and len(jr) >= 6:
                    response.preplace_position[-1].joint_position_rad = list(jr[:6])
            else:
                # 如果没有预放置姿态，创建空的
                empty_pos = CartesianPosition()
                response.preplace_position.append(empty_pos)
            
            # 放置姿态
            if result.T_B_E_place is not None and not np.allclose(result.T_B_E_place, np.eye(4)):
                place_pos = self._convert_transform_to_cartesian_position(result.T_B_E_place)
                response.place_position.append(place_pos)
                # 用模板关节角覆盖（place 来自模板 JSON）
                jd = getattr(result, 'joint_position_deg_place', None)
                jr = getattr(result, 'joint_position_rad_place', None)
                if jd is not None and len(jd) >= 6:
                    response.place_position[-1].joint_position_deg = list(jd[:6])
                if jr is not None and len(jr) >= 6:
                    response.place_position[-1].joint_position_rad = list(jr[:6])
            else:
                # 如果没有放置姿态，创建空的
                empty_pos = CartesianPosition()
                response.place_position.append(empty_pos)
            
            # 匹配到的模板姿态ID（用于回到该模板的拍照姿态等流程）
            if hasattr(response, 'matched_pose_ids'):
                response.matched_pose_ids.append(str(result.template_id))

            # 置信度
            response.confidence.append(result.confidence)
        
        # 生成可视化图像（如果有彩色图）
        if preprocessed_color is not None and features:
            try:
                # 使用第一个特征绘制可视化
                # 如果启用了 RemBG，则优先使用特征中保存的白底抠图（feature.color_image）
                base_image = features[0].color_image if features[0].color_image is not None else preprocessed_color
                vis_image = features[0].draw_features(base_image)
                
                # 转换为ROS图像消息
                vis_msg = self.cv_bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
                response.pose_image.append(vis_msg)
                
            except Exception as e:
                self.logger.warning(f'生成可视化图像失败: {e}')
    
    def _get_images_from_request(
        self,
        request: EstimatePose.Request
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """从请求中获取深度图和彩色图
        
        优先使用请求中的图像，如果没有则通过触发拍照获取
        
        Args:
            request: 服务请求
            
        Returns:
            (depth_image, color_image) 深度图和彩色图的元组
        """
        depth_image = None
        color_image = None
        
        # 检查请求中的图像
        has_depth_in_request = bool(getattr(request, "image", None) and getattr(request.image, "data", None))
        has_color_in_request = bool(getattr(request, "color_image", None) and getattr(request.color_image, "data", None))
        
        # 1.1 尝试从请求中获取深度图
        if has_depth_in_request:
            try:
                depth_image = self.cv_bridge.imgmsg_to_cv2(request.image, desired_encoding='passthrough')
            except Exception as e:
                self.logger.warning(f'深度图像转换失败: {e}')
        
        # 1.2 尝试从请求中获取彩色图
        if has_color_in_request:
            try:
                color_image = self.cv_bridge.imgmsg_to_cv2(request.color_image, desired_encoding='bgr8')
            except Exception as e:
                self.logger.warning(f'彩色图像转换失败: {e}')
        
        # 1.3 如果请求中没有图像，触发拍照后临时订阅获取图像
        if depth_image is None or color_image is None:
            # 临时订阅并等待图像
            trigger_start = time.time()
            latest_depth, latest_color = self._capture_images_on_trigger(timeout=10.0)
            trigger_time = time.time() - trigger_start
            
            if depth_image is None:
                if latest_depth is not None:
                    depth_image = latest_depth
                else:
                    self.logger.error(f'未获取到深度图')
            
            if color_image is None:
                if latest_color is not None:
                    color_image = latest_color
                else:
                    self.logger.error(f'未获取到彩色图')
        
        return depth_image, color_image
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])
    
    def _rotation_matrix_to_euler_rpy(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转欧拉角（RPY, ZYX顺序）"""
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        return np.array([roll, pitch, yaw])
    
    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """四元数 [x,y,z,w] 转 3x3 旋转矩阵（用于笛卡尔转换往返校验）"""
        x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
        ], dtype=np.float64)
        return R
    
    def _handle_list_templates(
        self,
        request: ListTemplates.Request,
        response: ListTemplates.Response
    ) -> ListTemplates.Response:
        """处理列出模板服务请求"""
        try:
            # 使用配置的模板根目录
            template_root_path = Path(self.template_root)
            
            if not template_root_path.exists():
                self.logger.error(f'模板目录不存在: {self.template_root}')
                response.success = False
                response.error_message = f'模板目录不存在: {self.template_root}'
                response.template_ids = []
                response.workpiece_ids = []
                return response
            
            # 如果指定了工件ID，只列出该工件
            if request.workpiece_id:
                workpiece_path = template_root_path / request.workpiece_id
                if workpiece_path.exists() and workpiece_path.is_dir():
                    response.template_ids = [request.workpiece_id]
                    response.workpiece_ids = [request.workpiece_id]
                else:
                    response.success = False
                    response.error_message = f'工件不存在: {request.workpiece_id}'
                    response.template_ids = []
                    response.workpiece_ids = []
                    return response
            else:
                # 列出所有工件（模板）
                template_ids = []
                for item in template_root_path.iterdir():
                    if item.is_dir() and not item.name.startswith('.'):
                        template_ids.append(item.name)
                
                response.template_ids = sorted(template_ids)
                response.workpiece_ids = sorted(template_ids)
            
            response.success = True
            self.logger.info(f'找到 {len(response.template_ids)} 个模板')
            
        except Exception as e:
            self.logger.error(f'列出模板失败: {e}')
            response.success = False
            response.error_message = str(e)
            response.template_ids = []
            response.workpiece_ids = []
        
        return response
    
    def _handle_update_params(
        self,
        request: UpdateParams.Request,
        response: UpdateParams.Response
    ) -> UpdateParams.Response:
        """处理参数更新请求（触发重新加载debug_thresholds.json）
        
        统一更新所有算法模块的阈值参数：
        - Preprocessor: 使用所有阈值参数
        - FeatureExtractor: 使用连通域筛选参数
        
        Args:
            request: 包含section的请求
            response: 响应对象
            
        Returns:
            更新后的响应
        """
        try:
            section = request.section
            self.logger.info(f'收到参数更新请求: section={section}')
            
            # 统一使用config_reader重新加载阈值参数
            debug_thresholds = self.config_reader.load_debug_thresholds()
            
            # 更新Preprocessor参数（阈值参数从config_reader获取，其他参数使用默认配置）
            if section == 'preprocessor' or section == 'all':
                preprocessor_params = self._merge_threshold_params(
                    self.config_reader.get_section('preprocessor'),
                    debug_thresholds,
                    'preprocessor'
                )
                self.preprocessor.set_parameters(preprocessor_params)
            
            # 更新FeatureExtractor参数（阈值参数从config_reader获取，其他参数使用默认配置）
            if section == 'feature_extractor' or section == 'all':
                feature_extractor_params = self._merge_threshold_params(
                    self.config_reader.get_section('feature_extractor'),
                    debug_thresholds,
                    'feature_extractor'
                )
                self.feature_extractor.set_parameters(feature_extractor_params)
            
            response.success = True
            response.message = f'参数已从debug_thresholds.json重新加载并应用到所有算法模块'
            self.logger.info(f'参数更新完成: section={section}')
            
        except Exception as e:
            response.success = False
            response.message = f'参数重新加载失败: {e}'
            self.logger.error(f'参数重新加载失败: {e}')
        
        return response
    
    def _handle_standardize_template(
        self,
        request: StandardizeTemplate.Request,
        response: StandardizeTemplate.Response
    ) -> StandardizeTemplate.Response:
        """处理标准化模板服务请求
        
        功能说明：
        1. 遍历模板目录下的所有pose目录（pose_1, pose_2, ...）
        2. 对每个pose目录进行标准化处理：
           - 读取原始图像（original_image.jpg + depth_image.png）
           - 使用与Debug选项卡相同的阈值参数进行预处理和特征提取
           - 生成标准化图像和掩码（image.jpg + mask.jpg）
           - 计算标准化姿态（如果存在原始姿态数据）
           - 保存template_info.json和preprocessed_image.jpg
        3. 返回处理结果统计（成功数、跳过数、处理的pose列表等）
        
        注意：
        - 标准化基于模板目录中已保存的图像，不需要实时拍照
        - 使用debug_thresholds.json中的阈值参数，确保与Debug预览一致
        - 如果某个pose处理失败，会跳过并继续处理其他pose
        
        Args:
            request: 服务请求（包含workpiece_id）
            response: 服务响应
            
        Returns:
            服务响应（包含处理结果统计）
        """
        t0 = time.time()
        try:
            # 步骤1: 获取并验证工件ID
            template_id = getattr(request, 'template_id', getattr(request, 'workpiece_id', ''))
            if not template_id:
                response.success = False
                if hasattr(response, 'error_message'):
                    response.error_message = '工件ID不能为空'
                return response

            # 步骤2: 验证模板目录是否存在
            template_dir = Path(self.template_root) / str(template_id)
            self.logger.info(f'收到模板标准化请求: workpiece_id={template_id}, dir={template_dir}')

            if not template_dir.exists():
                response.success = False
                if hasattr(response, 'error_message'):
                    response.error_message = f'模板目录不存在: {template_dir}'
                return response

            # 初始化处理结果统计列表
            processed_pose_ids = []  # 成功处理的姿态ID列表
            skipped_pose_ids = []  # 跳过的姿态ID列表

            # 步骤3: 加载阈值参数
            debug_params = self.config_reader.load_debug_thresholds()
            binary_threshold_min, binary_threshold_max = self._apply_debug_params_to_preprocessor(debug_params)

            # 步骤4: 查找所有pose目录（pose_1, pose_2, ...）
            pose_dirs = [
                p for p in template_dir.iterdir() #返回目录中所有条目的迭代器
                if p.is_dir() and p.name.startswith('pose_')
            ]
            pose_dirs.sort(key=lambda p: p.name)  # 按名称排序，确保处理顺序一致
            print(f'模板目录下的pose_dirs: {pose_dirs}')

            if not pose_dirs:
                response.success = False
                if hasattr(response, 'error_message'):
                    response.error_message = f'未找到任何pose目录: {template_dir}'
                return response

            # 步骤5: 遍历处理每个pose目录
            for pose_dir in pose_dirs:
                pose_id = pose_dir.name[5:]  # 去掉 "pose_" 前缀，获取pose ID（如 "1", "2"）
                try:
                    # 对单个pose目录进行标准化处理
                    # 包括：图像加载、预处理、特征提取、标准化、姿态计算、保存结果
                    success = self._standardize_single_pose_directory(
                        pose_id, pose_dir, template_dir,
                        binary_threshold_min, binary_threshold_max
                    )
                    if success:
                        processed_pose_ids.append(pose_id)
                        self.logger.info(f'姿态 {pose_id} 标准化成功')
                    else:
                        skipped_pose_ids.append(pose_id)

                except Exception as e:
                    # 单个pose处理异常不影响其他pose的处理
                    self.logger.warning(f'姿态 {pose_id} 标准化异常，已跳过: {e}')
                    skipped_pose_ids.append(pose_id)

            # 步骤6: 统计处理结果并填充响应
            processed_count = len(processed_pose_ids)
            skipped_count = len(skipped_pose_ids)

            response.success = (processed_count > 0)  # 至少成功处理一个pose才算成功
            if hasattr(response, 'processed_count'):
                response.processed_count = processed_count
            if hasattr(response, 'skipped_count'):
                response.skipped_count = skipped_count
            if hasattr(response, 'processed_pose_ids'):
                response.processed_pose_ids = processed_pose_ids
            if hasattr(response, 'skipped_pose_ids'):
                response.skipped_pose_ids = skipped_pose_ids

            # 设置错误消息（如果有失败的情况）
            if hasattr(response, 'error_message'):
                if processed_count == 0 and skipped_count > 0:
                    response.error_message = '所有姿态标准化失败'
                elif skipped_count > 0:
                    response.error_message = '部分姿态标准化失败'
                else:
                    response.error_message = ''

            # 设置成功消息
            if hasattr(response, 'message'):
                response.message = (
                    f'模板标准化完成: 处理 {processed_count} 个姿态, 跳过 {skipped_count} 个姿态'
                )

            # 记录处理耗时
            dt = time.time() - t0
            self.logger.info(
                f'模板标准化完成: 处理 {processed_count} 个姿态, 跳过 {skipped_count} 个姿态, 用时 {dt:.3f}s'
            )
            
        except Exception as e:
            self.logger.error(f'标准化模板失败: {e}')
            response.success = False
            if hasattr(response, 'message'):
                response.message = f"标准化模板异常: {str(e)}"
            if hasattr(response, 'error_message'):
                response.error_message = f"标准化模板异常: {str(e)}"
        
        return response
    
    def _standardize_single_pose_directory(
        self,
        pose_id: str,
        pose_dir: Path,
        template_dir: Path,
        binary_threshold_min: int,
        binary_threshold_max: int
    ) -> bool:
        """标准化单个pose目录
        
        Args:
            pose_id: 姿态ID
            pose_dir: 姿态目录
            template_dir: 模板目录
            binary_threshold_min: 最小阈值
            binary_threshold_max: 最大阈值
            
        Returns:
            是否成功
        """
        # 读取和验证图像
        depth_image, color_image = self._load_pose_images(pose_id, pose_dir)
        if depth_image is None or color_image is None:
            return False

        # 诊断深度图并检查阈值
        self._diagnose_depth_image(pose_id, depth_image, binary_threshold_min, binary_threshold_max)


        # 预处理和特征提取
        feature, preprocessed_color = self._process_pose_for_standardization(
            pose_id, depth_image, color_image, binary_threshold_min, binary_threshold_max
        )
        if feature is None:
            return False
        print(f'预处理和特征提取成功')


        # 标准化后的图像, 标准化后的掩码, 裁剪参数字典
        standardized_image, standardized_mask, crop_params = self._standardize_single_pose(
            pose_id, feature, preprocessed_color, color_image
        )
        print(f'标准化图像, 标准化掩码, 裁剪参数字典成功')
        
        # 加载抓取姿态, 准备姿态, 相机位姿
        T_B_E_grasp, T_B_E_preparation, T_B_E_camera = self._load_pose_data(
            pose_id, pose_dir
        )
        print(f'加载抓取姿态, 准备姿态, 相机位姿成功')
        
        # 计算标准化姿态
        T_B_E_standardized_grasp, T_B_E_standardized_preparation = self._compute_standardized_poses(
            pose_id, feature, crop_params, T_B_E_grasp, T_B_E_preparation,
            T_B_E_camera
        )
        print(f'计算标准化姿态成功')

        self._generate_gripper_visualization(
            pose_id, pose_dir, color_image, T_B_E_grasp, T_B_E_camera
        )
        print(f'生成夹爪可视化成功')
        
        # 保存标准化模板
        ok = self.template_standardizer.save_standardized_template(
            template_dir,
            pose_dir.name,
            standardized_image,
            standardized_mask,
            feature,
            T_B_E_grasp,
            T_B_E_standardized_grasp,
            T_B_E_preparation,
            T_B_E_standardized_preparation,
            crop_params=crop_params,
            save_metadata=False,
            preprocessed_image=preprocessed_color
        )
        
        return ok
    
    def _load_pose_images(
        self,
        pose_id: str,
        pose_dir: Path
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """加载pose目录中的图像
        
        Args:
            pose_id: 姿态ID
            pose_dir: 姿态目录
            
        Returns:
            (depth_image, color_image) 深度图和彩色图，如果加载失败则返回None
        """
        color_path = pose_dir / 'original_image.jpg'
        depth_path = pose_dir / 'depth_image.png'

        missing = []
        if not depth_path.exists():
            missing.append('深度图')
        if not color_path.exists():
            missing.append('彩色图')
        if missing:
            self.logger.warning(f'姿态 {pose_id} 跳过：缺少必要图像: {", ".join(missing)}')
            return None, None

        # 读取图像
        depth_image = cv2.imread(str(depth_path), cv2.IMREAD_ANYDEPTH)
        color_image = cv2.imread(str(color_path), cv2.IMREAD_COLOR)

        if depth_image is None:
            self.logger.warning(f'姿态 {pose_id} 跳过：无法读取深度图 {depth_path}')
            return None, None
        if color_image is None:
            self.logger.warning(f'姿态 {pose_id} 跳过：无法读取彩色图 {color_path}')
            return None, None
        
        return depth_image, color_image
    
    def _diagnose_depth_image(
        self,
        pose_id: str,
        depth_image: np.ndarray,
        binary_threshold_min: int,
        binary_threshold_max: int
    ) -> None:
        """诊断深度图并检查阈值
        
        Args:
            pose_id: 姿态ID
            depth_image: 深度图
            binary_threshold_min: 最小阈值
            binary_threshold_max: 最大阈值
        """
        valid_mask = (depth_image > 0) & (depth_image < 65535)
        if np.any(valid_mask):
            depth_min = int(np.min(depth_image[valid_mask]))
            depth_max = int(np.max(depth_image[valid_mask]))
            depth_mean = int(np.mean(depth_image[valid_mask]))
            valid_count = int(np.sum(valid_mask))
            total_pixels = depth_image.size
            self.logger.info(
                f'姿态 {pose_id} 深度图统计: '
                f'shape={depth_image.shape}, dtype={depth_image.dtype}, '
                f'有效像素={valid_count}/{total_pixels}, '
                f'范围=[{depth_min}, {depth_max}], 均值={depth_mean}'
            )
            self.logger.info(
                f'姿态 {pose_id} 使用阈值: '
                f'binary_threshold=[{binary_threshold_min}, {binary_threshold_max}]'
            )
            # 检查阈值是否在深度范围内
            if binary_threshold_min > depth_max:
                self.logger.warning(
                    f'姿态 {pose_id} 警告: 最小阈值 {binary_threshold_min} > 深度最大值 {depth_max}，'
                    f'可能导致无法检测到工件'
                )
            if binary_threshold_max < depth_min:
                self.logger.warning(
                    f'姿态 {pose_id} 警告: 最大阈值 {binary_threshold_max} < 深度最小值 {depth_min}，'
                    f'可能导致无法检测到工件'
                )
        else:
            self.logger.warning(f'姿态 {pose_id} 深度图无效：没有有效深度值')
    
    def _process_pose_for_standardization(
        self,
        pose_id: str,
        depth_image: np.ndarray,
        color_image: np.ndarray,
        binary_threshold_min: int,
        binary_threshold_max: int
        ) -> Tuple[Optional[ComponentFeature], Optional[np.ndarray]]:
        """处理pose进行标准化：预处理和特征提取
        
        Args:
            pose_id: 姿态ID
            depth_image: 深度图
            color_image: 彩色图
            binary_threshold_min: 最小阈值
            binary_threshold_max: 最大阈值
            
        Returns:
            (feature, preprocessed_color) 特征对象和预处理后的彩色图，如果失败则返回None
        """
        # 每个 pose 单独清空一次 RemBG 缓存，防止不同姿态之间复用同一张掩模
        self._clear_rembg_cache()

        # 预处理与特征提取（与在线估计保持一致）
        components, preprocessed_color = self.preprocessor.preprocess(
            depth_image,
            color_image,
            binary_threshold_min,
            binary_threshold_max
        )
        if not components:
            self.logger.warning(
                f'姿态 {pose_id} 跳过：未检测到工件连通域。'
                f'请检查阈值 [{binary_threshold_min}, {binary_threshold_max}] 是否匹配深度图范围'
            )
            return None, None

        features = self.feature_extractor.extract_features(components, preprocessed_color)
        if not features:
            self.logger.warning(f'姿态 {pose_id} 跳过：未提取到有效特征')
            return None, None

        # 选择面积最大的特征（更稳定）
        feature = max(features, key=lambda f: float(getattr(f, 'workpiece_area', 0.0)))

        # 检查模板标准化阶段是否启用 RemBG
        use_rembg = self._should_use_rembg()

        if use_rembg:
            rembg_mask, rembg_cutout = self._get_rembg_outputs(
                0,
                color_image,
                feature=feature,
                component_mask=feature.component_mask
            )
            if rembg_mask is not None:
                feature.component_mask = rembg_mask
            if rembg_cutout is not None:
                feature.color_image = rembg_cutout
                preprocessed_color = rembg_cutout
        
        return feature, preprocessed_color
    
    def _standardize_single_pose(
        self,
        pose_id: str,
        feature: ComponentFeature,
        preprocessed_color: Optional[np.ndarray],
        color_image: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, float]]:
        """标准化单个pose
        
        Args:
            pose_id: 姿态ID
            feature: 特征对象
            preprocessed_color: 预处理后的彩色图
            color_image: 原始彩色图
            
        Returns:
            (standardized_image, standardized_mask, crop_params) 标准化结果
        """
        image_for_standardize = preprocessed_color if preprocessed_color is not None else color_image

        # 标准化模板（裁剪、不旋转）
        standardized_image, standardized_mask, crop_params = self.template_standardizer.standardize_template(
            image_for_standardize,
            feature.component_mask,
            feature.workpiece_center,
            feature.valve_center,
            feature.standardized_angle,
            feature.workpiece_radius
        )
        
        return standardized_image, standardized_mask, crop_params
    
    def _load_pose_data(
        self,
        pose_id: str,
        pose_dir: Path
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], bool]:
        """加载pose的姿态数据
        
        从模板目录中加载三种姿态数据：
        1. 抓取姿态（grab_position.json）- 机器人抓取工件时的末端执行器位姿
        2. 准备姿态（preparation_position.json）- 机器人准备抓取时的位姿
        3. 相机位姿（camera_pose.json）- 拍摄模板图像时相机的位姿
        
        Args:
            pose_id: 姿态ID（如 "1", "2"）
            pose_dir: 姿态目录路径（如 templates/3211242785/pose_1/）
            
        Returns:
            - T_B_E_grasp: 抓取姿态变换矩阵（4x4），从机器人基座到末端执行器的变换grab_position.json
            - T_B_E_preparation: 准备姿态变换矩阵（4x4），从机器人基座到末端执行器的变换preparation_position.json
            - T_B_E_camera: 相机位姿变换矩阵（4x4），从机器人基座到末端执行器的变换camera_pose.json 
        """
        # 读取抓取姿态
        T_B_E_grasp = self._load_pose_from_file(pose_dir, 'grab_position.json')
        # 读取准备姿态
        T_B_E_preparation = self._load_pose_from_file(pose_dir, 'preparation_position.json')
        # 读取相机位姿
        T_B_E_camera = self._load_pose_from_file(pose_dir, 'camera_pose.json')
        
        return T_B_E_grasp, T_B_E_preparation, T_B_E_camera
    
    def _load_pose_from_file(
        self,
        pose_dir: Path,
        filename: str
    ) -> Optional[np.ndarray]:
        """从文件加载姿态
        
        Args:
            pose_dir: 姿态目录
            filename: 文件名
            
        Returns:
            姿态矩阵，如果加载失败则返回None
        """
        pose_path = pose_dir / filename
        if pose_path.exists():
            try:
                with open(pose_path, 'r', encoding='utf-8') as f:
                    pose_data = json.load(f)
                return self._load_pose_json_data(pose_data)
            except Exception as e:
                self.logger.warning(f'加载姿态文件失败: {pose_path}, 错误: {e}')
        return None
    
    def _compute_standardized_poses(
        self,
        pose_id: str,
        feature: ComponentFeature,
        crop_params: Dict[str, float],
        T_B_E_grasp: Optional[np.ndarray],
        T_B_E_preparation: Optional[np.ndarray],
        T_B_E_camera: Optional[np.ndarray]
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """计算标准化姿态
        
        Args:
            pose_id: 姿态ID
            feature: 特征对象
            crop_params: 裁剪参数
            T_B_E_grasp: 原始抓取姿态
            T_B_E_preparation: 原始准备姿态
            T_B_E_camera: 相机姿态
            
        Returns:
            (T_B_E_standardized_grasp, T_B_E_standardized_preparation) 标准化姿态
        """
        T_B_E_standardized_grasp = None
        T_B_E_standardized_preparation = None
        
        if (T_B_E_grasp is not None and T_B_E_camera is not None and 
            self.T_E_C is not None and self.camera_matrix is not None):
            try:
                # 构建特征参数字典
                feature_params = self.template_standardizer._build_feature_params_from_feature(
                    feature, crop_params
                )
                
                T_B_E_standardized_grasp = self.template_standardizer.compute_standardized_pose(
                    T_B_E_grasp, T_B_E_camera, self.T_E_C, self.camera_matrix, feature_params
                )
                if T_B_E_preparation is not None:
                    T_B_E_standardized_preparation = self.template_standardizer.compute_standardized_pose(
                        T_B_E_preparation, T_B_E_camera, self.T_E_C, self.camera_matrix, feature_params
                    )
            except Exception as e:
                self.logger.warning(f'姿态 {pose_id} 标准化姿态计算失败，将跳过姿态/可视化: {e}')
        else:
            # 缺少必要参数
            missing_parts = []
            if T_B_E_grasp is None:
                missing_parts.append('grab_position')
            if T_B_E_camera is None:
                missing_parts.append('camera_pose/preparation/grasp_as_camera')
            if self.T_E_C is None:
                missing_parts.append('T_E_C')
            if self.camera_matrix is None:
                missing_parts.append('camera_matrix')
            self.logger.warning(
                f'姿态 {pose_id} 缺少姿态/标定，跳过姿态计算和夹爪可视化: {", ".join(missing_parts)}'
            )
        # 格式化打印姿态信息
        def _format_pose(T, name):
            if T is None:
                return f'{name}: None'
            pos = T[:3, 3]
            rpy = self._rotation_matrix_to_euler_rpy(T[:3, :3])
            return f'{name}: pos=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})m, rpy=({np.degrees(rpy[0]):.2f}°, {np.degrees(rpy[1]):.2f}°, {np.degrees(rpy[2]):.2f}°)'
        
        self.logger.info(f'姿态 {pose_id} 标准化姿态计算完成:')
        self.logger.info(f'  {_format_pose(T_B_E_grasp, "原始抓取")}')
        self.logger.info(f'  {_format_pose(T_B_E_preparation, "原始准备")}')
        self.logger.info(f'  {_format_pose(T_B_E_camera, "原始相机")}')
        self.logger.info(f'  {_format_pose(T_B_E_standardized_grasp, "标准化抓取")}')
        self.logger.info(f'  {_format_pose(T_B_E_standardized_preparation, "标准化准备")}')
        return T_B_E_standardized_grasp, T_B_E_standardized_preparation
    
    def _generate_gripper_visualization(
        self,
        pose_id: str,
        pose_dir: Path,
        color_image: np.ndarray,
        T_B_E_grasp: Optional[np.ndarray],
        T_B_E_camera: Optional[np.ndarray]
    ) -> None:
        """生成夹爪可视化图像
        
        Args:
            pose_id: 姿态ID
            pose_dir: 姿态目录
            color_image: 彩色图
            T_B_E_grasp: 抓取姿态
            T_B_E_camera: 相机姿态
        """
        if (T_B_E_grasp is not None and T_B_E_camera is not None and 
            self.T_E_C is not None and self.camera_matrix is not None):
            try:
                gripper_vis_image = color_image.copy()
                T_B_C_template = T_B_E_camera @ self.T_E_C
                T_C_template_B = np.linalg.inv(T_B_C_template)
                T_C_template_E_grasp = T_C_template_B @ T_B_E_grasp
                self.template_standardizer.draw_gripper(
                    gripper_vis_image,
                    T_C_template_E_grasp,
                    self.camera_matrix,
                    self.dist_coeffs,
                    gripper_opening_mm=50.0,
                    gripper_length_mm=100.0
                )
                gripper_vis_path = pose_dir / 'gripper_visualization.jpg'
                cv2.imwrite(str(gripper_vis_path), gripper_vis_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            except Exception as e:
                self.logger.warning(f'姿态 {pose_id} 生成夹爪可视化失败: {e}')
