#!/usr/bin/env python3
"""
姿态估计模块

功能：
1. 加载模板库
2. 选择最佳匹配模板
3. 计算2D对齐矩阵
4. 计算最终姿态（3D变换）
5. 暴力匹配（遍历所有模板）
"""

import cv2
import numpy as np
import json
import logging
import traceback
import time
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

from .feature_extractor import ComponentFeature
from .preprocessor import Preprocessor
from .feature_extractor import FeatureExtractor


@dataclass
class TemplateItem:
    """模板项数据结构"""
    id: str                                      # 模板ID（如 "pose_1"）
    feature: Optional[ComponentFeature] = None   # 模板特征
    T_C_E_grasp: np.ndarray = field(default_factory=lambda: np.eye(4))  # 抓取姿态
    T_C_E_prep: np.ndarray = field(default_factory=lambda: np.eye(4))   # 准备姿态
    crop_path: str = ""                          # 模板图像路径
    mask: Optional[np.ndarray] = None            # 模板掩码


@dataclass
class PoseEstimationResult:
    """姿态估计结果"""
    template_id: str = ""
    T_B_E_grasp: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_B_E_prep: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_B_E_preplace: np.ndarray = field(default_factory=lambda: np.eye(4))  # 预放置姿态
    T_B_E_place: np.ndarray = field(default_factory=lambda: np.eye(4))  # 放置姿态
    confidence: float = 0.0
    bounding_box: Tuple[int, int, int, int] = (0, 0, 0, 0)  # (x, y, w, h)
    target_center_camera: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    has_target_center_camera: bool = False
    meta: Dict[str, float] = field(default_factory=dict)
    # 模板 preplace/place 的关节角（加载 JSON 时填充，用于响应中保留模板关节）
    joint_position_deg_preplace: Optional[List[float]] = None
    joint_position_rad_preplace: Optional[List[float]] = None
    joint_position_deg_place: Optional[List[float]] = None
    joint_position_rad_place: Optional[List[float]] = None


class PoseEstimator:
    """姿态估计器类"""
    
    def __init__(
        self,
        preprocessor: Preprocessor,
        feature_extractor: FeatureExtractor
    ):
        """初始化姿态估计器
        
        Args:
            preprocessor: 预处理器
            feature_extractor: 特征提取器
        """
        self.logger = logging.getLogger(__name__)
        self.preprocessor = preprocessor
        self.feature_extractor = feature_extractor
        self.templates = []
        self.calib_file_path = None  # 手眼标定文件路径（用于日志输出）
        
        # 暴力匹配参数（默认值，可通过set_parameters设置）
        self.brute_force_matching_enabled = False
        self.brute_force_angle_step_deg = 1.0  # 减小角度步进，提高匹配精度
        self.brute_force_max_threads = 4
        self.brute_force_rejection_threshold = 0.2  # 调整阈值，降低舍弃阈值
        self.brute_force_acceptance_threshold = 0.7
        self.brute_force_angle_matching_scale = 1.0  # 提高缩放因子，使用原始尺寸匹配
    
    def load_template_library(
        self,
        template_root: str,
        camera_matrix: np.ndarray,
        T_E_C: np.ndarray
    ) -> bool:
        """加载模板库
        
        Args:
            template_root: 模板库根目录
            camera_matrix: 相机内参矩阵
            T_E_C: 末端到相机的变换矩阵
            
        Returns:
            是否加载成功
        """
        try:
            template_root_path = Path(template_root)
            if not template_root_path.exists():
                self.logger.error(f'模板库目录不存在: {template_root}')
                return False
            
            self.templates = []
            
            # 遍历所有模板目录
            for template_dir in template_root_path.iterdir():
                if not template_dir.is_dir():
                    continue
                
                # 加载该模板的所有姿态
                self._load_template_poses(template_dir, camera_matrix, T_E_C)
            
            self.logger.info(f'成功加载 {len(self.templates)} 个模板')
            return len(self.templates) > 0
            
        except Exception as e:
            self.logger.error(f'加载模板库失败: {e}')
            return False
    
    def _load_template_poses(
        self,
        pose_dir: Path,
        camera_matrix: np.ndarray,
        T_E_C: np.ndarray
    ):
        """加载单个姿态模板
        
        Args:
            pose_dir: 姿态目录（如pose_1）
            camera_matrix: 相机内参矩阵
            T_E_C: 末端到相机的变换矩阵
        """
        try:
            # 直接加载template_info.json
            template_info_json = pose_dir / "template_info.json"
            if not template_info_json.exists():
                return
            
            template_info = None
            try:
                with open(template_info_json, 'r', encoding='utf-8') as f:
                    template_info = json.load(f)
            except Exception as e:
                self.logger.warning(f'加载模板文件失败 {template_info_json}: {e}')
                return
            
            # 从template_info.json中提取特征参数
            if not template_info or 'feature_parameters' not in template_info:
                self.logger.warning(f'模板 {pose_dir.name}: template_info.json中缺少feature_parameters')
                return
            
            feature_params = template_info['feature_parameters']
            
            # 创建特征（直接从template_info.json读取）
            feature = ComponentFeature(
                workpiece_center=(
                    float(feature_params.get('workpiece_center_x_original', 0.0)),
                    float(feature_params.get('workpiece_center_y_original', 0.0))
                ),
                workpiece_radius=float(feature_params.get('workpiece_radius_original', 0.0)),
                workpiece_area=float(feature_params.get('workpiece_area', 0.0)),
                valve_center=(
                    float(feature_params.get('valve_center_x', 0.0)),
                    float(feature_params.get('valve_center_y', 0.0))
                ),
                valve_radius=float(feature_params.get('valve_radius', 0.0)),
                valve_area=float(feature_params.get('valve_area', 0.0)),
                standardized_angle=float(feature_params.get('standardized_angle_rad', 0.0)),
                standardized_angle_deg=float(feature_params.get('standardized_angle_deg', 0.0))
            )
            
            # 加载掩码（优先使用mask.jpg，如果没有则尝试standardized_mask.jpg）
            mask_path = pose_dir / "mask.jpg"
            mask = None
            if mask_path.exists():
                mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            else:
                # 尝试使用standardized_mask.jpg
                mask_path_alt = pose_dir / "standardized_mask.jpg"
                if mask_path_alt.exists():
                    mask = cv2.imread(str(mask_path_alt), cv2.IMREAD_GRAYSCALE)
            
            # 创建模板项（使用image.jpg）
            image_path = pose_dir / "image.jpg"
            
            template_item = TemplateItem(
                id=pose_dir.name,
                feature=feature,
                crop_path=str(image_path),
                mask=mask
            )
            # 加载相机位姿（用于坐标转换）
            camera_pose_json = pose_dir / "camera_pose.json"
            T_B_E_camera = None
            if camera_pose_json.exists():
                T_B_E_camera = self._load_pose_json(str(camera_pose_json))
            
            # 如果没有camera_pose.json，尝试使用preparation_position.json
            if T_B_E_camera is None:
                prep_pose_json = pose_dir / "preparation_position.json"
                if prep_pose_json.exists():
                    T_B_E_camera = self._load_pose_json(str(prep_pose_json))
            
            # 加载抓取姿态（如果存在）
            # 优先从template_info.json加载标准化姿态，然后尝试grab_position.json，最后是pose.json
            T_B_E_grasp = None
            
            if template_info is not None and 'standardized_grasp_pose' in template_info:
                # 优先使用standardized_grasp_pose（标准化后的姿态）
                grasp_pose = template_info['standardized_grasp_pose'].get('cartesian_position', {})
                pos = grasp_pose.get('position', {})
                ori = grasp_pose.get('orientation', {})
                
                T_B_E_grasp = np.eye(4)
                T_B_E_grasp[0, 3] = pos.get('x', 0.0)
                T_B_E_grasp[1, 3] = pos.get('y', 0.0)
                T_B_E_grasp[2, 3] = pos.get('z', 0.0)
                
                # 四元数转旋转矩阵
                qx, qy, qz, qw = ori.get('x', 0.0), ori.get('y', 0.0), ori.get('z', 0.0), ori.get('w', 1.0)
                R = self._quaternion_to_rotation_matrix([qx, qy, qz, qw])
                T_B_E_grasp[:3, :3] = R
            
            # 如果template_info.json中没有，尝试从grab_position.json或pose.json加载
            if T_B_E_grasp is None:
                grab_pos_json = pose_dir / "grab_position.json"
                pose_json = pose_dir / "pose.json"
                
                if grab_pos_json.exists():
                    T_B_E_grasp = self._load_pose_json(str(grab_pos_json))
                elif pose_json.exists():
                    T_B_E_grasp = self._load_pose_json(str(pose_json))
            
            # 将T_B_E转换为T_C_E（相机坐标系到末端执行器）
            if T_B_E_grasp is not None:
                if T_B_E_camera is not None and T_E_C is not None and not np.allclose(T_E_C, np.eye(4)):
                    # T_B_C_template = T_B_E_camera * T_E_C
                    T_B_C_template = T_B_E_camera @ T_E_C
                    # T_C_template_B = inv(T_B_C_template)
                    T_C_template_B = np.linalg.inv(T_B_C_template)
                    # T_C_E_grasp = T_C_template_B * T_B_E_grasp
                    template_item.T_C_E_grasp = T_C_template_B @ T_B_E_grasp
                else:
                    # 如果没有相机位姿，直接使用T_B_E（但这不是正确的，会警告）
                    template_item.T_C_E_grasp = T_B_E_grasp
                    if T_B_E_camera is None:
                        self.logger.warning(f'模板 {pose_dir.name}: 未找到camera_pose.json，无法正确转换姿态')
                    if T_E_C is None or np.allclose(T_E_C, np.eye(4)):
                        self.logger.warning(f'模板 {pose_dir.name}: T_E_C未设置或为单位矩阵，无法正确转换姿态')
            else:
                self.logger.warning(f'模板 {pose_dir.name}: 未找到grab_position.json或pose.json')
            
            # 加载准备姿态（如果存在）
            # 优先从template_info.json加载标准化准备姿态，然后尝试preparation_position.json
            T_B_E_prep = None
            
            if template_info is not None and 'standardized_preparation_pose' in template_info:
                # 优先使用standardized_preparation_pose（标准化后的准备姿态）
                prep_pose = template_info['standardized_preparation_pose'].get('cartesian_position', {})
                pos = prep_pose.get('position', {})
                ori = prep_pose.get('orientation', {})
                
                T_B_E_prep = np.eye(4)
                T_B_E_prep[0, 3] = pos.get('x', 0.0)
                T_B_E_prep[1, 3] = pos.get('y', 0.0)
                T_B_E_prep[2, 3] = pos.get('z', 0.0)
                
                # 四元数转旋转矩阵
                qx, qy, qz, qw = ori.get('x', 0.0), ori.get('y', 0.0), ori.get('z', 0.0), ori.get('w', 1.0)
                R = self._quaternion_to_rotation_matrix([qx, qy, qz, qw])
                T_B_E_prep[:3, :3] = R
            
            # 如果template_info.json中没有，尝试从preparation_position.json加载
            if T_B_E_prep is None:
                prep_pos_json = pose_dir / "preparation_position.json"
                if prep_pos_json.exists():
                    T_B_E_prep = self._load_pose_json(str(prep_pos_json))
            
            if T_B_E_prep is not None:
                if T_B_E_camera is not None and T_E_C is not None and not np.allclose(T_E_C, np.eye(4)):
                    T_B_C_template = T_B_E_camera @ T_E_C
                    T_C_template_B = np.linalg.inv(T_B_C_template)
                    template_item.T_C_E_prep = T_C_template_B @ T_B_E_prep
                else:
                    template_item.T_C_E_prep = T_B_E_prep
            
            self.templates.append(template_item)
                
        except Exception as e:
            self.logger.warning(f'加载模板姿态失败 {pose_dir}: {e}')
    
    def _load_pose_json(self, pose_path: str) -> Optional[np.ndarray]:
        """加载姿态JSON文件
        
        支持两种格式：
        1. 直接格式: {"position": {"x": ..., "y": ..., "z": ...}, 
                      "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}}
        2. 嵌套格式: {"cartesian_position": {"position": {...}, "orientation": {...}}}
        
        Args:
            pose_path: JSON文件路径
            
        Returns:
            4x4变换矩阵 或 None
        """
        try:
            with open(pose_path, 'r', encoding='utf-8') as f:
                pose_data = json.load(f)
            
            # 尝试从嵌套格式提取（grab_position.json格式）
            if 'cartesian_position' in pose_data:
                cartesian = pose_data['cartesian_position']
                position = cartesian.get('position', {})
                orientation = cartesian.get('orientation', {})
            else:
                # 直接格式
                position = pose_data.get('position', {})
                orientation = pose_data.get('orientation', {})
            
            # 构建变换矩阵
            T = np.eye(4)
            T[0, 3] = position.get('x', 0.0)
            T[1, 3] = position.get('y', 0.0)
            T[2, 3] = position.get('z', 0.0)
            
            # 四元数转旋转矩阵
            q = [
                orientation.get('x', 0.0),
                orientation.get('y', 0.0),
                orientation.get('z', 0.0),
                orientation.get('w', 1.0)
            ]
            R = self._quaternion_to_rotation_matrix(q)
            T[:3, :3] = R
            
            return T
            
        except Exception as e:
            self.logger.warning(f'加载姿态JSON失败 {pose_path}: {e}')
            return None
    
    def _quaternion_to_rotation_matrix(self, q: List[float]) -> np.ndarray:
        """四元数转旋转矩阵
        
        Args:
            q: 四元数 [x, y, z, w]
            
        Returns:
            3x3旋转矩阵
        """
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数
        
        Args:
            R: 3x3旋转矩阵
            
        Returns:
            四元数 [x, y, z, w]
        """
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
        """旋转矩阵转欧拉角（RPY，ZYX顺序）
        
        Args:
            R: 3x3旋转矩阵
            
        Returns:
            欧拉角 [roll, pitch, yaw]（弧度）
        """
        # Roll (绕X轴旋转)
        roll = np.arctan2(R[2, 1], R[2, 2])
        
        # Pitch (绕Y轴旋转)
        pitch = -np.arcsin(R[2, 0])
        
        # Yaw (绕Z轴旋转)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        
        return np.array([roll, pitch, yaw])
    
    def set_parameters(self, params: Dict):
        """设置参数
        
        Args:
            params: 参数字典
        """
        self.brute_force_matching_enabled = params.get('brute_force_matching_enabled', False)
        self.brute_force_angle_step_deg = params.get('brute_force_angle_step_deg', 1.0)  # 默认1.0度
        self.brute_force_max_threads = params.get('brute_force_max_threads', 4)
        self.brute_force_rejection_threshold = params.get('brute_force_rejection_threshold', 0.2)  # 默认0.2
        self.brute_force_acceptance_threshold = params.get('brute_force_acceptance_threshold', 0.7)
        self.brute_force_angle_matching_scale = params.get('brute_force_angle_matching_scale', 1.0)  # 默认1.0（原始尺寸）
    
    def calculate_mask_iou(self, mask1: np.ndarray, mask2: np.ndarray) -> float:
        """计算两个掩膜的交并比（IoU - Intersection over Union）
        
        Args:
            mask1: 第一个掩膜（二值图像，0或255）
            mask2: 第二个掩膜（二值图像，0或255）
            
        Returns:
            重合度值（0.0-1.0），1.0表示完全重合
        """
        if mask1.size == 0 or mask2.size == 0 or mask1.shape != mask2.shape:
            return 0.0
        
        # 确保掩膜是二值图像
        m1 = mask1.copy()
        m2 = mask2.copy()
        
        if len(mask1.shape) == 3:
            m1 = cv2.cvtColor(m1, cv2.COLOR_BGR2GRAY)
        if len(mask2.shape) == 3:
            m2 = cv2.cvtColor(m2, cv2.COLOR_BGR2GRAY)
        
        # 二值化
        _, m1 = cv2.threshold(m1, 127, 255, cv2.THRESH_BINARY)
        _, m2 = cv2.threshold(m2, 127, 255, cv2.THRESH_BINARY)
        
        # 转换为布尔掩膜（0或1）
        m1 = (m1 > 0).astype(np.uint8)
        m2 = (m2 > 0).astype(np.uint8)
        
        # 计算交集和并集
        intersection = cv2.bitwise_and(m1, m2)
        union_mask = cv2.bitwise_or(m1, m2)
        
        intersection_area = np.count_nonzero(intersection)
        union_area = np.count_nonzero(union_mask)
        
        if union_area == 0:
            return 0.0
        
        return float(intersection_area) / float(union_area)
    
    def find_best_mask_alignment(
        self,
        template_mask: np.ndarray,
        target_mask: np.ndarray,
        target_center: Tuple[float, float],
        scale: float = 1.0,
        initial_angle_deg: float = 0.0,
        angle_range_deg: float = 180.0,
        angle_step_deg: float = 5.0
    ) -> Tuple[bool, float, float, np.ndarray]:
        """通过旋转搜索找到最佳掩膜对齐角度和重合度
        
        Args:
            template_mask: 模板掩膜（标准化后的掩膜，工件中心在掩膜中心）
            target_mask: 待估计工件的掩膜（输入图像坐标系）
            target_center: 待估计工件中心在输入图像中的位置
            scale: 缩放比例（基于外接圆半径）
            initial_angle_deg: 初始角度（度），用于确定搜索范围
            angle_range_deg: 搜索角度范围（度），在初始角度两侧各搜索此范围
            angle_step_deg: 角度搜索步长（度）
            
        Returns:
            (是否成功找到匹配, 最佳角度(度), 最佳置信度, 最佳对齐后的掩膜)
        """
        if template_mask.size == 0 or target_mask.size == 0:
            return False, 0.0, 0.0, np.array([])
        
        # 模板掩膜中心（标准化后的掩膜，工件中心在掩膜中心）
        template_mask_center = (
            template_mask.shape[1] / 2.0,
            template_mask.shape[0] / 2.0
        )
        
        # 计算搜索角度范围（与C++一致：±angle_range_deg，总范围 2*angle_range_deg）
        min_angle = initial_angle_deg - angle_range_deg
        max_angle = initial_angle_deg + angle_range_deg
        
        best_confidence = 0.0
        best_angle_deg = initial_angle_deg
        best_aligned_mask = None
        
        # 遍历所有角度
        angle = min_angle
        while angle <= max_angle:
            # 创建旋转变换矩阵（以模板掩膜中心为旋转中心）
            M = cv2.getRotationMatrix2D(template_mask_center, angle, scale)
            
            # 计算平移量：目标中心 - 变换后的掩膜中心
            # 由于以掩膜中心为旋转中心，旋转和缩放后掩膜中心位置不变
            M[0, 2] += target_center[0] - template_mask_center[0]
            M[1, 2] += target_center[1] - template_mask_center[1]
            
            # 变换模板掩膜到目标图像坐标系
            aligned_mask = cv2.warpAffine(
                template_mask, M, (target_mask.shape[1], target_mask.shape[0]),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0
            )
            
            # 重新二值化，确保掩膜是纯二值的（0或255）
            _, aligned_mask = cv2.threshold(aligned_mask, 127, 255, cv2.THRESH_BINARY)
            
            # 计算重合度（IoU）
            confidence = self.calculate_mask_iou(aligned_mask, target_mask)
            
            # 更新最佳匹配
            if confidence > best_confidence:
                best_confidence = confidence
                best_angle_deg = angle
                best_aligned_mask = aligned_mask.copy()
            
            angle += angle_step_deg
        
        return best_confidence > 0.0, best_angle_deg, best_confidence, best_aligned_mask if best_aligned_mask is not None else np.array([])
    
    def select_best_template(
        self,
        target_feature: ComponentFeature,
        templates: Optional[List[TemplateItem]] = None,
        target_mask: Optional[np.ndarray] = None,
        workpiece_template_dir: Optional[str] = None
    ) -> Tuple[int, float, Optional[float], Optional[float], Optional[np.ndarray]]:
        """选择最佳匹配模板
        
        Args:
            target_feature: 目标特征
            templates: 模板列表（如果为None，使用已加载的模板）
            target_mask: 目标掩膜（用于暴力匹配）
            workpiece_template_dir: 工件模板目录（用于加载模板掩膜）
            
        Returns:
            (最佳模板索引, 距离, 置信度, 最佳角度(度), 最佳对齐掩膜)
        """
        if templates is None:
            templates = self.templates
        
        if not templates:
            self.logger.warning('模板列表为空，无法选择最佳模板')
            return (-1, float('inf'), None, None, None)
        
        # 如果启用暴力匹配且有目标掩膜，使用暴力匹配模式
        if self.brute_force_matching_enabled and target_mask is not None and workpiece_template_dir:
            return self._brute_force_template_matching(
                target_feature, templates, target_mask, workpiece_template_dir
            )
        else:
            # 原有匹配模式：使用特征距离
            best_idx = -1
            best_distance = float('inf')
            
            for idx, template in enumerate(templates):
                if template.feature is None:
                    continue
                
                # 计算特征距离（简单欧氏距离）
                distance = self._calculate_feature_distance(target_feature, template.feature)
                
                if distance < best_distance:
                    best_distance = distance
                    best_idx = idx
            
            if best_idx >= 0:
                self.logger.info(f'特征距离匹配: 最佳模板索引={best_idx}, 距离={best_distance:.4f}')
            else:
                self.logger.warning('特征距离匹配未找到有效模板')
            
            return (best_idx, best_distance, None, None, None)
    
    def _brute_force_template_matching(
        self,
        target_feature: ComponentFeature,
        templates: List[TemplateItem],
        target_mask: np.ndarray,
        workpiece_template_dir: str
    ) -> Tuple[int, float, Optional[float], Optional[float], Optional[np.ndarray]]:
        """暴力模板匹配：通过旋转角度搜索最佳匹配（多线程）
        
        Args:
            target_feature: 目标特征
            templates: 模板列表
            target_mask: 待匹配工件掩膜
            workpiece_template_dir: 工件模板目录
            
        Returns:
            (最佳模板索引, 距离, 置信度, 最佳角度(度), 最佳对齐掩膜)
        """
        # 输入验证
        if not templates:
            self.logger.warning('暴力匹配：模板列表为空')
            return (-1, float('inf'), None, None, None)
        if target_mask.size == 0:
            self.logger.warning('暴力匹配：目标掩膜为空')
            return (-1, float('inf'), None, None, None)
        
        # 目标工件中心在输入图像中的位置
        target_center = (
            float(target_feature.workpiece_center[0]),
            float(target_feature.workpiece_center[1])
        )
        
        # 目标工件的标准化角度（弧度转度，与C++版本一致）
        target_standardized_angle_rad = target_feature.standardized_angle
        initial_angle_deg = np.degrees(target_standardized_angle_rad)
        
        # 搜索范围：±180度（总共360度范围，与C++一致）
        search_range_deg = 180.0
        angle_step_deg = float(self.brute_force_angle_step_deg)
        
        # 计算总搜索角度数（用于预估计算量）
        total_angles_per_template = int((2 * search_range_deg) / angle_step_deg) + 1
        
        # 核心计算变量
        self.logger.info(f'暴力匹配: 模板数={len(templates)}, 目标中心=({target_center[0]:.1f}, {target_center[1]:.1f}), 目标角度={initial_angle_deg:.2f}°, 搜索范围=±{search_range_deg:.1f}°, 角度步进={angle_step_deg:.2f}°')
        
        # ========== 在主线程预加载所有模板掩膜（与C++版本一致） ==========
        template_mask_data = []  # 存储(模板索引, 模板, 掩膜, 掩膜中心)
        
        for template_idx, template in enumerate(templates):
            template_dir = Path(workpiece_template_dir) / template.id
            # 优先尝试 mask.jpg（与C++版本一致），然后尝试 standardized_mask.jpg
            template_mask_path = None
            for mask_name in ["mask.jpg", "standardized_mask.jpg"]:
                candidate_path = template_dir / mask_name
                if candidate_path.exists():
                    template_mask_path = candidate_path
                    break
            
            if template_mask_path is None:
                self.logger.warning(f'[模板 {template_idx}] {template.id}: 未找到掩膜文件')
                continue
            
            template_mask = cv2.imread(str(template_mask_path), cv2.IMREAD_GRAYSCALE)
            if template_mask is None or template_mask.size == 0:
                self.logger.warning(f'[模板 {template_idx}] {template.id}: 掩膜加载失败')
                continue
            
            template_mask_center = (template_mask.shape[1] / 2.0, template_mask.shape[0] / 2.0)
            template_mask_data.append((template_idx, template, template_mask, template_mask_center))
        
        if not template_mask_data:
            self.logger.error('未找到任何有效的模板掩膜，匹配失败')
            return (-1, float('inf'), None, None, None)
        
        # 全局最佳结果（使用字典以便在线程间共享）
        global_best_result = {
            'template_idx': -1,
            'confidence': 0.0,
            'angle': 0.0,
            'mask': None
        }
        global_result_lock = threading.Lock()
        
        # 为每个模板维护最佳结果（用于调试输出，与C++版本一致）
        template_results = [{'best_confidence': 0.0, 'best_angle_deg': 0.0} for _ in range(len(templates))]
        template_results_lock = threading.Lock()
        
        # 提前终止标志
        should_early_exit = threading.Event()
        
        # 使用线程池并行处理模板（与C++版本一致：每个模板一个任务）
        num_threads = min(self.brute_force_max_threads, len(template_mask_data))
        
        start_time = time.time()
        completed_tasks = 0
        failed_tasks = 0
        with ThreadPoolExecutor(max_workers=num_threads) as executor:
            futures = [
                executor.submit(
                    self._process_template_for_matching,
                    template_idx=tmpl_idx,
                    template=tmpl,
                    template_mask=tmpl_mask,
                    template_mask_center=tmpl_mask_center,
                    target_feature=target_feature,
                    target_mask=target_mask,
                    target_center=target_center,
                    initial_angle_deg=initial_angle_deg,
                    search_range_deg=search_range_deg,
                    angle_step_deg=angle_step_deg,
                    global_best_result=global_best_result,
                    global_result_lock=global_result_lock,
                    template_results=template_results,
                    template_results_lock=template_results_lock,
                    should_early_exit=should_early_exit
                )
                for tmpl_idx, tmpl, tmpl_mask, tmpl_mask_center in template_mask_data
            ]
            
            # 等待所有任务完成
            for future in as_completed(futures):
                try:
                    future.result()
                    completed_tasks += 1
                except Exception as e:
                    failed_tasks += 1
                    self.logger.error(f'模板匹配任务异常: {e}', exc_info=True)
        
        elapsed_time = time.time() - start_time
        
        # 从共享字典中提取全局最佳结果
        global_best_template_idx = global_best_result['template_idx']
        global_best_confidence = global_best_result['confidence']
        global_best_angle_deg = global_best_result['angle']
        global_best_aligned_mask = global_best_result['mask']
        
        # 输出全局最佳结果
        if global_best_template_idx >= 0 and global_best_template_idx < len(templates):
            best_template = templates[global_best_template_idx]
            self.logger.info(f'最佳匹配: 模板ID={best_template.id}, 置信度={global_best_confidence:.4f}, 角度={global_best_angle_deg:.2f}°, 耗时={elapsed_time:.3f}秒')
        else:
            self.logger.warning('未找到任何匹配的模板')
        
        # 检查是否找到有效匹配（与C++版本一致）
        if global_best_template_idx < 0 or global_best_template_idx >= len(templates):
            self.logger.error('匹配失败: 未找到任何有效的模板索引')
            return (-1, float('inf'), None, None, None)
        
        # 检查舍弃阈值：如果最佳置信度低于舍弃阈值，则认为匹配失败（与C++版本一致）
        if global_best_confidence < self.brute_force_rejection_threshold:
            self.logger.warning(
                f'匹配失败: 最佳匹配置信度 {global_best_confidence:.4f} 低于舍弃阈值 {self.brute_force_rejection_threshold:.4f}'
            )
            self.logger.info(f'  建议: 检查模板质量、调整阈值或增加模板数量')
            return (-1, float('inf'), global_best_confidence, global_best_angle_deg, global_best_aligned_mask)
        
        # 转换为距离（置信度越高，距离越小）
        best_distance = 1.0 - global_best_confidence
        
        return (
            global_best_template_idx,
            best_distance,
            global_best_confidence,
            global_best_angle_deg,
            global_best_aligned_mask
        )
    
    def _process_template_for_matching(
        self,
        template_idx: int,
        template: TemplateItem,
        template_mask: np.ndarray,
        template_mask_center: Tuple[float, float],
        target_feature: ComponentFeature,
        target_mask: np.ndarray,
        target_center: Tuple[float, float],
        initial_angle_deg: float,
        search_range_deg: float,
        angle_step_deg: float,
        global_best_result: Dict,
        global_result_lock: threading.Lock,
        template_results: List[Dict],
        template_results_lock: threading.Lock,
        should_early_exit: threading.Event
    ) -> None:
        """处理单个模板的匹配（私有方法，用于多线程并行处理）
        
        Args:
            template_idx: 模板索引
            template: 模板项
            template_mask: 预加载的模板掩膜
            template_mask_center: 模板掩膜中心
            target_feature: 目标特征
            target_mask: 目标掩膜
            target_center: 目标中心
            initial_angle_deg: 初始角度（度）
            search_range_deg: 搜索范围（度）
            angle_step_deg: 角度步进（度）
            global_best_result: 全局最佳结果字典（线程安全共享）
            global_result_lock: 全局结果锁
            template_results: 模板结果列表
            template_results_lock: 模板结果锁
            should_early_exit: 提前退出标志
        """
        if should_early_exit.is_set():
            return None
        
        template_id = template.id
        
        # 计算相对初始角度（与C++版本一致）
        template_standardized_angle_rad = float(template.feature.standardized_angle) if template.feature else 0.0
        template_standardized_angle_deg = np.degrees(template_standardized_angle_rad)
        relative_angle_deg = initial_angle_deg - template_standardized_angle_deg
        
        # Step 1: 对齐中心（将模板掩膜变换到目标掩膜的坐标系）
        translate_M = np.array(
            [[1.0, 0.0, target_center[0] - template_mask_center[0]],
             [0.0, 1.0, target_center[1] - template_mask_center[1]]],
            dtype=np.float32,
        )
        template_mask_aligned = cv2.warpAffine(
            template_mask,
            translate_M,
            (target_mask.shape[1], target_mask.shape[0]),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        _, template_mask_aligned = cv2.threshold(template_mask_aligned, 127, 255, cv2.THRESH_BINARY)
        
        # Step 2: 裁剪（以目标中心为中心，取模板和目标半径的最大值）
        template_radius = float(template.feature.workpiece_radius) if template.feature else 0.0
        target_radius = float(target_feature.workpiece_radius)
        crop_radius = max(template_radius, target_radius)
        
        if crop_radius <= 1.0:
            self.logger.warning(f'[模板 {template_idx}] {template_id}: 裁剪半径无效 ({crop_radius:.1f})')
            return None
        
        crop_size = int(np.ceil(crop_radius * 2.0))
        crop_x = int(max(0.0, target_center[0] - crop_radius))
        crop_y = int(max(0.0, target_center[1] - crop_radius))
        crop_w = int(min(crop_size, target_mask.shape[1] - crop_x))
        crop_h = int(min(crop_size, target_mask.shape[0] - crop_y))
        
        if crop_w <= 1 or crop_h <= 1:
            self.logger.warning(f'[模板 {template_idx}] {template_id}: 裁剪尺寸无效 ({crop_w}x{crop_h})')
            return None
        if crop_x < 0 or crop_y < 0 or crop_x + crop_w > target_mask.shape[1] or crop_y + crop_h > target_mask.shape[0]:
            self.logger.warning(f'[模板 {template_idx}] {template_id}: 裁剪区域越界')
            return None
        
        target_mask_cropped = target_mask[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w].copy()
        template_mask_cropped = template_mask_aligned[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w].copy()
        
        if target_mask_cropped.size == 0 or template_mask_cropped.size == 0:
            self.logger.warning(f'[模板 {template_idx}] {template_id}: 裁剪后掩膜为空')
            return None
        
        target_center_in_crop = (target_center[0] - crop_x, target_center[1] - crop_y)
        
        # Step 3: 缩放（用于加速匹配，与C++版本一致）
        scale_factor = float(self.brute_force_angle_matching_scale)
        
        if abs(scale_factor - 1.0) < 1e-6:
            template_mask_scaled = template_mask_cropped
            target_mask_scaled = target_mask_cropped
            scaled_center = target_center_in_crop
        else:
            scaled_w = max(1, int(round(target_mask_cropped.shape[1] * scale_factor)))
            scaled_h = max(1, int(round(target_mask_cropped.shape[0] * scale_factor)))
            template_mask_scaled = cv2.resize(template_mask_cropped, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST)
            target_mask_scaled = cv2.resize(target_mask_cropped, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST)
            _, template_mask_scaled = cv2.threshold(template_mask_scaled, 127, 255, cv2.THRESH_BINARY)
            _, target_mask_scaled = cv2.threshold(target_mask_scaled, 127, 255, cv2.THRESH_BINARY)
            scaled_center = (target_center_in_crop[0] * scale_factor, target_center_in_crop[1] * scale_factor)
        
        if template_mask_scaled.size == 0 or target_mask_scaled.size == 0:
            self.logger.warning(f'[模板 {template_idx}] {template_id}: 缩放后掩膜为空')
            return None
        
        scaled_center = (
            float(np.clip(scaled_center[0], 0.0, template_mask_scaled.shape[1] - 1)),
            float(np.clip(scaled_center[1], 0.0, template_mask_scaled.shape[0] - 1)),
        )
        
        # Step 4: 角度搜索
        template_min_angle = relative_angle_deg - search_range_deg
        template_max_angle = relative_angle_deg + search_range_deg
        num_angles = int((template_max_angle - template_min_angle) / angle_step_deg) + 1
        
        local_best_confidence = 0.0
        local_best_angle_deg = 0.0
        angle_count = 0
        
        angle = template_min_angle
        while angle <= template_max_angle:
            if should_early_exit.is_set():
                break
            
            normalized_angle = angle
            while normalized_angle < 0.0:
                normalized_angle += 360.0
            while normalized_angle >= 360.0:
                normalized_angle -= 360.0
            
            M = cv2.getRotationMatrix2D(scaled_center, normalized_angle, 1.0)
            rotated_template = cv2.warpAffine(
                template_mask_scaled,
                M,
                (target_mask_scaled.shape[1], target_mask_scaled.shape[0]),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0,
            )
            _, rotated_template = cv2.threshold(rotated_template, 127, 255, cv2.THRESH_BINARY)
            
            confidence = self.calculate_mask_iou(rotated_template, target_mask_scaled)
            angle_count += 1
            
            
            if confidence > local_best_confidence:
                local_best_confidence = confidence
                local_best_angle_deg = angle
            
            # 提前通过阈值
            if confidence >= self.brute_force_acceptance_threshold:
                self.logger.info(f'[模板 {template_idx}] {template_id}: 达到接受阈值, 角度={angle:.1f}°, 置信度={confidence:.4f}')
                normalized_best_angle = angle
                while normalized_best_angle < 0.0:
                    normalized_best_angle += 360.0
                while normalized_best_angle >= 360.0:
                    normalized_best_angle -= 360.0
                
                transform_full = cv2.getRotationMatrix2D(template_mask_center, normalized_best_angle, 1.0)
                transform_full[0, 2] += target_center[0] - template_mask_center[0]
                transform_full[1, 2] += target_center[1] - template_mask_center[1]
                aligned_mask_full = cv2.warpAffine(
                    template_mask,
                    transform_full,
                    (target_mask.shape[1], target_mask.shape[0]),
                    flags=cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=0,
                )
                _, aligned_mask_full = cv2.threshold(aligned_mask_full, 127, 255, cv2.THRESH_BINARY)
                
                # 更新模板局部结果（修复：达到接受阈值时也要更新template_results）
                with template_results_lock:
                    if confidence > template_results[template_idx]['best_confidence']:
                        template_results[template_idx]['best_confidence'] = confidence
                        template_results[template_idx]['best_angle_deg'] = angle
                
                # 更新全局最佳结果
                with global_result_lock:
                    if confidence > global_best_result['confidence']:
                        global_best_result['template_idx'] = template_idx
                        global_best_result['confidence'] = confidence
                        global_best_result['angle'] = angle
                        global_best_result['mask'] = aligned_mask_full.copy()
                should_early_exit.set()
                return
            
            angle += angle_step_deg
        
        # 只输出有有效匹配的结果
        if local_best_confidence > 0.0:
            self.logger.info(f'[模板 {template_idx}] {template_id}: 最佳角度={local_best_angle_deg:.2f}°, 置信度={local_best_confidence:.4f}')
        
        # 更新模板局部最佳结果和全局最佳结果
        if local_best_confidence > 0.0:
            normalized_best_angle = local_best_angle_deg
            while normalized_best_angle < 0.0:
                normalized_best_angle += 360.0
            while normalized_best_angle >= 360.0:
                normalized_best_angle -= 360.0
            
            transform_full = cv2.getRotationMatrix2D(template_mask_center, normalized_best_angle, 1.0)
            transform_full[0, 2] += target_center[0] - template_mask_center[0]
            transform_full[1, 2] += target_center[1] - template_mask_center[1]
            aligned_mask_full = cv2.warpAffine(
                template_mask,
                transform_full,
                (target_mask.shape[1], target_mask.shape[0]),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0,
            )
            _, aligned_mask_full = cv2.threshold(aligned_mask_full, 127, 255, cv2.THRESH_BINARY)
            
            with template_results_lock:
                if local_best_confidence > template_results[template_idx]['best_confidence']:
                    template_results[template_idx]['best_confidence'] = local_best_confidence
                    template_results[template_idx]['best_angle_deg'] = local_best_angle_deg
            
            with global_result_lock:
                if local_best_confidence > global_best_result['confidence']:
                    old_best = global_best_result['confidence']
                    global_best_result['template_idx'] = template_idx
                    global_best_result['confidence'] = local_best_confidence
                    global_best_result['angle'] = local_best_angle_deg
                    global_best_result['mask'] = aligned_mask_full.copy()
    
    def _calculate_feature_distance(
        self,
        feature1: ComponentFeature,
        feature2: ComponentFeature
    ) -> float:
        """计算特征距离
        
        Args:
            feature1: 特征1
            feature2: 特征2
            
        Returns:
            距离值
        """
        # 归一化特征并计算欧氏距离
        # 这里使用简单的半径比和角度差
        
        # 半径比差
        radius_ratio_1 = feature1.valve_radius / feature1.workpiece_radius if feature1.workpiece_radius > 0 else 0
        radius_ratio_2 = feature2.valve_radius / feature2.workpiece_radius if feature2.workpiece_radius > 0 else 0
        radius_diff = abs(radius_ratio_1 - radius_ratio_2)
        
        # 角度差（归一化到0-180度）
        angle_diff = abs(feature1.standardized_angle_deg - feature2.standardized_angle_deg)
        angle_diff = min(angle_diff, 360 - angle_diff)
        angle_diff_norm = angle_diff / 180.0
        
        # 综合距离
        distance = np.sqrt(radius_diff**2 + angle_diff_norm**2)
        
        return distance
    
    def compute_2d_alignment(
        self,
        template_feature: ComponentFeature,
        target_feature: ComponentFeature,
        allow_scale: bool = False
    ) -> np.ndarray:
        """计算2D对齐矩阵
        
        Args:
            template_feature: 模板特征
            target_feature: 目标特征
            allow_scale: 是否允许尺度变化
            
        Returns:
            2x3相似变换矩阵
        """
        # 源点（模板）和目标点
        src_pts = np.array([
            template_feature.workpiece_center,
            template_feature.valve_center
        ], dtype=np.float32)
        
        dst_pts = np.array([
            target_feature.workpiece_center,
            target_feature.valve_center
        ], dtype=np.float32)
        
        # 计算相似变换矩阵
        if allow_scale:
            # 使用estimateAffinePartial2D计算相似变换（刚性+缩放）
            M, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
        else:
            # 只允许刚性变换（旋转+平移）
            # 计算旋转角度
            template_angle = template_feature.standardized_angle
            target_angle = target_feature.standardized_angle
            rotation_angle = np.degrees(target_angle - template_angle)
            
            # 计算变换矩阵
            center = template_feature.workpiece_center
            M_rot = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)
            
            # 计算平移
            rotated_center = M_rot @ np.array([center[0], center[1], 1.0])
            translation = np.array(target_feature.workpiece_center) - rotated_center[:2]
            
            M = M_rot.copy()
            M[0, 2] += translation[0]
            M[1, 2] += translation[1]
        
        return M if M is not None else np.eye(2, 3)
    
    def _get_depth_from_image(
        self,
        depth_image: np.ndarray,
        pixel_u: int,
        pixel_v: int,
        search_radius: int = 3,
        depth_scale: float = 0.00025
    ) -> Optional[float]:
        """从深度图获取指定像素位置的深度值（单位：米）
        
        Args:
            depth_image: 深度图像（uint16格式）
            pixel_u: 像素U坐标（列）
            pixel_v: 像素V坐标（行）
            search_radius: 搜索半径（像素）
            depth_scale: 深度缩放因子（默认0.00025，即depth_raw * 0.00025 = 深度（米））
            
        Returns:
            深度值（米），如果无效则返回None
        """
        if depth_image is None:
            return None
        
        height, width = depth_image.shape[:2]
        
        # 检查像素坐标是否在有效范围内
        if pixel_u < 0 or pixel_u >= width or pixel_v < 0 or pixel_v >= height:
            return None
        
        # 使用圆形搜索区域，从内到外搜索
        for r in range(0, search_radius + 1):
            depths = []
            for du in range(-r, r + 1):
                for dv in range(-r, r + 1):
                    # 检查是否在圆形范围内
                    if du * du + dv * dv > r * r:
                        continue
                    
                    u = pixel_u + du
                    v = pixel_v + dv
                    
                    if u < 0 or u >= width or v < 0 or v >= height:
                        continue
                    
                    # 读取深度值（原始像素值，16位无符号整数）
                    depth_value = depth_image[v, u]
                    
                    # 检查是否为有效深度值（0 和 65535 表示无效）
                    if depth_value > 0 and depth_value < 65535:
                        depths.append(float(depth_value))
            
            # 如果找到有效深度值，返回中位数（更鲁棒）
            if len(depths) > 0:
                median_depth_raw = float(np.median(depths))
                depth_meters = median_depth_raw * depth_scale
                return depth_meters
        
        # 未找到有效深度值
        return None
    
    def estimate_pose(
        self,
        target_feature: ComponentFeature,
        best_template: TemplateItem,
        camera_matrix: np.ndarray,
        T_B_C: np.ndarray,
        T_E_C: np.ndarray,
        depth_image: Optional[np.ndarray] = None,
        T_B_C_template: Optional[np.ndarray] = None,
        feature_distance: Optional[float] = None,
        dtheta_rad: Optional[float] = None
    ) -> PoseEstimationResult:
        """估计姿态（使用深度图计算3D位置）
        
        Args:
            target_feature: 目标特征
            best_template: 最佳匹配模板
            camera_matrix: 相机内参矩阵
            T_B_C: 基座到相机的变换矩阵（当前拍摄时）
            T_E_C: 末端到相机的变换矩阵
            depth_image: 深度图像（可选，如果提供则使用实际深度）
            T_B_C_template: 模板拍摄时的基座到相机的变换矩阵（可选）
            
        Returns:
            姿态估计结果
        """
        if self.calib_file_path:
            self.logger.info(f'手眼标定文件: {self.calib_file_path}')
        
        result = PoseEstimationResult()
        result.template_id = best_template.id
        
        # 获取相机内参
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        # 计算2D对齐角度差
        if dtheta_rad is not None:
            dtheta = dtheta_rad
        else:
            template_angle = best_template.feature.standardized_angle if best_template.feature else 0.0
            target_angle = target_feature.standardized_angle
            dtheta = target_angle - template_angle
        
        # 获取模板抓取姿态（已经是T_C_E_grasp，相机坐标系到末端执行器）
        T_C_E_grasp_template = best_template.T_C_E_grasp.copy()
        
        # 如果T_B_C_template未提供，假设与当前相同（眼在手上系统）
        if T_B_C_template is None:
            T_B_C_template = T_B_C.copy()
        
        # 步骤1: 从T_C_E_grasp获取Z坐标（相机坐标系下的深度）
        z_template_camera = T_C_E_grasp_template[2, 3]
        
        if z_template_camera <= 0:
            self.logger.warning(f'模板抓取姿态的Z坐标无效 ({z_template_camera:.3f})，使用模板姿态')
            # 将T_C_E转换为T_B_E
            T_B_C_template = T_B_C_template if T_B_C_template is not None else T_B_C
            T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template
            result.T_B_E_grasp = T_B_E_grasp_template
            if best_template.T_C_E_prep is not None:
                result.T_B_E_prep = T_B_C_template @ best_template.T_C_E_prep
            else:
                result.T_B_E_prep = np.eye(4)
            result.confidence = 0.5
            x, y = target_feature.workpiece_center
            r = target_feature.workpiece_radius
            result.bounding_box = (int(x - r), int(y - r), int(2 * r), int(2 * r))
            return result
        
        # 步骤2: 将T_C_E_grasp转换为T_B_E_grasp（基座坐标系）
        T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template
        template_grasp_pos = T_B_E_grasp_template[:3, 3]
        
        # 步骤3: 计算模板工件中心在基座坐标系下的位置
        template_center_u = best_template.feature.workpiece_center[0]
        template_center_v = best_template.feature.workpiece_center[1]
        
        # 先获取目标中心深度（后续会用到）
        target_center_u = target_feature.workpiece_center[0]
        target_center_v = target_feature.workpiece_center[1]
        target_center_z_camera = None
        if depth_image is not None:
            target_center_z_camera = self._get_depth_from_image(
                depth_image,
                int(target_center_u),
                int(target_center_v),
                search_radius=3,
                depth_scale=0.00025
            )
        
        # 模板中心位置的深度：如果拍照姿态相同（T_B_C == T_B_C_template），
        # 模板中心位置和目标中心位置在相机坐标系下的深度应该相同（因为它们都在相机前方相同的位置）
        # 因此使用目标中心深度来计算模板中心位置
        template_center_z_camera = z_template_camera  # 默认使用抓取位置的深度
        
        # 如果拍照姿态相同（T_B_C_template不为None），直接使用目标中心深度计算模板中心位置
        # 因为拍照姿态相同，模板中心位置和目标中心位置在相机坐标系下的深度应该相同
        if target_center_z_camera is not None and T_B_C_template is not None:
            # 直接使用目标中心深度计算模板中心位置（无需比较，因为拍照姿态一定相同）
            template_center_z_camera = target_center_z_camera
        
        template_center_x_camera = (template_center_u - cx) * template_center_z_camera / fx
        template_center_y_camera = (template_center_v - cy) * template_center_z_camera / fy
        
        template_center_camera = np.eye(4)
        template_center_camera[:3, 3] = [template_center_x_camera, template_center_y_camera, template_center_z_camera]
        template_center_base = T_B_C_template @ template_center_camera
        template_center_base_pos = template_center_base[:3, 3]
        
        # 步骤4: 计算模板抓取位置相对于模板中心的XY偏移
        offset_template_base = template_grasp_pos - template_center_base_pos
        
        # 步骤5: 计算目标工件中心在机器人基座坐标系下的位置
        # 注意：target_center_z_camera和target_center_u/v已经在步骤3中获取了
        if target_center_z_camera is None:
            target_center_z_camera = template_center_z_camera
        
        target_center_x_camera = (target_center_u - cx) * target_center_z_camera / fx
        target_center_y_camera = (target_center_v - cy) * target_center_z_camera / fy
        
        target_center_camera = np.eye(4)
        target_center_camera[:3, 3] = [target_center_x_camera, target_center_y_camera, target_center_z_camera]
        target_center_base = T_B_C @ target_center_camera
        target_center_base_pos = target_center_base[:3, 3]
        
        # 保存目标中心在相机坐标系下的位置
        result.target_center_camera = (target_center_x_camera, target_center_y_camera, target_center_z_camera)
        result.has_target_center_camera = True
        
        # ========== 自动抓取姿态计算（详细日志） ==========
        self.logger.info('========== 自动抓取姿态计算 ==========')
        
        # 【输入】打印输入参数
        self.logger.info('[输入]')
        self.logger.info(f'  模板ID: {best_template.id}')
        self.logger.info(f'  目标中心(像素): ({target_center_u:.1f}, {target_center_v:.1f}), 半径: {target_feature.workpiece_radius:.1f}px')
        self.logger.info(f'  角度差: {np.degrees(dtheta):.2f}° ({dtheta:.4f}rad)')
        self.logger.info(f'  模板抓取位置(基座系): ({template_grasp_pos[0]:.4f}, {template_grasp_pos[1]:.4f}, {template_grasp_pos[2]:.4f})m')
        self.logger.info(f'  模板中心位置(基座系): ({template_center_base_pos[0]:.4f}, {template_center_base_pos[1]:.4f}, {template_center_base_pos[2]:.4f})m')
        self.logger.info(f'  目标中心深度(相机系): {target_center_z_camera:.4f}m')
        
        # 步骤6: 计算抓取姿态
        # 构建绕Z轴的旋转矩阵（在基座坐标系中）
        cos_dtheta = np.cos(dtheta)
        sin_dtheta = np.sin(dtheta)
        R_z_base = np.array([
            [cos_dtheta, -sin_dtheta, 0],
            [sin_dtheta, cos_dtheta, 0],
            [0, 0, 1]
        ])
        
        # 计算模板中心与抓取位置的偏差（在基座坐标系中，只取XY）
        template_grasp_offset_xy = np.array([offset_template_base[0], offset_template_base[1], 0.0])
        
        # 【计算过程】打印中间值
        self.logger.info('[计算过程]')
        self.logger.info(f'  1. 模板抓取偏移(基座系XY): ({template_grasp_offset_xy[0]:.4f}, {template_grasp_offset_xy[1]:.4f})m')
        
        # 对目标中心的XY坐标应用这个偏差
        intermediate_pos = target_center_base_pos + template_grasp_offset_xy
        self.logger.info(f'  2. 中间位置(目标中心+偏移): ({intermediate_pos[0]:.4f}, {intermediate_pos[1]:.4f}, {intermediate_pos[2]:.4f})m')
        
        # 将中间数值绕目标工件中心的Z轴旋转dtheta角度
        intermediate_offset = intermediate_pos - target_center_base_pos
        rotated_intermediate_offset = R_z_base @ intermediate_offset
        self.logger.info(f'  3. 中间偏移向量: ({intermediate_offset[0]:.4f}, {intermediate_offset[1]:.4f}, {intermediate_offset[2]:.4f})m')
        self.logger.info(f'  4. 旋转后偏移向量: ({rotated_intermediate_offset[0]:.4f}, {rotated_intermediate_offset[1]:.4f}, {rotated_intermediate_offset[2]:.4f})m')
        
        # 计算目标抓取位置：目标中心 + 旋转后的偏移，Z直接使用模板的Z
        target_grasp_pos_base = target_center_base_pos + rotated_intermediate_offset
        target_grasp_pos_base[2] = template_grasp_pos[2]  # Z使用模板的Z
        
        # 计算抓取姿态的旋转
        R_template_grasp = T_B_E_grasp_template[:3, :3]
        R_target_grasp = R_z_base @ R_template_grasp
        
        # 构建最终的抓取姿态
        T_B_E_grasp_current = np.eye(4)
        T_B_E_grasp_current[:3, :3] = R_target_grasp
        T_B_E_grasp_current[:3, 3] = target_grasp_pos_base
        
        result.T_B_E_grasp = T_B_E_grasp_current
        
        # 从旋转矩阵提取欧拉角（RPY）
        roll = np.arctan2(R_target_grasp[2, 1], R_target_grasp[2, 2])
        pitch = -np.arcsin(R_target_grasp[2, 0])
        yaw = np.arctan2(R_target_grasp[1, 0], R_target_grasp[0, 0])
        
        # 【输出】打印最终结果
        self.logger.info('[输出]')
        self.logger.info(f'  抓取位置(基座系): ({target_grasp_pos_base[0]:.4f}, {target_grasp_pos_base[1]:.4f}, {target_grasp_pos_base[2]:.4f})m')
        self.logger.info(f'  抓取姿态(RPY): ({np.degrees(roll):.2f}°, {np.degrees(pitch):.2f}°, {np.degrees(yaw):.2f}°)')
        self.logger.info('========================================')
        
        # 计算准备姿态（如果有）
        target_prep_pos_base = None  # 初始化变量，用于最终汇总
        if best_template.T_C_E_prep is not None and not np.allclose(best_template.T_C_E_prep, np.eye(4)):
            # 将T_C_E_prep转换为T_B_E_prep
            T_B_E_prep_template = T_B_C_template @ best_template.T_C_E_prep
            prep_pos_template = T_B_E_prep_template[:3, 3]
            offset_prep_base = prep_pos_template - template_center_base_pos
            
            intermediate_prep_pos = target_center_base_pos + offset_prep_base[:3]
            intermediate_prep_offset = intermediate_prep_pos - target_center_base_pos
            rotated_prep_offset = R_z_base @ intermediate_prep_offset
            
            target_prep_pos_base = target_center_base_pos + rotated_prep_offset
            target_prep_pos_base[2] = prep_pos_template[2]
            
            R_template_prep = T_B_E_prep_template[:3, :3]
            R_target_prep = R_z_base @ R_template_prep
            
            T_B_E_prep_current = np.eye(4)
            T_B_E_prep_current[:3, :3] = R_target_prep
            T_B_E_prep_current[:3, 3] = target_prep_pos_base
            
            result.T_B_E_prep = T_B_E_prep_current
            
            prep_roll = np.arctan2(R_target_prep[2, 1], R_target_prep[2, 2])
            prep_pitch = -np.arcsin(R_target_prep[2, 0])
            prep_yaw = np.arctan2(R_target_prep[1, 0], R_target_prep[0, 0])
            
            self.logger.info('[准备姿态输出]')
            self.logger.info(f'  准备位置(基座系): ({target_prep_pos_base[0]:.4f}, {target_prep_pos_base[1]:.4f}, {target_prep_pos_base[2]:.4f})m')
            self.logger.info(f'  准备姿态(RPY): ({np.degrees(prep_roll):.2f}°, {np.degrees(prep_pitch):.2f}°, {np.degrees(prep_yaw):.2f}°)')
        else:
            result.T_B_E_prep = np.eye(4)
        
        # 加载预放置和放置姿态（如果存在）
        # 这些姿态通常不需要转换，直接使用模板的值
        # 注意：模板目录路径需要从外部传入，这里暂时跳过
        # 预放置和放置姿态将在ros2_communication.py中处理
        
        # 计算置信度
        # 如果提供了置信度（来自暴力匹配），直接使用
        # 否则基于特征距离计算
        if hasattr(self, '_last_match_confidence') and self._last_match_confidence is not None:
            result.confidence = self._last_match_confidence
            self.logger.info(f'匹配置信度: {result.confidence:.4f}')
            self._last_match_confidence = None  # 清除临时值
        elif feature_distance is not None:
            result.confidence = 1.0 / (1.0 + feature_distance)
            self.logger.info(f'置信度(特征距离): {result.confidence:.4f} (距离={feature_distance:.4f})')
        else:
            # 默认置信度，可以根据特征匹配质量调整
            result.confidence = 0.8
            self.logger.warning(f'使用默认置信度: {result.confidence:.4f}')
        
        # 获取边界框
        x, y = target_feature.workpiece_center
        r = target_feature.workpiece_radius
        result.bounding_box = (
            int(x - r), int(y - r),
            int(2 * r), int(2 * r)
        )
        return result
