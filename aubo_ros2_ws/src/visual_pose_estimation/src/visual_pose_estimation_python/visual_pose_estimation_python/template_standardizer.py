#!/usr/bin/env python3
"""
模板标准化模块

功能：
1. 标准化模板姿态（旋转到标准方向）
2. 保存标准化后的模板
3. 加载已标准化的模板
"""

import cv2
import numpy as np
import json
import logging
import traceback
from pathlib import Path
from typing import Dict, Tuple, Optional, Union
from dataclasses import dataclass
from .debug_visualizer import DebugVisualizer


@dataclass
class StandardizedTemplate:
    """标准化模板数据结构"""
    template_id: str                    # 模板ID
    pose_id: str                        # 姿态ID
    
    # 原始图像和掩码
    original_image_path: str            # 原始图像路径
    preprocessed_image_path: str        # 预处理图像路径（白底抠图）
    depth_display_path: str             # 处理后的深度图路径（用于显示，无遮罩框）
    mask_path: str                      # 掩码路径
    standardized_mask_path: str         # 标准化后掩码路径
    
    # 特征信息
    workpiece_center: Tuple[float, float]
    workpiece_radius: float
    valve_center: Tuple[float, float]
    valve_radius: float
    standardized_angle: float          # 弧度
    standardized_angle_deg: float      # 度
    
    # 标准化信息
    is_standardized: bool = False      # 是否已标准化


class TemplateStandardizer:
    """模板标准化器类"""
    
    def __init__(self):
        """初始化模板标准化器"""
        self.logger = logging.getLogger(__name__)
        self.debug_visualizer = DebugVisualizer()
    
    def standardize_template(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        workpiece_center: Tuple[float, float],
        valve_center: Tuple[float, float],
        standardized_angle: float,
        workpiece_radius: float
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, float]]:
        """标准化模板（裁剪工件区域，不旋转，参考C++版本）
        
        Args:
            image: 原始图像
            mask: 原始掩码
            workpiece_center: 工件中心
            valve_center: 阀体中心
            standardized_angle: 标准化角度（弧度，用于姿态计算，但不旋转图像）
            workpiece_radius: 工件半径
            
        Returns:
            (标准化后的图像, 标准化后的掩码, 裁剪参数字典)
        """
        # 计算裁剪区域（以工件外接圆为中心的正方形）
        crop_x, crop_y, crop_w, crop_h = self._calculate_crop_region(
            workpiece_center, workpiece_radius, image.shape
        )

        # 裁剪图像和掩码，并应用白色背景
        cropped_image, cropped_mask = self._crop_with_white_background(
            image, mask, crop_x, crop_y, crop_w, crop_h
        )
        
        # 计算裁剪后的阀体中心（在裁剪后的图像坐标系中）
        valve_center_cropped = np.array([
            valve_center[0] - crop_x,
            valve_center[1] - crop_y
        ], dtype=np.float32)
        
        # 旋转角度设为0（不旋转）
        rotation_angle = 0.0
        
        # 构建裁剪参数字典（用于后续保存）
        crop_params = {
            'crop_x': float(crop_x),
            'crop_y': float(crop_y),
            'crop_width': float(crop_w),
            'crop_height': float(crop_h),
            'rotation_angle_rad': rotation_angle,
            'rotation_angle_deg': rotation_angle * 180.0 / np.pi,
            'final_valve_center_x': float(valve_center_cropped[0]),
            'final_valve_center_y': float(valve_center_cropped[1]),
        }
        
        
        return cropped_image, cropped_mask, crop_params
    
    def _calculate_crop_region(
        self,
        workpiece_center: Tuple[float, float],
        workpiece_radius: float,
        image_shape: Tuple[int, int, int]
    ) -> Tuple[int, int, int, int]:
        """计算裁剪区域（以工件外接圆为中心的正方形）
        
        Args:
            workpiece_center: 工件中心坐标
            workpiece_radius: 工件半径
            image_shape: 图像尺寸 (height, width, channels)
            
        Returns:
            (crop_x, crop_y, crop_w, crop_h) 裁剪区域坐标和尺寸
        """
        radius = float(workpiece_radius)
        center = np.array(workpiece_center, dtype=np.float32)
        
        crop_size = int(np.ceil(radius * 2.0))
        crop_x = int(max(0.0, center[0] - radius))
        crop_y = int(max(0.0, center[1] - radius))
        crop_w = min(crop_size, image_shape[1] - crop_x)
        crop_h = min(crop_size, image_shape[0] - crop_y)
        
        return crop_x, crop_y, crop_w, crop_h
    
    def _crop_with_white_background(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        crop_x: int,
        crop_y: int,
        crop_w: int,
        crop_h: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """裁剪图像和掩码，并应用白色背景
        
        Args:
            image: 原始图像
            mask: 原始掩码
            crop_x, crop_y: 裁剪起始坐标
            crop_w, crop_h: 裁剪尺寸
            
        Returns:
            (cropped_image, cropped_mask) 裁剪后的图像和掩码
        """
        # 裁剪掩码
        cropped_mask = mask[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w].copy()
        
        # 使用掩膜抠图工件，背景用白色填充
        cropped_image = np.full((crop_h, crop_w, 3), 255, dtype=np.uint8)  # 白色背景（BGR格式）
        cropped_original = image[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w].copy()
        cropped_image[cropped_mask > 0] = cropped_original[cropped_mask > 0]
        cropped_image[cropped_mask == 0] = [255, 255, 255]  # 白色背景
        
        return cropped_image, cropped_mask
    
    def compute_standardized_pose(
        self,
        T_B_E_pose: np.ndarray,
        T_B_E_camera: np.ndarray,
        T_E_C: np.ndarray,
        camera_matrix: np.ndarray,
        feature_params: Dict[str, float]
    ) -> np.ndarray:
        """计算标准化姿态：将原始姿态转换为标准化坐标系下的姿态
        
        Args:
            T_B_E_pose: 原始准备姿态（基座坐标系）
            T_B_E_camera: 相机姿态（基座坐标系）
            T_E_C: 手眼变换矩阵
            camera_matrix: 相机内参矩阵
            feature_params: 特征参数字典
            
        Returns:
            标准化后的姿态（基座坐标系）
        """
        # 计算相机坐标系下的姿态
        T_B_C_camera = T_B_E_camera @ T_E_C
        T_C_camera_B = np.linalg.inv(T_B_C_camera)
        T_C_camera_E_pose = T_C_camera_B @ T_B_E_pose
        
        R_pose_camera = T_C_camera_E_pose[:3, :3]
        t_pose_camera = T_C_camera_E_pose[:3, 3]
        
        # 获取旋转角度（现在为0，因为不旋转）
        rotation_angle_rad = feature_params.get('rotation_angle_rad', 0.0)
        
        # 构建绕Z轴旋转矩阵
        cos_angle = np.cos(rotation_angle_rad)
        sin_angle = np.sin(rotation_angle_rad)
        rotation_z_camera = np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle, cos_angle, 0],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # 计算工件中心在相机坐标系下的位置
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        workpiece_center_x_original = feature_params['workpiece_center_x_original']
        workpiece_center_y_original = feature_params['workpiece_center_y_original']
        workpiece_center_z_camera = t_pose_camera[2]
        
        workpiece_center_camera = np.array([
            (workpiece_center_x_original - cx) * workpiece_center_z_camera / fx,
            (workpiece_center_y_original - cy) * workpiece_center_z_camera / fy,
            workpiece_center_z_camera
        ], dtype=np.float64)
        
        # 计算从工件中心到姿态位置的偏移
        offset_from_center = t_pose_camera - workpiece_center_camera
        offset_x = offset_from_center[0]
        offset_y = offset_from_center[1]
        
        # 旋转偏移（注意：由于不再旋转图像，rotation_angle_rad应该是0，所以不需要旋转）
        # 但是为了保持代码的一致性，仍然使用旋转矩阵（即使角度为0）
        offset_x_rotated = offset_x * cos_angle - offset_y * sin_angle
        offset_y_rotated = offset_x * sin_angle + offset_y * cos_angle
        
        # 计算标准化后的位置
        t_standardized_camera = np.array([
            workpiece_center_camera[0] + offset_x_rotated,
            workpiece_center_camera[1] + offset_y_rotated,
            t_pose_camera[2]
        ], dtype=np.float64)
        
        # 计算标准化后的旋转（注意：由于不再旋转，rotation_angle_rad=0，所以R_standardized_camera = R_pose_camera）
        R_standardized_camera = rotation_z_camera @ R_pose_camera
        
        # 构建标准化变换矩阵
        T_C_camera_E_standardized = np.eye(4, dtype=np.float64)
        T_C_camera_E_standardized[:3, :3] = R_standardized_camera
        T_C_camera_E_standardized[:3, 3] = t_standardized_camera
        
        # 转换回基座坐标系
        T_B_E_standardized = T_B_C_camera @ T_C_camera_E_standardized
        
        return T_B_E_standardized
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> Tuple[float, float, float, float]:
        """将旋转矩阵转换为四元数（参考C++版本）"""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return (x, y, z, w)
    
    def _pose_to_json(self, T_B_E: np.ndarray) -> Dict:
        """将姿态矩阵转换为JSON格式（参考C++版本）"""
        position = {
            "x": float(T_B_E[0, 3]),
            "y": float(T_B_E[1, 3]),
            "z": float(T_B_E[2, 3])
        }
        
        R = T_B_E[:3, :3]
        x, y, z, w = self._rotation_matrix_to_quaternion(R)
        
        orientation = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "w": float(w)
        }
        
        return {
            "cartesian_position": {
                "position": position,
                "orientation": orientation
            }
        }
    
    def _build_feature_params_from_feature(
        self,
        feature,
        crop_params: Optional[Dict[str, float]] = None
    ) -> Dict[str, float]:
        """从ComponentFeature对象构建特征参数字典（适配现有特征检测流程）
        
        Args:
            feature: ComponentFeature对象（从特征提取得到）
            crop_params: 裁剪参数字典（从standardize_template返回）
            
        Returns:
            特征参数字典
        """
        feature_params = {
            'workpiece_center_x_original': float(feature.workpiece_center[0]),
            'workpiece_center_y_original': float(feature.workpiece_center[1]),
            'workpiece_radius_original': float(feature.workpiece_radius),
            'workpiece_area': float(getattr(feature, 'workpiece_area', 0.0)),
            'valve_center_x': float(feature.valve_center[0]),
            'valve_center_y': float(feature.valve_center[1]),
            'valve_radius': float(feature.valve_radius),
            'valve_area': float(getattr(feature, 'valve_area', 0.0)),
            'standardized_angle_rad': float(feature.standardized_angle),
            'standardized_angle_deg': float(np.degrees(feature.standardized_angle)),
            'final_workpiece_radius': float(feature.workpiece_radius),
        }
        if crop_params:
            feature_params.update(crop_params)
        return feature_params
    
    def _save_preprocessed_image(
        self,
        preprocessed_image: np.ndarray,
        pose_dir: Path,
        feature_obj: Optional[object],
        feature_params: Dict[str, float]
    ) -> None:
        """保存预处理图像（含特征绘制）
        
        Args:
            preprocessed_image: 预处理图像（白色背景抠图）
            pose_dir: 姿态目录
            feature_obj: ComponentFeature对象（可选）
            feature_params: 特征参数字典
        """
        try:
            # 如果有ComponentFeature对象，直接调用其draw_features方法
            if feature_obj is not None:
                preprocessed_with_features = feature_obj.draw_features(preprocessed_image)
            else:
                # 使用debug_visualizer的draw_features_overlay方法
                feature_dict = {
                    'workpiece_center': (
                        feature_params.get('workpiece_center_x_original', 0.0),
                        feature_params.get('workpiece_center_y_original', 0.0)
                    ),
                    'workpiece_radius': feature_params.get('workpiece_radius_original', 0.0),
                    'valve_center': (
                        feature_params.get('valve_center_x', 0.0),
                        feature_params.get('valve_center_y', 0.0)
                    ),
                    'valve_radius': feature_params.get('valve_radius', 0.0),
                    'angle_deg': feature_params.get('standardized_angle_deg', 0.0)
                }
                preprocessed_with_features = self.debug_visualizer.draw_features_overlay(
                    preprocessed_image, [feature_dict], draw_bbox=False
                )
            
            # 为了便于人眼查看，这里以 RGB 顺序保存到 JPG
            preprocessed_rgb = cv2.cvtColor(preprocessed_with_features, cv2.COLOR_BGR2RGB)
            preprocessed_image_path = pose_dir / "preprocessed_image.jpg"
            cv2.imwrite(str(preprocessed_image_path), preprocessed_rgb, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self.logger.info(f'保存预处理图像: {preprocessed_image_path}')
        except Exception as e:
            self.logger.warning(f'绘制特征失败，保存未绘制特征的图像: {e}')
            preprocessed_with_features = preprocessed_image.copy()
            preprocessed_rgb = cv2.cvtColor(preprocessed_with_features, cv2.COLOR_BGR2RGB)
            preprocessed_image_path = pose_dir / "preprocessed_image.jpg"
            cv2.imwrite(str(preprocessed_image_path), preprocessed_rgb, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self.logger.info(f'保存预处理图像: {preprocessed_image_path}')
    
    def _build_template_info(
        self,
        feature_params: Dict[str, float],
        T_B_E_grasp: Optional[np.ndarray],
        T_B_E_standardized_grasp: Optional[np.ndarray],
        T_B_E_preparation: Optional[np.ndarray],
        T_B_E_standardized_preparation: Optional[np.ndarray]
    ) -> Dict:
        """构建template_info.json内容
        
        Args:
            feature_params: 特征参数字典
            T_B_E_grasp: 原始抓取姿态（可选）
            T_B_E_standardized_grasp: 标准化抓取姿态（可选）
            T_B_E_preparation: 原始准备姿态（可选）
            T_B_E_standardized_preparation: 标准化准备姿态（可选）
            
        Returns:
            template_info字典
        """
        template_info = {
            "feature_parameters": feature_params,
        }
        # 仅在姿态存在时添加姿态信息
        if T_B_E_grasp is not None and T_B_E_standardized_grasp is not None:
            template_info["original_grasp_pose"] = self._pose_to_json(T_B_E_grasp)
            template_info["standardized_grasp_pose"] = self._pose_to_json(T_B_E_standardized_grasp)
        if T_B_E_preparation is not None and T_B_E_standardized_preparation is not None:
            template_info["original_preparation_pose"] = self._pose_to_json(T_B_E_preparation)
            template_info["standardized_preparation_pose"] = self._pose_to_json(T_B_E_standardized_preparation)
        return template_info
    
    def _save_metadata(
        self,
        template_dir: Path,
        pose_id: str,
        image_path: Path,
        mask_path: Path,
        preprocessed_image: Optional[np.ndarray]
    ) -> None:
        """保存metadata.json（可选，默认关闭）
        
        Args:
            template_dir: 模板目录
            pose_id: 姿态ID
            image_path: 图像路径
            mask_path: 掩码路径
            preprocessed_image: 预处理图像（可选）
        """
        metadata = {
            "template_id": template_dir.name,
            "pose_id": pose_id,
            "image_path": str(image_path.relative_to(template_dir)),
            "mask_path": str(mask_path.relative_to(template_dir)),
            "is_standardized": True,
        }
        # 如果保存了预处理图像，添加到metadata中
        if preprocessed_image is not None:
            pose_dir = template_dir / pose_id
            preprocessed_image_path = pose_dir / "preprocessed_image.jpg"
            metadata["preprocessed_image_path"] = str(preprocessed_image_path.relative_to(template_dir))
        metadata_path = template_dir / pose_id / "metadata.json"
        with open(metadata_path, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
    
    def save_standardized_template(
        self,
        template_dir: Path,
        pose_id: str,
        standardized_image: np.ndarray,
        standardized_mask: np.ndarray,
        feature_params: Union[Dict[str, float], object],
        T_B_E_grasp: Optional[np.ndarray] = None,
        T_B_E_standardized_grasp: Optional[np.ndarray] = None,
        T_B_E_preparation: Optional[np.ndarray] = None,
        T_B_E_standardized_preparation: Optional[np.ndarray] = None,
        crop_params: Optional[Dict[str, float]] = None,
        save_metadata: bool = False,
        preprocessed_image: Optional[np.ndarray] = None
    ) -> bool:
        """保存标准化结果（精简版，对齐C++：image/mask/template_info）
        
        适配现有图像处理和特征检测流程：
        - 可直接传入ComponentFeature对象，自动提取特征参数
        - 也可传入已构建的feature_params字典
        
        Args:
            template_dir: 模板目录
            pose_id: 姿态ID
            standardized_image: 标准化后的图像（裁剪结果）
            standardized_mask: 标准化后的掩码（裁剪结果）
            feature_params: 特征参数字典 或 ComponentFeature对象
            T_B_E_grasp: 原始抓取姿态（可选，None时仅保存图像和掩码）
            T_B_E_standardized_grasp: 标准化抓取姿态（可选，None时仅保存图像和掩码）
            T_B_E_preparation: 原始准备姿态（可选）
            T_B_E_standardized_preparation: 标准化准备姿态（可选）
            crop_params: 裁剪参数字典（如果feature_params是ComponentFeature，需要提供）
            save_metadata: 是否额外写metadata.json（默认False，与C++一致）
            preprocessed_image: 预处理图像（白色背景抠图，类似debug选项卡中的预处理图像，可选）
        """
        # 适配：如果传入的是ComponentFeature对象，保存引用以便后续绘制特征
        feature_obj = None  # 保存ComponentFeature对象引用（用于绘制特征）
        if not isinstance(feature_params, dict):
            # 假设是ComponentFeature对象
            feature_obj = feature_params  # 保存对象引用
            try:
                feature_params = self._build_feature_params_from_feature(feature_params, crop_params)
            except Exception as e:
                self.logger.error(f'无法从特征对象提取参数: {e}')
                return False
        try:
            template_dir = Path(template_dir)
            pose_dir = template_dir / pose_id
            pose_dir.mkdir(parents=True, exist_ok=True)
            
            image_path = pose_dir / "image.jpg"
            mask_path = pose_dir / "mask.jpg"
            # standardized_image 是 BGR（OpenCV 内部格式），这里转成 RGB 存盘，便于查看
            image_rgb = cv2.cvtColor(standardized_image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(str(image_path), image_rgb)
            cv2.imwrite(str(mask_path), standardized_mask)
            
            # 保存预处理图像（类似debug选项卡中的预处理图像，白色背景抠图+特征绘制）
            if preprocessed_image is not None:
                self._save_preprocessed_image(
                    preprocessed_image, pose_dir, feature_obj, feature_params
                )

            # 模板设置保存的姿态：不再限制z值

            # 保存template_info.json：特征+姿态（对齐C++，姿态可选）
            template_info = self._build_template_info(
                feature_params,
                T_B_E_grasp,
                T_B_E_standardized_grasp,
                T_B_E_preparation,
                T_B_E_standardized_preparation
            )
            template_info_path = pose_dir / "template_info.json"
            with open(template_info_path, 'w', encoding='utf-8') as f:
                json.dump(template_info, f, indent=2, ensure_ascii=False)

            # 可选写metadata.json（默认关闭）
            # if save_metadata:
            #     self._save_metadata(
            #         template_dir, pose_id, image_path, mask_path, preprocessed_image
            #     )
            
            self.logger.info(f'保存标准化模板: {pose_dir}')
            return True
            
        except Exception as e:
            self.logger.error(f'保存标准化模板失败: {e}')
            return False
    
    def draw_gripper(
        self,
        image: np.ndarray,
        T_C_E: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        gripper_opening_mm: float = 50.0,
        gripper_length_mm: float = 100.0
    ) -> None:
        """在图像上绘制夹爪可视化
        
        参考 C++ 版本的 TemplateStandardizer::drawGripper 实现
        
        Args:
            image: 输入图像（会被修改）
            T_C_E: 末端执行器在相机坐标系下的变换矩阵 (4x4)
            camera_matrix: 相机内参矩阵 (3x3)
            dist_coeffs: 畸变系数（可选）
            gripper_opening_mm: 夹爪开口宽度（毫米）
            gripper_length_mm: 夹爪长度（毫米）
        """
        if T_C_E is None or camera_matrix is None:
            self.logger.warning('T_C_E 或 camera_matrix 为空，跳过夹爪绘制')
            return
        
        try:
            # 检查Z坐标是否在合理范围内（应该在相机前方）
            ee_pos = np.array([T_C_E[0, 3], T_C_E[1, 3], T_C_E[2, 3]])
            if ee_pos[2] <= 0:
                self.logger.warning(
                    f'夹爪末端Z坐标={ee_pos[2]:.3f}m ({ee_pos[2]*1000:.1f}mm) <= 0（在相机后方），无法投影到图像'
                )
                return
            # 提取末端执行器原点在相机坐标系下的位置
            ee_pos_camera = np.array([
                T_C_E[0, 3],
                T_C_E[1, 3],
                T_C_E[2, 3]
            ])
            
            # 提取末端执行器坐标系的三个轴在相机坐标系下的方向
            gripper_x_axis = T_C_E[:3, 0]
            gripper_y_axis = T_C_E[:3, 1]
            gripper_z_axis = T_C_E[:3, 2]
            
            # 归一化
            x_norm = np.linalg.norm(gripper_x_axis)
            y_norm = np.linalg.norm(gripper_y_axis)
            z_norm = np.linalg.norm(gripper_z_axis)
            if x_norm > 1e-6:
                gripper_x_axis = gripper_x_axis / x_norm
            if y_norm > 1e-6:
                gripper_y_axis = gripper_y_axis / y_norm
            if z_norm > 1e-6:
                gripper_z_axis = gripper_z_axis / z_norm
            
            # 夹爪坐标系定义：Z轴=指向方向，Y轴=开口方向
            pointing_direction = gripper_z_axis
            opening_direction = gripper_y_axis
            
            # 计算夹爪的两个手指位置
            # 注意：T_C_E中的位置单位应该是米，所以夹爪尺寸也需要转换为米
            finger_center = ee_pos_camera
            
            # 两个手指分别沿着开口方向的正负方向偏移（毫米转米）
            gripper_opening_m = gripper_opening_mm / 1000.0
            gripper_length_m = gripper_length_mm / 1000.0
            finger1_center = finger_center + opening_direction * (gripper_opening_m / 2.0)
            finger2_center = finger_center - opening_direction * (gripper_opening_m / 2.0)
            
            # 手指沿着指向方向延伸
            finger1_tip = finger1_center + pointing_direction * gripper_length_m
            finger2_tip = finger2_center + pointing_direction * gripper_length_m
            
            # 将3D点投影到2D像素坐标
            # 注意：所有3D坐标单位统一为米（m）
            object_points = np.array([
                finger_center,
                finger1_center,
                finger1_tip,
                finger2_center,
                finger2_tip
            ], dtype=np.float64)
            
            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]
            
            # 注意：相机内参fx/fy的单位通常取决于标定时使用的棋盘格单位
            # OpenCV标定通常使用毫米单位的棋盘格，所以fx单位是"像素/毫米"
            # 而我们的3D点单位是米，需要转换为毫米以匹配相机内参单位
            # 或者，如果相机内参单位是"像素/米"，则直接使用米单位的3D点
            
            # 尝试两种方式：
            # 方式1：假设相机内参单位是"像素/毫米"，将3D点转换为毫米
            # 方式2：假设相机内参单位是"像素/米"，直接使用米单位的3D点
            # 根据常见的相机标定实践，先尝试方式1（像素/毫米）
            
            image_points = []
            for i, pt_3d in enumerate(object_points):
                if pt_3d[2] > 0:  # 确保点在相机前方（单位：米）
                    # 将3D点从米转换为毫米，以匹配"像素/毫米"单位的相机内参
                    pt_3d_mm = pt_3d * 1000.0
                    u = fx * pt_3d_mm[0] / pt_3d_mm[2] + cx
                    v = fy * pt_3d_mm[1] / pt_3d_mm[2] + cy
                    image_points.append((u, v))
                    # 跳过无效点
                else:
                    image_points.append((-1, -1))  # 无效点
                    self.logger.warning(f'  点{i}: Z={pt_3d[2]:.3f}m ({pt_3d[2]*1000:.1f}mm) <= 0，无效')
            
            # 检查点是否有效（在图像范围内）
            img_h, img_w = image.shape[:2]
            
            def is_valid_point(pt):
                return pt[0] >= 0 and pt[1] >= 0 and pt[0] < img_w and pt[1] < img_h
            
            # 统计有效点数量
            valid_count = sum(1 for pt in image_points if is_valid_point(pt))
            
            # 绘制夹爪（红色）
            red_color = (0, 0, 255)  # BGR格式，红色
            thickness = 3
            circle_radius = 5
            
            # 绘制中心点
            if len(image_points) > 0 and is_valid_point(image_points[0]):
                cv2.circle(image, (int(image_points[0][0]), int(image_points[0][1])), 
                          circle_radius, red_color, -1)
            
            # 绘制第一个手指（从中心到指尖）
            if len(image_points) > 2 and is_valid_point(image_points[1]) and is_valid_point(image_points[2]):
                pt1 = (int(image_points[1][0]), int(image_points[1][1]))
                pt2 = (int(image_points[2][0]), int(image_points[2][1]))
                cv2.line(image, pt1, pt2, red_color, thickness)
                cv2.circle(image, pt2, circle_radius, red_color, -1)
            
            # 绘制第二个手指（从中心到指尖）
            if len(image_points) > 4 and is_valid_point(image_points[3]) and is_valid_point(image_points[4]):
                pt1 = (int(image_points[3][0]), int(image_points[3][1]))
                pt2 = (int(image_points[4][0]), int(image_points[4][1]))
                cv2.line(image, pt1, pt2, red_color, thickness)
                cv2.circle(image, pt2, circle_radius, red_color, -1)
            
            # 绘制连接两个手指中心的线
            if len(image_points) > 3 and is_valid_point(image_points[1]) and is_valid_point(image_points[3]):
                pt1 = (int(image_points[1][0]), int(image_points[1][1]))
                pt2 = (int(image_points[3][0]), int(image_points[3][1]))
                cv2.line(image, pt1, pt2, red_color, thickness)
                
        except Exception as e:
            self.logger.warning(f'绘制夹爪可视化失败: {e}')
