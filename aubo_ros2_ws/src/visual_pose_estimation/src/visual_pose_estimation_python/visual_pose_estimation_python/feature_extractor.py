#!/usr/bin/env python3
"""
特征提取模块

功能（参考trigger_depth.py）：
1. 初步筛选干扰连通域
2. 提取工件外接圆特征（大圆）
3. 提取阀体外接圆特征（小圆）
4. 计算标准化旋转角度
5. 多线程并行处理连通域
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import logging
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed


@dataclass
class ComponentFeature:
    """连通域特征结构体"""
    # 工件整体外接圆特征
    workpiece_center: Tuple[float, float]  # (x, y)
    workpiece_radius: float                # 像素
    workpiece_area: float                  # 像素²
    
    # 阀体外接圆特征
    valve_center: Tuple[float, float]      # (x, y)
    valve_radius: float                    # 像素
    valve_area: float                      # 像素²
    
    # 标准化旋转角度
    standardized_angle: float              # 弧度
    standardized_angle_deg: float          # 度
    
    # 原始连通域掩码（可选，用于调试）
    component_mask: Optional[np.ndarray] = None
    
    # 彩色工件图像（可选，用于可视化）
    color_image: Optional[np.ndarray] = None
    
    def draw_features(self, image: Optional[np.ndarray] = None) -> np.ndarray:
        """在图像上绘制特征可视化（参考trigger_depth.py）
        
        Args:
            image: 要绘制的图像（如果为None，使用color_image）
            
        Returns:
            绘制了特征的图像
        """
        if image is None:
            if self.color_image is not None:
                image = self.color_image.copy()
            else:
                # 创建白色背景
                image = np.full((480, 640, 3), (255, 255, 255), dtype=np.uint8)
        else:
            image = image.copy()
        
        img_h, img_w = image.shape[:2]
        
        # 绘制工件外接圆（绿色）
        if self.workpiece_center is not None and self.workpiece_radius > 0:
            wp_x = int(self.workpiece_center[0])
            wp_y = int(self.workpiece_center[1])
            wp_r = int(self.workpiece_radius)
            
            if 0 <= wp_x < img_w and 0 <= wp_y < img_h and wp_r > 0:
                wp_r = min(wp_r, max(img_w, img_h))
                cv2.circle(image, (wp_x, wp_y), wp_r, (0, 255, 0), 2)
                cv2.circle(image, (wp_x, wp_y), 5, (0, 255, 0), -1)
                
                # 绘制阀体外接圆（蓝色）
                if self.valve_center is not None and self.valve_radius > 0:
                    valve_x = int(self.valve_center[0])
                    valve_y = int(self.valve_center[1])
                    valve_r = int(self.valve_radius)
                    
                    if 0 <= valve_x < img_w and 0 <= valve_y < img_h and valve_r > 0:
                        valve_r = min(valve_r, max(img_w, img_h))
                        cv2.circle(image, (valve_x, valve_y), valve_r, (255, 0, 0), 2)
                        cv2.circle(image, (valve_x, valve_y), 5, (255, 0, 0), -1)
                        
                        # 绘制连线（青色）
                        cv2.line(image, (wp_x, wp_y), (valve_x, valve_y), (0, 255, 255), 2)
                        
                        # 绘制角度文本
                        angle_text = f'Angle: {self.standardized_angle_deg:.1f}°'
                        text_x = min(valve_x + 10, img_w - 100)
                        text_y = max(valve_y, 20)
                        cv2.putText(image, angle_text, (text_x, text_y),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 添加特征信息文本
        y_offset = 60
        if self.workpiece_center is not None:
            feature_text = f'Workpiece: Center({self.workpiece_center[0]:.1f}, {self.workpiece_center[1]:.1f}) R:{self.workpiece_radius:.1f}'
            cv2.putText(image, feature_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_offset += 20
        
        if self.valve_center is not None:
            valve_text = f'Valve: Center({self.valve_center[0]:.1f}, {self.valve_center[1]:.1f}) R:{self.valve_radius:.1f}'
            cv2.putText(image, valve_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            y_offset += 20
            
            angle_text = f'Angle: {self.standardized_angle_deg:.2f}° ({self.standardized_angle:.4f} rad)'
            cv2.putText(image, angle_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return image


class FeatureExtractor:
    """特征提取器类（参考trigger_depth.py实现）"""
    
    def __init__(self):
        """初始化特征提取器"""
        self.logger = logging.getLogger(__name__)
        self.parameters = {}
        self.result_mutex = threading.Lock()
        self._initialize_default_parameters()
    
    def _initialize_default_parameters(self):
        """初始化默认参数（参考trigger_depth.py）"""
        # 连通域筛选参数
        self.parameters['component_min_area'] = 1000.0
        self.parameters['component_max_area'] = 1000000.0
        self.parameters['component_min_aspect_ratio'] = 0.3
        self.parameters['component_max_aspect_ratio'] = 4.0
        self.parameters['component_min_width'] = 60.0
        self.parameters['component_min_height'] = 60.0
        
        # 大圆（工件）提取参数
        self.parameters['big_circle_combine_contours'] = 1.0  # bool: 是否合并轮廓
        self.parameters['big_circle_min_area'] = 100.0
        
        # 小圆（阀体）提取参数
        self.parameters['small_circle_erode_kernel'] = 11.0
        self.parameters['small_circle_erode_iterations'] = 1.0
        self.parameters['small_circle_largest_cc'] = 1.0  # bool: 只保留最大连通域
        self.parameters['small_circle_dilate_kernel'] = 9.0
        self.parameters['small_circle_dilate_iterations'] = 1.0
        
        # 多线程参数
        self.parameters['max_threads'] = 4.0
    
    def set_parameters(self, params: Dict[str, float]):
        """设置参数
        
        Args:
            params: 参数字典
        """
        self.parameters.update(params)
    
    def get_parameters(self) -> Dict[str, float]:
        """获取参数
        
        Returns:
            参数字典
        """
        return self.parameters.copy()
    
    def _get_param(self, key: str, fallback: float) -> float:
        """获取参数值（带默认值）
        
        Args:
            key: 参数键
            fallback: 默认值
            
        Returns:
            参数值
        """
        return self.parameters.get(key, fallback)
    
    def filter_components(self, components: List[np.ndarray]) -> List[np.ndarray]:
        """筛选连通域
        
        Args:
            components: 输入连通域列表
            
        Returns:
            筛选后的连通域列表
        """
        if not components:
            return []
        
        # 获取筛选参数
        min_area = self._get_param('component_min_area', 1000.0)
        max_area = self._get_param('component_max_area', 1000000.0)
        min_aspect = self._get_param('component_min_aspect_ratio', 0.3)
        max_aspect = self._get_param('component_max_aspect_ratio', 4.0)
        min_width = self._get_param('component_min_width', 60.0)
        min_height = self._get_param('component_min_height', 60.0)
        
        filtered = []
        for component_mask in components:
            # 计算面积
            area = cv2.countNonZero(component_mask)
            
            # 面积筛选
            if area < min_area or area > max_area:
                continue
            
            # 获取外接矩形
            contours, _ = cv2.findContours(
                component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            
            x, y, w, h = cv2.boundingRect(contours[0])
            
            # 尺寸筛选
            if w < min_width or h < min_height:
                continue
            
            # 长宽比筛选
            aspect_ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0.0
            if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
                continue
            
            filtered.append(component_mask)
        
        return filtered
    
    def _extract_workpiece_circle(
        self, 
        mask: np.ndarray,
        combine_contours: bool = True,
        min_area: int = 100
    ) -> Tuple[Optional[Tuple[float, float]], float]:
        """提取工件外接圆（大圆，参考trigger_depth.py的_extract_workpiece_circle）
        
        Args:
            mask: 连通域掩码
            combine_contours: 是否合并多个轮廓
            min_area: 最小轮廓面积
            
        Returns:
            (圆心坐标, 半径) 或 (None, 0.0)
        """
        if mask is None or mask.size == 0:
            return (None, 0.0)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return (None, 0.0)
        
        # 合并轮廓或选择最大轮廓
        if combine_contours and len(contours) > 1:
            # 过滤小轮廓
            filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
            
            if filtered_contours:
                # 合并所有符合条件的轮廓点
                all_points = np.vstack(filtered_contours)
                contours = [all_points]
            else:
                # 如果没有符合条件的，选择最大的
                largest_contour = max(contours, key=cv2.contourArea)
                contours = [largest_contour]
        else:
            # 只保留面积大于min_area的轮廓
            filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
            
            if filtered_contours:
                contours = filtered_contours
            else:
                # 如果都不符合，选择最大的
                all_contours = [c for c in contours if cv2.contourArea(c) > 0]
                if all_contours:
                    largest_contour = max(all_contours, key=cv2.contourArea)
                    contours = [largest_contour]
                else:
                    contours = []
        
        if not contours:
            return (None, 0.0)
        
        # 计算最小外接圆
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        
        center = (float(x), float(y))
        radius_float = float(radius)
        
        return (center, radius_float)
    
    def _extract_valve_circle(
        self, 
        mask: np.ndarray
    ) -> Tuple[Optional[Tuple[float, float]], float, Optional[np.ndarray]]:
        """提取阀体外接圆（小圆，参考trigger_depth.py的_extract_valve_circle）
        
        Args:
            mask: 连通域掩码
            
        Returns:
            (圆心坐标, 半径, 处理后的掩码) 或 (None, 0.0, None)
        """
        if mask is None or mask.size == 0:
            return (None, 0.0, None)
        
        original_area = cv2.countNonZero(mask)
        if original_area == 0:
            return (None, 0.0, None)
        
        # 获取参数
        erode_kernel_size = int(self._get_param('small_circle_erode_kernel', 11.0))
        erode_iterations = int(self._get_param('small_circle_erode_iterations', 1.0))
        dilate_kernel_size = int(self._get_param('small_circle_dilate_kernel', 9.0))
        dilate_iterations = int(self._get_param('small_circle_dilate_iterations', 1.0))
        largest_cc_only = bool(self._get_param('small_circle_largest_cc', 1.0))
        
        # 确保核大小为奇数
        if erode_kernel_size % 2 == 0:
            erode_kernel_size += 1
        if dilate_kernel_size % 2 == 0:
            dilate_kernel_size += 1
        
        # 腐蚀操作
        kernel_erode = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (erode_kernel_size, erode_kernel_size)
        )
        eroded = cv2.erode(mask, kernel_erode, iterations=erode_iterations)
        
        eroded_area = cv2.countNonZero(eroded)
        
        # 如果腐蚀后面积为0，尝试降低腐蚀强度
        if eroded_area == 0:
            self.logger.warning(
                f'腐蚀后掩码面积为0，尝试降低腐蚀强度 '
                f'(原始参数: 核={erode_kernel_size}, 迭代={erode_iterations})'
            )
            
            # 尝试减少迭代次数
            for reduced_iterations in range(erode_iterations - 1, 0, -1):
                eroded = cv2.erode(mask, kernel_erode, iterations=reduced_iterations)
                eroded_area = cv2.countNonZero(eroded)
                if eroded_area > 0:
                    # 降低腐蚀迭代次数成功
                    break
            
            # 如果还是0，尝试减小核大小
            if eroded_area == 0:
                for reduced_kernel in range(erode_kernel_size - 2, 3, -2):
                    if reduced_kernel < 3:
                        break
                    kernel_erode_reduced = cv2.getStructuringElement(
                        cv2.MORPH_ELLIPSE, (reduced_kernel, reduced_kernel)
                    )
                    eroded = cv2.erode(mask, kernel_erode_reduced, iterations=1)
                    eroded_area = cv2.countNonZero(eroded)
                    if eroded_area > 0:
                        # 降低腐蚀核大小成功
                        break
            
            # 如果仍然为0，返回失败
            if eroded_area == 0:
                self.logger.warning(
                    f'阀体外接圆提取失败: 即使降低腐蚀强度后掩码面积仍为0 (原始面积={original_area})'
                )
                return (None, 0.0, eroded)
        
        processed = eroded.copy()
        
        # 只保留最大连通域
        if largest_cc_only:
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                processed, connectivity=8
            )
            
            if num_labels > 1:
                # 找到面积最大的连通域（跳过背景标签0）
                max_area = 0
                max_label = 1
                for i in range(1, num_labels):
                    area = stats[i, cv2.CC_STAT_AREA]
                    if area > max_area:
                        max_area = area
                        max_label = i
                
                # 创建只包含最大连通域的掩码
                largest_cc = np.zeros_like(processed, dtype=np.uint8)
                largest_cc[labels == max_label] = 255
                processed = largest_cc
            else:
                self.logger.warning(f'阀体外接圆提取失败: 腐蚀后没有连通域 (标签数: {num_labels})')
                return (None, 0.0, processed)
        
        # 膨胀操作
        if dilate_iterations > 0 and dilate_kernel_size > 0:
            kernel_dilate = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (dilate_kernel_size, dilate_kernel_size)
            )
            processed = cv2.dilate(processed, kernel_dilate, iterations=dilate_iterations)
        
        dilated_area = cv2.countNonZero(processed)
        
        if dilated_area == 0:
            self.logger.warning('阀体外接圆提取失败: 膨胀后掩码面积为0')
            return (None, 0.0, processed)
        
        # 提取外接圆
        center, radius = self._extract_workpiece_circle(processed, False, 0)
        
        if center is None:
            contours, _ = cv2.findContours(
                processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if len(contours) == 0:
                self.logger.warning(f'阀体外接圆提取失败: 膨胀后没有轮廓 (面积: {dilated_area})')
            else:
                contour_areas = [cv2.contourArea(c) for c in contours]
                self.logger.warning(
                    f'阀体外接圆提取失败: 有{len(contours)}个轮廓但无法提取外接圆 '
                    f'(轮廓面积: {[f"{a:.0f}" for a in contour_areas]})'
                )
        
        return (center, radius, processed)
    
    def _calculate_standardized_angle(
        self,
        workpiece_center: Tuple[float, float],
        valve_center: Tuple[float, float]
    ) -> float:
        """计算标准化角度（参考trigger_depth.py的_calculate_standardized_angle）
        
        从工件中心到阀体中心的角度
        
        Args:
            workpiece_center: 工件外接圆圆心
            valve_center: 阀体外接圆圆心
            
        Returns:
            旋转角度（弧度）
        """
        if workpiece_center is None or valve_center is None:
            return 0.0
        
        dx = valve_center[0] - workpiece_center[0]
        dy = valve_center[1] - workpiece_center[1]
        angle = np.arctan2(dy, dx)
        
        return float(angle)
    
    def _extract_single_component_feature(
        self, 
        component_mask: np.ndarray
    ) -> Optional[ComponentFeature]:
        """提取单个连通域的特征
        
        Args:
            component_mask: 连通域二值掩码
            
        Returns:
            特征结构体 或 None（提取失败）
        """
        if component_mask is None or component_mask.size == 0:
            self.logger.warning('连通域掩码为空')
            return None
        
        # 计算工件面积
        workpiece_area = float(cv2.countNonZero(component_mask))
        
        # 提取工件外接圆（大圆）
        combine_contours = bool(self._get_param('big_circle_combine_contours', 1.0))
        min_area = int(self._get_param('big_circle_min_area', 100.0))
        
        workpiece_center, workpiece_radius = self._extract_workpiece_circle(
            component_mask, combine_contours, min_area
        )
        
        if workpiece_center is None:
            self.logger.warning('工件外接圆提取失败')
            return None
        
        # 提取阀体外接圆（小圆）
        valve_center, valve_radius, valve_mask = self._extract_valve_circle(component_mask)
        
        valve_area = 0.0
        if valve_mask is not None:
            valve_area = float(cv2.countNonZero(valve_mask))
        
        if valve_center is None:
            self.logger.warning('阀体外接圆提取失败')
            # 不返回None，而是使用默认值
            valve_center = workpiece_center
            valve_radius = 0.0
        
        # 计算标准化角度
        standardized_angle = self._calculate_standardized_angle(workpiece_center, valve_center)
        standardized_angle_deg = float(np.degrees(standardized_angle))
        
        # 创建特征结构体
        feature = ComponentFeature(
            workpiece_center=workpiece_center,
            workpiece_radius=workpiece_radius,
            workpiece_area=workpiece_area,
            valve_center=valve_center,
            valve_radius=valve_radius,
            valve_area=valve_area,
            standardized_angle=standardized_angle,
            standardized_angle_deg=standardized_angle_deg,
            component_mask=component_mask
        )
        
        return feature
    
    def extract_features(
        self, 
        components: List[np.ndarray],
        color_image: Optional[np.ndarray] = None
    ) -> List[ComponentFeature]:
        """提取连通域特征（主函数）
        
        处理流程：
        1. 初步筛选干扰连通域
        2. 根据筛选后的连通域数量启用相应数量的线程
        3. 并行处理每个连通域，提取特征
        4. 如果提供彩色图，在特征中保存彩色图像引用
        5. 返回所有连通域的特征列表
        
        Args:
            components: 连通域二值图像列表
            color_image: 彩色工件图像（可选，用于可视化）
            
        Returns:
            特征列表（每个连通域对应一个特征结构体）
        """
        if not components:
            self.logger.warning('输入连通域列表为空')
            return []
        
        # 获取特征提取参数
        min_area = self._get_param('component_min_area', 1000.0)
        max_area = self._get_param('component_max_area', 1000000.0)
        min_aspect = self._get_param('component_min_aspect_ratio', 0.3)
        max_aspect = self._get_param('component_max_aspect_ratio', 4.0)
        min_width = self._get_param('component_min_width', 60.0)
        min_height = self._get_param('component_min_height', 60.0)
        big_circle_combine = bool(self._get_param('big_circle_combine_contours', 1.0))
        big_circle_min_area = self._get_param('big_circle_min_area', 100.0)
        small_circle_erode_kernel = int(self._get_param('small_circle_erode_kernel', 11.0))
        small_circle_erode_iter = int(self._get_param('small_circle_erode_iterations', 1.0))
        small_circle_largest_cc = bool(self._get_param('small_circle_largest_cc', 1.0))
        small_circle_dilate_kernel = int(self._get_param('small_circle_dilate_kernel', 9.0))
        small_circle_dilate_iter = int(self._get_param('small_circle_dilate_iterations', 1.0))
        max_threads = int(self._get_param('max_threads', 4.0))
        
        # 1. 筛选连通域
        filtered_components = self.filter_components(components)
        
        if not filtered_components:
            self.logger.warning('筛选后连通域列表为空')
            return []
        
        # 2. 多线程处理
        num_threads = min(max_threads, len(filtered_components))
        
        features = []
        
        with ThreadPoolExecutor(max_workers=num_threads) as executor:
            # 提交所有任务
            future_to_idx = {
                executor.submit(self._extract_single_component_feature, comp): idx
                for idx, comp in enumerate(filtered_components)
            }
            
            # 收集结果
            for future in as_completed(future_to_idx):
                idx = future_to_idx[future]
                try:
                    feature = future.result()
                    if feature is not None:
                        # 如果提供了彩色图，保存引用
                        if color_image is not None:
                            feature.color_image = color_image
                        with self.result_mutex:
                            features.append(feature)
                except Exception as e:
                    self.logger.error(f'处理连通域 {idx} 时出错: {e}')
        
        self.logger.info(f'特征提取完成: {len(features)} 个特征')
        return features
