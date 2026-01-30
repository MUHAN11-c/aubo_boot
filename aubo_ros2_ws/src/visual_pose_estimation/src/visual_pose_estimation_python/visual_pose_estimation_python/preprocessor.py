#!/usr/bin/env python3
"""
深度图像预处理模块

功能（参考trigger_depth.py）：
1. 深度图0值插值处理（补齐法）
2. 图像缩放（根据缩放因子）
3. 深度阈值二值化
4. 连通域提取
5. 恢复原始尺寸
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
import logging


class Preprocessor:
    """深度图像预处理器类（参考trigger_depth.py实现）"""
    
    def __init__(self):
        """初始化预处理器"""
        self.logger = logging.getLogger(__name__)
        self.parameters = {}
        self._initialize_default_parameters()
    
    def _initialize_default_parameters(self):
        """初始化默认参数（适配深度图处理）"""
        # 深度图二值化参数（参考trigger_depth.py）
        self.parameters['binary_threshold_min'] = 0.0        # 最小深度阈值（原始值）
        self.parameters['binary_threshold_max'] = 65535.0    # 最大深度阈值（原始值）
        self.parameters['enable_zero_interp'] = 1.0          # 是否启用0值插值
        
        # 连通域筛选参数
        self.parameters['component_min_area'] = 1000.0
        self.parameters['component_max_area'] = 1000000.0
        self.parameters['component_min_aspect_ratio'] = 0.3
        self.parameters['component_max_aspect_ratio'] = 4.0
        self.parameters['component_min_width'] = 60.0
        self.parameters['component_min_height'] = 60.0
        self.parameters['component_max_count'] = 10.0
    
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
    
    def process_depth_image(
        self, 
        depth_image: np.ndarray, 
        enable_zero_interp: bool = True
    ) -> Tuple[np.ndarray, Dict]:
        """深度图处理函数，0值处理（补齐法）
        
        参考trigger_depth.py的_process_depth_image实现
        
        Args:
            depth_image: 输入的深度图像 (numpy array)
            enable_zero_interp: 是否启用0值插值处理（默认True）
        
        Returns:
            processed_image: 处理后的深度图像
            stats: 处理统计信息字典，包含处理的点数等信息
        """
        if depth_image is None or not isinstance(depth_image, np.ndarray):
            return depth_image, {}
        
        stats = {'filled_zeros': 0}
        
        try:
            if len(depth_image.shape) == 3:
                depth_data = depth_image[:, :, 0].copy()
            else:
                depth_data = depth_image.copy()
            
            original_shape = depth_data.shape
            original_dtype = depth_data.dtype
            
            if enable_zero_interp:
                depth_data, filled_zeros = self._interp_data(depth_data)
                stats['filled_zeros'] = filled_zeros
                assert depth_data.shape == original_shape
            
            if depth_data.shape != original_shape:
                depth_data = cv2.resize(depth_data, (original_shape[1], original_shape[0]), 
                                       interpolation=cv2.INTER_NEAREST)
            
            if original_dtype != depth_data.dtype:
                depth_data = depth_data.astype(original_dtype)
            
            return depth_data, stats
            
        except Exception as e:
            self.logger.error(f'深度图处理失败: {e}')
            return depth_image, stats
    
    def _interp_data(self, depth_image: np.ndarray) -> Tuple[np.ndarray, int]:
        """0值处理（补齐法）
        
        参考trigger_depth.py的_interp_data实现
        针对列或行中出现的个别深度数据为0的点，采用补齐法进行处理
        注意：不处理整行整列的0值，只处理个别0值点
        """
        if depth_image.size == 0:
            return depth_image, 0
        
        new_data = depth_image.copy()
        zero_mask = (new_data == 0)
        fn = np.sum(zero_mask)
        
        if fn == 0:
            return new_data, 0
        
        # 检测整行整列为0的情况
        row_means = np.mean(new_data, axis=1)
        invalid_rows = row_means == 0  # 整行为0的行
        
        col_means = np.mean(new_data, axis=0)
        invalid_cols = col_means == 0  # 整列为0的列
        
        try:
            # 处理行的个别0值点（跳过整行为0的行）
            for i in range(new_data.shape[0]):
                if invalid_rows[i]:
                    continue
                
                row = new_data[i, :]
                zero_indices = np.where(row == 0)[0]
                
                if len(zero_indices) == 0:
                    continue
                
                for idx in zero_indices:
                    if invalid_cols[idx]:
                        continue
                    
                    if idx == 0:
                        non_zero_idx = np.where(row[idx+1:] > 0)[0]
                        if len(non_zero_idx) > 0:
                            new_data[i, idx] = row[idx + 1 + non_zero_idx[0]]
                        elif i > 0:
                            new_data[i, idx] = new_data[i - 1, idx]
                    else:
                        new_data[i, idx] = new_data[i, idx - 1]
            
            # 处理列的个别0值点（跳过整列为0的列）
            remaining_zeros = (new_data == 0)
            if np.any(remaining_zeros):
                for j in range(new_data.shape[1]):
                    if invalid_cols[j]:
                        continue
                    
                    col = new_data[:, j]
                    zero_indices = np.where(col == 0)[0]
                    
                    if len(zero_indices) == 0:
                        continue
                    
                    for idx in zero_indices:
                        if invalid_rows[idx]:
                            continue
                        
                        if idx == 0:
                            non_zero_idx = np.where(col[idx+1:] > 0)[0]
                            if len(non_zero_idx) > 0:
                                new_data[idx, j] = col[idx + 1 + non_zero_idx[0]]
                            elif j > 0:
                                new_data[idx, j] = new_data[idx, j - 1]
                        else:
                            new_data[idx, j] = new_data[idx - 1, j]
            
            return new_data, fn
            
        except Exception as e:
            self.logger.error(f'插值处理失败: {e}')
            return new_data, fn
    
    def create_binary_image(
        self, 
        depth_image: np.ndarray,
        binary_threshold_min: int,
        binary_threshold_max: int
    ) -> np.ndarray:
        """创建二值化图像（参考trigger_depth.py的_create_binary_image）
        
        Args:
            depth_image: 深度图像
            binary_threshold_min: 最小深度阈值
            binary_threshold_max: 最大深度阈值
            
        Returns:
            二值化图像
        """
        # 获取无效深度值掩码
        invalid_mask = self._get_invalid_mask(depth_image)
        
        valid_mask = ~invalid_mask
        if not np.any(valid_mask):
            self.logger.warning(
                f'二值化处理: 没有有效深度值，阈值=[{binary_threshold_min}, {binary_threshold_max}]'
            )
        
        # 创建二值掩码：深度值在阈值范围内的为前景
        binary_mask = (depth_image >= binary_threshold_min) & (depth_image <= binary_threshold_max)
        binary_mask = binary_mask & ~invalid_mask
        
        # 创建二值图像
        binary_image = np.zeros_like(depth_image, dtype=np.uint8)
        binary_image[binary_mask] = 255
        binary_image[invalid_mask] = 0  # 无效深度值直接设为0（黑色）
        
        return binary_image
    
    def _get_invalid_mask(self, depth_image: np.ndarray) -> np.ndarray:
        """获取无效深度值掩码"""
        if depth_image.dtype == np.uint16:
            return (depth_image == 0) | (depth_image == 65535)
        elif depth_image.dtype in (np.float32, np.float64):
            return (depth_image == 0) | np.isnan(depth_image)
        else:
            return depth_image == 0
    
    def _extract_connected_components(self, binary_mask: np.ndarray) -> List[np.ndarray]:
        """提取连通域
        
        Args:
            binary_mask: 二值掩码图像
            
        Returns:
            连通域掩码列表
        """
        if binary_mask is None or binary_mask.size == 0:
            return []
        
        # 确保是二值图像
        if binary_mask.dtype != np.uint8:
            binary_mask = binary_mask.astype(np.uint8)
        
        # 提取连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary_mask, connectivity=8
        )
        
        components = []
        for i in range(1, num_labels):  # 跳过背景（标签0）
            # 创建单个连通域掩码
            component_mask = np.zeros_like(binary_mask, dtype=np.uint8)
            component_mask[labels == i] = 255
            components.append(component_mask)
        
        return components
    
    def _filter_components(self, components: List[np.ndarray]) -> List[np.ndarray]:
        """根据面积、长宽比等条件筛选连通域
        
        Args:
            components: 连通域列表
            
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
        max_count = int(self._get_param('component_max_count', 10.0))
        
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
        
        # 限制数量（保留面积最大的）
        if len(filtered) > max_count:
            # 按面积排序
            areas = [cv2.countNonZero(mask) for mask in filtered]
            sorted_indices = sorted(range(len(areas)), key=lambda i: areas[i], reverse=True)
            filtered = [filtered[i] for i in sorted_indices[:max_count]]
        
        return filtered
    
    
    def preprocess(
        self, 
        depth_image: np.ndarray,
        color_image: Optional[np.ndarray] = None,
        binary_threshold_min: Optional[int] = None,
        binary_threshold_max: Optional[int] = None
    ) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
        """预处理深度图像：二值化并提取连通域，可选地提取彩色工件图像
        
        处理流程（参考trigger_depth.py的display_preprocessed_image）：
        1. 深度图0值插值处理
        2. 基于深度阈值进行二值化
        3. 提取连通域
        4. 筛选连通域
        5. 如果提供彩色图，用掩模抠出彩色工件图像
        6. 返回连通域二值图像列表和彩色工件图像
        
        Args:
            depth_image: 输入深度图像（uint16或float格式）
            color_image: 输入彩色图像（BGR格式，可选）
            binary_threshold_min: 最小深度阈值（如果为None，使用参数中的值）
            binary_threshold_max: 最大深度阈值（如果为None，使用参数中的值）
            
        Returns:
            (连通域二值图像列表, 彩色工件图像或None)
        """
        if depth_image is None or depth_image.size == 0:
            self.logger.error('输入深度图像为空')
            return [], None
        
        # 获取阈值参数
        if binary_threshold_min is None:
            binary_threshold_min = int(self._get_param('binary_threshold_min', 0.0))
        if binary_threshold_max is None:
            binary_threshold_max = int(self._get_param('binary_threshold_max', 65535.0))
        
        enable_zero_interp = bool(self._get_param('enable_zero_interp', 1.0))
        
        # 获取连通域筛选参数
        min_area = self._get_param('component_min_area', 1000.0)
        max_area = self._get_param('component_max_area', 1000000.0)
        min_aspect = self._get_param('component_min_aspect_ratio', 0.3)
        max_aspect = self._get_param('component_max_aspect_ratio', 4.0)
        min_width = self._get_param('component_min_width', 60.0)
        min_height = self._get_param('component_min_height', 60.0)
        max_count = int(self._get_param('component_max_count', 10.0))
        
        # 1. 深度图处理（0值插值）
        processed_depth, stats = self.process_depth_image(depth_image, enable_zero_interp)
        
        # 2. 创建二值图像（使用处理后的深度图）
        binary_image = self.create_binary_image(
            processed_depth, 
            binary_threshold_min, 
            binary_threshold_max
        )
        
        # 3. 提取连通域
        components = self._extract_connected_components(binary_image)
        
        # 4. 筛选连通域
        filtered_components = self._filter_components(components)
        
        # 5. 如果提供了彩色图，使用掩模提取彩色工件图像
        preprocessed_color = None
        if color_image is not None:
            preprocessed_color = self.extract_color_workpiece(
                color_image,
                filtered_components,
                processed_depth,
                binary_threshold_min,
                binary_threshold_max
            )
        
        return filtered_components, preprocessed_color
    
    def extract_color_workpiece(
        self,
        color_image: np.ndarray,
        component_masks: List[np.ndarray],
        depth_image: np.ndarray,
        binary_threshold_min: int,
        binary_threshold_max: int
    ) -> np.ndarray:
        """从彩色图中提取工件区域（参考trigger_depth.py的display_preprocessed_image）
        
        Args:
            color_image: 彩色图像
            component_masks: 连通域掩模列表
            depth_image: 深度图像
            binary_threshold_min: 最小深度阈值（不参与掩模）
            binary_threshold_max: 最大深度阈值（不参与掩模）
            
        Returns:
            预处理后的彩色工件图像（白色背景）
        """
        img_h, img_w = color_image.shape[:2]
        
        # 如果没有检测到工件，返回原图
        if not component_masks:
            return color_image.copy()
        
        # 仅使用最大连通域作为掩模（component_masks 已按面积排序）
        component_mask = component_masks[0]
        
        # 创建白色背景的预处理图像
        preprocessed_image = np.full((img_h, img_w, 3), (255, 255, 255), dtype=np.uint8)
        
        # 使用最大连通域作为掩模（可选膨胀，平滑边缘）
        workpiece_mask = component_mask.astype(bool)
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        workpiece_mask_dilated = cv2.dilate(workpiece_mask.astype(np.uint8), kernel_dilate, iterations=1).astype(bool)
        
        # 使用掩码从彩色图中抠出工件
        preprocessed_image[workpiece_mask_dilated] = color_image[workpiece_mask_dilated]
        
        return preprocessed_image
