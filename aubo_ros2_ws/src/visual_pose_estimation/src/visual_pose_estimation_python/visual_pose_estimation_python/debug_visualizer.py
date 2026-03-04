"""
Debug 可视化模块
用于生成调试图像，帮助观察算法处理过程
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from .preprocessor import Preprocessor


class DebugVisualizer:
    """调试可视化器，生成各种调试图像"""
    
    def __init__(self):
        """初始化调试可视化器"""
        self.preprocessor = Preprocessor()
    
    def create_depth_display(
        self, 
        depth_image: np.ndarray,
        components: Optional[List[np.ndarray]] = None
    ) -> np.ndarray:
        """创建深度图显示（彩色映射）
        
        Args:
            depth_image: 深度图（uint16）
            components: 连通域列表（可选，用于标记）
            
        Returns:
            彩色映射的深度显示图
        """
        if depth_image is None or depth_image.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 使用 preprocessor 处理深度图（0值插值），参考 preprocessor.py (377-380)
        processed_depth, stats = self.preprocessor.process_depth_image(depth_image, enable_zero_interp=True)
        if stats.get('filled_zeros', 0) > 0:
            # 可选：记录填充的0值点数量（如果需要日志）
            pass
        
        # 归一化到 uint8（使用处理后的深度图）
        valid_mask = (processed_depth > 0) & (processed_depth < 65535)
        if np.any(valid_mask):
            min_val = np.min(processed_depth[valid_mask])
            max_val = np.max(processed_depth[valid_mask])
            
            depth_norm = np.zeros_like(processed_depth, dtype=np.uint8)
            if max_val > min_val:
                depth_norm[valid_mask] = np.clip(
                    ((processed_depth[valid_mask] - min_val) * 255.0 / (max_val - min_val)).astype(np.uint8),
                    0, 255
                )
        else:
            depth_norm = np.zeros_like(processed_depth, dtype=np.uint8)
        
        # 应用颜色映射
        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        
        # 标记无效值为黑色（使用处理后的深度图）
        invalid_mask = (processed_depth == 0) | (processed_depth == 65535)
        depth_colored[invalid_mask] = [0, 0, 0]
        
        # 筛选前不绘制轮廓，仅显示深度图
        # （components 仍保留参数以兼容调用方，但不用于绘制）
        
        return depth_colored
    
    def create_binary_display(
        self,
        depth_image: np.ndarray,
        binary_threshold_min: int,
        binary_threshold_max: int,
        components: Optional[List[np.ndarray]] = None,
        features: Optional[List[Dict]] = None
    ) -> np.ndarray:
        """创建二值图显示：使用去毛边后的连通域合并图（单独创建图像）
        
        当 components 非空时，用处理后的连通域合并成一张二值图显示；
        否则回退为阈值二值图。
        
        Args:
            depth_image: 深度图（用于尺寸或回退二值化）
            binary_threshold_min: 最小阈值（无连通域时回退用）
            binary_threshold_max: 最大阈值（无连通域时回退用）
            components: 去毛边后的连通域列表（用于生成显示图）
            features: 特征列表（用于绘制与彩色图一致的边界框）
            
        Returns:
            二值图显示（BGR）
        """
        h, w = depth_image.shape[:2]
        if components and len(components) > 0:
            # 用去毛边后的连通域单独创建二值图
            binary_display = np.zeros((h, w), dtype=np.uint8)
            for comp in components:
                if comp.shape[:2] == (h, w):
                    binary_display[comp > 0] = 255
            binary_display = cv2.cvtColor(binary_display, cv2.COLOR_GRAY2BGR)
        else:
            # 无连通域时回退为阈值二值图
            binary_display = self.preprocessor.create_binary_image(
                depth_image,
                binary_threshold_min,
                binary_threshold_max
            )
            binary_display = cv2.cvtColor(binary_display, cv2.COLOR_GRAY2BGR)
        
        if features:
            for feat in features:
                wp_center = feat.get('workpiece_center')
                wp_radius = feat.get('workpiece_radius')
                if wp_center and wp_radius:
                    x = int(wp_center[0] - wp_radius)
                    y = int(wp_center[1] - wp_radius)
                    wb = int(wp_radius * 2)
                    hb = int(wp_radius * 2)
                    cv2.rectangle(binary_display, (x, y), (x + wb, y + hb), (0, 255, 255), 2)
        
        return binary_display
    
    def draw_features_overlay(
        self,
        image: np.ndarray,
        features: List[Dict],
        draw_bbox: bool = True
    ) -> np.ndarray:
        """在图像上绘制特征覆盖层
        
        Args:
            image: 输入图像（BGR）
            features: 特征列表，每个特征包含：
                - workpiece_center: 工件中心 (x, y)
                - workpiece_radius: 工件半径
                - valve_center: 阀体中心 (x, y)
                - valve_radius: 阀体半径
                - angle_deg: 角度（度）
            draw_bbox: 是否绘制边界框
            
        Returns:
            带特征标注的图像
        """
        if image is None:
            return None
        
        overlay = image.copy()
        
        for feat in features:
            # 绘制工件外接圆（绿色）
            wp_center = feat.get('workpiece_center')
            wp_radius = feat.get('workpiece_radius')
            if wp_center and wp_radius:
                center_x, center_y = int(wp_center[0]), int(wp_center[1])
                radius = int(wp_radius)
                cv2.circle(overlay, (center_x, center_y), radius, (0, 255, 0), 2)
                cv2.circle(overlay, (center_x, center_y), 3, (0, 255, 0), -1)
                
                # 标注工件半径
                cv2.putText(overlay, f"WP: R={radius}", 
                          (center_x + radius + 5, center_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            # 绘制阀体外接圆（蓝色）
            valve_center = feat.get('valve_center')
            valve_radius = feat.get('valve_radius')
            if valve_center and valve_radius:
                center_x, center_y = int(valve_center[0]), int(valve_center[1])
                radius = int(valve_radius)
                cv2.circle(overlay, (center_x, center_y), radius, (255, 0, 0), 2)
                cv2.circle(overlay, (center_x, center_y), 3, (255, 0, 0), -1)
                
                # 标注阀体半径
                cv2.putText(overlay, f"Valve: R={radius}", 
                          (center_x + radius + 5, center_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                
                # 绘制连线和角度
                if wp_center:
                    wp_x, wp_y = int(wp_center[0]), int(wp_center[1])
                    cv2.line(overlay, (wp_x, wp_y), (center_x, center_y), (0, 255, 255), 2)
                    
                    # 标注角度
                    angle = feat.get('angle_deg', 0)
                    mid_x = (wp_x + center_x) // 2
                    mid_y = (wp_y + center_y) // 2
                    cv2.putText(overlay, f"{angle:.1f}°", 
                              (mid_x + 10, mid_y),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # 绘制边界框（如果需要）
            if draw_bbox and wp_center:
                # 计算包含工件的边界框
                x = int(wp_center[0] - wp_radius)
                y = int(wp_center[1] - wp_radius)
                w = int(wp_radius * 2)
                h = int(wp_radius * 2)
                cv2.rectangle(overlay, (x, y), (x+w, y+h), (0, 255, 255), 2)
                cv2.putText(overlay, f"Bbox: {w}x{h}", 
                          (x, max(y-10, 15)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return overlay
    
    def create_debug_panel(
        self,
        depth_image: np.ndarray,
        color_image: np.ndarray,
        components: List[np.ndarray],
        features: List[Dict],
        binary_threshold_min: int,
        binary_threshold_max: int,
        preprocessed_color: Optional[np.ndarray] = None
    ) -> Dict[str, np.ndarray]:
        """创建完整的调试面板（4张图像）
        
        Args:
            depth_image: 深度图
            color_image: 彩色图
            components: 连通域列表
            features: 特征列表
            binary_threshold_min: 最小阈值
            binary_threshold_max: 最大阈值
            preprocessed_color: 预处理后的彩色图（可选）
            
        Returns:
            包含4张调试图像的字典：
            - depth_display: 深度图显示
            - color_display: 彩色图显示（带特征）
            - binary_display: 二值图显示
            - preprocessed_display: 预处理图显示（带特征）
        """
        # 1. 深度图显示
        depth_display = self.create_depth_display(depth_image, components)
        
        # 2. 彩色图显示（带特征）
        color_display = self.draw_features_overlay(
            color_image.copy(), 
            features, 
            draw_bbox=True
        )
        
        # 3. 二值图显示（使用features绘制边界框，与彩色图保持一致）
        binary_display = self.create_binary_display(
            depth_image,
            binary_threshold_min,
            binary_threshold_max,
            components,
            features  # 传入features以使用相同的坐标系统
        )
        
        # 4. 预处理图显示（如果有）
        if preprocessed_color is not None:
            preprocessed_display = self.draw_features_overlay(
                preprocessed_color.copy(),
                features,
                draw_bbox=False
            )
        else:
            preprocessed_display = None
        
        return {
            'depth_display': depth_display,
            'color_display': color_display,
            'binary_display': binary_display,
            'preprocessed_display': preprocessed_display
        }
