#!/usr/bin/env python3
"""
配置读取模块

功能：
1. 提供所有配置参数的默认值
2. 加载Debug阈值参数（从 debug_thresholds.json）

默认路径：web_ui/configs/debug_thresholds.json（包内相对路径），可在代码中通过
debug_thresholds_path 或 load_debug_thresholds(debug_thresholds_file) 配置。
"""

import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional

# 默认阈值配置文件路径：web_ui/configs/debug_thresholds.json（包根上一级为 visual_pose_estimation_python 包目录）
_PKG_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DEBUG_THRESHOLDS_PATH = _PKG_ROOT / "web_ui" / "configs" / "debug_thresholds.json"
FALLBACK_DEBUG_THRESHOLDS_PATH = Path(
    "/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/configs/debug_thresholds.json"
)


class ConfigReader:
    """配置读取器类
    
    负责提供所有配置参数的默认值，并加载 debug_thresholds.json 中的阈值参数。
    默认从 web_ui/configs/debug_thresholds.json 读取，可通过 debug_thresholds_path 或
    load_debug_thresholds(debug_thresholds_file) 配置路径。
    """
    
    def __init__(self, debug_thresholds_path: Optional[str] = None):
        """初始化配置读取器
        
        Args:
            debug_thresholds_path: 可选，Debug 阈值配置文件路径（默认使用 web_ui/configs/debug_thresholds.json）
        """
        self.logger = logging.getLogger(__name__)
        self._debug_thresholds_path = Path(debug_thresholds_path) if debug_thresholds_path else None
        self._initialize_default_config()
    
    def _initialize_default_config(self):
        """初始化默认配置
        
        所有需要调整的参数都在这里定义默认值。
        阈值参数（binary_threshold_*, component_*等）不在此处定义，
        统一从 debug_thresholds.json 通过 load_debug_thresholds() 获取。
        """
        self.config = {
            # 预处理参数（非阈值参数）
            'preprocessor': {
                'scale_factor': 1.0,  # 图像缩放因子 (0.1-1.0)
            },
            
            # 特征提取参数（非阈值参数）
            'feature_extractor': {
                'big_circle_combine_contours': 1.0,  # 是否合并多个轮廓 (bool: 1.0=True, 0.0=False)
                'big_circle_min_area': 100.0,  # 大圆最小轮廓面积（像素²）
                'small_circle_erode_kernel': 11.0,  # 小圆腐蚀核大小（奇数）
                'small_circle_erode_iterations': 1.0,  # 小圆腐蚀迭代次数
                'small_circle_largest_cc': 1.0,  # 是否只保留最大连通域 (bool: 1.0=True, 0.0=False)
                'small_circle_dilate_kernel': 9.0,  # 小圆膨胀核大小（奇数）
                'small_circle_dilate_iterations': 1.0,  # 小圆膨胀迭代次数
                'max_threads': 4.0  # 最大线程数
            },
            
            # 姿态估计参数
            'pose_estimator': {
                'brute_force_matching_enabled': True,  # 是否启用暴力匹配模式
                'brute_force_angle_step_deg': 1.0,  # 角度步进（度）
                'brute_force_max_threads': 18,  # 最大线程数
                'brute_force_rejection_threshold': 0.2,  # 舍弃阈值 (0.0-1.0)
                'brute_force_acceptance_threshold': 0.7,  # 接受阈值 (0.0-1.0)
                'brute_force_angle_matching_scale': 0.8  # 角度匹配时缩小尺寸的比例 (0.0-1.0)
            },
            
            # 其他参数（template_root/calib_file 通常由 launch/ROS 参数传入；未指定时从 web_ui/configs 或标准路径查找）
            'debug': False,  # 调试模式开关
            'template_root': '',  # 模板根目录，空则节点内用包相对路径或 launch 默认值
            'calib_file': ''  # 手眼标定文件，空则从 web_ui/configs 或 hand_eye_calibration 标准路径查找
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """获取配置值
        
        支持嵌套键，如 'preprocessor.scale_factor'
        
        Args:
            key: 配置键
            default: 默认值
            
        Returns:
            配置值
        """
        try:
            keys = key.split('.')
            value = self.config
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default
    
    def get_section(self, section: str) -> Dict[str, Any]:
        """获取配置段
        
        Args:
            section: 段名称（'preprocessor', 'feature_extractor', 'pose_estimator'）
            
        Returns:
            配置字典
        """
        return self.config.get(section, {}).copy()
    
    def load_debug_thresholds(self, debug_thresholds_file: Optional[str] = None) -> Dict[str, Any]:
        """加载Debug阈值参数（从debug_thresholds.json）
        
        统一从config_reader获取所有阈值参数。
        
        Args:
            debug_thresholds_file: 可选，显式指定debug_thresholds.json路径
            
        Returns:
            Debug阈值参数字典，包含：
            - binary_threshold_min, binary_threshold_max: 二值化最小/最大阈值
            - component_min_area, component_max_area: 连通域最小/最大面积（像素）
            - component_min_aspect_ratio, component_max_aspect_ratio: 连通域最小/最大宽高比
            - component_min_width, component_min_height: 连通域最小宽度/高度（像素）
            - component_max_count: 连通域最大数量
            - enable_zero_interp: 是否启用零值插值
        """
        # 默认阈值参数
        default_params = {
            'binary_threshold_min': 1818,
            'binary_threshold_max': 2045,
            'component_min_area': 0,
            'component_max_area': 1546,
            'component_min_aspect_ratio': 0.3,
            'component_max_aspect_ratio': 8.9,
            'component_min_width': 32,
            'component_min_height': 47,
            'component_max_count': 1,
            'enable_zero_interp': True,
            'enable_smooth_edges': True,
            'smooth_edges_blur_sigma': 0,
            'use_rembg': False
        }
        
        # 确定要加载的文件路径：显式参数 > 实例配置 > 默认路径(web_ui/configs) > 备用绝对路径
        if debug_thresholds_file:
            config_path = Path(debug_thresholds_file)
        elif self._debug_thresholds_path is not None:
            config_path = self._debug_thresholds_path
        else:
            config_path = DEFAULT_DEBUG_THRESHOLDS_PATH if DEFAULT_DEBUG_THRESHOLDS_PATH.exists() else FALLBACK_DEBUG_THRESHOLDS_PATH
        
        # 加载配置文件
        if config_path and config_path.exists():
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    params = json.load(f)
                
                # 补齐缺失默认值
                for k, v in default_params.items():
                    if k not in params:
                        params[k] = v
                
                self.logger.info(f'✓ 已加载Debug阈值: {config_path}')
                return params
            except Exception as e:
                self.logger.warning(f'⚠ 加载Debug阈值文件失败: {config_path}, 错误: {e}')
        
        # 如果文件不存在或加载失败，使用默认值
        if config_path and not config_path.exists():
            self.logger.warning(f'⚠ Debug阈值文件不存在: {config_path}，使用默认阈值')
        else:
            self.logger.warning('⚠ 未找到Debug阈值文件，使用默认阈值')
        
        return default_params
