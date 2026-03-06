"""参数管理模块 - 统一的参数和路径配置中心

默认路径：web_ui/configs（本文件在 web_ui/scripts 下，ROOT_DIR=web_ui，CONFIG_DIR=web_ui/configs）。
阈值文件默认：web_ui/configs/debug_thresholds.json。可在 __init__(config_path=...) 中配置路径。
"""

import json
from pathlib import Path
from typing import Dict, Optional


class ParamsManager:
    """统一的参数管理器 - 唯一配置源
    
    统一管理阈值的保存和读取：
    - 默认路径：web_ui/configs/debug_thresholds.json
    - 可在 __init__(config_path=...) 中配置路径
    - 使用内存缓存减少文件IO，自动同步到文件
    """
    
    # 默认路径：web_ui/configs（本文件在 web_ui/scripts，parent.parent = web_ui）
    ROOT_DIR = Path(__file__).resolve().parent.parent
    CONFIG_DIR = ROOT_DIR / 'configs'
    PARAMS_FILE = CONFIG_DIR / 'debug_thresholds.json'
    
    def __init__(self, config_path: Optional[str] = None):
        """初始化参数管理器
        
        Args:
            config_path: 可选，自定义阈值配置文件路径；默认使用 web_ui/configs/debug_thresholds.json
        """
        if config_path:
            self.config_path = Path(config_path)
        else:
            self.config_path = self.PARAMS_FILE
        
        # 默认参数
        self.defaults = {
            'binary_threshold_min': 0,
            'binary_threshold_max': 2149,
            'component_min_area': 10,
            'component_max_area': 100000,
            'component_min_aspect_ratio': 0.3,
            'component_max_aspect_ratio': 4.0,
            'component_min_width': 60,
            'component_min_height': 60,
            'component_max_count': 3,
            'enable_zero_interp': True,
            'enable_smooth_edges': True,
            'smooth_edges_blur_sigma': 0,
            'use_rembg': False
        }
        
        # 内存缓存（避免频繁文件读写）
        self._cache = None
        self._cache_dirty = False  # 标记缓存是否已修改但未保存
    
    def load(self, force_reload: bool = False) -> Dict:
        """加载参数（统一读取接口）
        
        Args:
            force_reload: 强制从文件重新加载（忽略缓存）
            
        Returns:
            参数字典
        """
        # 如果缓存存在且未修改，直接返回缓存
        if not force_reload and self._cache is not None and not self._cache_dirty:
            return self._cache.copy()
        
        try:
            if self.config_path.exists():
                with open(self.config_path, 'r', encoding='utf-8') as f:
                    params = json.load(f)
            else:
                params = {}
            
            # 填充缺失的默认值
            for key, value in self.defaults.items():
                if key not in params:
                    params[key] = value
            
            # 更新缓存
            self._cache = params.copy()
            self._cache_dirty = False
            return params
        except Exception as e:
            # 加载失败时返回默认值
            self._cache = self.defaults.copy()
            self._cache_dirty = False
            return self.defaults.copy()
    
    def save(self, params: Dict = None) -> bool:
        """保存参数（统一保存接口）
        
        Args:
            params: 要保存的参数（如果为None，保存当前缓存）
            
        Returns:
            是否保存成功
        """
        try:
            # 如果未提供参数，使用缓存
            if params is None:
                if self._cache is None:
                    params = self.load()
                else:
                    params = self._cache.copy()
            
            # 确保所有默认参数都存在
            for key, value in self.defaults.items():
                if key not in params:
                    params[key] = value
            
            # 创建目录（如果不存在）
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            
            # 保存到文件
            with open(self.config_path, 'w', encoding='utf-8') as f:
                json.dump(params, f, indent=2, ensure_ascii=False)
            
            # 更新缓存
            self._cache = params.copy()
            self._cache_dirty = False
            return True
        except Exception:
            return False
    
    def update(self, key: str, value) -> bool:
        """更新单个参数（统一更新接口）
        
        Args:
            key: 参数名
            value: 参数值
            
        Returns:
            是否更新成功
        """
        # 加载当前参数（使用缓存）
        params = self.load()
        
        # 更新参数
        params[key] = value
        
        # 更新缓存
        self._cache = params.copy()
        self._cache_dirty = True
        
        # 立即保存到文件
        return self.save(params)
    
    def update_batch(self, updates: Dict) -> bool:
        """批量更新参数
        
        Args:
            updates: 参数字典 {key: value}
            
        Returns:
            是否更新成功
        """
        # 加载当前参数
        params = self.load()
        
        # 批量更新
        params.update(updates)
        
        # 更新缓存
        self._cache = params.copy()
        self._cache_dirty = True
        
        # 立即保存到文件
        return self.save(params)
    
    def reload(self) -> Dict:
        """强制从文件重新加载参数
        
        Returns:
            参数字典
        """
        return self.load(force_reload=True)
    
    def get_preprocessor_params(self) -> Dict:
        """获取Preprocessor参数（统一参数接口）
        
        所有算法模块都应该使用这些阈值参数
        
        Returns:
            Preprocessor参数字典
        """
        params = self.load()
        return {
            'binary_threshold_min': params.get('binary_threshold_min', self.defaults['binary_threshold_min']),
            'binary_threshold_max': params.get('binary_threshold_max', self.defaults['binary_threshold_max']),
            'component_min_area': params.get('component_min_area', self.defaults['component_min_area']),
            'component_max_area': params.get('component_max_area', self.defaults['component_max_area']),
            'component_min_aspect_ratio': params.get('component_min_aspect_ratio', self.defaults['component_min_aspect_ratio']),
            'component_max_aspect_ratio': params.get('component_max_aspect_ratio', self.defaults['component_max_aspect_ratio']),
            'component_min_width': params.get('component_min_width', self.defaults['component_min_width']),
            'component_min_height': params.get('component_min_height', self.defaults['component_min_height']),
            'component_max_count': params.get('component_max_count', self.defaults['component_max_count']),
            'enable_zero_interp': params.get('enable_zero_interp', self.defaults['enable_zero_interp']),
            'enable_smooth_edges': params.get('enable_smooth_edges', self.defaults['enable_smooth_edges']),
            'smooth_edges_blur_sigma': params.get('smooth_edges_blur_sigma', self.defaults['smooth_edges_blur_sigma'])
        }
    
    def get_feature_extractor_params(self) -> Dict:
        """获取FeatureExtractor参数（统一参数接口）
        
        FeatureExtractor也需要使用连通域筛选相关的阈值参数
        
        Returns:
            FeatureExtractor参数字典
        """
        params = self.load()
        return {
            'component_min_area': params.get('component_min_area', self.defaults['component_min_area']),
            'component_max_area': params.get('component_max_area', self.defaults['component_max_area']),
            'component_min_aspect_ratio': params.get('component_min_aspect_ratio', self.defaults['component_min_aspect_ratio']),
            'component_max_aspect_ratio': params.get('component_max_aspect_ratio', self.defaults['component_max_aspect_ratio']),
            'component_min_width': params.get('component_min_width', self.defaults['component_min_width']),
            'component_min_height': params.get('component_min_height', self.defaults['component_min_height']),
        }
