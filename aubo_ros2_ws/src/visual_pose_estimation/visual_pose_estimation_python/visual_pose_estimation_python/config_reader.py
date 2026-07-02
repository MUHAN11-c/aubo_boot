#!/usr/bin/env python3
"""
配置读取模块（兼容层）

委托给 config.py 统一 YAML 配置系统。
保留原有接口以兼容现有代码。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from .config import get_config, load_config, update_section


class ConfigReader:
    """配置读取器（兼容层：委托 config.py 统一配置）"""

    def __init__(self, debug_thresholds_path: Optional[str] = None):
        self.logger = logging.getLogger(__name__)
        self._cfg = get_config()
        if debug_thresholds_path:
            self.logger.info(f"debug_thresholds 已不再使用独立 JSON，统一在 YAML")

    # ------------------------------------------------------------------
    # 原有接口（兼容）
    # ------------------------------------------------------------------
    def get(self, key: str, default: Any = None) -> Any:
        """支持嵌套键，如 'preprocessor.scale_factor'"""
        try:
            keys = key.split(".")
            value = self._cfg
            for k in keys:
                if isinstance(value, dict):
                    value = value.get(k, default)
                else:
                    value = getattr(value, k, default)
            return value if value is not None else default
        except (KeyError, TypeError, AttributeError):
            return default

    def get_section(self, section: str) -> Dict[str, Any]:
        """获取配置段（返回副本，避免意外修改）。"""
        val = getattr(self._cfg, section, {})
        if isinstance(val, dict):
            return dict(val)
        return {}

    def load_debug_thresholds(
        self, debug_thresholds_file: Optional[str] = None
    ) -> Dict[str, Any]:
        """兼容原有 debug_thresholds.json 接口，返回预处理+连通域参数。

        参数统一从 default_config.yaml 的 preprocessor 段获取。
        """
        pp = self._cfg.preprocessor
        return {
            "binary_threshold_min": pp.get("binary_threshold_min", 1818),
            "binary_threshold_max": pp.get("binary_threshold_max", 2045),
            "component_min_area": pp.get("component_min_area", 0),
            "component_max_area": pp.get("component_max_area", 1546),
            "component_min_aspect_ratio": pp.get("component_min_aspect_ratio", 0.3),
            "component_max_aspect_ratio": pp.get("component_max_aspect_ratio", 8.9),
            "component_min_width": pp.get("component_min_width", 32),
            "component_min_height": pp.get("component_min_height", 47),
            "component_max_count": pp.get("component_max_count", 1),
            "enable_zero_interp": pp.get("enable_zero_interp", True),
            "enable_smooth_edges": pp.get("enable_smooth_edges", True),
            "smooth_edges_blur_sigma": pp.get("smooth_edges_blur_sigma", 0),
            "use_rembg": self._cfg.rembg.get("enabled", False),
        }
