"""
统一配置模块.

从 default_config.yaml 加载所有算法参数，各模块通过 get_section() 获取自己关心的配置段。

使用方式:
    from .config import get_config
    cfg = get_config()
    pp_params = cfg.preprocessor  # dict
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from .path_resolver import resolve_web_paths

_logger = logging.getLogger(__name__)

# 模块级单例
_config_instance: Optional[AppConfig] = None


@dataclass
class AppConfig:
    """应用统一配置——YAML 文件的直接映射"""

    camera: Dict[str, Any] = field(default_factory=dict)
    ros2: Dict[str, Any] = field(default_factory=dict)
    preprocessor: Dict[str, Any] = field(default_factory=dict)
    feature_extractor: Dict[str, Any] = field(default_factory=dict)
    pose_estimator: Dict[str, Any] = field(default_factory=dict)
    template: Dict[str, Any] = field(default_factory=dict)
    rembg: Dict[str, Any] = field(default_factory=dict)
    debug: Dict[str, Any] = field(default_factory=dict)

    # 手眼标定（由 ROS2 节点运行时填入）
    calib_file: str = ""
    camera_matrix: Any = None
    dist_coeffs: Any = None
    T_E_C: Any = None


class ConfigReader:
    """配置读取器：预处理/特征阈值等算法参数的统一读取入口（配置事实源为 YAML）."""

    def __init__(self):
        self._cfg = get_config()

    def get_section(self, section: str) -> Dict[str, Any]:
        """获取配置段（返回副本，避免意外修改）."""
        val = getattr(self._cfg, section, {})
        if isinstance(val, dict):
            return dict(val)
        return {}

    def load_debug_thresholds(self) -> Dict[str, Any]:
        """返回预处理+连通域阈值参数（含 rembg 开关），取自 default_config.yaml."""
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


def load_config(yaml_path: Optional[str] = None) -> AppConfig:
    """
    从 YAML 文件加载配置.

    Args:
        yaml_path: YAML 文件路径，None 则使用包内 default_config.yaml

    Returns:
        AppConfig 实例
    """
    global _config_instance

    if yaml_path is None:
        web_paths = resolve_web_paths()
        yaml_path = str(web_paths.configs_dir / "default_config.yaml")
        if not Path(yaml_path).exists():
            # 回退：尝试包源码目录
            this_dir = Path(__file__).resolve().parent
            alt = this_dir / "web_ui" / "configs" / "default_config.yaml"
            if alt.exists():
                yaml_path = str(alt)

    cfg = _load_yaml_file(yaml_path)
    _config_instance = AppConfig(
        camera=cfg.get("camera", {}),
        ros2=cfg.get("ros2", {}),
        preprocessor=cfg.get("preprocessor", {}),
        feature_extractor=cfg.get("feature_extractor", {}),
        pose_estimator=cfg.get("pose_estimator", {}),
        template=cfg.get("template", {}),
        rembg=cfg.get("rembg", {}),
        debug=cfg.get("debug", {}),
    )
    _logger.info(f"配置已加载: {yaml_path}")
    return _config_instance


def get_config() -> AppConfig:
    """获取当前配置单例（若未加载则自动加载）."""
    global _config_instance
    if _config_instance is None:
        _config_instance = load_config()
    return _config_instance


def update_section(section: str, params: Dict[str, Any]) -> None:
    """
    运行时更新某个配置段（用于参数更新服务 / debug 面板）.

    Args:
        section: 段名（如 'preprocessor', 'feature_extractor'）
        params: 要合并的参数字典
    """
    cfg = get_config()
    target = getattr(cfg, section, None)
    if isinstance(target, dict):
        target.update(params)
    else:
        _logger.warning(f"未知的配置段: {section}")


def _load_yaml_file(path: str) -> Dict[str, Any]:
    if not Path(path).exists():
        _logger.warning(f"配置文件不存在: {path}，使用空配置")
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        _logger.error(f"加载 YAML 配置失败: {path}, 错误: {e}")
        return {}
