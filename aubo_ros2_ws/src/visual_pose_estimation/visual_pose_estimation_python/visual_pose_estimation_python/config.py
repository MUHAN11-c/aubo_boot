"""
统一配置模块

从 default_config.yaml 加载所有算法参数，各模块通过 get_section() 获取自己关心的配置段。
替代原来的 config_reader.py 硬编码默认值 + debug_thresholds.json 分散配置。

使用方式:
    from .config import get_config
    cfg = get_config()
    pp_params = cfg.preprocessor  # dict
    fe_params = cfg.feature_extractor  # dict
"""

from __future__ import annotations

import logging
import os
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

    @property
    def depth_scale(self) -> float:
        return float(self.camera.get("depth_scale", 0.00025))

    @property
    def depth_search_radius(self) -> int:
        return int(self.pose_estimator.get("depth_search_radius", 3))


def load_config(yaml_path: Optional[str] = None) -> AppConfig:
    """从 YAML 文件加载配置。

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
    """获取当前配置单例（若未加载则自动加载）。"""
    global _config_instance
    if _config_instance is None:
        _config_instance = load_config()
    return _config_instance


def update_section(section: str, params: Dict[str, Any]) -> None:
    """运行时更新某个配置段（用于参数更新服务 / debug 面板）。

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
