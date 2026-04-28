"""
参数管理模块

负责 debug_thresholds.json 的加载、保存和更新操作。
与 ROS2 参数系统解耦，可独立运行。
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional


class ParamsManager:
    """参数管理器：内存参数 + JSON 文件持久化。"""

    def __init__(self, config_path: Optional[str] = None):
        self._logger = logging.getLogger(__name__)
        self._config_path = Path(config_path) if config_path else None
        self._params: Dict[str, Any] = {}

    @property
    def config_path(self) -> Optional[Path]:
        return self._config_path

    @config_path.setter
    def config_path(self, path: Optional[str]):
        self._config_path = Path(path) if path else None

    def load(self, config_path: Optional[str] = None) -> Dict[str, Any]:
        """从 JSON 文件加载参数。

        Args:
            config_path: JSON 文件路径，None 则使用实例默认路径

        Returns:
            参数字典
        """
        path = Path(config_path) if config_path else self._config_path
        if path is None or not path.exists():
            self._logger.warning(f"配置文件不存在: {path}")
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                self._params = json.load(f)
            self._logger.info(f"已加载参数: {path}")
            return dict(self._params)
        except Exception as e:
            self._logger.warning(f"加载参数失败: {path}, 错误: {e}")
            return {}

    def save(self, config_path: Optional[str] = None) -> bool:
        """保存参数到 JSON 文件。

        Args:
            config_path: JSON 文件路径，None 则使用实例默认路径

        Returns:
            是否保存成功
        """
        path = Path(config_path) if config_path else self._config_path
        if path is None:
            self._logger.error("未指定保存路径")
            return False
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._params, f, indent=2, ensure_ascii=False)
            self._logger.info(f"已保存参数至: {path}")
            return True
        except Exception as e:
            self._logger.error(f"保存参数失败: {path}, 错误: {e}")
            return False

    def update(self, key: str, value: Any) -> bool:
        """更新单个参数值。

        Args:
            key: 参数键名
            value: 参数值

        Returns:
            是否更新成功
        """
        self._params[key] = value
        return True

    def update_batch(self, params: Dict[str, Any]) -> bool:
        """批量更新参数。

        Args:
            params: 参数键值对

        Returns:
            是否更新成功
        """
        self._params.update(params)
        return True

    def get(self, key: str, default: Any = None) -> Any:
        """获取单个参数值。

        Args:
            key: 参数键名
            default: 默认值

        Returns:
            参数值或默认值
        """
        return self._params.get(key, default)

    def get_all(self) -> Dict[str, Any]:
        """获取所有参数的副本。

        Returns:
            参数字典的浅拷贝
        """
        return dict(self._params)
