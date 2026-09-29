# -*- coding: utf-8 -*-
"""
位姿合成段：反投影 + 模板抓取位姿合成（几何不变，单位单源）.

实际的位姿求解实现就是 PoseEstimator.estimate_pose（纯核，无 rclpy）；
本模块只收口「深度原始值 → 米」的单位常量：Percipio 深度原始值
× DEPTH_SCALE_DEFAULT = 米（0.25 mm/LSB）。深度换算只允许经
resolve_depth_scale(camera_cfg) 出来的值，禁止散落硬编码。
"""

from __future__ import annotations

from typing import Any, Dict

# Percipio 深度量化步距（米/LSB）：raw × 0.00025 = 米
DEPTH_SCALE_DEFAULT = 0.00025


def resolve_depth_scale(camera_cfg: Dict[str, Any]) -> float:
    """从配置 camera 段解析 depth_scale；缺省用 Percipio 默认值."""
    try:
        value = float((camera_cfg or {}).get('depth_scale', DEPTH_SCALE_DEFAULT))
    except (TypeError, ValueError):
        return DEPTH_SCALE_DEFAULT
    if value <= 0:
        return DEPTH_SCALE_DEFAULT
    return value
