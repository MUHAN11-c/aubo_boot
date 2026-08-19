"""
peach_common_py — 桃子采摘链路公共纯核库（重构阶段 A1）.

纯核零 ROS import（peach_common_py.ros 子包除外，AST 测试强制）。公共 API
汇总导出；ROS 适配器请显式 ``from peach_common_py.ros.clock_adapter import
RclpyClockAdapter``，避免纯核 import 链触碰 rclpy。
"""
from peach_common_py.bounded_worker import BoundedWorker
from peach_common_py.clock import Clock, ManualClock
from peach_common_py.depth_geometry import normalize_depth_to_uint16_mm
from peach_common_py.harvest_data import default_harvest_root, HarvestDataStore
from peach_common_py.registry import Registry
from peach_common_py.tf_utils import (
    gravity_camera_from_R,
    invert_transform,
    QuaternionValue,
    relative_motion,
    rotation_to_quat,
    transform_direction,
    transform_msg_to_matrix,
    transform_point,
)

__all__ = [
    'BoundedWorker',
    'Clock',
    'HarvestDataStore',
    'ManualClock',
    'QuaternionValue',
    'Registry',
    'default_harvest_root',
    'gravity_camera_from_R',
    'invert_transform',
    'normalize_depth_to_uint16_mm',
    'relative_motion',
    'rotation_to_quat',
    'transform_direction',
    'transform_msg_to_matrix',
    'transform_point',
]
