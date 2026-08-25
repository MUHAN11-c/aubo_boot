"""
CapturedFrame — 单帧采集结果的数据合约（纯数据结构，不依赖 ROS）.

单位约定：深度 uint16 [mm]；点云 / 位置 [m]；stamp 为浮点秒 [s]。
一帧 = 相机系原始数据（rgb/depth/K/stamp）+ base 系几何（T/位置/点云）+
诊断（有效深度占比、ICP/FK 选择与拒帧原因）。
"""
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class CapturedFrame:
    """一次成功采帧的全部内容（原始观测 + base 系几何 + 诊断标记）."""

    rgb: np.ndarray                      # (H, W, 3) uint8 BGR（OpenCV 惯例）
    depth_mm: np.ndarray                 # (H, W) uint16 深度 [mm]
    camera_K: dict                       # {"fx","fy","cx","cy","width","height"}
    stamp: float                         # 图像时间戳 [s]（按 depth.header.stamp）
    T_base_camera: np.ndarray            # (4, 4) base←camera 齐次矩阵
    T_base_camera_fk: Optional[np.ndarray] = None  # ICP 前的机器人 FK 位姿
    target_id: str = ''                  # 绑定的候选目标 ID（无候选时为空串）
    valid_depth_ratio: float = 0.0       # 有效深度占比 [0, 1]
    camera_position_base: Optional[np.ndarray] = None  # (3,) 相机位置 [m]
    cloud_base: Optional[np.ndarray] = None            # (N, 3) base 系点云 [m]
    cloud_rgb: Optional[np.ndarray] = None             # (N, 3) uint8 BGR（OpenCV 排列）
    diagnostic_flags: List[str] = field(default_factory=list)  # 如 'pose_icp'
    registration: dict = field(default_factory=dict)  # ICP fitness/RMSE/修正量

    def __post_init__(self):
        """补派生默认值：FK 缺省等于使用位姿，相机位置取使用位姿平移列."""
        if self.T_base_camera_fk is None:
            self.T_base_camera_fk = np.asarray(
                self.T_base_camera, dtype=np.float64).copy()
        if self.camera_position_base is None:
            self.camera_position_base = np.asarray(
                self.T_base_camera, dtype=np.float64)[:3, 3].copy()
