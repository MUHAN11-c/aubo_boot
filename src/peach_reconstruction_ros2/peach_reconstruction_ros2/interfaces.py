"""
抽象接口层 — 纯核契约（abc.ABC，对标 nav2_core 形态）.

设计文档 docs/superpowers/specs/2026-08-10-peach-layered-architecture.md
§2.2：ABC 只约束 workhorse 纯数据方法（numpy/纯 dataclass 进 →
numpy/dict 出，无副作用、不碰 ROS）；本模块不 import 实现（防循环），
实现发现用显式注册表字典（yolo_ros 先例），注册表随实现放在实现模块：
  - frame_collector.FRAME_STORES = {'default': FrameCollector}
  - cloud_builder.CLOUD_BUILDERS = {'open3d': Open3dCloudBuilder}
  - tsdf_volume.VOLUME_FUSIONS = {'open3d_scalable': LocalTsdf}
  - geometry_refiner.GEOMETRY_REFINERS = {'ransac': RansacGeometryRefiner}

数据成员契约（start/reset 之外的状态面）写在各 ABC docstring，
由实现继承 docstring 语义，不以 abstractmethod 强制（实例属性无法
满足 ABC 抽象成员，强转 property 会无谓放大 diff）。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np


class FrameStore(ABC):
    """
    批级帧栈数据持有者契约（对齐 frame_collector.FrameCollector）.

    数据成员契约（实现以实例属性提供）：``frames``（CapturedFrame 列表）、
    ``state``（IDLE/COLLECTING/READY/FAILED）、``target_id``、
    ``target_center``、``rejected_views``、``tf_failures``、
    ``last_rel_translation_m``、``last_rel_rotation_deg``。
    """

    @abstractmethod
    def start(self, target_id: str = '', target_center=None) -> str:
        """清空旧帧进入 COLLECTING 并绑定目标；返回状态说明字符串."""

    @abstractmethod
    def add_frame(self, frame) -> bool:
        """压栈一帧；满 max_views 拒收（False）."""

    @abstractmethod
    def remove_last(self):
        """弹出最后一帧；空栈返回 None."""

    @abstractmethod
    def reset(self) -> None:
        """清空全部帧/绑定/计数，状态回 IDLE."""

    @abstractmethod
    def finalize(self) -> Tuple[bool, str, Optional[np.ndarray]]:
        """结束采集：视角数达标拼接累加云并转 READY（ok, message, cloud）."""


class CloudBuilder(ABC):
    """单帧深度 → base 系点云契约（对齐 cloud_builder.build_cloud_base）."""

    @abstractmethod
    def build(self, depth_mm: np.ndarray, rgb_bgr: Optional[np.ndarray],
              camera_K: dict, T_base_camera: np.ndarray,
              stride: int = 1, target_mask=None
              ) -> Tuple[np.ndarray, Optional[np.ndarray], float]:
        """
        一帧深度反投影并变到 base 系.

        Args:
            depth_mm: (H, W) uint16 深度 [mm].
            rgb_bgr: (H, W, 3) uint8 BGR；None 只建几何.
            camera_K: 内参 dict {"fx","fy","cx","cy"}.
            T_base_camera: (4, 4) base←camera 位姿.
            stride: 降采样步长（像素）.

        Returns
        -------
            (xyz_base, colors_bgr|None, valid_depth_ratio)：
            (N, 3) [m] 点云、(N, 3) uint8 BGR 逐点颜色、有效深度占比.

        """


class VolumeFusion(ABC):
    """
    多帧 RGB-D 融合体积契约（对齐 tsdf_volume.LocalTsdf）.

    显式轻量生命周期：``reset()`` 清空体积复用实例（对标 nav2 生命周期
    四件套的轻量版；现有实现也可直接弃例新建，语义一致）。
    """

    @abstractmethod
    def integrate_frame(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                        camera_K: dict, T_base_camera: np.ndarray) -> None:
        """积分一帧（BGR 彩图 + uint16 毫米深度 + base←camera 位姿）."""

    @abstractmethod
    def extract_cloud(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """提取融合点云：(xyz (N, 3) [m], colors_bgr|None)."""

    @abstractmethod
    def reset(self) -> None:
        """清空体积与累计耗时，实例可复用."""


class GeometryRefiner(ABC):
    """TSDF 云几何二次拟合契约（对齐 geometry_refiner.refine_geometry）."""

    @abstractmethod
    def refine(self, cloud_xyz: np.ndarray, target_kind: str,
               config=None, axis_hint=None) -> dict:
        """
        圆柱/球 RANSAC 精化 + bottom→neck 消歧 + ACCEPT/REOBSERVE 门控.

        Args:
            cloud_xyz: (N, 3) 点 [m]（base_frame）.
            target_kind: 'bag'/'fruit'（'fruit' 以外按袋桃圆柱线）.
            config: geometry_refiner.RefitConfig；None 用默认.
            axis_hint: 可选 bottom→neck 方向先验；球体无内禀轴时使用.

        Returns
        -------
            dict（RefitResult 形态，键集见
            geometry_refiner.refine_geometry）：ok/reason/kind/status/
            n_points/center/axis/axis_point/bottom/neck/entry/radius/
            diameter/span_m/rmse/inlier_ratio/flags；失败 ok=False 不抛异常.

        """
