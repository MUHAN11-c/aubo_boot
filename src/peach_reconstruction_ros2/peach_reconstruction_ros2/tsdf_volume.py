"""
Open3D TSDF 封装 — finalize 时批量积分已采帧（无 ROS 依赖，懒加载 open3d）.

外参方向约定（最易错点，由 test_tsdf_volume.py 守门）：
  ROS 侧每帧存 ``T_base_camera``（camera→base，p_base = T @ p_camera）；
  Open3D ``integrate`` 的 extrinsic 要 world→camera，即
  ``T_camera_base = inv(T_base_camera)``。本模块变量一律带方向后缀。

颜色约定：CapturedFrame.rgb 为 OpenCV BGR uint8，Open3D 要 RGB——积分前
通道反转；提取云的 o3d 颜色为 [0,1] float RGB，导出时转回 uint8 BGR
（与 cloud_builder / pack_rgb_bgr 的发布约定衔接）。

深度约定：CapturedFrame.depth_mm 为 uint16 毫米，积分前转 float32 米制
（depth_scale=1.0）；0 与超 depth_trunc 的深度不参与积分。
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np

from peach_reconstruction_ros2.tf_utils import invert_transform

_O3D = None  # 懒加载缓存（无 open3d 的环境仍可 import 本模块）


def _require_open3d():
    """返回 open3d 模块；缺失时抛带指引的 RuntimeError."""
    global _O3D
    if _O3D is None:
        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError(
                'open3d 不可用：TSDF 功能须在工作区 venv（aubo_py3.12）'
                '解释器下运行') from exc
        _O3D = o3d
    return _O3D


class LocalTsdf:
    """局部 TSDF 体积：批量积分 → 提取 → ROI 裁剪 → 降采样 → 离群剔除."""

    def __init__(self, voxel_length: float = 0.003, sdf_trunc: float = 0.012,
                 depth_trunc: float = 1.5):
        """
        建 TSDF 体积（参数单位均 [m]）.

        Args:
            voxel_length: 体素边长 [m].
            sdf_trunc: 截断距离 [m].
            depth_trunc: 深度截断 [m]（更远的深度不积分）.

        Returns
        -------
            无返回值（None）；integrate_time_s 累计积分墙钟 [s].

        """
        o3d = _require_open3d()
        self.voxel_length = float(voxel_length)
        self.sdf_trunc = float(sdf_trunc)
        self.depth_trunc = float(depth_trunc)
        self._volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        self.integrate_time_s = 0.0

    def _make_rgbd(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                   camera_K: dict):
        """
        组 Open3D RGBDImage 与内参（BGR→RGB、uint16[mm]→float32[m]）.

        Args:
            rgb_bgr: (H, W, 3) uint8 BGR.
            depth_mm: (H, W) uint16 深度 [mm]（0 与超 depth_trunc 不积分）.
            camera_K: 内参 dict {"fx","fy","cx","cy"}.

        Returns
        -------
            (rgbd, intrinsic)：Open3D 对象对.

        """
        o3d = _require_open3d()
        h, w = depth_mm.shape[:2]
        # BGR → RGB（Open3D 颜色通道序）
        color = o3d.geometry.Image(
            np.ascontiguousarray(rgb_bgr[:, :, ::-1]))
        depth_m = np.ascontiguousarray(
            depth_mm.astype(np.float32) / 1000.0)  # [mm] → [m]
        depth = o3d.geometry.Image(depth_m)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=1.0,  # 深度已是米制，scale=1
            depth_trunc=self.depth_trunc, convert_rgb_to_intensity=False)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=int(w), height=int(h),
            fx=float(camera_K['fx']), fy=float(camera_K['fy']),
            cx=float(camera_K['cx']), cy=float(camera_K['cy']))
        return rgbd, intrinsic

    def _integrate(self, rgbd, intrinsic,
                   extrinsic_camera_base: np.ndarray) -> None:
        """
        底层积分入口：直接给 world→camera 外参（方向由调用方负责）.

        Args:
            rgbd: Open3D RGBDImage.
            intrinsic: Open3D PinholeCameraIntrinsic.
            extrinsic_camera_base: (4, 4) world(base)→camera 外参.

        Returns
        -------
            无返回值（None）；耗时累计进 integrate_time_s.

        """
        t0 = time.perf_counter()
        self._volume.integrate(rgbd, intrinsic,
                               np.asarray(extrinsic_camera_base,
                                          dtype=np.float64))
        self.integrate_time_s += time.perf_counter() - t0

    def integrate_frame(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                        camera_K: dict, T_base_camera: np.ndarray) -> None:
        """
        积分一帧：BGR 彩图 + uint16 毫米深度 + base←camera 位姿.

        Args:
            rgb_bgr: (H, W, 3) uint8 BGR（CapturedFrame.rgb）.
            depth_mm: (H, W) uint16 深度 [mm].
            camera_K: 内参 dict {"fx","fy","cx","cy"}.
            T_base_camera: (4, 4) camera→base；内部取逆得 T_camera_base
                （world→camera）传给 Open3D——方向反了云会整体错位.

        Returns
        -------
            无返回值（None）；耗时累计进 integrate_time_s.

        """
        rgbd, intrinsic = self._make_rgbd(rgb_bgr, depth_mm, camera_K)
        # 外参方向：ROS 存 camera→base，Open3D 要 base(world)→camera
        T_camera_base = invert_transform(
            np.asarray(T_base_camera, dtype=np.float64))
        self._integrate(rgbd, intrinsic, T_camera_base)

    def extract_cloud(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        提取 TSDF 点云.

        Returns
        -------
            (xyz, colors_bgr)：xyz 为 (N, 3) float64 [m]（base 系）；
            colors_bgr 为 (N, 3) uint8 BGR（无颜色时给 None）.

        """
        pcd = self._volume.extract_point_cloud()
        xyz = np.asarray(pcd.points, dtype=np.float64)
        colors = None
        if pcd.has_colors():
            rgb01 = np.asarray(pcd.colors, dtype=np.float64)  # [0,1] RGB
            colors = np.clip(np.round(rgb01[:, ::-1] * 255.0),
                             0, 255).astype(np.uint8)  # → BGR uint8
        return xyz, colors

    @staticmethod
    def crop_to_box(xyz: np.ndarray, colors: Optional[np.ndarray],
                    center, size_xyz) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        轴对齐盒裁剪（base 系 ROI）.

        Args:
            xyz: (N, 3) 点 [m].
            colors: (N, 3) 颜色或 None（与 xyz 同步过滤）.
            center: (3,) 盒中心 [m].
            size_xyz: (3,) 盒尺寸 [m]（local_volume.size_x/y/z）.

        Returns
        -------
            (xyz_in, colors_in)：盒内点与颜色；空云给 (0, 3) 空数组.

        """
        if xyz.size == 0:
            return xyz.reshape(0, 3), colors
        c = np.asarray(center, dtype=np.float64)
        half = np.asarray(size_xyz, dtype=np.float64) / 2.0
        mask = (np.abs(xyz - c) <= half).all(axis=1)
        colors_in = colors[mask] if colors is not None else None
        return xyz[mask], colors_in

    @staticmethod
    def voxel_downsample(xyz: np.ndarray, colors: Optional[np.ndarray],
                         voxel_size: float
                         ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        体素降采样（voxel_size ≤ 0 或空云时原样返回）.

        Args:
            xyz: (N, 3) 点 [m].
            colors: (N, 3) uint8 BGR 或 None.
            voxel_size: 体素边长 [m].

        Returns
        -------
            (xyz_down, colors_down).

        """
        if voxel_size <= 0.0 or xyz.size == 0:
            return xyz, colors
        o3d = _require_open3d()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if colors is not None and len(colors) == len(xyz):
            pcd.colors = o3d.utility.Vector3dVector(
                colors.astype(np.float64)[:, ::-1] / 255.0)  # BGR→RGB [0,1]
        down = pcd.voxel_down_sample(float(voxel_size))
        xyz_d = np.asarray(down.points, dtype=np.float64)
        colors_d = None
        if down.has_colors():
            colors_d = np.clip(np.round(
                np.asarray(down.colors)[:, ::-1] * 255.0), 0, 255).astype(np.uint8)
        return xyz_d, colors_d

    @staticmethod
    def statistical_filter(xyz: np.ndarray, colors: Optional[np.ndarray],
                           nb_neighbors: int = 20, std_ratio: float = 2.0
                           ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        统计离群剔除（open3d remove_statistical_outlier 常用默认：20 邻域 2σ）.

        点数不足 nb_neighbors+1 时原样返回（小云无可剔除意义）。

        Args:
            xyz: (N, 3) 点 [m].
            colors: (N, 3) uint8 BGR 或 None.
            nb_neighbors: 邻域点数.
            std_ratio: 标准差倍率阈值.

        Returns
        -------
            (xyz_in, colors_in)：内点与颜色.

        """
        if xyz.shape[0] <= nb_neighbors:
            return xyz, colors
        o3d = _require_open3d()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        _inlier_pcd, inlier_idx = pcd.remove_statistical_outlier(
            nb_neighbors=int(nb_neighbors), std_ratio=float(std_ratio))
        idx = np.asarray(inlier_idx, dtype=int)
        colors_in = colors[idx] if colors is not None else None
        return xyz[idx], colors_in
