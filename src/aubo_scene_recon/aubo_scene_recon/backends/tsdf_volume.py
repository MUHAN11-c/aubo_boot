"""Phase 2：Open3D ScalableTSDFVolume（RGB-D 融合）。"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import open3d as o3d

from aubo_scene_recon.backends.base import FusionBackend


def _height_colormap(xyz: np.ndarray) -> np.ndarray:
    """按 z 伪彩（蓝→青→绿→黄→红），保证无真彩时 RViz 仍可见。"""
    z = xyz[:, 2]
    zmin, zmax = float(np.min(z)), float(np.max(z))
    t = np.zeros(len(z), dtype=np.float64) if zmax <= zmin else (z - zmin) / (zmax - zmin)
    # 简易 jet 分段
    r = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0)
    return np.stack([r, g, b], axis=1)


class TsdfBackend(FusionBackend):
    """彩色+深度 TSDF；点云路径仍可用 integrate() 退化成投影较弱，推荐 integrate_rgbd。"""

    def __init__(
        self,
        voxel_size: float = 0.005,
        sdf_trunc: float = 0.04,
        max_map_points: int = 2_000_000,
        depth_scale: float = 1000.0,
        depth_max: float = 1.5,
    ):
        self.voxel_size = float(voxel_size)
        self.sdf_trunc = float(sdf_trunc)
        self.max_map_points = int(max_map_points)
        self.depth_scale = float(depth_scale)
        self.depth_max = float(depth_max)
        self._volume = self._new_volume()
        self._cache_xyz: Optional[np.ndarray] = None
        self._cache_colors: Optional[np.ndarray] = None
        self._dirty = True
        self._color_warned = False

    def _new_volume(self) -> o3d.pipelines.integration.ScalableTSDFVolume:
        return o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_size,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        )

    def integrate_rgbd(
        self,
        color_bgr_or_rgb: np.ndarray,
        depth_u16: np.ndarray,
        intrinsic: o3d.camera.PinholeCameraIntrinsic,
        T_map_cam: np.ndarray,
        color_is_bgr: bool = True,
    ) -> None:
        """融合一帧对齐的 RGB-D（depth 单位由 depth_scale 解释，默认 mm）。"""
        if color_is_bgr:
            color_rgb = color_bgr_or_rgb[:, :, ::-1].copy()
        else:
            color_rgb = color_bgr_or_rgb
        color_o3d = o3d.geometry.Image(np.ascontiguousarray(color_rgb, dtype=np.uint8))
        depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_u16, dtype=np.uint16))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=self.depth_scale,
            depth_trunc=self.depth_max,
            convert_rgb_to_intensity=False,
        )
        # Open3D integrate 需要 camera → world；我们有 T_map_cam (world←cam)
        self._volume.integrate(rgbd, intrinsic, np.linalg.inv(T_map_cam))
        self._dirty = True

    def integrate(
        self,
        xyz_cam: np.ndarray,
        colors: Optional[np.ndarray],
        T_map_cam: np.ndarray,
        min_range: float,
        max_range: float,
    ) -> None:
        # 点云路径：无内参时不走 TSDF，退化为临时点云累加提示
        raise RuntimeError(
            'TSDF backend 请走 RGB-D（color+depth+camera_info），'
            '不要用纯点云 integrate()')

    def _extract(self) -> None:
        if not self._dirty:
            return
        pcd = self._volume.extract_point_cloud()
        if self.voxel_size > 0 and len(pcd.points) > 0:
            pcd = pcd.voxel_down_sample(self.voxel_size)
        if len(pcd.points) > self.max_map_points and self.voxel_size > 0:
            vs = self.voxel_size
            while len(pcd.points) > self.max_map_points:
                vs *= 1.5
                pcd = pcd.voxel_down_sample(vs)
        if len(pcd.points) == 0:
            self._cache_xyz = np.zeros((0, 3), dtype=np.float64)
            self._cache_colors = None
        else:
            self._cache_xyz = np.asarray(pcd.points, dtype=np.float64)
            colors = None
            if pcd.has_colors():
                colors = np.asarray(pcd.colors, dtype=np.float64)
                # Open3D ScalableTSDF 有时几何正常但颜色全 0；用高度伪彩保证 RViz 可见
                if colors.size == 0 or float(np.max(colors)) < 1e-3:
                    colors = _height_colormap(self._cache_xyz)
                    if not self._color_warned:
                        # 只提示一次：真彩丢失时用伪彩
                        print(
                            '[aubo_scene_recon] TSDF 顶点色全黑，已改用高度伪彩；'
                            '要相机真彩请用 backend:=open3d',
                            flush=True)
                        self._color_warned = True
            else:
                colors = _height_colormap(self._cache_xyz)
            self._cache_colors = colors
        self._dirty = False

    def get_map(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        self._extract()
        assert self._cache_xyz is not None
        return self._cache_xyz, self._cache_colors

    def get_o3d_cloud(self) -> o3d.geometry.PointCloud:
        self._extract()
        pcd = o3d.geometry.PointCloud()
        if self._cache_xyz is not None and self._cache_xyz.size:
            pcd.points = o3d.utility.Vector3dVector(self._cache_xyz)
            if self._cache_colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(self._cache_colors)
        return pcd

    def reset(self) -> None:
        self._volume = self._new_volume()
        self._cache_xyz = None
        self._cache_colors = None
        self._dirty = True

    def num_points(self) -> int:
        self._extract()
        return 0 if self._cache_xyz is None else int(self._cache_xyz.shape[0])
