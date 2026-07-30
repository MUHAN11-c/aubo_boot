"""Phase 1：Open3D 点云累加 + voxel + 统计滤波。"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import open3d as o3d

from aubo_scene_recon.backends.base import FusionBackend
from aubo_scene_recon.pc_utils import filter_by_depth


class CloudAccumBackend(FusionBackend):
    """用 Open3D 做变换后体素合并与周期性去噪，比手写 numpy voxel 更稳。"""

    def __init__(
        self,
        voxel_size: float = 0.005,
        max_map_points: int = 2_000_000,
        outlier_every_n: int = 5,
        outlier_nb_neighbors: int = 20,
        outlier_std_ratio: float = 2.0,
    ):
        self.voxel_size = float(voxel_size)
        self.max_map_points = int(max_map_points)
        self.outlier_every_n = int(outlier_every_n)
        self.outlier_nb_neighbors = int(outlier_nb_neighbors)
        self.outlier_std_ratio = float(outlier_std_ratio)
        self._map = o3d.geometry.PointCloud()
        self._integrate_count = 0

    def integrate(
        self,
        xyz_cam: np.ndarray,
        colors: Optional[np.ndarray],
        T_map_cam: np.ndarray,
        min_range: float,
        max_range: float,
    ) -> None:
        xyz_f, col_f = filter_by_depth(xyz_cam, colors, min_range, max_range)
        if xyz_f.size == 0:
            return

        frame = o3d.geometry.PointCloud()
        frame.points = o3d.utility.Vector3dVector(xyz_f.astype(np.float64))
        if col_f is not None:
            frame.colors = o3d.utility.Vector3dVector(
                np.clip(col_f, 0.0, 1.0).astype(np.float64))
        frame.transform(T_map_cam.astype(np.float64))

        if len(self._map.points) == 0:
            self._map = frame
        else:
            self._map += frame

        if self.voxel_size > 0:
            self._map = self._map.voxel_down_sample(self.voxel_size)

        self._integrate_count += 1
        if (self.outlier_every_n > 0 and
                self._integrate_count % self.outlier_every_n == 0 and
                len(self._map.points) > self.outlier_nb_neighbors):
            self._map, _ = self._map.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio)

        # 超限：加大体素再压
        vs = self.voxel_size if self.voxel_size > 0 else 0.005
        while len(self._map.points) > self.max_map_points:
            vs *= 1.5
            self._map = self._map.voxel_down_sample(vs)

    def get_map(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        if len(self._map.points) == 0:
            return np.zeros((0, 3), dtype=np.float64), None
        xyz = np.asarray(self._map.points, dtype=np.float64)
        colors = None
        if self._map.has_colors():
            colors = np.asarray(self._map.colors, dtype=np.float64)
        return xyz, colors

    def get_o3d_cloud(self) -> o3d.geometry.PointCloud:
        return self._map

    def reset(self) -> None:
        self._map = o3d.geometry.PointCloud()
        self._integrate_count = 0

    def num_points(self) -> int:
        return int(len(self._map.points))
