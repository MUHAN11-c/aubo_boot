"""Open3D 融合后端单元测试."""

from __future__ import annotations

from pathlib import Path

from aubo_scene_recon.backends.cloud_accum import CloudAccumBackend
from aubo_scene_recon.pc_utils import (
    filter_by_depth,
    transform_points,
    voxel_downsample,
)
import numpy as np
import open3d as o3d


def test_transform_points_translation():
    xyz = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    T = np.eye(4)
    T[:3, 3] = [0.1, 0.2, 0.3]
    out = transform_points(xyz, T)
    np.testing.assert_allclose(out[0], [1.1, 0.2, 0.3], atol=1e-9)


def test_filter_by_depth():
    xyz = np.array([
        [0, 0, 0.1],
        [0, 0, 0.5],
        [0, 0, 2.0],
    ], dtype=np.float64)
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64)
    xyz_f, col_f = filter_by_depth(xyz, colors, 0.2, 1.5)
    assert xyz_f.shape[0] == 1
    np.testing.assert_allclose(xyz_f[0], [0, 0, 0.5])


def test_voxel_downsample_merges():
    xyz = np.array([
        [0.001, 0.0, 0.0],
        [0.002, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ], dtype=np.float64)
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64)
    out_xyz, _ = voxel_downsample(xyz, colors, 0.01)
    assert out_xyz.shape[0] == 2


def test_open3d_cloud_accum_and_save(tmp_path: Path):
    backend = CloudAccumBackend(
        voxel_size=0.01, max_map_points=100000, outlier_every_n=0)
    T = np.eye(4)
    T[0, 3] = 0.5
    xyz = np.array([[0.0, 0.0, 0.5], [0.0, 0.0, 0.6]], dtype=np.float64)
    colors = np.ones((2, 3), dtype=np.float64)
    backend.integrate(xyz, colors, T, 0.2, 1.5)
    assert backend.num_points() >= 1
    map_xyz, _ = backend.get_map()
    assert np.min(np.linalg.norm(map_xyz - np.array([0.5, 0.0, 0.5]), axis=1)) < 0.05

    path = tmp_path / 't.ply'
    assert o3d.io.write_point_cloud(str(path), backend.get_o3d_cloud())
    assert path.stat().st_size > 0

    backend.reset()
    assert backend.num_points() == 0
