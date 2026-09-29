# -*- coding: utf-8 -*-
"""零 ROS 纯核测试：layout（加载/摆位扰动确定性/相机 TF/manifest）."""

from __future__ import annotations

import math

import numpy as np
import pytest

from ivg_sim.layout import (
    GZ_OPTICAL_CONVENTION,
    apply_jitter,
    build_manifest,
    camera_optical_tf,
    camera_world_pose,
    load_layout,
    optical_frame_rotation,
)


@pytest.fixture()
def layout():
    return load_layout()


def test_load_layout_parses_all_sections(layout):
    assert layout.world_name == 'ivg_table'
    assert layout.table_top_z == pytest.approx(0.75)
    assert len(layout.placements) == 10
    assert all(p.radius_m > 0 for p in layout.placements)
    mustard = next(p for p in layout.placements if p.model == 'mustard_bottle')
    assert mustard.z == pytest.approx(0.75)  # 对象底面 = 桌面


def test_jitter_deterministic_and_bounded(layout):
    a = apply_jitter(layout, seed=7)
    b = apply_jitter(layout, seed=7)
    c = apply_jitter(layout, seed=8)
    assert [(p.x, p.y, p.yaw_deg) for p in a.placements] == \
        [(p.x, p.y, p.yaw_deg) for p in b.placements]
    assert a.seed == 7
    assert [(p.x, p.y) for p in a.placements] != \
        [(p.x, p.y) for p in c.placements]
    # 扰动保持在桌面内
    half_x = layout.table_size[0] / 2
    half_y = layout.table_size[1] / 2
    for p in a.placements:
        assert -half_x <= p.x <= half_x
        assert -half_y <= p.y <= half_y


def test_camera_pose_tilt(layout):
    """tilt=35°：相机沿 −视轴后退保持视距 0.75，光轴朝 +X 倾斜."""
    assert layout.camera_tilt_deg == pytest.approx(35.0)
    xyz, rpy = camera_world_pose(layout)
    tilt = math.radians(35.0)
    assert xyz[0] == pytest.approx(-0.75 * math.sin(tilt))
    assert xyz[2] == pytest.approx(0.75 + 0.75 * math.cos(tilt))
    assert rpy[1] == pytest.approx(math.pi / 2 - tilt)
    # 视轴（传感器 +X）指向桌面中心方向（含 +X 分量、向下）
    rot = _rpy_to_matrix(rpy)
    view = rot @ np.array([1, 0, 0])
    assert view[0] == pytest.approx(math.sin(tilt))
    assert view[2] == pytest.approx(-math.cos(tilt))


def _rpy_to_matrix(rpy):
    cr, cp, cy = (math.cos(v) for v in rpy)
    sr, sp, sy = (math.sin(v) for v in rpy)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def test_body_convention_rotation_maps_depth_axis():
    """body 约定（spike 实测）：点云 x=视轴向下、y/z 为世界横向."""
    rot = optical_frame_rotation(0.0, 'body')
    assert np.isclose(np.linalg.det(rot), 1.0)
    # 光学系 x 轴在世界系中 = (0,0,-1)（向下=深度方向）
    assert np.allclose(rot[:, 0], [0.0, 0.0, -1.0], atol=1e-9)
    # y 轴 = 世界 +Y；z 轴 = 世界 +X
    assert np.allclose(rot[:, 1], [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(rot[:, 2], [1.0, 0.0, 0.0], atol=1e-9)


def test_body_convention_with_tilt():
    """tilt 时 body x 轴 = 视轴（sin t, 0, −cos t）."""
    rot = optical_frame_rotation(35.0, 'body')
    t = math.radians(35.0)
    assert np.allclose(rot[:, 0], [math.sin(t), 0.0, -math.cos(t)], atol=1e-9)
    assert np.allclose(rot[:, 1], [0.0, 1.0, 0.0], atol=1e-9)
    assert np.isclose(np.linalg.det(rot), 1.0)


def test_optical_tf_quaternion_roundtrip(layout):
    xyz, quat = camera_optical_tf(layout, GZ_OPTICAL_CONVENTION)
    assert len(quat) == 4 and np.isclose(np.linalg.norm(quat), 1.0)
    # 四元数 → 矩阵应还原 optical_frame_rotation
    q = np.array(quat)
    rot = _quat_to_matrix(q)
    assert np.allclose(
        rot, optical_frame_rotation(layout.camera_tilt_deg, GZ_OPTICAL_CONVENTION),
        atol=1e-6,
    )


def _quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_manifest_roundtrip(layout):
    manifest = build_manifest(layout, world_sha256='deadbeef')
    assert manifest['world_sha256'] == 'deadbeef'
    assert len(manifest['objects']) == len(layout.placements)
    obj = manifest['objects'][0]
    assert obj['xyz'][2] == pytest.approx(layout.table_top_z)
    assert obj['top_center_xyz'][2] == pytest.approx(
        layout.table_top_z + obj['radius_m']
    )
    assert manifest['camera']['convention'] == GZ_OPTICAL_CONVENTION
