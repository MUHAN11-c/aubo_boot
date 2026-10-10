"""
RANSAC 两段式打分与单起点抛光的几何质量回归（preemptive 优化轮）.

合成圆柱/球体（固定种子 + 已知真值）锁恢复质量与确定性；子样本直通
路径锁「N ≤ 阈值不抽稀、零 rng 消耗」的等价机制。断言的是质量界与
确定性，不是与旧实现的逐数一致（旧单段打分在 N > 阈值时允许换赢家）。
"""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.common.geometry import (
    _preempt_screen,
    fit_cylinder_robust,
    fit_sphere_robust,
    polish_cylinder_axis,
    ransac_cylinder,
    ransac_sphere,
)


def _unit(v):
    """归一化向量."""
    return np.asarray(v, dtype=float) / np.linalg.norm(v)


def _orthonormal_basis(axis):
    """与 axis 正交的单位正交基 (u, v)."""
    a = np.asarray(axis, dtype=float)
    ref = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array(
        [0.0, 1.0, 0.0])
    u = _unit(np.cross(ref, a))
    v = _unit(np.cross(a, u))
    return u, v


def _make_cylinder(n=3000, radius=0.035, half_len=0.10, sigma=0.001,
                   center=(0.02, -0.03, 0.65), axis=(0.3, 0.5, 0.8),
                   seed=7):
    """合成圆柱表面点 + 径向法线（固定种子）."""
    rng = np.random.default_rng(seed)
    a = _unit(axis)
    u, v = _orthonormal_basis(a)
    theta = rng.uniform(0.0, 2.0 * np.pi, n)
    z = rng.uniform(-half_len, half_len, n)
    r = radius + rng.normal(0.0, sigma, n)
    c = np.asarray(center, dtype=float)
    pts = (c
           + np.outer(r * np.cos(theta), u)
           + np.outer(r * np.sin(theta), v)
           + np.outer(z, a))
    nrm = (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))
    return pts, nrm, a, radius, c


def _make_sphere(n=2000, radius=0.034, sigma=0.001,
                 center=(0.01, 0.02, 0.6), seed=11):
    """合成球面点 + 径向法线（固定种子）."""
    rng = np.random.default_rng(seed)
    dirs = rng.normal(size=(n, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    c = np.asarray(center, dtype=float)
    pts = c + (radius + rng.normal(0.0, sigma, n))[:, None] * dirs
    return pts, dirs, radius, c


def _axis_angle_deg(a, b):
    """两单位轴无向夹角（度）."""
    cosv = abs(float(np.dot(_unit(a), _unit(b))))
    return float(np.degrees(np.arccos(np.clip(cosv, 0.0, 1.0))))


def test_cylinder_recovery_quality():
    """合成圆柱：轴角 <2°、半径 ±1.5mm、内点率 ≥0.85、rms <2.5mm."""
    pts, nrm, axis_true, radius_true, _ = _make_cylinder()
    est = fit_cylinder_robust(pts, nrm)
    assert est is not None
    assert _axis_angle_deg(est['axis'], axis_true) < 2.0
    assert abs(est['radius'] - radius_true) < 0.0015
    assert est['inlier_ratio'] >= 0.85
    assert est['rms'] < 0.0025


def test_cylinder_deterministic_same_seed():
    """同种子两次调用逐数一致（含 N > 阈值的子样本路径）."""
    pts, nrm, axis_true, _, _ = _make_cylinder()
    a = ransac_cylinder(pts, nrm, seed=3)
    b = ransac_cylinder(pts, nrm, seed=3)
    assert a is not None and b is not None
    np.testing.assert_array_equal(a['axis'], b['axis'])
    np.testing.assert_array_equal(a['q0'], b['q0'])
    assert a['radius'] == b['radius']
    np.testing.assert_array_equal(a['inliers'], b['inliers'])


def test_sphere_recovery_quality():
    """合成球：球心 <2mm、半径 ±1.5mm、内点率 ≥0.85."""
    pts, nrm, radius_true, center_true = _make_sphere()
    est = fit_sphere_robust(pts, nrm)
    assert est is not None
    assert np.linalg.norm(est['center'] - center_true) < 0.002
    assert abs(est['radius'] - radius_true) < 0.0015
    assert est['inlier_ratio'] >= 0.85


def test_sphere_deterministic_same_seed():
    """同种子两次调用逐数一致."""
    pts, nrm, _, _ = _make_sphere()
    a = ransac_sphere(pts, nrm, seed=5)
    b = ransac_sphere(pts, nrm, seed=5)
    assert a is not None and b is not None
    np.testing.assert_array_equal(a['center'], b['center'])
    assert a['radius'] == b['radius']
    np.testing.assert_array_equal(a['inliers'], b['inliers'])


def test_preempt_screen_passthrough_below_threshold():
    """N ≤ 阈值：原数组直通（零 rng 消耗），等价机制锚点."""
    pts = np.arange(510 * 3, dtype=float).reshape(510, 3)
    rng = np.random.default_rng(0)
    out = _preempt_screen(pts, rng)
    assert out is pts
    # rng 未被消费：下一个数与全新 Generator 一致
    assert int(rng.integers(10**9)) == int(
        np.random.default_rng(0).integers(10**9))


def test_preempt_screen_subsample_shape_and_seed_stability():
    """N > 阈值：规模封顶 + 固定种子可复现（两次同种子同子样本）."""
    pts = np.arange(2000 * 3, dtype=float).reshape(2000, 3)
    out1 = _preempt_screen(pts, np.random.default_rng(1))
    out2 = _preempt_screen(pts, np.random.default_rng(1))
    assert out1.shape == (512, 3)
    np.testing.assert_array_equal(out1, out2)


def test_polish_single_start_recovers_axis():
    """单起点抛光：从 RANSAC 提示收敛到真值轴（<1°），方向与提示一致."""
    pts, nrm, axis_true, _, _ = _make_cylinder(n=1500)
    est = ransac_cylinder(pts, nrm)
    assert est is not None
    inl = est['inliers']
    sub = inl if len(inl) <= 800 else inl[np.linspace(
        0, len(inl) - 1, 800, dtype=int)]
    axis, q0 = polish_cylinder_axis(pts[sub], est['axis'])
    assert _axis_angle_deg(axis, axis_true) < 1.0
    assert float(np.dot(axis, est['axis'])) > 0.0
    assert np.all(np.isfinite(q0))
