# -*- coding: utf-8 -*-
"""零 ROS 纯核测试：score_against_gt（命中/覆盖/垂直度数学）."""

from __future__ import annotations

import numpy as np
import pytest

from ivg_sim.score_grasps import (
    COVERAGE_GATE,
    _approach_verticality,
    score_against_gt,
)


def _objects():
    return [
        {'model': 'apple', 'xyz': [0.10, 0.00, 0.75], 'radius_m': 0.045},
        {'model': 'mustard_bottle', 'xyz': [-0.18, 0.10, 0.75], 'radius_m': 0.10},
    ]


IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)  # approach Z 平行世界 Z → 垂直度 1


def test_hit_when_grasp_on_object():
    grasps = np.array([[0.10, 0.00, 0.80], [-0.18, 0.10, 0.83]])
    quats = np.array([IDENTITY_QUAT, IDENTITY_QUAT])
    result = score_against_gt(grasps, quats, _objects(), table_top_z=0.75)
    assert result['coverage'] == 1.0 and result['gate_pass'] is True
    assert all(r['hit'] for r in result['per_object'])
    assert result['mean_verticality'] == pytest.approx(1.0)


def test_miss_when_out_of_radius():
    grasps = np.array([[0.5, 0.5, 0.80]])
    quats = np.array([IDENTITY_QUAT])
    result = score_against_gt(grasps, quats, _objects(), table_top_z=0.75)
    assert result['coverage'] == 0.0 and result['gate_pass'] is False
    assert result['per_object'][0]['nearest_xy_dist_m'] > 0.0


def test_xy_hit_but_wrong_z_is_miss():
    """XY 命中但 z 在门外（如抓到桌面下方/半空）不算命中."""
    grasps = np.array([[0.10, 0.00, 0.20]])
    quats = np.array([IDENTITY_QUAT])
    result = score_against_gt(grasps, quats, _objects(), table_top_z=0.75)
    assert result['per_object'][0]['hit'] is False


def test_empty_grasps_reported_zero():
    grasps = np.zeros((0, 3))
    quats = np.zeros((0, 4))
    result = score_against_gt(grasps, quats, _objects(), table_top_z=0.75)
    assert result['n_grasps'] == 0 and result['coverage'] == 0.0


def test_partial_coverage_gate_semantics():
    """门判定 = coverage ≥ COVERAGE_GATE（当前基线门 0.4，见模块注释）."""
    grasps = np.array([[0.10, 0.00, 0.80]])  # 只命中 apple（1/2 = 0.5）
    quats = np.array([IDENTITY_QUAT])
    result = score_against_gt(grasps, quats, _objects(), table_top_z=0.75)
    assert result['coverage'] == pytest.approx(0.5)
    assert result['gate_pass'] == (0.5 >= COVERAGE_GATE)


def test_verticality_quat():
    # 绕 x 转 90°：approach Z 指向世界 -Y → 垂直度 0
    qx90 = (np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4))
    assert _approach_verticality(IDENTITY_QUAT) == pytest.approx(1.0)
    assert _approach_verticality(qx90) == pytest.approx(0.0, abs=1e-9)
