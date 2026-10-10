"""ScenePerceptionParams derived tool / gravity (yaml namespace, no ROS)."""
from __future__ import annotations

from types import SimpleNamespace as NS

import numpy as np
from peach_harvester.vision.scene_perception.params import ScenePerceptionParams
import pytest


def _snapshot(**over):
    base = {
        'gravity_hint_xyz': '',
        'gravity_mode': 'tf',
        'pipeline': NS(min_depth_m=0.3, max_depth_m=2.5),
        'tool': NS(
            D_inner=0.104, L_insert=0.2, L_blade=0.0, entry_d_tool=0.03,
            entry_d_s=0.0, clearance_min=0.005, margin_neck=0.015,
            version='1.1'),
    }
    base.update(over)
    return NS(**base)


def test_tool_derived_entry_standoff_sums_components():
    p = ScenePerceptionParams.from_params(_snapshot())
    assert p.tool.entry_standoff == pytest.approx(0.03)
    assert p.tool.d_inner_m == pytest.approx(0.104)
    assert p.tool.insert_length_m == pytest.approx(0.2)
    assert p.gravity_hint is None
    assert p.gravity_mode == 'tf'
    assert p.pipeline.max_depth_m == 2.5
    assert p.tool.version == '1.1'


def test_gravity_hint_parses_three_floats():
    p = ScenePerceptionParams.from_params(
        _snapshot(gravity_hint_xyz='0.1,-0.2,1.0'))
    assert np.allclose(p.gravity_hint, [0.1, -0.2, 1.0])


def test_bad_gravity_hint_raises():
    with pytest.raises(ValueError):
        ScenePerceptionParams.from_params(_snapshot(gravity_hint_xyz='1,2'))


def test_cross_field_depth_window_rejected():
    with pytest.raises(ValueError):
        ScenePerceptionParams.from_params(
            _snapshot(pipeline=NS(min_depth_m=2.6, max_depth_m=2.5)))
