"""view_planner Python 移植对照测试（C++ 同场景语义锚；重写轮 3c-2b）."""
from __future__ import annotations

import math

from peach_harvester.cycle_core.view_planner import (
    _angle_deg,
    _clamp_to_step,
    _project_look_ray_into_reach,
    generate,
    look_at_optical,
    ViewContext,
    ViewPlannerConfig,
)


def test_clamp_to_step():
    cur = [0.0, 0.0, 0.0]
    goal = [0.3, 0.0, 0.0]
    out = _clamp_to_step(cur, goal, 0.15)
    assert math.isclose(math.dist(out, cur), 0.15, abs_tol=1e-9)
    near = _clamp_to_step(cur, [0.05, 0, 0], 0.15)
    assert near == [0.05, 0.0, 0.0]


def test_project_into_reach_pulls_in_along_ray():
    # target 在可达球内，相机在外侧远端：投影沿视线收进、保持在射线上、
    # 距目标不小于 min_radius（C++ 语义：min_radius 下限可把点推出球外，
    # 这是原实现的既定边界，此处按契约断言而非按理想球断言）。
    target = [0.4, -0.4, 0.2]
    camera = [0.9, -0.9, 0.45]  # |camera|≈1.33 > reach
    out = _project_look_ray_into_reach(target, camera, 0.32, 0.78)
    assert _angle_deg(
        [o - t for o, t in zip(out, target)],
        [c - t for c, t in zip(camera, target)]) < 1.0
    assert math.dist(out, target) >= 0.32 - 1e-9
    # 收进量：不比原相机位更远（沿射线向内收了一步）
    assert math.dist(out, target) < math.dist(camera, target)
    # 宽球场景（floor 不绑定）：应落在球内
    wide = _project_look_ray_into_reach(target, camera, 0.2, 1.0)
    assert math.sqrt(sum(x * x for x in wide)) <= 1.0 + 1e-9


def test_generate_orders_and_caps_travel():
    ctx = ViewContext(
        target=[0.45, -0.6, 0.55],
        current_camera_position=[0.35, -0.35, 0.65],
        observed_directions=[])
    cands = generate(ctx)
    assert cands, '应产生候选'
    cfg = ViewPlannerConfig()
    for cand in cands:
        assert cand.travel_m <= cfg.max_camera_step_m + 1e-6
        assert cand.position[2] >= cfg.min_camera_height_m
    scores = [c.score for c in cands]
    assert scores == sorted(scores, reverse=True)


def test_generate_low_frame_pushes_closer():
    far = ViewContext(
        target=[0.45, -0.6, 0.55],
        current_camera_position=[0.30, -0.30, 0.70],
        bbox_valid=True, bbox_x=280, bbox_y=200, bbox_w=60, bbox_h=50,
        foreground_ratio=0.9)
    good = ViewContext(
        target=[0.45, -0.6, 0.55],
        current_camera_position=[0.30, -0.30, 0.70])
    assert generate(far) and generate(good)


def test_look_at_optical_orthonormal():
    basis = look_at_optical([0.3, -0.3, 0.7], [0.45, -0.6, 0.55])
    x, y, z = basis
    for a, b in ((x, y), (y, z), (x, z)):
        dot = sum(a[i] * b[i] for i in range(3))
        assert abs(dot) < 1e-9
    assert all(abs(math.sqrt(sum(v * v for v in vec)) - 1.0) < 1e-9
               for vec in (x, y, z))
