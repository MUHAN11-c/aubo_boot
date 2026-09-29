# -*- coding: utf-8 -*-
"""零 ROS 纯核测试：模型无关后处理链（裁剪/碰撞/NMS/topK）与工作区过滤."""

from __future__ import annotations

from ivg_graspnet.grasp_core import GRASP_ARRAY_LEN, GraspList
from ivg_graspnet.postprocess import (
    crop_workspace,
    GripperGeometry,
    PostprocessConfig,
    run_postprocess,
)
import numpy as np
import pytest


def _make_grasp(translation, score=0.5, width=0.04, rot=None):
    row = np.zeros(GRASP_ARRAY_LEN)
    row[0] = score
    row[1] = width
    row[2] = 0.02
    row[3] = 0.03
    row[4:13] = (np.eye(3) if rot is None else np.asarray(rot)).reshape(-1)
    row[13:16] = translation
    row[16] = -1
    return row


def test_crop_workspace_filters_box():
    points = np.array([
        [0.0, 0.0, 0.5],
        [10.0, 0.0, 0.5],   # x 越界
        [0.0, 10.0, 0.5],   # y 越界
        [0.0, 0.0, -1.0],   # z 越界
    ])
    cropped = crop_workspace(points, (-1.0, 1.0, -1.0, 1.0, 0.0, 1.0))
    assert cropped.shape == (1, 3)
    assert crop_workspace(points, None) is points


def test_run_postprocess_empty_passthrough():
    grasps = GraspList()
    out = run_postprocess(grasps, np.zeros((10, 3)), PostprocessConfig())
    assert len(out) == 0


def test_run_postprocess_chain_order():
    """Postprocess 链场景：开口裁剪 → 碰撞滤除 → NMS → 降序."""
    rng = np.random.default_rng(0)
    # 碰撞点团：落在原点抓取（w=0.02）的左指实体区（y ∈ (-w/2-fw, -w/2)）
    w, fw = 0.02, 0.01
    blob = rng.uniform(-0.002, 0.002, (300, 3)) + np.array([0.0, -(w / 2 + fw / 2), 0.0])

    rows = np.stack([
        _make_grasp([0, 0, 0], score=0.99, width=0.02),       # 碰撞 → 滤除
        _make_grasp([0.005, 0, 0], score=0.9, width=0.02),    # 同碰撞区 → 滤除
        _make_grasp([0.5, 0, 0], score=0.8, width=0.04),      # 干净区 NMS 胜者
        _make_grasp([0.505, 0, 0], score=0.6, width=0.04),    # 平移 < 0.03 → NMS 抑制
        _make_grasp([1.0, 0, 0], score=0.7, width=0.5),       # 超开口 → 裁剪后保留
    ])
    config = PostprocessConfig(
        collision_thresh=0.01, voxel_size=0.01, approach_dist=0.05,
        max_gripper_width=0.06, max_grasps=5,
        nms_translation_thresh=0.03, nms_rotation_thresh_deg=30.0,
    )
    out = run_postprocess(GraspList(rows.copy()), blob, config)

    # 原点两条被碰撞滤除；0.5 附近 NMS 留高分；远端宽抓取开口被裁到上限
    assert out.scores.tolist() == [0.8, 0.7]
    assert out.widths.tolist() == [0.04, pytest.approx(0.06)]


def test_run_postprocess_topk_limit():
    """Top-K 截取与降序."""
    rows = np.stack([
        _make_grasp([0.0, 0, 0], score=0.3),
        _make_grasp([0.1, 0, 0], score=0.9),
        _make_grasp([0.2, 0, 0], score=0.6),
        _make_grasp([0.3, 0, 0], score=0.5),
    ])
    config = PostprocessConfig(
        collision_thresh=0.0,  # 关碰撞
        max_grasps=2,
    )
    out = run_postprocess(GraspList(rows), np.zeros((1, 3)), config)
    assert out.scores.tolist() == [0.9, 0.6]


def test_gripper_geometry_authority():
    """碰撞几何单源：GripperGeometry 覆盖检测器默认值."""
    geom = GripperGeometry(finger_width=0.02, finger_length=0.08)
    from ivg_graspnet.postprocess import ModelFreeCollisionDetector

    detector = ModelFreeCollisionDetector(np.zeros((1, 3)), voxel_size=0.01, geometry=geom)
    assert detector.finger_width == 0.02
    assert detector.finger_length == 0.08
