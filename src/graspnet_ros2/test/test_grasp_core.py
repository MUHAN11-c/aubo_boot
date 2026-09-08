# -*- coding: utf-8 -*-
"""
零 ROS 纯核测试：GraspList（NMS/排序/掩码）、碰撞检测、旋转列重排与参数漂移守卫.

不 import rclpy、不造 DDS 现场；torch 算子部分用 importorskip 守卫，
在装有 torch 的 venv 下额外生效。
"""

from __future__ import annotations

from graspnet_ros2.grasp_core import (
    GRASP_ARRAY_LEN,
    GraspList,
    ModelFreeCollisionDetector,
    voxel_down_sample,
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


def test_grasplist_sort_and_indexing():
    rows = np.stack([_make_grasp([0, 0, 0.5], score=0.2),
                     _make_grasp([0.1, 0, 0.5], score=0.9),
                     _make_grasp([0.2, 0, 0.5], score=0.5)])
    gg = GraspList(rows)
    gg.sort_by_score()
    assert gg.scores.tolist() == pytest.approx([0.9, 0.5, 0.2])
    assert gg[0].score == pytest.approx(0.9)  # 整数索引 → Grasp
    sub = gg[1:]  # 切片 → GraspList
    assert isinstance(sub, GraspList) and len(sub) == 2
    assert gg.translations.shape == (3, 3)
    assert gg.rotation_matrices.shape == (3, 3, 3)


def test_grasplist_nms_suppresses_duplicates():
    # 同位置同姿态的重复抓取（不同分数）+ 一个远离的抓取
    rows = np.stack([
        _make_grasp([0, 0, 0.5], score=0.9),
        _make_grasp([0.005, 0, 0.5], score=0.8),   # 平移 < 0.03 且同旋转 → 抑制
        _make_grasp([0.5, 0.5, 0.5], score=0.1),   # 远离 → 保留
    ])
    kept = GraspList(rows).nms(translation_thresh=0.03, rotation_thresh=np.pi / 6)
    assert len(kept) == 2
    assert kept.scores.tolist() == pytest.approx([0.9, 0.1])


def test_grasplist_nms_keeps_same_translation_different_rotation():
    theta = np.pi / 3  # 60° > 30° 阈值
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rot = np.array([[1, 0, 0], [0, cos_t, -sin_t], [0, sin_t, cos_t]])
    rows = np.stack([
        _make_grasp([0, 0, 0.5], score=0.9),
        _make_grasp([0, 0, 0.5], score=0.8, rot=rot),
    ])
    kept = GraspList(rows).nms(translation_thresh=0.03, rotation_thresh=np.pi / 6)
    assert len(kept) == 2


def test_voxel_down_sample_reduces_and_centers():
    points = np.array([[0.001, 0.001, 0.001], [0.002, 0.002, 0.002],
                       [0.5, 0.5, 0.5]], dtype=np.float64)
    down = voxel_down_sample(points, 0.01)
    assert down.shape == (2, 3)
    assert np.allclose(down[0], points[:2].mean(axis=0))


def test_collision_detector_flags_grasp_on_point():
    # 点云落在左指实体区域（y 在 (-w/2-fw, -w/2)，x 在指长范围）→ 应判碰撞；
    # 注：两指之间的空腔不算碰撞区（那是允许抓取物所在）。
    rng = np.random.default_rng(0)
    w, fw = 0.02, 0.01
    blob = rng.uniform(-0.002, 0.002, (300, 3)) + np.array([0.0, -(w / 2 + fw / 2), 0.0])
    rows = np.stack([_make_grasp([0.0, 0.0, 0.0], score=0.9, width=w)])
    detector = ModelFreeCollisionDetector(blob, voxel_size=0.01)
    mask = detector.detect(GraspList(rows), approach_dist=0.05, collision_thresh=0.01)
    assert bool(mask[0]) is True

    # 对照：点云只在两指之间的空腔（|y| < w/2）→ 不算碰撞
    cavity = rng.uniform(-0.002, 0.002, (300, 3))
    detector2 = ModelFreeCollisionDetector(cavity, voxel_size=0.01)
    mask2 = detector2.detect(GraspList(rows), approach_dist=0.05, collision_thresh=0.01)
    assert bool(mask2[0]) is False


def test_graspnet_to_ros_rotation_remap():
    """旋转列重排守卫：GraspNet(approach,width,height) → ROS(width,height,approach)."""
    from graspnet_ros2.grasp_core import graspnet_to_ros_rotation

    # 各列可辨识的正交基：col0=(1,0,0), col1=(0,1,0), col2=(0,0,1)
    rot = np.eye(3)
    remapped = graspnet_to_ros_rotation(rot)
    # ROS X = 原 col1, Y = 原 col2, Z = 原 col0
    assert np.allclose(remapped[:, 0], rot[:, 1])
    assert np.allclose(remapped[:, 1], rot[:, 2])
    assert np.allclose(remapped[:, 2], rot[:, 0])
    # 右手系保持：det=+1
    assert np.isclose(np.linalg.det(remapped), 1.0)


def test_yaml_params_match_declared():
    """YAML 参数键与节点 declare_parameter 的并集守卫（防多处默认值漂移）."""
    import os
    import re

    import yaml

    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def declared_params(src_name):
        src = open(os.path.join(pkg_root, 'graspnet_ros2', src_name), encoding='utf-8').read()
        return set(re.findall(r"declare_parameter\('([a-z_]+)'", src))

    with open(os.path.join(pkg_root, 'config', 'graspnet.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    assert set(cfg.keys()) == {'graspnet_demo_points_node', 'publish_grasps_client'}
    node_params = set(cfg['graspnet_demo_points_node']['ros__parameters'])
    client_params = set(cfg['publish_grasps_client']['ros__parameters'])

    # 节点：model_path 由 launch 注入，不入 yaml
    declared_node = declared_params('graspnet_node.py')
    assert declared_node == node_params | {'model_path'}, (
        f'yaml 与节点参数漂移: yaml-only={node_params - declared_node}, '
        f'code-only={declared_node - node_params - {"model_path"}}'
    )

    # 客户端：继承 motion_controller 的 planning_group/base_frame/ee_link
    declared_client = declared_params('publish_grasps_client.py') | declared_params(
        'motion_controller.py'
    )
    assert declared_client == client_params, (
        f'yaml 与客户端参数漂移: yaml-only={client_params - declared_client}, '
        f'code-only={declared_client - client_params}'
    )


def test_torch_ops_match_reference():
    """纯 torch 算子与暴力参考实现精确比对（venv 有 torch 时生效）."""
    torch = pytest.importorskip('torch')
    from graspnet_ros2.graspnet_lib.pointnet2_ops import (
        ball_query,
        cylinder_query,
        furthest_point_sample,
    )

    rng = np.random.default_rng(0)
    xyz_np = rng.uniform(-1, 1, (2, 300, 3)).astype(np.float32)
    xyz = torch.from_numpy(xyz_np)
    new_xyz = xyz[:, :25]

    def ref_fps(xyz_t, npoint):
        b, n, _ = xyz_t.shape
        out = []
        for bi in range(b):
            sel = [0]
            dist = ((xyz_t[bi] - xyz_t[bi, 0]) ** 2).sum(-1)
            for _ in range(npoint - 1):
                nxt = int(torch.argmax(dist).item())
                sel.append(nxt)
                d = ((xyz_t[bi] - xyz_t[bi, nxt]) ** 2).sum(-1)
                dist = torch.minimum(dist, d)
            out.append(sel)
        return torch.tensor(out)

    assert torch.equal(furthest_point_sample(xyz, 32), ref_fps(xyz, 32))

    idx = ball_query(0.3, 7, xyz, new_xyz)
    for b in range(2):
        for j in range(new_xyz.shape[1]):
            d2 = ((xyz[b] - new_xyz[b, j]) ** 2).sum(-1)
            hits = torch.nonzero(d2 < 0.09, as_tuple=False).flatten().tolist()
            expect = (hits + [hits[0]] * 7)[:7] if hits else [0] * 7
            assert idx[b, j].tolist() == expect

    rot = torch.from_numpy(np.tile(np.eye(3).reshape(1, 1, 9), (2, 25, 1)).astype(np.float32))
    idx = cylinder_query(0.3, -0.02, 0.05, 7, xyz, new_xyz, rot)
    for b in range(2):
        for j in range(new_xyz.shape[1]):
            off = xyz[b] - new_xyz[b, j]
            hits = torch.nonzero(
                (off[:, 1] ** 2 + off[:, 2] ** 2 < 0.09)
                & (off[:, 0] > -0.02) & (off[:, 0] < 0.05),
                as_tuple=False,
            ).flatten().tolist()
            expect = (hits + [hits[0]] * 7)[:7] if hits else [0] * 7
            assert idx[b, j].tolist() == expect
