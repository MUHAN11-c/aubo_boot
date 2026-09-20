"""
W3 反投影/裁剪单实现的数值等价对拍（零 ROS）.

三份旧实现（pose_pipelines._to_points / visualization._bbox_cloud_xyzrgb /
geometry.estimate_normals 内联段）原样拷贝为参考实现；合成深度图含
0 / 65535 / 窗内外 / 边界(300mm) / stride 各情形，断言 allclose。
边界 300mm 同时锚定 float32 窗比较口径（旧 valid_depth_mask 的边界取舍
必须逐位保留）。
"""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.common.geometry import (
    backproject,
    bbox_cloud_xyzrgb,
    crop_mask_to_bbox,
    estimate_normals,
    pack_rgb_bgr,
)
from peach_harvester.vision.scene_perception.image_gates import (
    clip_bbox,
    valid_depth_mask,
)
from peach_harvester.vision.scene_perception.pose_pipelines import (
    RobustBagPosePipeline,
)

# ═══════════════════════════════════════════════════════════════
# 参考实现：W3 替换前逐字拷贝（改动即失效，勿"顺手优化"）
# ═══════════════════════════════════════════════════════════════


def _ref_valid_depth_mask(depth, min_depth_m: float, max_depth_m: float):
    # 原 image_gates.valid_depth_mask（窗口口径参考）
    z = depth.astype(np.float32) / 1000.0
    return (depth > 0) & (depth < 65535) & (z > min_depth_m) & (z < max_depth_m)


def _ref_to_points(depth, mask, xoff, yoff, K, min_depth_m, max_depth_m):
    # 原 pose_pipelines.RobustBagPosePipeline._to_points（含管线距离窗）
    ys, xs = np.where(mask & _ref_valid_depth_mask(depth, min_depth_m, max_depth_m))
    z = depth[ys, xs].astype(float) / 1000.0
    points = np.column_stack(((xs + xoff - K['cx']) * z / K['fx'],
                              (ys + yoff - K['cy']) * z / K['fy'], z))
    return points, np.column_stack((xs, ys))


def _ref_bbox_cloud_xyzrgb(rgb_bgr, depth_mm, K, bboxes, stride: int = 1):
    # 原 visualization._bbox_cloud_xyzrgb（无窗 + 逐框锚定 stride）
    h, w = depth_mm.shape[:2]
    mask = np.zeros((h, w), dtype=bool)
    for bbox in bboxes:
        x1, y1, x2, y2 = [int(v) for v in bbox]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        mask[y1:y2:stride, x1:x2:stride] = True
    valid = mask & (depth_mm > 0) & (depth_mm < 65535)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0,), dtype=np.float32)
    vs, us = np.where(valid)
    z = depth_mm[vs, us].astype(np.float64) / 1000.0
    fx, fy = float(K['fx']), float(K['fy'])
    cx, cy = float(K['cx']), float(K['cy'])
    x = (us.astype(np.float64) - cx) * z / fx
    y = (vs.astype(np.float64) - cy) * z / fy
    xyz = np.column_stack((x, y, z))
    rgb_packed = pack_rgb_bgr(rgb_bgr[vs, us])
    return xyz, rgb_packed


def _ref_estimate_normals(depth_roi, xoff: int, yoff: int, K: dict,
                          depth_jump_mm: int = 30):
    # 原 geometry.estimate_normals（内联反投影段未提取时的全文）
    h, w = depth_roi.shape
    z = depth_roi.astype(np.float64) / 1000.0
    valid = (depth_roi > 0) & (depth_roi < 65535)
    us = np.arange(w)[None, :] + xoff
    vs = np.arange(h)[:, None] + yoff
    X = (us - K['cx']) * z / K['fx']
    Y = (vs - K['cy']) * z / K['fy']
    du = np.zeros((h, w, 3))
    dv = np.zeros((h, w, 3))
    du[:, 1:-1, 0] = X[:, 2:] - X[:, :-2]
    du[:, 1:-1, 2] = z[:, 2:] - z[:, :-2]
    dv[1:-1, :, 1] = Y[2:, :] - Y[:-2, :]
    dv[1:-1, :, 2] = z[2:, :] - z[:-2, :]
    n = np.cross(du, dv)
    norm = np.linalg.norm(n, axis=2)
    with np.errstate(invalid='ignore', divide='ignore'):
        n = n / np.where(norm > 1e-12, norm, 1.0)[..., None]
    dot = n[..., 0] * X + n[..., 1] * Y + n[..., 2] * z
    flip = dot > 0
    n[flip] = -n[flip]
    jump_u = np.zeros((h, w), bool)
    jump_v = np.zeros((h, w), bool)
    jump_u[:, 1:-1] = (np.abs(depth_roi[:, 2:].astype(np.int32)
                              - depth_roi[:, :-2].astype(np.int32)) > depth_jump_mm)
    jump_v[1:-1, :] = (np.abs(depth_roi[2:, :].astype(np.int32)
                              - depth_roi[:-2, :].astype(np.int32)) > depth_jump_mm)
    nvalid = valid & (norm > 1e-12) & ~jump_u & ~jump_v
    nvalid[[0, -1], :] = False
    nvalid[:, [0, -1]] = False
    return n, nvalid


# ═══════════════════════════════════════════════════════════════
# 合成输入
# ═══════════════════════════════════════════════════════════════

_MIN_DEPTH_M, _MAX_DEPTH_M = 0.3, 1.5
_K = {'fx': 380.0, 'fy': 381.5, 'cx': 317.2, 'cy': 239.8}


def _synthetic_depth(seed: int = 0) -> np.ndarray:
    """12×15 深度图：背景带 + 0 / 65535 / 边界 300mm / 窗内外点 + 抖动."""
    rng = np.random.default_rng(seed)
    depth = rng.integers(600, 1100, size=(12, 15)).astype(np.uint16)
    depth[0, 0] = 0            # 无效：零深度
    depth[0, 1] = 65535        # 无效：饱和
    depth[1, 1] = 300          # 边界 == min（float32 口径下被纳入）
    depth[1, 2] = 250          # 窗内过近
    depth[2, 3] = 1500         # 边界 == max（严格小于 → 排除）
    depth[2, 4] = 2600         # 窗外过远
    depth[3, 5] = 1200         # 窗内（远离边界）
    return depth


def _synthetic_mask(shape) -> np.ndarray:
    rng = np.random.default_rng(7)
    return rng.random(shape) > 0.35


# ═══════════════════════════════════════════════════════════════
# 对拍用例
# ═══════════════════════════════════════════════════════════════


def test_backproject_window_matches_reference_to_points():
    depth = _synthetic_depth()
    mask = _synthetic_mask(depth.shape)
    ref_points, ref_pixels = _ref_to_points(
        depth, mask, 4, 2, _K, _MIN_DEPTH_M, _MAX_DEPTH_M)
    xyz, colors = backproject(
        depth, mask, _K, min_depth_m=_MIN_DEPTH_M, max_depth_m=_MAX_DEPTH_M,
        xoff=4, yoff=2)
    assert colors is None
    assert xyz.shape == ref_points.shape
    np.testing.assert_allclose(xyz, ref_points, rtol=1e-12, atol=1e-15)
    assert ref_points.shape[0] > 0
    # 300/1500mm 为窗边界探针：新旧同式同口径（float32 数组 × python 标量
    # 的比较语义在两侧一致），对拍即证明边界取舍逐位不变，不单独断言方向


def test_new_to_points_reuses_valid_mask_identically():
    depth = _synthetic_depth()
    mask = _synthetic_mask(depth.shape)
    ref_points, ref_pixels = _ref_to_points(
        depth, mask, 0, 0, _K, _MIN_DEPTH_M, _MAX_DEPTH_M)
    pipeline = RobustBagPosePipeline(
        min_depth_m=_MIN_DEPTH_M, max_depth_m=_MAX_DEPTH_M)
    valid = valid_depth_mask(depth, _MIN_DEPTH_M, _MAX_DEPTH_M)
    points, pixels = pipeline._to_points(depth, mask, 0, 0, _K, valid)
    np.testing.assert_allclose(points, ref_points, rtol=1e-12, atol=1e-15)
    np.testing.assert_array_equal(pixels, ref_pixels)


def test_backproject_no_window_keeps_zero_and_saturation_out():
    depth = _synthetic_depth()
    mask = np.ones(depth.shape, dtype=bool)
    xyz, _ = backproject(depth, mask, _K)
    ref_z = depth[(depth > 0) & (depth < 65535)].astype(np.float64) / 1000.0
    assert xyz.shape[0] == ref_z.size
    np.testing.assert_allclose(xyz[:, 2], ref_z, rtol=0, atol=0)


def test_bbox_cloud_matches_reference_all_strides():
    rng = np.random.default_rng(3)
    depth = _synthetic_depth(seed=1)
    rgb = rng.integers(0, 256, size=(*depth.shape, 3), dtype=np.uint8)
    bboxes = [(2, 3, 9, 10), (-4, 2, 5, 11), (10, 8, 40, 30), (6, 4, 6, 9)]
    for stride in (1, 3):
        ref_xyz, ref_rgb = _ref_bbox_cloud_xyzrgb(
            rgb, depth, _K, bboxes, stride=stride)
        xyz, rgb_packed = bbox_cloud_xyzrgb(
            rgb, depth, _K, bboxes, stride=stride)
        assert xyz.shape == ref_xyz.shape
        np.testing.assert_allclose(xyz, ref_xyz, rtol=1e-12, atol=1e-15)
        np.testing.assert_array_equal(rgb_packed, ref_rgb)


def test_backproject_stride_is_global_grid_anchor():
    depth = _synthetic_depth()
    mask = np.ones(depth.shape, dtype=bool)
    xyz_s, _ = backproject(depth, mask, _K, stride=2)
    ref = _ref_bbox_cloud_xyzrgb(
        np.zeros((*depth.shape, 3), np.uint8), depth, _K,
        [(0, 0, depth.shape[1], depth.shape[0])], stride=2)
    np.testing.assert_allclose(xyz_s, ref[0], rtol=1e-12, atol=1e-15)


def test_backproject_rgb_colors_align_with_points():
    depth = _synthetic_depth()
    mask = np.ones(depth.shape, dtype=bool)
    xyz, colors = backproject(depth, mask, _K, rgb_bgr=np.full(
        (*depth.shape, 3), 77, dtype=np.uint8))
    assert colors is not None and colors.shape == (xyz.shape[0], 3)
    assert (colors == 77).all()
    empty_xyz, empty_colors = backproject(
        depth, np.zeros(depth.shape, bool), _K,
        rgb_bgr=np.zeros((*depth.shape, 3), dtype=np.uint8))
    assert empty_xyz.shape == (0, 3)
    assert empty_colors.shape == (0, 3)


def test_estimate_normals_matches_reference():
    depth = _synthetic_depth(seed=2)
    ref_n, ref_valid = _ref_estimate_normals(depth, 5, 3, _K, depth_jump_mm=30)
    n, nvalid = estimate_normals(depth, 5, 3, _K, depth_jump_mm=30)
    np.testing.assert_allclose(n, ref_n, rtol=1e-12, atol=1e-15)
    np.testing.assert_array_equal(nvalid, ref_valid)


def test_crop_mask_to_bbox_matches_both_legacy_impls():
    rng = np.random.default_rng(11)
    full = rng.random((10, 14)) > 0.5
    for bbox in [(2, 3, 8, 9), (-3, -2, 6, 7), (9, 7, 30, 25), (0, 0, 14, 10)]:
        x1, y1, x2, y2 = (int(v) for v in bbox)
        # 旧 pipeline 版：全图清零式
        legacy_pipeline = full.copy()
        legacy_pipeline[:max(y1, 0), :] = 0
        legacy_pipeline[max(y2, 0):, :] = 0
        legacy_pipeline[:, :max(x1, 0)] = 0
        legacy_pipeline[:, max(x2, 0):] = 0
        keep = crop_mask_to_bbox(full, bbox)
        rebuilt = np.zeros_like(full)
        rebuilt[max(y1, 0):max(y2, 0), max(x1, 0):max(x2, 0)] = keep
        np.testing.assert_array_equal(rebuilt, legacy_pipeline)
        # 旧 inference 版：ROI 切片式（bbox 先经 clip_bbox 钳到图内）
        cx1, cy1, cx2, cy2 = clip_bbox(bbox, full.shape)
        legacy_slice = full[cy1:cy2, cx1:cx2]
        np.testing.assert_array_equal(keep, legacy_slice)
