"""
单帧门语义修正回归（2026-09-21 第二轮调参迭代）.

锁三处修正：①valid_ratio 目标级口径（|SAM∩valid|/|SAM|，替代被检测框
背景稀释的 ROI 均值）；②信息类 flags 不再压 ACCEPT（门控/信息分离）；
③长度塌缩显式化门 axis_length_consistent。
"""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.common.geometry import backproject
from peach_harvester.vision.scene_perception.contracts import (
    BagObservation,
    TOOL_GEOMETRY,
)
from peach_harvester.vision.scene_perception.image_gates import valid_depth_mask
from peach_harvester.vision.scene_perception.inference import CandidateEstimator
from peach_harvester.vision.scene_perception.pose_pipelines import (
    _gating_flags,
    axis_length_consistent,
    RobustBagPosePipeline,
)

_K = {'fx': 500.0, 'fy': 500.0, 'cx': 32.0, 'cy': 24.0,
      'width': 64, 'height': 48}


def _bench_depth() -> np.ndarray:
    """64×48 深度：目标 700mm 竖圆柱带 + 背景 2200mm（窗外）."""
    depth = np.full((48, 64), 2200, dtype=np.uint16)
    depth[8:40, 24:40] = 700
    return depth


def _target_mask() -> np.ndarray:
    """与目标圆柱带重合的 SAM 掩膜（全图尺寸）."""
    mask = np.zeros((48, 64), dtype=bool)
    mask[8:40, 24:40] = True
    return mask


def _obs() -> BagObservation:
    return BagObservation(
        rgb=None, depth=_bench_depth(), camera_K=dict(_K),
        frame_id='cam', gravity_hint=np.array([0.0, 1.0, 0.0]),
        detections=[{'class_id': 0, 'conf': 0.9, 'bbox': (16, 2, 48, 46)}],
        metadata={})


def test_build_masks_reports_sam_yield():
    """sam_yield=|SAM∩valid|/|SAM|：背景超窗不再稀释目标级占比."""
    est = CandidateEstimator(
        pipeline=RobustBagPosePipeline(tool=TOOL_GEOMETRY,
                                       min_depth_m=0.3, max_depth_m=1.5,
                                       min_points=100),
        min_mask_points=50)
    obs = _obs()
    masks, valid, sam_yield, _raw = est.build_masks(
        obs, (16, 2, 48, 46), _target_mask())
    assert masks['hybrid_dilated'] is not None
    # ROI 均值口径会被 2200mm 背景压到 ~0.3；目标级应接近 1.0
    roi = obs.depth[2:46, 16:48]
    roi_mean = float(valid_depth_mask(roi, 0.3, 1.5).mean())
    assert roi_mean < 0.5
    assert sam_yield is not None and sam_yield > 0.95
    assert sam_yield > roi_mean


def test_valid_ratio_target_semantics_lifts_confidence():
    """目标级 valid_ratio 进 confidence/σ：良测目标不再被背景压垮."""
    obs = _obs()
    pipe = RobustBagPosePipeline(tool=TOOL_GEOMETRY, min_depth_m=0.3,
                                 max_depth_m=1.5, min_points=100)
    bbox = (16, 2, 48, 46)
    mask = _target_mask()[2:46, 16:48]
    new = pipe.estimate(obs, 't', bbox, mask, 'mobile_sam',
                        target_valid_ratio=0.95)
    legacy = pipe.estimate(obs, 't', bbox, mask, 'mobile_sam')
    assert float(new.grasp_3d.confidence) > float(legacy.grasp_3d.confidence)
    assert new.grasp_3d.diagnostic_info['valid_depth_ratio'] > 0.9
    assert legacy.grasp_3d.diagnostic_info['valid_depth_ratio'] < 0.5


def test_informational_flags_do_not_block_accept():
    """
    良态锥形袋：仅信息类 flags 在列时单帧门给 ACCEPT（分区后可达）.

    合成竖直锥形袋（底粗顶窄、长 ~11cm、深度噪声 1.5mm）走完整
    estimate_modes 链（build_masks hybrid + landmarks），历史上该形态
    被 fruit_prior_auxiliary/taper_* 等过程 flags 一律压成 REOBSERVE
    （生产 56/56 恒 REOBSERVE、refine ACCEPT 偏好死码）。
    """
    K = dict(_K)
    depth = np.full((48, 64), 2200, dtype=np.uint16)
    rng = np.random.default_rng(3)
    # 短袋（z 跨 0.09m）：融合补全后 travel~0.075m，20° 默认轴不确定度
    # 预算 ~26mm < 30mm 径向余量——保证本测试只考「信息 flag 不压门」，
    # 不被误差预算门反客为主
    for j in range(48):
        t = (j - 6) / 28.0
        if not 0.0 <= t <= 1.0:
            continue
        z = (0.62 + 0.09 * t) * 1000
        r_px = 17 - 9 * t
        for i in range(64):
            if (i - 32) ** 2 <= r_px ** 2:
                depth[j, i] = int(z + rng.normal(0, 1.5))
    mask = np.zeros((48, 64), dtype=bool)
    mask[6:35, 13:51] = True
    bbox = (13, 5, 51, 36)
    obs = BagObservation(
        rgb=None, depth=depth, camera_K=K, frame_id='cam',
        gravity_hint=np.array([0.0, 1.0, 0.0]),
        detections=[{'class_id': 0, 'conf': 0.9, 'bbox': bbox}], metadata={})
    est = CandidateEstimator(
        pipeline=RobustBagPosePipeline(tool=TOOL_GEOMETRY,
                                       min_depth_m=0.3, max_depth_m=1.5,
                                       min_points=100),
        min_mask_points=50)
    result = est.estimate_modes(obs, 't', bbox, mask)['hybrid_dilated']
    flags = sorted(set(result.grasp_3d.diagnostic_flags))
    informational = [f for f in flags if f not in _gating_flags(flags)]
    # 过程类 flags 照常在列（可追溯），但不参与门
    assert 'fruit_prior_auxiliary' in informational
    assert _gating_flags(flags) == []
    assert result.grasp_3d.status == 'ACCEPT'


def test_gating_partition_classifies_known_flags():
    """分区表抽查：信息/门控各类代表 flag 归类正确."""
    informational = {
        'axis_from_pca', 'taper_neck', 'taper_polarity_swapped',
        'polarity_upper_hemisphere', 'taper_lower_hemisphere_ignored',
        'fruit_prior_auxiliary', 'neck_from_band', 'bottom_from_band',
        'gravity_defaulted', 'axis_from_profile_sign',
    }
    gating = {
        'low_valid_depth', 'small_foreground', 'tool_clearance_failed',
        'travel_too_short', 'axis_from_gravity_prior',
        'axis_orientation_uncertain', 'axis_2d_mismatch',
        'error_budget_exceeded', 'foreground_truncated',
        'unbagged_display_only', 'tf_stale', 'axis_length_inconsistent',
    }
    assert _gating_flags(sorted(informational)) == []
    assert _gating_flags(sorted(gating)) == sorted(gating)


def test_axis_length_consistent_detects_collapse():
    """长度塌缩门：3D 长度 < 2D 粗测一半且粗测 >8cm 判不一致."""
    # 200px 展程 @700mm/fx500 ⟹ ~0.30m 粗测；3D 长度 0.05m ⟹ 塌缩
    assert not axis_length_consistent(0.05, 120, 160, 0.70, 500.0)
    # 一致：3D 长度与粗测同量级
    assert axis_length_consistent(0.20, 120, 160, 0.70, 500.0)
    # 粗测本身退化（<8cm）时无判别力，不判
    assert axis_length_consistent(0.02, 20, 30, 0.70, 500.0)
    # 深度/焦距非法时无判别力
    assert axis_length_consistent(0.05, 120, 160, 0.0, 500.0)


def test_backproject_still_feeds_points_for_bench_target():
    """基准目标可反投影（前置一致性：掩膜∩valid 有点且窗内）."""
    depth = _bench_depth()
    mask = _target_mask()
    valid = valid_depth_mask(depth, 0.3, 1.5)
    pts, _ = backproject(depth, mask & valid, _K)
    assert len(pts) > 500
    assert 0.6 < float(np.median(pts[:, 2])) < 0.8


def test_bbox_fusion_recovers_depth_holed_bag_length():
    """
    2D+3D 融合：上半段深度空洞时，袋长按掩膜剪影/检测框补全.

    合成锥形袋（rows 6-40、z 0.62→0.73、物理轴长 ~0.11m）；上半段深度
    置 0（空洞）。hybrid 点云只剩下半段（旧口径 fit_len ~0.06m 系统性
    偏短），原始 SAM 剪影经检测框限幅补全后 fit_len 应恢复到接近全长。
    """
    K = dict(_K)
    depth = np.full((48, 64), 2200, dtype=np.uint16)
    rng = np.random.default_rng(3)
    for j in range(48):
        t = (j - 6) / 34.0
        if not 0.0 <= t <= 1.0:
            continue
        z = (0.62 + 0.11 * t) * 1000
        r_px = 17 - 9 * t
        for i in range(64):
            if (i - 32) ** 2 <= r_px ** 2:
                depth[j, i] = int(z + rng.normal(0, 1.5))
    depth[6:20, :] = 0  # 上半段深度空洞（RGB 可见、深度缺失）
    sam_mask = np.zeros((48, 64), dtype=bool)
    sam_mask[6:41, 13:51] = True  # 原始剪影：覆盖整段（含空洞区）
    bbox = (13, 4, 51, 43)
    obs = BagObservation(
        rgb=None, depth=depth, camera_K=K, frame_id='cam',
        gravity_hint=np.array([0.0, 1.0, 0.0]),
        detections=[{'class_id': 0, 'conf': 0.9, 'bbox': bbox}], metadata={})
    est = CandidateEstimator(
        pipeline=RobustBagPosePipeline(tool=TOOL_GEOMETRY,
                                       min_depth_m=0.3, max_depth_m=1.5,
                                       min_points=100),
        min_mask_points=50)
    result = est.estimate_modes(obs, 't', bbox, sam_mask)['hybrid_dilated']
    length = float(np.linalg.norm(
        np.asarray(result.grasp_3d.bag_neck)
        - np.asarray(result.grasp_3d.bag_bottom)))
    # 旧口径（点云可见段）≈ 0.06m；融合后应接近物理全长 0.11m
    assert length >= 0.085, length
    assert 'length_extended_from_2d' in result.grasp_3d.diagnostic_flags
