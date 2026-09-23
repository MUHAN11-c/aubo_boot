# F1：掩膜门最近邻 stamp 容差配对（批次1，2026-09-23 重构）。
# 高帧深度源（stereo 13.6fps）≫ 掩膜源（感知 ~7.5fps）时精确查表必失配；
# 容差内最近邻配对须命中，容差外/空缓存仍拒并带归因。
import numpy as np

from peach_harvester.vision.target_reconstruction.capture import (
    MaskContext, StrictMaskGate,
)


def _ctx(stamp_ns, mask_stamps, mask=None):
    mask = np.zeros((4, 4), dtype=np.uint8) if mask is None else mask
    mask[1:3, 1:3] = 1  # 4 像素 < min_mask_pixels 故意低，测的是 stamp 匹配层
    return MaskContext(
        stamp_ns=stamp_ns,
        depth_mm=np.full((4, 4), 1000, dtype=np.uint16),
        masks={s: (mask, None) for s in mask_stamps},
        bound_center=None,
        neighbor_centers=(),
    )


def test_exact_stamp_still_hits():
    gate = StrictMaskGate(require_target_mask=True, min_mask_pixels=1,
                          min_mask_depth_ratio=0.0, mask_stamp_tolerance_s=0.0)
    res = gate.check(_ctx(1_000_000_000, [1_000_000_000]))
    assert res.reason != '缺少所选 target_id 的容差内掩膜'


def test_nearest_within_tolerance_pairs():
    gate = StrictMaskGate(require_target_mask=True, min_mask_pixels=1,
                          min_mask_depth_ratio=0.0, mask_stamp_tolerance_s=0.08)
    # 深度帧 1.000s；掩膜最近 1.060s（Δ60ms ≤ 80ms）→ 配对成功（非 stamp 拒）
    res = gate.check(_ctx(1_000_000_000, [860_000_000, 1_060_000_000]))
    assert res.reason != '缺少所选 target_id 的容差内掩膜'


def test_outside_tolerance_still_rejects_with_reason():
    gate = StrictMaskGate(require_target_mask=True, min_mask_pixels=1,
                          min_mask_depth_ratio=0.0, mask_stamp_tolerance_s=0.08)
    res = gate.check(_ctx(1_000_000_000, [1_200_000_000]))
    assert res.reason == '缺少所选 target_id 的容差内掩膜'
    assert res.mask is None


def test_zero_tolerance_restores_strict_lookup():
    gate = StrictMaskGate(require_target_mask=True, min_mask_pixels=1,
                          min_mask_depth_ratio=0.0, mask_stamp_tolerance_s=0.0)
    res = gate.check(_ctx(1_000_000_000, [1_060_000_000]))
    assert res.reason == '缺少所选 target_id 的容差内掩膜'


def test_picks_nearest_when_multiple_in_window():
    gate = StrictMaskGate(require_target_mask=True, min_mask_pixels=1,
                          min_mask_depth_ratio=0.0, mask_stamp_tolerance_s=0.08)
    # ±都窗内：取 Δ 更近者（40ms 优于 70ms）；断言不因多候选误拒
    res = gate.check(_ctx(1_000_000_000, [930_000_000, 1_040_000_000]))
    assert res.reason != '缺少所选 target_id 的容差内掩膜'
