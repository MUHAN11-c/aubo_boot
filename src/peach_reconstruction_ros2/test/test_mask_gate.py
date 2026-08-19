"""StrictMaskGate：五道掩膜门逐门锚点 + 直通/通过路径（抽取前节点内联语义）."""
import unittest

import numpy as np

from peach_reconstruction_ros2.interfaces import MaskGate
from peach_reconstruction_ros2.mask_gate import (
    GateResult,
    MaskContext,
    StrictMaskGate,
)

_STAMP_NS = 1_234_567_890


def _depth(h=8, w=8, value=1000):
    """全有效 uint16 毫米深度."""
    return np.full((h, w), value, dtype=np.uint16)


def _mask(h=8, w=8, pixels=64):
    """左上角 pixels 个像素的 mono8 掩膜."""
    mask = np.zeros((h, w), dtype=np.uint8)
    mask.flat[:pixels] = 255
    return mask


def _ctx(mask=None, center=None, bound=None, stamp_ns=_STAMP_NS,
         depth=None, masks=None, neighbors=()):
    """组装 MaskContext；masks 缺省为 {stamp_ns: (mask, center)}."""
    if masks is None:
        masks = {} if mask is None else {stamp_ns: (mask, center)}
    return MaskContext(
        stamp_ns=stamp_ns,
        depth_mm=_depth() if depth is None else depth,
        masks=masks,
        bound_center=bound,
        neighbor_centers=neighbors)


class StrictMaskGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = StrictMaskGate(
            require_target_mask=True, min_mask_pixels=16,
            min_mask_depth_ratio=0.5, max_target_drift_m=0.05)
        self.assertIsInstance(self.gate, MaskGate)

    def test_disabled_passthrough(self):
        """require_target_mask=False：直通 (None, '')，与抽取前语义一致."""
        gate = StrictMaskGate(require_target_mask=False)
        result = gate.check(_ctx(mask=None))
        self.assertIsNone(result.mask)
        self.assertEqual(result.reason, '')
        self.assertTrue(result.passed)

    def test_pass_returns_mask(self):
        """四门全过：返回掩膜本体，reason 为空."""
        mask = _mask()
        result = self.gate.check(_ctx(mask=mask))
        self.assertIs(result.mask, mask)
        self.assertEqual(result.reason, '')
        self.assertTrue(result.passed)

    def test_missing_same_stamp_mask_rejects(self):
        """门 1 同戳：缓存无同时间戳条目 → 拒绝."""
        result = self.gate.check(_ctx(masks={999: (_mask(), None)}))
        self.assertIsNone(result.mask)
        self.assertIn('同时间戳掩膜', result.reason)

    def test_too_few_pixels_rejects(self):
        """门 2 像素数：掩膜 8 像素 < min_mask_pixels=16 → 拒绝并带数值."""
        result = self.gate.check(_ctx(mask=_mask(pixels=8)))
        self.assertIsNone(result.mask)
        self.assertIn('8 像素 < 16', result.reason)

    def test_low_depth_ratio_rejects(self):
        """门 3 有效深度占比：掩膜内深度全 0（占比 0）→ 拒绝."""
        result = self.gate.check(
            _ctx(mask=_mask(), depth=np.zeros((8, 8), dtype=np.uint16)))
        self.assertIsNone(result.mask)
        self.assertIn('有效深度占比', result.reason)

    def test_mask_depth_shape_mismatch_rejects(self):
        """门 3 前置：掩膜与深度尺寸不一致时 ValueError 转拒绝原因，不抛异常."""
        result = self.gate.check(
            _ctx(mask=_mask(h=4, w=4, pixels=16), depth=_depth(h=8, w=8)))
        self.assertIsNone(result.mask)
        self.assertIn('不一致', result.reason)

    def test_drift_rejects(self):
        """门 4 漂移：掩膜中心偏离绑定中心 > max_target_drift_m → 拒绝."""
        bound = np.array([0.0, 0.0, 0.5])
        center = np.array([0.2, 0.0, 0.5])  # 漂移 200 mm > 50 mm
        result = self.gate.check(
            _ctx(mask=_mask(), center=center, bound=bound))
        self.assertIsNone(result.mask)
        self.assertIn('目标漂移', result.reason)

    def test_drift_within_tolerance_passes(self):
        """漂移在容差内放行；bound 或 center 为 None 时漂移门不启用."""
        bound = np.array([0.0, 0.0, 0.5])
        center = np.array([0.03, 0.0, 0.5])  # 漂移 30 mm < 50 mm
        result = self.gate.check(
            _ctx(mask=_mask(), center=center, bound=bound))
        self.assertTrue(result.passed)
        # center None（感知未给中心）或 bound None（未绑定中心）均放行
        self.assertTrue(self.gate.check(
            _ctx(mask=_mask(), center=None, bound=bound)).passed)
        self.assertTrue(self.gate.check(
            _ctx(mask=_mask(), center=center, bound=None)).passed)

    def test_neighbor_too_close_rejects(self):
        """门 5 邻目标串扰：最近邻目标锚点间距 < min_neighbor_gap_m → 拒绝."""
        gate = StrictMaskGate(
            require_target_mask=True, min_mask_pixels=16,
            min_mask_depth_ratio=0.5, max_target_drift_m=0.05,
            min_neighbor_gap_m=0.15)
        bound = np.array([0.0, 0.0, 0.5])
        # 邻目标距绑定锚点 100 mm < 150 mm：拒帧防 TSDF 双层表面（I6）
        result = gate.check(_ctx(
            mask=_mask(), bound=bound,
            neighbors=(np.array([0.1, 0.0, 0.5]),)))
        self.assertIsNone(result.mask)
        self.assertIn('防串扰拒帧', result.reason)

    def test_neighbor_gate_disabled_and_distant_cases(self):
        """门 5 边界：间距达标放行；无邻目标/未绑定/参数 ≤0 时本门不启用."""
        gate = StrictMaskGate(
            require_target_mask=True, min_mask_pixels=16,
            min_mask_depth_ratio=0.5, max_target_drift_m=0.05,
            min_neighbor_gap_m=0.15)
        bound = np.array([0.0, 0.0, 0.5])
        # 间距 200 mm ≥ 150 mm：放行
        self.assertTrue(gate.check(_ctx(
            mask=_mask(), bound=bound,
            neighbors=(np.array([0.2, 0.0, 0.5]),))).passed)
        # 多个邻目标取最近者判定；最远近邻达标即放行
        self.assertTrue(gate.check(_ctx(
            mask=_mask(), bound=bound,
            neighbors=(np.array([0.2, 0.0, 0.5]),
                       np.array([0.5, 0.0, 0.5])))).passed)
        # 邻目标列表为空：本门不启用
        self.assertTrue(gate.check(
            _ctx(mask=_mask(), bound=bound, neighbors=())).passed)
        # 未绑定中心（bound None）：本门不启用
        self.assertTrue(gate.check(_ctx(
            mask=_mask(), bound=None,
            neighbors=(np.array([0.0, 0.0, 0.5]),))).passed)
        # min_neighbor_gap_m=0：整体关闭（贴脸邻目标也放行）
        off_gate = StrictMaskGate(
            require_target_mask=True, min_mask_pixels=16,
            min_mask_depth_ratio=0.5, max_target_drift_m=0.05,
            min_neighbor_gap_m=0.0)
        self.assertTrue(off_gate.check(_ctx(
            mask=_mask(), bound=bound,
            neighbors=(np.array([0.01, 0.0, 0.5]),))).passed)

    def test_gate_result_is_frozen_dataclass(self):
        """结果对象为 frozen 值对象（passed 属性 = reason 为空）."""
        self.assertTrue(GateResult(None, '').passed)
        self.assertFalse(GateResult(None, 'x').passed)


if __name__ == '__main__':
    unittest.main()
