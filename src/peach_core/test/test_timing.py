"""
RateEstimator / AdaptiveTimeout 全分支测试（协议 I3/I4）.

C++ 锚点值取自 peach_approach_grasp/test/test_safety_gate.cpp 的
adaptive_timeout_s 用例，保证 Python 统一实现与现网数值一致。
"""
import unittest

from peach_core.timing import AdaptiveTimeout, RateEstimator


class RateEstimatorTest(unittest.TestCase):
    """帧间隔 EMA：空样本/首样本/混合/异常过滤/参数校验."""

    def test_no_sample_is_none_safe(self):
        """无任何 update：interval/rate_hz 均为 None（None 安全）."""
        est = RateEstimator()
        self.assertIsNone(est.interval)
        self.assertIsNone(est.rate_hz)

    def test_first_update_only_records_clock(self):
        """首帧只记录时刻，不产生间隔样本."""
        est = RateEstimator()
        est.update(10.0)
        self.assertIsNone(est.interval)
        self.assertIsNone(est.rate_hz)

    def test_first_valid_interval_seeds_ema(self):
        """首个合法间隔直接作 EMA 初值（沿用现网 ema if None 纪律）."""
        est = RateEstimator()
        est.update(10.0)
        est.update(10.2)
        self.assertAlmostEqual(est.interval, 0.2)
        self.assertAlmostEqual(est.rate_hz, 5.0)

    def test_ema_blends_with_alpha(self):
        """默认 α=0.3：ema = 0.7×旧 + 0.3×新."""
        est = RateEstimator()
        est.update(0.0)
        est.update(0.2)   # ema = 0.2
        est.update(0.3)   # dt=0.1 → ema = 0.7×0.2 + 0.3×0.1 = 0.17
        self.assertAlmostEqual(est.interval, 0.17)

    def test_sub_millisecond_interval_rejected(self):
        """间隔 ≤1ms（同帧重复/时钟噪声）不进 EMA."""
        est = RateEstimator()
        est.update(0.0)
        est.update(0.0005)
        self.assertIsNone(est.interval)

    def test_pause_gap_rejected_but_rearms(self):
        """间隔 >30s（暂停后首帧）不进 EMA，但时刻更新、下一帧恢复有效."""
        est = RateEstimator()
        est.update(0.0)
        est.update(0.2)     # ema = 0.2
        est.update(100.0)   # 99.8s 间隔：丢弃
        self.assertAlmostEqual(est.interval, 0.2)
        est.update(100.5)   # 暂停后 0.5s 间隔：进 EMA
        self.assertAlmostEqual(est.interval, 0.7 * 0.2 + 0.3 * 0.5)

    def test_custom_alpha(self):
        """自定义 α 生效."""
        est = RateEstimator(alpha=0.5)
        est.update(0.0)
        est.update(0.2)
        est.update(0.6)     # dt=0.4 → ema = 0.5×0.2 + 0.5×0.4 = 0.3
        self.assertAlmostEqual(est.interval, 0.3)

    def test_invalid_alpha_raises(self):
        """α 越界抛 ValueError."""
        with self.assertRaises(ValueError):
            RateEstimator(alpha=0.0)
        with self.assertRaises(ValueError):
            RateEstimator(alpha=1.5)

    def test_invalid_filter_window_raises(self):
        """异常过滤区间非法抛 ValueError."""
        with self.assertRaises(ValueError):
            RateEstimator(min_interval_s=30.0, max_interval_s=1e-3)


class AdaptiveTimeoutTest(unittest.TestCase):
    """clamp(下限, factor×ema+offset, 上限) 全分支 + 现网锚点值."""

    def test_none_interval_falls_back_to_upper(self):
        """无实测（ema 未测得）返回 upper（回退档=配置上限）."""
        timeout = AdaptiveTimeout(lower=2.0, upper=6.0, factor=4.0, offset=1.0)
        self.assertEqual(timeout.value(None), 6.0)

    def test_frame_wait_anchor_clamps_to_floor(self):
        """现网锚点（C++ test_safety_gate）：4×0.2+1.0=1.8 → 夹到 2.0."""
        timeout = AdaptiveTimeout(lower=2.0, upper=6.0, factor=4.0, offset=1.0)
        self.assertAlmostEqual(timeout.value(0.2), 2.0)

    def test_frame_wait_anchor_clamps_to_cap(self):
        """现网锚点：4×1.28+1.0=6.12 → 夹到配置上限 6.0."""
        timeout = AdaptiveTimeout(lower=2.0, upper=6.0, factor=4.0, offset=1.0)
        self.assertAlmostEqual(timeout.value(1.28), 6.0)

    def test_target_age_anchor_in_range(self):
        """现网锚点：2.5×1.28+0.5=3.7（区间内原值通过）."""
        timeout = AdaptiveTimeout(
            lower=1.0, upper=10.0, factor=2.5, offset=0.5)
        self.assertAlmostEqual(timeout.value(1.28), 3.7)

    def test_target_age_anchor_clamps_to_floor(self):
        """现网锚点：2.5×0.2+0.5=1.0 → 恰在下限."""
        timeout = AdaptiveTimeout(
            lower=1.0, upper=10.0, factor=2.5, offset=0.5)
        self.assertAlmostEqual(timeout.value(0.2), 1.0)

    def test_in_range_value_passes_through(self):
        """区间内：factor×ema+offset 原值返回."""
        timeout = AdaptiveTimeout(lower=2.0, upper=6.0, factor=4.0, offset=1.0)
        self.assertAlmostEqual(timeout.value(0.5), 3.0)

    def test_default_offset_is_zero(self):
        """偏移默认 0（max_collect 用法：(min+settle+3)×ema）."""
        timeout = AdaptiveTimeout(
            lower=0.4 * 25.0, upper=float('inf'), factor=18.0)
        self.assertAlmostEqual(timeout.value(2.0), 36.0)

    def test_lower_above_upper_raises(self):
        """下限大于上限抛 ValueError."""
        with self.assertRaises(ValueError):
            AdaptiveTimeout(lower=6.0, upper=2.0, factor=4.0)

    def test_negative_factor_raises(self):
        """倍率为负抛 ValueError."""
        with self.assertRaises(ValueError):
            AdaptiveTimeout(lower=1.0, upper=2.0, factor=-1.0)


if __name__ == '__main__':
    unittest.main()
