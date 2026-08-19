"""
TimingStats 耗时累计器单测（纯核，合成样本路径）.

覆盖阶段 C 埋点契约：
  - timing 子对象键集恒定（diagnostics JSON / session metadata 共用）；
  - 初始快照全部数值非负、frames_timed=0（未采样投影 0.0）；
  - EMA（α=0.3）播种与递推语义；负样本钳位到 0；
  - refit/finalize 只记 last 值（不做 EMA）；
  - frames_timed 仅随 record_frame_total 递增。
说明：现有测试框架为纯核层（无 rclpy/open3d 节点级用例），节点接线
（_accept_frame/_finalize_now/_run_refit 打点）由本类契约 + 节点侧
代码审查保证；此处用合成耗时样本驱动与节点相同的 record_* 调用序列。
"""
import json
import unittest

from peach_reconstruction_ros2.timing import EMA_ALPHA, TimingStats

# 契约键集（与 timing.py 模块 docstring、diagnostics timing 子对象一致）
_TIMING_KEYS = {
    'icp_ms_ema',
    'tsdf_integrate_ms_ema',
    'frame_total_ms_ema',
    'refit_ms_last',
    'finalize_ms_last',
    'frames_timed',
}


class TimingStatsTest(unittest.TestCase):
    def test_initial_snapshot_keys_and_nonnegative(self):
        """初始快照：契约键集完整、全部数值非负、计数为 0、可 JSON 序列化."""
        snap = TimingStats().snapshot()
        self.assertEqual(set(snap), _TIMING_KEYS)
        for key, value in snap.items():
            self.assertGreaterEqual(value, 0, f'{key} 应非负')
        self.assertEqual(snap['frames_timed'], 0)
        self.assertEqual(snap['icp_ms_ema'], 0.0)
        self.assertEqual(snap['tsdf_integrate_ms_ema'], 0.0)
        self.assertEqual(snap['frame_total_ms_ema'], 0.0)
        # diagnostics_debug 走 json.dumps，键值必须原生可序列化
        json.dumps(snap)

    def test_ema_seeding_and_recursion(self):
        """EMA：首样本直接播种，其后按 α·sample + (1−α)·old 递推."""
        stats = TimingStats()
        stats.record_icp(10.0)
        self.assertAlmostEqual(stats.snapshot()['icp_ms_ema'], 10.0)
        stats.record_icp(20.0)
        expected = EMA_ALPHA * 20.0 + (1.0 - EMA_ALPHA) * 10.0
        self.assertAlmostEqual(stats.snapshot()['icp_ms_ema'], expected)
        # 再喂一个样本验证递推链不断
        stats.record_icp(30.0)
        expected = EMA_ALPHA * 30.0 + (1.0 - EMA_ALPHA) * expected
        self.assertAlmostEqual(stats.snapshot()['icp_ms_ema'], expected)

    def test_synthetic_frame_sequence(self):
        """合成两帧序列：分项 EMA 各自独立，frames_timed 只随成功帧递增."""
        stats = TimingStats()
        # 帧 1：ICP 12ms + TSDF 40ms + 总 80ms（成功收帧）
        stats.record_icp(12.0)
        stats.record_tsdf_integrate(40.0)
        stats.record_frame_total(80.0)
        # 帧 2：ICP 8ms + TSDF 20ms + 总 50ms
        stats.record_icp(8.0)
        stats.record_tsdf_integrate(20.0)
        stats.record_frame_total(50.0)
        snap = stats.snapshot()
        self.assertEqual(snap['frames_timed'], 2)
        self.assertAlmostEqual(
            snap['tsdf_integrate_ms_ema'],
            EMA_ALPHA * 20.0 + (1.0 - EMA_ALPHA) * 40.0)
        self.assertAlmostEqual(
            snap['frame_total_ms_ema'],
            EMA_ALPHA * 50.0 + (1.0 - EMA_ALPHA) * 80.0)
        for key, value in snap.items():
            self.assertGreaterEqual(value, 0, f'{key} 应非负')

    def test_refit_and_finalize_are_last_values(self):
        """refit/finalize 记 last 值：直接覆盖，不做 EMA 平滑."""
        stats = TimingStats()
        stats.record_refit(150.0)
        stats.record_finalize(900.0)
        stats.record_refit(120.0)
        stats.record_finalize(700.0)
        snap = stats.snapshot()
        self.assertAlmostEqual(snap['refit_ms_last'], 120.0)
        self.assertAlmostEqual(snap['finalize_ms_last'], 700.0)
        # last 记录不影响帧计数与分项 EMA
        self.assertEqual(snap['frames_timed'], 0)
        self.assertEqual(snap['icp_ms_ema'], 0.0)

    def test_negative_samples_clamped_to_zero(self):
        """防御：负耗时样本钳位到 0（时钟回拨/异常打点不污染非负契约）."""
        stats = TimingStats()
        stats.record_icp(-5.0)
        stats.record_refit(-1.0)
        snap = stats.snapshot()
        self.assertEqual(snap['icp_ms_ema'], 0.0)
        self.assertEqual(snap['refit_ms_last'], 0.0)


if __name__ == '__main__':
    unittest.main()
