"""ICP target 增量复用缓存（E4）：刷新节奏/自适应 k/增量拼接/关键事件复位."""
import importlib.util
import unittest

import numpy as np

from peach_reconstruction_ros2.icp_target_cache import (
    IcpTargetCache,
    IcpTargetRefreshConfig,
)

_HAS_OPEN3D = importlib.util.find_spec('open3d') is not None


def _cloud(n, seed=0, scale=1.0):
    """造 (n,3) 确定性伪随机点云（固定种子，scale 控制空间范围 [m]）."""
    return np.random.default_rng(seed).uniform(
        0.0, 0.05 * scale, size=(n, 3))


class IcpTargetCacheCadenceTest(unittest.TestCase):
    """刷新节奏：空缓存必刷新、append 推进计数、满 k 帧触发刷新."""

    def test_initial_state_requires_refresh(self):
        """初始（空缓存）：should_refresh 恒 True，周期保守起步=下限."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=2, max_period=4))
        self.assertIsNone(cache.current_target())
        self.assertTrue(cache.should_refresh())
        self.assertEqual(cache.period, 2)
        self.assertEqual(cache.target_size, 0)

    def test_full_then_incremental_cadence(self):
        """set_full 复位计数；每 append 一帧计数 +1；满 period 帧再触发刷新."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=2, max_period=4))
        cache.set_full(_cloud(100))
        self.assertFalse(cache.should_refresh())
        self.assertEqual(cache.full_refreshes, 1)
        cache.append_frame(_cloud(50, seed=1))
        self.assertFalse(cache.should_refresh())  # 第 1 帧 < period=2
        cache.append_frame(_cloud(50, seed=2))
        self.assertTrue(cache.should_refresh())   # 第 2 帧达到 period
        cache.set_full(_cloud(120, seed=3))
        self.assertFalse(cache.should_refresh())  # 全量刷新后计数归零
        self.assertEqual(cache.target_size, 120)

    def test_empty_extract_keeps_cache_empty(self):
        """全量提取为空云（模型未形成）：缓存保持空，下帧仍要求刷新."""
        cache = IcpTargetCache(IcpTargetRefreshConfig())
        cache.set_full(np.zeros((0, 3)))
        self.assertIsNone(cache.current_target())
        self.assertTrue(cache.should_refresh())

    def test_append_merges_points(self):
        """增量拼接：target = 全量基线 + 已采帧修正后云."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=8))
        cache.set_full(_cloud(100))
        cache.append_frame(_cloud(50, seed=1))
        cache.append_frame(_cloud(30, seed=2))
        self.assertEqual(cache.target_size, 180)
        self.assertEqual(cache.incremental_appends, 2)


class IcpTargetCacheAdaptiveTest(unittest.TestCase):
    """自适应 k：稳定拉长、漂移/回退/拒帧收回、迟滞带保持."""

    @staticmethod
    def _stretch_to_max(cache, n=5):
        """喂小修正量 ICP 样本使周期拉长到上限（EMA 迅速收敛到稳定区）."""
        for _ in range(n):
            cache.note_result('icp', 0.0005)  # 0.5mm ≪ 1/4×漂移阈值 1.25mm

    def test_stable_corrections_stretch_period(self):
        """修正量 EMA 长期 ≤1/4 阈值（稳定）→ k 拉长到上限."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=5, drift_ratio=0.5,
            max_translation_m=0.010))  # 漂移阈值 5mm
        self._stretch_to_max(cache)
        self.assertEqual(cache.period, 5)

    def test_drift_shrinks_period(self):
        """修正量 EMA 爬升超过漂移阈值（5mm）→ k 收回下限."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=5, drift_ratio=0.5,
            max_translation_m=0.010))
        self._stretch_to_max(cache)
        self.assertEqual(cache.period, 5)
        for _ in range(10):
            cache.note_result('icp', 0.006)  # 6mm 持续样本，EMA 终会越阈
        self.assertEqual(cache.period, 1)

    def test_fk_fallback_and_reject_shrink_immediately(self):
        """FK 回退/拒帧 = 对齐风险信号：k 立即收回下限，不混入 EMA."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=5, drift_ratio=0.5,
            max_translation_m=0.010))
        self._stretch_to_max(cache)
        cache.note_result('fk', 0.0)
        self.assertEqual(cache.period, 1)
        self._stretch_to_max(cache)
        cache.note_result('reject', 0.02)
        self.assertEqual(cache.period, 1)

    def test_hysteresis_band_keeps_period(self):
        """EMA 落在 (1/4, 1)×阈值迟滞带：周期保持现值不来回振荡."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=5, drift_ratio=0.5,
            max_translation_m=0.010))  # 阈值 5mm，迟滞带 (1.25, 5)mm
        self._stretch_to_max(cache)
        for _ in range(3):
            cache.note_result('icp', 0.003)  # 3mm 落在迟滞带内
        self.assertEqual(cache.period, 5)    # 不收也不再放，保持

    def test_invalidate_resets_all(self):
        """关键事件复位：缓存/计数/EMA/周期全部回保守初值."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=5, drift_ratio=0.5,
            max_translation_m=0.010))
        cache.set_full(_cloud(100))
        self._stretch_to_max(cache)
        cache.append_frame(_cloud(50, seed=1))
        cache.invalidate()
        self.assertIsNone(cache.current_target())
        self.assertTrue(cache.should_refresh())
        self.assertEqual(cache.period, 1)
        # 复位后需重新攒稳定证据才会拉长
        cache.set_full(_cloud(100, seed=2))
        self.assertEqual(cache.period, 1)

    def test_config_bounds_normalized(self):
        """上限<下限的配置错误归一为 max=min（不抛错），行为退化为定周期."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=3, max_period=1))
        self.assertEqual(cache.period, 3)
        self._stretch_to_max(cache)
        self.assertEqual(cache.period, 3)  # max 被钳到 min，拉长无效


class IcpTargetCacheBoundTest(unittest.TestCase):
    """增量拼接容量护栏：超限按体素降采样（需 open3d，colcon 环境跳过）."""

    @unittest.skipUnless(_HAS_OPEN3D, 'open3d 不可用（colcon 系统 python）')
    def test_incremental_bound_downsamples(self):
        """拼接超过 max_incremental_points → 体素降采样收敛容量."""
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=1, max_period=8, max_incremental_points=200,
            downsample_voxel=0.01))
        # 点云全部落在 0.05m 立方内：1cm 体素至多 5×5×5=125 个，
        # 降采样后必 ≤125 < 200
        cache.set_full(_cloud(150))
        cache.append_frame(_cloud(150, seed=1))
        self.assertLessEqual(cache.target_size, 200)
        self.assertGreater(cache.target_size, 0)


if __name__ == '__main__':
    unittest.main()
