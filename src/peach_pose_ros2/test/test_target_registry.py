"""TargetRegistry：身份记忆（新发/复用/半径/类别/EMA/同帧去重/淘汰/轴归一化/恢复匹配/确认机制）."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.target_registry import TargetRegistry


class TargetRegistryTest(unittest.TestCase):
    def setUp(self):
        # 半径 0.06 m、EMA α=0.3，与节点默认参数一致
        self.reg = TargetRegistry(match_radius=0.06, max_targets=50,
                                  position_ema=0.3)

    def test_new_candidate_gets_new_id(self):
        """空表首个候选发 target_0，obs_count=1，ID 计入同帧占用."""
        tid, is_new = self.reg.match_or_register(
            [1.0, 0.0, 0.5], class_id=0, status='ACCEPT', now=1.0)
        self.assertEqual(tid, 'target_0')
        self.assertTrue(is_new)
        t = self.reg.get(tid)
        self.assertEqual(t['obs_count'], 1)
        self.assertEqual(t['last_status'], 'ACCEPT')
        np.testing.assert_allclose(t['position'], [1.0, 0.0, 0.5])

    def test_reappear_within_radius_reuses_id_and_grows_obs_count(self):
        """消失后近距离重现：复用原 ID，obs_count 增长，位置 EMA 收敛."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        # 中间隔若干帧（目标消失，注册表不删除），重现于半径内另一点
        self.reg.begin_frame()
        tid1, is_new = self.reg.match_or_register(
            [0.03, 0.0, 1.0], class_id=0, status='REOBSERVE', now=10.0)
        self.assertEqual(tid1, tid0)
        self.assertFalse(is_new)
        t = self.reg.get(tid0)
        self.assertEqual(t['obs_count'], 2)
        self.assertEqual(t['last_seen'], 10.0)
        self.assertEqual(t['last_status'], 'REOBSERVE')
        # EMA: 0.7*[0,0,1] + 0.3*[0.03,0,1] = [0.009, 0, 1]
        np.testing.assert_allclose(t['position'], [0.009, 0.0, 1.0], atol=1e-9)

    def test_far_position_registers_new_id(self):
        """距离超 match_radius：发新 ID，序号单调增不复用."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame()
        tid1, is_new = self.reg.match_or_register([0.5, 0.0, 1.0], class_id=0, now=2.0)
        self.assertEqual(tid0, 'target_0')
        self.assertEqual(tid1, 'target_1')
        self.assertTrue(is_new)
        self.assertEqual(self.reg.stats()['n_targets'], 2)

    def test_different_class_never_matches(self):
        """同类才匹配：class 1 贴脸也不命中 class 0 的表项."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame()
        tid1, is_new = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=1, now=2.0)
        self.assertNotEqual(tid1, tid0)
        self.assertTrue(is_new)
        self.assertEqual(self.reg.get(tid0)['obs_count'], 1)

    def test_position_ema_converges_toward_observations(self):
        """α=0.3 下反复同点观测，位置单调向观测点收敛（方向正确）."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 0.0], class_id=0, now=0.0)
        obs = np.array([0.05, 0.0, 0.0])
        prev_dist = float(np.linalg.norm(self.reg.get(tid)['position'] - obs))
        for k in range(1, 8):
            self.reg.begin_frame()
            self.reg.match_or_register(obs, class_id=0, now=float(k))
            dist = float(np.linalg.norm(self.reg.get(tid)['position'] - obs))
            self.assertLess(dist, prev_dist)
            prev_dist = dist
        self.assertLess(prev_dist, 0.06 * 0.7 ** 7 + 1e-9)

    def test_same_frame_two_candidates_do_not_share_entry(self):
        """同帧双候选：第二个即使更近也不撞已占用表项，发新 ID."""
        self.reg.begin_frame()
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        tid1, is_new = self.reg.match_or_register([0.01, 0.0, 1.0], class_id=0, now=1.0)
        self.assertNotEqual(tid1, tid0)
        self.assertTrue(is_new)
        self.assertEqual(self.reg.get(tid0)['obs_count'], 1)

    def test_max_targets_evicts_least_recently_seen(self):
        """超限淘汰最久未见：新注册挤掉 last_seen 最旧的表项，序号不复用."""
        reg = TargetRegistry(match_radius=0.06, max_targets=2, position_ema=0.3)
        reg.match_or_register([0.0, 0.0, 0.0], class_id=0, now=1.0)   # target_0
        reg.begin_frame()
        reg.match_or_register([1.0, 0.0, 0.0], class_id=0, now=5.0)   # target_1
        # 刷新 target_0 的 last_seen，使 target_1 成为最久未见者
        reg.begin_frame()
        reg.match_or_register([0.0, 0.0, 0.0], class_id=0, now=6.0)
        reg.begin_frame()
        tid, is_new = reg.match_or_register([2.0, 0.0, 0.0], class_id=0, now=7.0)
        self.assertEqual(tid, 'target_2')
        self.assertTrue(is_new)
        self.assertEqual(sorted(reg.target_ids()), ['target_0', 'target_2'])

    def test_axis_ema_result_is_normalized(self):
        """双方都有轴：符号对齐 + EMA 后归一化（结果恒为单位向量，方向收敛）."""
        tid, _ = self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, axis=[0.0, 0.0, 1.0], now=1.0)
        self.reg.begin_frame()
        # 反向轴观测：符号对齐（视作 +Z）后 EMA，不能退化成零向量
        self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, axis=[0.0, 0.0, -1.0], now=2.0)
        ax = self.reg.get(tid)['axis']
        self.assertAlmostEqual(float(np.linalg.norm(ax)), 1.0, places=9)
        self.assertGreater(float(ax[2]), 0.9)
        # 同向扰动观测：EMA 向新轴收敛且仍为单位向量
        self.reg.begin_frame()
        self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, axis=[0.3, 0.0, 1.0], now=3.0)
        ax = self.reg.get(tid)['axis']
        self.assertAlmostEqual(float(np.linalg.norm(ax)), 1.0, places=9)
        self.assertGreater(float(ax[0]), 0.0)

    def test_diameter_zero_observation_does_not_corrupt_ema(self):
        """直径 0（无效观测）不参与 EMA，保留历史值."""
        tid, _ = self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, diameter=0.08, now=1.0)
        self.reg.begin_frame()
        self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, diameter=0.0, now=2.0)
        self.assertAlmostEqual(self.reg.get(tid)['diameter'], 0.08, places=9)

    def test_zero_or_nonfinite_axis_is_treated_as_unavailable(self):
        """退化轴不能进入注册表；契约保持 axis 为单位向量或 None."""
        tid, _ = self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, axis=[0.0, 0.0, 0.0], now=1.0)
        self.assertIsNone(self.reg.get(tid)['axis'])
        self.reg.begin_frame()
        self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0,
            axis=[np.nan, 0.0, 1.0], now=2.0)
        self.assertIsNone(self.reg.get(tid)['axis'])

    def test_nonfinite_position_or_time_is_rejected(self):
        """NaN/Inf 不得污染距离匹配与 LRU 淘汰所依赖的注册表状态."""
        with self.assertRaises(ValueError):
            self.reg.match_or_register(
                [np.nan, 0.0, 1.0], class_id=0, now=1.0)
        with self.assertRaises(ValueError):
            self.reg.match_or_register(
                [0.0, 0.0, 1.0], class_id=0, now=np.inf)


class RecoveryMatchTest(unittest.TestCase):
    """恢复匹配：正常匹配未命中才启用的二段匹配（抗锚点跳变/翻类）."""

    def setUp(self):
        # 与节点默认一致：恢复半径 = 0.06 × 2.0 = 0.12 m，允许跨类
        self.reg = TargetRegistry(match_radius=0.06, max_targets=50,
                                  position_ema=0.3, recovery_scale=2.0,
                                  cross_class_recovery=True)

    def test_anchor_jump_within_recovery_radius_reuses_id(self):
        """锚点跳变 0.09 m（>0.06 但 ≤0.12）：恢复匹配复用原 ID 并做 EMA."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame()
        tid1, is_new = self.reg.match_or_register(
            [0.09, 0.0, 1.0], class_id=0, now=2.0)
        self.assertEqual(tid1, tid0)
        self.assertFalse(is_new)
        # EMA: 0.7*[0,0,1] + 0.3*[0.09,0,1] = [0.027, 0, 1]
        np.testing.assert_allclose(
            self.reg.get(tid0)['position'], [0.027, 0.0, 1.0], atol=1e-9)

    def test_beyond_recovery_radius_registers_new_id(self):
        """跳变超恢复半径（>0.12 m）：仍发新 ID."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame()
        tid1, is_new = self.reg.match_or_register(
            [0.2, 0.0, 1.0], class_id=0, now=2.0)
        self.assertNotEqual(tid1, tid0)
        self.assertTrue(is_new)

    def test_class_flip_recovery_reuses_id_but_keeps_entry_class(self):
        """bag→nobag 翻类：恢复匹配跨类命中复用 ID，表项类别不跟随翻转."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame()
        tid1, is_new = self.reg.match_or_register(
            [0.01, 0.0, 1.0], class_id=1, now=2.0)
        self.assertEqual(tid1, tid0)
        self.assertFalse(is_new)
        self.assertEqual(self.reg.get(tid0)['class_id'], 0)

    def test_normal_match_still_prefers_same_class_neighbor(self):
        """常态保护：同类近距离命中走第一段，不被远处跨类表项抢走."""
        tid0, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame()
        tid1, _ = self.reg.match_or_register([0.1, 0.0, 1.0], class_id=1, now=2.0)
        # 同类候选距 tid0 0.02（第一段命中），距 tid1 0.12（恢复段边界）：
        # 须命中 tid0 而非 tid1
        self.reg.begin_frame()
        tid2, is_new = self.reg.match_or_register(
            [0.02, 0.0, 1.0], class_id=0, now=3.0)
        self.assertEqual(tid2, tid0)
        self.assertFalse(is_new)
        self.assertEqual(self.reg.get(tid1)['obs_count'], 1)

    def test_recovery_disabled_by_default_params(self):
        """默认构造（recovery_scale=1.0 且不允许跨类）：行为与旧版一致."""
        reg = TargetRegistry(match_radius=0.06, max_targets=50,
                             position_ema=0.3)
        tid0, _ = reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        reg.begin_frame()
        tid1, is_new = reg.match_or_register(
            [0.09, 0.0, 1.0], class_id=0, now=2.0)
        self.assertNotEqual(tid1, tid0)
        self.assertTrue(is_new)

    def test_invalid_recovery_scale_rejected(self):
        """recovery_scale < 1 抛 ValueError."""
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, recovery_scale=0.5)


class ConfirmationTest(unittest.TestCase):
    """确认机制：短暂出现不长期记录（confirm_frames + tentative_ttl_sec）."""

    def setUp(self):
        # 3 帧确认、未确认 TTL 1.0 s（测试注入同一时钟基准的 now）
        self.reg = TargetRegistry(match_radius=0.06, max_targets=50,
                                  position_ema=0.3, confirm_frames=3,
                                  tentative_ttl_sec=1.0)

    def test_transient_sighting_is_evicted_after_ttl(self):
        """只出现一次的误检：未确认，超 TTL 后在 begin_frame 被清除."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertFalse(self.reg.get(tid)['confirmed'])
        self.reg.begin_frame(now=2.5)   # 1.5 s 未再命中 > TTL
        self.assertIsNone(self.reg.get(tid))
        self.assertEqual(self.reg.stats()['n_targets'], 0)
        # 序号不复用：下一新目标仍是 target_1
        self.reg.begin_frame(now=2.6)
        tid2, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=2.6)
        self.assertEqual(tid2, 'target_1')

    def test_persistent_target_confirmed_and_never_ttl_evicted(self):
        """连续命中 3 帧转正；转正后长期不出现也不被 TTL 清除."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertFalse(self.reg.get(tid)['confirmed'])
        self.reg.begin_frame(now=1.1)
        self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.1)
        self.assertFalse(self.reg.get(tid)['confirmed'])
        self.reg.begin_frame(now=1.2)
        self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.2)
        self.assertTrue(self.reg.get(tid)['confirmed'])
        self.reg.begin_frame(now=100.0)  # 远超 TTL，但已确认不清除
        self.assertIsNotNone(self.reg.get(tid))

    def test_confirmed_entry_wins_over_nearer_tentative(self):
        """已确认目标优先：候选落在已确认半径内时，不撞更近的未确认表项."""
        confirmed, _ = self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, now=1.0)
        for k in range(1, 3):  # 再命中 2 帧 → obs_count=3 转正
            self.reg.begin_frame(now=1.0 + 0.1 * k)
            self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0,
                                       now=1.0 + 0.1 * k)
        # 0.09 m 外出现一个未确认目标（超出正常匹配半径，注册为新表项）
        self.reg.begin_frame(now=2.0)
        tent, _ = self.reg.match_or_register([0.09, 0.0, 1.0], class_id=0, now=2.0)
        self.assertFalse(self.reg.get(tent)['confirmed'])
        # 候选距已确认 0.04、距未确认 0.05（更近）：须命中已确认者
        self.reg.begin_frame(now=2.1)
        tid, is_new = self.reg.match_or_register(
            [0.04, 0.0, 1.0], class_id=0, now=2.1)
        self.assertEqual(tid, confirmed)
        self.assertFalse(is_new)
        self.assertEqual(self.reg.get(tent)['obs_count'], 1)

    def test_default_confirm_frames_1_keeps_old_behavior(self):
        """默认 confirm_frames=1：注册即确认，TTL 机制不生效（兼容旧行为）."""
        reg = TargetRegistry(match_radius=0.06)
        tid, _ = reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertTrue(reg.get(tid)['confirmed'])
        reg.begin_frame(now=1000.0)
        self.assertIsNotNone(reg.get(tid))

    def test_invalid_confirm_params_rejected(self):
        """confirm_frames < 1 或 tentative_ttl_sec ≤ 0 抛 ValueError."""
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, confirm_frames=0)
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, tentative_ttl_sec=0.0)


if __name__ == '__main__':
    unittest.main()
