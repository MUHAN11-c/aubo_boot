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
    """确认机制：短暂出现不长期记录（confirm_frames + tentative_ttl_frames）."""

    def setUp(self):
        # 3 帧确认、未确认 TTL 2 帧（按帧计，帧率以运行状态为准）
        self.reg = TargetRegistry(match_radius=0.06, max_targets=50,
                                  position_ema=0.3, confirm_frames=3,
                                  tentative_ttl_frames=2)

    def test_transient_sighting_is_evicted_after_ttl(self):
        """只出现一次的误检：未确认，连续超 TTL 帧未命中后被清除."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertFalse(self.reg.get(tid)['confirmed'])
        self.reg.begin_frame()   # 1 帧未命中 ≤ TTL，保留
        self.reg.begin_frame()   # 2 帧未命中 ≤ TTL，保留
        self.assertIsNotNone(self.reg.get(tid))
        self.reg.begin_frame()   # 3 帧未命中 > TTL → 清除
        self.assertIsNone(self.reg.get(tid))
        self.assertEqual(self.reg.stats()['n_targets'], 0)
        # 序号不复用：下一新目标仍是 target_1
        self.reg.begin_frame()
        tid2, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=2.6)
        self.assertEqual(tid2, 'target_1')

    def test_persistent_target_confirmed_and_never_ttl_evicted(self):
        """连续命中 3 帧转正；转正后长期不出现也不被 TTL 清除."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertFalse(self.reg.get(tid)['confirmed'])
        for k in range(1, 3):
            self.reg.begin_frame()
            self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0,
                                       now=1.0 + 0.1 * k)
        self.assertTrue(self.reg.get(tid)['confirmed'])
        for _ in range(100):     # 远超 TTL，但已确认不清除
            self.reg.begin_frame()
        self.assertIsNotNone(self.reg.get(tid))

    def test_confirmed_entry_wins_over_nearer_tentative(self):
        """已确认目标优先：候选落在已确认半径内时，不撞更近的未确认表项."""
        confirmed, _ = self.reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, now=1.0)
        for k in range(1, 3):  # 再命中 2 帧 → obs_count=3 转正
            self.reg.begin_frame()
            self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0,
                                       now=1.0 + 0.1 * k)
        # 0.09 m 外出现一个未确认目标（超出正常匹配半径，注册为新表项）
        self.reg.begin_frame()
        tent, _ = self.reg.match_or_register([0.09, 0.0, 1.0], class_id=0, now=2.0)
        self.assertFalse(self.reg.get(tent)['confirmed'])
        # 候选距已确认 0.04、距未确认 0.05（更近）：须命中已确认者
        self.reg.begin_frame()
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
        for _ in range(100):
            reg.begin_frame()
        self.assertIsNotNone(reg.get(tid))

    def test_invalid_confirm_params_rejected(self):
        """confirm_frames < 1 或 tentative_ttl_frames < 1 抛 ValueError."""
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, confirm_frames=0)
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, tentative_ttl_frames=0)


class MaxAgeEvictionTest(unittest.TestCase):
    """
    max_age_s 墙钟淘汰（阶段 D1 语义变更：已确认表项也按龄淘汰）.

    理由（协议 2.4/阶段 D 第 6 条）：跨场景/跨批次陈旧锚点会在新场景被
    恢复匹配误命中抢走新目标身份；长期存活性按墙钟秒判定，与帧率无关。
    """

    def setUp(self):
        # max_age 10 s（小值便于测试），confirm_frames=1 注册即确认
        self.reg = TargetRegistry(match_radius=0.06, max_age_s=10.0)

    def test_confirmed_entry_evicted_after_max_age(self):
        """已确认表项 last_seen 超 max_age_s 未命中，begin_frame 注入同钟淘汰."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertTrue(self.reg.get(tid)['confirmed'])
        self.reg.begin_frame(now=5.0)    # 龄 4 s ≤ 10 s：保留
        self.assertIsNotNone(self.reg.get(tid))
        self.reg.begin_frame(now=11.5)   # 龄 10.5 s > 10 s：淘汰
        self.assertIsNone(self.reg.get(tid))
        # 序号不复用：新目标仍是 target_1
        tid2, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=12.0)
        self.assertEqual(tid2, 'target_1')

    def test_recent_hit_refreshes_max_age(self):
        """超龄前再次命中刷新 last_seen，不被淘汰."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame(now=9.0)
        self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=9.0)
        self.reg.begin_frame(now=18.5)   # 距 9.0 仅 9.5 s ≤ 10 s
        self.assertIsNotNone(self.reg.get(tid))
        self.reg.begin_frame(now=19.5)   # 10.5 s > 10 s：淘汰
        self.assertIsNone(self.reg.get(tid))

    def test_no_now_skips_max_age_eviction(self):
        """begin_frame 不注入 now 时跳过墙钟淘汰（防双时钟混比误清）."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        for _ in range(50):              # 帧 TTL 对 confirmed 本就不生效
            self.reg.begin_frame()
        self.assertIsNotNone(self.reg.get(tid))

    def test_invalid_max_age_or_swing_params_rejected(self):
        """max_age_s ≤ 0 / swing_threshold_m ≤ 0 / swing_frames < 1 抛错."""
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, max_age_s=0.0)
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, swing_threshold_m=0.0)
        with self.assertRaises(ValueError):
            TargetRegistry(match_radius=0.06, swing_frames=0)

    def test_clear_returns_count_and_empties_table(self):
        """clear() 清全部表项并返回清前数量；序号计数器不复位."""
        self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.reg.begin_frame(now=1.1)
        self.reg.match_or_register([0.5, 0.0, 1.0], class_id=0, now=1.1)
        self.assertEqual(self.reg.clear(), 2)
        self.assertEqual(self.reg.target_ids(), [])
        self.assertEqual(self.reg.clear(), 0)
        # 清空后新发 ID 全局单调（不与清空前已下发下游的 ID 撞号）
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=2.0)
        self.assertEqual(tid, 'target_2')


class SwingDetectionTest(unittest.TestCase):
    """
    摆动检测（阶段 D1，协议 2.4）：观测残差连击置位/对称连击清除.

    残差 = 当前观测锚点 − 注册表 EMA 位置的模长（EMA 更新前）；阈值
    0.03 m、连击 3 帧与节点 wind.* 默认一致。
    """

    def setUp(self):
        # α=1.0：EMA 完全跟随最新观测，残差 = 相邻两帧观测点的距离，
        # 序列可精确预测（交替点位制造 0.04 连击，同点复测残差为 0）
        self.reg = TargetRegistry(match_radius=0.06, position_ema=1.0,
                                  swing_threshold_m=0.03, swing_frames=3)

    def _hit(self, pos, now):
        self.reg.begin_frame(now=now)
        tid, is_new = self.reg.match_or_register(pos, class_id=0, now=now)
        self.assertFalse(is_new)
        return tid

    def test_swing_flag_set_and_cleared_symmetrically(self):
        """连续 3 帧残差超阈值置 swinging；连续 3 帧低于阈值清除."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self.assertFalse(self.reg.get(tid)['swinging'])
        # 交替点位：每帧残差 0.04 > 0.03；连击 1、2 帧未置位，第 3 帧置位
        self._hit([0.04, 0.0, 1.0], now=1.1)   # 上连击 1
        self._hit([0.0, 0.0, 1.0], now=1.2)    # 上连击 2
        self.assertFalse(self.reg.get(tid)['swinging'])
        self._hit([0.04, 0.0, 1.0], now=1.3)   # 上连击 3 → 置位
        self.assertTrue(self.reg.get(tid)['swinging'])
        # 平息对称：同点复测残差 0，连击 1、2 帧保持，第 3 帧清除
        self._hit([0.04, 0.0, 1.0], now=1.4)   # 下连击 1
        self._hit([0.04, 0.0, 1.0], now=1.5)   # 下连击 2
        self.assertTrue(self.reg.get(tid)['swinging'])
        self._hit([0.04, 0.0, 1.0], now=1.6)   # 下连击 3 → 清除
        self.assertFalse(self.reg.get(tid)['swinging'])

    def test_lost_frames_do_not_vote(self):
        """目标 LOST 帧不增不清连击（无观测不是平息证据）."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self._hit([0.04, 0.0, 1.0], now=1.1)   # 上连击 1
        self._hit([0.0, 0.0, 1.0], now=1.2)    # 上连击 2
        for k in range(3, 8):                  # 5 帧 LOST：连击保持
            self.reg.begin_frame(now=1.0 + 0.1 * k)
        self.assertFalse(self.reg.get(tid)['swinging'])
        self._hit([0.04, 0.0, 1.0], now=1.8)   # 上连击 3 → 置位
        self.assertTrue(self.reg.get(tid)['swinging'])

    def test_residual_below_threshold_resets_up_streak(self):
        """连击中途一帧低于阈值即清零上连击（须重新连满 3 帧）."""
        tid, _ = self.reg.match_or_register([0.0, 0.0, 1.0], class_id=0, now=1.0)
        self._hit([0.04, 0.0, 1.0], now=1.1)   # 上连击 1
        self._hit([0.04, 0.0, 1.0], now=1.2)   # 残差 0：上连击清零
        self._hit([0.09, 0.0, 1.0], now=1.3)   # 残差 0.05：上连击 1
        self._hit([0.09, 0.0, 1.0], now=1.4)   # 残差 0：再次清零
        self.assertFalse(self.reg.get(tid)['swinging'])
        self._hit([0.13, 0.0, 1.0], now=1.5)   # 残差 0.04：上连击 1
        self._hit([0.09, 0.0, 1.0], now=1.6)   # 残差 0.04：上连击 2
        self.assertFalse(self.reg.get(tid)['swinging'])
        self._hit([0.13, 0.0, 1.0], now=1.7)   # 残差 0.04：上连击 3 → 置位
        self.assertTrue(self.reg.get(tid)['swinging'])


if __name__ == '__main__':
    unittest.main()
