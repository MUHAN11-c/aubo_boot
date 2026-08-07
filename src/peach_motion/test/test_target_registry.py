"""TargetRegistry：归并同 ID、状态迁移、DONE 不调度、LOST 超时."""
import unittest

import numpy as np

from peach_motion.target_registry import (
    DONE,
    LOST,
    NEW,
    OBSERVING,
    READY,
    TargetRegistry,
)


def _obs(x, y=0.0, z=0.5, status=0, kind='bag'):
    """造一条观测（ACCEPT 袋装桃，默认 z=0.5 m）."""
    return [(np.array([x, y, z], dtype=float), kind, status)]


class TargetRegistryTest(unittest.TestCase):
    def setUp(self):
        self.reg = TargetRegistry(
            merge_radius=0.08, ready_frames=3, ready_pos_drift=0.02,
            lost_timeout_sec=2.0)

    def test_same_peach_merges_to_one_id(self):
        """同位置重复帧归并同一 ID，不产生新目标."""
        self.reg.update(_obs(0.30), now=0.0)
        ids = self.reg.update(_obs(0.31), now=0.1)   # 漂移 0.01 < 0.08
        self.assertEqual(ids, ['T001'])
        self.assertEqual(len(self.reg.snapshot()), 1)

    def test_distant_observation_registers_new_target(self):
        """超出归并半径的观测注册新目标（T002）."""
        self.reg.update(_obs(0.30), now=0.0)
        ids = self.reg.update(_obs(0.60), now=0.1)   # 相距 0.30 > 0.08
        self.assertEqual(ids, ['T002'])
        self.assertEqual(len(self.reg.snapshot()), 2)
        statuses = {r.target_id: r.status for r in self.reg.snapshot()}
        self.assertEqual(statuses['T001'], NEW)      # 仅首帧注册，尚未吸收

    def test_ready_after_stable_accept_streak(self):
        """连续 ACCEPT 且漂移小：NEW → OBSERVING → READY."""
        now = 0.0
        for i in range(5):
            self.reg.update(_obs(0.30 + i * 0.001), now=now)
            now += 0.1
        rec = self.reg.snapshot()[0]
        self.assertEqual(rec.status, READY)
        self.assertIs(self.reg.next_target(), rec)

    def test_done_target_not_rescheduled(self):
        """DONE 目标只更新观测，不再被调度；另有 READY 才可选."""
        for i in range(5):
            self.reg.update(_obs(0.30), now=i * 0.1)
        self.reg.mark_done('T001', 'ok')
        self.assertIsNone(self.reg.next_target())
        # DONE 后仍有观测灌入：ID 不变、状态保持 DONE
        self.reg.update(_obs(0.305), now=1.0)
        rec = self.reg.snapshot()[0]
        self.assertEqual(rec.status, DONE)
        self.assertEqual(rec.last_result, 'ok')
        self.assertAlmostEqual(float(rec.position[0]), 0.305, places=6)

    def test_lost_after_timeout(self):
        """超时未再见的活动目标标 LOST；EXECUTING/DONE 不受影响."""
        self.reg.update(_obs(0.30), now=0.0)
        self.reg.update([], now=3.0)     # 空帧，now-last_seen > 2.0
        rec = self.reg.snapshot()[0]
        self.assertEqual(rec.status, LOST)
        # LOST 重现后回 OBSERVING 重新累积
        self.reg.update(_obs(0.30), now=3.1)
        self.assertEqual(rec.status, OBSERVING)

    def test_new_then_observing_on_second_hit(self):
        """首帧 NEW，第二帧命中后 OBSERVING."""
        self.reg.update(_obs(0.30), now=0.0)
        rec = self.reg.snapshot()[0]
        self.assertEqual(rec.status, NEW)
        self.reg.update(_obs(0.30), now=0.1)
        self.assertEqual(rec.status, OBSERVING)


if __name__ == '__main__':
    unittest.main()
