"""BindSwitchHoldoff：selected 切换防抖状态机全迁移路径（注入假时钟）."""
import unittest

from peach_reconstruction_ros2.bind_holdoff import BindSwitchHoldoff


class _FakeClock:
    """手动推进的单调假时钟（秒）."""

    def __init__(self):
        self.t = 100.0

    def now(self):
        """读当前时刻."""
        return self.t

    def advance(self, dt):
        """推进 dt 秒."""
        self.t += dt


class BindSwitchHoldoffTest(unittest.TestCase):
    def setUp(self):
        self.clock = _FakeClock()
        self.holdoff = BindSwitchHoldoff(holdoff_s=2.0, now=self.clock.now)

    def test_follow_when_no_session_or_no_binding(self):
        """无进行中会话（IDLE）或从未跟随任何目标：一切变化立即 follow."""
        # IDLE：无会话可毁，变化直通，且不留挂起
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=False),
            BindSwitchHoldoff.FOLLOW)
        self.assertIsNone(self.holdoff.pending_id)
        # 未绑定任何目标（bound=''）：直通
        self.assertEqual(
            self.holdoff.arbitrate('B', '', session_active=True),
            BindSwitchHoldoff.FOLLOW)
        self.assertIsNone(self.holdoff.pending_id)

    def test_follow_when_requested_equals_bound(self):
        """Requested 本就等于绑定 ID 且无挂起：常规直通."""
        self.assertEqual(
            self.holdoff.arbitrate('A', 'A', session_active=True),
            BindSwitchHoldoff.FOLLOW)

    def test_pend_then_wait_then_commit(self):
        """A→B 持续超 holdoff：pend → wait → commit（到期放弃重绑）."""
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.assertEqual(self.holdoff.pending_id, 'B')
        # 挂起未满 2s：维持旧绑定
        self.clock.advance(1.0)
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.WAIT)
        # 满 2s：到期 commit，挂起清除
        self.clock.advance(1.0)
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.COMMIT)
        self.assertIsNone(self.holdoff.pending_id)

    def test_cancel_on_switch_back_within_holdoff(self):
        """A→B→A 瞬态抖动：holdoff 内切回原 ID 即 cancel，会话零扰动."""
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.clock.advance(0.5)
        self.assertEqual(
            self.holdoff.arbitrate('A', 'A', session_active=True),
            BindSwitchHoldoff.CANCEL)
        self.assertIsNone(self.holdoff.pending_id)
        # 取消后再偏离重新走完整 pend→commit 流程
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.PEND)

    def test_cancel_returns_follow_when_session_inactive(self):
        """切回绑定 ID 但会话已非活跃：清挂起并 follow（无需 cancel 语义）."""
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.assertEqual(
            self.holdoff.arbitrate('A', 'A', session_active=False),
            BindSwitchHoldoff.FOLLOW)
        self.assertIsNone(self.holdoff.pending_id)

    def test_repend_on_another_new_id_resets_timer(self):
        """抖动链 A→B→C：改挂到另一个新 ID 重新记时，永不到期."""
        self.assertEqual(
            self.holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.clock.advance(1.9)
        # 挂起快到期时换成 C：重新记时
        self.assertEqual(
            self.holdoff.arbitrate('C', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.assertEqual(self.holdoff.pending_id, 'C')
        # C 维持 1.9s（累计偏离 3.8s 但 C 自身未满）：仍 wait
        self.clock.advance(1.9)
        self.assertEqual(
            self.holdoff.arbitrate('C', 'A', session_active=True),
            BindSwitchHoldoff.WAIT)
        self.clock.advance(0.1)
        self.assertEqual(
            self.holdoff.arbitrate('C', 'A', session_active=True),
            BindSwitchHoldoff.COMMIT)

    def test_selected_cleared_pends_like_switch(self):
        """Selected 变空（''）同样挂起：持续空超时才 commit（防瞬态丢锁毁会话）."""
        self.assertEqual(
            self.holdoff.arbitrate('', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.assertEqual(self.holdoff.pending_id, '')
        self.clock.advance(2.0)
        self.assertEqual(
            self.holdoff.arbitrate('', 'A', session_active=True),
            BindSwitchHoldoff.COMMIT)

    def test_zero_holdoff_commits_on_next_consistent_message(self):
        """holdoff_s=0：下一条维持新 ID 的观测即到期（近似旧版立即跟随）."""
        holdoff = BindSwitchHoldoff(holdoff_s=0.0, now=self.clock.now)
        self.assertEqual(
            holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.PEND)
        self.assertEqual(
            holdoff.arbitrate('B', 'A', session_active=True),
            BindSwitchHoldoff.COMMIT)


if __name__ == '__main__':
    unittest.main()
