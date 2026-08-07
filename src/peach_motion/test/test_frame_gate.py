"""FrameGate：假时间下的等新帧/超时语义（wait_next 等的是"入口之后的新帧"）."""
import unittest

from peach_motion.pick_executor import FrameGate


class FrameGateTest(unittest.TestCase):
    def test_timeout_without_new_frame(self):
        """等待期间无新帧：超时返回 False，且确实等满了 timeout."""
        clock = [0.0]

        def now():
            return clock[0]

        def sleep(sec):
            clock[0] += sec

        gate = FrameGate(timeout_sec=0.5)
        self.assertFalse(gate.wait_next(now, sleep))
        self.assertGreater(clock[0], 0.5)

    def test_frame_arriving_during_wait_passes(self):
        """等待期间来新帧：立即放行 True."""
        clock = [0.0]
        gate = FrameGate(timeout_sec=1.0)

        def now():
            return clock[0]

        def sleep_then_frame(sec):
            clock[0] += sec
            if clock[0] >= 0.1:
                gate.note_frame()          # 模拟 0.1s 后感知帧到达

        self.assertTrue(gate.wait_next(now, sleep_then_frame))
        self.assertLess(clock[0], 1.0)

    def test_each_waypoint_needs_new_frame(self):
        """已计数的旧帧不算数：连续两次 wait_next，第二次须再来一帧."""
        clock = [0.0]

        def now():
            return clock[0]

        def sleep(sec):
            clock[0] += sec

        gate = FrameGate(timeout_sec=0.3)
        gate.note_frame()                              # 旧帧（入口前到达）
        self.assertFalse(gate.wait_next(now, sleep))   # 不算新帧 → 超时


if __name__ == '__main__':
    unittest.main()
