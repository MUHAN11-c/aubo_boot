"""Clock 抽象 / ManualClock / RclpyClockAdapter（假对象，无需 rclpy）."""
import unittest

from peach_core.clock import Clock, ManualClock
from peach_core.ros.clock_adapter import RclpyClockAdapter


class _FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _FakeRclpyClock:
    def __init__(self):
        self.stamp_ns = 0

    def now(self):
        return _FakeTime(self.stamp_ns)


class ClockContractTest(unittest.TestCase):
    """协议 I3 时钟抽象契约."""

    def test_clock_is_abstract(self):
        """Clock 基类不可直接实例化."""
        with self.assertRaises(TypeError):
            Clock()

    def test_manual_clock_starts_at_given_time(self):
        """虚拟时钟起始时刻可指定，now 不自动前进."""
        clock = ManualClock(start=5.0)
        self.assertEqual(clock.now(), 5.0)
        self.assertEqual(clock.now(), 5.0)

    def test_manual_clock_advance(self):
        """推进虚拟时间：前进 dt 秒并返回新时刻."""
        clock = ManualClock()
        self.assertEqual(clock.advance(0.5), 0.5)
        self.assertEqual(clock.advance(0.25), 0.75)
        self.assertEqual(clock.now(), 0.75)

    def test_manual_clock_rejects_rewind(self):
        """负 dt 抛 ValueError（单调语义禁止倒退）."""
        clock = ManualClock()
        with self.assertRaises(ValueError):
            clock.advance(-1.0)

    def test_rclpy_adapter_returns_seconds(self):
        """适配器把 nanoseconds ×1e-9 转秒（与现网写法一致）."""
        fake = _FakeRclpyClock()
        clock = RclpyClockAdapter(fake)
        fake.stamp_ns = 1_500_000_000
        self.assertAlmostEqual(clock.now(), 1.5)


if __name__ == '__main__':
    unittest.main()
