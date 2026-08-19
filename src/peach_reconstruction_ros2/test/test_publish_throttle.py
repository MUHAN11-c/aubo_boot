"""发布节流器（E4）：on-change 触发/间隔内抑制/间隔到补发/force 透传."""
import unittest

from peach_reconstruction_ros2.publish_throttle import PublishThrottle


class _FakeClock:
    """可手动推进的假时钟（注入 PublishThrottle.now，协议 I3 测试形态）."""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t


class PublishThrottleTest(unittest.TestCase):
    def setUp(self):
        """每用例独立节流器 + 假时钟（min_interval=0.2s）."""
        self.clock = _FakeClock()
        self.throttle = PublishThrottle(
            min_interval_s=0.2, now=self.clock.now)

    def test_first_call_publishes(self):
        """首次调用（无任何发布记录）：直接放行."""
        self.assertTrue(self.throttle.should_publish('cloud', (1,)))

    def test_unchanged_key_suppressed(self):
        """零变化：key 不变一律抑制（闩锁保留最后一帧，时间推进也不发）."""
        self.assertTrue(self.throttle.should_publish('cloud', (1,)))
        for dt in (0.1, 1.0, 10.0):
            self.clock.t += dt
            self.assertFalse(self.throttle.should_publish('cloud', (1,)))

    def test_change_within_interval_suppressed_then_passed(self):
        """间隔内变化被抑制且不记 key；间隔到后同一新 key 补发（变化不丢）."""
        self.assertTrue(self.throttle.should_publish('cloud', (1,)))
        self.clock.t = 0.10
        # 间隔内（0.10 < 0.2）：抑制；关键是不记录 (2,) 为已发 key
        self.assertFalse(self.throttle.should_publish('cloud', (2,)))
        self.clock.t = 0.15
        self.assertFalse(self.throttle.should_publish('cloud', (2,)))
        # 间隔到（0.25-0.0 ≥ 0.2）：同一新 key 仍判为已变化 → 放行补发
        self.clock.t = 0.25
        self.assertTrue(self.throttle.should_publish('cloud', (2,)))
        # 补发后该 key 成为已发版本，再次零变化抑制
        self.assertFalse(self.throttle.should_publish('cloud', (2,)))

    def test_force_bypasses_interval_and_onchange(self):
        """force=True（产物清空同步事件）：绕过间隔门与 on-change 立即透传."""
        self.assertTrue(self.throttle.should_publish('cloud', (1,)))
        self.clock.t = 0.05  # 间隔内 + key 未变，双重抑制条件下仍放行
        self.assertTrue(
            self.throttle.should_publish('cloud', (1,), force=True))

    def test_min_interval_zero_disables_interval_gate(self):
        """min_interval_s=0：间隔门关闭，key 变化即透（只留 on-change）."""
        throttle = PublishThrottle(min_interval_s=0.0, now=self.clock.now)
        self.assertTrue(throttle.should_publish('cloud', (1,)))
        self.clock.t = 0.001
        self.assertTrue(throttle.should_publish('cloud', (2,)))
        self.assertFalse(throttle.should_publish('cloud', (2,)))

    def test_topics_tracked_independently(self):
        """按话题独立记账：A 的发布不影响 B 的首次放行."""
        self.assertTrue(self.throttle.should_publish('a', (1,)))
        self.clock.t = 0.05  # 对 A 在间隔内，对 B 无记录
        self.assertTrue(self.throttle.should_publish('b', (1,)))
        self.assertFalse(self.throttle.should_publish('a', (2,)))

    def test_reset_clears_books(self):
        """重置后清空记账：同 key 再次放行（测试隔离/节点复位备用接口）."""
        self.assertTrue(self.throttle.should_publish('cloud', (1,)))
        self.throttle.reset()
        self.assertTrue(self.throttle.should_publish('cloud', (1,)))


if __name__ == '__main__':
    unittest.main()
