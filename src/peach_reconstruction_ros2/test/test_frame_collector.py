"""FrameCollector：状态机流转 + 视角过滤（重复/跳变/放行覆盖）+ 帧栈管理."""
import unittest

import numpy as np

from peach_reconstruction_ros2.captured_frame import CapturedFrame
from peach_reconstruction_ros2.frame_collector import (
    CollectorConfig,
    FrameCollector,
    STATE_IDLE,
    STATE_READY,
)


def _T(x=0.0, rot_z_deg=0.0):
    """构造「沿 x 平移 + 绕 z 旋转」的 4×4 测试位姿."""
    T = np.eye(4)
    rad = np.radians(rot_z_deg)
    T[:3, :3] = np.array([[np.cos(rad), -np.sin(rad), 0.0],
                          [np.sin(rad), np.cos(rad), 0.0],
                          [0.0, 0.0, 1.0]])
    T[:3, 3] = [x, 0.0, 0.0]
    return T


def _frame(T):
    """最小 CapturedFrame（3 点空云，只关心位姿）."""
    return CapturedFrame(
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_mm=np.zeros((2, 2), dtype=np.uint16),
        camera_K={'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0},
        stamp=0.0, T_base_camera=T,
        cloud_base=np.zeros((3, 3)))


class ViewFilterTest(unittest.TestCase):
    def setUp(self):
        self.col = FrameCollector()
        self.col.start('target_0', np.array([0.0, 0.0, 0.3]))

    def test_first_frame_no_filter(self):
        """首帧不做视角检查（无参照帧）."""
        ok, reason, trans, rot = self.col.check_view(_T())
        self.assertTrue(ok)
        self.assertEqual(reason, 'first_frame')
        self.assertIsNone(trans)
        self.assertIsNone(rot)

    def test_duplicate_rejected(self):
        """与上一帧平移 1 mm（< 2 mm）且无旋转 → 重复视角拒帧."""
        self.col.config.allow_duplicate_views = False
        self.assertTrue(self.col.add_frame(_frame(_T(0.0))))
        ok, reason, trans, _ = self.col.check_view(_T(0.001))
        self.assertFalse(ok)
        self.assertAlmostEqual(trans, 0.001)
        self.assertIn('过近', reason)

    def test_sufficient_motion_accepted(self):
        """平移 30 mm（[20, 80] mm 内）→ 收."""
        self.col.add_frame(_frame(_T(0.0)))
        ok, reason, _, _ = self.col.check_view(_T(0.03))
        self.assertTrue(ok)
        self.assertEqual(reason, 'ok')

    def test_jump_rejected(self):
        """平移 200 mm（> 80 mm）→ 跳变拒帧."""
        self.col.add_frame(_frame(_T(0.0)))
        ok, reason, _, _ = self.col.check_view(_T(0.2))
        self.assertFalse(ok)
        self.assertIn('跳变', reason)

    def test_rotation_only_accepted(self):
        """纯旋转 10°（≥ 5° 下限）不算重复 → 收."""
        self.col.add_frame(_frame(_T(0.0)))
        ok, _, _, rot = self.col.check_view(_T(0.0, rot_z_deg=10.0))
        self.assertTrue(ok)
        self.assertAlmostEqual(rot, 10.0, places=9)

    def test_allow_duplicate_override(self):
        """allow_duplicate_views=true：重复视角放行并返回 duplicate_allowed."""
        col = FrameCollector(CollectorConfig(allow_duplicate_views=True))
        col.start()
        col.add_frame(_frame(_T(0.0)))
        ok, reason, _, _ = col.check_view(_T(0.001))
        self.assertTrue(ok)
        self.assertEqual(reason, 'duplicate_allowed')


class StateMachineTest(unittest.TestCase):
    def test_finalize_needs_min_views(self):
        """不足 min_views 拒 finalize；补足转 READY 并拼接全部帧点云."""
        col = FrameCollector(CollectorConfig(min_views=2, recommended_views=3))
        col.start('target_0')
        ok, _, cloud = col.finalize()
        self.assertFalse(ok)
        self.assertIsNone(cloud)
        col.add_frame(_frame(_T(0.0)))
        col.add_frame(_frame(_T(0.03)))
        ok, msg, cloud = col.finalize()
        self.assertTrue(ok)
        self.assertEqual(col.state, STATE_READY)
        self.assertEqual(cloud.shape, (6, 3))  # 两帧各 3 点
        self.assertIn('少于推荐', msg)  # 2 < recommended=3 的提示

    def test_finalize_from_idle_rejected(self):
        """IDLE 态不能 finalize."""
        col = FrameCollector()
        ok, msg, _ = col.finalize()
        self.assertFalse(ok)
        self.assertIn('IDLE', msg)

    def test_remove_last_and_empty(self):
        """弹栈：有帧弹出最后一帧；空栈返回 None."""
        col = FrameCollector()
        col.start()
        self.assertIsNone(col.remove_last())
        col.add_frame(_frame(_T(0.0)))
        self.assertIsNotNone(col.remove_last())
        self.assertEqual(len(col.frames), 0)

    def test_max_views_guard(self):
        """max_views 上限：满栈后 add_frame 拒收."""
        col = FrameCollector(CollectorConfig(
            max_views=2, auto_finalize_at_max=True))
        col.start()
        self.assertTrue(col.add_frame(_frame(_T(0.0))))
        self.assertTrue(col.add_frame(_frame(_T(0.03))))
        self.assertFalse(col.add_frame(_frame(_T(0.06))))

    def test_reset_clears(self):
        """调用 reset 后帧栈 / 目标 / 计数清空，状态回 IDLE."""
        col = FrameCollector()
        col.start('t', np.zeros(3))
        col.add_frame(_frame(_T(0.0)))
        col.rejected_views = 3
        col.reset()
        self.assertEqual(col.state, STATE_IDLE)
        self.assertEqual(len(col.frames), 0)
        self.assertEqual(col.target_id, '')
        self.assertEqual(col.rejected_views, 0)
        self.assertEqual(col.accumulated_cloud().shape, (0, 3))


class AccumulatedRgbTest(unittest.TestCase):
    def test_rgb_concatenated_in_frame_order(self):
        """多帧颜色按帧序拼接，行数与累加云逐点对齐."""
        col = FrameCollector()
        col.start()
        c1 = np.full((3, 3), 10, dtype=np.uint8)
        c2 = np.full((3, 3), 20, dtype=np.uint8)
        f1 = _frame(_T(0.0))
        f1.cloud_rgb = c1
        f2 = _frame(_T(0.03))
        f2.cloud_rgb = c2
        col.add_frame(f1)
        col.add_frame(f2)
        rgb = col.accumulated_rgb()
        self.assertEqual(rgb.shape, (6, 3))
        np.testing.assert_array_equal(rgb[:3], c1)
        np.testing.assert_array_equal(rgb[3:], c2)
        self.assertEqual(col.accumulated_cloud().shape[0], rgb.shape[0])

    def test_missing_rgb_gives_none(self):
        """任一帧缺颜色 → accumulated_rgb 给 None（发布侧回退纯 xyz）."""
        col = FrameCollector()
        col.start()
        col.add_frame(_frame(_T(0.0)))  # cloud_rgb=None
        self.assertIsNone(col.accumulated_rgb())
        self.assertIsNone(FrameCollector().accumulated_rgb())  # 空栈


class AutoModeTest(unittest.TestCase):
    def test_auto_start_gating(self):
        """自动开始门：auto_mode 开且 IDLE 才允许；COLLECTING 不重复开始."""
        self.assertTrue(FrameCollector().should_auto_start())
        self.assertFalse(
            FrameCollector(CollectorConfig(auto_mode=False)).should_auto_start())
        col = FrameCollector()
        col.start()
        self.assertFalse(col.should_auto_start())

    def test_first_frame_capture_directly(self):
        """首帧直采（无参照位姿，不受间隔门限制）."""
        col = FrameCollector()
        col.start()
        action, reason = col.auto_capture_decision(_T(0.0), float('inf'))
        self.assertEqual(action, 'capture')
        self.assertIn('首帧', reason)

    def test_threshold_interval_and_warn_gates(self):
        """未达阈跳过 / 达阈采 / 间隔门优先 / 超上限只告警照采 / 纯旋转达阈."""
        col = FrameCollector()
        col.start()
        col.add_frame(_frame(_T(0.0)))
        # 未达阈：相机仍连续接收，但近重复视角不积分
        action, _ = col.auto_capture_decision(_T(0.001), 999.0)
        self.assertEqual(action, 'skip')
        # 达阈：平移 30 mm ≥ 20 mm → capture
        action, _ = col.auto_capture_decision(_T(0.03), 999.0)
        self.assertEqual(action, 'capture')
        # 显式配置非零间隔时，间隔门仍可用
        col.config.auto_min_interval_s = 1.0
        action, reason = col.auto_capture_decision(_T(0.03), 0.5)
        self.assertEqual(action, 'skip')
        self.assertIn('间隔门', reason)
        # 超上限：平移 200 mm > 80 mm → warn_capture（只告警不拒帧）
        action, reason = col.auto_capture_decision(_T(0.2), 999.0)
        self.assertEqual(action, 'warn_capture')
        self.assertIn('超上限', reason)
        # 旋转单独达阈：10° ≥ 5°（平移 0）→ capture
        action, _ = col.auto_capture_decision(_T(0.0, rot_z_deg=10.0), 999.0)
        self.assertEqual(action, 'capture')

    def test_auto_mode_off_never_auto(self):
        """auto_mode=false：决策恒 skip（回纯手动，不自动采）."""
        col = FrameCollector(CollectorConfig(auto_mode=False))
        col.start()
        action, reason = col.auto_capture_decision(_T(0.0), float('inf'))
        self.assertEqual(action, 'skip')
        self.assertIn('auto_mode=false', reason)

    def test_auto_finalize_gating(self):
        """满 max_views 且双开关开才触发自动完成信号."""
        col = FrameCollector(CollectorConfig(
            max_views=2, auto_finalize_at_max=True))
        col.start()
        self.assertFalse(col.should_auto_finalize())
        col.add_frame(_frame(_T(0.0)))
        col.add_frame(_frame(_T(0.03)))
        self.assertTrue(col.should_auto_finalize())
        # auto_finalize_at_max=false → 不触发
        col = FrameCollector(
            CollectorConfig(max_views=2, auto_finalize_at_max=False))
        col.start()
        col.add_frame(_frame(_T(0.0)))
        col.add_frame(_frame(_T(0.03)))
        self.assertFalse(col.should_auto_finalize())
        # auto_mode=false → 不触发
        col = FrameCollector(CollectorConfig(max_views=2, auto_mode=False))
        col.start()
        col.add_frame(_frame(_T(0.0)))
        col.add_frame(_frame(_T(0.03)))
        self.assertFalse(col.should_auto_finalize())


if __name__ == '__main__':
    unittest.main()
