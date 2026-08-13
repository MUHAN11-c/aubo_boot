"""capture_gate：手动/自动两路采帧公共门禁的逐条判据与顺序覆盖."""
import unittest

from peach_reconstruction_ros2.capture_gate import (
    capture_gate,
    GATE_ALLOW,
    GATE_DENY,
    GATE_NEED_TF,
    GATE_SKIP,
)


def _ok_values(**overrides):
    """构造一套「全过」判据值；overrides 单点打翻."""
    values = {
        'frame_available': True,
        'frame_count': 1,
        'max_views': 8,
        'mask_reason': '',
        'stamp_sec': 10.0,
        'last_captured_stamp_sec': 9.0,
        'frame_age_s': 0.05,
        'max_frame_age_s': 1.0,
        'require_robot_static': True,
        'joint_states_seen': True,
        'max_joint_vel': 0.001,
        'static_joint_vel_thresh': 0.01,
        'cam_frame_ok': True,
        'base_frame': 'base_link',
        'cam_frame': 'camera_color_optical_frame',
        'tf_available': True,
        'automatic': False,
    }
    values.update(overrides)
    return values


class CaptureGateAllowTest(unittest.TestCase):
    def test_all_pass_allows(self):
        """全部判据通过且 TF 成功 → allow."""
        decision = capture_gate(**_ok_values())
        self.assertEqual(decision.action, GATE_ALLOW)
        self.assertEqual(decision.reason, '')

    def test_tf_pending_requests_lookup(self):
        """前置全过但未查 TF（None）→ need_tf，请调用方查询后重评."""
        decision = capture_gate(**_ok_values(tf_available=None))
        self.assertEqual(decision.action, GATE_NEED_TF)

    def test_tf_pending_still_fails_early_gate(self):
        """前置门禁未过时即使未查 TF 也直接拒绝（不进入 need_tf）."""
        decision = capture_gate(
            **_ok_values(tf_available=None, frame_available=False))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertIn('尚无同步 RGB-D 帧', decision.reason)


class CaptureGateManualDenyTest(unittest.TestCase):
    """手动模式（automatic=False）：失败映射 deny 并带计数标记."""

    def test_full_stack_denies(self):
        """满栈拒绝并计 rejected_views."""
        decision = capture_gate(**_ok_values(frame_count=8))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertEqual(
            decision.reason, '已达 max_views=8，请 finalize 或 remove_last')
        self.assertTrue(decision.count_reject)
        self.assertFalse(decision.count_tf_failure)

    def test_no_frame_denies(self):
        """无缓存帧拒绝."""
        decision = capture_gate(**_ok_values(frame_available=False))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertEqual(decision.reason, '尚无同步 RGB-D 帧（确认相机/回放在线）')

    def test_mask_reason_passthrough(self):
        """掩膜门禁原因原样透传."""
        decision = capture_gate(**_ok_values(mask_reason='缺少掩膜'))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertEqual(decision.reason, '缺少掩膜')

    def test_same_stamp_denies(self):
        """与上次采帧同帧拒绝."""
        decision = capture_gate(
            **_ok_values(stamp_sec=9.0, last_captured_stamp_sec=9.0))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertIn('缓存帧未更新', decision.reason)

    def test_stale_frame_denies_with_message_values(self):
        """陈帧拒绝，消息含龄期与阈值（与原内联实现逐字一致）."""
        decision = capture_gate(
            **_ok_values(frame_age_s=2.5, max_frame_age_s=1.0))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertEqual(
            decision.reason,
            '缓存帧龄期 2.50 s > max_frame_age_s=1.0（陈帧拒采）')

    def test_static_gates(self):
        """静止门：未收 joint_states 与速度超阈分别拒绝."""
        unseen = capture_gate(**_ok_values(joint_states_seen=False))
        self.assertEqual(unseen.action, GATE_DENY)
        self.assertIn('未收到 /joint_states', unseen.reason)
        moving = capture_gate(**_ok_values(max_joint_vel=0.5))
        self.assertEqual(moving.action, GATE_DENY)
        self.assertEqual(
            moving.reason,
            '机器人未静止：最大关节速度 0.5000 rad/s > 0.01')
        # 不要求静止时不再检查这两项
        relaxed = capture_gate(**_ok_values(
            require_robot_static=False, joint_states_seen=False,
            max_joint_vel=9.0))
        self.assertEqual(relaxed.action, GATE_ALLOW)

    def test_empty_cam_frame_denies_without_reject_count(self):
        """空 frame_id 拒绝但不计 rejected_views（非帧质量问题）."""
        decision = capture_gate(**_ok_values(cam_frame_ok=False))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertFalse(decision.count_reject)

    def test_tf_failure_denies_and_counts_tf(self):
        """TF 失败：计 tf_failures 而非 rejected_views，消息含坐标系对."""
        decision = capture_gate(**_ok_values(tf_available=False))
        self.assertEqual(decision.action, GATE_DENY)
        self.assertEqual(
            decision.reason,
            'TF base_link←camera_color_optical_frame 查询失败'
            '（已计 tf_failures）')
        self.assertFalse(decision.count_reject)
        self.assertTrue(decision.count_tf_failure)


class CaptureGateAutoSkipTest(unittest.TestCase):
    """自动模式（automatic=True）：同样判据映射 skip，计数标记不变."""

    def test_failures_become_skip(self):
        """自动模式失败一律 skip（调用方不记 rejected_views）."""
        for overrides in (
                {'frame_count': 8},
                {'frame_available': False},
                {'mask_reason': '缺少掩膜'},
                {'stamp_sec': 9.0, 'last_captured_stamp_sec': 9.0},
                {'frame_age_s': 2.5},
                {'joint_states_seen': False},
                {'max_joint_vel': 0.5},
                {'cam_frame_ok': False}):
            decision = capture_gate(**_ok_values(automatic=True, **overrides))
            self.assertEqual(decision.action, GATE_SKIP, overrides)

    def test_tf_failure_skip_still_counts_tf(self):
        """自动模式 TF 失败 skip 且仍计 tf_failures."""
        decision = capture_gate(**_ok_values(automatic=True,
                                             tf_available=False))
        self.assertEqual(decision.action, GATE_SKIP)
        self.assertTrue(decision.count_tf_failure)
        self.assertFalse(decision.count_reject)

    def test_auto_allow(self):
        """自动模式全过同样 allow."""
        decision = capture_gate(**_ok_values(automatic=True))
        self.assertEqual(decision.action, GATE_ALLOW)


class CaptureGateOrderTest(unittest.TestCase):
    def test_first_failing_gate_wins(self):
        """顺序即优先级：无帧 + 陈帧 + 满栈同时成立时满栈先报."""
        decision = capture_gate(**_ok_values(
            frame_count=8, frame_available=False, frame_age_s=99.0))
        self.assertIn('max_views', decision.reason)
        decision = capture_gate(**_ok_values(
            frame_available=False, frame_age_s=99.0))
        self.assertIn('尚无同步 RGB-D 帧', decision.reason)


if __name__ == '__main__':
    unittest.main()
