"""感知→重建边界契约：候选安全门禁与绑定目标类别保持."""
from types import SimpleNamespace
import unittest

import numpy as np

from peach_reconstruction_ros2.candidate_contract import (
    candidate_axis_hint,
    select_reconstruction_candidate,
    TargetKindMemory,
)


def _point(x, y, z):
    """构造含 x/y/z 属性的最小点."""
    return SimpleNamespace(x=x, y=y, z=z)


def _candidate(target_id, status=0, frame='base_link', flags=None):
    """构造最小三维候选."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        target_id=target_id,
        status=status,
        diagnostic_flags=list(flags or []),
        bag_bottom=_point(0.1, 0.2, 0.3),
        bag_neck=_point(0.1, 0.2, 0.5),
        translation_direction=_point(0.0, 2.0, 0.0),
    )


def _candidate_array(*candidates, frame='base_link'):
    """构造候选数组并统一数组坐标系."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        candidates=list(candidates),
    )


class CandidateContractTest(unittest.TestCase):
    def test_accept_candidate_center(self):
        """合法 base_link ACCEPT 候选可绑定，中心为 bottom/neck 中点."""
        msg = _candidate_array(_candidate('target_0'))
        target_id, center = select_reconstruction_candidate(msg, 'base_link')
        self.assertEqual(target_id, 'target_0')
        np.testing.assert_allclose(center, [0.1, 0.2, 0.4])

    def test_wrong_array_frame_rejected(self):
        """相机系数组不得被误当成 base_link 候选."""
        msg = _candidate_array(
            _candidate('target_0', frame='camera_color_optical_frame'),
            frame='camera_color_optical_frame')
        target_id, center = select_reconstruction_candidate(msg, 'base_link')
        self.assertEqual(target_id, '')
        self.assertIsNone(center)

    def test_tf_diagnostic_candidate_rejected(self):
        """tf_stale/unavailable/untracked 候选均不得启动重建."""
        for flag in ('tf_stale', 'tf_unavailable', 'target_untracked'):
            with self.subTest(flag=flag):
                msg = _candidate_array(_candidate('target_0', flags=[flag]))
                target_id, center = select_reconstruction_candidate(
                    msg, 'base_link')
                self.assertEqual(target_id, '')
                self.assertIsNone(center)

    def test_safe_reobserve_is_fallback(self):
        """异常 ACCEPT 被过滤后，可退到安全的 REOBSERVE 候选."""
        msg = _candidate_array(
            _candidate('bad', status=0, flags=['tf_stale']),
            _candidate('safe', status=1))
        target_id, _ = select_reconstruction_candidate(msg, 'base_link')
        self.assertEqual(target_id, 'safe')

    def test_preferred_stable_id_wins_over_array_order(self):
        """全局计划指定 ID 后，不得因候选数组顺序改变重建对象."""
        msg = _candidate_array(
            _candidate('other'), _candidate('selected'))
        target_id, _ = select_reconstruction_candidate(
            msg, 'base_link', preferred_target_id='selected')
        self.assertEqual(target_id, 'selected')

    def test_preferred_id_is_hard_constraint(self):
        """首选质量较低或暂时缺失时，不得静默绑定其他 ACCEPT 目标."""
        msg = _candidate_array(
            _candidate('selected', status=1), _candidate('other'))
        target_id, _ = select_reconstruction_candidate(
            msg, 'base_link', preferred_target_id='selected')
        self.assertEqual(target_id, 'selected')

        target_id, center = select_reconstruction_candidate(
            msg, 'base_link', preferred_target_id='missing')
        self.assertEqual(target_id, '')
        self.assertIsNone(center)

    def test_bound_axis_hint_is_normalized(self):
        """重建冻结与绑定 ID 对应的有限单位方向先验."""
        msg = _candidate_array(_candidate('selected'))
        np.testing.assert_allclose(
            candidate_axis_hint(msg, 'selected'), [0.0, 1.0, 0.0])
        self.assertIsNone(candidate_axis_hint(msg, 'missing'))


class TargetKindContractTest(unittest.TestCase):
    def test_bound_kind_survives_target_disappearance(self):
        """绑定 fruit 消失后仍保持球拟合类别，不回退 bag."""
        memory = TargetKindMemory()
        memory.update([
            SimpleNamespace(target_id='target_7', target_kind='fruit')])
        memory.bind('target_7')
        memory.update([])

        kind, defaulted = memory.resolve()
        self.assertEqual(kind, 'fruit')
        self.assertFalse(defaulted)


if __name__ == '__main__':
    unittest.main()
