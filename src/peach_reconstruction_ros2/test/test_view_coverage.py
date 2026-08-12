"""主动视觉视角覆盖摘要测试."""
from types import SimpleNamespace
import unittest

import numpy as np

from peach_reconstruction_ros2.view_coverage import summarize_view_coverage


def _frame(position, ratio=0.6):
    """构造覆盖摘要所需的最小帧替身."""
    return SimpleNamespace(
        camera_position_base=np.asarray(position, dtype=np.float64),
        stamp=1.0,
        valid_depth_ratio=ratio,
        registration={'mode': 'fk'},
        diagnostic_flags=['pose_fk'],
    )


class ViewCoverageTest(unittest.TestCase):
    def test_reports_orthogonal_baseline_and_quality(self):
        """两个正交方向得到 90 度基线，深度质量取均值和最小值."""
        result = summarize_view_coverage(
            [_frame([1.0, 0.0, 0.0], 0.5),
             _frame([0.0, 2.0, 0.0], 0.7)],
            [0.0, 0.0, 0.0])
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['max_baseline_deg'], 90.0)
        self.assertAlmostEqual(result['valid_depth_ratio_mean'], 0.6)
        self.assertAlmostEqual(result['range_max_m'], 2.0)
        self.assertEqual(len(result['views']), 2)

    def test_missing_center_is_explicitly_invalid(self):
        """没有目标中心时不得伪造覆盖角，返回可诊断原因."""
        result = summarize_view_coverage([_frame([1.0, 0.0, 0.0])], None)
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'], 'target_center_unavailable')


if __name__ == '__main__':
    unittest.main()
