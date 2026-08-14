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

    def test_same_stop_frames_cluster_into_one_view(self):
        """同机位连帧聚为一个机位：基线按机位代表算，不被兄弟帧稀释."""
        # 机位 A（+X 方向）3 帧微抖，机位 B（约 30° 外）2 帧
        frames = [
            _frame([1.00, 0.00, 0.0], 0.6),
            _frame([1.00, 0.02, 0.0], 0.6),
            _frame([1.00, -0.02, 0.0], 0.5),
            _frame([0.87, 0.50, 0.0], 0.7),
            _frame([0.87, 0.52, 0.0], 0.7),
        ]
        result = summarize_view_coverage(frames, [0.0, 0.0, 0.0])
        self.assertTrue(result['valid'])
        self.assertEqual(result['view_count'], 2)
        self.assertEqual(result['frame_count'], 5)
        self.assertAlmostEqual(result['max_baseline_deg'], 30.0, delta=2.0)
        # 两个机位各 3/2 帧：若按帧算 mean_nearest≈0°，聚类后≈30°
        self.assertAlmostEqual(
            result['mean_nearest_baseline_deg'], 30.0, delta=2.0)

    def test_spread_stops_report_per_stop_views(self):
        """分散机位各自成簇，view_count 等于机位数."""
        frames = [
            _frame([1.0, 0.0, 0.0]),
            _frame([1.0, 0.05, 0.0]),
            _frame([0.0, 1.0, 0.0]),
            _frame([-1.0, 0.0, 0.0]),
        ]
        result = summarize_view_coverage(frames, [0.0, 0.0, 0.0])
        self.assertEqual(result['view_count'], 3)
        self.assertAlmostEqual(result['max_baseline_deg'], 180.0, delta=3.0)


if __name__ == '__main__':
    unittest.main()
