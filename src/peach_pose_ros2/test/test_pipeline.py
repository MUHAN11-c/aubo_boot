"""袋线 RobustBagPosePipeline：合成深度上的状态门控冒烟."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.pipeline import RobustBagPosePipeline


K = {'fx': 640.0, 'fy': 636.0, 'cx': 640.0, 'cy': 360.0}


class RobustBagPosePipelineTest(unittest.TestCase):
    def _observation(self, depth):
        return BagObservation(rgb=np.zeros((*depth.shape, 3), dtype=np.uint8), depth=depth,
                              camera_K=K, detections=[
                                  {'bbox': (500, 220, 780, 620), 'class_id': 0, 'conf': .9}])

    def test_valid_target_has_consistent_2d_3d_status(self):
        depth = np.zeros((720, 1280), dtype=np.uint16)
        depth[220:620, 500:780] = 900
        mask = np.zeros_like(depth, dtype=bool)
        mask[220:620, 500:780] = True
        result = RobustBagPosePipeline().estimate(
            self._observation(depth), 'frame-1', (500, 220, 780, 620), mask, 'test')
        self.assertEqual(result.grasp_2d.status, result.grasp_3d.status)
        self.assertIsNotNone(result.grasp_3d.entry_start)
        self.assertGreater(result.metrics['n_points'], 100)

    def test_missing_depth_is_rejected(self):
        depth = np.zeros((720, 1280), dtype=np.uint16)
        result = RobustBagPosePipeline().estimate(
            self._observation(depth), 'frame-1', (500, 220, 780, 620))
        self.assertEqual(result.grasp_3d.status, 'REJECT')
        self.assertIn('insufficient_measured_points', result.grasp_3d.diagnostic_flags)

    def test_oversize_bag_is_never_accepted(self):
        depth = np.zeros((720, 1280), dtype=np.uint16)
        depth[160:660, 300:980] = 800
        mask = np.zeros_like(depth, dtype=bool)
        mask[160:660, 300:980] = True
        result = RobustBagPosePipeline().estimate(
            self._observation(depth), 'frame-1', (300, 160, 980, 660), mask, 'test')
        self.assertNotEqual(result.grasp_3d.status, 'ACCEPT')
        self.assertIn('tool_clearance_failed', result.grasp_3d.diagnostic_flags)

    def test_diameter_uses_centreline_not_bottom_surface_point(self):
        depth = np.zeros((720, 1280), dtype=np.uint16)
        bbox = (615, 260, 665, 560)
        depth[260:560, 615:665] = 1000
        mask = np.zeros_like(depth, dtype=bool)
        mask[260:560, 615:665] = True
        result = RobustBagPosePipeline().estimate(
            self._observation(depth), 'known-width', bbox, mask, 'test')
        diameter = result.metrics['bag_diameter_upper_m']
        # 50 px at z=1 m and fx=640 is a visible width of about 78 mm.
        self.assertGreater(diameter, 0.065)
        self.assertLess(diameter, 0.085)
        self.assertNotIn('tool_clearance_failed', result.grasp_3d.diagnostic_flags)


if __name__ == '__main__':
    unittest.main()
