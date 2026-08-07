"""CandidateEstimator：单结果契约与 SAM 缺失时显式 REOBSERVE."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.candidates import CandidateEstimator
from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.pipeline import TargetPoseResult


K = {'fx': 640.0, 'fy': 636.0, 'cx': 640.0, 'cy': 360.0}


class CandidateEstimatorTest(unittest.TestCase):
    def setUp(self):
        # 合成竖直柱状深度块，模拟袋 ROI
        self.depth = np.zeros((720, 1280), dtype=np.uint16)
        self.bbox = (610, 250, 670, 570)
        self.depth[250:570, 610:670] = 900
        self.rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.obs = BagObservation(
            rgb=self.rgb, depth=self.depth, camera_K=K,
            detections=[{'bbox': self.bbox, 'class_id': 0, 'conf': 0.9}])

    def test_single_result_contract(self):
        """有 SAM 时：返回单个 TargetPoseResult，2D/3D 状态一致且落在三态内."""
        sam = np.zeros_like(self.depth, dtype=bool)
        sam[250:570, 610:670] = True
        estimator = CandidateEstimator()
        result = estimator.estimate(self.obs, 'target', self.bbox, sam)
        self.assertIsInstance(result, TargetPoseResult)
        self.assertEqual(result.target_id, 'target')
        # class_id=0 → 袋线；策略 ID 带单模式后缀
        self.assertEqual(result.target_kind, 'bag')
        self.assertEqual(result.grasp_3d.strategy_id,
                         'robust_bag_pose:hybrid_dilated')
        self.assertEqual(result.grasp_2d.status, result.grasp_3d.status)
        self.assertIn(result.grasp_3d.status, {'ACCEPT', 'REOBSERVE', 'REJECT'})
        # 单值耗时：非负浮点（掩膜构造 + 几何）
        self.assertGreaterEqual(estimator.last_timings_ms, 0.0)

    def test_missing_sam_is_reobserve_not_silent_depth_fallback(self):
        """无 SAM：必须 REOBSERVE + mask_unavailable，禁止静默深度回退."""
        result = CandidateEstimator().estimate(
            self.obs, 'target', self.bbox, None)
        self.assertEqual(result.grasp_3d.status, 'REOBSERVE')
        self.assertIn('mask_unavailable', result.grasp_3d.diagnostic_flags)


if __name__ == '__main__':
    unittest.main()
