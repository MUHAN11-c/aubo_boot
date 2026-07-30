import unittest

import numpy as np

from peach_pose_ros2.peach_pose.candidates import CandidateEstimator, MODE_IDS
from peach_pose_ros2.peach_pose.contracts import BagObservation


K = {"fx": 640.0, "fy": 636.0, "cx": 640.0, "cy": 360.0}


class CandidateEstimatorTest(unittest.TestCase):
    def setUp(self):
        self.depth = np.zeros((720, 1280), dtype=np.uint16)
        self.bbox = (610, 250, 670, 570)
        self.depth[250:570, 610:670] = 900
        self.rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.obs = BagObservation(
            rgb=self.rgb, depth=self.depth, camera_K=K,
            detections=[{"bbox": self.bbox, "class_id": 0, "conf": 0.9}])

    def test_all_modes_use_one_explicit_result_contract(self):
        sam = np.zeros_like(self.depth, dtype=bool)
        sam[250:570, 610:670] = True
        results = CandidateEstimator().estimate_modes(
            self.obs, "target", self.bbox, sam)
        self.assertEqual(tuple(results), MODE_IDS)
        for mode, result in results.items():
            self.assertEqual(result.grasp_2d.status, result.grasp_3d.status, mode)
            self.assertIn(result.grasp_3d.status, {"ACCEPT", "REOBSERVE", "REJECT"})

    def test_missing_sam_is_reobserve_not_silent_depth_fallback(self):
        results = CandidateEstimator().estimate_modes(
            self.obs, "target", self.bbox, None)
        for mode in MODE_IDS:
            self.assertEqual(results[mode].grasp_3d.status, "REOBSERVE")
            self.assertIn("mask_unavailable", results[mode].grasp_3d.diagnostic_flags)


if __name__ == "__main__":
    unittest.main()
