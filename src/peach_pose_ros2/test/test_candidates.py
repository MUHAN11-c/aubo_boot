"""CandidateEstimator：模式契约与 SAM 缺失时显式 REOBSERVE；检测框 IoS 去重."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.candidates import (
    CandidateEstimator,
    dedup_overlapping_detections,
    MODE_IDS,
)
from peach_pose_ros2.peach_pose.contracts import BagObservation


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

    def test_all_modes_use_one_explicit_result_contract(self):
        """有 SAM 时：2D/3D 状态一致，且落在三态内."""
        sam = np.zeros_like(self.depth, dtype=bool)
        sam[250:570, 610:670] = True
        results = CandidateEstimator().estimate_modes(
            self.obs, 'target', self.bbox, sam)
        self.assertEqual(tuple(results), MODE_IDS)
        for mode, result in results.items():
            self.assertEqual(result.grasp_2d.status, result.grasp_3d.status, mode)
            self.assertIn(result.grasp_3d.status, {'ACCEPT', 'REOBSERVE', 'REJECT'})

    def test_missing_sam_is_reobserve_not_silent_depth_fallback(self):
        """无 SAM：必须 REOBSERVE + mask_unavailable，禁止静默深度回退."""
        results = CandidateEstimator().estimate_modes(
            self.obs, 'target', self.bbox, None)
        for mode in MODE_IDS:
            self.assertEqual(results[mode].grasp_3d.status, 'REOBSERVE')
            self.assertIn('mask_unavailable', results[mode].grasp_3d.diagnostic_flags)


class DedupOverlappingDetectionsTest(unittest.TestCase):
    """IoS 去重：一框基本包含另一框才删，留大框（并列取高置信度），跨类生效."""

    @staticmethod
    def _det(bbox, class_id=0, conf=0.9):
        return {'bbox': bbox, 'class_id': class_id, 'conf': conf}

    def test_contained_small_box_is_suppressed_keeping_larger(self):
        """小框被大框包含（局部误检）：删小留大，跨类同样生效."""
        big = self._det((100, 100, 300, 300), class_id=0, conf=0.7)
        small = self._det((150, 150, 200, 200), class_id=1, conf=0.95)
        out = dedup_overlapping_detections([small, big], ios_threshold=0.6)
        self.assertEqual(out, [big])

    def test_partial_overlap_adjacent_fruits_are_kept(self):
        """相邻两桃部分重叠（IoS 低）：两框都保留，不误删."""
        a = self._det((100, 100, 220, 220), class_id=0)
        b = self._det((200, 100, 320, 220), class_id=0)  # 重叠仅 20px 宽
        out = dedup_overlapping_detections([a, b], ios_threshold=0.6)
        self.assertEqual(len(out), 2)

    def test_equal_area_keeps_higher_confidence(self):
        """面积并列（同框不同类）：保留置信度高者."""
        hi = self._det((100, 100, 200, 200), class_id=1, conf=0.9)
        lo = self._det((100, 100, 200, 200), class_id=0, conf=0.6)
        out = dedup_overlapping_detections([lo, hi], ios_threshold=0.6)
        self.assertEqual(out, [hi])

    def test_disjoint_and_disabled_threshold_passthrough(self):
        """不相交不动；阈值 ≥1.0 等效关闭，原样返回."""
        a = self._det((0, 0, 50, 50))
        b = self._det((100, 100, 150, 150))
        self.assertEqual(len(dedup_overlapping_detections([a, b], 0.6)), 2)
        c = self._det((10, 10, 40, 40))  # 被 a 包含
        out = dedup_overlapping_detections([a, c], ios_threshold=1.0)
        self.assertEqual(len(out), 2)

    def test_empty_input(self):
        """空输入返回空列表."""
        self.assertEqual(dedup_overlapping_detections([], 0.6), [])


if __name__ == '__main__':
    unittest.main()
