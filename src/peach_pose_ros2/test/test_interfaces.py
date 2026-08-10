"""interfaces.py 接口层：ABC 实例契约 + POSE_ESTIMATORS 注册表与路由一致."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.candidates import CandidateEstimator
from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.inference import InferenceEngine
from peach_pose_ros2.peach_pose.interfaces import (
    Detector,
    POSE_ESTIMATORS,
    PoseEstimator,
    Segmenter,
)
from peach_pose_ros2.peach_pose.pipeline import (
    RobustBagPosePipeline,
    RobustFruitPosePipeline,
)


def _obs(class_id: int) -> BagObservation:
    """最小观测：仅携带路由所需的 detections.class_id."""
    return BagObservation(
        rgb=np.zeros((4, 4, 3), dtype=np.uint8),
        depth=np.zeros((4, 4), dtype=np.uint16),
        camera_K={'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0},
        detections=[{'bbox': (0, 0, 4, 4), 'class_id': class_id, 'conf': 0.9}])


class AbcContractTest(unittest.TestCase):
    """现有实现是接口层 ABC 的实例（构造均懒加载，不触 GPU/模型）."""

    def test_inference_engine_is_detector_and_segmenter(self):
        engine = InferenceEngine()
        self.assertIsInstance(engine, Detector)
        self.assertIsInstance(engine, Segmenter)

    def test_pipelines_are_pose_estimators(self):
        self.assertIsInstance(RobustBagPosePipeline(), PoseEstimator)
        self.assertIsInstance(RobustFruitPosePipeline(), PoseEstimator)


class RegistryTest(unittest.TestCase):
    """注册表键集与 candidates 类别路由一致（YOLO 标签契约不变）."""

    def test_registry_keys_and_classes(self):
        """{'bag','fruit'} 两键，值即袋线/果线实现类."""
        self.assertEqual(set(POSE_ESTIMATORS), {'bag', 'fruit'})
        self.assertIs(POSE_ESTIMATORS['bag'], RobustBagPosePipeline)
        self.assertIs(POSE_ESTIMATORS['fruit'], RobustFruitPosePipeline)
        for cls in POSE_ESTIMATORS.values():
            self.assertTrue(issubclass(cls, PoseEstimator))

    def test_candidates_routes_by_registry_instances(self):
        """class_id 0→bag 实例、1→fruit 实例、空检测→bag；实例与注册表同键."""
        est = CandidateEstimator()
        kind, pipe = est._pipeline_for(_obs(0))
        self.assertEqual(kind, 'bag')
        self.assertIsInstance(pipe, POSE_ESTIMATORS['bag'])
        kind, pipe = est._pipeline_for(_obs(1))
        self.assertEqual(kind, 'fruit')
        self.assertIsInstance(pipe, POSE_ESTIMATORS['fruit'])
        obs = _obs(0)
        obs.detections = []
        kind, _ = est._pipeline_for(obs)
        self.assertEqual(kind, 'bag')


if __name__ == '__main__':
    unittest.main()
