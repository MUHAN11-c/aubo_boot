"""interfaces.py 接口层：ABC 实例契约 + 注册表名与 impls 显式注册清单一致."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose import impls as _impls  # noqa: F401  触发注册
from peach_pose_ros2.peach_pose.candidates import CandidateEstimator
from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.harvest_plan import CollectLockPolicy
from peach_pose_ros2.peach_pose.inference import MobileSam, UltralyticsYolo
from peach_pose_ros2.peach_pose.interfaces import (
    Detector,
    DETECTORS,
    LOCK_POLICIES,
    LockPolicy,
    MATCHERS,
    POSE_PIPELINES,
    PosePipeline,
    Segmenter,
    SEGMENTERS,
    TargetMatcher,
)
from peach_pose_ros2.peach_pose.pipeline import (
    RobustBagPosePipeline,
    RobustFruitPosePipeline,
)
from peach_pose_ros2.peach_pose.target_registry import (
    SpatialEmaMatcher,
    TargetRegistry,
)


def _obs(class_id: int) -> BagObservation:
    """最小观测：仅携带路由所需的 detections.class_id."""
    return BagObservation(
        rgb=np.zeros((4, 4, 3), dtype=np.uint8),
        depth=np.zeros((4, 4), dtype=np.uint16),
        camera_K={'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0},
        detections=[{'bbox': (0, 0, 4, 4), 'class_id': class_id, 'conf': 0.9}])


class AbcContractTest(unittest.TestCase):
    """默认实现是接口层 ABC 的实例（构造均懒加载，不触 GPU/模型）."""

    def test_default_impls_are_abc_instances(self):
        self.assertIsInstance(UltralyticsYolo(), Detector)
        self.assertIsInstance(MobileSam(), Segmenter)
        self.assertIsInstance(RobustBagPosePipeline(), PosePipeline)
        self.assertIsInstance(RobustFruitPosePipeline(), PosePipeline)
        self.assertIsInstance(SpatialEmaMatcher(), TargetMatcher)
        self.assertIsInstance(CollectLockPolicy(), LockPolicy)

    def test_pipeline_kind_class_attribute(self):
        """PosePipeline.kind 类属性：bag/fruit 两线键值正确且互异."""
        self.assertEqual(RobustBagPosePipeline.kind, 'bag')
        self.assertEqual(RobustFruitPosePipeline.kind, 'fruit')
        self.assertEqual(RobustBagPosePipeline().kind, 'bag')
        self.assertEqual(RobustFruitPosePipeline().kind, 'fruit')


class RegistryTest(unittest.TestCase):
    """注册表名集与 impls 显式注册清单一致（yaml *.impl 的合法值）."""

    def test_registered_names_and_classes(self):
        self.assertEqual(DETECTORS.names(), ('yolo',))
        self.assertEqual(SEGMENTERS.names(), ('mobile_sam',))
        self.assertEqual(POSE_PIPELINES.names(), ('robust_bag', 'robust_fruit'))
        self.assertEqual(MATCHERS.names(), ('spatial_ema',))
        self.assertEqual(LOCK_POLICIES.names(), ('collect_lock',))

    def test_create_returns_abc_instances(self):
        self.assertIsInstance(DETECTORS.create('yolo'), Detector)
        self.assertIsInstance(SEGMENTERS.create('mobile_sam'), Segmenter)
        self.assertIsInstance(
            POSE_PIPELINES.create('robust_bag'), PosePipeline)
        self.assertIsInstance(
            POSE_PIPELINES.create('robust_fruit'), PosePipeline)
        self.assertIsInstance(MATCHERS.create('spatial_ema'), TargetMatcher)
        self.assertIsInstance(LOCK_POLICIES.create('collect_lock'), LockPolicy)

    def test_create_unknown_name_raises_key_error(self):
        with self.assertRaises(KeyError):
            DETECTORS.create('bogus')


class CandidateRoutingTest(unittest.TestCase):
    """candidates 类别路由：class_id 0→bag 实例、1→fruit 实例（契约不变）."""

    def test_candidates_routes_by_class_id(self):
        est = CandidateEstimator()
        kind, pipe = est._pipeline_for(_obs(0))
        self.assertEqual(kind, 'bag')
        self.assertIsInstance(pipe, RobustBagPosePipeline)
        kind, pipe = est._pipeline_for(_obs(1))
        self.assertEqual(kind, 'fruit')
        self.assertIsInstance(pipe, RobustFruitPosePipeline)
        obs = _obs(0)
        obs.detections = []
        kind, _ = est._pipeline_for(obs)
        self.assertEqual(kind, 'bag')

    def test_fruit_pipeline_shares_bag_tool(self):
        """果线复用袋线 ToolGeometry（刀具契约一致，节点注入同 tool）."""
        est = CandidateEstimator()
        self.assertIs(est.fruit_pipeline.tool, est.pipeline.tool)


class TargetRegistryMatcherSplitTest(unittest.TestCase):
    """TargetRegistry 默认装配 SpatialEmaMatcher（未注入时的兼容路径）."""

    def test_default_matcher_created_from_scalar_kwargs(self):
        reg = TargetRegistry(match_radius=0.06, recovery_scale=3.5,
                             cross_class_recovery=True)
        self.assertAlmostEqual(reg.match_radius, 0.06)

    def test_injected_matcher_is_used(self):
        matcher = SpatialEmaMatcher(match_radius=0.12)
        reg = TargetRegistry(matcher=matcher)
        self.assertAlmostEqual(reg.match_radius, 0.12)


if __name__ == '__main__':
    unittest.main()
