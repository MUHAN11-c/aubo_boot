"""
接口层契约测试：每类 ABC 配一个 fake 实现，驱动调用端完整调用路径.

验证目标（2.14 装配规则）：调用端只依赖接口层 ABC，不依赖任何默认
实现——fake 实现注入后，完整调用路径（InferenceEngine 委托、
CandidateEstimator 类别路由、TargetRegistry 匹配/注册、
GlobalHarvestPlan 收齐锁定）全部可用且行为符合接口契约。
"""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.candidates import CandidateEstimator
from peach_pose_ros2.peach_pose.contracts import (
    BagGrasp2D,
    BagGraspReference3D,
    BagObservation,
    TOOL_GEOMETRY,
)
from peach_pose_ros2.peach_pose.harvest_plan import GlobalHarvestPlan
from peach_pose_ros2.peach_pose.inference import InferenceEngine
from peach_pose_ros2.peach_pose.interfaces import (
    Detector,
    LockEvent,
    LockPolicy,
    MatchResult,
    PosePipeline,
    Segmenter,
    TargetMatcher,
)
from peach_pose_ros2.peach_pose.pipeline import TargetPoseResult
from peach_pose_ros2.peach_pose.target_registry import TargetRegistry


class FakeDetector(Detector):
    """记录调用、返回固定检测框的 fake 检测器."""

    def __init__(self):
        """初始化调用记录."""
        self.calls = []
        self.reset_count = 0

    def detect(self, rgb):
        """返回一个固定框并记录输入 shape."""
        self.calls.append(rgb.shape)
        return [{'class_id': 0, 'class_name': 'peach_bag',
                 'bbox': (1, 2, 30, 40), 'conf': 0.9}]

    def reset(self):
        """记录 reset 调用."""
        self.reset_count += 1


class FakeSegmenter(Segmenter):
    """按 prompt 框回显全 True 掩膜的 fake 分割器."""

    def __init__(self):
        """初始化调用记录."""
        self.calls = []
        self.reset_count = 0

    def segment(self, rgb, bboxes):
        """对每个 prompt 框返回全 True 掩膜."""
        self.calls.append(list(bboxes))
        return [(np.ones(rgb.shape[:2], dtype=bool), bbox) for bbox in bboxes]

    def reset(self):
        """记录 reset 调用."""
        self.reset_count += 1


class InferenceEngineContractTest(unittest.TestCase):
    """InferenceEngine 只依赖 Detector/Segmenter 接口（fake 驱动委托路径）."""

    def test_detect_segment_reset_delegate_to_interfaces(self):
        detector = FakeDetector()
        segmenter = FakeSegmenter()
        engine = InferenceEngine(detector=detector, segmenter=segmenter)
        rgb = np.zeros((8, 8, 3), dtype=np.uint8)
        dets = engine.detect(rgb)
        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0]['bbox'], (1, 2, 30, 40))
        self.assertEqual(detector.calls, [(8, 8, 3)])
        segs = engine.segment(rgb, [(1, 2, 30, 40)])
        self.assertEqual(len(segs), 1)
        mask, bbox = segs[0]
        self.assertEqual(bbox, (1, 2, 30, 40))
        self.assertTrue(bool(mask.all()))
        self.assertEqual(segmenter.calls, [[(1, 2, 30, 40)]])
        engine.reset()
        self.assertEqual(detector.reset_count, 1)
        self.assertEqual(segmenter.reset_count, 1)


class FakePosePipeline(PosePipeline):
    """记录 estimate 调用并返回罐头结果的 fake 位姿管线."""

    def __init__(self, kind, tool=TOOL_GEOMETRY):
        """指定 kind 与共享 ToolGeometry（接口实例属性契约）."""
        self.kind = kind
        self.tool = tool
        self.min_depth_m = 0.1
        self.max_depth_m = 5.0
        self.calls = []

    def estimate(self, obs, target_id, bbox, mask=None,
                 mask_source='depth_fallback'):
        """记录调用并返回 ACCEPT 罐头结果."""
        self.calls.append((target_id, tuple(bbox), mask_source))
        g2d = BagGrasp2D(detection_bbox=tuple(bbox), status='ACCEPT')
        g3d = BagGraspReference3D(status='ACCEPT')
        return TargetPoseResult(target_id, g2d, g3d, mask_source, {})


def _dense_obs(class_id: int) -> BagObservation:
    """40×40 满有效深度（1 m）观测：掩膜构造全程可走通."""
    return BagObservation(
        rgb=np.zeros((40, 40, 3), dtype=np.uint8),
        depth=np.full((40, 40), 1000, dtype=np.uint16),
        camera_K={'fx': 100.0, 'fy': 100.0, 'cx': 20.0, 'cy': 20.0},
        detections=[{'bbox': (0, 0, 40, 40), 'class_id': class_id,
                     'conf': 0.9}])


class CandidateRoutingContractTest(unittest.TestCase):
    """CandidateEstimator 只依赖 PosePipeline 接口（fake 驱动路由与估计）."""

    def test_fake_pipelines_drive_full_estimate_path(self):
        bag = FakePosePipeline('bag')
        fruit = FakePosePipeline('fruit', tool=bag.tool)
        est = CandidateEstimator(
            pipeline=bag, fruit_pipeline=fruit, min_mask_points=10)
        sam_mask = np.ones((40, 40), dtype=bool)
        # class_id=0 → 袋线 fake；hybrid_dilated 掩膜可构造（满深度满掩膜）
        results = est.estimate_modes(
            _dense_obs(0), 'target_0', (0, 0, 40, 40), sam_mask)
        result = results['hybrid_dilated']
        self.assertEqual(len(bag.calls), 1)
        self.assertEqual(len(fruit.calls), 0)
        self.assertEqual(result.grasp_3d.status, 'ACCEPT')
        self.assertEqual(result.target_kind, 'bag')
        self.assertEqual(
            result.grasp_3d.strategy_id, 'robust_bag_pose:hybrid_dilated')
        # class_id=1 → 果线 fake（同一调用端代码路径）
        results = est.estimate_modes(
            _dense_obs(1), 'target_1', (0, 0, 40, 40), sam_mask)
        self.assertEqual(len(fruit.calls), 1)
        self.assertEqual(results['hybrid_dilated'].target_kind, 'fruit')

    def test_mask_unavailable_short_circuits_before_pipeline(self):
        """SAM 缺失 → 显式 REOBSERVE，fake 的 estimate 不被调用."""
        bag = FakePosePipeline('bag')
        est = CandidateEstimator(
            pipeline=bag, fruit_pipeline=FakePosePipeline('fruit'))
        results = est.estimate_modes(
            _dense_obs(0), 'target_0', (0, 0, 40, 40), None)
        self.assertEqual(len(bag.calls), 0)
        result = results['hybrid_dilated']
        self.assertEqual(result.grasp_3d.status, 'REOBSERVE')
        self.assertIn('mask_unavailable', result.grasp_3d.diagnostic_flags)


class FakeMatcher(TargetMatcher):
    """脚本化命中序列的 fake 匹配器（不持表，表由注册表传入）."""

    def __init__(self, hits):
        """hits: 逐次 match 调用应返回的 target_id（None=未命中）."""
        self._hits = list(hits)
        self.calls = []

    @property
    def match_radius(self):
        """匹配半径（注册表 match_radius 属性透传用）."""
        return 0.06

    def match(self, anchor, class_id, table, frame_used):
        """弹出脚本化命中结果并记录入参（表由调用方传入的契约）."""
        target_id = self._hits.pop(0) if self._hits else None
        self.calls.append((tuple(anchor), class_id,
                           sorted(table), set(frame_used)))
        return MatchResult(target_id=target_id, distance=0.01)


class TargetRegistryContractTest(unittest.TestCase):
    """TargetRegistry 只依赖 TargetMatcher 接口（fake 驱动命中/注册路径）."""

    def test_fake_matcher_drives_register_and_hit_paths(self):
        # 第 1 次未命中 → 注册 target_0；第 2 次脚本命中 target_0 → EMA 路径
        matcher = FakeMatcher([None, 'target_0'])
        reg = TargetRegistry(matcher=matcher, position_ema=0.5)
        tid, is_new = reg.match_or_register(
            [0.0, 0.0, 1.0], class_id=0, status='ACCEPT', now=1.0)
        self.assertEqual((tid, is_new), ('target_0', True))
        # fake 收到的表就是注册表内部的表（不持表契约）
        _, _, table_keys, _ = matcher.calls[0]
        self.assertEqual(table_keys, [])
        reg.begin_frame()
        tid, is_new = reg.match_or_register(
            [0.0, 0.0, 2.0], class_id=0, status='ACCEPT', now=2.0)
        self.assertEqual((tid, is_new), ('target_0', False))
        _, _, table_keys, _ = matcher.calls[1]
        self.assertEqual(table_keys, ['target_0'])
        # 命中后 EMA 由注册表完成：0.5×[0,0,1] + 0.5×[0,0,2]
        np.testing.assert_allclose(reg.get('target_0')['position'],
                                   [0.0, 0.0, 1.5])
        self.assertEqual(reg.stats()['n_matched'], 1)
        self.assertEqual(reg.stats()['n_registered'], 1)


class FakeLockPolicy(LockPolicy):
    """脚本化 LockEvent 的 fake 锁定策略（不持锁定后状态）."""

    def __init__(self, event_on_update):
        """event_on_update: 第几次 update 返回 LockEvent（1 起计）."""
        self.min_collect_frames = 1
        self.lock_settle_frames = 0
        self.max_collect_s = 100.0
        self._event_on_update = event_on_update
        self._n_updates = 0
        self.reset_count = 0
        self.received_now = []

    def update(self, records, now):
        """到达脚本帧数时返回 LockEvent（records 原样打包）."""
        self._n_updates += 1
        self.received_now.append(now)
        if self._n_updates == self._event_on_update:
            return LockEvent(records=tuple(records))
        return None

    def reset(self):
        """记录 reset 并允许再次触发."""
        self.reset_count += 1
        self._n_updates = 0


def _rec(target_id, distance=1.0, confirmed=True):
    """最小候选记录（排序键只需 target_id/距离/确认位）."""
    return {'target_id': target_id, 'confirmed': confirmed, 'status': 0,
            'camera_distance_m': distance, 'confidence': 0.9,
            'diagnostic_flags': []}


class HarvestPlanContractTest(unittest.TestCase):
    """GlobalHarvestPlan 只依赖 LockPolicy 接口（fake 驱动锁定/推进/重置）."""

    def test_fake_policy_drives_lock_advance_and_reset(self):
        policy = FakeLockPolicy(event_on_update=2)
        plan = GlobalHarvestPlan(
            max_targets=5, prefer_lower_first=True, lock_policy=policy)
        frame = [_rec('far', distance=1.2), _rec('near', distance=0.7)]
        plan.update(frame, now=1.0)
        self.assertFalse(plan.locked)
        current = plan.update(frame, now=2.0)
        # LockEvent 到达 → plan 一次性排序锁定（先近后远）
        self.assertTrue(plan.locked)
        self.assertEqual(plan.locked_ids, ('near', 'far'))
        self.assertEqual(plan.selected_target_id, 'near')
        self.assertEqual(plan.snapshot_id, 1)
        self.assertEqual(set(current), {'near', 'far'})
        # now 显式透传到策略（协议 I3：plan 不自行取时钟）
        self.assertEqual(policy.received_now, [1.0, 2.0])
        # 推进与状态查询走 plan 自身记账，与策略无关
        self.assertEqual(plan.complete_selected(), 'far')
        self.assertEqual(plan.harvest_status('near'), 'HARVESTED')
        # reset 联动策略重置，下一轮可重新锁定
        plan.reset()
        self.assertEqual(policy.reset_count, 1)
        self.assertFalse(plan.locked)
        plan.update(frame, now=3.0)
        plan.update(frame, now=4.0)
        self.assertTrue(plan.locked)
        self.assertEqual(plan.snapshot_id, 2)

    def test_max_collect_s_passthrough_to_policy(self):
        """帧率自适应写 plan.max_collect_s 透传到策略实例."""
        policy = FakeLockPolicy(event_on_update=99)
        plan = GlobalHarvestPlan(lock_policy=policy)
        plan.max_collect_s = 12.5
        self.assertEqual(policy.max_collect_s, 12.5)
        self.assertEqual(plan.min_collect_frames, 1)
        self.assertEqual(plan.lock_settle_frames, 0)


if __name__ == '__main__':
    unittest.main()
