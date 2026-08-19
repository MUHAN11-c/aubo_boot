"""observation_quality：跟踪状态四分类映射 + 检测框触边 + 光照质量统计."""
import unittest

from peach_pose_ros2.peach_pose.observation_quality import (
    bbox_touches_image_edge,
    classify_tracking_status,
    LightingMeter,
    STATUS_DEPTH_VOID,
    STATUS_LOST,
    STATUS_OBSERVED,
    STATUS_OCCLUDED,
    STATUS_OUT_OF_VIEW,
)


class BboxEdgeTest(unittest.TestCase):
    """OUT_OF_VIEW 证据原语：检测框是否触图像边缘（640×480）."""

    def test_inner_box_does_not_touch(self):
        """图内框（含贴边前 1 像素）不判触边."""
        self.assertFalse(bbox_touches_image_edge((10, 10, 100, 100), 640, 480))
        self.assertFalse(bbox_touches_image_edge((1, 1, 639, 479), 640, 480))

    def test_each_edge_touches(self):
        """四边各自贴边均判触边（含出界裁剪后的越界坐标）."""
        self.assertTrue(bbox_touches_image_edge((0, 10, 100, 100), 640, 480))
        self.assertTrue(bbox_touches_image_edge((10, 0, 100, 100), 640, 480))
        self.assertTrue(bbox_touches_image_edge((10, 10, 640, 100), 640, 480))
        self.assertTrue(bbox_touches_image_edge((10, 10, 100, 480), 640, 480))


class ClassifyTrackingStatusTest(unittest.TestCase):
    """四分类映射（阶段 D1）：优先级 无观测 > 掩膜缺失 > 深度空洞 > 正常."""

    def test_disappeared_target_splits_by_edge_evidence(self):
        """无观测：触边消失 → OUT_OF_VIEW；图内消失 → LOST."""
        self.assertEqual(
            classify_tracking_status(
                has_observation=False, has_mask=False, mask_depth_ratio=None,
                min_depth_ratio=0.35, last_bbox_touched_edge=True),
            STATUS_OUT_OF_VIEW)
        self.assertEqual(
            classify_tracking_status(
                has_observation=False, has_mask=False, mask_depth_ratio=None,
                min_depth_ratio=0.35, last_bbox_touched_edge=False),
            STATUS_LOST)

    def test_observed_without_mask_is_occluded(self):
        """有观测无掩膜 → OCCLUDED（边缘证据不参与——目标还在视野内）."""
        self.assertEqual(
            classify_tracking_status(
                has_observation=True, has_mask=False, mask_depth_ratio=None,
                min_depth_ratio=0.35, last_bbox_touched_edge=True),
            STATUS_OCCLUDED)

    def test_mask_with_low_depth_ratio_is_depth_void(self):
        """掩膜在但有效深度占比低于阈值 → DEPTH_VOID；占比缺失按 0 保守判."""
        self.assertEqual(
            classify_tracking_status(
                has_observation=True, has_mask=True, mask_depth_ratio=0.2,
                min_depth_ratio=0.35, last_bbox_touched_edge=False),
            STATUS_DEPTH_VOID)
        self.assertEqual(
            classify_tracking_status(
                has_observation=True, has_mask=True, mask_depth_ratio=None,
                min_depth_ratio=0.35, last_bbox_touched_edge=False),
            STATUS_DEPTH_VOID)

    def test_mask_with_good_depth_ratio_is_observed(self):
        """掩膜与深度都达标 → OBSERVED；边界值（==阈值）判达标."""
        self.assertEqual(
            classify_tracking_status(
                has_observation=True, has_mask=True, mask_depth_ratio=0.6,
                min_depth_ratio=0.35, last_bbox_touched_edge=False),
            STATUS_OBSERVED)
        self.assertEqual(
            classify_tracking_status(
                has_observation=True, has_mask=True, mask_depth_ratio=0.35,
                min_depth_ratio=0.35, last_bbox_touched_edge=False),
            STATUS_OBSERVED)


class LightingMeterTest(unittest.TestCase):
    """光照质量：EMA 收敛、连续低质判定、空样本帧跳过、脏样本过滤."""

    def test_low_quality_after_consecutive_bad_frames(self):
        """深度占比持续低于阈值，连击满 bad_frames 置 low_quality."""
        meter = LightingMeter(alpha=0.3, min_depth_ratio=0.35,
                              min_conf_mean=0.3, bad_frames=5)
        for frame in range(4):
            meter.update([0.2, 0.25], [0.9])
            self.assertFalse(meter.low_quality, frame)
        meter.update([0.2, 0.25], [0.9])
        self.assertTrue(meter.low_quality)
        snap = meter.snapshot()
        self.assertEqual(snap['bad_streak'], 5)
        self.assertTrue(snap['low_quality'])
        self.assertAlmostEqual(snap['conf_mean'], 0.9)

    def test_good_frame_resets_streak(self):
        """连击中途一帧双达标即清零（对称清除，与摆动判定同风格）."""
        meter = LightingMeter(alpha=1.0, min_depth_ratio=0.35,
                              min_conf_mean=0.3, bad_frames=3)
        meter.update([0.2], [0.9])      # 低质 1（深度不达标）
        meter.update([0.2], [0.9])      # 低质 2
        meter.update([0.8], [0.9])      # 达标 → 清零
        meter.update([0.2], [0.9])      # 低质 1（重新计）
        meter.update([0.2], [0.9])      # 低质 2
        self.assertFalse(meter.low_quality)
        meter.update([0.2], [0.9])      # 低质 3 → 置位
        self.assertTrue(meter.low_quality)

    def test_conf_dimension_alone_can_trigger(self):
        """置信度均值 EMA 低于阈值同样计低质（两维度任一）."""
        meter = LightingMeter(alpha=1.0, min_depth_ratio=0.35,
                              min_conf_mean=0.3, bad_frames=2)
        meter.update([0.9], [0.2])      # 深度达标、置信度不达标
        self.assertFalse(meter.low_quality)
        meter.update([0.9], [0.2])
        self.assertTrue(meter.low_quality)

    def test_empty_or_dirty_samples_are_skipped(self):
        """空样本帧不进 EMA 不计低质；nan 样本被过滤不污染估计."""
        meter = LightingMeter(alpha=0.5, min_depth_ratio=0.35,
                              min_conf_mean=0.3, bad_frames=2)
        meter.update([0.9], [0.9])
        meter.update([], [])            # 锁定集目标全部无掩膜观测：跳过
        self.assertEqual(meter.snapshot()['bad_streak'], 0)
        meter.update([float('nan')], [0.9])   # 脏样本过滤后深度分量不更新
        self.assertAlmostEqual(meter.snapshot()['depth_ratio'], 0.9)

    def test_invalid_params_rejected(self):
        """α/阈值/连击帧数区间校验."""
        with self.assertRaises(ValueError):
            LightingMeter(alpha=0.0)
        with self.assertRaises(ValueError):
            LightingMeter(min_depth_ratio=1.5)
        with self.assertRaises(ValueError):
            LightingMeter(min_conf_mean=-0.1)
        with self.assertRaises(ValueError):
            LightingMeter(bad_frames=0)


if __name__ == '__main__':
    unittest.main()
