"""segmentation_gate 纯核：锁定后 selected-only SAM 门控（阶段 H，协议 2.13-E1）."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.segmentation_gate import (
    plan_segmentation_bboxes,
    project_positions_to_pixels,
)

# 640x480 理想针孔内参（f=500）：z=1m 处世界系 (x, y) 米 ↔ 像素 (320+500x, 240+500y)
K = {'fx': 500.0, 'fy': 500.0, 'cx': 320.0, 'cy': 240.0, 'width': 640,
     'height': 480}


def _det(bbox):
    """构造最小检测 dict（门控只读 'bbox' 键）."""
    return {'bbox': bbox}


class ProjectPositionsToPixelsTest(unittest.TestCase):
    """世界系锚点 → 本帧像素：正投/剔除语义."""

    def test_identity_transform_projects_pinhole(self):
        """单位变换下 pinhole 投影：光轴点→主点，偏移点→线性像素偏移."""
        px = project_positions_to_pixels(
            {'target_0': np.array([0.0, 0.0, 1.0]),
             'target_1': np.array([0.1, -0.2, 1.0])},
            np.eye(4), K)
        self.assertEqual(px['target_0'], (320.0, 240.0))
        self.assertAlmostEqual(px['target_1'][0], 370.0)
        self.assertAlmostEqual(px['target_1'][1], 140.0)

    def test_transform_rotation_translation_applied(self):
        """T_cam_world 的平移真实生效（相机系 x 平移 0.1m → u 右移 50px）."""
        T = np.eye(4)
        T[0, 3] = 0.1  # 世界系锚点在相机系 x 方向 +0.1m
        px = project_positions_to_pixels(
            {'target_0': np.array([0.0, 0.0, 1.0])}, T, K)
        self.assertAlmostEqual(px['target_0'][0], 320.0 + 50.0)

    def test_behind_camera_and_outside_image_dropped(self):
        """相机后方 / 图像外 / 非有限锚点一律剔除（视为本帧不可见）."""
        px = project_positions_to_pixels(
            {'behind': np.array([0.0, 0.0, -1.0]),
             'outside': np.array([5.0, 0.0, 1.0]),   # u=2820 超出 640
             'nan': np.array([np.nan, 0.0, 1.0]),
             'visible': np.array([0.0, 0.0, 1.0])},
            np.eye(4), K)
        self.assertEqual(list(px), ['visible'])

    def test_no_image_size_keeps_offscreen_projection(self):
        """内参缺 width/height 时不做视野裁剪（仅剔相机后方）."""
        k = {key: K[key] for key in ('fx', 'fy', 'cx', 'cy')}
        px = project_positions_to_pixels(
            {'outside': np.array([5.0, 0.0, 1.0])}, np.eye(4), k)
        self.assertIn('outside', px)


class PlanSegmentationBboxesTest(unittest.TestCase):
    """门控策略矩阵：锁定前/开关关/TF 不可用全量，锁定后只锁定集."""

    def setUp(self):
        """两框场景：A 含锁定锚点 (150, 150)，B 不含任何锚点."""
        self.dets = [_det((100, 100, 200, 200)), _det((400, 300, 500, 400))]

    def test_unlocked_returns_all(self):
        """锁定前行为不变：全量框照常（收齐窗口期新目标也要掩膜确认）."""
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=False,
            anchor_px={'target_0': (150.0, 150.0)})
        self.assertEqual(out, [(100, 100, 200, 200), (400, 300, 500, 400)])

    def test_switch_off_returns_all(self):
        """开关关闭回退旧行为：锁定后仍全量分割."""
        out = plan_segmentation_bboxes(
            self.dets, locked_only=False, locked=True,
            anchor_px={'target_0': (150.0, 150.0)})
        self.assertEqual(len(out), 2)

    def test_anchor_none_falls_back_to_all(self):
        """锁定但锚点不可投影（TF 不可用帧）：无法识别锁定框，宁多勿漏."""
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=True, anchor_px=None)
        self.assertEqual(len(out), 2)

    def test_locked_selects_only_anchor_containing_box(self):
        """锁定后只分割包含锁定锚点投影的框（segment_ms 随框数下降）."""
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=True,
            anchor_px={'target_0': (150.0, 150.0)})
        self.assertEqual(out, [(100, 100, 200, 200)])

    def test_locked_empty_anchor_set_segments_nothing(self):
        """锁定目标本帧全部不可见（空锚点集）：SAM 零推理."""
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=True, anchor_px={})
        self.assertEqual(out, [])

    def test_anchor_on_edge_selected_via_margin(self):
        """锚点恰好落在框外扩 margin 内仍选中（框回归抖动兜底）."""
        # 框 A 右缘 x=200，margin=10%×100px=10 → 210 内的锚点仍选中
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=True,
            anchor_px={'target_0': (209.0, 150.0)})
        self.assertEqual(out, [(100, 100, 200, 200)])
        # 超出 margin 则不选中
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=True,
            anchor_px={'target_0': (211.0, 150.0)})
        self.assertEqual(out, [])

    def test_multiple_anchors_select_union(self):
        """多锁定目标各自选中所在框（并集，顺序与检测序一致）."""
        out = plan_segmentation_bboxes(
            self.dets, locked_only=True, locked=True,
            anchor_px={'target_0': (150.0, 150.0), 'target_1': (450.0, 350.0)})
        self.assertEqual(out, [(100, 100, 200, 200), (400, 300, 500, 400)])


if __name__ == '__main__':
    unittest.main()
