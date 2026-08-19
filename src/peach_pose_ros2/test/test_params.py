"""params.py 参数层：yaml↔declare 双向同步 + from_node 装载/校验/组装."""
import dataclasses
from pathlib import Path
import unittest

import numpy as np

from peach_pose_ros2.params import (
    DECLARE_DEFAULTS,
    DESCRIPTIONS,
    PeachPoseParams,
)

YAML_PATH = Path(__file__).resolve().parents[1] / 'config' / 'peach_pose.yaml'


class _FakeParam:
    """最小参数替身：from_node 只触 .value 与 .get_parameter_value().string_value."""

    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        return type('V', (), {'string_value': self.value})()


class _FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, msg):
        self.warnings.append(msg)


class _FakeNode:
    """不依赖 ROS 上下文的节点替身：declare 捕获 + 按字典供参."""

    def __init__(self, values):
        self._values = dict(values)
        self.declared = {}
        self.logger = _FakeLogger()

    def declare_parameter(self, name, default, descriptor):
        self.declared[name] = (default, descriptor)

    def get_parameter(self, name):
        return _FakeParam(self._values[name])

    def get_logger(self):
        return self.logger


class YamlDeclareSyncTest(unittest.TestCase):
    """YAML 权威哲学强制：config/peach_pose.yaml 与 declare 默认值逐项一致."""

    def test_yaml_defaults_match_declare_defaults(self):
        """ros__parameters 与 DECLARE_DEFAULTS 键集与逐值完全相等（63 项）."""
        import yaml  # 延迟 import：仅本测试需要 PyYAML
        doc = yaml.safe_load(YAML_PATH.read_text())
        yaml_params = doc['peach_pose_node']['ros__parameters']
        self.assertEqual(set(yaml_params), set(DECLARE_DEFAULTS))
        for key, declare_val in DECLARE_DEFAULTS.items():
            self.assertEqual(
                yaml_params[key], declare_val,
                f'{key}: yaml={yaml_params[key]!r} vs declare={declare_val!r}')

    def test_every_declare_key_has_chinese_description(self):
        """每个 declare 键都有非空中文 descriptor（与节点原行为一致）."""
        self.assertEqual(set(DESCRIPTIONS), set(DECLARE_DEFAULTS))
        for key, desc in DESCRIPTIONS.items():
            self.assertTrue(desc.strip(), key)


class DeclareCollectTest(unittest.TestCase):
    """declare(node)：63 键全量 declare，默认值与 descriptor 原样透传."""

    def test_declare_registers_all_defaults(self):
        node = _FakeNode({})
        PeachPoseParams.declare(node)
        self.assertEqual(len(node.declared), 63)
        for key, (default, descriptor) in node.declared.items():
            self.assertEqual(default, DECLARE_DEFAULTS[key], key)
            self.assertEqual(descriptor.description, DESCRIPTIONS[key], key)


class FromNodeTest(unittest.TestCase):
    """from_node：默认装载、派生字段组装、非法值校验（假节点，无 ROS 上下文）."""

    def test_defaults_load_and_derived_fields(self):
        """默认字典装载：tool 组装 entry_standoff=和值，*.impl 注册名透传."""
        p = PeachPoseParams.from_node(_FakeNode(DECLARE_DEFAULTS))
        self.assertEqual(p.color_topic, '/camera/color/image_raw')
        self.assertEqual(p.sync_slop_s, 0.05)
        self.assertIsNone(p.gravity_hint)
        self.assertEqual(p.gravity_mode, 'fixed')
        self.assertAlmostEqual(p.tool.entry_standoff, 0.070)
        self.assertAlmostEqual(p.tool.D_inner, 0.104)
        self.assertTrue(p.target_memory_enable)
        # 2.14 装配：实现注册名按默认装载（实例由节点按名 create）
        self.assertEqual(p.detector_impl, 'yolo')
        self.assertEqual(p.segmenter_impl, 'mobile_sam')
        self.assertEqual(p.pipeline_bag_impl, 'robust_bag')
        self.assertEqual(p.pipeline_fruit_impl, 'robust_fruit')
        self.assertEqual(p.matcher_impl, 'spatial_ema')
        self.assertEqual(p.lock_impl, 'collect_lock')

    def test_gravity_hint_parsed(self):
        """gravity_hint_xyz 解析为 ndarray."""
        values = dict(DECLARE_DEFAULTS)
        values['gravity_hint_xyz'] = '0.0, 1.0, 0.0'
        p = PeachPoseParams.from_node(_FakeNode(values))
        np.testing.assert_allclose(p.gravity_hint, [0.0, 1.0, 0.0])

    def test_bad_gravity_hint_raises(self):
        """gravity_hint_xyz 非 3 分量抛 ValueError（与节点原行为一致）."""
        values = dict(DECLARE_DEFAULTS)
        values['gravity_hint_xyz'] = '1,2'
        with self.assertRaises(ValueError):
            PeachPoseParams.from_node(_FakeNode(values))

    def test_unknown_gravity_mode_falls_back_with_warning(self):
        """未知 gravity_mode 告警并回退 fixed（白名单校验）."""
        values = dict(DECLARE_DEFAULTS)
        values['gravity_mode'] = 'bogus'
        node = _FakeNode(values)
        p = PeachPoseParams.from_node(node)
        self.assertEqual(p.gravity_mode, 'fixed')
        self.assertEqual(len(node.logger.warnings), 1)

    def test_detection_cloud_stride_clamped(self):
        """detection_cloud_stride <1 clamp 到 1."""
        values = dict(DECLARE_DEFAULTS)
        values['detection_cloud_stride'] = 0
        p = PeachPoseParams.from_node(_FakeNode(values))
        self.assertEqual(p.detection_cloud_stride, 1)

    def test_outdoor_perception_defaults_loaded(self):
        """阶段 D1 新增参数：默认值装载（锚点衰减/摆动/光照/容量/深度窗）."""
        p = PeachPoseParams.from_node(_FakeNode(DECLARE_DEFAULTS))
        self.assertAlmostEqual(p.target_memory_anchor_max_age_s, 30.0)
        self.assertAlmostEqual(p.target_memory_anchor_drop_s, 120.0)
        self.assertAlmostEqual(p.target_memory_max_age_s, 600.0)
        self.assertAlmostEqual(p.wind_swing_threshold_m, 0.03)
        self.assertEqual(p.wind_swing_frames, 3)
        self.assertAlmostEqual(p.lighting_min_depth_ratio, 0.35)
        self.assertAlmostEqual(p.lighting_min_conf_mean, 0.3)
        self.assertEqual(p.lighting_bad_frames, 5)
        self.assertEqual(p.sam_max_bboxes, 16)
        self.assertEqual(p.sam_min_area, 100)
        self.assertAlmostEqual(p.yolo_nms_iou, 0.5)
        self.assertEqual(p.min_mask_points, 50)
        self.assertAlmostEqual(p.pipeline_min_depth_m, 0.3)
        self.assertAlmostEqual(p.pipeline_max_depth_m, 2.5)
        self.assertEqual(p.pipeline_min_points, 100)

    def test_outdoor_perception_overrides_and_clamps(self):
        """阶段 D1 新增参数：覆盖装载与整数 clamp（sam_max_bboxes 等 ≥1）."""
        values = dict(DECLARE_DEFAULTS)
        values['wind.swing_frames'] = 5
        values['sam_max_bboxes'] = 0
        values['min_mask_points'] = 0
        values['pipeline.min_points'] = 40
        p = PeachPoseParams.from_node(_FakeNode(values))
        self.assertEqual(p.wind_swing_frames, 5)
        self.assertEqual(p.sam_max_bboxes, 1)
        self.assertEqual(p.min_mask_points, 1)
        self.assertEqual(p.pipeline_min_points, 40)

    def test_stage_h_efficiency_defaults(self):
        """阶段 H（2.13-E1 + debug 默认关）：新开关默认 true，debug 图默认关."""
        p = PeachPoseParams.from_node(_FakeNode(DECLARE_DEFAULTS))
        self.assertTrue(p.pipeline_locked_only_segmentation)
        self.assertFalse(p.publish_debug_image)

    def test_stage_h_efficiency_overrides(self):
        """阶段 H 参数覆盖装载：锁定后全量分割回退 / debug 图显式开."""
        values = dict(DECLARE_DEFAULTS)
        values['pipeline.locked_only_segmentation'] = False
        values['publish_debug_image'] = True
        p = PeachPoseParams.from_node(_FakeNode(values))
        self.assertFalse(p.pipeline_locked_only_segmentation)
        self.assertTrue(p.publish_debug_image)


class FieldSetConsistencyTest(unittest.TestCase):
    """dataclass 字段与 declare 键的映射完备（防新增参数漏装载）."""

    def test_declare_keys_covered_by_fields(self):
        """每个 declare 键要么同名字段（点号换下划线），要么属 tool.* 组装."""
        fields = {f.name for f in dataclasses.fields(PeachPoseParams)}
        tool = PeachPoseParams.from_node(_FakeNode(DECLARE_DEFAULTS)).tool
        for key in DECLARE_DEFAULTS:
            if key.startswith('tool.'):
                self.assertTrue(hasattr(tool, key[5:]), key)
            else:
                self.assertIn(key.replace('.', '_'), fields, key)


if __name__ == '__main__':
    unittest.main()
