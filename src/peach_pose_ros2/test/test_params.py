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
        """ros__parameters 与 DECLARE_DEFAULTS 键集与逐值完全相等（41 项）."""
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
    """declare(node)：41 键全量 declare，默认值与 descriptor 原样透传."""

    def test_declare_registers_all_defaults(self):
        node = _FakeNode({})
        PeachPoseParams.declare(node)
        self.assertEqual(len(node.declared), 41)
        for key, (default, descriptor) in node.declared.items():
            self.assertEqual(default, DECLARE_DEFAULTS[key], key)
            self.assertEqual(descriptor.description, DESCRIPTIONS[key], key)


class FromNodeTest(unittest.TestCase):
    """from_node：默认装载、派生字段组装、非法值校验（假节点，无 ROS 上下文）."""

    def test_defaults_load_and_derived_fields(self):
        """默认字典装载：tool 组装 entry_standoff=和值，注册表按默认开启构建."""
        p = PeachPoseParams.from_node(_FakeNode(DECLARE_DEFAULTS))
        self.assertEqual(p.color_topic, '/camera/color/image_raw')
        self.assertEqual(p.sync_slop_s, 0.05)
        self.assertIsNone(p.gravity_hint)
        self.assertEqual(p.gravity_mode, 'fixed')
        self.assertAlmostEqual(p.tool.entry_standoff, 0.070)
        self.assertAlmostEqual(p.tool.D_inner, 0.104)
        self.assertTrue(p.target_memory_enable)
        self.assertIsNotNone(p.target_registry)
        self.assertAlmostEqual(p.target_registry.match_radius, 0.06)

    def test_gravity_hint_parsed_and_registry_disabled(self):
        """gravity_hint_xyz 解析为 ndarray；target_memory.enable=false → 注册表 None."""
        values = dict(DECLARE_DEFAULTS)
        values['gravity_hint_xyz'] = '0.0, 1.0, 0.0'
        values['target_memory.enable'] = False
        p = PeachPoseParams.from_node(_FakeNode(values))
        np.testing.assert_allclose(p.gravity_hint, [0.0, 1.0, 0.0])
        self.assertFalse(p.target_memory_enable)
        self.assertIsNone(p.target_registry)

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
