"""参数层：yaml↔declare 默认值双向同步 + 字段集一致 + 装载/冻结语义."""
import dataclasses
from pathlib import Path
import unittest

from peach_reconstruction_ros2.params import ReconstructionParams
import yaml

_CONFIG = (Path(__file__).resolve().parents[1]
           / 'config' / 'reconstruction.yaml')


class _FakeParam:
    """鸭子类型参数值（对齐 rclpy Parameter 的 .value 访问面）."""

    def __init__(self, value):
        self.value = value


class _FakeNode:
    """鸭子类型节点：declare_parameter 记录默认值，get_parameter 读回."""

    def __init__(self):
        self.declared = {}

    def declare_parameter(self, name, default, _descriptor=None):
        self.declared[name] = default

    def get_parameter(self, name):
        return _FakeParam(self.declared[name])


class ParamsSyncTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.yaml_params = yaml.safe_load(_CONFIG.read_text(
            encoding='utf-8'))['peach_reconstruction_node']['ros__parameters']
        cls.code_defaults = ReconstructionParams.defaults_flat()

    def test_yaml_and_declare_keys_bidirectional(self):
        """YAML 键集 == declare 键集（双向，35 个）."""
        self.assertEqual(set(self.yaml_params), set(self.code_defaults))
        self.assertEqual(len(self.code_defaults), 47)

    def test_yaml_and_declare_values_equal(self):
        """YAML 默认值 == declare 默认值（逐项，类型与数值一致）."""
        for name, code_value in self.code_defaults.items():
            self.assertIn(name, self.yaml_params)
            self.assertEqual(self.yaml_params[name], code_value,
                             f'{name}: yaml={self.yaml_params[name]!r} '
                             f'!= declare={code_value!r}')

    def test_field_set_matches_ros_names(self):
        """Dataclass 字段集（组前缀拼接）== ROS 参数名集（无遗漏/无冗余）."""
        inst = ReconstructionParams()
        flat = set()
        for group_name in ('frames', 'camera', 'capture', 'view_filter',
                           'icp', 'local_volume', 'tsdf', 'cloud_filter', 'refit',
                           'session'):
            group = getattr(inst, group_name)
            flat |= {f'{group_name}.{k}' for k in vars(group)}
        flat |= {'sync_slop_s', 'tf_timeout_sec', 'depth_scale_unit'}
        self.assertEqual(flat, set(self.code_defaults))


class ParamsLoadTest(unittest.TestCase):
    def test_declare_from_node_roundtrip_defaults(self):
        """declare→from_node 全默认往返：装载值 == 字段默认值."""
        node = _FakeNode()
        ReconstructionParams.declare(node)
        self.assertEqual(set(node.declared),
                         set(ReconstructionParams.defaults_flat()))
        params = ReconstructionParams.from_node(node)
        self.assertEqual(params.capture.min_views, 4)
        self.assertEqual(params.view_filter.min_translation, 0.002)
        self.assertEqual(params.icp.max_translation, 0.010)
        self.assertEqual(params.tsdf.voxel_length, 0.003)
        self.assertEqual(params.refit.entry_standoff_m, 0.070)
        self.assertEqual(params.frames.base_frame, 'base_link')
        self.assertEqual(params.session.root_dir, '')

    def test_from_node_reads_override_and_strips(self):
        """from_node 读节点覆盖值；字符串字段 strip（base_frame/root_dir 语义）."""
        node = _FakeNode()
        ReconstructionParams.declare(node)
        node.declared['capture.min_views'] = 2
        node.declared['frames.base_frame'] = '  base_link  '
        params = ReconstructionParams.from_node(node)
        self.assertEqual(params.capture.min_views, 2)
        self.assertEqual(params.frames.base_frame, 'base_link')

    def test_params_frozen(self):
        """Frozen 语义：装载后改字段抛 FrozenInstanceError."""
        node = _FakeNode()
        ReconstructionParams.declare(node)
        params = ReconstructionParams.from_node(node)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            params.depth_scale_unit = 1.0


if __name__ == '__main__':
    unittest.main()
