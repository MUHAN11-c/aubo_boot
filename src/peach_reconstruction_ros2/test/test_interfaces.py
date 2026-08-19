"""接口层：实现均为 ABC 实例 + Registry 登记 + ABC 抽象强制 + 默认清单."""
import importlib.util
import unittest

import numpy as np

from peach_reconstruction_ros2.cloud_builder import Open3dCloudBuilder
from peach_reconstruction_ros2.frame_collector import FrameCollector
from peach_reconstruction_ros2.geometry_refiner import (
    CylinderRefitter,
    SphereRefitter,
)
from peach_reconstruction_ros2.icp_refiner import BoundedIcp, IcpConfig
from peach_reconstruction_ros2.interfaces import (
    CLOUD_BUILDERS,
    CloudBuilder,
    FRAME_STORES,
    FrameStore,
    MASK_GATES,
    MaskGate,
    Refiner,
    REFINERS,
    Refitter,
    REFITTERS,
    Volume,
    VOLUMES,
)
from peach_reconstruction_ros2.mask_gate import StrictMaskGate
from peach_reconstruction_ros2.tsdf_volume import LocalTsdf

_HAS_O3D = importlib.util.find_spec('open3d') is not None


class InterfaceContractTest(unittest.TestCase):
    def test_implementations_are_abc_instances(self):
        """可实例化的实现均为对应 ABC 实例."""
        self.assertIsInstance(FrameCollector(), FrameStore)
        self.assertIsInstance(Open3dCloudBuilder(), CloudBuilder)
        self.assertIsInstance(BoundedIcp(IcpConfig()), Refiner)
        self.assertIsInstance(CylinderRefitter(), Refitter)
        self.assertIsInstance(SphereRefitter(), Refitter)
        self.assertIsInstance(StrictMaskGate(), MaskGate)
        # LocalTsdf 构造依赖 open3d，类级契约用 issubclass 验证
        self.assertTrue(issubclass(LocalTsdf, Volume))

    def test_registries_contain_default_implementations(self):
        """六个 Registry 按默认注册名登记实现（显式注册清单，协议 2.14）."""
        self.assertIs(FRAME_STORES.create('default').__class__, FrameCollector)
        self.assertIs(CLOUD_BUILDERS.create('open3d_cloud').__class__,
                      Open3dCloudBuilder)
        self.assertIs(REFINERS.create('bounded_icp',
                                      config=IcpConfig()).__class__, BoundedIcp)
        self.assertIs(REFITTERS.create('cylinder_refit').__class__,
                      CylinderRefitter)
        self.assertIs(REFITTERS.create('sphere_refit').__class__,
                      SphereRefitter)
        self.assertIs(MASK_GATES.create('strict_mask_gate').__class__,
                      StrictMaskGate)
        self.assertEqual(
            set(REFITTERS.names()), {'cylinder_refit', 'sphere_refit'})

    def test_registry_create_rejects_unknown_name(self):
        """未注册名 create 抛 KeyError，错误信息列出可用名称."""
        with self.assertRaises(KeyError) as ctx:
            CLOUD_BUILDERS.create('no_such_impl')
        self.assertIn('open3d_cloud', str(ctx.exception))

    def test_registry_rejects_duplicate_registration(self):
        """同名重复注册抛 ValueError（显式清单防静默覆盖）."""
        with self.assertRaises(ValueError):
            CLOUD_BUILDERS.register('open3d_cloud', Open3dCloudBuilder)

    def test_abc_enforces_abstract_methods(self):
        """ABC 抽象强制：缺实现的子类不可实例化."""
        for abc in (FrameStore, CloudBuilder, Refiner, Volume, Refitter,
                    MaskGate):
            class _Bad(abc):
                pass
            with self.assertRaises(TypeError):
                _Bad()

    @unittest.skipUnless(_HAS_O3D, 'open3d 不可用（系统 python3 跳过）')
    def test_volume_instance_and_reset(self):
        """融合体积实现是 Volume 实例；reset 清空体积与耗时（契约钩子）."""
        volume = VOLUMES.create('local_tsdf')
        self.assertIsInstance(volume, Volume)
        volume.integrate_time_s = 1.5
        volume.reset()
        self.assertEqual(volume.integrate_time_s, 0.0)


class InterfaceFakeContractTest(unittest.TestCase):
    """
    契约测试：每 ABC 一个 fake 实现，验证调用端只依赖接口表面.

    fake 经 Registry 临时注册（唯一名，进程内不冲突）→ create 注入 →
    以真实调用端的调用形态驱动，证明调用端不触达 ABC 之外的成员。
    """

    def test_cloud_builder_fake_via_registry(self):
        """Fake CloudBuilder：注册→create→build 全走 ABC 签名."""
        calls = []

        class _FakeCloudBuilder(CloudBuilder):
            def build(self, depth_mm, rgb_bgr=None, camera_K=None,
                      T_base_camera=None, stride=1, target_mask=None):
                calls.append((depth_mm.shape, stride, target_mask))
                return np.zeros((1, 3)), None, 0.5

        CLOUD_BUILDERS.register('fake_cloud_contract', _FakeCloudBuilder)
        builder = CLOUD_BUILDERS.create('fake_cloud_contract')
        self.assertIsInstance(builder, CloudBuilder)
        depth = np.zeros((4, 4), dtype=np.uint16)
        xyz, colors, ratio = builder.build(
            depth, None, {'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0},
            np.eye(4), target_mask=None)
        self.assertEqual(xyz.shape, (1, 3))
        self.assertIsNone(colors)
        self.assertEqual(ratio, 0.5)
        self.assertEqual(len(calls), 1)

    def test_refiner_fake_via_registry(self):
        """Fake Refiner：注册→create（config kwargs 透传）→refine 鸭子结果."""
        cfg = IcpConfig(min_points=7)

        class _FakeResult:
            def __init__(self):
                self.mode = 'fk'
                self.correction = np.eye(4)
                self.fitness = -1.0
                self.rmse = -1.0
                self.translation_m = 0.0
                self.rotation_deg = 0.0
                self.reason = 'fake'

            @property
            def accepted(self):
                return self.mode in ('icp', 'fk')

        class _FakeRefiner(Refiner):
            def __init__(self, config):
                self.config = config

            def refine(self, source_fk_base, target_base):
                return _FakeResult()

        REFINERS.register('fake_refiner_contract', _FakeRefiner)
        refiner = REFINERS.create('fake_refiner_contract', config=cfg)
        self.assertIsInstance(refiner, Refiner)
        self.assertEqual(refiner.config.min_points, 7)
        result = refiner.refine(np.zeros((3, 3)), np.zeros((0, 3)))
        self.assertTrue(result.accepted)

    def test_volume_fake_via_registry(self):
        """Fake Volume：注册→create→integrate/extract/reset 全走 ABC 签名."""

        class _FakeVolume(Volume):
            def __init__(self, voxel_length=0.003, sdf_trunc=0.012,
                         depth_trunc=1.5, now=None):
                self.frames = 0
                self._now = now

            def integrate_frame(self, rgb_bgr, depth_mm, camera_K,
                                T_base_camera):
                self.frames += 1

            def extract_cloud(self):
                return np.zeros((self.frames, 3)), None

            def reset(self):
                self.frames = 0

        VOLUMES.register('fake_volume_contract', _FakeVolume)
        volume = VOLUMES.create('fake_volume_contract', voxel_length=0.005)
        self.assertIsInstance(volume, Volume)
        volume.integrate_frame(np.zeros((2, 2, 3), np.uint8),
                               np.zeros((2, 2), np.uint16), {}, np.eye(4))
        xyz, colors = volume.extract_cloud()
        self.assertEqual(xyz.shape, (1, 3))
        self.assertIsNone(colors)
        volume.reset()
        self.assertEqual(volume.extract_cloud()[0].shape, (0, 3))

    def test_refitter_fake_drives_select_refitter(self):
        """Fake Refitter 两线：select_refitter 选线只依赖 kind 与 ABC."""
        from peach_reconstruction_ros2.geometry_refiner import select_refitter

        class _FakeRefitter(Refitter):
            def __init__(self, kind):
                self._kind = kind

            def refit(self, cloud_xyz, target_kind='bag', config=None,
                      axis_hint=None):
                return {'ok': True, 'kind': self._kind}

        refitters = {'cylinder': _FakeRefitter('cylinder'),
                     'sphere': _FakeRefitter('sphere')}
        # 'fruit' 选球线；其余（含未知/空）一律圆柱线（缺省袋桃）
        self.assertIs(select_refitter(refitters, 'fruit'), refitters['sphere'])
        for kind in ('bag', '', 'unknown_kind'):
            self.assertIs(select_refitter(refitters, kind),
                          refitters['cylinder'])
        result = select_refitter(refitters, 'fruit').refit(
            np.zeros((30, 3)), 'fruit')
        self.assertEqual(result['kind'], 'sphere')

    def test_mask_gate_fake_via_registry(self):
        """Fake MaskGate：注册→create（配置 kwargs 透传）→check 鸭子结果."""
        from peach_reconstruction_ros2.mask_gate import (
            GateResult,
            MaskContext,
        )

        class _FakeMaskGate(MaskGate):
            def __init__(self, require_target_mask=True, min_mask_pixels=300,
                         min_mask_depth_ratio=0.35,
                         max_target_drift_m=0.04):
                self.min_mask_pixels = min_mask_pixels

            def check(self, mask_ctx):
                return GateResult(None, 'fake_reject')

        MASK_GATES.register('fake_mask_gate_contract', _FakeMaskGate)
        gate = MASK_GATES.create('fake_mask_gate_contract',
                                 min_mask_pixels=42)
        self.assertIsInstance(gate, MaskGate)
        self.assertEqual(gate.min_mask_pixels, 42)
        ctx = MaskContext(stamp_ns=1, depth_mm=np.zeros((2, 2), np.uint16),
                          masks={}, bound_center=None)
        result = gate.check(ctx)
        self.assertFalse(result.passed)
        self.assertEqual(result.reason, 'fake_reject')


if __name__ == '__main__':
    unittest.main()
