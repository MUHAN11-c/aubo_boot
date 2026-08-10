"""接口层：四个实现是 ABC 实例 + 注册表登记 + ABC 抽象强制."""
import importlib.util
import unittest

from peach_reconstruction_ros2.cloud_builder import (
    CLOUD_BUILDERS,
    Open3dCloudBuilder,
)
from peach_reconstruction_ros2.frame_collector import (
    FRAME_STORES,
    FrameCollector,
)
from peach_reconstruction_ros2.geometry_refiner import (
    GEOMETRY_REFINERS,
    RansacGeometryRefiner,
)
from peach_reconstruction_ros2.interfaces import (
    CloudBuilder,
    FrameStore,
    GeometryRefiner,
    VolumeFusion,
)
from peach_reconstruction_ros2.tsdf_volume import (
    LocalTsdf,
    VOLUME_FUSIONS,
)

_HAS_O3D = importlib.util.find_spec('open3d') is not None


class InterfaceContractTest(unittest.TestCase):
    def test_implementations_are_abc_instances(self):
        """可实例化的实现均为对应 ABC 实例（FrameStore/CloudBuilder/Refiner）."""
        self.assertIsInstance(FrameCollector(), FrameStore)
        self.assertIsInstance(Open3dCloudBuilder(), CloudBuilder)
        self.assertIsInstance(RansacGeometryRefiner(), GeometryRefiner)
        # LocalTsdf 构造依赖 open3d，类级契约用 issubclass 验证
        self.assertTrue(issubclass(LocalTsdf, VolumeFusion))

    def test_registries_contain_implementations(self):
        """注册表字典按名登记实现类（显式发现，yolo_ros 先例）."""
        self.assertIs(FRAME_STORES['default'], FrameCollector)
        self.assertIs(CLOUD_BUILDERS['open3d'], Open3dCloudBuilder)
        self.assertIs(VOLUME_FUSIONS['open3d_scalable'], LocalTsdf)
        self.assertIs(GEOMETRY_REFINERS['ransac'], RansacGeometryRefiner)

    def test_abc_enforces_abstract_methods(self):
        """ABC 抽象强制：缺实现的子类不可实例化."""
        class _Bad(CloudBuilder):
            pass
        with self.assertRaises(TypeError):
            _Bad()

    @unittest.skipUnless(_HAS_O3D, 'open3d 不可用（系统 python3 跳过）')
    def test_volume_fusion_instance_and_reset(self):
        """体积融合实现是 VolumeFusion 实例；reset 清空体积与耗时（契约钩子）."""
        volume = LocalTsdf()
        self.assertIsInstance(volume, VolumeFusion)
        volume.integrate_time_s = 1.5
        volume.reset()
        self.assertEqual(volume.integrate_time_s, 0.0)


if __name__ == '__main__':
    unittest.main()
