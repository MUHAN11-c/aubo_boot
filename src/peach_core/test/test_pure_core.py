"""
纯核 import guard：peach_core/ 全树（ros/ 子包除外）零 ROS import.

豁免规则:
  - peach_core/ros/ 是唯一允许 import rclpy 的子包（适配器层）；
  - tf_transformations 是纯数学库（transformations.py 移植，无
    rclpy/消息依赖），沿用重建包纯核 guard 的放行结论；
  - cv2/numpy/yaml 等纯算法/序列化依赖放行。

协议 I3 附加护栏:
  timing.py 禁止 import time（帧率估计全部时间由调用方注入 now）。
"""
import ast
from pathlib import Path
import unittest

# ROS 面包名（出现即违规；cv2/numpy/yaml/tf_transformations 等纯库放行）
BANNED_ROOTS = {
    'rclpy', 'rosidl', 'rcl_interfaces', 'rcutils', 'rosgraph',
    'sensor_msgs', 'sensor_msgs_py', 'geometry_msgs', 'std_msgs',
    'vision_msgs', 'visualization_msgs', 'builtin_interfaces', 'action_msgs',
    'peach_pose_msgs', 'peach_harvest_msgs', 'aubo_msgs',
    'cv_bridge', 'tf2_ros', 'tf2_msgs', 'message_filters',
    'ament_index_python',
}

PKG_ROOT = Path(__file__).resolve().parents[1] / 'peach_core'
ROS_SUBPKG = PKG_ROOT / 'ros'


def _imported_roots(path: Path) -> set:
    """AST 提取一个 .py 文件的全部顶层 import 根包名."""
    roots = set()
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                roots.add(alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            # 相对导入（level>0）必为包内，跳过；level==0 取根包名
            if node.level == 0 and node.module:
                roots.add(node.module.split('.')[0])
    return roots


class PureCoreImportGuardTest(unittest.TestCase):
    """纯核层零 ROS import（层间单向依赖的硬性护栏）."""

    def test_no_ros_imports_outside_ros_subpackage(self):
        """递归扫描 peach_core/（排除 ros/），断言无 BANNED_ROOTS import."""
        files = sorted(
            f for f in PKG_ROOT.rglob('*.py')
            if ROS_SUBPKG not in f.parents)
        self.assertTrue(files, 'pure core tree is empty?')
        offenders = []
        for f in files:
            bad = _imported_roots(f) & BANNED_ROOTS
            if bad:
                offenders.append(f'{f.relative_to(PKG_ROOT)}: {sorted(bad)}')
        self.assertEqual(offenders, [])

    def test_ros_subpackage_is_the_only_exemption_zone(self):
        """ros/ 子包必须存在且仅含适配器（guard 豁免区不是笔误）."""
        self.assertTrue((ROS_SUBPKG / '__init__.py').is_file())
        self.assertTrue((ROS_SUBPKG / 'clock_adapter.py').is_file())

    def test_timing_module_does_not_import_time(self):
        """协议 I3：timing.py 禁止 import time（时间一律注入 now）."""
        roots = _imported_roots(PKG_ROOT / 'timing.py')
        self.assertNotIn('time', roots)

    def test_clock_module_does_not_import_time(self):
        """协议 I3：clock.py 抽象层同样不得自行取系统时钟."""
        roots = _imported_roots(PKG_ROOT / 'clock.py')
        self.assertNotIn('time', roots)


if __name__ == '__main__':
    unittest.main()
