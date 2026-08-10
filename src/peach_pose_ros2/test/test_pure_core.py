"""纯核 import guard：peach_pose/ 全树（含 offline/ 与 interfaces.py）零 ROS import."""
import ast
from pathlib import Path
import unittest

# ROS 面包名（出现即违规；cv2/numpy/torch/open3d 等纯算法依赖允许）
BANNED_ROOTS = {
    'rclpy', 'rcl_interfaces', 'sensor_msgs', 'sensor_msgs_py', 'geometry_msgs',
    'std_msgs', 'vision_msgs', 'visualization_msgs', 'peach_pose_msgs',
    'cv_bridge', 'tf2_ros', 'tf_transformations', 'message_filters',
    'ament_index_python',
}

PURE_CORE_ROOT = (
    Path(__file__).resolve().parents[1] / 'peach_pose_ros2' / 'peach_pose')


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
    """数据层/接口层/算法层零 ROS import（层间单向依赖的硬性护栏）."""

    def test_no_ros_imports_in_pure_core(self):
        """递归扫描 peach_pose/ 全部 .py，断言无 BANNED_ROOTS 任何 import."""
        files = sorted(PURE_CORE_ROOT.rglob('*.py'))
        self.assertTrue(files, 'pure core tree is empty?')
        offenders = []
        for f in files:
            bad = _imported_roots(f) & BANNED_ROOTS
            if bad:
                offenders.append(f'{f.relative_to(PURE_CORE_ROOT)}: {sorted(bad)}')
        self.assertEqual(offenders, [])

    def test_interfaces_module_itself_clean(self):
        """接口层 interfaces.py 单独断言（契约文件必须最干净）."""
        f = PURE_CORE_ROOT / 'interfaces.py'
        self.assertTrue(f.is_file())
        self.assertEqual(_imported_roots(f) & BANNED_ROOTS, set())


if __name__ == '__main__':
    unittest.main()
