"""
纯核 import guard：纯核模块零 ROS import（设计文档 §3 层间依赖规则）.

guard 清单（数据层/算法层/接口层，均不得 import ROS）：
captured_frame / capture_gate / frame_collector / cloud_builder /
mask_gate / tsdf_volume / geometry_refiner / overlap / interfaces /
session_io / view_coverage / timing / bind_holdoff / icp_target_cache /
publish_throttle。

刻意排除（编排/参数层，ROS 耦合是本职，注释固化）：
  - reconstruction_node.py：编排主节点（rclpy/sensor_msgs 等）；
  - publishers.py：发布面 mixin（geometry_msgs/sensor_msgs/sensor_msgs_py/
    std_msgs/peach_pose_msgs 消息类型，A14 自节点拆出）；
  - auto_controller.py：自动状态机驱动 mixin（编排层；仅 import 纯核
    模块，无 ROS import，不列入反向守门）；
  - params.py：参数层（rcl_interfaces.ParameterDescriptor）；
  - visualization.py：Marker 构造直接 import geometry_msgs/std_msgs/
    visualization_msgs 消息类型，属编排层可视化胶水；
  - status_messages.py：诊断/许可 dict → 类型化消息组装（peach_harvest_msgs/
    geometry_msgs/std_msgs 消息类型），供节点与契约测试共用。

放行说明：tf_transformations 是纯数学库（transformations.py 移植，无
rclpy/消息依赖），peach_core.tf_utils 使用不算 ROS import；
open3d/cv2/scipy/yaml/peach_pose_ros2.peach_pose.fitting（纯 numpy）
同理放行。peach_core 为独立纯核包（其自身 test_pure_core 强制零
ROS import），本包纯核模块 import peach_core.* 不视为 ROS 耦合。
"""
from pathlib import Path
import re
import unittest

_PURE_CORE = (
    'captured_frame',
    'capture_gate',
    'frame_collector',
    'bind_holdoff',
    'cloud_builder',
    'mask_gate',
    'tsdf_volume',
    'geometry_refiner',
    'overlap',
    'interfaces',
    'session_io',
    'view_coverage',
    'timing',
    'icp_target_cache',
    'publish_throttle',
)

# 行首 import 形式匹配 ROS 包（docstring 里的 'geometry_msgs/Transform'
# 之类非 import 文本不匹配）
_BANNED = re.compile(
    r'^\s*(?:from|import)\s+'
    r'(rclpy|rcl_interfaces|sensor_msgs|sensor_msgs_py|geometry_msgs|'
    r'std_msgs|visualization_msgs|peach_pose_msgs|aubo_msgs|cv_bridge|'
    r'tf2_ros|message_filters|ament_index_python)\b')

_PKG_DIR = Path(__file__).resolve().parents[1] / 'peach_reconstruction_ros2'


class PureCoreImportGuardTest(unittest.TestCase):
    def test_guard_modules_have_no_ros_import(self):
        """清单模块逐行扫描：无 rclpy/消息/TF 等 ROS import 行."""
        for module in _PURE_CORE:
            path = _PKG_DIR / f'{module}.py'
            self.assertTrue(path.is_file(), f'模块文件缺失: {path}')
            for lineno, line in enumerate(
                    path.read_text(encoding='utf-8').splitlines(), 1):
                m = _BANNED.match(line)
                self.assertIsNone(
                    m, f'{module}.py:{lineno} 引入 ROS 依赖: {line.strip()}')

    def test_orchestration_files_are_ros_coupled_by_design(self):
        """反向守门：编排/参数层确实 ROS 耦合（清单划分不是笔误）."""
        for name in ('reconstruction_node', 'publishers', 'params',
                     'visualization', 'status_messages'):
            path = _PKG_DIR / f'{name}.py'
            hits = [line for line in
                    path.read_text(encoding='utf-8').splitlines()
                    if _BANNED.match(line)]
            self.assertTrue(hits, f'{name}.py 应有 ROS import（编排/参数层）')


if __name__ == '__main__':
    unittest.main()
