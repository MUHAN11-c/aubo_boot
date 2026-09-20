"""W2/V1 回归：MobileSam.segment 不得吞异常（上层逐目标回退依赖抛出）.

inference 模块经 CandidateEstimator 导入 pose_pipelines（其顶层 import
geometry_msgs——已知 V5 纯核违规，W3 拆分时消除）；零 ROS 门环境按缺依赖
跳过，colcon test（ROS 环境）真跑。
"""
from __future__ import annotations

import threading

import numpy as np
import pytest

try:
    from peach_harvester.vision.scene_perception.inference import MobileSam
except ImportError:  # 零 ROS 门环境：inference→pose_pipelines→geometry_msgs
    MobileSam = None


class _Boom:
    def __call__(self, *args, **kwargs):
        raise RuntimeError('sam-backend-down')


def _sam_with_broken_backend():
    # object.__new__ 绕开构造（_resolve_device 依赖 torch），只填 segment
    # 用到的属性——纯核测试不拉推理栈。
    sam = object.__new__(MobileSam)
    sam._sam = _Boom()
    sam._device = 'cpu'
    sam._lock = threading.Lock()
    sam._sam_max_bboxes = 8
    sam._sam_min_area = 1
    return sam


def test_segment_propagates_backend_exception():
    if MobileSam is None:
        pytest.skip('geometry_msgs unavailable（inference 导入链需 ROS 环境）')
    sam = _sam_with_broken_backend()
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    with pytest.raises(RuntimeError):
        sam.segment(rgb, [(1, 1, 6, 6)])
