"""
接口层 — 感知算法抽象基类（abc.ABC）与实现注册表.

对标 nav2_core 的纯接口包形态（设计文档
docs/superpowers/specs/2026-08-10-peach-layered-architecture.md §2.2）：

- ``Detector``：workhorse ``detect(rgb) -> list[dict]``（YOLO 检测）；
- ``Segmenter``：workhorse ``segment(rgb, bboxes) -> [(mask, bbox), ...]``
  （SAM 分割；签名为批量形，节点整帧一次调用后按 bbox 取回各目标掩膜）；
- ``PoseEstimator``：workhorse ``estimate(obs, target_id, bbox, mask,
  mask_source) -> TargetPoseResult``（袋线/果线位姿 + 安全门控）。

实现声明继承：``InferenceEngine(Detector, Segmenter)``、
``RobustBagPosePipeline(PoseEstimator)``（RobustFruitPosePipeline 经袋线
传递继承）。签名本就齐备，继承仅声明契约，不改任何行为。

``POSE_ESTIMATORS`` 注册表（yolo_ros type_to_model 先例）：字典本体在本
模块，**由 pipeline.py 在类定义后显式登记**（import 本模块不拉入实现，
避免 interfaces ↔ pipeline 循环 import）；candidates.py 的路由与
test_interfaces.py 都以本表为准。本模块属纯核：零 ROS import
（test_pure_core.py 强制）。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

import numpy as np

from .contracts import BagObservation

if TYPE_CHECKING:
    # 仅类型标注用，运行期不 import（避免 interfaces ↔ pipeline 循环导入）
    from .pipeline import TargetPoseResult

# 位姿估计器注册表：kind → 实现类；由 pipeline.py 底部显式登记
# （{'bag': RobustBagPosePipeline, 'fruit': RobustFruitPosePipeline}）
POSE_ESTIMATORS: Dict[str, type] = {}


class Detector(ABC):
    """目标检测器接口：RGB 进 → 检测 dict 列表出（无副作用的纯推理面）."""

    @abstractmethod
    def detect(self, rgb: np.ndarray) -> List[dict]:
        """
        对 (H, W, 3) BGR 图跑检测，返回 [{'bbox','class_id','conf',...}].

        Args:
            rgb: (H, W, 3) BGR 图像.

        Returns
        -------
            检测 dict 列表（字段约定见 InferenceEngine.detect docstring）.

        """


class Segmenter(ABC):
    """实例分割器接口：RGB + prompt 框 → 二值掩膜列表（批量签名）."""

    @abstractmethod
    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        对 (H, W, 3) BGR 图按 prompt 框分割，返回 [(binary_mask, bbox)].

        Args:
            rgb: (H, W, 3) BGR 图像.
            bboxes: [(x1, y1, x2, y2), ...] prompt 框（像素）.

        Returns
        -------
            [(掩膜, 框)] 列表；掩膜按 bbox 与 prompt 框对应（过小掩膜被
            丢弃，返回项与 prompt 非一一对齐）.

        """


class PoseEstimator(ABC):
    """单目标位姿估计器接口：观测 + 前景掩膜 → 显式安全状态的位姿结果."""

    @abstractmethod
    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> TargetPoseResult:
        """
        估计 bbox 内单个目标的抓取位姿与三态安全门控.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            mask: 外部前景掩膜或 None.
            mask_source: 掩膜来源标签（写入结果追溯）.

        Returns
        -------
            TargetPoseResult（grasp_2d/grasp_3d + status ∈ 三态）.

        """
