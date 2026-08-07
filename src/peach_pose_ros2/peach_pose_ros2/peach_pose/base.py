"""
后端抽象接口 — typing.Protocol 结构化子类型.

InferenceEngine（detect/segment）与 RobustBagPosePipeline（estimate）等现有实现
天然满足以下接口（鸭子类型，无需显式继承、无需改动继承关系）；面向接口声明，
便于替换检测/分割/位姿后端（如换模型、换管线）而不动编排层。
"""
from typing import List, Optional, Protocol, Tuple, TYPE_CHECKING

import numpy as np

from .contracts import BagObservation

if TYPE_CHECKING:
    from .pipeline import TargetPoseResult


class Detector(Protocol):
    """检测后端接口：BGR 图 → 检测 dict 列表（InferenceEngine.detect 满足）."""

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """
        对整图跑目标检测.

        Args:
            rgb: (H, W, 3) BGR 图（OpenCV 惯例）.

        Returns
        -------
            [{"class_id", "class_name", "bbox": (x1,y1,x2,y2), "conf"}, ...]，
            按置信度降序.

        """
        ...


class Segmenter(Protocol):
    """分割后端接口：BGR 图 + 框 prompt → 掩膜列表（InferenceEngine.segment 满足）."""

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        以检测框为 box prompt 做实例分割.

        Args:
            rgb: (H, W, 3) BGR 图.
            bboxes: [(x1, y1, x2, y2), ...]（像素）.

        Returns
        -------
            [(bool 掩膜 (H,W), bbox), ...]（过小掩膜由后端丢弃）.

        """
        ...


class PosePipeline(Protocol):
    """位姿管线接口：单目标观测 → 显式三态结果（RobustBagPosePipeline 满足）."""

    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> 'TargetPoseResult':
        """
        估计 bbox 内单个目标，返回显式安全状态（ACCEPT/REOBSERVE/REJECT）.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            mask: 外部前景掩膜；None 走深度降级.
            mask_source: 掩膜来源标签，写入诊断.

        Returns
        -------
            TargetPoseResult（status 三态 + metrics 诊断）.

        """
        ...
