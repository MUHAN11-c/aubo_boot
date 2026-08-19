"""
接口层 — 感知算法抽象基类（abc.ABC）与实现注册表（2.14 装配规则）.

对标 nav2_core 的纯接口包形态（设计文档
docs/superpowers/specs/2026-08-10-peach-layered-architecture.md §2.2），
A3 起全部六类可替换组件在此正式化：

- ``Detector``：``detect(rgb) -> list[dict]``（YOLO 检测）；
- ``Segmenter``：``segment(rgb, bboxes) -> [(mask, bbox), ...]``
  （SAM 分割；签名为批量形，节点整帧一次调用后按 bbox 取回各目标掩膜）；
- ``PosePipeline``：``estimate(obs, target_id, bbox, mask, mask_source)
  -> TargetPoseResult`` + 类属性 ``kind``（袋线/果线位姿 + 安全门控）；
- ``TargetMatcher``：``match(anchor, class_id, table, frame_used)
  -> MatchResult``（世界系身份匹配 + 恢复段；表由调用方 TargetRegistry
  持有，匹配器不持表）；
- ``LockPolicy``：``update(records, now) -> LockEvent | None``
  （收齐窗口关闭判定；锁定后的集合记账由调用方 GlobalHarvestPlan 持有）。

装配规则（2.14）：每类 ABC 配一个 ``peach_common_py.registry.Registry``，
实现类在 ``impls.py`` 显式注册清单按名登记（暂不用自注册装饰器，集中
一处便于审阅装配面）；yaml 以 ``*.impl`` 参数按名选择实现，节点构造期
``Registry.create`` 注入，调用端只持有 ABC 引用。

本模块属纯核：零 ROS import（test_pure_core.py AST 强制）；契约类型
（MatchResult / LockEvent）为不可变值对象，不依赖任何实现模块，避免
interfaces ↔ 实现 循环 import。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

import numpy as np
from peach_common_py.registry import Registry

from .contracts import BagObservation

if TYPE_CHECKING:
    # 仅类型标注用，运行期不 import（避免 interfaces ↔ pipeline 循环导入）
    from .pipeline import TargetPoseResult

# 实现注册表（2.14）：按名登记/创建，默认实现在 impls.py 显式注册
DETECTORS: Registry['Detector'] = Registry('检测器')
SEGMENTERS: Registry['Segmenter'] = Registry('分割器')
POSE_PIPELINES: Registry['PosePipeline'] = Registry('位姿管线')
MATCHERS: Registry['TargetMatcher'] = Registry('目标匹配器')
LOCK_POLICIES: Registry['LockPolicy'] = Registry('锁定策略')


@dataclass(frozen=True)
class MatchResult:
    """
    TargetMatcher 匹配结果（不可变值对象）.

    target_id 为命中的历史表项 ID；None 表示未命中（调用方发新 ID）。
    distance 为命中距离（米），未命中时为查询半径（诊断用）。
    """

    target_id: Optional[str]
    distance: float
    status: str = 'ok'


@dataclass(frozen=True)
class LockEvent:
    """
    LockPolicy 窗口关闭事件（不可变值对象）.

    records 为收齐窗口关闭时累积的确认记录快照（每 target_id 取窗口内
    最新一帧，顺序为首次入集序）；排序/截断/锁定记账由调用方
    （GlobalHarvestPlan）完成，策略本身不持有锁定后状态。
    """

    records: Tuple[dict, ...]


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
            检测 dict 列表（字段约定见 UltralyticsYolo.detect docstring）.

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


class PosePipeline(ABC):
    """
    单目标位姿管线接口：观测 + 前景掩膜 → 显式安全状态的位姿结果.

    类属性 ``kind`` 为管线种类键（'bag' / 'fruit'），candidates 类别路由
    与 strategy_id 拼写以它为准。实例属性契约（调用端 candidates 构造
    前景掩膜时读取）：``tool``（ToolGeometry，版本追溯与净空门控）、
    ``min_depth_m`` / ``max_depth_m``（有效深度区间，掩膜构造共用）。
    """

    kind: str = ''
    tool: object = None
    min_depth_m: float = 0.3
    max_depth_m: float = 2.5

    @abstractmethod
    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> 'TargetPoseResult':
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


class TargetMatcher(ABC):
    """
    世界系目标匹配器接口：候选锚点 + 目标表 → 命中结果（不持表）.

    表（table）由调用方 TargetRegistry 持有，匹配器只读：键为
    target_id，值为表项 dict（至少含 'class_id'、'position' (3,)、
    'confirmed' 三个键，见 target_registry.py 表项模式）。
    """

    @abstractmethod
    def match(self, anchor: np.ndarray, class_id: int,
              table: Dict[str, dict], frame_used: set) -> MatchResult:
        """
        在表中为一个世界系候选找匹配表项（含恢复段策略）.

        Args:
            anchor: (3,) 世界系位置（米），候选目标的空间锚点.
            class_id: 候选类别.
            table: target_id → 表项 dict（只读）.
            frame_used: 本帧已命中的 target_id 集合（同帧去重，跳过）.

        Returns
        -------
            MatchResult：target_id 为 None 表示未命中（调用方发新 ID）.

        """


class LockPolicy(ABC):
    """
    收齐窗口锁定策略接口：逐帧记录进 → 窗口关闭事件出（不持锁定后状态）.

    实例属性契约（调用端节点帧率自适应时读写）：
    ``min_collect_frames`` / ``lock_settle_frames``（静止判定参数，只读
    使用）、``max_collect_s``（超时上限，节点按实测帧率 EMA 自适应改写）、
    ``accumulated_count``（收齐窗口累积的已确认目标数，只读；缺陷 R-D8
    发现进度摘要用，窗口未累积任何确认目标时为 0）。
    """

    min_collect_frames: int = 10
    lock_settle_frames: int = 5
    max_collect_s: float = 25.0
    accumulated_count: int = 0

    @abstractmethod
    def update(self, records: List[dict], now: float) -> Optional[LockEvent]:
        """
        输入当前帧记录；窗口关闭时返回 LockEvent，否则 None.

        Args:
            records: 本帧候选 record dict 列表（须带 target_id；含
                confirmed/status/距离等键）.
            now: 当前时刻 (s)，由调用方注入（协议 I3：实现内禁止自行取
                时钟）.

        Returns
        -------
            LockEvent（窗口关闭，含累积确认记录快照）或 None.

        """

    @abstractmethod
    def reset(self) -> None:
        """清空窗口状态，允许下一轮全局观测重新收齐."""
