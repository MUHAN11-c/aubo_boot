"""
目标掩膜质量门 — 同戳/像素数/有效深度占比/漂移/邻目标间距五道门（纯核，零 ROS）.

职责:
  采帧公共门禁的掩膜段（自 reconstruction_node._target_mask_for_frame
  内联逻辑抽取，行为不变，重构阶段 A3；E2 追加第 5 道邻目标串扰门）：
  只积分全局计划所选 target_id 的精确时刻掩膜，过小目标、深度空洞、
  风吹漂移与邻近目标串扰一律拒帧。
  本模块实现 interfaces.MaskGate，注册名 'strict_mask_gate'（2.14 装配：
  节点按 yaml mask_gate.impl 经 MASK_GATES.create 注入）。

五道门（顺序即优先级，前四道语义与抽取前逐条一致）:
  1. 同戳：掩膜缓存须含与深度图 header.stamp 完全相同时间戳的条目；
  2. 像素数：掩膜像素 < min_mask_pixels 视为远距小目标或严重遮挡；
  3. 有效深度占比：掩膜内有效深度占比 < min_mask_depth_ratio（强光/
     反光空洞）；
  4. 漂移：掩膜中心相对绑定中心漂移 > max_target_drift_m（风动门）；
  5. 邻目标间距（E2）：绑定锚点与其他锁定目标锚点的最近距离 <
     min_neighbor_gap_m 时拒帧——防邻近目标点云混入形成 TSDF 不可
     回滚双层表面（I6）；TSDF 积分不可回滚，宁可停采等拉开视角也
     不放行串扰帧。绑定锚点缺失（未绑定中心）或锁定集为空/仅含
     绑定目标时本门不启用；min_neighbor_gap_m ≤ 0 整体关闭。

协议条款:
  纯核零 ROS import（test_pure_core.py AST 强制）；门禁配置由编排层
  经构造参数注入，逐帧数据走 MaskContext。

线程模型:
  实例构造后只读（配置不可变），check 无副作用，可在持锁回调内调用。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional, Tuple

import numpy as np

from peach_target_reconstruction.cloud_builder import apply_target_mask
from peach_target_reconstruction.interfaces import MASK_GATES, MaskGate


@dataclass(frozen=True)
class MaskContext:
    """
    一帧的掩膜门判据（纯数据，由节点按缓存帧组装）.

    Attributes
    ----------
        stamp_ns: 当前帧图像时间戳 [ns]（深度图 header.stamp）.
        depth_mm: (H, W) uint16 深度 [mm].
        masks: 掩膜缓存 {stamp_ns: (mask, center)}；mask 为 (H, W) uint8
            mono8，center 为 (3,) 目标中心（base 系 [m]）或 None.
        bound_center: (3,) 绑定目标中心（base 系 [m]）或 None（漂移门/
            邻目标门参照；collector.target_center）.
        neighbor_centers: 其他锁定目标锚点中心元组（(3,) base 系 [m]，
            已剔除绑定目标自身；E2 邻目标串扰门数据源，缺省空元组 =
            无邻目标，本门不启用）.

    """

    stamp_ns: int
    depth_mm: np.ndarray
    masks: Mapping[int, Tuple[np.ndarray, Optional[np.ndarray]]]
    bound_center: Optional[np.ndarray]
    neighbor_centers: Tuple[np.ndarray, ...] = field(default=())


@dataclass(frozen=True)
class GateResult:
    """
    掩膜门判定结果（纯数据）.

    Attributes
    ----------
        mask: 通过时为可用掩膜（(H, W) uint8）；门禁未启用或拒绝时 None.
        reason: 拒绝原因（中文）；空串表示通过（或门禁未启用）.

    """

    mask: Optional[np.ndarray]
    reason: str = ''

    @property
    def passed(self) -> bool:
        """Reason 为空即通过（含门禁未启用的直通情形）."""
        return not self.reason


class StrictMaskGate(MaskGate):
    """
    interfaces.MaskGate 的默认实现：五道门全过的严格掩膜门（无状态）.

    配置经构造注入（与 capture.* 参数一一对应）；require_target_mask
    为 False 时直通（返回 (None, '')，与抽取前语义一致）。
    """

    def __init__(self, require_target_mask: bool = True,
                 min_mask_pixels: int = 300,
                 min_mask_depth_ratio: float = 0.35,
                 max_target_drift_m: float = 0.04,
                 min_neighbor_gap_m: float = 0.15):
        """
        注入五道门配置（值与 capture.* 参数一致）.

        Args:
            require_target_mask: False 时掩膜门整体直通.
            min_mask_pixels: 目标掩膜最少像素数.
            min_mask_depth_ratio: 掩膜内有效深度占比下限.
            max_target_drift_m: 目标中心最大漂移 [m].
            min_neighbor_gap_m: 绑定锚点与其他锁定目标锚点的最小间距
                [m]（E2 串扰门）；≤0 关闭本门.

        Returns
        -------
            无返回值（None）.

        """
        self.require_target_mask = bool(require_target_mask)
        self.min_mask_pixels = int(min_mask_pixels)
        self.min_mask_depth_ratio = float(min_mask_depth_ratio)
        self.max_target_drift_m = float(max_target_drift_m)
        self.min_neighbor_gap_m = float(min_neighbor_gap_m)

    def check(self, mask_ctx: MaskContext) -> GateResult:
        """
        按固定顺序评估五道掩膜门（前四道与抽取前内联实现逐条对应）.

        Args:
            mask_ctx: 一帧的判据（时间戳/深度/掩膜缓存/绑定中心/邻目标
                锚点）.

        Returns
        -------
            GateResult；passed 时 mask 为可用掩膜（门禁关闭时为 None）.

        """
        if not self.require_target_mask:
            return GateResult(None, '')
        entry = mask_ctx.masks.get(mask_ctx.stamp_ns)
        if entry is None:
            return GateResult(None, '缺少所选 target_id 的同时间戳掩膜')
        mask, center = entry
        pixels = int(np.count_nonzero(mask))
        if pixels < self.min_mask_pixels:
            return GateResult(
                None,
                f'目标掩膜仅 {pixels} 像素 < {self.min_mask_pixels}')
        try:
            _masked, ratio = apply_target_mask(mask_ctx.depth_mm, mask)
        except ValueError as exc:
            return GateResult(None, str(exc))
        if ratio < self.min_mask_depth_ratio:
            return GateResult(
                None,
                f'掩膜内有效深度占比 {ratio:.2f} < '
                f'{self.min_mask_depth_ratio:.2f}')
        bound = mask_ctx.bound_center
        if center is not None and bound is not None:
            drift = float(np.linalg.norm(center - np.asarray(bound)))
            if drift > self.max_target_drift_m:
                return GateResult(
                    None,
                    f'目标漂移 {drift * 1000.0:.1f} mm > '
                    f'{self.max_target_drift_m * 1000.0:.1f} mm')
        # 门 5（E2 邻目标串扰）：绑定锚点与其他锁定目标锚点过近时拒帧。
        # 邻近目标的掩膜/点云会局部落入本目标 ROI，混入在线 TSDF 后形成
        # 不可回滚双层表面（I6）；TSDF 无单帧撤销，宁可停采等视角拉开
        if bound is not None and self.min_neighbor_gap_m > 0.0:
            bound_arr = np.asarray(bound)
            gaps = [float(np.linalg.norm(np.asarray(c) - bound_arr))
                    for c in mask_ctx.neighbor_centers]
            if gaps:
                nearest = min(gaps)
                if nearest < self.min_neighbor_gap_m:
                    return GateResult(
                        None,
                        f'邻近锁定目标锚点间距 {nearest * 1000.0:.1f} mm < '
                        f'{self.min_neighbor_gap_m * 1000.0:.1f} mm'
                        f'（防串扰拒帧，I6）')
        return GateResult(mask, '')


# 显式注册清单（2.14）：注册名 'strict_mask_gate'，yaml mask_gate.impl 默认值
MASK_GATES.register('strict_mask_gate', StrictMaskGate)
