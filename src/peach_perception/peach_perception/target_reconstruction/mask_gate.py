from __future__ import annotations
"""掩膜五道质量门（MaskGate 判定本体）。"""

from dataclasses import dataclass, field
from typing import Mapping, Optional, Tuple

import numpy as np
from peach_perception.target_reconstruction.cloud_builder import apply_target_mask


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
    # 检测框面积（像素²，>0 有效）：绑定目标与各邻居并行携带。串扰门按
    # 面积比豁免「远小于绑定目标」的邻居框（叶片遮挡残片/误检，09-01）。
    bound_area: float = 0.0
    neighbor_areas: Tuple[float, ...] = field(default=())


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


class StrictMaskGate:
    """
    五道门全过的严格掩膜门（唯一实现，无状态）.

    配置经构造注入（与 capture.* 参数一一对应）；require_target_mask
    为 False 时直通（返回 (None, '')，与抽取前语义一致）。
    """

    def __init__(self, require_target_mask: bool = True,
                 min_mask_pixels: int = 300,
                 min_mask_depth_ratio: float = 0.35,
                 max_target_drift_m: float = 0.04,
                 min_neighbor_gap_m: float = 0.15,
                 neighbor_gap_area_ratio: float = 2.0):
        """
        注入六道门配置（值与 capture.* 参数一致）.

        Args:
            require_target_mask: False 时掩膜门整体直通.
            min_mask_pixels: 目标掩膜最少像素数.
            min_mask_depth_ratio: 掩膜内有效深度占比下限.
            max_target_drift_m: 目标中心最大漂移 [m].
            min_neighbor_gap_m: 绑定锚点与其他锁定目标锚点的最小间距
                [m]（E2 串扰门）；≤0 关闭本门.
            neighbor_gap_area_ratio: 串扰门小框豁免比（≤0 关闭豁免）：
                邻居检测框面积 < 绑定框面积/本值时视为遮挡残片/误检，
                不计入串扰间距（近距双检常见同一颗袋的大框+残片小框，
                不豁免则两颗互相锁死，09-01 现场 58.5 mm 即此类）.

        Returns
        -------
            无返回值（None）.

        """
        self.require_target_mask = bool(require_target_mask)
        self.min_mask_pixels = int(min_mask_pixels)
        self.min_mask_depth_ratio = float(min_mask_depth_ratio)
        self.max_target_drift_m = float(max_target_drift_m)
        self.min_neighbor_gap_m = float(min_neighbor_gap_m)
        self.neighbor_gap_area_ratio = float(neighbor_gap_area_ratio)

    def check(self, mask_ctx: MaskContext) -> GateResult:
        """
        按固定顺序评估五道掩膜门（前四道与抽取前内联实现逐条对应）.

        Args:
            mask_ctx: 一帧的判据（时间戳/深度/掩膜缓存/绑定中心/邻目标
                锚点）.

        Returns
        -------
            GateResult；reason 为空即通过时 mask 为可用掩膜（门禁关闭时为 None）.

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
        # 不可回滚双层表面（I6）；TSDF 无单帧撤销，宁可停采等视角拉开。
        # 小框豁免：面积远小于绑定框的邻居（叶片遮挡残片/误检）不计入
        # 间距——近距双检常见同一颗袋的大框+残片小框，不豁免则互相锁死。
        if bound is not None and self.min_neighbor_gap_m > 0.0:
            bound_arr = np.asarray(bound)
            areas = mask_ctx.neighbor_areas
            effective = []
            for idx, c in enumerate(mask_ctx.neighbor_centers):
                neighbor_area = (
                    float(areas[idx]) if idx < len(areas) else 0.0)
                if (self.neighbor_gap_area_ratio > 0.0
                        and mask_ctx.bound_area > 0.0
                        and neighbor_area > 0.0
                        and neighbor_area * self.neighbor_gap_area_ratio
                        < mask_ctx.bound_area):
                    continue
                effective.append(c)
            gaps = [float(np.linalg.norm(np.asarray(c) - bound_arr))
                    for c in effective]
            if gaps:
                nearest = min(gaps)
                if nearest < self.min_neighbor_gap_m:
                    return GateResult(
                        None,
                        f'邻近锁定目标锚点间距 {nearest * 1000.0:.1f} mm < '
                        f'{self.min_neighbor_gap_m * 1000.0:.1f} mm'
                        f'（防串扰拒帧，I6）')
        return GateResult(mask, '')
