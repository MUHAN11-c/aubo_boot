"""
重建节点帧缓存 mixin：同步帧环与同戳掩膜缓存（宿主契约见下，不自带 __init__）.

宿主（TargetReconstructionNode）须持有：
`_frame_ring` / `_frame_ring_max` / `_latest_frame`（同步帧环）、
`_target_masks`（同戳掩膜缓存）、`_locked_target_centers` /
`_locked_target_areas` / `_preferred_target_id`（邻目标串扰门输入）、
`_mask_gate`（capture.MaskGate 实例）、`collector`（帧采集器）。
严格同戳语义：重建只用精确时间戳 TF/掩膜，不回退 latest。
"""
from __future__ import annotations

import numpy as np

from peach_perception.target_reconstruction.mask_gate import MaskContext


class FrameStoreMixin:
    """帧环与掩膜缓存方法集（宿主契约见模块 docstring；不自带 __init__）."""

    def _stamp_ns(self, stamp_msg) -> int:
        """ROS Time → 纳秒整数（与掩膜缓存键一致）."""
        return int(stamp_msg.sec) * 1000000000 + int(stamp_msg.nanosec)

    def _clear_frame_ring(self) -> None:
        """换绑/reset 时丢弃未对齐的陈帧，避免旧目标点云混入."""
        self._frame_ring = {}
        self._latest_frame = None

    def _push_frame_ring(self, frame_tuple) -> None:
        """
        按 stamp_ns 写入帧环，超出容量丢最旧.

        须持宿主 ``_state_lock``：读者（``_select_cached_frame``，持锁）
        对环做 ``list(...)`` 快照迭代，无锁并发插入会触发
        ``RuntimeError: dictionary changed size during iteration``。
        """
        stamp_msg = frame_tuple[3]
        stamp_ns = self._stamp_ns(stamp_msg)
        with self._state_lock:
            self._frame_ring.pop(stamp_ns, None)
            self._frame_ring[stamp_ns] = frame_tuple
            while len(self._frame_ring) > self._frame_ring_max:
                self._frame_ring.pop(next(iter(self._frame_ring)))
        self._latest_frame = frame_tuple

    def _select_cached_frame(
            self, prefer_stamp_sec=None, prefer_cam_frame=None):
        """
        取采帧缓存：优先指定 stamp；否则取「有同戳掩膜的最新帧」.

        严格同戳语义不变：不回退 latest TF。掩膜尚未到达的最新帧留在环
        内，等感知回调再驱动。环空返回 None。
        """
        if prefer_stamp_sec is not None:
            # 持锁遍历：_push_frame_ring 与本读同锁（写侧注释见上）
            for frame in list(self._frame_ring.values()):
                if abs(float(frame[4]) - float(prefer_stamp_sec)) > 1e-9:
                    continue
                if (prefer_cam_frame is not None
                        and (frame[5] or '') != prefer_cam_frame):
                    continue
                return frame
            return None
        for stamp_ns in reversed(list(self._frame_ring.keys())):
            if stamp_ns in self._target_masks:
                return self._frame_ring[stamp_ns]
        if self._frame_ring:
            return next(reversed(list(self._frame_ring.values())))
        return self._latest_frame

    @staticmethod
    def _candidate_center(candidate):
        """候选几何袋底/袋颈中点（base 系 [m]）；非有限或全零时返回 None."""
        bottom = np.array([
            candidate.bag_bottom.x,
            candidate.bag_bottom.y,
            candidate.bag_bottom.z], dtype=np.float64)
        neck = np.array([
            candidate.bag_neck.x,
            candidate.bag_neck.y,
            candidate.bag_neck.z], dtype=np.float64)
        center = 0.5 * (bottom + neck)
        if not np.all(np.isfinite(center)) or not np.any(center):
            return None
        return center

    def _target_mask_for_frame(self, stamp_msg, depth_mm):
        """取严格同时间戳掩膜并过五道质量门（判定本体在 MaskGate 实现）."""
        stamp_ns = self._stamp_ns(stamp_msg)
        # 邻目标锚点=锁定集锚点缓存剔除绑定目标自身（E2 串扰门输入）；
        # 框面积并行携带，供串扰门按面积比豁免小框邻居
        neighbors = tuple(
            (c, self._locked_target_areas.get(tid, 0.0))
            for tid, c in self._locked_target_centers.items()
            if tid != self._preferred_target_id)
        centers = tuple(c for c, _ in neighbors)
        areas = tuple(a for _, a in neighbors)
        result = self._mask_gate.check(MaskContext(
            stamp_ns=stamp_ns,
            depth_mm=depth_mm,
            masks=self._target_masks,
            bound_center=self.collector.target_center,
            neighbor_centers=centers,
            bound_area=self._locked_target_areas.get(
                self._preferred_target_id, 0.0),
            neighbor_areas=areas))
        return result.mask, result.reason
