"""
目标注册表 — 空间配准 + 状态机 + 防重复调度（纯逻辑，不依赖 rclpy）.

消费感知候选（位置/类别/三态），按 merge_radius 最近邻归并为持久目标档案；
状态机 NEW → OBSERVING → READY → EXECUTING → DONE/FAILED（LOST 为旁路）；
DONE/FAILED 目标只更新观测、不再参与调度。统计供 ~/registry / ~/stats 发布。
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

# 状态机取值（与 peach_pose_msgs/TargetInfo 的 STATUS_* 常量一一对应）
NEW = 0         # 新注册
OBSERVING = 1   # 观测中（未稳定）
READY = 2       # 可处理（连续 ACCEPT 且位置稳定）
EXECUTING = 3   # 正在执行接近/插入
DONE = 4        # 已处理完成（不再调度）
FAILED = 5      # 处理失败（记原因，不再调度）
LOST = 6        # 超时未再见（保留档案）

STATUS_NAMES = {
    NEW: 'NEW', OBSERVING: 'OBSERVING', READY: 'READY', EXECUTING: 'EXECUTING',
    DONE: 'DONE', FAILED: 'FAILED', LOST: 'LOST',
}

# 一条观测：(position (3,), kind, status_int[, confidence])（第 4 项可选）
Observation = Tuple[np.ndarray, str, int]


@dataclass
class TargetRecord:
    """单个目标的持久档案（base_link 系，米；时刻为秒）."""

    target_id: str            # 持久 ID（T%03d）
    position: np.ndarray      # 最新观测位置 (3,)
    kind: str                 # 'bag' | 'fruit'
    status: int = NEW         # 状态机当前状态（见模块常量）
    attempts: int = 0         # 已尝试次数
    last_result: str = ''     # 最近一次结果/失败原因
    first_seen: float = 0.0   # 首次观测时刻 (s)
    last_seen: float = 0.0    # 最近观测时刻 (s)
    accept_streak: int = 0    # 连续 ACCEPT 帧数（READY 判定用）
    confidence: float = 0.0   # 最近一次 ACCEPT 观测的置信度（调度打分用）
    frames_to_ready: int = -1  # 达到 READY 时累计吸收的观测帧数（-1=未 READY）
    perception_id: str = ''   # 最近一次观测的感知侧 target_id（调度→执行桥接用）


class TargetRegistry:
    """
    目标注册表：把逐帧候选配准成持久目标，并维护调度状态机.

    READY 判定：连续 accept_streak ≥ ready_frames 且相邻 ACCEPT 观测的位置
    漂移 ≤ ready_pos_drift；非 ACCEPT 观测清零连续计数。
    """

    def __init__(self, merge_radius: float = 0.08, ready_frames: int = 5,
                 ready_pos_drift: float = 0.02, lost_timeout_sec: float = 2.0):
        """
        构造注册表.

        Args:
            merge_radius: 空间归并半径 (m)，观测与档案距离小于该值视为同一目标.
            ready_frames: READY 所需的连续 ACCEPT 帧数.
            ready_pos_drift: READY 允许的最大相邻帧位置漂移 (m).
            lost_timeout_sec: 超过该时长 (s) 未再观测到则标 LOST.

        Returns
        -------
            无返回值（None）.

        """
        self.merge_radius = float(merge_radius)
        self.ready_frames = max(1, int(ready_frames))
        self.ready_pos_drift = float(ready_pos_drift)
        self.lost_timeout_sec = float(lost_timeout_sec)
        self._records: Dict[str, TargetRecord] = {}
        self._next_seq = 1

    def update(self, observations: List[Observation], now: float) -> List[str]:
        """
        灌入一帧观测，返回本帧触及的目标 ID 列表.

        Args:
            observations: [(position (3,) 米, kind, status_int[, confidence
                [, perception_id]]), ...]；status_int 取 BagGraspCandidate 的
                三态（0=ACCEPT）；confidence 可选（调度打分用，缺省 0.0）；
                perception_id 可选（感知侧 target_id，调度→执行桥接用）.
            now: 当前时刻 (s).

        Returns
        -------
            本帧配准/新注册的目标 ID 列表（含 DONE/FAILED 的观测更新）.

        """
        touched = []
        for obs in observations:
            position, kind, status = obs[0], obs[1], obs[2]
            confidence = float(obs[3]) if len(obs) > 3 else 0.0
            perception_id = str(obs[4]) if len(obs) > 4 else ''
            position = np.asarray(position, dtype=float)
            rec = self._match(position)
            if rec is None:
                rec = self._register(position, kind, now)
            else:
                self._absorb(rec, position, status, now, confidence)
            if perception_id:
                rec.perception_id = perception_id
            touched.append(rec.target_id)
        self._mark_lost(now, set(touched))
        return touched

    def next_target(self) -> Optional[TargetRecord]:
        """
        取下一个待调度目标（READY 中最早注册者）.

        Returns
        -------
            TargetRecord 或 None（无 READY；DONE/FAILED/EXECUTING 不在候选）.

        """
        ready = [r for r in self._records.values() if r.status == READY]
        if not ready:
            return None
        return min(ready, key=lambda r: r.first_seen)

    def mark_executing(self, target_id: str) -> None:
        """
        标记目标进入执行（attempts+1）.

        Args:
            target_id: 目标 ID.

        Returns
        -------
            无返回值（None）；未知 ID 忽略.

        """
        rec = self._records.get(target_id)
        if rec is not None:
            rec.status = EXECUTING
            rec.attempts += 1

    def mark_done(self, target_id: str, result: str = '') -> None:
        """
        标记目标处理完成（此后只更新观测，不再调度）.

        Args:
            target_id: 目标 ID.
            result: 结果描述（写入 last_result）.

        Returns
        -------
            无返回值（None）；未知 ID 忽略.

        """
        rec = self._records.get(target_id)
        if rec is not None:
            rec.status = DONE
            rec.last_result = result

    def mark_failed(self, target_id: str, result: str = '') -> None:
        """
        标记目标处理失败（记原因，此后只更新观测，不再调度）.

        Args:
            target_id: 目标 ID.
            result: 失败原因（写入 last_result）.

        Returns
        -------
            无返回值（None）；未知 ID 忽略.

        """
        rec = self._records.get(target_id)
        if rec is not None:
            rec.status = FAILED
            rec.last_result = result

    def snapshot(self) -> List[TargetRecord]:
        """
        全部档案快照（按 ID 排序）.

        Returns
        -------
            TargetRecord 列表（含 LOST/DONE/FAILED）.

        """
        return [self._records[k] for k in sorted(self._records)]

    def stats(self) -> Dict[str, int]:
        """
        按状态统计目标数.

        Returns
        -------
            {'total': N, 'NEW': n0, ..., 'LOST': n6}（缺的状态为 0）.

        """
        out = {'total': len(self._records)}
        for code, name in STATUS_NAMES.items():
            out[name] = sum(1 for r in self._records.values() if r.status == code)
        return out

    def _match(self, position: np.ndarray) -> Optional[TargetRecord]:
        """最近邻配准：距离 < merge_radius 的既有档案（无则 None）."""
        best, best_d = None, self.merge_radius
        for rec in self._records.values():
            d = float(np.linalg.norm(position - rec.position))
            if d < best_d:
                best, best_d = rec, d
        return best

    def _register(self, position: np.ndarray, kind: str, now: float) -> TargetRecord:
        """注册新目标（T%03d，初始 NEW）."""
        rec = TargetRecord(
            target_id=f'T{self._next_seq:03d}', position=position, kind=kind,
            first_seen=now, last_seen=now)
        self._next_seq += 1
        self._records[rec.target_id] = rec
        return rec

    def _absorb(self, rec: TargetRecord, position: np.ndarray,
                status: int, now: float, confidence: float = 0.0) -> None:
        """吸收一条命中观测：更新位置/时刻，推进 READY 状态机."""
        drift = float(np.linalg.norm(position - rec.position))
        rec.position = position
        rec.last_seen = now
        # DONE/FAILED/EXECUTING 只更新观测，不再改变调度状态
        if rec.status in (DONE, FAILED, EXECUTING):
            return
        if rec.status == LOST:
            rec.status = OBSERVING      # 重现目标回到观测中，重新累积
            rec.accept_streak = 0
        if rec.status == NEW:
            rec.status = OBSERVING
        if status == 0:                 # ACCEPT
            rec.accept_streak += 1
            rec.confidence = confidence
            if (rec.accept_streak >= self.ready_frames
                    and drift <= self.ready_pos_drift):
                rec.status = READY
                if rec.frames_to_ready < 0:
                    # +1 计入注册首帧（注册本身即一次观测）
                    rec.frames_to_ready = rec.accept_streak + 1
        else:
            rec.accept_streak = 0

    def _mark_lost(self, now: float, seen: set) -> None:
        """超时未再见的活动目标（NEW/OBSERVING/READY）标 LOST."""
        for rec in self._records.values():
            if (rec.target_id not in seen
                    and rec.status in (NEW, OBSERVING, READY)
                    and now - rec.last_seen > self.lost_timeout_sec):
                rec.status = LOST


def ready_order(records: List[TargetRecord], conf_weight: float = 0.6,
                near_weight: float = 0.4) -> List[TargetRecord]:
    """
    READY 目标调度排序：score = conf_weight·置信度 + near_weight·近距.

    近距 = 1/(1+距原点距离)（越近分越高）；DONE/FAILED/EXECUTING 不在候选
    （只筛 READY）。

    Args:
        records: 档案列表（内部只取 READY）.
        conf_weight: 置信度权重.
        near_weight: 近距权重.

    Returns
    -------
        按 score 降序的 READY 档案列表（可空）.

    """
    ready = [r for r in records if r.status == READY]

    def _score(rec):
        dist = float(np.linalg.norm(rec.position))
        near = 1.0 / (1.0 + dist)
        return conf_weight * rec.confidence + near_weight * near

    return sorted(ready, key=_score, reverse=True)
