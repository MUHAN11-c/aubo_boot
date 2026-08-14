"""
全局采摘目标计划：收齐式窗口锁定、稳定 ID、多维优先级与选中目标（零 ROS import）.

收齐式锁定语义：reset（或构造）后进入收齐窗口，逐帧 ``update()`` 把确认目标
并入累积集；窗口关闭（帧数与静止条件满足，或超时兜底）时对累积集一次性
排序、截断并锁定。锁定后目标集合与优先级冻结，新出现的 ID 不再入集，
仅在当前目标完成后按固定优先级推进。

线程安全约定：本类自身无锁；写路径（update/complete_selected/reset）与
读路径（各公共属性）由编排层（peach_pose_node）的同一把锁统一保护，
保证单写者语义（选型理由见 peach_pose_node 中 ``_plan_lock`` 注释）。
"""
from __future__ import annotations

import math
import time
from typing import Dict, Optional


_BLOCKING_FLAGS = frozenset({
    'tf_stale', 'tf_unavailable', 'target_untracked',
})
_STATUS_REJECT = 2


def _finite_or(value: float, fallback: float) -> float:
    """非有限浮点（nan/inf）回退为 fallback，供排序键防御脏数据."""
    return value if math.isfinite(value) else fallback


def _selectable(record) -> bool:
    """目标是否可作为靠近重建对象."""
    flags = set(record.get('diagnostic_flags', ()))
    return (
        bool(record.get('target_id'))
        and bool(record.get('confirmed'))
        and int(record.get('status', _STATUS_REJECT)) != _STATUS_REJECT
        and not flags & _BLOCKING_FLAGS
    )


class GlobalHarvestPlan:
    """
    收齐窗口关闭后一次性锁定全局目标集合，锁定后按固定优先级推进.

    构造参数：max_targets 为锁定目标数量上限（≥1），超出按排序键截断；
    min_collect_frames 为收齐窗口最少累积帧数（≥1），达到后才允许按静止
    条件关闭窗口；lock_settle_frames 为连续无新增确认 ID 的帧数（≥0），
    与最少帧数联合判定目标集合已稳定；max_collect_s 为窗口最长时长
    (s，>0)，超时强制关闭兜底（空集也锁定），时钟取 time.monotonic()
    或 update 注入的 now；prefer_lower_first 决定排序是否启用高度键
    （先低后高），False 时高度不参与排序。

    """

    def __init__(self, max_targets: int = 20, min_collect_frames: int = 10,
                 lock_settle_frames: int = 5, max_collect_s: float = 25.0,
                 prefer_lower_first: bool = True):
        """建未锁定计划并进入收齐窗口（容量≥1、帧数≥0、时长>0 校验）."""
        if max_targets < 1:
            raise ValueError('max_targets 须 ≥ 1')
        if min_collect_frames < 1:
            raise ValueError('min_collect_frames 须 ≥ 1')
        if lock_settle_frames < 0:
            raise ValueError('lock_settle_frames 须 ≥ 0')
        if max_collect_s <= 0.0:
            raise ValueError('max_collect_s 须 > 0')
        self.max_targets = int(max_targets)
        self.min_collect_frames = int(min_collect_frames)
        self.lock_settle_frames = int(lock_settle_frames)
        self.max_collect_s = float(max_collect_s)
        self.prefer_lower_first = bool(prefer_lower_first)
        self.snapshot_id = 0
        self.locked_ids = ()
        self.priorities = {}
        self.selected_target_id = ''
        self.completed_ids = set()
        self.current_selectable_ids = set()
        self._locked = False
        self._accumulated: Dict[str, dict] = {}
        self._collect_frames = 0        # 窗口内已累积帧数（update 调用次数）
        self._last_new_id_frame = 0     # 最近一次出现新增确认 ID 的帧序号
        self._window_start: Optional[float] = None  # 首帧时间戳（懒启动）

    @property
    def locked(self) -> bool:
        """是否已锁定全局目标集合（与 locked_ids 解耦：空集也算锁定）."""
        return self._locked

    @property
    def target_count(self) -> int:
        """锁定目标数量."""
        return len(self.locked_ids)

    def _rank_key(self, record):
        """
        生成多维确定性排序键（升序排最优先）.

        键序设计依据：
          1. status 升序——先 ACCEPT(0) 后 REOBSERVE(1)，REJECT(2) 垫底；
          2. camera_distance_m——先近后远：先清外围目标，减少深入冠层时
             碰枝与自遮挡；
          3. base_height_m（仅 prefer_lower_first）——先低后高：避免摘高处
             目标时碰落低处果实（Xiong 草莓采摘机实证顺序）；
          4. -confidence——同距同高时置信度高者优先；
          5. target_id——字符串确定性 tie-break，保证跨帧排序稳定。
        record 缺键时按最差值兜底（距离/高度 inf、置信度 0、REJECT）。
        """
        status = int(record.get('status', _STATUS_REJECT))
        distance = _finite_or(
            float(record.get('camera_distance_m', math.inf)), math.inf)
        if distance <= 0.0:
            distance = math.inf
        confidence = _finite_or(float(record.get('confidence', 0.0)), 0.0)
        key = [status, distance]
        if self.prefer_lower_first:
            key.append(_finite_or(
                float(record.get('base_height_m', math.inf)), math.inf))
        key.extend([-confidence, str(record.get('target_id', ''))])
        return tuple(key)

    def update(self, records, now: Optional[float] = None):
        """
        输入当前帧记录；未锁定期间累积确认目标，窗口关闭时一次性锁定.

        Args:
            records: 本帧候选 record dict 列表（含 confirmed/status/距离等）.
            now: 时间戳 (s)，None 用 time.monotonic()；测试注入值须与窗口
                内各帧同一时钟基准（与 TargetRegistry 的 now 注入同约定）.

        Returns
        -------
            dict：本帧 target_id → record（仅含带 ID 的记录）.

        """
        current = {
            str(record.get('target_id')): record
            for record in records
            if record.get('target_id')
        }
        self.current_selectable_ids = {
            target_id for target_id, record in current.items()
            if _selectable(record)
        }
        if not self._locked:
            ts = time.monotonic() if now is None else float(now)
            if self._window_start is None:
                self._window_start = ts
            self._collect_frames += 1
            # 确认目标（含 REOBSERVE，不限于 selectable）并入累积集；
            # 同 ID 后者覆盖，窗口关闭时取最新一帧的质量量
            for record in current.values():
                if not record.get('confirmed'):
                    continue
                target_id = str(record.get('target_id'))
                if target_id not in self._accumulated:
                    self._last_new_id_frame = self._collect_frames
                self._accumulated[target_id] = record
            # 静止关闭追加前提"无进行中确认"：当前帧仍有未确认记录（确认
            # 进度攒帧中）时不得关窗，否则低置信/闪烁场景会在确认完成前
            # 锁定空集（2026-08-14 真机：0.3 阈值下检测迟到，窗口提前锁定空集）。
            has_pending_confirmation = any(
                not record.get('confirmed') for record in current.values())
            settled = (
                self._collect_frames >= self.min_collect_frames
                and self._collect_frames - self._last_new_id_frame
                >= self.lock_settle_frames
                and not has_pending_confirmation)
            timed_out = ts - self._window_start >= self.max_collect_s
            if settled or timed_out:
                self._lock_now()
        elif not self.selected_target_id:
            # 锁定后 selected 为空（完成全部或质量未恢复）时，按固定优先级
            # 重选当前帧可选择者
            self.selected_target_id = next(
                (target_id for target_id in self.locked_ids
                 if target_id not in self.completed_ids
                 and target_id in self.current_selectable_ids), '')
        return current

    def _lock_now(self) -> None:
        """窗口关闭：累积集排序截断，一次性写入锁定状态（允许空集锁定）."""
        ordered = sorted(
            self._accumulated.values(), key=self._rank_key)[:self.max_targets]
        ids = []
        seen = set()
        for record in ordered:
            target_id = str(record.get('target_id', ''))
            # 去重兜底：累积集 dict 键本已唯一，此处防御外部构造的脏 record
            if target_id and target_id not in seen:
                seen.add(target_id)
                ids.append(target_id)
        self.locked_ids = tuple(ids)
        self.priorities = {
            target_id: index + 1
            for index, target_id in enumerate(self.locked_ids)
        }
        selectable_ids = {
            str(record.get('target_id'))
            for record in self._accumulated.values()
            if _selectable(record)
        }
        self.selected_target_id = next(
            (target_id for target_id in self.locked_ids
             if target_id in selectable_ids), '')
        self._locked = True
        self.snapshot_id += 1

    def complete_selected(self) -> str:
        """标记当前目标已完成，并按固定优先级推进到下一个未完成 ID."""
        if self.selected_target_id:
            self.completed_ids.add(self.selected_target_id)
        self.selected_target_id = next(
            (target_id for target_id in self.locked_ids
             if target_id not in self.completed_ids
             and target_id in self.current_selectable_ids), '')
        return self.selected_target_id

    def harvest_status(self, target_id: str) -> str:
        """返回目标在本轮固定计划中的采摘状态."""
        if target_id in self.completed_ids:
            return 'HARVESTED'
        if target_id == self.selected_target_id:
            return 'SELECTED'
        if target_id not in self.current_selectable_ids:
            return 'WAITING_QUALITY'
        return 'PLANNED'

    def priority(self, target_id: str) -> int:
        """返回固定优先级；未锁定 ID 返回 0."""
        return int(self.priorities.get(target_id, 0))

    def reset(self) -> None:
        """清空目标集合与收齐窗口状态，允许下一轮全局观测重新锁定."""
        self.locked_ids = ()
        self.priorities = {}
        self.selected_target_id = ''
        self.completed_ids.clear()
        self.current_selectable_ids.clear()
        self._locked = False
        self._accumulated.clear()
        self._collect_frames = 0
        self._last_new_id_frame = 0
        self._window_start = None
