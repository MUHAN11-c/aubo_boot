"""全局采摘目标计划：固定数量、稳定 ID、优先级与选中目标（零 ROS import）."""
from __future__ import annotations

import math


_BLOCKING_FLAGS = frozenset({
    'tf_stale', 'tf_unavailable', 'target_untracked',
})
_STATUS_REJECT = 2


def _rank_key(record):
    """按安全状态、相机距离、置信度和 ID 生成确定性排序键."""
    status = int(record.get('status', _STATUS_REJECT))
    distance = float(record.get('camera_distance_m', math.inf))
    confidence = float(record.get('confidence', 0.0))
    if not math.isfinite(distance) or distance <= 0.0:
        distance = math.inf
    if not math.isfinite(confidence):
        confidence = 0.0
    return status, distance, -confidence, str(record.get('target_id', ''))


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
    """首轮稳定观测锁定数量与优先级，仅在当前完成后推进安全目标."""

    def __init__(self, max_targets: int = 20):
        """创建未锁定计划."""
        if max_targets < 1:
            raise ValueError('max_targets 须 ≥ 1')
        self.max_targets = int(max_targets)
        self.snapshot_id = 0
        self.locked_ids = ()
        self.priorities = {}
        self.selected_target_id = ''
        self.completed_ids = set()
        self.current_selectable_ids = set()

    @property
    def locked(self) -> bool:
        """是否已锁定全局目标集合."""
        return bool(self.locked_ids)

    @property
    def target_count(self) -> int:
        """锁定目标数量."""
        return len(self.locked_ids)

    def update(self, records):
        """输入当前帧记录；首次出现可抓取确认目标时锁定整批确认目标."""
        current = {
            str(record.get('target_id')): record
            for record in records
            if record.get('target_id')
        }
        self.current_selectable_ids = {
            target_id for target_id, record in current.items()
            if _selectable(record)
        }
        if not self.locked:
            confirmed = [
                record for record in current.values()
                if bool(record.get('confirmed'))
            ]
            selectable = [record for record in confirmed if _selectable(record)]
            if selectable:
                ordered = sorted(confirmed, key=_rank_key)[:self.max_targets]
                self.locked_ids = tuple(
                    str(record['target_id']) for record in ordered)
                self.priorities = {
                    target_id: index + 1
                    for index, target_id in enumerate(self.locked_ids)
                }
                selectable_ids = {
                    str(record['target_id']) for record in selectable}
                self.selected_target_id = next(
                    target_id for target_id in self.locked_ids
                    if target_id in selectable_ids)
                self.snapshot_id += 1
        elif not self.selected_target_id:
            self.selected_target_id = next(
                (target_id for target_id in self.locked_ids
                 if target_id not in self.completed_ids
                 and target_id in self.current_selectable_ids), '')
        return current

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
        """清空目标集合，允许下一轮全局拍照重新确定数量."""
        self.locked_ids = ()
        self.priorities = {}
        self.selected_target_id = ''
        self.completed_ids.clear()
        self.current_selectable_ids.clear()
