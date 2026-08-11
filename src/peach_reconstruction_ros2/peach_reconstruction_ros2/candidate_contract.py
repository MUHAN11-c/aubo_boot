"""感知→重建边界的纯数据契约（零 ROS import）."""
from __future__ import annotations

import numpy as np


_STATUS_ACCEPT = 0
_STATUS_REJECT = 2
_BLOCKING_CANDIDATE_FLAGS = frozenset({
    'tf_stale', 'tf_unavailable', 'target_untracked',
})


def select_reconstruction_candidate(
        msg, base_frame: str, preferred_target_id: str = ''):
    """选择可安全用于重建的候选，并返回 (target_id, center_base)."""
    if msg is None or not msg.candidates:
        return '', None
    if msg.header.frame_id != base_frame:
        return '', None
    eligible = []
    for cand in msg.candidates:
        candidate_frame = cand.header.frame_id or msg.header.frame_id
        flags = set(cand.diagnostic_flags)
        if candidate_frame != base_frame:
            continue
        if flags & _BLOCKING_CANDIDATE_FLAGS:
            continue
        eligible.append(cand)
    if preferred_target_id:
        eligible.sort(
            key=lambda cand: cand.target_id != preferred_target_id)
    best = next(
        (cand for cand in eligible if cand.status == _STATUS_ACCEPT), None)
    if best is None:
        best = next(
            (cand for cand in eligible if cand.status != _STATUS_REJECT), None)
    if best is None:
        return '', None
    bottom = np.array([best.bag_bottom.x, best.bag_bottom.y,
                       best.bag_bottom.z], dtype=np.float64)
    neck = np.array([best.bag_neck.x, best.bag_neck.y,
                     best.bag_neck.z], dtype=np.float64)
    if not np.all(np.isfinite(bottom)) or not np.all(np.isfinite(neck)):
        return '', None
    center = 0.5 * (bottom + neck)
    if not np.any(center):
        center = bottom if np.any(bottom) else neck
    if not np.any(center):
        center = None
    return best.target_id, center


class TargetKindMemory:
    """保存最新类别映射，并在目标离场后保持本轮绑定类别."""

    def __init__(self):
        """初始化为空映射和未绑定状态."""
        self.latest = {}
        self.bound_target_id = ''
        self.bound_kind = ''

    def update(self, fittings) -> None:
        """用当前感知帧更新类别，并刷新已绑定目标的类别."""
        self.latest = {
            fitting.target_id: fitting.target_kind
            for fitting in fittings if fitting.target_id
        }
        if self.bound_target_id in self.latest:
            self.bound_kind = self.latest[self.bound_target_id]

    def bind(self, target_id: str) -> None:
        """开始一轮重建时绑定目标及其当前类别."""
        self.bound_target_id = target_id or ''
        self.bound_kind = self.latest.get(self.bound_target_id, '')

    def reset(self) -> None:
        """结束当前绑定；最新感知映射保留给下一轮启动."""
        self.bound_target_id = ''
        self.bound_kind = ''

    def resolve(self):
        """返回规范化类别及是否发生缺省."""
        kind = self.bound_kind or self.latest.get(self.bound_target_id, '')
        if not kind:
            return 'bag', True
        return ('fruit' if kind == 'fruit' else 'bag'), False
