"""算法证据门：stale TF、走廊、遮挡。空输入不得声称 clear."""
from __future__ import annotations

OCCLUSION_CLEAR = 'clear'
OCCLUSION_UNKNOWN = 'unknown'
OCCLUSION_LEAF = 'leaf_occluded'
OCCLUSION_BRANCH = 'branch_blocked'
OCCLUSION_NEIGHBOR = 'neighbor_overlap'
OCCLUSION_DAMAGED = 'damaged_or_wet'

CORRIDOR_CLEAR = 'clear'
CORRIDOR_BLOCKED = 'blocked'
CORRIDOR_UNKNOWN = 'unknown'


def may_commit_identity(tf_ok: bool) -> bool:
    """权威身份只在精确 stamp TF 上更新."""
    return bool(tf_ok)


def occlusion_class(*, inputs_available: bool, classified: str = '') -> str:
    """遮挡输入缺失 → UNKNOWN，不得默认 clear（F16）."""
    if not inputs_available:
        return OCCLUSION_UNKNOWN
    return classified or OCCLUSION_CLEAR


def corridor_status(*, points_present: bool, blocked: bool) -> str:
    """走廊与环境扫掠分项：空数据不是 clear（F11）."""
    if not points_present:
        return CORRIDOR_UNKNOWN
    return CORRIDOR_BLOCKED if blocked else CORRIDOR_CLEAR


def corridor_clear(status: str) -> bool:
    """Bool gate is true only when status is clear."""
    return status == CORRIDOR_CLEAR
