from __future__ import annotations
"""采帧 skip 短码：中文门禁原因 → 稳定码。"""


# (子串, 短码) 顺序即优先级
_SKIP_PATTERNS = (
    ('缺少所选 target_id 的同时间戳掩膜', 'missing_mask'),
    ('目标掩膜仅', 'mask_pixels'),
    ('掩膜内有效深度占比', 'mask_depth_ratio'),
    ('目标漂移', 'target_drift'),
    ('邻近锁定目标锚点间距', 'neighbor_gap'),
    ('缓存帧龄期', 'stale_frame'),
    ('缓存帧未更新', 'same_stamp'),
    ('查询失败（已计 tf_failures）', 'tf_failure'),
    ('近重复视角', 'near_duplicate'),
    ('尚无同步 RGB-D', 'no_frame'),
    ('已达 max_views', 'max_views'),
    ('机器人未静止', 'robot_not_static'),
    ('未收到 /joint_states', 'robot_not_static'),
    ('frame_id 为空', 'empty_frame_id'),
    ('缓存帧已更新', 'frame_changed'),
    ('配准拒帧', 'icp_reject'),
    ('间隔门', 'min_interval'),
    ('非精确时间 TF', 'tf_inexact'),
    ('TSDF 在线积分失败', 'tsdf_integrate'),
)


def classify_skip_reason(reason: str) -> str:
    """中文门禁原因 → 稳定短码；空串给 empty."""
    text = str(reason or '')
    if not text:
        return 'empty'
    for needle, code in _SKIP_PATTERNS:
        if needle in text:
            return code
    return 'other'
