"""从观测里选下一个未入账目标."""


def next_target_id(observations, claimed, preferred=()):
    """
    先走 goal.target_ids，否则取已确认且未入账的第一项.

    observations 可为 None。未锁定集合（BeginScene 后重建窗口）不选，
    避免把上一幕的 ID 派到技能空锁定缓存上。
    """
    claimed = set(claimed)
    for tid in preferred:
        if tid and tid not in claimed:
            return str(tid)
    if observations is None:
        return ''
    if not bool(getattr(observations, 'target_set_locked', False)):
        return ''
    for item in observations.observations:
        tid = getattr(item, 'target_id', '')
        if tid and tid not in claimed and getattr(item, 'confirmed', False):
            return str(tid)
    return ''
