"""臂是否停稳。停稳才把最新画面送进检测。"""
from __future__ import annotations


def arm_is_settled(velocities, limit: float = 0.08) -> bool:
    """
    关节速度都低于 limit（rad/s）视为停稳.

    没有速度数组时不当成在动，避免关节话题只有位置时画面一直空白。
    """
    if velocities is None:
        return True
    return all(abs(float(v)) <= limit for v in velocities)
