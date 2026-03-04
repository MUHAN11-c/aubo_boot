#!/usr/bin/env python3
"""
按最小点间隔抽稀轨迹，缓解笛卡尔路径密集点导致的节流卡顿。
关节空间点间隔通常已满足，基本不受影响。
"""

from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory


def _duration_to_sec(d: DurationMsg) -> float:
    return float(d.sec) + 1e-9 * float(d.nanosec)


def resample_trajectory_min_interval(traj: JointTrajectory, min_interval_sec: float) -> JointTrajectory:
    """
    与 ROS1 一致：按最小点间隔抽稀轨迹。
    关节空间通常点间隔已 >= min_interval，几乎不受影响；笛卡尔点密则会被抽稀，减少 segment 密度、缓解卡顿。
    min_interval_sec <= 0 时不抽稀。
    """
    if min_interval_sec <= 0 or len(traj.points) < 3:
        return traj
    kept = [traj.points[0]]
    last_t = _duration_to_sec(traj.points[0].time_from_start)
    for i in range(1, len(traj.points) - 1):
        t = _duration_to_sec(traj.points[i].time_from_start)
        if t - last_t >= min_interval_sec:
            kept.append(traj.points[i])
            last_t = t
    kept.append(traj.points[-1])
    out = JointTrajectory()
    out.joint_names = list(traj.joint_names)
    out.points = kept
    return out
