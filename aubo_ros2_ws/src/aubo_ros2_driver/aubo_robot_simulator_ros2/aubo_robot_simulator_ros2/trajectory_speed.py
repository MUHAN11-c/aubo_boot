#!/usr/bin/env python3
"""
与 ROS1 aubo_controller/script/aubo_controller/trajectory_speed 逻辑一致：
- 速度缩放轨迹（scale_trajectory_speed）
- 按最小点间隔抽稀轨迹（resample_trajectory_min_interval），缓解笛卡尔路径密集点导致的节流卡顿
"""

from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _duration_to_sec(d: DurationMsg) -> float:
    return float(d.sec) + 1e-9 * float(d.nanosec)


def _sec_to_duration(sec: float) -> DurationMsg:
    out = DurationMsg()
    out.sec = int(sec)
    out.nanosec = int(round((sec - int(sec)) * 1e9))
    return out


def scale_trajectory_speed(traj: JointTrajectory, scale: float) -> JointTrajectory:
    """与 ROS1 scale_trajectory_speed 一致：时间除以 scale，速度乘 scale，加速度乘 scale²。"""
    if scale <= 0:
        return traj
    new_traj = JointTrajectory()
    new_traj.joint_names = list(traj.joint_names)
    n_joints = len(traj.joint_names)
    points = []
    for i in range(len(traj.points)):
        pt = traj.points[i]
        point = JointTrajectoryPoint()
        point.positions = list(pt.positions)
        total_sec = _duration_to_sec(pt.time_from_start)
        new_sec = total_sec / scale
        point.time_from_start = _sec_to_duration(new_sec)
        v_in = (list(pt.velocities) + [0.0] * n_joints)[:n_joints]
        a_in = (list(pt.accelerations) + [0.0] * n_joints)[:n_joints]
        point.velocities = [v_in[j] * scale for j in range(n_joints)]
        point.accelerations = [a_in[j] * scale * scale for j in range(n_joints)]
        points.append(point)
    new_traj.points = points
    return new_traj


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
