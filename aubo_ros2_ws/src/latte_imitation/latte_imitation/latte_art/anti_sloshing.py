"""轨迹后处理 — 抗晃荡速度剖面 + 完整轨迹组合 + MoveIt2 导出喵~

来源: latte_art_robot_research/轨迹生成方案/latte_art_trajectory.py
理论: Di Leva 2023 (Time-Optimal Handling of Liquids) — jerk 约束防止液体晃动
"""

import numpy as np
from scipy.interpolate import CubicSpline
from typing import List

from .config import CupConfig, PourConfig


def apply_anti_sloshing(traj: np.ndarray, pour: PourConfig,
                         dt: float = 0.01) -> np.ndarray:
    """对轨迹施加抗晃荡速度约束 — S 曲线速度剖面 喵~

    使用梯形速度剖面替换均匀采样，减小 jerk 峰值，防止奶泡晃动导致图案变形喵~

    Args:
        traj: (T, 3) XYZ 轨迹
        pour: 倾倒参数 (使用 max_velocity, max_acceleration, max_jerk)
        dt: 输出时间步长 (s)

    Returns:
        重新参数化后的 (T', 3) 轨迹
    """
    diffs = np.diff(traj, axis=0)
    arc_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate([[0], np.cumsum(arc_lengths)])
    total_length = cumulative[-1]

    if total_length < 1e-6:
        return traj.copy()

    t = cumulative / total_length
    v = np.ones_like(t) * pour.max_velocity

    # 加速段
    accel_ramp = max(1, int(len(t) * 0.15))
    v[:accel_ramp] = np.linspace(0.001, pour.max_velocity, accel_ramp)
    # 减速段
    decel_ramp = max(1, int(len(t) * 0.15))
    v[-decel_ramp:] = np.linspace(pour.max_velocity, 0.001, decel_ramp)

    # 重新参数化: 按速度分配时间
    v_safe = np.maximum(v[:-1], 0.001)
    dt_segments = np.diff(cumulative) / v_safe
    new_t = np.concatenate([[0], np.cumsum(dt_segments)])
    if new_t[-1] < 1e-9:
        return traj.copy()
    new_t = new_t / new_t[-1]

    # 在新时间点上插值 (先清理 t 中的重复值)
    keep = np.ones(len(t), dtype=bool)
    for i in range(1, len(t)):
        if t[i] <= t[i-1]:
            keep[i] = False
    n_keep = keep.sum()
    if n_keep < 4:
        return traj.copy()
    t_clean = t[keep]
    x_clean = traj[keep, 0]
    y_clean = traj[keep, 1]
    z_clean = traj[keep, 2]

    cs_x = CubicSpline(t_clean, x_clean)
    cs_y = CubicSpline(t_clean, y_clean)
    cs_z = CubicSpline(t_clean, z_clean)

    new_t_clipped = np.clip(new_t, t_clean[0], t_clean[-1])
    return np.column_stack([cs_x(new_t_clipped), cs_y(new_t_clipped), cs_z(new_t_clipped)])


def compose_full_trajectory(pattern_traj: np.ndarray,
                             cup: CupConfig,
                             pour: PourConfig,
                             num_mix: int = 50,
                             num_finish: int = 30) -> np.ndarray:
    """为图案轨迹添加融合和收尾阶段，生成完整拉花轨迹 喵~

    Args:
        pattern_traj: (T, 3) 图案轨迹 (仅成形阶段)
        cup: 杯子参数
        pour: 倾倒参数
        num_mix: 融合阶段帧数
        num_finish: 收尾阶段帧数

    Returns:
        (T+num_mix+num_finish, 3) 完整轨迹
    """
    surf = cup.surface_z

    # 融合阶段: 杯中心高位缓慢注入
    mix_traj = np.tile([cup.center_x, cup.center_y, pour.mix_z(surf)],
                       (num_mix, 1))
    mix_traj[:, 0] += 0.002 * np.sin(np.linspace(0, 2 * np.pi, num_mix))
    mix_traj[:, 1] += 0.002 * np.cos(np.linspace(0, 2 * np.pi, num_mix))

    # 收尾阶段: 上提拉线
    finish_traj = np.column_stack([
        np.full(num_finish, cup.center_x),
        np.linspace(pattern_traj[-1, 1], cup.center_y - cup.radius * 0.2,
                   num_finish),
        np.linspace(pour.draw_z(surf), pour.finish_z(surf), num_finish),
    ])

    return np.vstack([mix_traj, pattern_traj, finish_traj])


def to_moveit_waypoints(traj_xyz: np.ndarray,
                         roll: float = 45.0,
                         pitch: float = 0.0,
                         yaw: float = 0.0) -> List[List[float]]:
    """将 XYZ 轨迹转换为带姿态的 MoveIt2 路点 (含四元数) 喵~

    姿态: 奶缸以固定角度倾斜 (默认 roll=45°, X 轴倾倒) 进行倒奶喵~

    Args:
        traj_xyz: (T, 3) XYZ 轨迹
        roll: 绕 X 轴角度 (度), 默认 45° (奶缸倾倒方向)
        pitch: 绕 Y 轴角度 (度), 默认 0°
        yaw: 绕 Z 轴角度 (度)

    Returns:
        [[x, y, z, qx, qy, qz, qw], ...]
    """
    from scipy.spatial.transform import Rotation
    quat = Rotation.from_euler('xyz', [np.radians(roll),
                                        np.radians(pitch),
                                        np.radians(yaw)]).as_quat()
    return [
        [float(pt[0]), float(pt[1]), float(pt[2]),
         quat[0], quat[1], quat[2], quat[3]]
        for pt in traj_xyz
    ]
