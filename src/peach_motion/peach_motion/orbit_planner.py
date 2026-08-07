"""
轨道规划器 — 球面弧线预接近路径点生成（纯 numpy）.

起点=当前 TCP，终点=目标前方 approach_standoff 的预接近点（目标 −axis 侧，
即 entry 一侧）；中间点分布在以目标为球心的球面弧线上（侧向绕行、逐步
逼近，半径从起点距离单调收敛到 standoff）；每个 waypoint 的姿态让 TCP
Z 轴（AUBO E5 的 wrist3_Link 法兰方向）始终指向目标（朝向 = 目标 − 路径点）。

arc_deg 为总弧角上限：起点→终点方向的自然夹角 φ ≤ arc_deg 时走完整大圆
弧（两端点精确）；φ > arc_deg 时只走末段 arc_deg（终点恒精确，起点为
弧线上被截断的可行点，调用方负责评估首段可达性）。
"""
from typing import List, Tuple

import numpy as np


def _quat_look_at(direction, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """
    Z 轴朝 direction 的单位四元数 (x,y,z,w)（up 参考定横滚）.

    Args:
        direction: (3,) 目标朝向（无需单位化；近零向量给单位四元数）.
        up: (3,) 世界上参考（默认 +Z）.

    Returns
    -------
        (4,) 四元数 (x, y, z, w)；direction 退化时给 (0,0,0,1).

    """
    z = np.asarray(direction, dtype=float)
    if float(np.linalg.norm(z)) < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0])
    z = z / np.linalg.norm(z)
    x = np.cross(up, z)
    if float(np.linalg.norm(x)) < 1e-6:      # 与 up 共线：换参考轴
        x = np.cross(np.array([1.0, 0.0, 0.0]), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack((x, y, z))           # 列 = X/Y/Z 轴（右手系）
    return _rotation_to_quat(R)


def _rotation_to_quat(R: np.ndarray) -> np.ndarray:
    """
    3×3 旋转矩阵 → 单位四元数 (x,y,z,w)（Shepperd 法，纯 numpy 自含）.

    按迹/最大对角元分四支取值，避免除小量的数值退化（与
    peach_pose_ros2.conversions 的手写版同构；此处自含以保持纯 numpy）。

    Args:
        R: (3, 3) 旋转矩阵.

    Returns
    -------
        (4,) 四元数 (x, y, z, w).

    """
    import math
    m = np.asarray(R, dtype=float)
    t = float(np.trace(m))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw, qx = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        qy, qz = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw, qx = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        qy, qz = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw, qx = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        qy, qz = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw, qx = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        qy, qz = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return np.array([qx, qy, qz, qw])


def make_orbit_waypoints(current_tcp, target, axis, approach_standoff: float,
                         arc_deg: float, n_points: int
                         ) -> List[Tuple[np.ndarray, np.ndarray]]:
    """
    生成球面弧线预接近路径点.

    Args:
        current_tcp: (3,) 当前 TCP 位置（输出系，米）——起点.
        target: (3,) 目标参考点（袋底/球心，输出系，米）——球心.
        axis: (3,) 套入轴（底→颈；预接近点 = target − standoff·axis）.
        approach_standoff: 预接近距离 (m)，终点距目标的半径.
        arc_deg: 总弧角上限 (deg)（自然夹角更小时按自然夹角）.
        n_points: 路径点数（≥2，含起点与终点）.

    Returns
    -------
        [(position (3,), quat_xyzw (4,)), ...]，长度 n_points；
        起点=当前 TCP、终点=预接近点（截断情形见模块 docstring）；
        非法输入（零轴/起点即球心/n_points<2）给空列表.

    """
    if n_points < 2:
        return []
    p0 = np.asarray(current_tcp, dtype=float)
    t = np.asarray(target, dtype=float)
    a = np.asarray(axis, dtype=float)
    norm_a = float(np.linalg.norm(a))
    r0 = float(np.linalg.norm(p0 - t))
    if norm_a < 1e-9 or r0 < 1e-6:
        return []
    u_end = -a / norm_a                    # 预接近方向（entry 侧）
    u0 = (p0 - t) / r0
    # 自然夹角 φ（u0 → u_end），实际扫掠角 capped 到 arc_deg
    cos = float(np.clip(u0 @ u_end, -1.0, 1.0))
    phi = float(np.arccos(cos))
    sweep = min(np.radians(float(arc_deg)), phi)
    n = np.cross(u0, u_end)
    if float(np.linalg.norm(n)) < 1e-6:    # 近共线：任取 ⊥ 轴（sweep≈0 走直线逼近）
        ref = np.array([1.0, 0.0, 0.0]) if abs(u0[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
        n = np.cross(u0, ref)
    n = n / np.linalg.norm(n)
    waypoints = []
    for i in range(n_points):
        s = i / (n_points - 1)
        # 从 (φ−sweep) 转到 φ：sweep=φ 时起点恰为当前 TCP；截断时落在弧线中段
        ang = (phi - sweep) + s * sweep
        # Rodrigues 旋转：u0 绕 n 转 ang
        u = (u0 * np.cos(ang) + np.cross(n, u0) * np.sin(ang)
             + n * float(n @ u0) * (1.0 - np.cos(ang)))
        r = (1.0 - s) * r0 + s * approach_standoff   # 半径单调逼近 standoff
        p = t + r * u
        q = _quat_look_at(t - p)             # TCP Z 轴指向目标
        waypoints.append((p, q))
    return waypoints
