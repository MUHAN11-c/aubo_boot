#!/usr/bin/env python3
"""5 点力 CoP 仿真的公共常量与几何工具。

坐标与点位唯一来源：serial_force/doc/force_pad_positions.md
（tcp 系，mm；膜片平面 tcp z = -72.29 mm，法线 +Z_tcp）。
"""
import math

import numpy as np

# 通道 → tcp 系坐标 (x, y)，mm；顺序固定 1/2/3/5/7
PADS = {
    1: (15.75, 4.30),
    2: (9.45, -13.30),
    3: (-9.65, -13.15),
    5: (0.0, 16.45),
    7: (-15.70, 4.55),
}
CH = [1, 2, 3, 5, 7]
JN = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
      'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
HOME = [0.425083, 0.195177, 1.677740, 1.461739, -0.500161, 0.038621]

Z0 = -0.07229          # 膜片平面在 tcp 系的 z，m
CENTER_MM = 1.2        # CoP 居中门限
MAX_ROT_DEG = 5.0      # 单次试验累计旋转上限


def cop(forces):
    """5 点力 → (CoP[x, y] mm, 总力 N)。零力返回 (0,0)。"""
    tot = float(sum(forces))
    if tot <= 1e-9:
        return np.zeros(2), 0.0
    px = sum(forces[i] * PADS[c][0] for i, c in enumerate(CH)) / tot
    py = sum(forces[i] * PADS[c][1] for i, c in enumerate(CH)) / tot
    return np.array([px, py]), tot


def q2R(q):
    """geometry_msgs Quaternion → 3x3 旋转矩阵。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def aaR(axis, angle):
    """轴角 → 旋转矩阵（右乘到当前姿态用）。"""
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a) + 1e-12
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def R2q(R):
    """旋转矩阵 → (x, y, z, w)。"""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w
