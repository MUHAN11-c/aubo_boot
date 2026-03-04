# Core: point, direction, batch, frame, line, handedness (no ROS)，使用 NumPy 做矩阵/向量运算
import numpy as np
from typing import Tuple
from enum import Enum


class Handedness(Enum):
    Right = 0
    Left = 1


def _ensure_3(p) -> np.ndarray:
    return np.asarray(p, dtype=np.float64).reshape(3)


def _ensure_4x4(T) -> np.ndarray:
    return np.asarray(T, dtype=np.float64).reshape(4, 4)


def _ensure_3x3(M) -> np.ndarray:
    return np.asarray(M, dtype=np.float64).reshape(3, 3)


# 手系变换：右手系↔左手系。右手系 X×Y=Z，左手系 X×Y=-Z。公式: 反射矩阵 M = diag(1,1,-1)，[x,y,z]' → [x,y,-z]
def handedness_transform(from_h: Handedness, to_h: Handedness) -> np.ndarray:
    if from_h == to_h:
        return np.eye(3)
    return np.diag([1.0, 1.0, -1.0])

def right_to_left() -> np.ndarray:
    return handedness_transform(Handedness.Right, Handedness.Left)

def left_to_right() -> np.ndarray:
    return handedness_transform(Handedness.Left, Handedness.Right)

# 刚体变换单点: p' = T*[p;1] 取前三维。公式: p' = R*p + t，T = [R|t; 0 0 0 1] 行优先
def transform_point(p: np.ndarray, T: np.ndarray) -> np.ndarray:
    p = _ensure_3(p)
    T = _ensure_4x4(T)
    ph = np.append(p, 1.0)
    out = T @ ph
    return out[:3]


# 方向向量变换: v' = R*v（仅旋转）；可选单位化 v' := v'/|v'|
def transform_direction(v: np.ndarray, R: np.ndarray, normalize: bool = True) -> np.ndarray:
    v = _ensure_3(v)
    R = _ensure_3x3(R)
    out = R @ v
    if normalize:
        n = np.linalg.norm(out)
        if n > 1e-10:
            out = out / n
    return out

# 多点变换: 对每点 p_i 做 p'_i = T*[p_i;1] 取前三维。即 P' = (T * P_h^T)^T 取前三列，P_h = [P, 1]
def transform_points(P: np.ndarray, T: np.ndarray) -> np.ndarray:
    P = np.asarray(P, dtype=np.float64)
    if P.ndim == 1:
        P = P.reshape(1, -1)
    T = _ensure_4x4(T)
    N = P.shape[0]
    ones = np.ones((N, 1))
    P_h = np.hstack([P[:, :3], ones])
    return (T @ P_h.T).T[:, :3]


# 位姿变换: origin' = T_B_A*origin；new_R = R_B_A @ R。公式: T_new = T_B_A * T_old（架 T_old = [R|origin; 0 0 0 1]）
def transform_pose(
    origin: np.ndarray, R: np.ndarray, T_B_A: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    origin = _ensure_3(origin)
    R = _ensure_3x3(R)
    T_B_A = _ensure_4x4(T_B_A)
    new_origin = transform_point(origin, T_B_A)
    R_B_A = T_B_A[:3, :3]
    new_R = R_B_A @ R
    return new_origin, new_R

# 完整架变换: T_new = T_B_A @ T_old（4×4 矩阵乘），同一刚体变换作用到架的原点与姿态
def transform_frame(T_old: np.ndarray, T_B_A: np.ndarray) -> np.ndarray:
    T_old = _ensure_4x4(T_old)
    T_B_A = _ensure_4x4(T_B_A)
    return T_B_A @ T_old

def transform_directions(V: np.ndarray, R: np.ndarray, normalize: bool = True) -> np.ndarray:
    V = np.asarray(V, dtype=np.float64)
    R = _ensure_3x3(R)
    if V.ndim == 1:
        V = V.reshape(1, -1)
    return (R @ V.T).T if not normalize else np.array([transform_direction(V[i], R, True) for i in range(V.shape[0])])

# 直线（一点+方向）变换: point' = T*point；direction' = R*direction 并单位化（方向仅旋转，不平移）
def transform_line(
    point: np.ndarray, direction: np.ndarray, T: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    point = _ensure_3(point)
    direction = _ensure_3(direction)
    T = _ensure_4x4(T)
    point_out = transform_point(point, T)
    direction_out = transform_direction(direction, T[:3, :3], True)
    return point_out, direction_out
