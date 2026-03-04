# Core: 2D/3D 投影、反投影、重投影误差（无 ROS），使用 NumPy 计算
import numpy as np
from dataclasses import dataclass
from typing import Union

INVALID_PIXEL = -1.0


# 相机内参 K = [fx 0 cx; 0 fy cy; 0 0 1]，针孔模型
@dataclass
class CameraIntrinsics:
    fx: float = 1.0
    fy: float = 1.0
    cx: float = 0.0
    cy: float = 0.0


def _to_array3(p) -> np.ndarray:
    return np.asarray(p, dtype=np.float64).reshape(3)


def _to_array2(p) -> np.ndarray:
    return np.asarray(p, dtype=np.float64).reshape(2)


# 针孔投影：相机系 3D (X,Y,Z) → 像素 (u,v)。公式: u = fx*X/Z + cx, v = fy*Y/Z + cy；Z≤0 视为无效
def project_3d_to_2d(point_3d: Union[tuple, list, np.ndarray], K: CameraIntrinsics) -> np.ndarray:
    p = _to_array3(point_3d)
    Z = p[2]
    if Z <= 0:
        return np.array([INVALID_PIXEL, INVALID_PIXEL])
    u = K.fx * p[0] / Z + K.cx
    v = K.fy * p[1] / Z + K.cy
    return np.array([u, v])


def project_3d_to_2d_batch(P_3d: np.ndarray, K: CameraIntrinsics) -> np.ndarray:
    P = np.asarray(P_3d, dtype=np.float64)
    if P.ndim == 1:
        P = P.reshape(1, -1)
    N = P.shape[0]
    uv = np.empty((N, 2))
    for i in range(N):
        uv[i] = project_3d_to_2d(P[i], K)
    return uv


# 反投影：像素 (u,v) + 深度 Z → 相机系 3D。公式: X = (u-cx)*Z/fx, Y = (v-cy)*Z/fy, Z = depth（与投影逆推）
def unproject_2d_to_3d(u: float, v: float, depth: float, K: CameraIntrinsics) -> np.ndarray:
    Z = depth
    X = (u - K.cx) * Z / K.fx
    Y = (v - K.cy) * Z / K.fy
    return np.array([X, Y, Z])


def unproject_2d_to_3d_batch(
    uv: np.ndarray, depths: np.ndarray, K: CameraIntrinsics
) -> np.ndarray:
    uv = np.asarray(uv, dtype=np.float64)
    depths = np.asarray(depths, dtype=np.float64).flatten()
    if uv.shape[0] != len(depths):
        raise ValueError("uv and depths size mismatch")
    N = uv.shape[0]
    out = np.empty((N, 3))
    for i in range(N):
        out[i] = unproject_2d_to_3d(uv[i, 0], uv[i, 1], depths[i], K)
    return out


# 重投影误差：观测 (u_obs,v_obs) 与 3D 投影 (u_proj,v_proj) 的像素距离。公式: e² = (u_obs-u_proj)²+(v_obs-v_proj)²；squared 时返回 e²
def reprojection_error(
    point_2d_obs: Union[tuple, list, np.ndarray],
    point_3d: Union[tuple, list, np.ndarray],
    K: CameraIntrinsics,
    squared: bool = False,
) -> float:
    uv_obs = _to_array2(point_2d_obs)
    proj = project_3d_to_2d(point_3d, K)
    if proj[0] == INVALID_PIXEL:
        return np.nan
    du = uv_obs[0] - proj[0]
    dv = uv_obs[1] - proj[1]
    e2 = du * du + dv * dv
    return e2 if squared else np.sqrt(e2)


def reprojection_error_batch(
    uv_obs: np.ndarray, P_3d: np.ndarray, K: CameraIntrinsics, squared: bool = False
) -> np.ndarray:
    uv_obs = np.asarray(uv_obs, dtype=np.float64)
    P_3d = np.asarray(P_3d, dtype=np.float64)
    if uv_obs.shape[0] != P_3d.shape[0]:
        raise ValueError("uv_obs and P_3d size mismatch")
    N = uv_obs.shape[0]
    out = np.empty(N)
    for i in range(N):
        out[i] = reprojection_error(uv_obs[i], P_3d[i], K, squared)
    return out


# 重投影误差 RMS。公式: RMS = sqrt((1/N)*sum_i e_i²)，N 为有效点数
def reprojection_error_rms(
    uv_obs: np.ndarray, P_3d: np.ndarray, K: CameraIntrinsics
) -> float:
    if uv_obs.shape[0] == 0:
        return 0.0
    errs = reprojection_error_batch(uv_obs, P_3d, K, squared=True)
    valid = np.isfinite(errs)
    if not np.any(valid):
        return np.nan
    return np.sqrt(np.mean(errs[valid]))
