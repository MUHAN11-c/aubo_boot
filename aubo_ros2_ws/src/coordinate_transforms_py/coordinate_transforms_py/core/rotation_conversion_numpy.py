# 旋转表示转换（可选分支：仅用 NumPy，无 scipy 依赖）
# 与 rotation_conversion.py 同 API，适合无 scipy 环境；公式与 ROS/tf2 约定一致 (x,y,z,w), 内旋 XYZ
import numpy as np
from typing import Union


def _ensure_3x3(M: np.ndarray) -> np.ndarray:
    return np.asarray(M, dtype=np.float64).reshape(3, 3)


def _ensure_quat(q) -> np.ndarray:
    a = np.asarray(q, dtype=np.float64).flatten()
    if len(a) != 4:
        raise ValueError("Quaternion must have 4 elements (x,y,z,w)")
    return a


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    # 归一化: q := q / |q|, |q| = sqrt(x²+y²+z²+w²)
    n = np.linalg.norm(q)
    if n > 1e-10:
        q = q / n
    return q


# 四元数 q → 旋转矩阵 R（3×3）
# 公式: R_ij 由单位四元数 q=(x,y,z,w) 得
#   R = [ 1-2(y²+z²)   2(xy-zw)   2(xz+yw)
#         2(xy+zw)   1-2(x²+z²)  2(yz-xw)
#         2(xz-yw)   2(yz+xw)   1-2(x²+y²) ]
def quaternion_to_rotation_matrix(q: Union[tuple, list, np.ndarray]) -> np.ndarray:
    q = _ensure_quat(q)
    q = _quat_normalize(q)
    x, y, z, w = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


# 旋转矩阵 R → 单位四元数 q。Shepperd 法: 按 trace 与对角元选分支保证数值稳定
# trace = R00+R11+R22; 若 trace>0: s=0.5/sqrt(trace+1), w=0.25/s, x=(R21-R12)*s, y=(R02-R20)*s, z=(R10-R01)*s
def rotation_matrix_to_quaternion(R_mat: np.ndarray) -> np.ndarray:
    R_mat = _ensure_3x3(R_mat)
    R = R_mat
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    q = np.empty(4)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[1, 2] - R[2, 1]) * s
        q[1] = (R[2, 0] - R[0, 2]) * s
        q[2] = (R[0, 1] - R[1, 0]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q[3] = (R[1, 2] - R[2, 1]) / s
        q[0] = 0.25 * s
        q[1] = (R[0, 1] + R[1, 0]) / s
        q[2] = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q[3] = (R[2, 0] - R[0, 2]) / s
        q[0] = (R[0, 1] + R[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q[3] = (R[0, 1] - R[1, 0]) / s
        q[0] = (R[0, 2] + R[2, 0]) / s
        q[1] = (R[1, 2] + R[2, 1]) / s
        q[2] = 0.25 * s
    return _quat_normalize(q)


# 欧拉角 RPY → 四元数。内旋 X→Y→Z: q = q_roll * q_pitch * q_yaw
# 公式: qx = sr*cp*cy - cr*sp*sy, qy = cr*sp*cy + sr*cp*sy, qz = cr*cp*sy - sr*sp*cy, qw = cr*cp*cy + sr*sp*sy
# 其中 cr=cos(r/2), sr=sin(r/2), cp=cos(p/2), sp=sin(p/2), cy=cos(y/2), sy=sin(y/2)
def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    q = np.array([
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    ], dtype=np.float64)
    return _quat_normalize(q)


# 四元数 q → RPY。公式: sin(pitch)=2(w*y-z*x); roll=atan2(2(w*x+y*z),1-2(x²+y²)); yaw=atan2(2(w*z+x*y),1-2(y²+z²))
# 万向锁时 pitch=±π/2，仅 roll+yaw 组合有定义
def quaternion_to_rpy(q: Union[tuple, list, np.ndarray]) -> np.ndarray:
    q = _ensure_quat(q)
    q = _quat_normalize(q)
    x, y, z, w = q[0], q[1], q[2], q[3]
    sinp = 2.0 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp) if np.abs(sinp) < 1 else np.copysign(np.pi / 2.0, sinp)
    roll = np.arctan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))
    yaw = np.arctan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    return np.array([roll, pitch, yaw], dtype=np.float64)


# RPY → 旋转矩阵。R = Rz(yaw)*Ry(pitch)*Rx(roll)
# 公式: R = [ cp*cy   -cr*sy+sr*sp*cy   sr*sy+cr*sp*cy
#            cp*sy    cr*cy+sr*sp*sy   -sr*cy+cr*sp*sy
#            -sp      sr*cp             cr*cp ]
def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cp*cy, -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy],
        [cp*sy, cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy],
        [-sp, sr*cp, cr*cp],
    ], dtype=np.float64)


# 旋转矩阵 R → RPY。先 R→q 再 q→RPY
def rotation_matrix_to_rpy(R_mat: np.ndarray) -> np.ndarray:
    q = rotation_matrix_to_quaternion(R_mat)
    return quaternion_to_rpy(q)
