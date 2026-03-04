# Core: quaternion, rotation matrix, RPY (no ROS)，使用 scipy.spatial.transform.Rotation 计算
# 四元数约定 (x, y, z, w)，欧拉角内旋顺序 XYZ（roll→pitch→yaw），与 ROS/tf2 一致
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Union


def _ensure_3x3(M: np.ndarray) -> np.ndarray:
    return np.asarray(M, dtype=np.float64).reshape(3, 3)


def _ensure_quat(q) -> np.ndarray:
    a = np.asarray(q, dtype=np.float64).flatten()
    if len(a) != 4:
        raise ValueError("Quaternion must have 4 elements (x,y,z,w)")
    return a


# 四元数 q → 旋转矩阵 R（3×3）
# 公式: R = 2*q*q^T - (q·q)I + 2*q_w*[q_v]_× + 2*[q_v]_×^2
# 展开: R = [ 1-2(y²+z²)   2(xy-zw)   2(xz+yw);  2(xy+zw)  1-2(x²+z²)  2(yz-xw);  2(xz-yw)  2(yz+xw)  1-2(x²+y²) ]
def quaternion_to_rotation_matrix(q: Union[tuple, list, np.ndarray]) -> np.ndarray:
    q = _ensure_quat(q)
    r = R.from_quat(q)
    return r.as_matrix()


# 旋转矩阵 R → 单位四元数 q。常用 Shepperd 法按 trace 选分支: trace=R00+R11+R22, s=sqrt(trace+1), w=(trace+1)/(4s), x=(R21-R12)/(4s) 等
def rotation_matrix_to_quaternion(R_mat: np.ndarray) -> np.ndarray:
    R_mat = _ensure_3x3(R_mat)
    r = R.from_matrix(R_mat)
    return r.as_quat()  # (x,y,z,w)


# 欧拉角 RPY(roll,pitch,yaw) → 四元数。内旋 XYZ: q = q_roll*q_pitch*q_yaw
# 公式: qx = sr*cp*cy - cr*sp*sy, qy = cr*sp*cy + sr*cp*sy, qz = cr*cp*sy - sr*sp*cy, qw = cr*cp*cy + sr*sp*sy（半角 r/2,p/2,y/2）
def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r = R.from_euler('xyz', [roll, pitch, yaw])
    return r.as_quat()


# 四元数 q → RPY。公式: sin(pitch)=2(w*y-z*x); roll=atan2(2(w*x+y*z),1-2(x²+y²)); yaw=atan2(2(w*z+x*y),1-2(y²+z²))
def quaternion_to_rpy(q: Union[tuple, list, np.ndarray]) -> np.ndarray:
    q = _ensure_quat(q)
    r = R.from_quat(q)
    return r.as_euler('xyz')


# RPY → 旋转矩阵。R = Rz(yaw)*Ry(pitch)*Rx(roll)。公式: R = [ cp*cy  -cr*sy+sr*sp*cy  sr*sy+cr*sp*cy; ... ]
def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r = R.from_euler('xyz', [roll, pitch, yaw])
    return r.as_matrix()


# 旋转矩阵 R → RPY。先 R→q 再 q→RPY
def rotation_matrix_to_rpy(R_mat: np.ndarray) -> np.ndarray:
    R_mat = _ensure_3x3(R_mat)
    r = R.from_matrix(R_mat)
    return r.as_euler('xyz')
