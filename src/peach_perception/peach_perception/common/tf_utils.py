from __future__ import annotations
"""几何原语：拟合、深度单位、TF、点云 RGB 打包。"""

from dataclasses import dataclass

import numpy as np

# 向量/轴线原语（unit_vector / angle_between_deg / axis_radial_distance）
# 单一事实源在 fitting.py；此处 re-export 维持本模块既有导入路径。
from peach_perception.common.fitting import (  # noqa: F401
    angle_between_deg,
    axis_radial_distance,
    unit_vector
)
from scipy.spatial.transform import Rotation

# 四元数模长平方回退阈值：与原 tf_transformations.quaternion_matrix 的
# _EPS（numpy.finfo(float64).eps）一致；低于此值回退单位旋转。
_QUAT_NORM_EPS = float(np.finfo(np.float64).eps)


@dataclass(frozen=True)
class QuaternionValue:
    """
    单位四元数不可变值对象（geometry_msgs/Quaternion 的纯核替代）.

    纯核不得 import geometry_msgs，故 rotation_to_quat 返回本值对象；
    字段语义与消息一致 (x, y, z, w)。编排层需要消息时自行构造：
    ``Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)``。
    """

    x: float
    y: float
    z: float
    w: float

    def as_tuple(self) -> tuple:
        """返回 (x, y, z, w) 元组，供 scipy Rotation 等数组接口使用."""
        return (self.x, self.y, self.z, self.w)


def transform_msg_to_matrix(transform) -> np.ndarray:
    """
    Transform（鸭子类型）→ 4×4 齐次矩阵 T（p_out = R@p_in + t）.

    有限且模长平方 ≥ float64 eps 的四元数走官方 scipy Rotation 组合
    （与原 tf_transformations 组合在 1e-12 内数值等价——该等价仅对此
    类输入成立）；零/亚 eps 范数四元数回退单位旋转、仅保留平移（沿用
    tf_transformations quaternion_matrix 的同阈值回退语义，区别于
    scipy from_quat 对零范数抛 ValueError）。非有限四元数同样回退单位
    旋转：旧实现会产出 NaN 矩阵污染下游，属未定义垃圾路径，不再保留。
    统一自原 peach_perception.scene_perception.tf_utils._transform_msg_to_matrix
    与 peach_perception.target_reconstruction.tf_utils.transform_msg_to_matrix。

    Args:
        transform: 带 .translation.x/y/z 与 .rotation.x/y/z/w 的对象.

    Returns
    -------
        (4, 4) float64 齐次矩阵（平移单位随消息，通常为米）.

    """
    tr = transform.translation
    q = transform.rotation
    qv = np.asarray([q.x, q.y, q.z, q.w], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    if np.all(np.isfinite(qv)) and float(qv @ qv) >= _QUAT_NORM_EPS:
        T[:3, :3] = Rotation.from_quat(qv).as_matrix()
    T[:3, 3] = (tr.x, tr.y, tr.z)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    4×4 齐次矩阵求逆：T_camera_base = inv(T_base_camera).

    官方 np.linalg.inv（通用 4×4 求逆）：刚体矩阵上数值误差 ~1e-16，
    与手写 [R.T, -R.T@t] 在测试锚点精度（atol=1e-12）内无差别；
    输入的刚性由 test_tf_utils 正逆互反用例守门，无需自造刚体特化。
    （实现沿用原 peach_perception.target_reconstruction.tf_utils.invert_transform。）

    Args:
        T: (4, 4) 齐次矩阵.

    Returns
    -------
        (4, 4) float64 逆矩阵.

    """
    return np.linalg.inv(np.asarray(T, dtype=np.float64))


def relative_motion(T_a: np.ndarray, T_b: np.ndarray) -> tuple:
    """
    两个 base←camera 位姿间的相对运动量（视角过滤用）.

    保留 numpy 闭式（官方无更直等价物）：须绕四元数或 Rotation 对象
    才能取「两旋转夹角」，反而多一次构造往返；trace 闭式
    R_rel→arccos((tr−1)/2) 是教科书标准式，单次矩阵乘即得。
    （实现沿用原 peach_perception.target_reconstruction.tf_utils.relative_motion。）

    Args:
        T_a: (4, 4) 本帧位姿.
        T_b: (4, 4) 参考帧位姿（上一已采帧）.

    Returns
    -------
        (translation_m, rotation_deg)：平移差范数 [m] 与相对旋转角 [deg].

    """
    R_rel = T_a[:3, :3] @ T_b[:3, :3].T
    cos_angle = float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
    rotation_deg = float(np.degrees(np.arccos(cos_angle)))
    translation_m = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    return translation_m, rotation_deg


def rotation_to_quat(R: np.ndarray) -> QuaternionValue:
    """
    3×3 旋转矩阵 → 单位四元数值对象（官方 scipy Rotation.as_quat）.

    Args:
        R: (3, 3) 旋转矩阵（非正交输入的行为随官方实现，调用方保证刚性）.

    Returns
    -------
        QuaternionValue（x, y, z, w），模长为 1.

    """
    q = Rotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # [x, y, z, w]
    return QuaternionValue(
        x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def transform_point(T: np.ndarray, point) -> np.ndarray:
    """
    点按齐次矩阵变换：p_out = R@p_in + t（None 透传）.

    Args:
        T: (4, 4) 齐次矩阵，输出系←输入系.
        point: (3,) 点坐标（米），None 原样返回.

    Returns
    -------
        (3,) float64 输出系点坐标；输入 None 时返回 None.

    """
    if point is None:
        return None
    return T[:3, :3] @ np.asarray(point, dtype=float) + T[:3, 3]


def transform_direction(T: np.ndarray, direction) -> np.ndarray:
    """
    方向向量按齐次矩阵变换：只乘 R 不加平移，并重新归一化（None 透传）.

    平移不影响方向；近零退化向量不归一化（防除零），原样返回旋转结果。

    Args:
        T: (4, 4) 齐次矩阵，输出系←输入系.
        direction: (3,) 方向向量，None 原样返回.

    Returns
    -------
        (3,) float64 输出系单位方向向量；输入 None 时返回 None.

    """
    if direction is None:
        return None
    d = T[:3, :3] @ np.asarray(direction, dtype=float)
    n = float(np.linalg.norm(d))
    return d / n if n > 1e-9 else d


def gravity_camera_from_R(R_out_cam: np.ndarray) -> np.ndarray:
    """
    由 output←camera 旋转反推相机系重力方向（gravity_mode='tf' 用）.

    约定 output_frame（如 base_link）内重力向量为 [0, 0, -1]（竖直向下）；
    方向向量只乘旋转、不加平移：g_cam = normalize(R_out_cam.T @ g_out)。
    （实现沿用原 peach_perception.scene_perception.tf_utils._gravity_camera_from_R。）

    Args:
        R_out_cam: (3, 3) 旋转矩阵，output_frame←相机系.

    Returns
    -------
        (3,) 相机系单位重力向量；退化（近零）时原样返回.

    """
    g = np.asarray(R_out_cam, dtype=float).T @ np.array([0.0, 0.0, -1.0])
    n = float(np.linalg.norm(g))
    return g / n if n > 1e-9 else g


def pack_rgb_bgr(colors_bgr: np.ndarray) -> np.ndarray:
    """
    (N, 3) uint8 BGR → (N,) float32 位打包（0xRRGGBB，RViz RGB8 约定）.

    Args:
        colors_bgr: (N, 3) uint8 数组，列序为 B、G、R（OpenCV 惯例）.

    Returns
    -------
        (N,) float32 视图（位内容为 0xRRGGBB）；空输入给 (0,) 空数组.

    """
    colors = np.asarray(colors_bgr, dtype=np.uint8).reshape(-1, 3)
    if colors.shape[0] == 0:
        return np.zeros((0,), dtype=np.float32)
    b = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    r = colors[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """
    对 (N, 3) 点应用齐次刚体变换 p_out = R@p_in + t，不修改输入.

    Args:
        points: (N, 3) 点.
        transform: (4, 4) 齐次矩阵，输出系←输入系.

    Returns
    -------
        (N, 3) float64 变换后点；空输入给 (0, 3) 空数组.

    """
    xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    T = np.asarray(transform, dtype=np.float64)
    return xyz @ T[:3, :3].T + T[:3, 3]
