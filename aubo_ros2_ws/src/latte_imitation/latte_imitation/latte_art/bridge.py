"""桥接函数 — 参数化 XYZ 轨迹 → CartesianTrajectory 对象喵~

BFF (纯数学预览) 和 ROS2 LatteImitationNode (真机执行) 共用此桥接,
保证同一 latte_art 输出 → 同一 CartesianTrajectory 格式喵~
"""

import numpy as np
from typing import Optional


def euler_deg_to_quat(roll_deg: float, pitch_deg: float,
                       yaw_deg: float) -> np.ndarray:
    """欧拉角(度) → 四元数 [x,y,z,w] (Hamilton convention, 内旋 ZYX) 喵~

    与 latte_imitation.trajectory_transform.euler_deg_to_quat 完全一致的实现,
    避免 latte_art 模块反向依赖 trajectory_transform 喵~
    """
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def parametric_to_cartesian(
    xyz: np.ndarray,
    roll_deg: float = 0.0,
    pitch_deg: float = 45.0,
    yaw_deg: float = 0.0,
    dt: float = 0.05,
    episode_idx: int = -1,
    frame_id: str = "base_link",
) -> "CartesianTrajectory":
    """将参数化 XYZ 轨迹转换为 CartesianTrajectory 对象喵~

    为纯位置轨迹添加固定姿态 (奶缸前倾倒奶), 使其兼容现有 6 阶段管线喵~

    Args:
        xyz: (T, 3) XYZ 位置轨迹
        roll_deg: 绕 X 轴旋转 (度)
        pitch_deg: 绕 Y 轴旋转 (度), 默认 45° = 奶缸前倾倒奶
        yaw_deg: 绕 Z 轴旋转 (度)
        dt: 时间步长 (s)
        episode_idx: 标记为 -1 = 生成轨迹 (区分于录制的 0-39)
        frame_id: 坐标系 ID

    Returns:
        CartesianTrajectory 对象, 可直接送入 retarget → safety → MoveIt2 管线喵~
    """
    from latte_imitation.trajectory import CartesianTrajectory

    T = len(xyz)
    quat = euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    orientations = np.tile(quat.astype(np.float32), (T, 1))
    timestamps = np.arange(T, dtype=np.float32) * dt

    return CartesianTrajectory(
        positions=xyz.astype(np.float32),
        orientations=orientations,
        timestamps=timestamps,
        dt=dt,
        episode_idx=episode_idx,
        frame_id=frame_id,
    )
