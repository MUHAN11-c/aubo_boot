"""
笛卡尔轨迹 6-DOF 刚性变换 — RM65 数据集 → AUBO 工作空间喵~

=== 问题 ===

  latte-pour-demos 数据集中的轨迹是 RM65 机械臂的末端笛卡尔位姿，
  在 RM65 的 base frame 中表达。实际使用时杯子由 AUBO E5 夹持，
  相机检测杯子在 AUBO base frame 中的位姿 (camera_pose) 喵~

=== 变换逻辑 ===

  给定:
    - 数据集轨迹 T = [p_0, p_1, ..., p_N]  (RM65 base frame)
    - 相机检测杯子位姿 P_target = (pos_target, quat_target)  (AUBO base frame)

  计算刚体变换:
    p0 = T[0]                                   原始起点
    q_orig = T.orientations[0]                   原始第一帧姿态
    R_rel = R(q_target) @ R(q_orig)^T           相对旋转矩阵
    p_new_i = R_rel @ (p_i - p0) + pos_target    变换后位置

  变换特性:
    - 刚性 (保距保角): path_length 不变喵~
    - 旋转中心 = 第一帧位置 (杯子上方) 喵~
    - 纯平移时 R_rel = I，退化为简单 offset 喵~

=== 参考 ===

  四元数 → 旋转矩阵公式 (Hamilton convention, xyzw):
    R = [[1-2(y²+z²), 2(xy-wz), 2(xz+wy)],
         [2(xy+wz), 1-2(x²+z²), 2(yz-wx)],
         [2(xz-wy), 2(yz+wx), 1-2(x²+y²)]]
"""

import numpy as np
from geometry_msgs.msg import Pose

from .trajectory import CartesianTrajectory


def is_default_pose(pose: Pose) -> bool:
    """判断 Pose 是否为 ROS2 默认值 (未显式设置 start_pose) 喵~

    ROS2 service call 未填 start_pose 字段时，所有数值默认为 0，
    orientation.w 默认为 1.0 (identity quaternion) 喵~

    Returns:
        True 如果 pose 是所有字段为默认零/identity 喵~
    """
    return (abs(pose.position.x) < 1e-9 and
            abs(pose.position.y) < 1e-9 and
            abs(pose.position.z) < 1e-9 and
            abs(pose.orientation.x) < 1e-9 and
            abs(pose.orientation.y) < 1e-9 and
            abs(pose.orientation.z) < 1e-9 and
            abs(pose.orientation.w - 1.0) < 1e-9)


def quat_to_rot(q):
    """四元数 xyzw (Hamilton convention) → 3x3 旋转矩阵喵~

    Args:
        q: array-like [x, y, z, w] 喵~

    Returns:
        np.ndarray (3, 3) 旋转矩阵喵~
    """
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),   2*(x*z + w*y)],
        [2*(x*y + w*z),    1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),    2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def rot_to_quat(R):
    """3x3 旋转矩阵 → 四元数 [x, y, z, w] (Hamilton convention) 喵~

    使用最大元素法保证数值稳定性喵~
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quat_multiply(q1, q2):
    """Hamilton 乘积 q1 * q2，两者均为 [x, y, z, w] 喵~"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def apply_start_pose(cart: CartesianTrajectory, start_pose: Pose) -> CartesianTrajectory:
    """将轨迹做 6-DOF 刚性变换，使第一帧对齐到相机检测的杯子位姿喵~

    变换步骤:
      1. 以第一帧位置 p0 为旋转中心 (杯子上方) 喵~
      2. 计算 R_rel = R_target @ R_orig^T (原始姿态 → 目标姿态的相对旋转) 喵~
      3. 对所有轨迹点: p_new = R_rel @ (p - p0) + p_target 喵~

    Args:
        cart: 原始 CartesianTrajectory (RM65 base frame, episode_*.npz) 喵~
        start_pose: 目标起点位姿 (AUBO base frame, 相机检测到的杯子位姿) 喵~
                    position: 杯子在世界坐标系中的位置喵~
                    orientation: 杯子的朝向 (四元数 xyzw) 喵~

    Returns:
        变换后的 CartesianTrajectory，路径长度不变 (刚性变换保距) 喵~

    Example:
        # 杯子在 (0.3, 0.0, 0.5)，杯口朝前 (identity)
        apply_start_pose(cart, Pose(position=(0.3, 0, 0.5), orientation=(0,0,0,1)))

        # 杯子倾斜 180° (绕 Z 轴翻转)
        apply_start_pose(cart, Pose(position=(0.3, 0, 0.5), orientation=(0,0,1,0)))
    """
    p0 = cart.positions[0].copy()
    p_target = np.array([start_pose.position.x,
                         start_pose.position.y,
                         start_pose.position.z])

    # 原始第一帧姿态 — trajectory 数据中可能无 orientation (纯位置轨迹) 喵~
    q_orig = (cart.orientations[0] if cart.orientations is not None
              else np.array([0., 0., 0., 1.]))
    q_tgt = np.array([start_pose.orientation.x, start_pose.orientation.y,
                      start_pose.orientation.z, start_pose.orientation.w])

    R_orig = quat_to_rot(q_orig)
    R_tgt = quat_to_rot(q_tgt)
    R_rel = R_tgt @ R_orig.T  # 相对旋转: 原始姿态 → 目标姿态喵~

    # 变换所有轨迹点: 先绕杯子中心旋转，再平移到杯子位置喵~
    new_positions = np.array([R_rel @ (p - p0) + p_target for p in cart.positions])

    # 旋转 orientation: q_new = q_rel * q_orig (Hamilton 乘积) 喵~
    if cart.orientations is not None:
        q_rel = rot_to_quat(R_rel)
        new_orientations = np.array([quat_multiply(q_rel, q) for q in cart.orientations])
    else:
        new_orientations = None

    return CartesianTrajectory(
        positions=new_positions,
        orientations=new_orientations,
        timestamps=cart.timestamps.copy(),
        dt=cart.dt,
        episode_idx=cart.episode_idx,
        frame_id=cart.frame_id,
    )
