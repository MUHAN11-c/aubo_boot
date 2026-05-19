"""
SE(3) 笛卡尔轨迹重定目标 — RM65 数据集 → AUBO 工作空间喵~

=== 理论依据 ===

  SPOT (arXiv:2411.00965, Section IV-A):
    Object-centric SE(3) trajectory: convert source object poses into target
    object's frame → canonical space. "This transforms multiple demonstration
    trajectories into a canonical space regardless of their absolute configurations."

  Isaac Teleop (NVIDIA, Se3RetargeterConfig):
    Se3RelRetargeter: outputs 6D delta (position delta + rotation vector).
    target_offset_roll/pitch/yaw (degrees, intrinsic XYZ Euler).
    本项目 Option B (默认) = Se3RelRetargeter 语义: 朝向作为增量叠加。

  SO(3) Action Representations (Savva/Schuck et al., 2025, Table 2):
    Delta tangent vector in local frame: BEST performance across ALL algorithms
    (PPO, SAC, TD3) and ALL reward formulations (dense, sparse).
    "For code: use quaternions (Hamilton convention) for storage and computation."

  SVRC Trajectory Representation:
    Object-relative Cartesian → Very High generalization.
    Cartesian delta actions → High generalization.

  ROS 2 REP-103 + tf2 Quaternion.h:
    四元数 Hamilton 约定 (xyzw): q 和 -q 表示同一旋转 (double-cover SO(3)).
    RPY 使用 intrinsic ZYX = extrinsic XYZ (绕固定轴 roll→X, pitch→Y, yaw→Z).

=== 变换流程 ===

  Step 1: 计算旋转矩阵 R_rel
    R_user = quat_to_rot(euler_deg_to_quat(rpy_user))
    R_cup  = quat_to_rot(q_cup)
    R_rel  = R_user @ R_cup        ← Option B (默认): 相对叠加旋转

  Step 2: 位置变换 (以 Frame 0 为旋转中心)
    p0 = cart.positions[0]
    p_new[i] = R_rel @ (p_orig[i] - p0) + p_target

  Step 3: 朝向变换 (Hamilton 乘积: 先 q_orig, 后 q_rel)
    q_rel = rot_to_quat(R_rel)
    q_new[i] = q_rel * q_orig[i]  ← Hamilton 右乘约定

=== 四元数公式 (Hamilton convention, xyzw) ===

  四元数 → 旋转矩阵:
    R = [[1-2(y²+z²), 2(xy-wz), 2(xz+wy)],
         [2(xy+wz), 1-2(x²+z²), 2(yz-wx)],
         [2(xz-wy), 2(yz+wx), 1-2(x²+y²)]]

  来源: ROS 2 tf2/LinearMath/Quaternion.h

  Hamilton 乘积 q1 * q2 (先应用 q2, 再应用 q1):
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
"""

import numpy as np
from geometry_msgs.msg import Pose

from .trajectory import CartesianTrajectory


# ═══════════════════════════════════════════════════════════════════
# Pose 工具
# ═══════════════════════════════════════════════════════════════════

def is_default_position(pose: Pose) -> bool:
    """position 三字段全零 → 自动从 TF 获取位置喵~"""
    return (
        abs(pose.position.x) < 1e-9
        and abs(pose.position.y) < 1e-9
        and abs(pose.position.z) < 1e-9
    )


def is_default_orientation(pose: Pose) -> bool:
    """orientation=(0,0,0,1) → 不旋转轨迹朝向 (纯平移) 喵~"""
    return (
        abs(pose.orientation.x) < 1e-9
        and abs(pose.orientation.y) < 1e-9
        and abs(pose.orientation.z) < 1e-9
        and abs(pose.orientation.w - 1.0) < 1e-9
    )


# 向后兼容别名喵~
is_default_pose = lambda pose: is_default_position(pose) and is_default_orientation(pose)


# ═══════════════════════════════════════════════════════════════════
# 四元数工具 (Hamilton convention, xyzw)
# ═══════════════════════════════════════════════════════════════════

def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """四元数 [x,y,z,w] (Hamilton convention) → 3×3 旋转矩阵喵~

    公式来源: ROS 2 tf2/LinearMath/Quaternion.h
      R = [[1-2(y²+z²), 2(xy-wz), 2(xz+wy)],
           [2(xy+wz), 1-2(x²+z²), 2(yz-wx)],
           [2(xz-wy), 2(yz+wx), 1-2(x²+y²)]]

    参考: SO(3) Action Repr. (Savva et al., 2025 Section 2.1) — Hamilton 约定
    """
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → 四元数 [x,y,z,w] (Hamilton convention) 喵~

    使用最大元素法保证数值稳定性。当 trace > 0 时优先用 trace 分支,
    否则选最大对角元素对应分支喵~
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


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton 乘积 q1 * q2 = 先应用 q2, 再应用 q1 喵~

    参考: SO(3) Action Repr. (Savva et al., 2025 Section 2.1)
          "Quaternion Hamiltonian product represents composition of rotations"
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def euler_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """欧拉角(度) → 四元数 [x,y,z,w] (内旋 ZYX = 外旋 XYZ) 喵~

    公式验证: 与 ROS 2 tf2::Quaternion::setRPY() 完全一致 喵~
        setValue(
            sinRoll*cosPitch*cosYaw - cosRoll*sinPitch*sinYaw,  // x
            cosRoll*sinPitch*cosYaw + sinRoll*cosPitch*sinYaw,  // y
            cosRoll*cosPitch*sinYaw - sinRoll*sinPitch*cosYaw,  // z
            cosRoll*cosPitch*cosYaw + sinRoll*sinPitch*sinYaw   // w
        );

    参考:
        ROS 2 REP-103: 固定轴 RPY (绕 X, 绕 Y, 绕 Z) = 内旋 ZYX
        Isaac Teleop: target_offset_roll/pitch/yaw (intrinsic XYZ Euler)
    """
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    ])


def quat_to_euler_deg(q: np.ndarray) -> tuple[float, float, float]:
    """四元数 [x,y,z,w] → 欧拉角 (度, 内旋 ZYX) 喵~

    返回: (roll_deg, pitch_deg, yaw_deg)
    """
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    # roll (绕 X 轴)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # pitch (绕 Y 轴)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2.0, sinp)
    else:
        pitch = np.arcsin(sinp)
    # yaw (绕 Z 轴)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return tuple(np.degrees([roll, pitch, yaw]))


# ═══════════════════════════════════════════════════════════════════
# 核心变换: SE(3) 重定目标
# ═══════════════════════════════════════════════════════════════════

def compute_rotation_matrix(
    rpy_user: tuple[float, float, float],
    q_cup: np.ndarray,
) -> np.ndarray:
    """计算 SE(3) 重定目标的旋转矩阵 R_rel 喵~

    Option B (相对叠加旋转):
        R_rel = R(rpy_user) @ R(q_cup)
        → 先转杯子方向, 再转用户可调角度
        → 保留轨迹固有倾斜 (对应 Isaac Teleop Se3RelRetargeter)

    参考:
        Isaac Teleop: Se3RetargeterConfig target_offset_roll/pitch/yaw (degrees,
            intrinsic XYZ Euler).
        SVRC: Object-relative Cartesian → Very High generalization.

    Args:
        rpy_user: (roll_deg, pitch_deg, yaw_deg) 用户可调角度
        q_cup:    杯子位姿四元数 [x,y,z,w] (AUBO base frame)

    Returns:
        3×3 旋转矩阵 R_rel
    """
    q_user = euler_deg_to_quat(*rpy_user)
    R_user = quat_to_rot(q_user)
    R_cup = quat_to_rot(q_cup)
    return R_user @ R_cup


def compute_rotation_matrix_absolute(
    rpy_user: tuple[float, float, float],
    q_cup: np.ndarray,
    q_orig: np.ndarray,
) -> np.ndarray:
    """计算 SE(3) 重定目标的旋转矩阵 R_rel (Option A: 绝对目标朝向) 喵~

    Option A:
        R_rel = R(rpy_user) @ R(q_cup) @ R(q_orig)^T
        → Frame 0 的绝对朝向 = rpy_user * q_cup
        → "抹平"原始轨迹倾斜 (对应 Isaac Teleop Se3AbsRetargeter)

    使用场景: 需要工具精确对齐到指定绝对朝向时喵~
    不推荐用于 pouring 任务 (会丢失 pouring tilt 技能) 喵~

    参考:
        Isaac Teleop: Se3AbsRetargeter — outputs 7D absolute pose
        Google Pouring Dataset (Sermanet 2017): Tool tilt is critical
    """
    q_user = euler_deg_to_quat(*rpy_user)
    R_user = quat_to_rot(q_user)
    R_cup = quat_to_rot(q_cup)
    R_orig_inv = quat_to_rot(q_orig).T
    return R_user @ R_cup @ R_orig_inv


def retarget_trajectory(
    cart: CartesianTrajectory,
    start_pose: Pose,
    rpy_user: tuple[float, float, float] = (0.0, 0.0, 0.0),
    absolute_orientation: bool = False,
) -> CartesianTrajectory:
    """SE(3) 重定目标: RM65 base frame → AUBO base frame 喵~

    变换公式:
        p_new[i] = R_rel @ (p_orig[i] - p_orig[0]) + p_target
        q_new[i] = q_rel * q_orig[i]  (Hamilton 乘积)

    特性:
        - 刚性 (保距保角): path_length 不变 喵~
        - 旋转中心 = 第一帧位置 (杯子上方) 喵~
        - 纯平移时 (rpy_user=0, q_cup=identity): R_rel = I 喵~

    理论依据:
        SPOT (arXiv:2411.00965): Object-centric SE(3) 轨迹, cross-embodiment
        Isaac Teleop: Se3RelRetargeter (Option B) / Se3AbsRetargeter (Option A)
        SVRC: Object-relative Cartesian → Very High generalization

    Args:
        cart:                原始 CartesianTrajectory (RM65 base frame) 喵~
        start_pose:          目标起点位姿 (杯子位姿, AUBO base frame) 喵~
        rpy_user:            (roll_deg, pitch_deg, yaw_deg) 用户可调旋转 喵~
        absolute_orientation: False=相对叠加 (默认, 保留倾斜) /
                             True=绝对目标朝向 (Frame0 = rpy_user*q_cup) 喵~

    Returns:
        变换后的 CartesianTrajectory (AUBO base frame) 喵~
    """
    p0 = cart.positions[0].copy()
    p_target = np.array([
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z,
    ])
    q_cup = np.array([
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w,
    ])

    # 检查是否需要旋转
    all_zero_rpy = all(abs(r) < 1e-9 for r in rpy_user)
    is_identity_cup = (
        abs(q_cup[0]) < 1e-9 and abs(q_cup[1]) < 1e-9
        and abs(q_cup[2]) < 1e-9 and abs(q_cup[3] - 1.0) < 1e-9
    )
    rotate = not (all_zero_rpy and is_identity_cup)

    if rotate:
        if absolute_orientation and cart.orientations is not None:
            q_orig = cart.orientations[0]
            R_rel = compute_rotation_matrix_absolute(rpy_user, q_cup, q_orig)
        else:
            R_rel = compute_rotation_matrix(rpy_user, q_cup)
    else:
        R_rel = np.eye(3)

    # 位置变换: p_new = R_rel @ (p - p0) + p_target
    new_positions = np.array([
        R_rel @ (p - p0) + p_target for p in cart.positions
    ])

    # 朝向变换: q_new = q_rel * q_orig (Hamilton 乘积)
    if cart.orientations is not None and rotate:
        q_rel = rot_to_quat(R_rel)
        new_orientations = np.array([
            quat_multiply(q_rel, q) for q in cart.orientations
        ])
    else:
        new_orientations = (
            None if cart.orientations is None
            else cart.orientations.copy()
        )

    return CartesianTrajectory(
        positions=new_positions,
        orientations=new_orientations,
        timestamps=cart.timestamps.copy(),
        dt=cart.dt,
        episode_idx=cart.episode_idx,
        frame_id=cart.frame_id,
    )
