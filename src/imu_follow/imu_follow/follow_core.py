"""
imu_follow 纯核：四元数/旋转向量原语与 IMU→TCP 姿态映射（零 ROS、零第三方）.

约定：四元数一律 (x, y, z, w) 元组、单位化；旋转向量 v 的模即转角
（rad）、方向即体轴。IMU 姿态增量按体轴分量符号表映射到 TCP 体轴（每
分量幅值不变、模不变），死区与锥限幅在映射后统一施加。大角度下该映射
是「手感」近似而非严格共轭/镜像，配 follow.max_delta_rad 小锥使用。
"""

import math

_EPS = 1e-12


def quat_normalize(q):
    """归一化四元数；近零向量按单位姿态返回."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conj(q):
    """共轭（单位四元数即逆）."""
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    """四元数乘法 a·b（合成旋转：先 b 后 a）."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz)


def rotvec_to_quat(v):
    """旋转向量 → 四元数（Rodrigues；模为零返回单位姿态）."""
    theta = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if theta < _EPS:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(0.5 * theta) / theta
    return (v[0] * s, v[1] * s, v[2] * s, math.cos(0.5 * theta))


def quat_to_rotvec(q):
    """四元数 → 旋转向量（先折到 w>=0 短弧，转角 ∈ [0, π]）."""
    x, y, z, w = quat_normalize(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    w = min(1.0, w)
    theta = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < _EPS or theta < _EPS:
        return (0.0, 0.0, 0.0)
    k = theta / s
    return (x * k, y * k, z * k)


def map_signs(v, signs):
    """按体轴符号表映射旋转向量分量（各分量幅值与模不变）."""
    return (v[0] * signs[0], v[1] * signs[1], v[2] * signs[2])


def apply_deadband(v, deadband_rad):
    """模长不足死区视为无输入；过界原样通过（无滞回）."""
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < deadband_rad:
        return (0.0, 0.0, 0.0)
    return (v[0], v[1], v[2])


def clamp_rotvec(v, max_rad):
    """模长钳到锥角上限，方向不变."""
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n <= max_rad or n < _EPS:
        return (v[0], v[1], v[2])
    k = max_rad / n
    return (v[0] * k, v[1] * k, v[2] * k)


def delta_rotvec(q_ref, q_now):
    """两姿态的相对旋转向量（体轴）：conj(q_ref)·q_now."""
    return quat_to_rotvec(quat_mul(quat_conj(q_ref), q_now))


def target_orientation(q_tcp_ref, q_imu_ref, q_imu_now, signs, deadband_rad,
                       max_delta_rad):
    """跟随目标姿态：q_tcp_ref · clamp(deadband(map(Δimu)))."""
    v = delta_rotvec(q_imu_ref, q_imu_now)
    v = map_signs(v, signs)
    v = apply_deadband(v, deadband_rad)
    v = clamp_rotvec(v, max_delta_rad)
    return quat_normalize(quat_mul(q_tcp_ref, rotvec_to_quat(v)))


def slerp_toward(q_from, q_to, alpha):
    """向 q_to 插值一步（alpha∈[0,1]，走短弧；近平行走 nlerp）."""
    a = quat_normalize(q_from)
    b = quat_normalize(q_to)
    alpha = max(0.0, min(1.0, alpha))
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-x for x in b)
        dot = -dot
    if dot > 0.9995:
        return quat_normalize(
            tuple(x + alpha * (y - x) for x, y in zip(a, b)))
    theta = math.acos(min(1.0, dot))
    s = math.sin(theta)
    wa = math.sin((1.0 - alpha) * theta) / s
    wb = math.sin(alpha * theta) / s
    return quat_normalize(tuple(wa * x + wb * y for x, y in zip(a, b)))


def clamp_joint_step(target, current, max_step):
    """逐关节把 target 相对 current 的单步位移钳到 max_step."""
    out = []
    for t, c in zip(target, current):
        d = t - c
        if d > max_step:
            t = c + max_step
        elif d < -max_step:
            t = c - max_step
        out.append(float(t))
    return out


def quat_rotate(q, v):
    """把三维向量 v 按单位四元数 q 旋转（q 为体 → 目标系姿态）."""
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_mul(quat_mul(q, qv), quat_conj(q))
    return (r[0], r[1], r[2])


def scale_vector(v, k):
    """三维向量数乘."""
    return (v[0] * k, v[1] * k, v[2] * k)


def insertion_step(travel_m, speed_m_s, dt_s, max_travel_m):
    """
    插入推进行程积分一步：travel + speed·dt，钳到 max_travel.

    套入直线段（~/insert_start）的行程数学：速度运行期可改参，积分按
    当前速度走；达到 max_travel 即封顶（调用方据此自动停推进）。
    """
    return min(travel_m + max(0.0, speed_m_s) * max(0.0, dt_s), max_travel_m)


def insertion_position(ref_pos, direction, travel_m):
    """插入期间的位置目标：参考点沿锁向方向推进 travel 米."""
    return (
        ref_pos[0] + direction[0] * travel_m,
        ref_pos[1] + direction[1] * travel_m,
        ref_pos[2] + direction[2] * travel_m)
