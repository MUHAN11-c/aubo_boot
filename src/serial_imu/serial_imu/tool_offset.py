"""工具偏移估计纯核：四元数运算与偏移解算，无 ROS 依赖."""
from __future__ import annotations

import math

# 口径：清零时刻工具居中（零位，对应设计稿方案 B「零位在每次接触前采集」），
# 偏移 = 模组姿态自清零的变化量经安装角共轭、扣除臂自身运动后的旋转，
# 表达在名义 tool_axis 系。安装共轭必需：模组倒装（2026-09-09 实测
# roll≈177°）时若按单位阵处理，偏移轴镜像、跟随方向反号构成正反馈。

DEG = 180.0 / math.pi


def qmul(a, b):
    """四元数乘（w,x,y,z）."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    """共轭（单位四元数的逆）."""
    return (q[0], -q[1], -q[2], -q[3])


def qnorm(q):
    return math.sqrt(sum(x * x for x in q))


def normalize(q):
    """归一化；范数过小视为无效返回 None."""
    n = qnorm(q)
    if n < 0.5:
        return None
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def qrot(q, v):
    """向量旋转：v' = q ⊗ v ⊗ q*."""
    r = qmul(qmul(q, (0.0, v[0], v[1], v[2])), qconj(q))
    return r[1:]


def from_rpy_deg(rpy_deg):
    """ZYX 内旋欧拉（度）→ 四元数（w,x,y,z），与 tf 惯例一致."""
    roll, pitch, yaw = (a / DEG / 2.0 for a in rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def qangle(a, b):
    """两个单位四元数之间的夹角（度）."""
    d = qmul(qconj(a), b)
    w = max(-1.0, min(1.0, d[0]))
    return 2.0 * math.acos(w) * DEG


def vec_angle(a, b):
    """两向量夹角（度）；任一为零向量返回 None."""
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(x * x for x in b))
    if na < 1e-9 or nb < 1e-9:
        return None
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.acos(max(-1.0, min(1.0, c))) * DEG


class ToolOffsetEstimator:
    """IMU→名义工具系偏移解算（mount=清零姿态下标定的安装姿态）."""

    def __init__(self, mount_quat, pivot_depth_m=0.0):
        # mount_quat = R_tool_imu：模组在名义工具系的安装姿态。
        # pivot_depth_m：自适应机构铰点在工具系沿 −Z 的深度；0 = 绕筒口纯旋转。
        self._mount = normalize(mount_quat)
        self._pivot = float(pivot_depth_m)
        self._zero = None

    @property
    def zeroed(self):
        return self._zero is not None

    def zero(self, q_imu, q_base_tool):
        """清零：记录模组姿态与 base→tool_axis 姿态（此刻工具居中)."""
        qi = normalize(q_imu)
        qbt = normalize(q_base_tool)
        if qi is None or qbt is None:
            return False
        self._zero = (qi, qbt)
        return True

    def update(self, q_imu, q_base_tool):
        """解算当前偏移，返回 (q_offset, translation) 或未清零时 None."""
        if self._zero is None:
            return None
        qi = normalize(q_imu)
        qbt = normalize(q_base_tool)
        if qi is None or qbt is None:
            return None
        q0, qbt0 = self._zero
        # 模组体系下自清零的姿态变化
        q_delta = qmul(qconj(q0), qi)
        # 扣臂运动 + 安装共轭 → 名义工具系偏移
        q_off = qmul(qmul(qmul(qmul(qconj(qbt), qbt0), self._mount), q_delta),
                     qconj(self._mount))
        # 铰点 (0,0,−d) 固定于名义系，筒口 = p + R·(−p)
        d = self._pivot
        if d != 0.0:
            tip = qrot(q_off, (0.0, 0.0, d))
            trans = (tip[0], tip[1], tip[2] - d)
        else:
            trans = (0.0, 0.0, 0.0)
        return q_off, trans


def zero_gate_ok(acc, mount_quat, expected_tool_gravity, tol_deg):
    """清零重力自检：实测比力方向 vs 预期 M⁻¹·g_tool，超差拒绝清零."""
    # 防倒装/贴歪/不在零位的安装问题进系统；acc 无效或方向异常返回 False。
    if len(acc) != 3:
        return False
    mag = math.sqrt(sum(x * x for x in acc))
    if not 0.5 < mag < 20.0:
        return False
    m = normalize(mount_quat)
    if m is None:
        return False
    pred = qrot(qconj(m), expected_tool_gravity)
    ang = vec_angle(acc, pred)
    return ang is not None and ang <= tol_deg
