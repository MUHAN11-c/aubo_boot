"""坐标系修正与对齐：倒装 Rx 与 parent 对齐（无 ROS）."""
from __future__ import annotations

from dataclasses import dataclass
import math

from serial_imu.protocol import ImuSample

DEG = 180.0 / math.pi
IDENTITY = (1.0, 0.0, 0.0, 0.0)


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


def normalize(q):
    """归一化；范数过小视为无效返回 None."""
    n = math.sqrt(sum(x * x for x in q))
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


def rpy_deg_from_quat(q):
    """四元数（w,x,y,z）→ ZYX 内旋欧拉（度）."""
    qn = normalize(q)
    if qn is None:
        return None
    w, x, y, z = qn
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll * DEG, pitch * DEG, yaw * DEG)


def qangle(a, b):
    """两个单位四元数之间的夹角（度）."""
    d = qmul(qconj(a), b)
    w = max(-1.0, min(1.0, d[0]))
    return 2.0 * math.acos(w) * DEG


@dataclass(frozen=True)
class ImuFrame:
    """acc/gyro/mag 与姿态（wxyz）；q_valid 为 False 时姿态不可用."""

    acc: tuple[float, float, float]
    gyro: tuple[float, float, float]
    mag: tuple[float, float, float]
    q_wxyz: tuple[float, float, float, float]
    q_valid: bool

    @classmethod
    def from_sample(cls, sample: ImuSample):
        """协议样本 → 模组体轴，不做任何旋转."""
        return cls(
            acc=(sample.acc_x, sample.acc_y, sample.acc_z),
            gyro=(sample.gyro_x, sample.gyro_y, sample.gyro_z),
            mag=(sample.mag_x, sample.mag_y, sample.mag_z),
            q_wxyz=(sample.orientation_w, sample.orientation_x,
                    sample.orientation_y, sample.orientation_z),
            q_valid=sample.orientation_valid,
        )


def _apply_static(r_link_mod, frame: ImuFrame) -> ImuFrame:
    """模组体轴 → imu_link：向量左乘 R，姿态右乘 R⁻¹."""
    q = frame.q_wxyz
    if frame.q_valid:
        qn = normalize(q)
        if qn is not None:
            q = qmul(qn, qconj(r_link_mod))
    return ImuFrame(
        acc=qrot(r_link_mod, frame.acc),
        gyro=qrot(r_link_mod, frame.gyro),
        mag=qrot(r_link_mod, frame.mag),
        q_wxyz=q,
        q_valid=frame.q_valid,
    )


def _apply_align(q_corr, frame: ImuFrame) -> ImuFrame:
    """把 capture 得到的 q_corr 乘到姿态，并同样转 acc/gyro/mag."""
    qc = normalize(q_corr)
    if qc is None:
        return frame
    q_out = frame.q_wxyz
    if frame.q_valid:
        qn = normalize(frame.q_wxyz)
        if qn is not None:
            q_out = qmul(qc, qn)
    return ImuFrame(
        acc=qrot(qc, frame.acc),
        gyro=qrot(qc, frame.gyro),
        mag=qrot(qc, frame.mag),
        q_wxyz=q_out,
        q_valid=frame.q_valid,
    )


class FramePipeline:
    """静态坐标系修正 + 可选相对 parent 的误差清零."""

    def __init__(self, frame_rpy_deg):
        """frame_rpy_deg 为模组体轴→imu_link 的 ZYX 欧拉（度）."""
        r = from_rpy_deg([float(v) for v in frame_rpy_deg])
        self._r = normalize(r) or IDENTITY
        self._q_align = None

    def raw(self, sample: ImuSample) -> ImuFrame:
        """模组体轴，协议原样."""
        return ImuFrame.from_sample(sample)

    def framed(self, sample: ImuSample) -> ImuFrame:
        """仅静态修正（倒装→imu_link），未乘 parent 对齐."""
        return _apply_static(self._r, self.raw(sample))

    def corrected(self, sample: ImuSample) -> ImuFrame:
        """静态修正后再乘对齐（若已 capture）."""
        framed = self.framed(sample)
        if self._q_align is None:
            return framed
        return _apply_align(self._q_align, framed)

    def capture(self, q_parent, q_framed):
        """令 q_corr ⊗ q_framed = q_parent；当前差当安装/航向误差."""
        qp = normalize(q_parent)
        qi = normalize(q_framed)
        if qp is None or qi is None:
            return False, '四元数无效'
        self._q_align = qmul(qp, qconj(qi))
        rpy = rpy_deg_from_quat(self._q_align)
        return True, f'residual_rpy_deg={_fmt_rpy(rpy)}'

    @property
    def aligned(self) -> bool:
        """是否已 capture 过对齐."""
        return self._q_align is not None

    def residual_rpy_deg(self):
        """对齐校正的 ZYX 欧拉（度）；未对齐返回 None."""
        if self._q_align is None:
            return None
        return rpy_deg_from_quat(self._q_align)


def _fmt_rpy(rpy):
    """日志用欧拉三元组；无效四元数时写 invalid."""
    if rpy is None:
        return 'invalid'
    return f'({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f})'
