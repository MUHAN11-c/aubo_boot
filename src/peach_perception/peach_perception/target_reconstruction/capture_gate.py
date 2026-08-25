"""
采帧公共门禁纯函数（手动 ~/capture_frame 与自动 _try_auto_capture 共用）.

判据全部以值传入（满栈/无帧/掩膜/同帧/帧龄/静止/空 frame_id/TF 结果），
函数内零副作用、零 ROS 依赖。TF 查询本身有阻塞与日志副作用，留在节点：
门禁先以 tf_available=None 调用，前置全过时返回 GATE_NEED_TF，节点查完
TF 再以真实结果重评一次。手动模式拒绝（deny：写服务响应、按
count_reject 计 rejected_views），自动模式跳过（skip：调用方静默或
debug 日志），同一判据两种出口由 automatic 一个开关映射。
"""
from dataclasses import dataclass
from typing import Optional

GATE_ALLOW = 'allow'
GATE_DENY = 'deny'
GATE_SKIP = 'skip'
GATE_NEED_TF = 'need_tf'


@dataclass(frozen=True)
class GateDecision:
    """capture_gate 的判定结果（纯数据）."""

    action: str  # GATE_ALLOW / GATE_DENY / GATE_SKIP / GATE_NEED_TF
    reason: str = ''  # 拒绝/跳过原因（中文）；allow 与 need_tf 时为空
    count_reject: bool = True  # deny 时是否计 collector.rejected_views
    count_tf_failure: bool = False  # 是否计 collector.tf_failures


def capture_gate(
        *,
        frame_available: bool,
        frame_count: int,
        max_views: int,
        mask_reason: str,
        stamp_sec: float,
        last_captured_stamp_sec: float,
        frame_age_s: float,
        max_frame_age_s: float,
        require_robot_static: bool,
        joint_states_seen: bool,
        max_joint_vel: float,
        static_joint_vel_thresh: float,
        cam_frame_ok: bool,
        base_frame: str,
        cam_frame: str,
        tf_available: Optional[bool],
        automatic: bool) -> GateDecision:
    """
    按固定顺序评估采帧公共门禁（与重构前两路内联实现逐条对应）.

    顺序即优先级：满栈 → 无帧 → 掩膜 → 同帧 → 帧龄 → 静止 → 空
    frame_id → TF。前 7 项任一不过即定案；全过且 tf_available=None
    返回 GATE_NEED_TF，请调用方完成 TF 查询后以真实结果重评。

    Args:
        frame_available: 是否已有同步 RGB-D 缓存帧.
        frame_count: 已采帧数.
        max_views: 帧栈上限.
        mask_reason: 目标掩膜门禁原因（'' 表示通过或未启用）.
        stamp_sec: 缓存帧图像时间戳 [s].
        last_captured_stamp_sec: 上次成功采帧的图像时间戳 [s].
        frame_age_s: 缓存帧龄期 [s].
        max_frame_age_s: 允许的最大帧龄 [s].
        require_robot_static: 是否要求机器人静止.
        joint_states_seen: 是否已收到 /joint_states.
        max_joint_vel: 最近最大关节速度幅值 [rad/s].
        static_joint_vel_thresh: 静止判定阈值 [rad/s].
        cam_frame_ok: 深度图 header.frame_id 非空.
        base_frame: 机器人基座系名（仅用于 TF 失败消息）.
        cam_frame: 相机光学系名（仅用于 TF 失败消息）.
        tf_available: TF 查询结果；None 表示尚未查询.
        automatic: True=自动模式（失败映射 skip），False=手动服务
            （失败映射 deny）.

    Returns
    -------
        GateDecision；action 为 GATE_ALLOW 时 reason 为空.

    """
    def _fail(reason: str, count_reject: bool = True,
              count_tf_failure: bool = False) -> GateDecision:
        return GateDecision(
            action=GATE_SKIP if automatic else GATE_DENY,
            reason=reason,
            count_reject=count_reject,
            count_tf_failure=count_tf_failure)

    if frame_count >= max_views:
        return _fail(f'已达 max_views={max_views}，请 finalize 或 remove_last')
    if not frame_available:
        return _fail('尚无同步 RGB-D 帧（确认相机/回放在线）')
    if mask_reason:
        return _fail(mask_reason)
    if stamp_sec <= last_captured_stamp_sec:
        return _fail('缓存帧未更新（与上次采帧同帧），请等下一帧')
    if frame_age_s > max_frame_age_s:
        return _fail(
            f'缓存帧龄期 {frame_age_s:.2f} s > '
            f'max_frame_age_s={max_frame_age_s}（陈帧拒采）')
    if require_robot_static:
        if not joint_states_seen:
            return _fail('require_robot_static=true 但未收到 /joint_states')
        if max_joint_vel > static_joint_vel_thresh:
            return _fail(
                f'机器人未静止：最大关节速度 {max_joint_vel:.4f} rad/s '
                f'> {static_joint_vel_thresh}')
    if not cam_frame_ok:
        return _fail('深度图 header.frame_id 为空，无法查 TF',
                     count_reject=False)
    if tf_available is None:
        return GateDecision(action=GATE_NEED_TF)
    if not tf_available:
        return _fail(
            f'TF {base_frame}←{cam_frame} 查询失败（已计 tf_failures）',
            count_reject=False, count_tf_failure=True)
    return GateDecision(action=GATE_ALLOW)
