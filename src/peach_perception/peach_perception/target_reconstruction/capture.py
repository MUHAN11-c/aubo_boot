from __future__ import annotations
"""采帧：门禁、帧栈、绑定防抖、自动采集。"""

from dataclasses import dataclass, field
import time
from typing import (
    Callable,
    List,
    Mapping,
    Optional,
    Tuple,
)

import numpy as np
from peach_perception.common.ema import ScalarEma
from peach_perception.common.geometry import relative_motion
from peach_perception.target_reconstruction.integrate import apply_target_mask
from peach_perception.target_reconstruction.interfaces import (
    FRAME_STORES,
    FrameStore,
    MASK_GATES,
    MaskGate,
)
from peach_perception.target_reconstruction.refine import candidate_axis_hint


# === captured_frame.py ===

@dataclass
class CapturedFrame:
    """一次成功采帧的全部内容（原始观测 + base 系几何 + 诊断标记）."""

    rgb: np.ndarray                      # (H, W, 3) uint8 BGR（OpenCV 惯例）
    depth_mm: np.ndarray                 # (H, W) uint16 深度 [mm]
    camera_K: dict                       # {"fx","fy","cx","cy","width","height"}
    stamp: float                         # 图像时间戳 [s]（按 depth.header.stamp）
    T_base_camera: np.ndarray            # (4, 4) base←camera 齐次矩阵
    T_base_camera_fk: Optional[np.ndarray] = None  # ICP 前的机器人 FK 位姿
    target_id: str = ''                  # 绑定的候选目标 ID（无候选时为空串）
    valid_depth_ratio: float = 0.0       # 有效深度占比 [0, 1]
    camera_position_base: Optional[np.ndarray] = None  # (3,) 相机位置 [m]
    cloud_base: Optional[np.ndarray] = None            # (N, 3) base 系点云 [m]
    cloud_rgb: Optional[np.ndarray] = None             # (N, 3) uint8 BGR（OpenCV 排列）
    diagnostic_flags: List[str] = field(default_factory=list)  # 如 'pose_icp'
    registration: dict = field(default_factory=dict)  # ICP fitness/RMSE/修正量

    def __post_init__(self):
        """补派生默认值：FK 缺省等于使用位姿，相机位置取使用位姿平移列."""
        if self.T_base_camera_fk is None:
            self.T_base_camera_fk = np.asarray(
                self.T_base_camera, dtype=np.float64).copy()
        if self.camera_position_base is None:
            self.camera_position_base = np.asarray(
                self.T_base_camera, dtype=np.float64)[:3, 3].copy()


# === skip_codes.py ===

# (子串, 短码) 顺序即优先级
_SKIP_PATTERNS = (
    ('缺少所选 target_id 的同时间戳掩膜', 'missing_mask'),
    ('目标掩膜仅', 'mask_pixels'),
    ('掩膜内有效深度占比', 'mask_depth_ratio'),
    ('目标漂移', 'target_drift'),
    ('邻近锁定目标锚点间距', 'neighbor_gap'),
    ('缓存帧龄期', 'stale_frame'),
    ('缓存帧未更新', 'same_stamp'),
    ('查询失败（已计 tf_failures）', 'tf_failure'),
    ('近重复视角', 'near_duplicate'),
    ('连续运动超上限', 'motion_jump'),
    ('尚无同步 RGB-D', 'no_frame'),
    ('已达 max_views', 'max_views'),
    ('机器人未静止', 'robot_not_static'),
    ('未收到 /joint_states', 'robot_not_static'),
    ('frame_id 为空', 'empty_frame_id'),
    ('缓存帧已更新', 'frame_changed'),
    ('配准拒帧', 'icp_reject'),
    ('间隔门', 'min_interval'),
    ('非精确时间 TF', 'tf_inexact'),
    ('TSDF 在线积分失败', 'tsdf_integrate'),
)


def classify_skip_reason(reason: str) -> str:
    """中文门禁原因 → 稳定短码；空串给 empty."""
    text = str(reason or '')
    if not text:
        return 'empty'
    for needle, code in _SKIP_PATTERNS:
        if needle in text:
            return code
    return 'other'


# === capture_gate.py ===

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


# === bind_holdoff.py ===

class BindSwitchHoldoff:
    """selected 切换防抖状态机（注入时钟；每消息 arbitrate 一次）."""

    # 动作枚举（字符串，节点按此分支；保持纯数据便于日志/测试断言）
    FOLLOW = 'follow'  # 直通跟随（无可毁会话 / 未偏离 / 无挂起）
    PEND = 'pend'      # 开始挂起（含改挂到另一个新 ID 重新记时）
    WAIT = 'wait'      # 挂起中未到期，维持旧绑定
    CANCEL = 'cancel'  # holdoff 内切回原 ID，取消挂起
    COMMIT = 'commit'  # 挂起到期，执行放弃重绑

    def __init__(self, holdoff_s: float = 2.0,
                 now: Callable[[], float] = time.monotonic):
        """
        注入挂起时长与时钟.

        Args:
            holdoff_s: 切换挂起时长 [s]；requested 须持续超过本时长才
                commit。0 等价于「下一条维持新 ID 的观测即到期」，近似
                旧版立即跟随行为。
            now: 单调时钟（秒），节点注入 RclpyClockAdapter.now（I3）.

        Returns
        -------
            无返回值（None）.

        """
        self.holdoff_s = float(holdoff_s)
        self._now = now
        self._pending_id: Optional[str] = None  # 挂起中的候选新 ID（含 ''）
        self._pending_since: float = 0.0        # 挂起起始时刻 [s]

    @property
    def pending_id(self) -> Optional[str]:
        """当前挂起的新 ID；None 表示无挂起（诊断/测试观察口）."""
        return self._pending_id

    def arbitrate(self, requested_id: str, bound_id: str,
                  session_active: bool) -> str:
        """
        对一条观测的 selected_target_id 做防抖仲裁.

        Args:
            requested_id: 本帧感知全局计划 selected_target_id（可为 ''）.
            bound_id: 节点当前跟随的目标 ID（_preferred_target_id；空串
                表示尚未跟随任何目标）.
            session_active: 是否有进行中会话（collector.state != IDLE）；
                False 时无会话可毁，一切变化立即 follow 并清挂起.

        Returns
        -------
            动作：FOLLOW / PEND / WAIT / CANCEL / COMMIT（见类常量）.

        """
        if requested_id == bound_id:
            # 切回（或本就未偏离）绑定 ID：取消挂起；无挂起即常规直通
            if self._pending_id is None:
                return self.FOLLOW
            self._pending_id = None
            return self.CANCEL if session_active else self.FOLLOW
        if not session_active or not bound_id:
            # IDLE（无会话可毁）或从未跟随任何目标：立即跟随，不挂起
            self._pending_id = None
            return self.FOLLOW
        # requested 偏离绑定 ID 且有进行中会话：进入/维持挂起
        if self._pending_id != requested_id:
            # 新候选（含从某个挂起 ID 改挂到另一个）：重新记时
            self._pending_id = requested_id
            self._pending_since = self._now()
            return self.PEND
        if self._now() - self._pending_since >= self.holdoff_s:
            # 同一新 ID 持续超过挂起时长：到期，执行放弃重绑
            self._pending_id = None
            return self.COMMIT
        return self.WAIT


# === timing.py ===

# EMA 平滑系数（新样本权重）；0.3 在响应速度与抗单帧抖动间取折中
EMA_ALPHA = 0.3


class TimingStats:
    """重建流水线耗时累计器（分项 EMA + 单次 last 值；纯数值，零 ROS）."""

    def __init__(self):
        """清零：EMA 分项未播种（快照投影 0.0），计数为 0."""
        self._icp_ms = ScalarEma(EMA_ALPHA)
        self._tsdf_integrate_ms = ScalarEma(EMA_ALPHA)
        self._frame_total_ms = ScalarEma(EMA_ALPHA)
        self._refit_ms_last = 0.0
        self._finalize_ms_last = 0.0
        self._frames_timed = 0

    def record_icp(self, sample_ms: float) -> None:
        """记录一次 ICP refine 耗时 [ms]（每帧至多一次，拒帧也计入）."""
        self._icp_ms.update(max(0.0, float(sample_ms)))

    def record_tsdf_integrate(self, sample_ms: float) -> None:
        """记录一次 TSDF 在线积分+产物刷新耗时 [ms]（仅积分成功路径）."""
        self._tsdf_integrate_ms.update(max(0.0, float(sample_ms)))

    def record_frame_total(self, sample_ms: float) -> None:
        """记录一次成功采帧的 _accept_frame 总耗时 [ms]，并递增计数."""
        self._frame_total_ms.update(max(0.0, float(sample_ms)))
        self._frames_timed += 1

    def record_refit(self, sample_ms: float) -> None:
        """记录最近一次 refit 调用耗时 [ms]（last 值，含失败路径）."""
        self._refit_ms_last = max(0.0, float(sample_ms))

    def record_finalize(self, sample_ms: float) -> None:
        """记录最近一次 _finalize_now 总耗时 [ms]（last 值，含失败路径）."""
        self._finalize_ms_last = max(0.0, float(sample_ms))

    @staticmethod
    def _project(ema: ScalarEma) -> float:
        """内部 EMA → 快照标量：未播种投影为 0.0."""
        return 0.0 if not ema.seeded else float(ema.value)

    def snapshot(self) -> dict:
        """
        投影诊断 timing 子对象（JSON 可序列化，键集恒定）.

        Returns
        -------
            六个契约键的 dict（见模块 docstring）；全部数值非负，
            frames_timed=0 表示尚无成功采帧计时样本.

        """
        return {
            'icp_ms_ema': self._project(self._icp_ms),
            'tsdf_integrate_ms_ema': self._project(self._tsdf_integrate_ms),
            'frame_total_ms_ema': self._project(self._frame_total_ms),
            'refit_ms_last': float(self._refit_ms_last),
            'finalize_ms_last': float(self._finalize_ms_last),
            'frames_timed': int(self._frames_timed),
        }


# === frame_collector.py ===

STATE_IDLE = 'IDLE'
STATE_COLLECTING = 'COLLECTING'
STATE_READY = 'READY'


@dataclass
class CollectorConfig:
    """采帧与视角过滤配置（平移 [m]，旋转 [deg]，间隔 [s]）."""

    min_views: int = 2                # finalize 所需最少机位数
    recommended_views: int = 5        # 推荐视角数（不足仅提示）
    max_views: int = 8                # 帧栈上限
    min_translation: float = 0.002    # [m] 与上一帧最小平移（低于=近重复）
    max_translation: float = 0.080    # [m] 与上一帧最大平移（高于=跳变）
    min_rotation_deg: float = 1.0     # [deg] 最小旋转
    max_rotation_deg: float = 25.0    # [deg] 最大旋转
    allow_duplicate_views: bool = True  # True 时重复视角仅告警不拒帧
    # ── 自动模式（默认开；False 使用纯手动 Trigger 服务流）──
    auto_mode: bool = True            # 自动开始/采帧/完成总开关
    auto_finalize_at_max: bool = False  # 连续扫描默认由用户 finalize
    auto_min_interval_s: float = 0.0  # [s] 0=每个唯一时间戳均进入质量门


class FrameCollector(FrameStore):
    """
    采帧流程的纯逻辑核心：状态机 + 视角过滤 + CapturedFrame 帧栈.

    ROS 侧的门禁（帧新鲜度 / 机器人静止 / TF 查询）在节点里做；
    本类只管「该不该收、收了放哪、什么时候算完」。
    """

    def __init__(self, config: Optional[CollectorConfig] = None):
        """
        构造收集器.

        Args:
            config: 采帧配置；None 用 CollectorConfig 默认值.

        Returns
        -------
            无返回值（None）；初始状态 IDLE.

        """
        self.config = config or CollectorConfig()
        self.reset()

    def reset(self) -> None:
        """清空全部帧 / 绑定目标 / 计数，状态回 IDLE."""
        self.state = STATE_IDLE
        self.frames: List = []
        self.target_id = ''
        self.target_center: Optional[np.ndarray] = None
        self.rejected_views = 0
        self.tf_failures = 0
        self.skip_reasons: dict = {}
        self.last_skip_code = ''
        self.last_skip_reason = ''
        self.last_rel_translation_m: Optional[float] = None
        self.last_rel_rotation_deg: Optional[float] = None

    def note_skip(self, code: str, reason: str = '') -> None:
        """按原因码累计一次跳过（自动 skip 与手动 deny 共用）."""
        key = str(code or 'other')
        self.skip_reasons[key] = int(self.skip_reasons.get(key, 0)) + 1
        self.last_skip_code = key
        self.last_skip_reason = str(reason or '')

    @property
    def skipped_views(self) -> int:
        """全部门禁跳过次数（含同戳掩膜缺失等自动 skip）."""
        return int(sum(self.skip_reasons.values()))

    def start(self, target_id: str = '', target_center=None) -> str:
        """
        清空旧帧并进入 COLLECTING，绑定当前最优候选目标.

        Args:
            target_id: 绑定候选的 target_id；空串表示未绑定.
            target_center: (3,) 目标中心（base 系 [m]）；None 表示未知.

        Returns
        -------
            状态说明字符串（作服务响应 message）.

        """
        center = None
        if target_center is not None:
            center = np.asarray(target_center, dtype=np.float64)
        self.reset()
        self.state = STATE_COLLECTING
        self.target_id = target_id or ''
        self.target_center = center
        if self.target_id:
            return f'开始重建，绑定目标 {self.target_id}'
        return '开始重建（当前无候选，未绑定目标）'

    def check_view(self, T_base_camera: np.ndarray
                   ) -> Tuple[bool, str, Optional[float], Optional[float]]:
        """
        视角过滤：本帧位姿与上一已采帧的相对运动检查.

        规则：平移与旋转**同时**低于下限 = 重复视角；任一**高于**上限 = 跳变。
        重复视角在 allow_duplicate_views=True 时放行（reason='duplicate_allowed'，
        由调用方告警）；跳变恒拒。首帧不检查。

        Args:
            T_base_camera: (4, 4) 本帧 base←camera 位姿.

        Returns
        -------
            (ok, reason, rel_translation_m, rel_rotation_deg)；首帧后两个为 None.

        """
        if not self.frames:
            return True, 'first_frame', None, None
        trans, rot = relative_motion(T_base_camera, self.frames[-1].T_base_camera)
        self.last_rel_translation_m = trans
        self.last_rel_rotation_deg = rot
        if (trans > self.config.max_translation
                or rot > self.config.max_rotation_deg):
            return False, (f'视角跳变过大：平移 {trans * 1000.0:.1f} mm / '
                           f'旋转 {rot:.1f} deg 超上限'), trans, rot
        if (trans < self.config.min_translation
                and rot < self.config.min_rotation_deg):
            if self.config.allow_duplicate_views:
                return True, 'duplicate_allowed', trans, rot
            return False, (f'与上一帧视角过近：平移 {trans * 1000.0:.1f} mm / '
                           f'旋转 {rot:.1f} deg 低于下限'), trans, rot
        return True, 'ok', trans, rot

    # ------------------------------------------------------------------
    # 自动模式决策（纯逻辑，节点只做 TF/订阅接线）
    # ------------------------------------------------------------------
    def should_auto_start(self) -> bool:
        """
        是否允许自动开始（auto_mode 开且 IDLE；候选有无由节点判断）.

        Returns
        -------
            bool.

        """
        return self.config.auto_mode and self.state == STATE_IDLE

    def should_auto_finalize(self) -> bool:
        """
        是否触发自动完成：auto 双开关开且 COLLECTING 且满 max_views.

        Returns
        -------
            bool.

        """
        return (self.config.auto_mode and self.config.auto_finalize_at_max
                and self.state == STATE_COLLECTING
                and len(self.frames) >= self.config.max_views)

    def auto_capture_decision(self, T_base_camera: np.ndarray,
                              since_last_capture_s: float
                              ) -> Tuple[str, str]:
        """
        自动采帧决策（纯逻辑）.

        规则（与手动 check_view 的严格拒帧不同，自动模式以「跳过」代替拒绝）：
        首帧直采 → 可选间隔门 → 近重复视角跳过积分 → 其余采集。
        转移中的超采由 capture_gate.require_robot_static 拦下；本函数见到
        相对上一已采帧偏大的位移，表示已经到了新机位，应当采集，不能当
        成「连续运动超上限」丢掉（现场 12°/0.40 m 环绕约 84 mm，大于旧的
        max_translation=80 mm，会把 captured_views 钉死在 1）。

        Args:
            T_base_camera: (4, 4) 本帧 base←camera 位姿.
            since_last_capture_s: 距上次成功采帧的时间 [s]（首帧传 inf 即可）.

        Returns
        -------
            (action, reason)：action ∈ {'capture', 'skip'}；
            auto_mode=False 时恒 ('skip', ...)，回纯手动.

        """
        if not self.config.auto_mode:
            return 'skip', 'auto_mode=false（纯手动模式）'
        if not self.frames:
            return 'capture', '首帧直采'
        if since_last_capture_s < self.config.auto_min_interval_s:
            return 'skip', (f'间隔门：{since_last_capture_s:.2f} s < '
                            f'auto_min_interval_s='
                            f'{self.config.auto_min_interval_s}')
        trans, rot = relative_motion(T_base_camera,
                                     self.frames[-1].T_base_camera)
        self.last_rel_translation_m = trans
        self.last_rel_rotation_deg = rot
        moved = (trans >= self.config.min_translation
                 or rot >= self.config.min_rotation_deg)
        if not moved:
            return 'skip', (
                f'近重复视角不积分：平移 {trans * 1000.0:.1f} mm / '
                f'旋转 {rot:.2f} deg')
        return 'capture', (
            f'新机位：平移 {trans * 1000.0:.1f} mm / 旋转 {rot:.2f} deg')

    def add_frame(self, frame) -> bool:
        """
        压栈一帧；已达 max_views 拒收.

        Args:
            frame: CapturedFrame.

        Returns
        -------
            是否成功入栈.

        """
        if len(self.frames) >= self.config.max_views:
            return False
        self.frames.append(frame)
        return True

    def remove_last(self):
        """
        弹出最后一帧.

        Returns
        -------
            被弹出的 CapturedFrame；空栈返回 None.

        """
        if not self.frames:
            return None
        return self.frames.pop()

    def accumulated_cloud(self) -> np.ndarray:
        """
        拼接全部已采帧的 base 系点云.

        Returns
        -------
            (M, 3) float64 点云 [m]；无帧时给 (0, 3) 空数组.

        """
        clouds = [f.cloud_base for f in self.frames
                  if f.cloud_base is not None and f.cloud_base.size]
        if not clouds:
            return np.zeros((0, 3), dtype=np.float64)
        return np.vstack(clouds)

    def accumulated_rgb(self) -> Optional[np.ndarray]:
        """
        拼接全部已采帧的颜色通道（与 accumulated_cloud 逐点对应）.

        Returns
        -------
            (M, 3) uint8 BGR 颜色；任一帧缺颜色或无帧时给 None
            （调用方回退纯 xyz 发布）.

        """
        if not self.frames:
            return None
        if any(f.cloud_rgb is None for f in self.frames):
            return None
        colors = [f.cloud_rgb for f in self.frames if f.cloud_rgb.size]
        if not colors:
            return None
        return np.vstack(colors)

    def finalize(self) -> Tuple[bool, str, Optional[np.ndarray]]:
        """
        结束采集：帧数达标则拼接累加云并转 READY.

        独立机位门由节点 ``_wait_min_views`` / ``_finalize_now`` 判定.
        本函数只保证帧栈不少于 min_views，避免空云转 READY.

        Returns
        -------
            (ok, message, cloud_or_None)：不足 min_views 帧时 ok=False 且
            保持 COLLECTING.

        """
        if self.state != STATE_COLLECTING:
            return False, (f'当前状态 {self.state} 不能 finalize'
                           '（需 COLLECTING）'), None
        n = len(self.frames)
        if n < self.config.min_views:
            return False, (f'已采 {n} 帧 < min_views={self.config.min_views}，'
                           '继续采帧或 reset'), None
        cloud = self.accumulated_cloud()
        if cloud is None or int(cloud.shape[0]) == 0:
            return False, (
                f'已采 {n} 帧但累加云为空，保持 COLLECTING'), None
        self.state = STATE_READY
        msg = f'局部重建完成：{n} 帧，{cloud.shape[0]} 点'
        if n < self.config.recommended_views:
            msg += f'（少于推荐 {self.config.recommended_views} 帧）'
        return True, msg, cloud


# 显式注册清单（2.14）：注册名 'default'（yaml frame_store.impl）
FRAME_STORES.register('default', FrameCollector)


# === mask_gate.py ===

@dataclass(frozen=True)
class MaskContext:
    """
    一帧的掩膜门判据（纯数据，由节点按缓存帧组装）.

    Attributes
    ----------
        stamp_ns: 当前帧图像时间戳 [ns]（深度图 header.stamp）.
        depth_mm: (H, W) uint16 深度 [mm].
        masks: 掩膜缓存 {stamp_ns: (mask, center)}；mask 为 (H, W) uint8
            mono8，center 为 (3,) 目标中心（base 系 [m]）或 None.
        bound_center: (3,) 绑定目标中心（base 系 [m]）或 None（漂移门/
            邻目标门参照；collector.target_center）.
        neighbor_centers: 其他锁定目标锚点中心元组（(3,) base 系 [m]，
            已剔除绑定目标自身；E2 邻目标串扰门数据源，缺省空元组 =
            无邻目标，本门不启用）.

    """

    stamp_ns: int
    depth_mm: np.ndarray
    masks: Mapping[int, Tuple[np.ndarray, Optional[np.ndarray]]]
    bound_center: Optional[np.ndarray]
    neighbor_centers: Tuple[np.ndarray, ...] = field(default=())
    # 检测框面积（像素²，>0 有效）：绑定目标与各邻居并行携带。串扰门按
    # 面积比豁免「远小于绑定目标」的邻居框（叶片遮挡残片/误检，09-01）。
    bound_area: float = 0.0
    neighbor_areas: Tuple[float, ...] = field(default=())


@dataclass(frozen=True)
class GateResult:
    """
    掩膜门判定结果（纯数据）.

    Attributes
    ----------
        mask: 通过时为可用掩膜（(H, W) uint8）；门禁未启用或拒绝时 None.
        reason: 拒绝原因（中文）；空串表示通过（或门禁未启用）.

    """

    mask: Optional[np.ndarray]
    reason: str = ''

    @property
    def passed(self) -> bool:
        """Reason 为空即通过（含门禁未启用的直通情形）."""
        return not self.reason


class StrictMaskGate(MaskGate):
    """
    interfaces.MaskGate 的默认实现：五道门全过的严格掩膜门（无状态）.

    配置经构造注入（与 capture.* 参数一一对应）；require_target_mask
    为 False 时直通（返回 (None, '')，与抽取前语义一致）。
    """

    def __init__(self, require_target_mask: bool = True,
                 min_mask_pixels: int = 300,
                 min_mask_depth_ratio: float = 0.35,
                 max_target_drift_m: float = 0.04,
                 min_neighbor_gap_m: float = 0.15,
                 neighbor_gap_area_ratio: float = 2.0):
        """
        注入六道门配置（值与 capture.* 参数一致）.

        Args:
            require_target_mask: False 时掩膜门整体直通.
            min_mask_pixels: 目标掩膜最少像素数.
            min_mask_depth_ratio: 掩膜内有效深度占比下限.
            max_target_drift_m: 目标中心最大漂移 [m].
            min_neighbor_gap_m: 绑定锚点与其他锁定目标锚点的最小间距
                [m]（E2 串扰门）；≤0 关闭本门.
            neighbor_gap_area_ratio: 串扰门小框豁免比（≤0 关闭豁免）：
                邻居检测框面积 < 绑定框面积/本值时视为遮挡残片/误检，
                不计入串扰间距（近距双检常见同一颗袋的大框+残片小框，
                不豁免则两颗互相锁死，09-01 现场 58.5 mm 即此类）.

        Returns
        -------
            无返回值（None）.

        """
        self.require_target_mask = bool(require_target_mask)
        self.min_mask_pixels = int(min_mask_pixels)
        self.min_mask_depth_ratio = float(min_mask_depth_ratio)
        self.max_target_drift_m = float(max_target_drift_m)
        self.min_neighbor_gap_m = float(min_neighbor_gap_m)
        self.neighbor_gap_area_ratio = float(neighbor_gap_area_ratio)

    def check(self, mask_ctx: MaskContext) -> GateResult:
        """
        按固定顺序评估五道掩膜门（前四道与抽取前内联实现逐条对应）.

        Args:
            mask_ctx: 一帧的判据（时间戳/深度/掩膜缓存/绑定中心/邻目标
                锚点）.

        Returns
        -------
            GateResult；passed 时 mask 为可用掩膜（门禁关闭时为 None）.

        """
        if not self.require_target_mask:
            return GateResult(None, '')
        entry = mask_ctx.masks.get(mask_ctx.stamp_ns)
        if entry is None:
            return GateResult(None, '缺少所选 target_id 的同时间戳掩膜')
        mask, center = entry
        pixels = int(np.count_nonzero(mask))
        if pixels < self.min_mask_pixels:
            return GateResult(
                None,
                f'目标掩膜仅 {pixels} 像素 < {self.min_mask_pixels}')
        try:
            _masked, ratio = apply_target_mask(mask_ctx.depth_mm, mask)
        except ValueError as exc:
            return GateResult(None, str(exc))
        if ratio < self.min_mask_depth_ratio:
            return GateResult(
                None,
                f'掩膜内有效深度占比 {ratio:.2f} < '
                f'{self.min_mask_depth_ratio:.2f}')
        bound = mask_ctx.bound_center
        if center is not None and bound is not None:
            drift = float(np.linalg.norm(center - np.asarray(bound)))
            if drift > self.max_target_drift_m:
                return GateResult(
                    None,
                    f'目标漂移 {drift * 1000.0:.1f} mm > '
                    f'{self.max_target_drift_m * 1000.0:.1f} mm')
        # 门 5（E2 邻目标串扰）：绑定锚点与其他锁定目标锚点过近时拒帧。
        # 邻近目标的掩膜/点云会局部落入本目标 ROI，混入在线 TSDF 后形成
        # 不可回滚双层表面（I6）；TSDF 无单帧撤销，宁可停采等视角拉开。
        # 小框豁免：面积远小于绑定框的邻居（叶片遮挡残片/误检）不计入
        # 间距——近距双检常见同一颗袋的大框+残片小框，不豁免则互相锁死。
        if bound is not None and self.min_neighbor_gap_m > 0.0:
            bound_arr = np.asarray(bound)
            areas = mask_ctx.neighbor_areas
            effective = []
            for idx, c in enumerate(mask_ctx.neighbor_centers):
                neighbor_area = (
                    float(areas[idx]) if idx < len(areas) else 0.0)
                if (self.neighbor_gap_area_ratio > 0.0
                        and mask_ctx.bound_area > 0.0
                        and neighbor_area > 0.0
                        and neighbor_area * self.neighbor_gap_area_ratio
                        < mask_ctx.bound_area):
                    continue
                effective.append(c)
            gaps = [float(np.linalg.norm(np.asarray(c) - bound_arr))
                    for c in effective]
            if gaps:
                nearest = min(gaps)
                if nearest < self.min_neighbor_gap_m:
                    return GateResult(
                        None,
                        f'邻近锁定目标锚点间距 {nearest * 1000.0:.1f} mm < '
                        f'{self.min_neighbor_gap_m * 1000.0:.1f} mm'
                        f'（防串扰拒帧，I6）')
        return GateResult(mask, '')


# 显式注册清单（2.14）：注册名 'strict_mask_gate'，yaml mask_gate.impl 默认值
MASK_GATES.register('strict_mask_gate', StrictMaskGate)


# === auto_controller.py ===

class AutoControllerMixin:
    """自动状态机驱动方法集（宿主契约见模块 docstring；不自带 __init__）."""

    def _auto_drive(self):
        """
        自动模式驱动：每个新同步帧回调末尾调用一次.

        流程：IDLE 且有候选 → 自动开始；COLLECTING → 满 max_views 自动
        finalize，否则尝试自动采帧；READY/FAILED 停采，等 reset/start 进
        下一轮。所有"不行"都只对当前帧跳过/告警，不打断流程。
        并发收敛：本方法可由 worker 线程（_process_rgbd）与 executor 线程
        （_on_target_observations）并发进入；collector/TSDF/产物读写全程
        持 _state_lock（RLock 允许锁内嵌套调 _finalize_now）。唯一例外是
        采帧的阻塞式 TF 查询：锁内 begin 采集判据 → 锁外查询（最长
        tf_timeout，期间服务/心跳可取锁）→ 锁内 finish 按 stamp 复核收口
        （见 _gated_capture_finish 竞态说明）。
        """
        with self._state_lock:
            if self.collector.state == STATE_IDLE and \
                    self.collector.should_auto_start():
                self._auto_start()
            if self.collector.state != STATE_COLLECTING:
                return
            if self.collector.should_auto_finalize():
                ok, message = self._finalize_now()
                if ok:
                    self.get_logger().info(f'自动完成：{message}')
                return
            if len(self.collector.frames) >= self.params.capture.max_views:
                # 满栈后静默等待 finalize，避免每帧重复构云/ICP和刷屏
                return
            decision, tf_request = self._gated_capture_begin(automatic=True)
        if tf_request is None:
            # 前置门禁已定案（skip），无需 TF 查询。
            if decision.reason:
                self._record_auto_skip(
                    decision.reason,
                    count_reject=decision.count_reject,
                    count_tf_failure=decision.count_tf_failure)
            return
        # 锁外：阻塞式 TF 查询（不得持 _state_lock）。
        tf_result = self._gated_capture_query_tf(tf_request)
        with self._state_lock:
            decision, context = self._gated_capture_finish(
                automatic=True, tf_request=tf_request, tf_result=tf_result)
            self._auto_capture_commit(decision, context)

    def _auto_start(self):
        """自动开始：绑定当前最优候选进 COLLECTING；无候选则保持 IDLE 等待."""
        target_id, center = self._best_candidate()
        if not target_id:
            self.get_logger().debug(
                '自动开始：无 initial_pose 候选，保持 IDLE')
            return
        message = self.collector.start(target_id, center)
        self._target_kind_memory.bind(target_id)
        self._last_captured_stamp_sec = -1.0
        self._reset_products(create_volume=True)
        self._bound_axis_hint = candidate_axis_hint(
            self._latest_candidates, target_id)
        self.get_logger().info(f'自动开始：{message}')
        self._publish_all()

    def _auto_capture_commit(self, decision, context) -> None:
        """
        自动采帧落地段（须持 _state_lock）：门禁结果 → 间隔/视角决策 → 建云.

        decision 非 GATE_ALLOW 即按 skip 跳过（按需计 tf_failures）；
        context 为 ALLOW 时的帧上下文（见 _gated_capture_finish）。
        """
        if decision.action != GATE_ALLOW:
            if decision.reason:
                self._record_auto_skip(
                    decision.reason,
                    count_reject=decision.count_reject,
                    count_tf_failure=decision.count_tf_failure)
            elif decision.count_tf_failure:
                self.collector.tf_failures += 1
            return
        (rgb, depth_mm, K, stamp_sec,
         T_base_camera, tf_status, target_mask) = context
        if self._last_captured_stamp_sec > 0.0:
            since_last = stamp_sec - self._last_captured_stamp_sec
        else:
            since_last = float('inf')  # 首帧不受间隔门限制
        action, reason = self.collector.auto_capture_decision(
            T_base_camera, since_last)
        if action != 'capture':
            self._record_auto_skip(reason)
            return
        accepted, message = self._accept_frame(
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
            target_mask=target_mask)
        if not accepted:
            self.collector.rejected_views += 1
            self._record_auto_skip(message, count_reject=False)
            self.get_logger().warning(f'自动采帧未入库：{message}')

    def _record_auto_skip(
            self, reason: str, *, count_reject: bool = False,
            count_tf_failure: bool = False) -> None:
        """自动 skip 落账：原因码计数 + 节流 WARN + harvest_data 事件."""
        code = classify_skip_reason(reason)
        self.collector.note_skip(code, reason)
        if count_tf_failure:
            self.collector.tf_failures += 1
        if count_reject:
            self.collector.rejected_views += 1
        self.get_logger().warning(
            f'自动采帧跳过 [{code}]：{reason}',
            throttle_duration_sec=1.0)
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'frame_skipped',
            'target_id': self.collector.target_id,
            'code': code, 'reason': reason,
        })
