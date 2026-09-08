from __future__ import annotations
"""帧栈（FrameCollector）：采帧状态机与自动模式决策。"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from peach_perception.common.tf_utils import relative_motion

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
    min_rotation_deg: float = 1.0     # [deg] 最小旋转
    # ── 自动模式（默认开；False 使用纯手动 Trigger 服务流）──
    auto_mode: bool = True            # 自动开始/采帧/完成总开关
    auto_finalize_at_max: bool = False  # 连续扫描默认由用户 finalize
    auto_min_interval_s: float = 0.0  # [s] 0=每个唯一时间戳均进入质量门


class FrameCollector:
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

        规则（跳过而非拒帧，保积分序）：首帧直采 → 可选间隔门 →
        近重复视角跳过积分 → 其余采集。
        转移中的超采由 capture_gate.require_robot_static 拦下；本函数见到
        相对上一已采帧偏大的位移，表示已经到了新机位，应当采集，不能当
        成「连续运动超上限」丢掉（旧视角过滤上限 80 mm 会把现场 12°/0.40 m
        环绕约 84 mm 的合法短移拒掉，captured_views 钉死在 1）。

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
