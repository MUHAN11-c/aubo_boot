"""
FrameCollector — 采帧状态机 + 视角过滤 + 帧栈管理（纯 Python，不依赖 ROS）.

状态机：IDLE → COLLECTING → READY（FAILED 预留给后续 Phase 的失败路径）。
视角过滤：本帧与上一已采帧比较相对平移 [m] / 旋转 [deg]——两者同时低于
下限视为重复视角，任一高于上限视为跳变（Move-Stop-Capture 应小幅移动）。
自动模式决策（should_auto_start / auto_capture_decision /
should_auto_finalize）同为纯函数放本模块，节点只做 TF/订阅接线。
"""
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from peach_reconstruction_ros2.tf_utils import relative_motion

STATE_IDLE = 'IDLE'
STATE_COLLECTING = 'COLLECTING'
STATE_READY = 'READY'
STATE_FAILED = 'FAILED'


@dataclass
class CollectorConfig:
    """采帧与视角过滤配置（平移 [m]，旋转 [deg]，间隔 [s]）."""

    min_views: int = 4                # finalize 所需最少视角数
    recommended_views: int = 5        # 推荐视角数（不足仅提示）
    max_views: int = 8                # 帧栈上限
    min_translation: float = 0.020    # [m] 与上一帧最小平移（低于=重复视角）
    max_translation: float = 0.080    # [m] 与上一帧最大平移（高于=跳变）
    min_rotation_deg: float = 5.0     # [deg] 最小旋转
    max_rotation_deg: float = 25.0    # [deg] 最大旋转
    allow_duplicate_views: bool = False  # True 时重复视角仅告警不拒帧
    # ── 自动模式（默认开；False 回 Phase 2 纯手动服务流）──
    auto_mode: bool = True            # 自动开始/采帧/完成总开关
    auto_finalize_at_max: bool = True  # 采满 max_views 自动 finalize
    auto_min_interval_s: float = 2.0  # [s] 两次自动采帧最小间隔


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
        self.last_rel_translation_m: Optional[float] = None
        self.last_rel_rotation_deg: Optional[float] = None

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
        首帧直采 → 间隔门（距上次 < auto_min_interval_s 跳过）→ 平移 ≥
        min_translation **或** 旋转 ≥ min_rotation_deg 即采；超上限
        （max_translation/max_rotation_deg）只告警照采（'warn_capture'）。

        Args:
            T_base_camera: (4, 4) 本帧 base←camera 位姿.
            since_last_capture_s: 距上次成功采帧的时间 [s]（首帧传 inf 即可）.

        Returns
        -------
            (action, reason)：action ∈ {'capture', 'warn_capture', 'skip'}；
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
            return 'skip', (f'视角未达阈：平移 {trans * 1000.0:.1f} mm / '
                            f'旋转 {rot:.2f} deg')
        if (trans > self.config.max_translation
                or rot > self.config.max_rotation_deg):
            return 'warn_capture', (f'连续运动超上限：平移 '
                                    f'{trans * 1000.0:.1f} mm / 旋转 '
                                    f'{rot:.2f} deg（自动模式只告警仍采帧）')
        return 'capture', 'ok'

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
        结束采集：视角数达标则拼接累加云并转 READY.

        Returns
        -------
            (ok, message, cloud_or_None)：不足 min_views 时 ok=False 且
            保持 COLLECTING.

        """
        if self.state != STATE_COLLECTING:
            return False, (f'当前状态 {self.state} 不能 finalize'
                           '（需 COLLECTING）'), None
        n = len(self.frames)
        if n < self.config.min_views:
            return False, (f'已采 {n} 视角 < min_views={self.config.min_views}，'
                           '继续采帧或 reset'), None
        cloud = self.accumulated_cloud()
        self.state = STATE_READY
        msg = f'局部重建完成：{n} 视角，{cloud.shape[0]} 点'
        if n < self.config.recommended_views:
            msg += f'（少于推荐 {self.config.recommended_views} 视角）'
        return True, msg, cloud
