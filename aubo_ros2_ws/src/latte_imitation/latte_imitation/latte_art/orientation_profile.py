"""动态朝向剖面 — 为参数化轨迹生成每帧独立的奶缸倾倒姿态喵~

三阶段 roll 规律 (来自多源交叉验证):
  融合 [0~25%]:   roll 45°→30° 线性下降 (奶缸从高位大倾斜逐渐贴近液面)
  成形 [25~85%]:  roll 30°±3° 微变 (贴近液面, 奶泡浮于表面)
  收尾 [85~100%]: roll 30°→60° 线性上升 (奶缸上提, 细流 draw-through)

pitch = 0 (全程无侧倾, 液体不洒)
yaw = 0 (canonical frame, 后续 SE(3) retarget 统一变换)
"""

import numpy as np
from typing import Optional

from .config import CupConfig, PourConfig


def compute_roll_profile(
    total_frames: int,
    mix_end: int,
    draw_end: int,
    pour: Optional[PourConfig] = None,
) -> np.ndarray:
    """为每帧计算 roll 角 (度) — X 轴倾倒方向喵~

    Args:
        total_frames: 轨迹总帧数
        mix_end:      融合阶段结束帧索引 (exclusive)
        draw_end:     成形阶段结束帧索引 (exclusive)
        pour:         倾倒参数 (使用朝向剖面默认值), None 时用默认

    Returns:
        (total_frames,) float64 — 每帧的 roll 角 (度)
    """
    if pour is None:
        pour = PourConfig()

    roll = np.zeros(total_frames, dtype=np.float64)

    # ── 融合阶段 [0, mix_end): 45° → 30° ──
    if mix_end > 0:
        roll[:mix_end] = np.linspace(pour.mix_roll_start_deg,
                                      pour.mix_roll_end_deg, mix_end)

    # ── 成形阶段 [mix_end, draw_end): base ± variation 微变 ──
    draw_frames = max(1, draw_end - mix_end)
    t_draw = np.linspace(0.0, 1.0, draw_frames)
    roll[mix_end:draw_end] = (pour.draw_roll_base_deg +
        pour.draw_roll_variation_deg * np.sin(2.0 * np.pi * 0.5 * t_draw))

    # ── 收尾阶段 [draw_end, total_frames): 30° → 60° ──
    finish_frames = max(1, total_frames - draw_end)
    roll[draw_end:] = np.linspace(pour.finish_roll_start_deg,
                                   pour.finish_roll_end_deg, finish_frames)

    return roll


def assemble_cartesian_with_orientation(
    xyz: np.ndarray,
    roll_profile: np.ndarray,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    dt: float = 0.05,
    episode_idx: int = -1,
    frame_id: str = "base_link",
) -> "CartesianTrajectory":
    """将 XYZ 轨迹 + 动态 roll 剖面组装为 CartesianTrajectory 喵~

    每帧独立四元数: q[i] = euler_deg_to_quat(roll_profile[i], pitch_deg, yaw_deg)

    与 bridge.parametric_to_cartesian() 的区别:
      - 每帧 roll 由剖面决定 (非固定 45°)
      - 保留了原有的 pitch/yaw 参数入口
    """
    from latte_imitation.trajectory import CartesianTrajectory
    from latte_imitation.trajectory_transform import euler_deg_to_quat

    T = len(xyz)
    if len(roll_profile) != T:
        raise ValueError(
            f"roll_profile length ({len(roll_profile)}) != xyz length ({T})"
        )

    orientations = np.zeros((T, 4), dtype=np.float32)
    for i in range(T):
        q = euler_deg_to_quat(float(roll_profile[i]), pitch_deg, yaw_deg)
        orientations[i] = q.astype(np.float32)

    timestamps = np.arange(T, dtype=np.float32) * dt
    return CartesianTrajectory(
        positions=np.asarray(xyz, dtype=np.float32),
        orientations=orientations,
        timestamps=timestamps,
        dt=dt,
        episode_idx=episode_idx,
        frame_id=frame_id,
    )
