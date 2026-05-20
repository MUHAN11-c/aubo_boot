"""拉花图案参数配置 dataclasses — 纯 Python, 无 ROS 依赖喵~

来源: latte_art_robot_research/轨迹生成方案/latte_art_trajectory.py
理论: Sunergos Milk Training Video + latteartguide.com 多源交叉验证
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class CupConfig:
    """咖啡杯参数喵~

    所有坐标在机械臂 base_link 坐标系中喵~
    """
    center_x: float = 0.0       # 杯口中心 X (m)
    center_y: float = 0.0       # 杯口中心 Y (m)
    surface_z: float = 0.15     # 液面高度 Z (m)
    radius: float = 0.04        # 杯口半径 (m) — 标准拿铁杯约 8cm 直径喵~


@dataclass
class PourConfig:
    """倒奶拉花动作参数喵~

    所有高度均为相对于液面的偏移量喵~
    来源: Sunergos 视频原文数值 + latteartguide.com 交叉验证
    """
    # ── 三阶段高度 (液面相对偏移) ──
    mix_height_offset: float = 0.076     # 融合阶段: 液面上方 3英寸=7.6cm (视频原文)
    draw_height_offset: float = 0.006    # 成形阶段: 液面上方 1/4英寸=0.6cm (视频原文)
    finish_height_offset: float = 0.076  # 收尾阶段: 抬回 3英寸=7.6cm (draw-through)

    # ── 摆动参数 ──
    wiggle_amplitude: float = 0.006      # 摆动振幅 6mm (参考: 手腕微动→奶液钟摆)
    wiggle_frequency: float = 2.4        # 摆动频率 2.4Hz (40条心形数据速度域FFT中位数标定, IQR 1.2-5.1Hz) 喵~

    # ── 速度约束 (抗晃荡) ──
    max_velocity: float = 0.05           # 最大速度 5cm/s
    max_acceleration: float = 0.1        # 最大加速度
    max_jerk: float = 0.5                # 最大加加速度 (jerk)

    # ── 工具姿态 ──
    pour_pitch_deg: float = 45.0         # 倒奶俯仰角 (度), 奶缸前倾

    def mix_z(self, surface_z: float = 0.15) -> float:
        return surface_z + self.mix_height_offset

    def draw_z(self, surface_z: float = 0.15) -> float:
        return surface_z + self.draw_height_offset

    def finish_z(self, surface_z: float = 0.15) -> float:
        return surface_z + self.finish_height_offset
