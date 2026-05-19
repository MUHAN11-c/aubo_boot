#!/usr/bin/env python3
"""
ROS2 机械臂咖啡拉花轨迹生成器
=================================
将拉花图案（心形、树叶、郁金香）参数化为 3D 笛卡尔轨迹，
输出为 MoveIt2 可用的 Pose 序列。

使用方法:
    python latte_art_trajectory.py --pattern heart --output waypoints.csv

支持图案: heart, rosetta, tulip, swan, custom
"""

import numpy as np
from scipy.interpolate import CubicSpline, splprep, splev
from dataclasses import dataclass
from typing import List, Tuple, Optional
import argparse
import csv


# ═══════════════════════════════════════════════════════════════
# 配置参数
# ═══════════════════════════════════════════════════════════════

@dataclass
class CupConfig:
    """咖啡杯参数"""
    center_x: float = 0.0      # 杯口中心 X (m)
    center_y: float = 0.0      # 杯口中心 Y (m)
    surface_z: float = 0.15    # 液面高度 Z (m)
    radius: float = 0.04       # 杯口半径 (m) — 标准拿铁杯约 8cm 直径


@dataclass
class PourConfig:
    """倒奶拉花动作参数"""
    mix_height_offset: float = 0.076    # 融合阶段: 液面上方 3英寸 = 7.6cm (视频原文)
    draw_height_offset: float = 0.006   # 成形阶段: 液面上方 1/4英寸 = 0.6cm (视频原文)
    finish_height_offset: float = 0.076 # 收尾阶段: 抬回 3英寸 = 7.6cm (视频原文 draw-through)

    # Rosetta 摆动参数
    wiggle_amplitude: float = 0.006     # 摆动振幅 6mm
    wiggle_frequency: float = 5.0       # 摆动频率 5Hz

    # 速度约束 (抗晃荡)
    max_velocity: float = 0.05          # 最大速度 5cm/s
    max_acceleration: float = 0.1       # 最大加速度
    max_jerk: float = 0.5               # 最大加加速度

    @property
    def mix_z(self) -> float:
        return 0.15 + self.mix_height_offset  # 简化; 实际需传入 CupConfig

    @property
    def draw_z(self) -> float:
        return 0.15 + self.draw_height_offset

    @property
    def finish_z(self) -> float:
        return 0.15 + self.finish_height_offset


# ═══════════════════════════════════════════════════════════════
# 拉花图案轨迹生成器
# ═══════════════════════════════════════════════════════════════

class LatteArtTrajectory:
    """生成各类拉花图案的 3D 末端执行器轨迹"""

    def __init__(self, cup: CupConfig, pour: PourConfig):
        self.cup = cup
        self.pour = pour

    # ── 心形 ──────────────────────────────────────────

    def heart(self, num_points: int = 200) -> np.ndarray:
        """
        心形拉花轨迹 — 五步法 (基于 latteartguide.com + Sunergos)
        Step 3: 固定中心点左右摇摆→圆形白色底
        Step 4: 竖直抬起→"吸力让圆形顶部形成弧线"→心形顶部
        Step 5: 划穿→完成
        """
        t = np.linspace(0, 1, num_points)
        mix_end = int(num_points * 0.25)
        sway_end = int(num_points * 0.70)
        lift_end = int(num_points * 0.85)

        x = np.full(num_points, self.cup.center_x)
        y = np.full(num_points, self.cup.center_y + self.cup.radius * 0.3)
        z = np.full(num_points, self.pour.mix_z)

        # 摇摆阶段: 固定中心点，轻微左右摆动形成圆形
        t_sway = np.linspace(0, 2 * np.pi * 3, sway_end - mix_end)
        x[mix_end:sway_end] = self.cup.center_x + self.pour.wiggle_amplitude * 0.3 * np.sin(t_sway)
        y[mix_end:sway_end] = self.cup.center_y + self.cup.radius * 0.15
        z[mix_end:sway_end] = self.pour.draw_z

        # 竖直抬起阶段 (关键: 抬升产生吸力, 圆形顶部形成弧线)
        z[sway_end:lift_end] = np.linspace(self.pour.draw_z, self.pour.finish_z, lift_end - sway_end)
        x[sway_end:lift_end] = self.cup.center_x
        y[sway_end:lift_end] = self.cup.center_y + self.cup.radius * 0.15

        # 划穿收尾
        x[lift_end:] = self.cup.center_x
        y[lift_end:] = np.linspace(
            self.cup.center_y + self.cup.radius * 0.15,
            self.cup.center_y - self.cup.radius * 0.2,
            num_points - lift_end
        )
        z[lift_end:] = self.pour.finish_z

        return np.column_stack([x, y, z])

    # ── 树叶 (Rosetta) ────────────────────────────────

    def rosetta(self, num_points: int = 350) -> np.ndarray:
        """
        树叶 (Rosetta) 拉花轨迹 — 两阶段模型 (基于 latteartguide.com)
        阶段2a: 推云 (杯中心摆动，靠流速推动图案)
        阶段2b: 回拉 + 摆幅线性递减
        阶段3:  收拢 (摆幅→0)
        阶段4:  划穿收尾
        """
        t = np.linspace(0, 1, num_points)
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        z = np.zeros(num_points)

        # 时间点分配 (比例)
        mix_ratio = 0.22        # 融合
        cloud_ratio = 0.18      # 推云
        pullback_ratio = 0.30   # 回拉+递减
        shrink_ratio = 0.12     # 收拢
        finish_ratio = 0.18     # 收尾

        idx = 0
        # ---- 融合阶段 ----
        n_mix = int(num_points * mix_ratio)
        x[idx:idx+n_mix] = self.cup.center_x
        y[idx:idx+n_mix] = self.cup.center_y + self.cup.radius * 0.35
        z[idx:idx+n_mix] = self.pour.mix_z
        idx += n_mix

        # ---- 推云阶段 (摆动, 不后移) ----
        n_cloud = int(num_points * cloud_ratio)
        A0 = self.pour.wiggle_amplitude
        f = self.pour.wiggle_frequency
        t_cloud = np.linspace(0, 1, n_cloud)
        x[idx:idx+n_cloud] = self.cup.center_x + A0 * np.sin(2 * np.pi * f * t_cloud)
        y[idx:idx+n_cloud] = self.cup.center_y + self.cup.radius * 0.1  # 杯中心
        z[idx:idx+n_cloud] = self.pour.draw_z
        idx += n_cloud

        # ---- 回拉 + 摆幅线性递减 ----
        n_pull = int(num_points * pullback_ratio)
        t_pull = np.linspace(0, 1, n_pull)
        A_decay = A0 * (1 - t_pull)  # 线性递减到 0
        x[idx:idx+n_pull] = self.cup.center_x + A_decay * np.sin(2 * np.pi * f * t_pull)
        y[idx:idx+n_pull] = np.linspace(
            self.cup.center_y + self.cup.radius * 0.1,   # 杯中心
            self.cup.center_y - self.cup.radius * 0.3,   # 杯近端
            n_pull
        )
        z[idx:idx+n_pull] = self.pour.draw_z
        idx += n_pull

        # ---- 收拢阶段 (摆动→0) ----
        n_shrink = int(num_points * shrink_ratio)
        A_small = A0 * 0.15
        t_shrink = np.linspace(0, 1, n_shrink)
        A_final = A_small * (1 - t_shrink)
        x[idx:idx+n_shrink] = self.cup.center_x + A_final * np.sin(2 * np.pi * f * 0.5 * t_shrink)
        y[idx:idx+n_shrink] = self.cup.center_y - self.cup.radius * 0.3
        z[idx:idx+n_shrink] = self.pour.draw_z
        idx += n_shrink

        # ---- 划穿收尾 ----
        n_finish = num_points - idx
        x[idx:] = self.cup.center_x
        y[idx:] = np.linspace(
            self.cup.center_y - self.cup.radius * 0.3,
            self.cup.center_y + self.cup.radius * 0.3,
            n_finish
        )
        z[idx:] = self.pour.finish_z

        return np.column_stack([x, y, z])

    # ── 郁金香 ────────────────────────────────────────

    def tulip(self, layers: int = 3, num_points: int = 350) -> np.ndarray:
        """
        郁金香 (多层同心堆叠) — 基于 latteartguide.com
        每层: 杯近端定点摇摆 + 前推穿过前层形成半圆
        每层半圆同心嵌套在前一层内部，摆幅递减
        """
        t = np.linspace(0, 1, num_points)
        x = np.full(num_points, self.cup.center_x)
        y = np.full(num_points, self.cup.center_y + self.cup.radius * 0.35)
        z = np.full(num_points, self.pour.mix_z)

        # 融合阶段
        mix_end = int(num_points * 0.22)
        x[:mix_end] = self.cup.center_x
        y[:mix_end] = self.cup.center_y + self.cup.radius * 0.35
        z[:mix_end] = self.pour.mix_z

        # 每层成形
        draw_start = mix_end
        draw_duration = int(num_points * 0.55 / layers)  # 每层点数
        finish_start = draw_start + draw_duration * layers

        base_amplitude = self.pour.wiggle_amplitude * 0.6
        base_push = self.cup.radius * 0.2

        for i in range(layers):
            l_start = draw_start + i * draw_duration
            l_mid = l_start + draw_duration // 3
            l_end = l_start + draw_duration

            # 第 i 层: 起始 Y 逐层后退 (杯近端方向)
            y_layer_start = self.cup.center_y + self.cup.radius * (0.25 - i * 0.07)
            # 摆幅逐层递减
            A_layer = base_amplitude * (1.0 - i * 0.2)
            # 前推距离逐层递增 (为了穿过前面的层)
            push_dist = base_push + i * self.cup.radius * 0.03

            # 摇摆阶段
            t_sway = np.linspace(0, 2 * np.pi * 2, l_mid - l_start)
            x[l_start:l_mid] = self.cup.center_x + A_layer * np.sin(t_sway)
            y[l_start:l_mid] = y_layer_start
            z[l_start:l_mid] = self.pour.draw_z

            # 前推阶段
            t_push = np.linspace(0, 1, l_end - l_mid)
            x[l_mid:l_end] = self.cup.center_x + A_layer * 0.3 * np.sin(t_push * 2 * np.pi)
            y[l_mid:l_end] = y_layer_start + t_push * push_dist
            z[l_mid:l_end] = self.pour.draw_z

            # 层间短停 + 抬高 (模拟停止倒奶)
            if i < layers - 1:
                gap_start = l_end
                gap_end = min(l_end + max(1, draw_duration // 15), finish_start)
                x[gap_start:gap_end] = self.cup.center_x
                y[gap_start:gap_end] = y_layer_start + push_dist
                z[gap_start:gap_end] = self.pour.mix_z  # 抬高停止成形

        # 收尾 draw-through
        x[finish_start:] = self.cup.center_x
        y[finish_start:] = np.linspace(
            self.cup.center_y + self.cup.radius * 0.05,
            self.cup.center_y + self.cup.radius * 0.35,
            num_points - finish_start
        )
        z[finish_start:] = self.pour.finish_z

        return np.column_stack([x, y, z])

    # ── 天鹅 ──────────────────────────────────────────

    def swan(self, num_points: int = 400) -> np.ndarray:
        """天鹅拉花轨迹 — 组合动作 (树叶 + 心形 + 拉线)"""
        # 阶段比例
        body_end = int(num_points * 0.5)    # 身体 (Rosetta)
        neck_end = int(num_points * 0.75)    # 颈部 (拉线)
        # head_end = num_points               # 头部 (小圆)

        traj = np.zeros((num_points, 3))

        # 身体: Rosetta 摆动
        t_body = np.linspace(0, 1, body_end)
        traj[:body_end, 0] = self.cup.center_x + 0.006 * np.sin(2 * np.pi * 4.0 * t_body)
        traj[:body_end, 1] = np.linspace(
            self.cup.center_y + 0.02,  # 从杯前开始
            self.cup.center_y - 0.01,  # 移到杯中后方
            body_end
        )
        traj[:body_end, 2] = self.pour.draw_z

        # 颈部: 弧线拉回前方
        t_neck = np.linspace(0, 1, neck_end - body_end)
        traj[body_end:neck_end, 0] = self.cup.center_x
        traj[body_end:neck_end, 1] = np.linspace(-0.01, 0.025, neck_end - body_end)
        traj[body_end:neck_end, 2] = self.pour.draw_z + 0.005

        # 头部: 小圆形
        angle = np.linspace(0, 2 * np.pi, num_points - neck_end)
        traj[neck_end:, 0] = self.cup.center_x + 0.006 * np.cos(angle)
        traj[neck_end:, 1] = 0.025 + 0.006 * np.sin(angle)
        traj[neck_end:, 2] = self.pour.draw_z

        return traj

    # ── 自定义 (从图像生成) ───────────────────────────

    def from_image(self, image_path: str, num_points: int = 200) -> np.ndarray:
        """
        从拉花图案图像生成轨迹
        用 Canny 边缘检测提取轮廓，样条拟合为平滑路径
        注意: 需要安装 opencv-python
        """
        try:
            import cv2
        except ImportError:
            raise ImportError("需要安装 opencv-python: pip install opencv-python")

        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"无法读取图像: {image_path}")

        edges = cv2.Canny(img, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            raise ValueError("未检测到轮廓")

        main_contour = max(contours, key=cv2.contourArea).squeeze()
        if main_contour.ndim != 2 or main_contour.shape[0] < 4:
            raise ValueError("轮廓点不足，需要至少 4 个点用于样条拟合")

        # 归一化到杯口坐标
        contour_norm = main_contour.astype(np.float64)
        contour_norm -= contour_norm.min(axis=0)
        contour_norm /= contour_norm.max(axis=0)
        contour_norm = contour_norm * (self.cup.radius * 0.6) + np.array(
            [self.cup.center_x - self.cup.radius * 0.3,
             self.cup.center_y - self.cup.radius * 0.3]
        )

        # 样条拟合平滑
        tck, u = splprep([contour_norm[:, 0], contour_norm[:, 1]], s=1.0)
        u_new = np.linspace(0, 1, num_points)
        x_smooth, y_smooth = splev(u_new, tck)

        # 添加高度: 全程贴近液面 (仅图案轨迹，不含融合/收尾)
        z = np.full(num_points, self.pour.draw_z)

        return np.column_stack([x_smooth, y_smooth, z])


# ═══════════════════════════════════════════════════════════════
# 轨迹后处理
# ═══════════════════════════════════════════════════════════════

def apply_anti_sloshing(traj: np.ndarray, pour: PourConfig,
                         dt: float = 0.01) -> np.ndarray:
    """
    对轨迹施加抗晃荡速度约束
    使用 S 曲线速度剖面替换均匀采样，减小 jerk 峰值
    """
    # 计算弧长
    diffs = np.diff(traj, axis=0)
    arc_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate([[0], np.cumsum(arc_lengths)])
    total_length = cumulative[-1]

    # 生成 S 曲线速度剖面 (简化: 梯形)
    t = cumulative / total_length  # 归一化时间 [0, 1]
    v = np.ones_like(t) * pour.max_velocity

    # 加速段
    accel_ramp = int(len(t) * 0.15)
    v[:accel_ramp] = np.linspace(0.001, pour.max_velocity, accel_ramp)
    # 减速段
    decel_ramp = int(len(t) * 0.15)
    v[-decel_ramp:] = np.linspace(pour.max_velocity, 0.001, decel_ramp)

    # 重新参数化: 按速度分配时间
    dt_segments = np.diff(cumulative) / v[:-1]
    new_t = np.concatenate([[0], np.cumsum(dt_segments)])
    new_t = new_t / new_t[-1]

    # 在新时间点上插值
    cs_x = CubicSpline(t, traj[:, 0])
    cs_y = CubicSpline(t, traj[:, 1])
    cs_z = CubicSpline(t, traj[:, 2])

    return np.column_stack([
        cs_x(new_t), cs_y(new_t), cs_z(new_t)
    ])


def compose_full_trajectory(pattern_traj: np.ndarray,
                             cup: CupConfig,
                             pour: PourConfig,
                             num_mix: int = 50,
                             num_finish: int = 30) -> np.ndarray:
    """
    为图案轨迹添加融合和收尾阶段，生成完整拉花轨迹
    """
    # 融合阶段: 杯中心高位缓慢注入
    mix_traj = np.tile([cup.center_x, cup.center_y, pour.mix_z],
                       (num_mix, 1))
    # 加入微小扰动模拟融合搅拌
    mix_traj[:, 0] += 0.002 * np.sin(np.linspace(0, 2*np.pi, num_mix))

    # 收尾阶段: 上提拉线
    finish_traj = np.column_stack([
        np.full(num_finish, cup.center_x),
        np.linspace(pattern_traj[-1, 1], cup.center_y - cup.radius * 0.2, num_finish),
        np.linspace(pour.draw_z, pour.finish_z, num_finish),
    ])

    return np.vstack([mix_traj, pattern_traj, finish_traj])


# ═══════════════════════════════════════════════════════════════
# 导出
# ═══════════════════════════════════════════════════════════════

def to_moveit_waypoints(traj_xyz: np.ndarray,
                         roll: float = 0.0,
                         pitch: float = np.pi / 4,  # 奶缸倾斜 45°
                         yaw: float = 0.0) -> List[List[float]]:
    """
    将 XYZ 轨迹转换为带姿态的 MoveIt2 路点
    姿态: 奶缸以固定角度倾斜 (约 45°) 进行倒奶
    返回: [[x, y, z, qx, qy, qz, qw], ...]
    """
    from scipy.spatial.transform import Rotation
    quat = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    # scipy quat 格式: [x, y, z, w]
    return [
        [float(pt[0]), float(pt[1]), float(pt[2]),
         quat[0], quat[1], quat[2], quat[3]]
        for pt in traj_xyz
    ]


def save_csv(waypoints: List[List[float]], filepath: str):
    """保存路点到 CSV"""
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writerows(waypoints)
    print(f"已保存 {len(waypoints)} 个路点到 {filepath}")


# ═══════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="拉花轨迹生成器")
    parser.add_argument('--pattern', choices=['heart', 'rosetta', 'tulip', 'swan', 'custom'],
                        default='heart', help='拉花图案类型')
    parser.add_argument('--image', type=str, help='自定义图案图像路径 (仅 --pattern custom)')
    parser.add_argument('--output', type=str, default='latte_art_waypoints.csv',
                        help='输出 CSV 路径')
    parser.add_argument('--layers', type=int, default=3, help='郁金香层数')
    parser.add_argument('--no-sloshing', action='store_true', help='禁用抗晃荡约束')
    args = parser.parse_args()

    cup = CupConfig()
    pour = PourConfig()

    traj_gen = LatteArtTrajectory(cup, pour)

    # 生成图案轨迹
    if args.pattern == 'heart':
        traj = traj_gen.heart()
    elif args.pattern == 'rosetta':
        traj = traj_gen.rosetta()
    elif args.pattern == 'tulip':
        traj = traj_gen.tulip(layers=args.layers)
    elif args.pattern == 'swan':
        traj = traj_gen.swan()
    elif args.pattern == 'custom':
        if not args.image:
            parser.error("--pattern custom 需要 --image 参数")
        traj = traj_gen.from_image(args.image)
    else:
        raise ValueError(f"未知图案: {args.pattern}")

    # 添加融合和收尾阶段
    full_traj = compose_full_trajectory(traj, cup, pour)

    # 抗晃荡约束
    if not args.no_sloshing:
        full_traj = apply_anti_sloshing(full_traj, pour)

    # 转换为 MoveIt2 格式
    waypoints = to_moveit_waypoints(full_traj)

    # 保存
    save_csv(waypoints, args.output)

    # 输出统计
    print(f"图案: {args.pattern}")
    print(f"路点数: {len(waypoints)}")
    print(f"轨迹范围 X: [{full_traj[:, 0].min():.4f}, {full_traj[:, 0].max():.4f}]")
    print(f"轨迹范围 Y: [{full_traj[:, 1].min():.4f}, {full_traj[:, 1].max():.4f}]")
    print(f"轨迹范围 Z: [{full_traj[:, 2].min():.4f}, {full_traj[:, 2].max():.4f}]")


if __name__ == '__main__':
    main()
