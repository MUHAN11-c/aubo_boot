"""拉花图案轨迹生成器 — 将图案参数化为 3D 笛卡尔轨迹喵~

来源: latte_art_robot_research/轨迹生成方案/latte_art_trajectory.py
理论: Sunergos 视频 (YouTube/B站 BV1uE411g7MJ) + latteartguide.com 进阶教程
支持图案: heart, rosetta, tulip, swan, custom (from_image)
"""

import numpy as np
from scipy.interpolate import splprep, splev
from typing import Optional

from .config import CupConfig, PourConfig


class LatteArtTrajectory:
    """生成各类拉花图案的 3D 末端执行器轨迹 (仅 XYZ 位置) 喵~

    坐标系: 机械臂 base_link (X前 Y左 Z上, REP-103)
    输出: (T, 3) float numpy array — XYZ 位置序列
    姿态: 由调用方通过 to_moveit_waypoints() 或 bridge.py 添加喵~
    """

    def __init__(self, cup: Optional[CupConfig] = None,
                 pour: Optional[PourConfig] = None):
        self.cup = cup or CupConfig()
        self.pour = pour or PourConfig()

    # ── 心形 (Heart) ──────────────────────────────────────

    def heart(self, num_points: int = 200) -> np.ndarray:
        """心形拉花轨迹 — 五步法 (基于 latteartguide.com + Sunergos) 喵~

        Step 3: 固定中心点微摆 → 圆形白色底
        Step 4: 竖直抬起 → "吸力让圆形顶部形成弧线" → 心形顶部凹陷
        Step 5: 划穿收尾 (draw-through)
        """
        cup, pour = self.cup, self.pour
        t = np.linspace(0, 1, num_points)
        mix_end = int(num_points * 0.25)
        sway_end = int(num_points * 0.70)
        lift_end = int(num_points * 0.85)

        # 轨迹在 (0,0) 原点生成, cup_center 由外部在 retarget 后叠加 喵~
        r = cup.radius
        surf = cup.surface_z
        x = np.full(num_points, 0.0)
        y = np.full(num_points, r * 0.3)
        z = np.full(num_points, pour.mix_z(surf))

        # 成形微摆阶段: 固定中心点, 轻微左右摆动形成圆形
        t_sway = np.linspace(0, 2 * np.pi * 3, sway_end - mix_end)
        x[mix_end:sway_end] = pour.wiggle_amplitude * 0.3 * np.sin(t_sway)
        y[mix_end:sway_end] = r * 0.15
        z[mix_end:sway_end] = pour.draw_z(surf)

        # 竖直抬起阶段 (关键: 抬升产生吸力, 圆形顶部形成弧线)
        z[sway_end:lift_end] = np.linspace(pour.draw_z(surf), pour.finish_z(surf),
                                            lift_end - sway_end)
        x[sway_end:lift_end] = 0.0
        y[sway_end:lift_end] = r * 0.15

        # 划穿收尾

        y[lift_end:] = np.linspace(r * 0.15, -r * 0.2, num_points - lift_end)
        z[lift_end:] = pour.finish_z(surf)

        return np.column_stack([x, y, z])

    # ── 树叶 (Rosetta) ────────────────────────────────────

    def rosetta(self, num_points: int = 350) -> np.ndarray:
        """树叶 (Rosetta) 拉花轨迹 — 四阶段模型 (基于 latteartguide.com) 喵~

        阶段2a: 推云 (杯中心摆动, 靠流速推动图案)
        阶段2b: 回拉 + 摆幅线性递减
        阶段3:  收拢 (摆幅→0)
        阶段4:  划穿收尾
        """
        cup, pour = self.cup, self.pour
        surf = cup.surface_z
        t = np.linspace(0, 1, num_points)
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        z = np.zeros(num_points)

        mix_ratio = 0.22
        cloud_ratio = 0.18
        pullback_ratio = 0.30
        shrink_ratio = 0.12
        finish_ratio = 0.18

        idx = 0
        # 融合阶段
        n_mix = int(num_points * mix_ratio)
        x[idx:idx+n_mix] = cup.center_x
        y[idx:idx+n_mix] = cup.center_y + cup.radius * 0.35
        z[idx:idx+n_mix] = pour.mix_z(surf)
        idx += n_mix

        # 推云阶段 (摆动, 不后移)
        n_cloud = int(num_points * cloud_ratio)
        A0 = pour.wiggle_amplitude
        f = pour.wiggle_frequency
        t_cloud = np.linspace(0, 1, n_cloud)
        x[idx:idx+n_cloud] = cup.center_x + A0 * np.sin(2 * np.pi * f * t_cloud)
        y[idx:idx+n_cloud] = cup.center_y + cup.radius * 0.1
        z[idx:idx+n_cloud] = pour.draw_z(surf)
        idx += n_cloud

        # 回拉 + 摆幅线性递减
        n_pull = int(num_points * pullback_ratio)
        t_pull = np.linspace(0, 1, n_pull)
        A_decay = A0 * (1 - t_pull)
        x[idx:idx+n_pull] = cup.center_x + A_decay * np.sin(2 * np.pi * f * t_pull)
        y[idx:idx+n_pull] = np.linspace(
            cup.center_y + cup.radius * 0.1,
            cup.center_y - cup.radius * 0.3,
            n_pull,
        )
        z[idx:idx+n_pull] = pour.draw_z(surf)
        idx += n_pull

        # 收拢阶段 (摆动→0)
        n_shrink = int(num_points * shrink_ratio)
        A_small = A0 * 0.15
        t_shrink = np.linspace(0, 1, n_shrink)
        A_final = A_small * (1 - t_shrink)
        x[idx:idx+n_shrink] = cup.center_x + A_final * np.sin(
            2 * np.pi * f * 0.5 * t_shrink)
        y[idx:idx+n_shrink] = cup.center_y - cup.radius * 0.3
        z[idx:idx+n_shrink] = pour.draw_z(surf)
        idx += n_shrink

        # 划穿收尾
        n_finish = num_points - idx
        x[idx:] = cup.center_x
        y[idx:] = np.linspace(
            cup.center_y - cup.radius * 0.3,
            cup.center_y + cup.radius * 0.3,
            n_finish,
        )
        z[idx:] = pour.finish_z(surf)

        return np.column_stack([x, y, z])

    # ── 郁金香 (Tulip) ────────────────────────────────────

    def tulip(self, layers: int = 3, num_points: int = 350) -> np.ndarray:
        """郁金香 (多层同心堆叠) — 基于 latteartguide.com 喵~

        每层: 杯近端定点摇摆 + 前推穿过前层形成半圆
        每层半圆同心嵌套在前一层内部, 摆幅递减
        """
        cup, pour = self.cup, self.pour
        surf = cup.surface_z
        t = np.linspace(0, 1, num_points)
        x = np.full(num_points, cup.center_x)
        y = np.full(num_points, cup.center_y + cup.radius * 0.35)
        z = np.full(num_points, pour.mix_z(surf))

        layers = max(1, layers)
        mix_end = int(num_points * 0.22)
        x[:mix_end] = cup.center_x
        y[:mix_end] = cup.center_y + cup.radius * 0.35
        z[:mix_end] = pour.mix_z(surf)

        draw_start = mix_end
        draw_duration = max(1, int(num_points * 0.55 / layers))
        finish_start = draw_start + draw_duration * layers

        base_amplitude = pour.wiggle_amplitude * 0.6
        base_push = cup.radius * 0.2

        for i in range(layers):
            l_start = draw_start + i * draw_duration
            l_mid = l_start + draw_duration // 3
            l_end = min(l_start + draw_duration, finish_start)

            y_layer_start = cup.center_y + cup.radius * (0.25 - i * 0.07)
            A_layer = base_amplitude * (1.0 - i * 0.2)
            push_dist = base_push + i * cup.radius * 0.03

            t_sway = np.linspace(0, 2 * np.pi * 2, max(1, l_mid - l_start))
            segment_len = min(len(t_sway), l_mid - l_start)
            x[l_start:l_start+segment_len] = cup.center_x + A_layer * np.sin(t_sway[:segment_len])
            y[l_start:l_start+segment_len] = y_layer_start
            z[l_start:l_start+segment_len] = pour.draw_z(surf)

            n_push = max(1, l_end - l_mid)
            t_push = np.linspace(0, 1, n_push)
            x[l_mid:l_end] = cup.center_x + A_layer * 0.3 * np.sin(t_push * 2 * np.pi)
            y[l_mid:l_end] = y_layer_start + t_push * push_dist
            z[l_mid:l_end] = pour.draw_z(surf)

            if i < layers - 1:
                gap_start = l_end
                gap_end = min(l_end + max(1, draw_duration // 15), finish_start)
                n_gap = max(0, gap_end - gap_start)
                if n_gap > 0:
                    x[gap_start:gap_end] = cup.center_x
                    y[gap_start:gap_end] = y_layer_start + push_dist
                    z[gap_start:gap_end] = pour.mix_z(surf)

        n_finish = num_points - finish_start
        if n_finish > 0:
            x[finish_start:] = cup.center_x
            y[finish_start:] = np.linspace(
                cup.center_y + cup.radius * 0.05,
                cup.center_y + cup.radius * 0.35,
                n_finish,
            )
            z[finish_start:] = pour.finish_z(surf)

        return np.column_stack([x, y, z])

    # ── 天鹅 (Swan) ───────────────────────────────────────

    def swan(self, num_points: int = 400) -> np.ndarray:
        """天鹅拉花轨迹 — 组合动作 (树叶 + 心形 + 拉线) 喵~"""
        cup, pour = self.cup, self.pour
        surf = cup.surface_z

        body_end = int(num_points * 0.5)
        neck_end = int(num_points * 0.75)

        traj = np.zeros((num_points, 3))

        # 身体: Rosetta 摆动
        t_body = np.linspace(0, 1, body_end)
        traj[:body_end, 0] = cup.center_x + 0.006 * np.sin(2 * np.pi * 4.0 * t_body)
        traj[:body_end, 1] = np.linspace(
            cup.center_y + 0.02,
            cup.center_y - 0.01,
            body_end,
        )
        traj[:body_end, 2] = pour.draw_z(surf)

        # 颈部: 弧线拉回前方
        t_neck = np.linspace(0, 1, neck_end - body_end)
        traj[body_end:neck_end, 0] = cup.center_x
        traj[body_end:neck_end, 1] = np.linspace(-0.01, 0.025, neck_end - body_end)
        traj[body_end:neck_end, 2] = pour.draw_z(surf) + 0.005

        # 头部: 小圆形
        n_head = num_points - neck_end
        angle = np.linspace(0, 2 * np.pi, n_head)
        traj[neck_end:, 0] = cup.center_x + 0.006 * np.cos(angle)
        traj[neck_end:, 1] = 0.025 + 0.006 * np.sin(angle)
        traj[neck_end:, 2] = pour.draw_z(surf)

        return traj

    # ── 自定义 (从图像生成) ────────────────────────────────

    def from_image(self, image_path: str, num_points: int = 200) -> np.ndarray:
        """从拉花图案图像生成轨迹 — Canny 边缘检测 + 样条拟合 喵~

        注意: 需要安装 opencv-python
        """
        try:
            import cv2
        except ImportError:
            raise ImportError("需要安装 opencv-python: pip install opencv-python")

        cup, pour = self.cup, self.pour

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

        contour_norm = main_contour.astype(np.float64)
        contour_norm -= contour_norm.min(axis=0)
        contour_norm /= contour_norm.max(axis=0)
        contour_norm = contour_norm * (cup.radius * 0.6) + np.array(
            [cup.center_x - cup.radius * 0.3,
             cup.center_y - cup.radius * 0.3]
        )

        tck, u = splprep([contour_norm[:, 0], contour_norm[:, 1]], s=1.0)
        u_new = np.linspace(0, 1, num_points)
        x_smooth, y_smooth = splev(u_new, tck)

        z = np.full(num_points, pour.draw_z(cup.surface_z))

        return np.column_stack([x_smooth, y_smooth, z])
