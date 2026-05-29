"""
笛卡尔末端轨迹 — 数据结构、IO、统计、采样、安全校验喵~

=== 数据模型 ===

  CartesianTrajectory 存储 RM65 录制的拉花轨迹:
    positions:    (T,3) float32 — 末端 XYZ 位置 (米)
    orientations: (T,4) float32 — 四元数 xyzw (Hamilton convention, ROS 2 标准)
    timestamps:   (T,) float32 — 时间戳 (秒, 从 0 开始)
    dt:           时间步长 (0.05s = 20fps)
    episode_idx:  0-39

=== 理论依据 ===

  SPOT (arXiv:2411.00965):
    Object-centric SE(3) 轨迹 — 存储绝对位姿序列 (来自真实机器人 FK)
    Keyframe selection: frame added when velocity zero OR exceeds distance threshold

  SVRC Trajectory Representation:
    Object-relative Cartesian → Very High generalization
    推荐用自适应采样替代固定步长采样

  ROS 2 REP-103:
    四元数 Hamilton 约定 (xyzw), 坐标系 X 前 Y 左 Z 上
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from glob import glob
from typing import Optional

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Header


@dataclass
class CartesianTrajectory:
    """笛卡尔末端轨迹 (T 帧, 3D 位置 + 4D 四元数姿态) 喵~

    属性:
        positions:    (T, 3) float32 末端 XYZ 位置 (米)
        orientations: (T, 4) float32 四元数 xyzw (Hamilton convention)
        timestamps:   (T,)  float32 时间戳 (秒)
        dt:           时间步长 (秒), 默认 0.05 (20fps)
        episode_idx:  数据集 episode 编号 (0-39)
        frame_id:     坐标系 ID, 默认 "base_link"

    参考:
        SPOT (arXiv:2411.00965) — Object-centric SE(3) 轨迹存储
        ROS 2 REP-103 — Hamilton 四元数约定 (xyzw)
    """

    positions: np.ndarray          # (T, 3)
    timestamps: np.ndarray         # (T,)
    dt: float
    orientations: Optional[np.ndarray] = None  # (T, 4) quaternion xyzw
    episode_idx: int = -1
    frame_id: str = "base_link"

    def __post_init__(self):
        self.positions = np.asarray(self.positions, dtype=float)
        self.timestamps = np.asarray(self.timestamps, dtype=float)
        if self.orientations is not None:
            self.orientations = np.asarray(self.orientations, dtype=float)

    # ═══ IO ═══════════════════════════════════════════════════════

    @classmethod
    def load(cls, path: str) -> CartesianTrajectory:
        """从 npz 文件加载轨迹喵~

        npz 文件格式 (来自 ridxm/latte-pour-demos):
            positions:     (T, 3) float32 — 末端 XYZ 位置
            orientations:  (T, 4) float32 — 四元数 xyzw
            timestamps:    (T,)  float32 — 时间戳
            dt:            scalar float64 — 时间步长
            episode_idx:   scalar int64   — episode 编号
        """
        data = np.load(path)
        return cls(
            positions=data["positions"],
            orientations=data.get("orientations"),
            timestamps=data["timestamps"],
            dt=float(data["dt"]),
            episode_idx=int(data.get("episode_idx", -1)),
            frame_id=str(data.get("frame_id", "base_link")),
        )

    @classmethod
    def load_all(cls, resource_dir: str, arm: str) -> dict[int, CartesianTrajectory]:
        """加载 resource/cartesian/{arm}/ 下所有 npz 文件喵~

        Returns:
            dict[int, CartesianTrajectory]: {episode_idx: trajectory}
        """
        from collections import OrderedDict
        result = OrderedDict()
        pattern = os.path.join(resource_dir, arm, "episode_*.npz")
        for f in sorted(glob(pattern)):
            basename = os.path.basename(f)
            try:
                ep = int(basename.replace("episode_", "").replace(".npz", ""))
            except ValueError:
                continue
            result[ep] = cls.load(f)
        return result

    @classmethod
    def from_xyz(
        cls,
        xyz: np.ndarray,
        orientations: Optional[np.ndarray] = None,
        dt: float = 0.05,
        episode_idx: int = -1,
        frame_id: str = "base_link",
    ) -> "CartesianTrajectory":
        """从纯 XYZ 位置创建 CartesianTrajectory (用于参数化生成) 喵~

        episode_idx=-1 标记为生成轨迹，区别于录制的 0-39 喵~

        Args:
            xyz: (T, 3) float array — XYZ 位置
            orientations: (T, 4) float array — 四元数 xyzw, 默认固定 45° pitch
            dt: 时间步长 (s)
            episode_idx: episode 编号, -1 = 生成轨迹
            frame_id: 坐标系 ID
        """
        T = len(xyz)
        if orientations is None:
            # 默认奶缸前倾 45° 姿态
            from .trajectory_transform import euler_deg_to_quat
            q = euler_deg_to_quat(45.0, 0.0, 0.0)
            orientations = np.tile(q.astype(np.float32), (T, 1))
        timestamps = np.arange(T, dtype=np.float32) * dt
        return cls(
            positions=np.asarray(xyz, dtype=np.float32),
            orientations=np.asarray(orientations, dtype=np.float32),
            timestamps=timestamps,
            dt=dt,
            episode_idx=episode_idx,
            frame_id=frame_id,
        )

    def save(self, path: str):
        """保存为 npz 压缩文件喵~"""
        kwargs = dict(
            positions=self.positions.astype(np.float32),
            timestamps=self.timestamps.astype(np.float32),
            dt=np.float32(self.dt),
            episode_idx=self.episode_idx,
            frame_id=self.frame_id,
        )
        if self.orientations is not None:
            kwargs["orientations"] = self.orientations.astype(np.float32)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        np.savez_compressed(path, **kwargs)

    # ═══ 属性 ═════════════════════════════════════════════════════

    @property
    def num_frames(self) -> int:
        """轨迹总帧数喵~"""
        return len(self.positions)

    @property
    def start(self) -> np.ndarray:
        """第一帧位置 (3,) 喵~"""
        return self.positions[0]

    @property
    def end(self) -> np.ndarray:
        """最后一帧位置 (3,) 喵~"""
        return self.positions[-1]

    @property
    def bbox(self) -> np.ndarray:
        """轨迹外包盒 [[x_min,x_max],[y_min,y_max],[z_min,z_max]] 喵~"""
        return np.array([
            [self.positions[:, 0].min(), self.positions[:, 0].max()],
            [self.positions[:, 1].min(), self.positions[:, 1].max()],
            [self.positions[:, 2].min(), self.positions[:, 2].max()],
        ])

    def __len__(self) -> int:
        return self.num_frames

    def __getitem__(self, idx: int) -> np.ndarray:
        return self.positions[idx]

    # ═══ 统计 ═════════════════════════════════════════════════════

    def path_length(self) -> float:
        """笛卡尔路径总长度 (米) = Σ ||p_i - p_{i-1}|| 喵~

        刚性变换保距，变换后路径长度不变喵~
        """
        return float(np.sum(np.linalg.norm(np.diff(self.positions, axis=0), axis=1)))

    def velocity_profile(self) -> np.ndarray:
        """帧间速度 (m/s) = ||Δp|| / dt 喵~"""
        return np.linalg.norm(np.diff(self.positions, axis=0), axis=1) / self.dt

    # ═══ 采样 ═════════════════════════════════════════════════════

    def adaptive_sample(self, displacement_threshold: float = 0.005) -> list[int]:
        """自适应关键帧采样 — 基于笛卡尔位移阈值喵~

        理论依据 (SPOT, Section IV-A):
            "a frame is added if the relative velocity is zero (indicating a
             change in direction) or if it exceeds a certain distance threshold
             from the previous keyframe"

        算法:
            保留第一帧和最后一帧始终在采样集中。
            对中间帧: 当累积位移 >= threshold 时, 添加该帧为关键帧。

        阈值 0.005m (5mm) 依据:
            原始轨迹帧间位移 ≈ 1.53m / 400帧 ≈ 3.8mm
            阈值略大于平均位移 → 约每 1.3 帧采样一个 waypoint
            → 约 300 waypoints (MoveIt2 IK 总计 < 1s)

        Args:
            displacement_threshold: 累积位移阈值 (米), 默认 0.005

        Returns:
            list[int]: 关键帧索引列表
        """
        if self.num_frames <= 2:
            return list(range(self.num_frames))

        indices = [0]  # 始终保留第一帧
        accumulated = 0.0
        last_pos = self.positions[0]

        for i in range(1, self.num_frames - 1):
            delta = np.linalg.norm(self.positions[i] - last_pos)
            accumulated += delta
            if accumulated >= displacement_threshold:
                indices.append(i)
                accumulated = 0.0
                last_pos = self.positions[i]

        indices.append(self.num_frames - 1)  # 始终保留最后一帧
        return indices

    def resample(self, target_fps: float) -> CartesianTrajectory:
        """降采样到指定帧率喵~

        Args:
            target_fps: 目标帧率 (Hz)

        Returns:
            新的 CartesianTrajectory
        """
        if target_fps <= 0:
            return self
        step = max(1, int(1.0 / (target_fps * self.dt)))
        idx = list(range(0, self.num_frames, step))
        if idx[-1] != self.num_frames - 1:
            idx.append(self.num_frames - 1)
        return self._slice(idx)

    # ═══ 平滑 ═════════════════════════════════════════════════════

    def smooth_savgol(self, window: int = 7, order: int = 3) -> CartesianTrajectory:
        """Savitzky-Golay 滤波器平滑位置数据喵~

        window=7, order=3 针对 400 帧轨迹数据优化。

        Args:
            window: 滑动窗口长度 (奇数), 默认 7
            order:  多项式阶数, 默认 3

        Returns:
            平滑后的 CartesianTrajectory (新对象, 原数据不动)
        """
        try:
            from scipy.signal import savgol_filter
        except ImportError:
            # 无 scipy 时回退到简单移动平均
            return self._smooth_ma(window)

        if window % 2 == 0:
            window += 1  # SG 窗口必须为奇数
        if window >= self.num_frames:
            window = self.num_frames - 2 if self.num_frames > 3 else self.num_frames
            if window % 2 == 0:
                window -= 1

        smoothed = savgol_filter(self.positions, window, order, axis=0)
        return CartesianTrajectory(
            positions=smoothed,
            orientations=self.orientations,
            timestamps=self.timestamps.copy(),
            dt=self.dt,
            episode_idx=self.episode_idx,
            frame_id=self.frame_id,
        )

    def _smooth_ma(self, window: int) -> CartesianTrajectory:
        """移动平均平滑 (无 scipy 时的回退方案) 喵~"""
        if window < 3 or window >= self.num_frames:
            return self
        kernel = np.ones(window) / window
        pad = window // 2
        smoothed = np.apply_along_axis(
            lambda col: np.convolve(col, kernel, mode="same"), 0, self.positions
        )
        # 边界用原始值填充
        smoothed[:pad] = self.positions[:pad]
        smoothed[-pad:] = self.positions[-pad:]
        return CartesianTrajectory(
            positions=smoothed,
            orientations=self.orientations,
            timestamps=self.timestamps.copy(),
            dt=self.dt,
            episode_idx=self.episode_idx,
            frame_id=self.frame_id,
        )

    # ═══ 安全校验 ═════════════════════════════════════════════════

    def check_workspace_bounds(
        self,
        x_range: tuple[float, float],
        y_range: tuple[float, float],
        z_range: tuple[float, float],
    ) -> tuple[bool, str, int]:
        """检查所有帧是否在工作空间边界内喵~

        理论依据:
            AUBO E5 workspace_limits.yaml + limit_workspace.py
            默认安全边界: X[-0.7,0.7] Y[-0.35,0.35] Z[0.0,0.7]

        Args:
            x_range: (x_min, x_max) 米
            y_range: (y_min, y_max) 米
            z_range: (z_min, z_max) 米

        Returns:
            (is_safe, message, first_violation_frame):
                is_safe: True=所有帧在安全区内
                message: 违规描述 (安全时为空)
                first_violation_frame: 首次违规帧索引 (安全时为 -1)
        """
        x_ok = (self.positions[:, 0] >= x_range[0]) & (self.positions[:, 0] <= x_range[1])
        y_ok = (self.positions[:, 1] >= y_range[0]) & (self.positions[:, 1] <= y_range[1])
        z_ok = (self.positions[:, 2] >= z_range[0]) & (self.positions[:, 2] <= z_range[1])

        all_ok = x_ok & y_ok & z_ok
        if np.all(all_ok):
            return True, "", -1

        first_bad = int(np.argmin(all_ok))
        p = self.positions[first_bad]
        violations = []
        if not x_ok[first_bad]:
            violations.append(f"X={p[0]:.3f} ∉ [{x_range[0]:.3f}, {x_range[1]:.3f}]")
        if not y_ok[first_bad]:
            violations.append(f"Y={p[1]:.3f} ∉ [{y_range[0]:.3f}, {y_range[1]:.3f}]")
        if not z_ok[first_bad]:
            violations.append(f"Z={p[2]:.3f} ∉ [{z_range[0]:.3f}, {z_range[1]:.3f}]")
        msg = f"Frame {first_bad}/{self.num_frames}: " + ", ".join(violations)
        return False, msg, first_bad

    # ═══ ROS2 导出 ════════════════════════════════════════════════

    def to_pose(self, idx: int) -> Pose:
        """导出第 idx 帧为 geometry_msgs/Pose 喵~"""
        p = self.positions[idx]
        pose = Pose(
            position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        )
        if self.orientations is not None:
            q = self.orientations[idx]
            pose.orientation = Quaternion(
                x=float(q[0]), y=float(q[1]),
                z=float(q[2]), w=float(q[3]),
            )
        else:
            pose.orientation.w = 1.0
        return pose

    def to_pose_stamped(self, idx: int, stamp=None) -> PoseStamped:
        """导出第 idx 帧为 geometry_msgs/PoseStamped 喵~"""
        ps = PoseStamped(
            header=Header(frame_id=self.frame_id),
            pose=self.to_pose(idx),
        )
        if stamp is not None:
            ps.header.stamp = stamp
        return ps

    def to_ros2_path(self, step: int = 5, stamp=None) -> RosPath:
        """导出完整轨迹为 nav_msgs/Path 喵~

        Args:
            step: 采样步长 (每 step 帧导出一个 waypoint)
            stamp: 可选时间戳
        """
        path = RosPath()
        path.header.frame_id = self.frame_id
        if stamp is not None:
            path.header.stamp = stamp
        for i in range(0, self.num_frames, step):
            path.poses.append(self.to_pose_stamped(i, stamp))
        return path

    # ═══ 切片 ═════════════════════════════════════════════════════

    def segment(self, start_idx: int, end_idx: int) -> CartesianTrajectory:
        """截取轨迹片段 [start_idx, end_idx) 喵~"""
        return CartesianTrajectory(
            positions=self.positions[start_idx:end_idx],
            orientations=(
                self.orientations[start_idx:end_idx]
                if self.orientations is not None else None
            ),
            timestamps=self.timestamps[start_idx:end_idx] - self.timestamps[start_idx],
            dt=self.dt,
            episode_idx=self.episode_idx,
            frame_id=self.frame_id,
        )

    def _slice(self, indices: list[int]) -> CartesianTrajectory:
        """按索引列表切片喵~"""
        return CartesianTrajectory(
            positions=self.positions[indices],
            orientations=(
                self.orientations[indices]
                if self.orientations is not None else None
            ),
            timestamps=self.timestamps[indices],
            dt=self.dt,
            episode_idx=self.episode_idx,
            frame_id=self.frame_id,
        )
