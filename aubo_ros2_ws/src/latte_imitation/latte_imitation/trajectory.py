"""笛卡尔末端轨迹：存储、查询、导出 ROS2 消息。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from glob import glob

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Header


@dataclass
class CartesianTrajectory:
    """笛卡尔末端轨迹 (T 帧，3D 位置 + 4D 四元数姿态)。"""

    positions: np.ndarray     # (T, 3)
    timestamps: np.ndarray    # (T,)
    dt: float
    orientations: np.ndarray = None  # (T, 4) quaternion xyzw
    episode_idx: int = -1
    frame_id: str = "base_link"

    def __post_init__(self):
        self.positions = np.asarray(self.positions, dtype=float)
        self.timestamps = np.asarray(self.timestamps, dtype=float)
        if self.orientations is not None:
            self.orientations = np.asarray(self.orientations, dtype=float)

    # ── IO ──────────────────────────────────────────────────

    @classmethod
    def load(cls, path: str) -> CartesianTrajectory:
        """从 npz 文件加载。"""
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
        """加载 resource/cartesian/{arm}/ 下所有 npz 文件。

        Returns:
            OrderedDict-like: {episode_idx: CartesianTrajectory}
        """
        from collections import OrderedDict
        result = OrderedDict()
        pattern = os.path.join(resource_dir, "cartesian", arm, "episode_*.npz")
        for f in sorted(glob(pattern)):
            basename = os.path.basename(f)
            try:
                ep = int(basename.replace("episode_", "").replace(".npz", ""))
            except ValueError:
                continue
            result[ep] = cls.load(f)
        return result

    def save(self, path: str):
        """保存为 npz 文件。"""
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

    # ── 属性 ────────────────────────────────────────────────

    @property
    def num_frames(self) -> int:
        return len(self.positions)

    @property
    def start(self) -> np.ndarray:
        return self.positions[0]

    @property
    def end(self) -> np.ndarray:
        return self.positions[-1]

    def __len__(self) -> int:
        return self.num_frames

    def __getitem__(self, idx: int) -> np.ndarray:
        return self.positions[idx]

    def segment(self, start_idx: int, end_idx: int) -> CartesianTrajectory:
        return CartesianTrajectory(
            positions=self.positions[start_idx:end_idx],
            orientations=self.orientations[start_idx:end_idx] if self.orientations is not None else None,
            timestamps=self.timestamps[start_idx:end_idx] - self.timestamps[start_idx],
            dt=self.dt, episode_idx=self.episode_idx, frame_id=self.frame_id,
        )

    # ── 统计 ────────────────────────────────────────────────

    def path_length(self) -> float:
        diffs = np.diff(self.positions, axis=0)
        return float(np.sum(np.linalg.norm(diffs, axis=1)))

    def velocity_profile(self) -> np.ndarray:
        diffs = np.diff(self.positions, axis=0)
        return np.linalg.norm(diffs, axis=1) / self.dt

    # ── ROS2 导出 ───────────────────────────────────────────

    def to_pose(self, idx: int) -> Pose:
        p = self.positions[idx]
        pose = Pose(position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        if self.orientations is not None:
            q = self.orientations[idx]
            pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                           z=float(q[2]), w=float(q[3]))
        else:
            pose.orientation.w = 1.0
        return pose

    def to_pose_stamped(self, idx: int, stamp=None) -> PoseStamped:
        ps = PoseStamped(header=Header(frame_id=self.frame_id), pose=self.to_pose(idx))
        if stamp is not None:
            ps.header.stamp = stamp
        return ps

    def to_ros2_path(self, step: int = 5, stamp=None) -> RosPath:
        path = RosPath()
        path.header.frame_id = self.frame_id
        if stamp is not None:
            path.header.stamp = stamp
        for i in range(0, self.num_frames, step):
            path.poses.append(self.to_pose_stamped(i, stamp))
        return path
