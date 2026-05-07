"""HuggingFace 数据集加载器：下载并解析 latte-pour-demos 的 parquet 文件。"""

import os
import numpy as np
import pandas as pd

# HF 代理问题解决
for _key in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "all_proxy", "ALL_PROXY"):
    os.environ.pop(_key, None)

from huggingface_hub import list_repo_files, hf_hub_download  # noqa: E402

DATASET_REPO = "ridxm/latte-pour-demos"
PARQUET_PATTERN = "data/chunk-000/episode_{ep:06d}.parquet"
DEFAULT_FPS = 20.0

# 右臂关节数据切片：14D 中 dims 7..12 是右臂 6 个关节角，dim 13 是夹爪
RIGHT_ARM_JOINTS = slice(7, 13)
RIGHT_ARM_GRIPPER = 13


class DatasetLoader:
    """加载 ridxm/latte-pour-demos 数据集，提取右臂拉花轨迹。"""

    def __init__(self, repo_id=DATASET_REPO, cache_dir=None, fps=DEFAULT_FPS):
        self.repo_id = repo_id
        self.cache_dir = cache_dir
        self.fps = fps
        self.dt = 1.0 / fps
        self._file_list = None

    def list_episodes(self):
        """返回远程仓库中所有可用的 episode 编号。"""
        if self._file_list is None:
            files = list_repo_files(self.repo_id)
            self._file_list = files
        episodes = []
        for f in self._file_list:
            if "data/chunk-000/episode_" in f and f.endswith(".parquet"):
                # 提取 episode 编号
                basename = f.rsplit("/", 1)[-1]
                ep_str = basename.replace("episode_", "").replace(".parquet", "")
                try:
                    episodes.append(int(ep_str))
                except ValueError:
                    pass
        return sorted(episodes)

    def download_episode(self, episode_idx):
        """下载并缓存指定 episode 的 parquet 文件，返回本地路径。"""
        filename = PARQUET_PATTERN.format(ep=episode_idx)
        local_path = hf_hub_download(
            repo_id=self.repo_id,
            filename=filename,
            repo_type="dataset",
            cache_dir=self.cache_dir,
        )
        return local_path

    def load_episode(self, episode_idx):
        """加载一个 episode，返回右臂关节轨迹和元信息。

        Returns:
            dict:
                joint_positions: np.ndarray (T, 6)  右臂 6 关节角 (rad)
                actions:         np.ndarray (T, 6)  动作（关节目标位置）
                timestamps:      np.ndarray (T,)    从 0 开始的秒数
                dt:              float              时间步长 (0.05s)
                episode_index:   int
                num_frames:      int
        """
        local_path = self.download_episode(episode_idx)
        df = pd.read_parquet(local_path)

        # 提取 observation.state: 每行是 numpy array (14,)
        raw_states = np.stack(df["observation.state"].values)  # (T, 14)
        joint_positions = raw_states[:, RIGHT_ARM_JOINTS]       # (T, 6)

        # 提取 action（14D 动作目标）
        raw_actions = np.stack(df["action"].values)             # (T, 14)
        actions = raw_actions[:, RIGHT_ARM_JOINTS]              # (T, 6)

        num_frames = len(raw_states)
        timestamps = np.arange(num_frames) * self.dt

        # 检查夹爪（应近似恒定）
        gripper_values = raw_states[:, RIGHT_ARM_GRIPPER]

        return {
            "joint_positions": joint_positions,
            "actions": actions,
            "timestamps": timestamps,
            "dt": self.dt,
            "episode_index": episode_idx,
            "num_frames": num_frames,
            "gripper_values": gripper_values,
        }

    def load_from_local(self, parquet_path, episode_index=0):
        """从本地 parquet 文件直接加载（无需 HF 下载）。

        Args:
            parquet_path: 本地 .parquet 文件路径
            episode_index: 返回值中的 episode 编号
        """
        return self._parse_parquet(parquet_path, episode_index)

    def _parse_parquet(self, path, episode_index):
        """解析 parquet 文件，提取右臂数据。"""
        df = pd.read_parquet(path)

        raw_states = np.stack(df["observation.state"].values)
        joint_positions = raw_states[:, RIGHT_ARM_JOINTS]

        raw_actions = np.stack(df["action"].values)
        actions = raw_actions[:, RIGHT_ARM_JOINTS]

        num_frames = len(raw_states)
        timestamps = np.arange(num_frames) * self.dt

        return {
            "joint_positions": joint_positions,
            "actions": actions,
            "timestamps": timestamps,
            "dt": self.dt,
            "episode_index": episode_index,
            "num_frames": num_frames,
            "gripper_values": raw_states[:, RIGHT_ARM_GRIPPER],
        }

    def load_all_episodes(self, max_episodes=None):
        """加载所有 episode，返回列表。

        Args:
            max_episodes: 限制最大加载数量 (None = 全部40个)

        Returns:
            list of dict
        """
        episodes = self.list_episodes()
        if max_episodes is not None:
            episodes = episodes[:max_episodes]
        results = []
        for ep in episodes:
            data = self.load_episode(ep)
            results.append(data)
        return results
