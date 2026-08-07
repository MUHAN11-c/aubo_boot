"""
会话数据落盘 — peach_records/<时间戳>/ 三文件 + rosbag2 录制（纯逻辑）.

每个作业会话一个目录：
  targets.jsonl  逐行一个目标档案（注册表快照/状态迁移）
  events.jsonl   逐行一条事件（帧触及、调度、执行结果等）
  summary.json   会话级汇总（结束或按需写入）
  bags/pick_<ts>/  rosbag2 录制（record=true 时 subprocess 拉起，close 停）
record=false 时全部操作退化为 no-op（不落盘）。
"""
import json
from pathlib import Path
import subprocess
import time
from typing import List, Optional

# 录制话题清单（感知/融合/执行/调度/TF 全链路）
BAG_TOPICS = [
    '/peach_pose_node/grasp_candidates',
    '/peach_pose_node/fitting',
    '/recon_refiner_node/refined_candidates',
    '/recon_refiner_node/fusion_stats',
    '/recon_refiner_node/refinement_evolution',
    '/pick_executor/stage_events',
    '/pick_executor/execution_trace',
    '/mission_manager/registry',
    '/mission_manager/stats',
    '/tf',
    '/tf_static',
]


class SessionStore:
    """作业会话记录器：创建会话目录并追加写 JSONL / 汇总 JSON."""

    def __init__(self, root: str = 'peach_records', enabled: bool = True):
        """
        构造记录器.

        Args:
            root: 记录根目录（相对 CWD 或绝对路径）.
            enabled: False 时所有写入为 no-op（不落盘）.

        Returns
        -------
            无返回值（None）.

        """
        self.root = Path(root)
        self.enabled = bool(enabled)
        self.session_dir: Optional[Path] = None
        self._bag_proc: Optional[subprocess.Popen] = None

    def start_session(self, stamp: Optional[float] = None) -> Optional[Path]:
        """
        开新会话：创建 peach_records/<时间戳>/ 并初始化 targets/events 两个 JSONL.

        Args:
            stamp: 会话时刻（epoch 秒）；None 用当前时间.

        Returns
        -------
            会话目录路径；未启用时返回 None.

        """
        if not self.enabled:
            return None
        ts = time.strftime('%Y%m%dT%H%M%S', time.localtime(stamp or time.time()))
        self.session_dir = self.root / ts
        self.session_dir.mkdir(parents=True, exist_ok=True)
        # 先建空文件，保证会话目录三文件齐备（summary.json 由 write_summary 写）
        (self.session_dir / 'targets.jsonl').touch()
        (self.session_dir / 'events.jsonl').touch()
        return self.session_dir

    def log_target(self, record: dict) -> None:
        """
        追加一条目标档案到 targets.jsonl.

        Args:
            record: 可 JSON 序列化的目标档案字典.

        Returns
        -------
            无返回值（None）；未开会话/未启用时忽略.

        """
        self._append('targets.jsonl', record)

    def log_event(self, event: dict) -> None:
        """
        追加一条事件到 events.jsonl.

        Args:
            event: 可 JSON 序列化的事件字典.

        Returns
        -------
            无返回值（None）；未开会话/未启用时忽略.

        """
        self._append('events.jsonl', event)

    def write_summary(self, summary: dict) -> None:
        """
        写会话汇总 summary.json（覆盖写）.

        Args:
            summary: 可 JSON 序列化的汇总字典.

        Returns
        -------
            无返回值（None）；未开会话/未启用时忽略.

        """
        if not self.enabled or self.session_dir is None:
            return
        path = self.session_dir / 'summary.json'
        path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
                        encoding='utf-8')

    def _append(self, name: str, row: dict) -> None:
        """向会话目录的 JSONL 文件追加一行（UTF-8，非 ASCII 原样）."""
        if not self.enabled or self.session_dir is None:
            return
        with (self.session_dir / name).open('a', encoding='utf-8') as f:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')

    def start_bag(self, topics: Optional[List[str]] = None) -> bool:
        """
        拉起 rosbag2 录制子进程（bags/pick_<时间戳>，话题清单见 BAG_TOPICS）.

        Args:
            topics: 话题清单；None 用 BAG_TOPICS.

        Returns
        -------
            True=已拉起；未启用/未开会话/拉起失败给 False.

        """
        if not self.enabled or self.session_dir is None:
            return False
        out = self.session_dir / 'bags' / time.strftime('pick_%Y%m%dT%H%M%S')
        out.parent.mkdir(parents=True, exist_ok=True)
        cmd = ['ros2', 'bag', 'record', '-o', str(out)] + list(
            topics or BAG_TOPICS)
        try:
            self._bag_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, FileNotFoundError):
            self._bag_proc = None
            return False
        return True

    def stop_bag(self) -> None:
        """停止 rosbag2 录制（terminate → 超时 kill）."""
        if self._bag_proc is None:
            return
        self._bag_proc.terminate()
        try:
            self._bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._bag_proc.kill()
            self._bag_proc.wait(timeout=5.0)
        self._bag_proc = None

    def close(self, summary: dict) -> None:
        """
        会话收尾：写 summary.json + 停 rosbag2.

        Args:
            summary: 可 JSON 序列化的汇总字典.

        Returns
        -------
            无返回值（None）.

        """
        self.write_summary(summary)
        self.stop_bag()
