"""采摘运行数据管理：manifest、事件流和目标掩膜（零 ROS import）."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path

import cv2
import numpy as np
import yaml


def default_harvest_root() -> Path:
    """解析 harvest_runs 根目录，可由 AUBO_HARVEST_DATA_DIR 覆盖."""
    override = os.environ.get('AUBO_HARVEST_DATA_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'peach_pose_ros2').is_dir():
            return parent / 'harvest_runs'
    return Path.cwd() / 'harvest_runs'


class HarvestDataStore:
    """每轮采摘的轻量可查询事件库；RGB-D 大数据仍由重建 session 保存."""

    def __init__(self, root=None):
        """创建尚未开始的存储器."""
        self.root = Path(root) if root else default_harvest_root()
        self.run_dir = None
        self.latest_state = {}

    def start(self, run_id: str, manifest: dict) -> Path:
        """创建运行目录并原子写 manifest.yaml."""
        self.run_dir = self.root / run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        (self.run_dir / 'masks').mkdir()
        document = dict(manifest)
        document['harvest_run_id'] = run_id
        document['created_at'] = datetime.now(timezone.utc).isoformat()
        tmp = self.run_dir / 'manifest.yaml.tmp'
        tmp.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
            encoding='utf-8')
        tmp.replace(self.run_dir / 'manifest.yaml')
        self.latest_state = document
        return self.run_dir

    def attach(self, run_id: str) -> bool:
        """附着到既有运行目录，供重建进程追加同一事件链."""
        candidate = self.root / run_id
        if not run_id or not candidate.is_dir():
            return False
        self.run_dir = candidate
        return True

    def append_event(self, event: dict) -> None:
        """追加 JSONL 事件并刷新 latest_state.json."""
        if self.run_dir is None:
            return
        record = dict(event)
        record.setdefault(
            'recorded_at', datetime.now(timezone.utc).isoformat())
        with (self.run_dir / 'events.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.latest_state = record
        source = str(record.get('source', 'perception'))
        tmp = self.run_dir / f'latest_{source}.json.tmp'
        tmp.write_text(
            json.dumps(record, ensure_ascii=False, indent=2),
            encoding='utf-8')
        tmp.replace(self.run_dir / f'latest_{source}.json')

    def save_mask(self, target_id: str, stamp_ns: int,
                  mask: np.ndarray) -> str:
        """保存选中目标的 mono8 PNG 掩膜并返回相对路径."""
        if self.run_dir is None or mask is None:
            return ''
        binary = (np.asarray(mask) > 0).astype(np.uint8) * 255
        path = self.run_dir / 'masks' / f'{stamp_ns}_{target_id}.png'
        if not cv2.imwrite(str(path), binary):
            raise OSError(f'掩膜保存失败: {path}')
        return str(path.relative_to(self.run_dir))

    def query(self) -> dict:
        """返回当前运行路径与最后事件，供 ROS 查询服务/状态话题复用."""
        return {
            'run_dir': '' if self.run_dir is None else str(self.run_dir),
            'latest': dict(self.latest_state),
        }
