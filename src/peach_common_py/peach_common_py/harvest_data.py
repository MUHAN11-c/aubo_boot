"""
采摘运行数据管理：manifest、事件流和目标掩膜（零 ROS import）.

职责:
  每轮采摘的轻量可查询事件库（自 peach_scene_perception.harvest_data 迁移，
  重构阶段 A1）；RGB-D 大数据仍由重建 session 保存。

输入/输出契约:
  start() 创建运行目录并原子写 manifest.yaml；attach() 附着到既有目录
  供重建进程追加同一事件链；append_event() 追加 events.jsonl 并刷新
  latest_<source>.json；save_mask() 落 mono8 PNG 掩膜（按目标节流）。

修复记录（A1）:
  events.jsonl 追加加 fcntl.flock 排他锁——感知与重建两个进程 attach 同一
  run_dir 并发追加时，原实现无锁可能交错写出行撕裂（O_APPEND 只保证单次
  write 原子，但标准库文本 IO 有用户态缓冲，一次 write 调用可能被拆分）。
  flock 为建议锁，全链路（感知/重建/工具）都走本类即互斥；POSIX 专用
  （项目目标平台仅 Ubuntu）。

协议条款:
  纯核零 ROS import（test_pure_core.py AST 强制）；cv2/numpy/yaml 为
  纯算法/序列化依赖，放行。

线程模型:
  实例无线程同步，同一实例须单线程使用；跨线程/跨进程并发追加由
  events.jsonl 的 flock 保证（各自打开独立文件描述符即互相阻塞）。
"""
from __future__ import annotations

from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import yaml


def default_harvest_root() -> Path:
    """解析 harvest_runs 根目录，可由 AUBO_HARVEST_DATA_DIR 覆盖."""
    override = os.environ.get('AUBO_HARVEST_DATA_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'peach_scene_perception').is_dir():
            return parent / 'harvest_runs'
    return Path.cwd() / 'harvest_runs'


class HarvestDataStore:
    """每轮采摘的轻量可查询事件库；RGB-D 大数据仍由重建 session 保存."""

    def __init__(self, root=None):
        """创建尚未开始的存储器."""
        self.root = Path(root) if root else default_harvest_root()
        self.run_dir = None
        self.latest_state = {}
        # target_id → 上次掩膜落盘的 time.monotonic() 时刻（save_mask 节流用）
        self._mask_last_saved = {}

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
        """
        追加 JSONL 事件并刷新 latest_state.json.

        events.jsonl 追加持 fcntl 排他锁：感知/重建双进程 attach 同一
        run_dir 并发写时互斥，防止行撕裂（修复 A1 前双进程追加无锁）。
        """
        if self.run_dir is None:
            return
        record = dict(event)
        record.setdefault(
            'recorded_at', datetime.now(timezone.utc).isoformat())
        with (self.run_dir / 'events.jsonl').open(
                'a', encoding='utf-8') as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                stream.write(json.dumps(record, ensure_ascii=False) + '\n')
                # flush 在锁内完成，保证解锁前数据已入内核页缓存
                stream.flush()
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        self.latest_state = record
        source = str(record.get('source', 'perception'))
        tmp = self.run_dir / f'latest_{source}.json.tmp'
        tmp.write_text(
            json.dumps(record, ensure_ascii=False, indent=2),
            encoding='utf-8')
        tmp.replace(self.run_dir / f'latest_{source}.json')

    def save_mask(self, target_id: str, stamp_ns: int,
                  mask: np.ndarray, min_interval_s: float = 1.0) -> str:
        """
        保存选中目标的 mono8 PNG 掩膜并返回相对路径.

        每目标按 monotonic 时钟节流（间隔 < min_interval_s 直接返回 ''），
        防长观测期 masks/ 文件数无界；写失败仍抛 OSError 由调用方记日志。
        """
        if self.run_dir is None or mask is None:
            return ''
        now = time.monotonic()
        last = self._mask_last_saved.get(target_id)
        if last is not None and now - last < min_interval_s:
            return ''
        binary = (np.asarray(mask) > 0).astype(np.uint8) * 255
        path = self.run_dir / 'masks' / f'{stamp_ns}_{target_id}.png'
        if not cv2.imwrite(str(path), binary):
            raise OSError(f'掩膜保存失败: {path}')
        # 仅写成功才记录时刻：失败帧下一帧可立即重试
        self._mask_last_saved[target_id] = now
        return str(path.relative_to(self.run_dir))

    def query(self) -> dict:
        """返回当前运行路径与最后事件，供 ROS 查询服务/状态话题复用."""
        return {
            'run_dir': '' if self.run_dir is None else str(self.run_dir),
            'latest': dict(self.latest_state),
        }
