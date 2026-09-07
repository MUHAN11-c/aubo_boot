"""runs/ 落盘：manifest、events.jsonl、节流掩膜 PNG."""
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


def default_runs_root() -> Path:
    """过程数据唯一根目录：工作区 ``runs/``."""
    override = os.environ.get('AUBO_RUNS_DIR') or os.environ.get(
        'AUBO_HARVEST_DATA_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'peach_interfaces').is_dir():
            return parent / 'runs'
    return Path.cwd() / 'runs'


def resolve_runs_root(configured: str = '') -> Path:
    """参数/yaml 给出绝对路径则用之，否则回到 ``default_runs_root()``."""
    text = str(configured or '').strip()
    if text:
        path = Path(text)
        if path.is_absolute():
            return path
    return default_runs_root()


def default_harvest_root() -> Path:
    """兼容旧名，等同 ``default_runs_root()``."""
    return default_runs_root()


class HarvestDataStore:
    """
    每轮采摘的轻量可查询事件库；RGB-D 大数据仍由重建 session 保存.

    单根会话目录（R7）：executor 批次在跑时 base_dir 指向
    ``runs/<request_id>/perception_data``，start/attach 的轮目录落其下；
    base_dir 为 None 时维持旧布局 ``<root>/<run_id>``（无批次回退）。
    """

    def __init__(self, root=None, base_dir=None):
        """创建尚未开始的存储器；base_dir 由节点按 executor run_id 设置."""
        self.root = Path(root) if root else default_harvest_root()
        self.base_dir = Path(base_dir) if base_dir else None
        self.run_dir = None
        self.latest_state = {}
        # target_id → 上次掩膜落盘的 time.monotonic() 时刻（save_mask 节流用）
        self._mask_last_saved = {}

    def _resolve(self, run_id: str) -> Path:
        """轮目录：批次在跑=base_dir/run_id，否则 root/run_id（旧布局）."""
        base = self.base_dir if self.base_dir is not None else self.root
        return base / run_id

    def start(self, run_id: str, manifest: dict) -> Path:
        """创建运行目录并原子写 manifest.yaml."""
        self.run_dir = self._resolve(run_id)
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
        candidate = self._resolve(run_id)
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
        token = f'{os.getpid()}_{time.time_ns()}'
        tmp = self.run_dir / f'latest_{source}.{token}.json.tmp'
        try:
            tmp.write_text(
                json.dumps(record, ensure_ascii=False, indent=2),
                encoding='utf-8')
            tmp.replace(self.run_dir / f'latest_{source}.json')
        except OSError:
            # 并发同名 tmp 或 run_dir 已切走：events.jsonl 已落，latest 可丢
            try:
                tmp.unlink(missing_ok=True)
            except OSError:
                pass

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
