from __future__ import annotations
"""运行时原语：时钟、注册表、有界 worker、runs/ 落盘。"""

from abc import ABC, abstractmethod
from collections import deque
from datetime import datetime, timezone
import fcntl
import json
import logging
import os
from pathlib import Path
import threading
import time
import traceback
from typing import (
    Callable,
    Dict,
    Generic,
    Type,
    TypeVar,
)

import cv2
import numpy as np
import yaml


# === clock.py ===

class Clock(ABC):
    """
    单调时钟抽象基类（协议 I3）.

    生命周期：通常与节点同寿，由编排层构造并注入各纯核组件。
    线程安全：实现方保证 now() 可并发调用。可替换性：真机用
    RclpyClockAdapter，单测/回放开 ManualClock。
    """

    @abstractmethod
    def now(self) -> float:
        """返回当前单调秒（float）."""


class ManualClock(Clock):
    """
    测试用手动时钟：时间只在 advance() 时前进，确定性可复现.

    仅供单线程测试/回放使用（无内部同步）。
    """

    def __init__(self, start: float = 0.0):
        """创建虚拟时钟，起始时刻 start 秒."""
        self._now = float(start)

    def now(self) -> float:
        """返回当前虚拟时刻（不自动前进）."""
        return self._now

    def advance(self, dt: float) -> float:
        """
        虚拟时间前进 dt 秒并返回新时刻.

        Raises
        ------
            ValueError: dt 为负（单调语义禁止倒退）.

        """
        if dt < 0.0:
            raise ValueError(f'ManualClock 禁止倒退: dt={dt}')
        self._now += dt
        return self._now


# === registry.py ===

T = TypeVar('T')


class Registry(Generic[T]):
    """
    按名注册/创建可替换实现的注册表（2.14 装配规则）.

    用法::

        DETECTORS: Registry[Detector] = Registry('检测器')

        @DETECTORS.register('yolo')
        class YoloDetector(Detector):
            ...

        detector = DETECTORS.create('yolo', weights='best.pt')

    生命周期：模块级单例，随进程存活。线程安全：导入期注册、运行期
    只读（见模块 docstring）。
    """

    def __init__(self, kind: str = '组件'):
        """创建空注册表；kind 为错误信息中的类别名（如 '检测器'）."""
        self._kind = kind
        self._items: Dict[str, Type[T]] = {}

    def register(self, name: str, cls: Type[T] = None):
        """
        注册实现类；支持直接调用与装饰器两种写法.

        Args:
            name: 注册名（配置文件中选择实现的键）.
            cls: 实现类；省略时返回装饰器.

        Raises
        ------
            ValueError: 同名重复注册（错误信息含名称）.

        """
        def _do_register(impl: Type[T]) -> Type[T]:
            if name in self._items:
                raise ValueError(
                    f'{self._kind}名称重复注册: {name!r}')
            self._items[name] = impl
            return impl

        if cls is None:
            return _do_register
        return _do_register(cls)

    def create(self, name: str, **kwargs) -> T:
        """
        按名实例化已注册实现，kwargs 透传构造函数.

        Raises
        ------
            KeyError: 名称未注册（错误信息列出全部可用名称）.

        """
        try:
            cls = self._items[name]
        except KeyError:
            available = ', '.join(self.names()) or '（空）'
            raise KeyError(
                f'未注册的{self._kind}: {name!r}，可用: {available}') from None
        return cls(**kwargs)

    def names(self) -> tuple:
        """返回全部已注册名称（排序后的元组，只读）."""
        return tuple(sorted(self._items))

    def __contains__(self, name: str) -> bool:
        """支持 ``name in registry`` 查询."""
        return name in self._items


# === bounded_worker.py ===

# Copyright 2026 wjz


Item = TypeVar('Item')

# 纯核不能 import ROS，走 stdlib logging（print 会污染 stdout）
_logger = logging.getLogger(__name__)


class BoundedWorker(Generic[Item]):
    """在独立线程串行执行任务并显式统计丢弃项."""

    def __init__(
            self, process: Callable[[Item], None], *, capacity: int,
            drop_oldest: bool):
        """创建 worker；容量必须为正数."""
        if capacity < 1:
            raise ValueError('capacity 必须大于零')
        self._process = process
        self._capacity = capacity
        self._drop_oldest = drop_oldest
        self._queue = deque()
        self._condition = threading.Condition()
        self._closing = False
        self._drain = True
        self.dropped = 0
        self._thread = threading.Thread(
            target=self._run, name='bounded-worker', daemon=True)
        self._thread.start()

    def submit(self, item: Item) -> bool:
        """提交任务；满队列时按策略替换旧项或拒绝新项."""
        with self._condition:
            if self._closing:
                return False
            if len(self._queue) >= self._capacity:
                self.dropped += 1
                if not self._drop_oldest:
                    return False
                self._queue.popleft()
            self._queue.append(item)
            self._condition.notify()
            return True

    def close(self, *, drain: bool) -> None:
        """停止接收并等待线程；drain 决定是否处理剩余任务."""
        with self._condition:
            self._closing = True
            self._drain = drain
            if not drain:
                self.dropped += len(self._queue)
                self._queue.clear()
            self._condition.notify_all()
        self._thread.join()

    def _run(self) -> None:
        consecutive_errors = 0
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._queue or self._closing)
                if self._closing and (not self._drain or not self._queue):
                    return
                item = self._queue.popleft()
            # 任务异常不得杀 worker 线程（否则 submit 照常返回 True、节点静默
            # 无输出）：记错误日志（含 traceback）后继续处理后续任务；连续异常
            # 计数用于日志节流——第 1 次必打，之后每 10 次打一次
            try:
                self._process(item)
            except Exception:  # noqa: BLE001
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 10 == 0:
                    _logger.error(
                        'bounded-worker 任务处理异常（连续第 %d 次）:\n%s',
                        consecutive_errors, traceback.format_exc())
            else:
                consecutive_errors = 0


# === harvest_data.py ===

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
