"""时钟、有界 worker、runs 落盘、标量 EMA（纯核，零 ROS）."""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from datetime import datetime, timezone
import fcntl
import json
import logging
import os
from pathlib import Path
import queue
import threading
import time
import traceback
from typing import (
    Callable,
    Generic,
    Optional,
    TypeVar,
)

import cv2
import numpy as np
from peach_common.paths import safe_component
import yaml

# 纯核不能 import ROS，走 stdlib logging（print 会污染 stdout）
_logger = logging.getLogger(__name__)

Item = TypeVar('Item')


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

    V4（异步落盘）：append_event/save_mask 的文件 IO 经单后台写线程 +
    ``queue.Queue(maxsize=512)``——感知 plan_lock 与重建采帧链不再被磁盘
    抖动拖住（旧版同步写在 plan_lock 持有区内逐帧 imwrite/追加 JSONL）。
    队列满时同步降级写（极端背压下短暂回到旧行为，事件不丢；此刻调用
    方与写线程瞬时双写，events.jsonl 的 fcntl 排他锁仍保证行不撕裂，
    但极端下两条事件可能小幅乱序）。事件顺序在常态下单写者保持 FIFO。
    close(drain=True) 排空队列（节点 destroy 前调用）。
    """

    _WRITE_QUEUE_MAX = 512

    def __init__(self, root=None, base_dir=None):
        """创建尚未开始的存储器；base_dir 由节点按 executor run_id 设置."""
        self.root = Path(root) if root else default_harvest_root()
        self.base_dir = Path(base_dir) if base_dir else None
        self.run_dir = None
        self.latest_state = {}
        # target_id → 上次掩膜落盘的 time.monotonic() 时刻（save_mask 节流用）
        self._mask_last_saved = {}
        # V4：单后台写线程（daemon；close() 排空收口）
        self._write_queue: queue.Queue = queue.Queue(maxsize=self._WRITE_QUEUE_MAX)
        self._writer = threading.Thread(
            target=self._write_loop, name='harvest-data-writer', daemon=True)
        self._writer.start()

    def _write_loop(self) -> None:
        """后台写线程：FIFO 消费写队列；单条失败记日志不杀线程."""
        while True:
            item = self._write_queue.get()
            try:
                if item is None:
                    return
                self._write_item(item)
            except Exception:  # noqa: BLE001 写线程不得死：单条失败记日志继续
                _logger.error('落盘写线程任务异常:\n%s', traceback.format_exc())
            finally:
                self._write_queue.task_done()

    def _write_item(self, item) -> None:
        """执行一条写任务（事件追加或掩膜 PNG；worker 与降级路径共用）."""
        kind, payload = item
        if kind == 'event':
            record, run_dir = payload
            self._append_event_sync(record, run_dir)
        else:
            path, binary = payload
            if not cv2.imwrite(str(path), binary):
                _logger.error('掩膜保存失败: %s', path)

    def _enqueue_write(self, item) -> None:
        """入队；队列满时同步降级写（事件不丢）."""
        try:
            self._write_queue.put_nowait(item)
        except queue.Full:
            self._write_item(item)

    def close(self, *, drain: bool = True) -> None:
        """
        停止写线程（V4）.

        drain=True 先排空队列再收口（节点 destroy 用）；drain=False 直接
        放弃未写条目（进程即将退出、容忍丢失时用）。
        """
        if not self._writer.is_alive():
            return
        if drain:
            self._write_queue.join()
        try:
            self._write_queue.put_nowait(None)
        except queue.Full:  # pragma: no cover - join 后必有空位，防御
            return
        self._writer.join(timeout=5.0)

    def _resolve(self, run_id: str) -> Path:
        """
        轮目录：批次在跑=base_dir/run_id，否则 root/run_id（旧布局）.

        run_id 是消息来源（executor 广播），入路径前经 safe_component
        净化（W1 路径穿越修复：拒绝分隔符/上跳/NUL，非法折叠 fallback）。
        """
        base = self.base_dir if self.base_dir is not None else self.root
        return base / safe_component(run_id, 'run')

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

    def start_harvest_run(self, plan, params, executor_run_id: str = '') -> str:
        """
        为刚锁定的目标集合创建轮目录与 manifest（W3 自节点下沉的领域方法）.

        批次在跑：轮目录落 runs/<request_id>/perception_data/<轮ID>；无批次
        回退旧布局（root/<轮ID>）。request_id 为消息来源，入路径前经
        safe_component 净化（W1 路径穿越修复）。调用方（节点）须在
        plan_lock 持有区调用；轮 ID 格式与拆分前逐字一致。

        Args:
            plan: GlobalHarvestPlan（读 snapshot/targets/selected）.
            params: ScenePerceptionParams（读版本与 output_frame）.
            executor_run_id: 调度 HarvestState.run_id；空串=无批次.

        Returns
        -------
            生成的 harvest_run_id（同时写入 self.run_dir 与 manifest）.

        """
        if executor_run_id:
            self.base_dir = (
                default_runs_root()
                / safe_component(executor_run_id, 'harvest')
                / 'perception_data')
        else:
            self.base_dir = None
        now = datetime.now()
        run_id = (
            f'harvest_{now.strftime("%Y%m%dT%H%M%S_%f")}_s{plan.snapshot_id}')
        targets = [
            {'target_id': target_id, 'priority': plan.priority(target_id)}
            for target_id in plan.locked_ids
        ]
        self.start(run_id, {
            'snapshot_id': plan.snapshot_id,
            'target_count': plan.target_count,
            'selected_target_id': plan.selected_target_id,
            'targets': targets,
            'model_version': params.model_version,
            'calibration_version': params.calibration_version,
            'output_frame': params.output_frame,
        })
        self.append_event({
            'source': 'perception', 'event': 'global_targets_locked',
            'target_count': plan.target_count,
            'selected_target_id': plan.selected_target_id,
        })
        return run_id

    def append_event(self, event: dict) -> None:
        """
        追加 JSONL 事件并刷新 latest_state.json（V4：经写队列异步落盘）.

        events.jsonl 追加持 fcntl 排他锁：感知/重建双进程 attach 同一
        run_dir 并发写时互斥，防止行撕裂（修复 A1 前双进程追加无锁）。
        recorded_at 在调用线程取（入队序=事件序）；latest_state 随写线程
        完成后刷新，query() 可能滞后一条（诊断通道，可接受）。
        """
        if self.run_dir is None:
            return
        record = dict(event)
        record.setdefault(
            'recorded_at', datetime.now(timezone.utc).isoformat())
        self._enqueue_write(('event', (record, self.run_dir)))

    def _append_event_sync(self, record: dict, run_dir: Path) -> None:
        """事件落盘本体（写线程/降级路径执行；与旧同步版逐字一致）."""
        with (run_dir / 'events.jsonl').open(
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
        tmp = run_dir / f'latest_{source}.{token}.json.tmp'
        try:
            tmp.write_text(
                json.dumps(record, ensure_ascii=False, indent=2),
                encoding='utf-8')
            tmp.replace(run_dir / f'latest_{source}.json')
        except OSError:
            # 并发同名 tmp 或 run_dir 已切走：events.jsonl 已落，latest 可丢
            try:
                tmp.unlink(missing_ok=True)
            except OSError:
                pass

    def save_mask(self, target_id: str, stamp_ns: int,
                  mask: np.ndarray, min_interval_s: float = 1.0) -> str:
        """
        保存选中目标的 mono8 PNG 掩膜并返回相对路径（V4：经写队列异步写）.

        每目标按 monotonic 时钟节流（间隔 < min_interval_s 直接返回 ''），
        防长观测期 masks/ 文件数无界；记账移至入队时刻（写失败由写线程
        记日志，不再向调用方抛 OSError——异步路径无法回传异常；节流窗口
        语义不变）。路径与相对名在调用线程按当时 run_dir 计算（事件归属
        入队时的轮目录）。
        """
        if self.run_dir is None or mask is None:
            return ''
        now = time.monotonic()
        last = self._mask_last_saved.get(target_id)
        if last is not None and now - last < min_interval_s:
            return ''
        binary = (np.asarray(mask) > 0).astype(np.uint8) * 255
        path = self.run_dir / 'masks' / f'{stamp_ns}_{target_id}.png'
        relative = str(path.relative_to(self.run_dir))
        self._enqueue_write(('mask', (path, binary)))
        self._mask_last_saved[target_id] = now
        return relative

    def query(self) -> dict:
        """返回当前运行路径与最后事件，供 ROS 查询服务/状态话题复用."""
        return {
            'run_dir': '' if self.run_dir is None else str(self.run_dir),
            'latest': dict(self.latest_state),
        }


class ScalarEma:
    """
    标量指数滑动平均（thread-unsafe：调用方自行保证单线程访问）.

    α 取值口径：0.3 在响应速度与抗单帧抖动间取折中（沿用原
    capture.EMA_ALPHA / integrate._CORR_EMA_ALPHA 注释）。
    首个有效样本直接作初值，其后
    ``value ← α·sample + (1−α)·value``（α 为新样本权重）。
    """

    def __init__(self, alpha: float = 0.3):
        """初始化未播种的 EMA；alpha 为新样本权重 ∈ (0, 1]."""
        self._alpha = float(alpha)
        self._value: Optional[float] = None

    def update(self, sample: float) -> float:
        """
        注入一个样本并返回更新后的 EMA；首个样本直接作初值.

        Args:
            sample: 本次样本值（调用方保证非负/有限；脏样本先过滤）.

        Returns
        -------
            更新后的 EMA.

        """
        value = float(sample)
        if self._value is None:
            self._value = value
        else:
            self._value = (self._alpha * value
                           + (1.0 - self._alpha) * self._value)
        return self._value

    def reset(self) -> None:
        """清空播种状态（回到无样本）."""
        self._value = None

    @property
    def value(self) -> Optional[float]:
        """当前 EMA；尚无样本时 None."""
        return self._value

    @property
    def seeded(self) -> bool:
        """是否已有样本（未播种时按调用方约定处理，如视为达标）."""
        return self._value is not None
