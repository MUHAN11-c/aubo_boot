"""
会话级 MCAP 过程记录器：整栈启停绑定，单流 bag 替代旧 9 路 jsonl 落盘.

生命周期（决策 0019）：节点 on_configure 开 ``runs/session_<时间戳>/bag/``
（rosbag2_py SequentialWriter + MCAP），节点关闭（destroy/shutdown）排空
队列后收尾 bag 并返回目录供自动报告与体积回收。批次边界不再由记录器
开合——events/state 消息自带 request_id，报告侧按其分组还原。

rosbag2_py/rclpy.serialization 均在写线程内延迟导入：本模块可在无 ROS
上下文下被导入与单测。
"""

from __future__ import annotations

from pathlib import Path
import queue
import threading
import time


def session_folder(root, now: float | None = None) -> Path:
    """会话目录命名：<root>/session_<yyyyMMdd_HHmmss>（bag 在其 bag/ 子目录）."""
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now or time.time()))
    return Path(root) / f'session_{stamp}'


def message_type_name(message) -> str:
    """消息实例 → rosbag2 类型串（pkg/msg/Type），供 TopicMetadata 用."""
    cls = type(message)
    return f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'


class Recorder:
    """会话 bag 写入器：回调只入队，唯一执行盘写的是守护写线程."""

    def __init__(self, root_dir='runs', enabled: bool = True,
                 on_info=None, log_warning=lambda msg: None,
                 now: float | None = None):
        """初始化并启动写线程；enabled 时入队开会话 bag（on_info 回调状态）."""
        self._root = Path(root_dir)
        self._enabled = bool(enabled)
        self._on_info = on_info or (lambda info: None)
        self._log_warning = log_warning
        self._lock = threading.Lock()
        self._queue: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._session_dir: Path | None = None
        self._bag_dir: Path | None = None
        self._open_failed = False
        self._thread = threading.Thread(
            target=self._writer_loop, name='peach-web-recorder', daemon=True)
        self._thread.start()
        if self._enabled:
            self._queue.put(('open', now))
        self._on_info(self.info())

    @property
    def enabled(self) -> bool:
        """录制总开关（on_activate 全量录制门控读用）."""
        return self._enabled

    # ------------------------------------------------------------------
    # observability 回调入口（只入队，绝不阻塞）
    # ------------------------------------------------------------------
    def info(self) -> dict:
        """当前记录状态（前端状态栏显示用；directory=bag URI）."""
        with self._lock:
            return {
                'enabled': self._enabled,
                'directory': str(self._bag_dir) if self._bag_dir else None,
                'session': str(self._session_dir) if self._session_dir else None,
            }

    def handle_raw(self, topic: str, message,
                   timestamp_ns: int | None = None) -> None:
        """原始消息入队写 bag；未启用或打开失败时静默丢弃（监控优先）."""
        if not self._enabled or self._open_failed:
            return
        self._queue.put((
            'msg', str(topic), message,
            int(timestamp_ns if timestamp_ns is not None else time.time_ns())))

    def close(self) -> Path | None:
        """排空队列并收尾 bag；返回 bag 目录（未启用/打开失败给 None）."""
        if self._enabled:
            self._queue.put(('close',))
            self._queue.join()
        self._stop.set()
        self._thread.join(timeout=10.0)
        with self._lock:
            return self._bag_dir

    # ------------------------------------------------------------------
    # 写线程：唯一执行盘写的地方
    # ------------------------------------------------------------------
    def _writer_loop(self) -> None:
        """顺序处理 open/msg/close；写盘异常只降级告警，线程不死."""
        writer = None
        created: set[str] = set()
        while True:
            try:
                job = self._queue.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():
                    break
                continue
            try:
                kind = job[0]
                if kind == 'open':
                    writer = self._open_bag(writer)
                elif kind == 'msg':
                    writer = self._write_message(writer, created, job)
                elif kind == 'close':
                    self._close_writer(writer)
                    writer = None
            except Exception as error:  # 单任务失败只告警，线程不死
                self._log_warning(f'bag 记录任务失败（跳过）: {error}')
            finally:
                self._queue.task_done()
        if writer is not None:  # close 任务未达（异常路径）的兜底收尾
            self._close_writer(writer)

    def _open_bag(self, writer):
        """
        开会话 bag；重复调用或已失败则跳过；成功后回调 info.

        bag 目录本身由 rosbag2 创建（writer.open 拒绝已存在的目录），这里
        只保证会话层目录存在；同秒重启等命名冲突时追加序号后移。
        """
        if writer is not None or self._open_failed:
            return writer
        from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions

        session = session_folder(self._root)
        while (session / 'bag').exists():
            session = session.with_name(f'{session.name}_x')
        bag_dir = session / 'bag'
        try:
            session.mkdir(parents=True, exist_ok=True)
            writer = SequentialWriter()
            writer.open(
                StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'))
        except Exception as error:
            self._open_failed = True
            self._log_warning(f'会话 bag 打开失败，本会话不录制: {error}')
            return None
        with self._lock:
            self._session_dir = session
            self._bag_dir = bag_dir
        self._on_info(self.info())
        return writer

    def _write_message(self, writer, created: set, job):
        """单条消息写 bag；话题首见时建 TopicMetadata（类型取自消息类）."""
        if writer is None:
            return writer
        from rosbag2_py import TopicMetadata
        from rclpy.serialization import serialize_message

        _, topic, message, stamp_ns = job
        if topic not in created:
            writer.create_topic(TopicMetadata(
                id=0, name=topic, type=message_type_name(message),
                serialization_format='cdr'))
            created.add(topic)
        writer.write(topic, serialize_message(message), stamp_ns)
        return writer

    def _close_writer(self, writer) -> None:
        """收尾 bag（metadata.yaml 落盘）；重复关闭无害."""
        if writer is None:
            return
        try:
            writer.close()
        except Exception as error:  # 收尾失败不掩饰：报告侧会走 reindex 兜底
            self._log_warning(f'bag 收尾异常（报告侧将尝试 reindex）: {error}')
