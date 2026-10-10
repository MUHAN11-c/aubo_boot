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


# O1（2026-09-24 S3 真相机轮勘定：26G bag 构成=高频控制流族非相机）：
# 关节/动态 TF/CM introspection/关节状态高频不减信息量地撑大 bag，按话题
# 统一限 20Hz（首帧必录）——周期节拍与 TCP 轨迹离线重算足够分辨；
# /tf_static 例外（启动期一批互不相同的静态变换，限速会丢 child frame）。
# S4 轮（2026-09-24）实测补齐漏网：statistics/values 两族 20 万条、
# dynamic_joint_states 与 JTC controller_state 各 10 万条、moveit_servo
# 周期性重发 PlanningScene 1.7 万条——全部归入限速族。
CONTROL_FLOW_TOPICS = {
    '/joint_states', '/tf', '/diagnostics', '/dynamic_joint_states',
}
CONTROL_FLOW_SUBSTRINGS = (
    'introspection', 'joint_status', 'io_states', 'statistics',
    'controller_state',
)
CONTROL_FLOW_MIN_INTERVAL_S = 0.05
# PlanningScene 族（moveit_servo 24Hz 重发 + monitored 3.8Hz）：有界世界
# 状态非节拍流，20Hz 限速几乎不减量——单独 1Hz 档（首帧必录）。
PLANNING_SCENE_TOPICS = {
    '/moveit_servo/publish_planning_scene', '/monitored_planning_scene',
}
PLANNING_SCENE_MIN_INTERVAL_S = 1.0
# 感知派生诊断族（S4 轮实测占袋 79%：debug_image 3.0G+raw 1.9G+
# target_observations 1.2G+masks 0.9G / 12min）：进袋 2Hz 已足够复盘相变
# （密集视觉证据另有 45s 截图与 RViz 视频）；实时流不受影响，仅限进袋。
DERIVED_STREAM_SUBSTRINGS = (
    'debug_image', 'masks', 'target_observations',
)
DERIVED_STREAM_MIN_INTERVAL_S = 0.5


class Recorder:
    """会话 bag 写入器：回调只入队，唯一执行盘写的是守护写线程."""

    def __init__(self, root_dir='runs', enabled: bool = True,
                 on_info=None, log_warning=lambda msg: None,
                 now: float | None = None, queue_depth: int = 512):
        """
        初始化并启动写线程；enabled 时入队开会话 bag（on_info 回调状态）.

        queue_depth 给写队列上界（drop-oldest）：'all' 档 ~50MB/s 时盘速
        掉队不再无界吃内存，丢最旧保最新并计数（info().drops 供诊断）。
        """
        self._root = Path(root_dir)
        self._enabled = bool(enabled)
        self._on_info = on_info or (lambda info: None)
        self._log_warning = log_warning
        self._lock = threading.Lock()
        self._queue: queue.Queue = queue.Queue(
            maxsize=max(1, int(queue_depth)))
        self._drops = 0
        self._drop_warn_t = 0.0
        self._throttle_ts: dict[str, float] = {}  # O1 控制流族限速时间戳
        self._stop = threading.Event()
        self._closed = threading.Event()
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

    @property
    def session_dir(self) -> Path | None:
        """当前会话目录（startup.json/selfcheck.json 等会话工件写于此）."""
        with self._lock:
            return self._session_dir

    # ------------------------------------------------------------------
    # observability 回调入口（只入队，绝不阻塞）
    # ------------------------------------------------------------------
    def info(self) -> dict:
        """当前记录状态（前端状态栏/诊断用；directory=bag URI）."""
        with self._lock:
            return {
                'enabled': self._enabled,
                'directory': str(self._bag_dir) if self._bag_dir else None,
                'session': str(self._session_dir) if self._session_dir else None,
                'queue_size': self._queue.qsize(),
                'queue_depth': self._queue.maxsize,
                'drops': self._drops,
            }

    def handle_raw(self, topic: str, message,
                   timestamp_ns: int | None = None) -> None:
        """原始消息入队写 bag；未启用或打开失败时静默丢弃（监控优先）."""
        if not self._enabled or self._open_failed:
            return
        topic = str(topic)
        if self._control_flow_throttled(topic):
            return
        self._enqueue((
            'msg', topic, message,
            int(timestamp_ns if timestamp_ns is not None else time.time_ns())))

    def _control_flow_throttled(self, topic: str) -> bool:
        """限速族：控制流 20Hz / 派生诊断 2Hz / PlanningScene 1Hz；其余全量."""
        if topic == '/tf_static':
            return False
        if topic in PLANNING_SCENE_TOPICS:
            min_interval = PLANNING_SCENE_MIN_INTERVAL_S
        elif any(s in topic for s in DERIVED_STREAM_SUBSTRINGS):
            min_interval = DERIVED_STREAM_MIN_INTERVAL_S
        elif topic in CONTROL_FLOW_TOPICS or any(
            s in topic for s in CONTROL_FLOW_SUBSTRINGS
        ):
            min_interval = CONTROL_FLOW_MIN_INTERVAL_S
        else:
            return False
        now = time.monotonic()
        if now - self._throttle_ts.get(topic, 0.0) < min_interval:
            return True
        self._throttle_ts[topic] = now
        return False

    def _enqueue(self, item) -> None:
        """有界入队：满时丢最旧保最新（宁丢旧帧不撑内存）."""
        try:
            self._queue.put_nowait(item)
            return
        except queue.Full:
            pass
        try:
            self._queue.get_nowait()
            # 被丢的那条也要销账，否则 close() 的 queue.join() 永远等不齐
            self._queue.task_done()
            self._queue.put_nowait(item)
        except (queue.Empty, queue.Full):
            pass
        self._note_drop()

    def _put_control(self, item) -> bool:
        """open/close 必须入队：满则丢最旧腾位，禁止阻塞 put（SIGINT 挂死源）."""
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            pass
        try:
            self._queue.get_nowait()
            self._queue.task_done()
            self._queue.put_nowait(item)
            self._note_drop()
            return True
        except (queue.Empty, queue.Full):
            self._log_warning('bag 控制任务未能入队（队列满且腾位失败）')
            return False

    def _note_drop(self) -> None:
        """丢帧计数 + 10s 节流告警（持续掉队=盘速不足，值得看见）."""
        self._drops += 1
        now = time.monotonic()
        if now - self._drop_warn_t >= 10.0:
            self._drop_warn_t = now
            self._log_warning(
                f'bag 写队列掉队丢帧：累计 {self._drops}'
                '（盘速不足，丢最旧保最新）')

    def close(self, timeout_s: float = 8.0) -> Path | None:
        """排空队列并收尾 bag；有界等待，避免 SIGINT 时 queue.join 永久挂死."""
        budget = max(0.1, float(timeout_s))
        if self._enabled:
            self._put_control(('close',))
            if not self._closed.wait(timeout=budget):
                self._log_warning(
                    f'bag 收尾超时 {budget:.1f}s，强制停写线程')
        self._stop.set()
        self._thread.join(timeout=min(2.0, budget))
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
                    self._closed.set()
            except Exception as error:  # 单任务失败只告警，线程不死
                self._log_warning(f'bag 记录任务失败（跳过）: {error}')
            finally:
                self._queue.task_done()
        if writer is not None:  # close 任务未达（异常路径）的兜底收尾
            self._close_writer(writer)
        self._closed.set()

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
