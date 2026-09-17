#!/usr/bin/env python3
"""
全量录制：计算图通配发现订阅，域内全部话题自动进会话 bag.

设计（09-17 用户口径"整个 peach 过程全量录制，便于仿真复现与问题分析"）：
- 不枚举话题——activate 后周期扫描 `get_topic_names_and_types()`，新话题
  自动建 bag 专用订阅（消息类型运行期 import），删掉话题不再重订。
- 大流护栏：camera raw 图像/点云（/camera/ 前缀 + image/points/depth）在
  `level=std` 档限 1Hz（首帧必录 + 周期限速）；`level=all` 不限速（真全量，
  stereo 13.4fps 约 50MB/s，20GB 预算约 1.7h，超预算靠既有 retention 回收）。
- 排除：/rosout（专用订阅已录）、/parameter_events（噪声）。
"""
from __future__ import annotations

import threading
import time

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

EXCLUDE_TOPICS = {'/rosout', '/parameter_events', '/clock'}
RAW_PATTERNS = ('/image_raw', '/image_rect', '/points', '/depth', '/ir')
SCAN_PERIOD_S = 2.0


def _import_message(type_name: str):
    """'pkg/msg/Type' → 消息类（rosidl 运行期 import；失败返回 None）."""
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_name)
    except Exception:  # noqa: BLE001 未安装/类型失效
        return None


def _is_raw_camera(topic: str) -> bool:
    return topic.startswith('/camera/') and any(
        p in topic for p in RAW_PATTERNS)


class CatchAllRecorder:
    """通配话题发现 → bag 专用订阅（线程安全，activate 期启动）."""

    def __init__(self, node, recorder, level: str = 'std'):
        self._node = node
        self._recorder = recorder
        self._level = level
        self._subs = []
        self._recorded = set()
        self._last_raw_ts: dict[str, float] = {}
        self._timer = None
        self._lock = threading.Lock()

    def start(self) -> None:
        """启动：立即扫一遍计算图 + 周期发现新话题（activate 期调用）."""
        self._scan()
        self._timer = self._node.create_timer(SCAN_PERIOD_S, self._scan)

    def stop(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        with self._lock:
            self._subs.clear()

    def _allow(self, topic: str) -> bool:
        if topic in EXCLUDE_TOPICS or topic in self._recorded:
            return False
        if self._level == 'core':
            return False
        return True

    def _scan(self) -> None:
        try:
            names = self._node.get_topic_names_and_types()
        except Exception:  # noqa: BLE001 计算图查询失败下轮重试
            return
        for topic, types in names:
            if not self._allow(topic) or not types:
                continue
            msg_type = _import_message(types[0])
            if msg_type is None:
                continue
            sub = self._make_subscription(topic, msg_type)
            if sub is not None:
                with self._lock:
                    self._recorded.add(topic)
                    self._subs.append(sub)
                self._node.get_logger().info(
                    f'全量录制订阅: {topic} [{types[0]}]')

    def _make_subscription(self, topic: str, msg_type):
        # 端到端 QoS 兼容优先：传感器流 BEST_EFFORT 也能录；unknown 兜底 RELIABLE
        qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        try:
            sub = self._node.create_subscription(
                msg_type, topic, self._cb(topic), qos)
        except Exception:  # noqa: BLE001 类型/QoS 不兼容则按 RELIABLE 重试
            try:
                qos.reliability = ReliabilityPolicy.RELIABLE
                sub = self._node.create_subscription(
                    msg_type, topic, self._cb(topic), qos)
            except Exception as error:  # noqa: BLE001
                self._node.get_logger().warning(
                    f'全量录制订阅失败 {topic}: {error}')
                return None
        return sub

    def _cb(self, topic: str):
        def callback(message) -> None:
            if self._recorder is None:
                return
            if self._level != 'all' and _is_raw_camera(topic):
                now = time.monotonic()
                if now - self._last_raw_ts.get(topic, 0.0) < 1.0:
                    return  # std 档：相机 raw 大流限 1Hz（首帧必录）
                self._last_raw_ts[topic] = now
            try:
                self._recorder.handle_raw(topic, message)
            except Exception as error:  # noqa: BLE001 录制失败不拖垮订阅
                self._node.get_logger().warning(
                    f'全量录制写袋失败 {topic}: {error}')
        return callback
