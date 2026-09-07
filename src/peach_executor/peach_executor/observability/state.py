"""Re-export split modules (behavior-preserving shim)."""
from __future__ import annotations

import threading
import time

from peach_executor.observability.codec import finite_or_none
from peach_executor.observability.job import build_harvest_job
from peach_executor.observability.metrics import MetricsSampler

__all__ = ['MetricsSampler', 'ObservabilityState']


class ObservabilityState:
    """
    HTTP 与 ROS 回调之间的线程安全最新值缓存（只读监控，无写入口）.

    写侧 copy-on-write、读侧短锁浅拷贝（见模块 docstring）；snapshot()
    返回值只读，叶子与缓存共享引用。
    """

    def __init__(self):
        """创建空状态."""
        self._lock = threading.Lock()
        self._revision = 0
        self._started = time.time()
        self._values = {
            'perception': {
                'harvest': {},
                'targets': {},
            },
            'reconstruction': {
                'status': {},
                'diagnostics': {},
                'grasp_decision': {},
            },
            'refined': {
                'pose': {},
                'axis': {},
                'diagnostics': {},
            },
            'manipulation': {
                'status': {},
                'hypothesis': {},
            },
            'task_executor': {
                'state': {},
                # 批次过程/审计事件时间线（按到达顺序追加，超限截断头部）
                'events': [],
            },
            # 机械臂状态（aubo_msgs/RobotStatus）+ latest TF 末端摘要
            'robot': {
                'status': {},
                'tcp': {},
            },
            # 系统/GPU/进程性能采样（独立线程写入，单次整体替换）
            'metrics': {
                'sample': {},
            },
            # 监控数据落盘记录器状态（enabled + 当前 run 目录）
            'record': {
                'info': {},
            },
            # 各节点当前参数只读镜像：{节点名: {参数名: 标量值}}
            'params': {},
        }
        self._updated = {}

    def update(self, section: str, key: str, value) -> None:
        """更新一个结构化状态区段."""
        now = time.time()
        with self._lock:
            self._values[section][key] = finite_or_none(value)
            self._updated[f'{section}.{key}'] = now
            self._revision += 1

    def events_snapshot(self) -> list:
        """事件时间线浅拷贝（呈现层选择叠加等只读消费）."""
        with self._lock:
            return list(self._values['task_executor']['events'])

    def append_event(self, value, limit: int = 100) -> None:
        """追加一条批次事件到环形缓冲，只保留最近 limit 条."""
        now = time.time()
        with self._lock:
            events = self._values['task_executor']['events']
            events.append(finite_or_none(value))
            if len(events) > limit:
                del events[:len(events) - limit]
            self._updated['task_executor.events'] = now
            self._revision += 1

    def update_params(self, node_name: str, values: dict) -> None:
        """整体替换一个节点的参数镜像并刷新其时间戳."""
        now = time.time()
        with self._lock:
            self._values['params'][node_name] = finite_or_none(values)
            self._updated[f'params.{node_name}'] = now
            self._revision += 1

    def snapshot(self) -> dict:
        """返回浏览器状态快照与话题年龄（只读浅拷贝视图，禁止原地改叶子）."""
        now = time.time()
        with self._lock:
            # 短锁内两层浅拷贝：区段 dict 复制一层（顶层键替换不写穿），
            # events 列表复制一份（写侧原地追加/截头）；其余叶子按引用
            # 共享，靠写侧 copy-on-write 保证不被改动。序列化惰性留给
            # HTTP 层，省掉旧的 json 往返深拷贝。
            result = {}
            for section, values in self._values.items():
                copied = dict(values)
                events = copied.get('events')
                if isinstance(events, list):
                    copied['events'] = list(events)
                result[section] = copied
            result['system'] = {
                'revision': self._revision,
                'server_time': now,
                'uptime_s': now - self._started,
                'topic_age_s': {
                    key: round(now - stamp, 3)
                    for key, stamp in self._updated.items()},
            }
        result['job'] = build_harvest_job(result)
        return result
