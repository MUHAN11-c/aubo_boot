# Copyright 2026 wjz
"""
HTTP 与 ROS 回调之间的线程安全最新值缓存（DashboardState）.

线程模型与拷贝契约（阶段 H 效率项 2.13：/api/state 浅拷贝）：
- 写侧（ROS 回调/采样线程）一律 copy-on-write：update/update_params 把
  `finite_or_none` 新建出来的整棵叶子对象整体替换进区段，绝不原地改已
  存入的叶子；唯一原地改的是 orchestration.events 列表（追加+截头）。
- 读侧（HTTP 线程 snapshot()）只在短锁内做两层浅拷贝：区段 dict 复制
  一层、events 列表复制一份，叶子值按引用共享；JSON 序列化惰性推迟到
  HTTP 层（http_server 响应时 json.dumps 一次），不再在锁内做
  json 往返深拷贝（旧实现对大目标快照每请求序列化两次）。
- 因此 snapshot() 返回值是**只读**视图：消费方（http_server）拿到后只做
  序列化，禁止原地修改——改叶子会写穿到缓存。API 输出 schema 不变。
"""

from __future__ import annotations

import threading
import time

from .codec import finite_or_none


class DashboardState:
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
            'approach': {
                'status': {},
            },
            'orchestration': {
                'state': {},
                # 批次过程/审计事件时间线（按到达顺序追加，超限截断头部）
                'events': [],
            },
            # 机械臂状态（aubo_msgs/RobotStatus）
            'robot': {
                'status': {},
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

    def append_event(self, value, limit: int = 100) -> None:
        """追加一条批次事件到环形缓冲，只保留最近 limit 条."""
        now = time.time()
        with self._lock:
            events = self._values['orchestration']['events']
            events.append(finite_or_none(value))
            if len(events) > limit:
                del events[:len(events) - limit]
            self._updated['orchestration.events'] = now
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
            return result
