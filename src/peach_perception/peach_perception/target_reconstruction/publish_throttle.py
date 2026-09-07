from __future__ import annotations
"""重建发布：诊断状态消息、点云节流与 PublisherMixin、session 落盘.

Marker 构造在 markers.py（namespace 契约不变）；点云消息组装在本模块
xyzrgb_to_cloud_msg。
"""

import time
from typing import (
    Callable,
    Dict,
    Hashable,
    Optional,
)


class PublishThrottle:
    """
    on-change + 最小间隔发布节流器（按话题独立记账）.

    生命周期：随节点构造创建、全程复用；min_interval_s<=0 时间隔门失效
    （只留 on-change），on-change 本身的总开关（publish.on_change_only）
    在编排层判定，不经本类。
    """

    def __init__(self, min_interval_s: float = 0.2,
                 now: Optional[Callable] = None):
        """保存最小间隔与注入时钟；now 为单调秒，None 则用 time.perf_counter."""
        self._min_interval = max(0.0, float(min_interval_s))
        self._now = now if now is not None else time.perf_counter
        # 每话题最近一次「实际发布」的版本 key 与时刻；未发布过无记录
        self._published_key: Dict[str, Hashable] = {}
        self._published_at: Dict[str, float] = {}

    def should_publish(self, topic: str, key: Hashable,
                       force: bool = False) -> bool:
        """
        判定本次是否真正发布.

        Args:
        ----
            topic: 话题标识（记账键，用固定字符串如 'local_cloud'）.
            key: 内容版本 key（可哈希；内容未变须相等，变了须不等——由
                调用方用帧数/版本号等廉价标量组元组，不做内容哈希）.
            force: True 绕过 on-change 与间隔门（产物清空同步事件）.

        Returns
        -------
            True=立即发布（并记录 key 与时刻）；False=抑制（不记录，
            变化留待下次调用补发）.

        """
        if not force:
            if topic in self._published_key \
                    and self._published_key[topic] == key:
                return False  # 零变化：抑制（闩锁保留最后一帧）
            last = self._published_at.get(topic)
            if last is not None and self._now() - last < self._min_interval:
                return False  # 间隔内抑制：不记 key，下次调用仍判为已变化
        self._published_key[topic] = key
        self._published_at[topic] = self._now()
        return True

    def reset(self) -> None:
        """清空全部记账（节点复位/测试隔离用；现状无调用方，接口备用）."""
        self._published_key.clear()
        self._published_at.clear()
