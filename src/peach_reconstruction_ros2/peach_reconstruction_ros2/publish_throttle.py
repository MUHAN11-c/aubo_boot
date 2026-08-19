"""发布节流（协议 2.13-E4）：on-change + 最小间隔（纯核零 ROS，注入时钟）.

职责:
  点云/Marker 类大消息（/peach/reconstruction/local_cloud、tsdf_cloud、
  markers；单次 PointCloud2 可达数万点，逐次重发是发布面主要开销）按
  「内容版本 key」on-change 过滤 + 最小间隔节流：

  - key 不变：一律抑制。三个话题均为 transient_local 闩锁（depth=1），
    订阅者/RViz 保留最后一帧，零变化重发被抑制不会丢显示；
  - key 已变但距上次实际发布不足 min_interval：抑制，且**不记录**新 key
    ——下次调用仍判为已变化，间隔到后补发最新版本（变化只延迟、不丢）；
  - key 已变且间隔已到：放行并记录（key, 时刻）；
  - force=True（_reset_products 产物清空等须立即同步 RViz 的事件）：
    绕过 on-change 与间隔门立即放行。

  心跳/状态/diagnostics/grasp_decision 1Hz 活性三件套与 refit 三件套
  不经过本节流（前者是就绪门/新鲜度门的载体，后者是事件级小消息且承担
  闩锁覆盖防陈旧语义）。

线程模型:
  无内部锁；全部调用发生在节点 _state_lock 持锁段（_publish_all），与
  发布面 mixin 同一并发约定。
"""
from __future__ import annotations

import time
from typing import Callable, Dict, Hashable, Optional


class PublishThrottle:
    """on-change + 最小间隔发布节流器（按话题独立记账）.

    生命周期：随节点构造创建、全程复用；min_interval_s<=0 时间隔门失效
    （只留 on-change），on-change 本身的总开关（publish.on_change_only）
    在编排层判定，不经本类。
    """

    def __init__(self, min_interval_s: float = 0.2,
                 now: Optional[Callable] = None):
        """
        保存最小间隔与注入时钟（协议 I3）.

        Args:
            min_interval_s: 同一话题两次实际发布的最小间隔 [s]；<=0 关闭
                间隔门（key 变化即放行）.
            now: 单调时钟（返回 float 秒，协议 I3 由编排层注入，如
                RclpyClockAdapter.now）；None 回退 time.perf_counter
                （纯核自包含缺省，单测可注入假时钟）.

        """
        self._min_interval = max(0.0, float(min_interval_s))
        self._now = now if now is not None else time.perf_counter
        # 每话题最近一次「实际发布」的版本 key 与时刻；未发布过无记录
        self._published_key: Dict[str, Hashable] = {}
        self._published_at: Dict[str, float] = {}

    def should_publish(self, topic: str, key: Hashable,
                       force: bool = False) -> bool:
        """判定本次是否真正发布.

        Args:
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
