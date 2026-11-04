# ADR 0004：单一 rosbag2 录制路径

## 决策

会话 bag 只由 `peach_observability` 节点内 rosbag2 MCAP writer 管理；删除独立 `record_bag.launch.py` 旁路。

## 理由

- 两套录制路径会产生不同话题集合/目录/回收语义。
- 节点内 writer 与 lifecycle、会话目录、报告、预算回收天然绑定。
- 仍是官方 `rosbag2` API，不自定义存储格式。

## 后果

- 只需验证一条路径。
- 录制开关/报告/回收由 observability 参数统一控制。
