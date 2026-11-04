# 数据记录

## 唯一路径

会话 bag 使用 `peach_observability` 节点内的 rosbag2 MCAP 写线程；随节点 lifecycle 启停、收尾并生成报告。独立的 `record_bag.launch.py` 旁路已删除，避免两套录制路径。

- 输出：`runs/session_*/bag/`；
- 记录：状态/事件/观测/重建/动作/诊断/`/rosout`（按配置）；
- 图像/点云由配置开关控制；
- 体积预算、报告与回收由 observability 统一负责。

## 不记录

- `src/peach_sim` 不在范围；
- Web 静态资源不进 bag；
- 力/CoP 数据通过 `/force/*` 进入同一会话。
