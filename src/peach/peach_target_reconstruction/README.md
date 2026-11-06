# peach_target_reconstruction

当前目标 RGB-D 局部重建 Lifecycle 节点：按曝光时刻 TF、掩膜和质量门积分，输出精化几何、模型与 GraspDecision；不发运动。

源码分工、采集和收口流程见 [架构](doc/architecture.md)，话题、BuildTargetModel、调试服务和参数覆盖见 [接口](doc/io.md)。以下保留近期实现边界与复核口径。

RGB-D 进入帧环前核对深度曝光时间、单通道布局、图像与标定尺寸以及针孔内参。
零时间戳会被拒绝，以免 tf2 将其解释为请求最新位姿。
采集和 ICP 整数下限在启动及运行时均校验。被拒绝的修改保留上一组有效参数。

感知的退化轴可带 `axis_conditioned_from_current_cloud`，表示同曝光点云的连续
几何参考，仍保持原 REOBSERVE/REJECT 资格下限。本节点对已绑定 ID 从
`initial_pose`/`target_observations` 读取有限轴作为 hint，并同步候选中心；读取
hint 本身不要求 ACCEPT。因此弱轴感知几何变化可能改变重建输入，即使共享
`estimate_bag_landmarks(axis_hint=...)` 算法保持不变。独立 refit、融合、轴冲突、
包络与 GraspDecision 身份/时效门继续决定模型和许可，连续初值不直接产生许可。

`sync_slop_s` 必须大于 0，避免 Jazzy 近似同步器在零容差下永不配对；`capture.min_views` 必须不大于 `capture.max_views`，避免帧栈封顶后仍达不到 Build 最少视角数。同步容差与三项视图预算（min/recommended/max_views）在构造期捕获，均为只读参数：可通过启动 YAML/override 配置，运行期写入会被拒绝，修改后须重启节点。`recommended_views` 是建议值，不额外改变 min/max 的可接受区间。

## 开关语义

- `capture.auto_mode`：本节点自动采帧开关，运行期可改；false 只停自动采帧，手动 BuildTargetModel 仍可用。
- 显式 Build 复用相同的曝光时刻 TF、静止、掩膜和质量门；结束后保留该次 READY 模型，直到 reset 或换目标。它不修改自动采帧参数。
- 进程是否启动由 bringup 的 `reconstruction_enabled` 控制，不在本包参数中伪装成“关闭模块”。
- `skip_reconstruction` 属于 supervisor 作业策略，不会跨包改写本节点参数。
- 部署通过 `calibration_version` 标识当前相机内外参。未配置时仍可查看拟合结果，BuildTargetModel 的回执会提示缺少标定版本，臂侧模型身份门会拒绝执行。

cleanup 在销毁 ROS 实体前停止并等待采帧 worker。诊断 Updater 在进程内复用，cleanup 暂停定时器，再 configure 时恢复，不重复声明参数或注册任务。
直接 lifecycle shutdown 复用相同收尾，唤醒等待中的 Build、拒绝新请求；未 configure 或已 cleanup 时也可安全调用，迟到回调不会再发布到已销毁实体。

构云与 ICP 在状态锁外计算，成功及拒绝结果都须在锁内复核采集世代、目标和 COLLECTING 状态后才更新 ICP 刷新周期、修正量 EMA 和耗时统计。reset、换目标或收口期间返回的旧计算不能污染新会话缓存，也不能积分入库；重复成功曝光同样不重复落账。

产物发布先判定既有各通道节流结果。只有本次需要发送 `local_cloud`，或需要发送没有 TSDF 可用的 `preview_cloud` 时，才拼接采集帧的 XYZ/RGB；其它通道及心跳仍按原路径组装。空帧清屏、缺颜色、TSDF 优先预览、最终采集世代复核及同模型许可有效期冻结均不变。24 帧各 5000 点的 CPU 探针中，全节流路径每次消除 3.24 MB 的数组拼接；这不是采集、ICP 或 TSDF 积分耗时的优化。

Build 在 finalize 前后、每条消息发布前和 Action 成功提交前重新检查取消与同一整次 Build 期限，取消优先。最终产物先延迟发布；若计算、消息组装或结果封装返回时已取消或超时，就在同一个状态锁内清空会话与产物，记录 `build_result_discarded` 并清屏，再提交失败终态。其它回调不能在清理锁缝中读到迟到的 READY/许可。自动 finalize 和 Trigger 的默认发布行为不变。取消前已发送的消息须通过清屏失效，DDS 多话题发布不是原子事务。这些检查不抢占正在执行的数值库或 publish 调用，也不解决计算自身的卡顿。

`diagnostics_debug.timing.phase_ms_last` 保存单调时钟测得的阶段耗时，包括 TSDF 提取、柱/球 refit、采集云关键点、视角/融合、几何记录及最终发布。原 `refit_ms_last` 仍只表示柱/球 refit，`finalize_ms_last` 保留原口径；新增诊断时钟不参与几何、资格门、许可有效期或 Build 超时。嵌套阶段重叠，不能相加当总时长。已有 `/diagnostics` 健康回调在独立互斥回调组读取正在执行的阶段，超过 2 秒时标 WARN，并对同次阶段记录一次最多 20 帧的 Python 线程栈；不新增后台线程。

1 Hz 结构化心跳取快照和提交时均不等待状态锁，忙时跳过并计入 `heartbeat_skipped_busy`，保留原世代复核和许可冻结时效。调度状态镜像订阅用独立互斥回调组，避免多个相同锁的等待者占满四个 executor 线程、使取消请求排不到执行机会。忙时不重发旧许可伪装新鲜；健康诊断仍可报告当前慢阶段。数值库持有 GIL 或其它执行器资源耗尽时仍可能延迟，不能据此承诺硬实时取消或心跳。
