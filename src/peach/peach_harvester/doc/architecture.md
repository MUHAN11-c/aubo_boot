# peach_harvester 架构

## 职责

采摘大脑：视觉事实、目标重建、批次决策；场景障碍独立进程。禁止直接写关节/工具 IO，跨包只走 IDL。

## 源码地图

`peach_harvester/brain.py`：三个节点合进程；`vision/scene_perception/`：检测/分割/身份/锁定集；`vision/target_reconstruction/`：采帧、融合、模型与许可；`vision/scene_obstacles/`：Survey触发的场景快照；`supervisor/`：harvest_fsm纯核、编排、选果和账本；`config/`、`model/`、`test/`：部署、模型和验证。

## 入口、输入输出与参数

整栈使用 brain.launch.py；独立台架入口以 setup.py console_scripts 为准。输入RGB-D、精确stamp TF、关节及作业状态；输出观测/初值、模型/许可、批次状态/事件/账本。scene_obstacles仅写world collision objects。参数通过 config YAML + attach；grasp_standoffs为共用停位几何源。接口详见 [io.md](io.md)。

## 生命周期、并发与失败

brain三节点共享MultiThreadedExecutor，节点内默认互斥组、节点间可并行，重活在线程中运行；保持Unconfigured，由系统管理器按感知→重建→臂→调度激活。scene_obstacles独立不进名单。批次态只经harvest_fsm.react/reducer/_apply；唯一选果输入是target_observations，grasp_decision仅缓存许可。Survey成功才Begin。默认 `reconstruct_in_trajectory=true`：Build绑定后启动臂侧周期，FinalizeAndValidate等同目标精化/许可到齐并过质量门后才规划，避免串行观察消耗冻结的模型有效窗；显式false保留fast/conservative补视，过期仍拒绝。默认PREGRASP_ONLY等待ACK。重建积分必须精确stamp，失帧/旧TF/身份漂移拒收，不能用latest补积分。失败写稳定原因，ledger由调度维护；固定相机+mock腕不提供新机位。

brain入口将 `KeyboardInterrupt` 和 `ExternalShutdownException` 作为正常停栈；先 `executor.shutdown()` 等工作回调退出，再销毁节点，最后仅在上下文仍有效时关闭。其他异常仍向上传播，不能把回调故障伪装成正常退出。

## 验证与限制

`colcon test --packages-select peach_harvester --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 模型时间与单视质量

`vision/target_reconstruction/publish.py`固定同model_revision首次generated_at；GraspDecision.header.stamp使用该值，心跳不改model_stamp或valid_until。Build结果TargetModel复用相同生成时间/截止时间，diagnostics_debug顶层model_stamp为浮点秒。独立单机位成功最多MEDIUM，至少2个独立机位才可HIGH；连帧数不代替机位数，质量档变化不另改GraspDecision接触门。READY/accepted表示模型状态，不自动授予套入、剪切或真机运动。

重建Build的有界采集/终检编排由`_collect_and_finalize_build`负责：最多三次单机位新曝光、一个既有时间预算，失败输入经`_restart_single_view_build`替换，不累加重复视角。`_finalize_now`及几何函数继续执行原质量检查；三维包络与二维轮廓不可观测使用不同文本拒因。调度等待动作的公共路径可监视并行Build结果，明确失败即取消依赖它的FULL，保留质量证据到终局账本。
