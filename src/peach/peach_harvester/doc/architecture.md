# peach_harvester 架构

## 职责

采摘大脑：视觉事实、目标重建、批次决策；场景障碍独立进程。禁止直接写关节/工具 IO，跨包只走 IDL。

## 源码地图

`peach_harvester/brain.py`：三个节点合进程；`vision/scene_perception/`：检测/分割/身份/锁定集；`vision/target_reconstruction/`：采帧、融合、模型与许可；`vision/scene_obstacles/`：Survey触发的场景快照；`supervisor/`：harvest_fsm纯核、编排、选果和账本；`config/`、`model/`、`test/`：部署、模型和验证。

## 入口、输入输出与参数

整栈使用 brain.launch.py；独立台架入口以 setup.py console_scripts 为准。输入RGB-D、精确stamp TF、关节及作业状态；输出观测/初值、模型/许可、批次状态/事件/账本。scene_obstacles仅写world collision objects。参数通过 config YAML + attach；grasp_standoffs为共用停位几何源。工具档案由 `harvest_system` 默认 `bite_shear_v1` 注入；独立 brain.launch 声明默认仍 adaptive。接口详见 [io.md](io.md)。一批实现流程见 [系统源码设计](../../peach_bringup/doc/design.md)。

## 生命周期、并发与失败

brain三节点共享MultiThreadedExecutor，节点内默认互斥组、节点间可并行，重活在线程中运行；保持Unconfigured，由系统管理器按感知→重建→臂→调度激活。scene_obstacles独立不进名单。批次态只经harvest_fsm.react/reducer/_apply；唯一选果输入是target_observations，grasp_decision仅缓存许可。Survey成功才Begin。默认 `reconstruct_in_trajectory=true`：Build绑定后启动臂侧周期，FinalizeAndValidate非观察周期先等同目标GraspDecision（含精化几何）并过质量门后才规划，避免 `waitForRefined` 先超时导致决策到达后仍跳过；显式false保留fast/conservative补视，过期仍拒绝。默认PREGRASP_ONLY等待ACK。重建积分必须精确stamp，失帧/旧TF/身份漂移拒收，不能用latest补积分。失败写稳定原因，ledger由调度维护；固定相机+mock腕不提供新机位。

brain入口将 `KeyboardInterrupt` 和 `ExternalShutdownException` 作为正常停栈；先 `executor.shutdown()` 等工作回调退出，再销毁节点，最后仅在上下文仍有效时关闭。其他异常仍向上传播，不能把回调故障伪装成正常退出。

## 批次命令、选果与参数

调度命令循环每步走 ROS 2 子 logger `peach_supervisor.batch`（消息 `command=` / `batch` / `target` / `message`）。状态只经 `harvest_fsm.react` / reducer / `_apply`。首巡：Survey 成功且 `scene_epoch==0` → BeginScene → WAIT_LOCK。回访：Survey 成功后直接 SELECT，不再 Begin。

| 命令 | 含义 | 关键参数（部署默认） |
|------|------|----------------------|
| navigate | 固定座直接 NAV_OK | 无底盘动作 |
| survey | 去拍照位；成功刷新场景障碍 | `action_timeout_s=40` |
| begin_scene | 仅首巡开收齐窗 | `service_timeout_s=5` |
| wait_lock | 等本世代锁定集 | `survey_wait_s=15` / `survey_dwell_s=2` |
| select | 深度窗 ∩ CheckReachability | `selection_depth_min_m=0.3` / `max=1.6`；`execute_pregrasp_only=true` |
| dispatch | 绑定 Build 后派臂 | `reconstruct_in_trajectory=true`（不再串行补视） |
| execute_full | PREGRASP_ONLY 或 FULL | 同上 `execute_pregrasp_only`；`execution_enabled` 默认 false |

选果输入只有 `target_observations`。资格：已锁定、已确认、未入账、非裸果、非贴边、非 `tool_clearance_failed`。点名 `goal.target_ids` 过同一资格，但不受深度/IK 窗。`selection_priority` 默认 0：仍按点名或自动第一颗。大于 0 时本批只抓锁定集里 `priority` 等于该值的一颗，并把它写回 `goal.target_ids`，抓完即收批；点名名单不参与。调试页「每轮按顺序」每轮改这个数，顺序号来自本批拍照锁定，不用开批前那份可能已换代的 ID。CheckReachability 不可用才回退 `selection_reach_min_m=0.15` / `max_m=0.88` 半径窗。SELECT 用感知建议行程；默认 `execute_pregrasp_only=true` 只检预抓取停位 IK。仅显式套入（`execute_pregrasp_only=false`）才 `require_sleeve`。臂侧 Finalize 用精化行程再检一次，且同样仅非预抓取才检套入直线。

选果预检走 `peach_supervisor.contact`，消息 `caller=SELECT target=… travel=… reachable=… code=…`。`code=ok` 才入选。默认不检套入；`sleeve_no_cartesian` 只在显式套入预检出现，表示预抓取和套入终点都有 IK、沿轴直线未走满——看同秒臂侧 `peach_arm.contact` 的 `CheckReachability` 各滚转 `fraction`。无候选则 `empty_survey_limit=2` 次回访后结算。`goal.target_ids` 非空且已全部入账时 CYCLE_DONE 发 `EMPTY_LIMIT` 立即结算，不再空扫。单果墙钟预算 40 s（相对人工偏保守）：收齐 `confirm_frames=2` / `min_collect_frames=4` / `lock_settle_frames=2` / `max_collect_s=12`；重建帧环 12 s / 96 帧 / 192 MiB。2026-10-16 真实相机+mock 短测 5/5 见 [测试指导](../../peach_system_tests/doc/testing.md)（账本 24–27 s，RunHarvest 38–43 s）。rqt_console 按 logger 名过滤；文件在 `$ROS_HOME/log`。

## 验证与限制

`colcon test --packages-select peach_harvester --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 模型时间与单视质量

`vision/target_reconstruction/publish.py`固定同model_revision首次generated_at；GraspDecision.header.stamp使用该值，心跳不改model_stamp或valid_until。Build结果TargetModel复用相同生成时间/截止时间，diagnostics_debug顶层model_stamp为浮点秒。独立单机位成功最多MEDIUM，至少2个独立机位才可HIGH；连帧数不代替机位数，质量档变化不另改GraspDecision接触门。READY/accepted表示模型状态，不自动授予套入、剪切或真机运动。

重建Build的有界采集/终检编排由`_collect_and_finalize_build`负责：最多三次单机位新曝光、一个既有时间预算，失败输入经`_restart_single_view_build`替换，不累加重复视角。`_finalize_now`及几何函数继续执行原质量检查；三维包络与二维轮廓不可观测使用不同文本拒因。调度等待动作的公共路径可监视并行Build结果，明确失败即取消依赖它的FULL，保留质量证据到终局账本。

## 动作终态与周期时间（2026-10-23）

ExecuteTarget在途feedback中的recovery标志允许锁存，收到该动作终态后采用臂侧终态值：FULL成功返程可清除，PREGRASP_ONLY或异常要求恢复时保留。feedback回调绑定本次调用token，结果收口后迟到包不能复活action_active或恢复门；网页不能自动代替人工ACK。mock/real共用这些语义。

派发时记录start_unix_ns/start_ros_ns，臂侧动作返回立即记录end_unix_ns/end_ros_ns；账本details同时保存cycle_id/scene_epoch/time_basis。动作结束点早于取消并行Build和落盘，分析不得再由ledger mtime假装精确时间。未经过派发的过滤/跳过只有真实结束点，明确source_clock_end_only。


## 视觉帧与发布一致性（2026-10-23）

`debug_image_raw`在RGB-D入口直接预览，运动门只暂停推理；`debug_image`表示实际完成推理的那次曝光。2D框、掩膜和箭头只来自本帧检测，源身份/抓取顺序可记忆，像素几何不跨帧复用。空检测、运动开始和BeginScene覆盖清除检测云/marker。感知worker任务以frame_generation隔离BeginScene、运动和生命周期变化，身份提交及发布都在plan_lock内复核，避免旧推理污染新场景。换窗和销毁排空事件写线程。

重建在采集提交临界区再次检查stamp严格递增，两个入口同时准备同曝光只积分一次。观测掩膜解码和预抓取TF查询同样带generation复核；发布由同一身份/模型/帧栈快照构造，锁外完成云和marker后再复核并串行发送，旧云与许可不能尾随新世代清屏。mock与real使用相同边界，不调整工具工艺门或质量阈值。
