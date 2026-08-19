# peach_approach_grasp

## 简介

本包是桃子采摘链路的 C++ 编排层。它不重复做 2D/3D 算法，而是消费
`peach_pose_ros2` 的固定目标集合和 `peach_reconstruction_ros2` 的在线重建质量，
通过 MoveIt 生成“安全观察位 → 多视角补观测 → 精化 → 直线接近 → 工具动作 → 原路撤回”
流程。

最优的工程方案不是从远处沿一条固定曲线直接抓取：固定曲线无法针对遮挡、深度空洞和
不可达位姿调整。当前实现采用目标中心球面上的离散自适应视点，优先选择约 15° 新基线、
较小运动量和安全半径内的候选点；机械臂在每个小段后等待重建确实接收精确时刻 RGB-D，
再根据覆盖质量决定下一段。这样形成近似球面螺旋/弧线的折线扫描，同时保留逐帧闭环。
最终接触段交给 MoveIt Task Constructor（MTC）：OMPL 先搜索无碰路径到精化入口，
MTC `CartesianPath + MoveRelative` 再沿袋轴低速直线插入和同轴撤回。行为树
BehaviorTree.CPP 4 负责任务分支、质量门、工具 IO 与失败撤离，运动规划和任务决策职责分离。

## 当前开发范围：套袋桃

当前主动观察和接触流程以**套袋桃**为唯一验收主线：观察围绕袋体中心展开，
finalize 后要求圆柱/袋轴精化通过，再沿袋轴规划低速直线插入和同轴撤离。
裸桃结果目前仅用于感知与重建的离线研究，现阶段验收流程不得将其用于工具接触。
默认 `execution.enabled=false`、`grasp.enabled=false`、`tool.enabled=false` 保持不变；
任何真机运动仍须人工 arm，并完成现场安全确认。

设计依据包括：果园 NBV 研究显示，形状补全结合视角差异可改善果实重建与尺寸估计；
语义主动视点在真实植物实验中优于固定、随机和纯体积策略。运动层遵循 MoveIt Pilz 的
`PTP`/`LIN` 语义，未改动本项目冻结的 AUBO 驱动。

- NBV-SC：<https://arxiv.org/abs/2209.15376>
- 语义主动重建：<https://arxiv.org/abs/2306.09801>
- 农业主动视觉伺服：<https://arxiv.org/abs/1908.01885>
- MoveIt 官方 MTC 抓取教程：<https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html>
- MoveIt Task Constructor 官方仓库：<https://github.com/moveit/moveit_task_constructor>
- BehaviorTree.CPP 官方仓库：<https://github.com/BehaviorTree/BehaviorTree.CPP>
- BehaviorTree.ROS2 官方仓库：<https://github.com/BehaviorTree/BehaviorTree.ROS2>
- MoveIt Pilz：<https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html>

## 使用方法

构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-up-to peach_approach_grasp
source install/setup.bash
```

先按现有文档启动机械臂、相机、感知与重建，再启动本节点：

```bash
ros2 launch peach_approach_grasp approach_grasp.launch.py
```

本节点是 LifecycleNode（A8）：launch 自动触发 configure→activate；运动输出
权限绑定 Active 态——Unconfigured/Inactive/ErrorProcessing 下一切运动类入口
（RunTargetCycle action、start_cycle、go_to_photo_pose、preview_*、
set_execution_armed、工具 IO）一律拒绝并给出原因。configure 失败（非法参数、
机器人模型缺失）停在 Unconfigured，可修复后经 `ros2 lifecycle set` 重新触发；
deactivate 等价于 CANCEL_NOW（真取消活动周期后落 Inactive，接触段 recovery
锁语义不变）。MoveIt/MTC 接口由同名伴随节点承载（MGI/MTC 只接受
rclcpp::Node），对话题/服务名无影响。

构建后需要重启 `move_group`/bringup，因为 MoveIt 启动文件新增了官方
`move_group/ExecuteTaskSolutionCapability`；旧进程不会热加载该 capability。

默认 `execution.enabled=false`，调用周期只验证候选观察位能否规划，**绝不发送运动**：

```bash
ros2 service call /peach_approach_grasp_node/start_cycle std_srvs/srv/Trigger '{}'
ros2 service call /peach_approach_grasp_node/query_state std_srvs/srv/Trigger '{}'
```

批次编排器不调用 Trigger，而是经 action 驱动单目标周期（反馈携带编排状态，结果带
outcome 分级，见「周期终态与 outcome」）：

```bash
ros2 action send_goal /peach_approach_grasp_node/run_target_cycle \
  peach_harvest_msgs/action/RunTargetCycle \
  "{request_id: demo-1, run_id: demo, cycle_id: demo-1, target_id: <ID>, mode: 2}"
```

受理门按 mode 分流（2026-08-19，阶段 E 残局抬质量能力端）：`FULL`/`PREVIEW`
维持钉死语义——goal 目标必须仍是感知 selected 缓存目标；`OBSERVE_ONLY`
（残局抬质量，只观察+精化验证不抓取）的 goal 目标在 FULL 终局后已被感知计划
complete、selected 为空，改为命中**锁定集锚点缓存**即受理（`onTargets` 把每帧
`observations` 中全部 confirmed 目标——含非 selected——逐条维护进
`TargetCache`，`target_set_locked=false` 或 `harvest_run_id` 切换时清空；拒绝日志
区分"不在锁定集/锚点缺失"）。OBSERVE_ONLY 周期执行体的目标快照/新鲜帧等待/
目标安全门一律按 goal 钉入 ID 走锁定集缓存（`cycleTargetSnapshot` 单点分流），
BT 在 `FinalizeAndValidate` 后经 `IsObserveOnly` 短路到 `ReportObserveOnly`，
不进再确认/MTC/工具/撤离段，终局按 `PLAN_READY` 上报 SUCCEEDED。

全局拍照位姿服务（批次编排器在发现/复扫轮次开始前调用；守卫运行中周期与接触段
recovery，先复核安全门，再按 `photo_pose_named_target` 的 SRDF 命名状态规划，
Pilz `PTP` 失败回退 OMPL；`execution.enabled=false` 时仅规划不运动）：

```bash
ros2 service call /peach_approach_grasp_node/go_to_photo_pose std_srvs/srv/Trigger '{}'
```

重建 finalize 且 `grasp_decision.allowed=true` 后，可继续预览后续接触轨迹：

```bash
# 当前位置 → 精化入口 → 沿袋轴直线插入
ros2 service call /peach_approach_grasp_node/preview_approach_insert \
  std_srvs/srv/Trigger '{}'

# 当前位置 → 精化入口 → 直线插入 → 沿原轴撤离
ros2 service call /peach_approach_grasp_node/preview_full_contact \
  std_srvs/srv/Trigger '{}'
```

两个预览服务在接口层硬编码 `execute=false`，不读取 arm，也不会发送运动、工具 IO
或目标完成请求。规划成功后状态为 `PREVIEW_READY`；在 RViz2 的
**Motion Planning Tasks** 面板选择 `peach_approach_insert` 或
`peach_full_contact_preview` 查看分阶段机械臂轨迹。

RViz2 添加 `MarkerArray`，话题选择
`/peach_approach_grasp_node/planned_views`，可查看候选观察方向。数据 Web 界面可订阅
`/peach_approach_grasp_node/status`；其 JSON 含状态、目标 ID、视图数、角基线、深度比例、
精化 RMSE/inlier 和最终允许状态。

要查看 MTC 的分阶段轨迹，在 RViz2 中添加 **Motion Planning Tasks** 面板；MTC 会把当前
`peach_approach_insert` 或 `peach_linear_retreat` 的最优 solution 发布到 introspection。
任务决策树位于 `config/harvest_tree.xml`，无需重新编译 C++ 即可调整非算法性分支顺序。

真机分阶段验证时，先复制参数文件，把 `execution.enabled` 改为 `true`，但保持
`grasp.enabled=false`、速度/加速度 0.05。每次周期必须在现场确认安全后一次性解锁：

```bash
ros2 service call /peach_approach_grasp_node/set_execution_armed \
  std_srvs/srv/SetBool '{data: true}'
ros2 service call /peach_approach_grasp_node/start_cycle std_srvs/srv/Trigger '{}'
```

周期成功、失败或取消后自动解除 arm。若取消发生在 MTC 接触轨迹已经下发后，
节点进入 `RECOVERY_REQUIRED` 并阻止新周期；它不会在取消后擅自继续运动。现场必须
先通过示教器或 MoveIt 人工确认已沿安全方向撤离，再调用只清状态的服务（该服务不发运动）：

```bash
ros2 service call /peach_approach_grasp_node/acknowledge_recovery \
  std_srvs/srv/Trigger '{}'
```

只有完成无工具的观察运动验收，并接入真实末端工具
及碰撞模型后，才允许同时启用 `grasp.enabled` 和 `tool.enabled`。本包不会上电、松刹车、
恢复安全停止，也不会调用 `/aubo_dashboard/startup`。

## 执行逻辑

1. 锁存当前 `selected_target_id`、初始入口、轴和目标中心；目标切换、丢失或数据过期立即
   阻止下一段运动。
2. 读取 `base_link→camera_depth_optical_frame`，生成目标中心球面候选。姿态按 ROS 光学
   坐标系 `+Z` 朝目标，TF 再换算成 `tcp` 的 MoveIt 目标（规划组 tip 即末端 TCP）。
3. 首段使用 Pilz `PTP` 到安全观察位；后续使用短 `LIN` 弦段构成弧形补观测轨迹。每到一位
   等待重建 `captured_views` 增长，禁止开环扫完整条曲线。观察段为预算制扫描
   （2.13-E2）：以保证 `scan.min_effective_views=2` 次有效视点观测（移动到位且收到
   新鲜目标帧）为下限，达到下限且质量收敛即提前收口；运行预算为
   `max(scan.time_budget_s, 2.5×实测移动+等帧成本EMA)`，剩余预算按实测 EMA 换不起
   一个有效视点即预测性收口、强制 finalize 走降级链；`scan.maximum_moves` 仅兜底。
4. 重建输出至少 3 个合格视图、最大角基线至少 15°、平均最近邻角基线至少 6°、平均
   有效深度比例至少 0.40，且感知/重建 ID 一致、数据新鲜，才调用
   `finalize_reconstruction`（阈值以 `config/approach_grasp.yaml` 为权威源，
   此处仅为现行档说明）。
5. 只有 `READY`、同 ID 精化结果、`grasp_decision.allowed=true`、拟合 RMSE/inlier 通过，
   才进入 `READY_FOR_GRASP`；精化未达标但身份一致的锚点还在时按候选锚点降级抓取
   （入口点=锚点−轴·(行程+`grasp.fallback_standoff_m` 袋外余量），终局 reason 带
   `降级抓取(degraded_anchor)` 标记）。
6. 抓取启用后先过 `ReconfirmTarget` 抓取前再确认（2.7-RECONFIRM）：等一窗新鲜观测
   （窗口按观测话题实测帧间隔 EMA 自适应，上限 `grasp.reconfirm_wait_s`），复核身份
   一致、锚点漂移 ≤ `grasp.reconfirm_tolerance_m`、无 `target_swinging` 摆动；漂移超限
   用最新锚点重算 entry/axis 一次再复核，摆动在窗口预算内等平息；窗口耗尽/漂移超限/
   摆动不息累计达 `grasp.reconfirm_max_attempts` 次即放弃，周期终局 SKIPPED_QUALITY
   （reason 含具体原因）。`grasp.allow_stale_anchor=true` 退化为旧"按静态锚点继续"
   行为（验证期遗留，室外默认关闭）。
7. 再确认通过后行为树进入 `MTCApproachAndInsert`：`CurrentState → MoveTo(OMPL) →
   MoveRelative(CartesianPath)`。OMPL 只用于枝叶环境中的自由空间入口搜索，接触段必须保持
   精化轴直线；规划后、执行前再次检查机器人状态、目标 ID/新鲜度和取消标志
   （`allow_stale_anchor=false` 时观测仍陈旧则按 SKIPPED_QUALITY 拒绝，不再二次放行）。
   plan-while-waiting（2.13-E3，`grasp.mtc_preplan=true` 默认开）：再确认等待窗口机械臂
   静止，期间按当前锚点在后台线程预规划该 MTC 任务（只规划不执行）；再确认通过且最终
   锚点与规划基准锚点漂移 ≤ `grasp.replan_threshold_m`（0.02m）时直接执行预规划解，
   超限/预规划失败/漂移重算过几何才内联重规划；取消与 deactivate 经 cancel→preempt
   正确丢弃预规划任务。
8. 行为树单独执行工具 IO，成功后用另一条 MTC `CurrentState → MoveRelative` 原轴撤回，
   最后才完成目标。完成推进权归属：action（RunTargetCycle）驱动的周期由批次编排器按
   终态统一调 `complete_selected_target` 推进感知计划，本节点不再调用，避免双写；
   手动 `start_cycle` 周期没有编排器，仍由本节点自推进兜底。工具失败也尝试 MTC 原轴
   撤离；撤离仍检查机器人安全状态，但不要求被末端遮挡的目标继续可见。接触/撤离不
   回退为任意 OMPL 路径。接触轨迹执行后若取消或撤离失败，状态锁定为
   `RECOVERY_REQUIRED`，必须现场人工撤离并确认，禁止自动开始下一周期。

关键安全门包括：一次性人工 arm、机器人状态新鲜且无急停/错误/断电、目标仍可见且 ID
不变、重建质量收敛、MoveIt 碰撞与可达性规划成功。环境风动枝叶属于动态障碍；当前仅依赖
规划场景和逐帧目标可见性，正式室外部署仍应增加近场避障传感器、末端力/触觉确认和工具
闭合反馈，不能把本版本直接视为无人值守量产安全系统。

## 周期终态与 outcome

周期状态在节点内部一律以 `enum class CycleState`（`cycle_state.hpp`）流转，状态 JSON
中的字符串只是发布层投影，终局判定只认枚举——修复了旧字符串分类把 `PLAN_READY` /
`READY_FOR_GRASP` 误判为 FAILED 的问题。终局映射（`terminalOutcome`）：

| 终态 | 终局 |
|---|---|
| `SUCCEEDED` / `PREVIEW_READY` / `PLAN_READY` / `READY_FOR_GRASP` | SUCCEEDED（只规划与未使能抓取两档均为圆满终态） |
| `CANCELED` | CANCELED |
| `FAILED` / `PREVIEW_FAILED` | FAILED |
| `RECOVERY_REQUIRED` | RECOVERY_REQUIRED（接触段不确定，需人工） |

action Result 的 `outcome` 再按失败点细分（`pending_outcome_`，BT 失败时记录）：
视角均不可达或扫描上限未收敛、接触段入口点落入保护区（`scan.protected_zones`，
F1）、MTC 规划阶段失败（未启动执行）记
`SKIPPED_UNREACHABLE`；finalize 后最终质量门失败记 `SKIPPED_QUALITY`；执行已启动后
的失败记 `FAILED`。编排器据此分级记账并自动推进，只有 `recovery_required=true` 才
中断批次等人工。

## 软件框架

```text
peach_pose_ros2 ─ target_observations ─┐
peach_reconstruction_ros2 ─ quality ──┼─ BehaviorTree.CPP
                         ─ refined_* ─┤   ├─ 主动视点循环（MoveGroup PTP/LIN）
aubo_io_controller ─ robot_status ────┘   ├─ MTC：OMPL 到入口 + Cartesian 插入/撤离
                                          ├─ 可选 SetIO 工具
                                          ├─ status(JSON)
                                          └─ planned_views(MarkerArray)
```

- `view_planner.*`：无 ROS 依赖的候选视点生成、评分、相机 look-at 和工具轴姿态。
- `protected_zones.*`：无 ROS 依赖的环境几何保护区（F1）——`scan.protected_zones`
  stride-6 轴对齐盒的解析（畸形盒逐条丢弃）与点包含判定（闭区间含表面），视点
  生成剔除与 MTC 接触段入口剔除共用同一谓词。
- `quality_gate.*`：无 ROS 依赖的 ID、新鲜度、覆盖和精化质量门。
- `target_cache.*`：选中目标、重建诊断与精化结果的线程安全缓存及 ID 一致性调和。
- `safety_gate.*`：机器人状态与目标观测新鲜度的纯核安全门。
- `*_base.hpp` + `impl_factory.hpp` / `motion_factory.hpp`：四类可替换职责的抽象基类与
  按名工厂（`view_planner.impl=spherical_adaptive`、`quality_gate.impl=threshold`、
  `safety_gate.impl=robot_status_gate`、`motion.impl=moveit_motion`；未知名启动即抛
  `std::invalid_argument` 并列出可用名）。调用端只持基类指针；I5：硬件安全门任何实现
  不得旁路。
- `motion_interface.cpp` + `motion_interface_impl.*`：节点侧运动薄壳（Trigger、工具 IO、
  接触轨迹预览、`go_to_photo_pose`）与 MoveGroup 规划/执行默认实现（moveit_motion）。
- `bt_nodes.cpp`：行为树节点（视点循环、finalize 质量门、抓取前再确认、MTC、工具、
  目标完成）。
- `reconfirm_policy.hpp` / `grasp_geometry.hpp`：抓取前再确认（2.7-RECONFIRM）的
  逐样本判定纯核（漂移容差/摆动平息/超限计数）与降级/重算入口点纯函数。
- `cycle_action.cpp`：周期工作线程、RunTargetCycle action 与各 Trigger/SetBool 服务。
- `cycle_state.hpp` / `action_contract.hpp`：`CycleState` 枚举与终局映射、action 契约。
- `config/harvest_tree.xml`：可检查的采摘行为树拓扑和失败传播顺序；按阶段拆为
  SubTree 库（观察扫描/质量验证/靠近抓取/工具动作/撤离收尾，协议 2.16-9），
  主树只负责组合，节点注册名不变。
- `grasp_task.*`：MTC 接近/插入/撤离任务（stage 工厂消重），发布 MTC introspection solution。
- `approach_grasp_node.cpp`（本体约 590 行）+ `approach_grasp_node_impl.hpp`：ROS/MoveIt
  接线、参数与状态投影。
- `config/approach_grasp.yaml`：所有默认参数（含 `photo_pose_named_target`）；代码默认值
  与 YAML 同步维护。
- `test/`：视点几何、排序和质量门单元测试，外加 `test_impl_contract.cpp` 可替换接口
  契约测试（每基类 fake + 工厂按名创建/未知名抛错）。

本包新增的上游数据只有重建 diagnostics 中的 `view_coverage`：它来自真正被 TSDF 接受的
帧及其精确相机位姿，不使用规划轨迹猜测采集效果。session metadata 同步保存该字段，便于
复现实验和调参。

## 附录 A：CycleState → TargetPhase 投影（A13，2026-08-18）

RunTargetCycle 反馈的 `state.target_phase` 由 `targetPhase(CycleState)`
（`cycle_state.hpp`）投影，编排器 `set_target_phase` 消费以驱动批次过程线。
投影一名一义（常量与 `HarvestState.msg` 的 `TARGET_*` 由 `cycle_action.cpp`
static_assert 与 `test_action_contract.cpp` 双向钉死）：

| CycleState | TargetPhase |
|---|---|
| `IDLE` / `CANCELED` | TARGET_IDLE（无取消相，编排器记账后同回 IDLE） |
| `PLAN_OBSERVATION` / `MOVE_TO_VIEW` / `WAIT_FRAME` | OBSERVING |
| `FINALIZE` | FINALIZING（质量门验证在其内完成，无独立 VALIDATING 态） |
| `MTC_APPROACH_INSERT` / `PREVIEW_CONTACT_PLANNING` | APPROACHING |
| `ACTUATE_TOOL` | TOOL_ACTION |
| `MTC_RETREAT` | RETREATING |
| `PLAN_READY` / `READY_FOR_GRASP` / `PREVIEW_READY` | COMPLETING（plan-only 圆满收尾） |
| `SUCCEEDED` | TARGET_SUCCEEDED |
| `FAILED` / `PREVIEW_FAILED` / `RECOVERY_REQUIRED` | TARGET_FAILED |

A13 前反馈从不填充该字段（恒 TARGET_IDLE），过程线在周期内停在 IDLE；
本次补齐投影（消费方不变量：仅在 `target_active` 期间驱动阶段显示，
终局仍以 action Result 的 outcome 为准）。

## 附录 B：配置边界审计（A12，2026-08-18）

判据同编排器（键只承载「选实现 + 数值/名称/开关/阈值」为合格）。

| 文件 | 键数 | 合格 | 例外 |
|---|---:|---:|---|
| `config/approach_grasp.yaml` | 66 | 66 | 无 |

逐键结论：`*.impl` 四键为按名选实现（未知名启动抛错）；`frames.*`、
`moveit.*`、`photo_pose_named_target`、`behavior_tree.xml` 为名称/路径/数值；
`scan.*`、`quality.*`、`timeouts.*` 为几何与阈值数值；`execution.*`/`grasp.*`/
`tool.*` 为开关+阈值。周期状态转移全部在 `cycle_state.hpp` 枚举与 BT
（`harvest_tree.xml`）中，策略分支不由配置键表达。未发现违规键。
