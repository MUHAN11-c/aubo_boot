# peach 端到端（E2E）测试方案 v2.0（按阶段组织）

日期：2026-09-24　分支：`test/20260909-field-traj`（tip=be79ca2，行号以当日磁盘为准）
性质：**方案文档**（本轮只做全面审查 + 方案规划，不动代码、不启栈、不跑轮次；实施分后续轮）。
依据：peach 全仓探读（架构 / 测试基建 / 战役现状三路，关键结论带 file:line）+ 三份活文档当轮核对 + runs/ 与 campaign/analysis/ 磁盘证据复核。

**本轮五项用户裁定**：① 仅规划文档，评审通过后下一轮实施；② 自动化 E2E 场景深度=**全矩阵含故障注入**（分两期落地）；③ A-P3-1 move_group TEM stop 风暴=**关 TEM 重定基线**（根因另行跟）；④ peach_sim Gazebo 物理闭环**不纳入**（维持决策 0028「只做场景建模与摆位」）；⑤ 分阶段测试的内在结构 = **解析法先验 → 程序实跑（全程录制）→ 三者互相验证**——三方=解析法 / 实跑在线结果 / 录制全量数据（session bag）+ rviz2 窗口视频（详细分析），贯穿各阶段。

> **v2.0 变更记录**（2026-09-24）：**结构重组为按阶段组织**——正文主体改为阶段一～六，每阶段自含「目标 / 前置 / 测试内容 / 三方互证执行 / 录制档位 / 验收门 / 产物 / 风险回退」；层视角（E1–E4）降为 §2.1 映射表；排期章并入各阶段。内容沿用 v1.2 已核实事实，无新增裁定。
> **v1.2**：互证扩三方（录制腿 + C2 bag 复算 + C3 视频取证 + 录制档位分层 + 完整性门）。**v1.1**：互证方法论（三段式 + 三分法 + 案册注册表）。**v1.0**：初版（审查结论 + 四层设计；一期/二期用例计数 v2.0 修正为 9+6=15）。

---

## 0. 结论摘要

1. **测试金字塔单测厚、系统层空**：纯核 pytest 370+ 用例、peach_arm 12 套 gtest、回放塔三层冻结基线都在；但集成层只有 1 个**不发 goal** 的 launch_testing 冒烟，系统层零自动化。**最大结构性空白：supervisor 批次链（RunHarvest→FSM→选果→派发→账本→ACK）没有任何自动化覆盖**——`sim_field_targets.py` 直接向 `/peach_arm/execute_target` 发周期（:674）绕过调度；战役 `run_round.sh` 走全链但依赖真相机+操作员在场。
2. **补空白不需要新造框架**：注入协议、世代对齐（`HarvestState.msg:60` 自带 `scene_epoch`，锁定判定 `executor_node.py:1662-1667`）、解析先验引擎（`replay_oracle`/`analytic_roll_ladder`/`harvest_fsm.react`）、录制腿（会话 bag + `record_rviz_harvest.sh` + `collect_round.py`）全部现成，缺的只是接线与档位约定。
3. **六个阶段递进**（§3–§8）：地基与缓解 → 注入矩阵收口（互证首秀）→ 自动化批次链一期 → 二期（故障注入+本地满配轮）→ 真相机战役轮 → 真机前置。每阶段内部固定 A（解析先验）→ B（实跑+全程录制）→ C（三方互证收口）。
4. **E2 注入矩阵先收口**：磁盘最新 M1=18/20（两失配例已定性）、M2=90%、M3=86% 全未过门，A-P3-1 TEM 风暴是 M3 稳定性阻断项——先关 TEM+裁定边界例+复跑重定基线，后续阶段的通过率断言才有干净分母。

---

## 1. 审查结论（各阶段设计依据，已核实）

### 1.1 测试金字塔现状（AGENTS.md §10 口径）

| 层 | 社区工具 | 本仓现状 | 判定 |
|----|----------|----------|------|
| Lint | ament_lint_auto / uncrustify / flake8 / pep257 | 各 Python 包齐；peach_arm 以 uncrustify+cppcheck 为准（cpplint 版权头冲突是记档例外） | ✅ |
| Unit（纯核 pytest） | 零 ROS 纯核 | harvester 217、common 40、observability 41、vegetation 16、sim 23、bringup 4、system_tests 33 | ✅ 厚 |
| Unit（C++ gtest） | ament_add_gtest | 仅 peach_arm：12 套 | ✅（其他 C++ 包零 gtest，驱动栈只读不碰） |
| Integration | launch_testing + isolated domain | 仅 `test_mock_launch.py`（栈起来+关节序+lifecycle+bond+退出码，**不发 RunHarvest**）；`add_launch_test` 未隔离域 | ⚠️ 薄 |
| System | 独立 `*_tests` 包 | `sim_field_targets.py` 手工（绕过调度、不进 colcon）；campaign 脚本半结构化；peach_sim 纯核仅场景校验 | ❌ 最大缺口 |
| Field | 真机 + bag + 命名 request_id | runs/ + ledger + testing-log + campaign 门 | ✅ KEEP |

### 1.2 核心缺口清单（按危害排序）

1. **supervisor 批次链零自动化**：FSM 迁移、选果谓词、并行派发、recovery ACK、账本、ControlTask 语义——只有纯核 pytest 与真相机战役两层，中间没有「注入感知+全链+断言」自动化层。
2. **launch_testing 隔离不合规**：未用 `add_ros_isolated_launch_test`、无独立 `ROS_DOMAIN_ID`，与并发战役栈（域 61）有串扰风险。
3. **M1–M5 注入矩阵手工且未过门**（磁盘证据见 §4 阶段二）；改动后复跑未完成（09-23 轨迹定型轮遗留）。
4. **r0_gate.sh 测试文件清单漂移**：`test_reconstruction_decision_validity.py` 等新文件不在门内（testing-log.md:564 记档）。
5. mock 下结构性不可达路径只能留单测层：CUT_UNCONFIRMED、SetIO 失败负路径、mock 开 tool 必挂 TOOL_STATE_UNKNOWN 且不撤退（C 级红线，**E1 全用例禁开 tool**）。

### 1.3 已核实的承重细节

| # | 事实 | 证据 |
|---|------|------|
| 1 | 观测契约两段式：锁定前 `observations` 恒空、只发 `collecting_count/pending_count`；锁定后 `target_set_locked=true`+固定 ID 集 | `PeachTargetObservationArray.msg` 头注释 |
| 2 | 世代对齐可观测：`HarvestState.msg:60` 自带 `scene_epoch`（latched）——注入器订 `/peach_supervisor/state` 即可对齐；锁定判定=epoch 相等∧run_id 相符∧locked | `BeginScene.srv`；`executor_node.py:1105/1662-1667` |
| 3 | 注入协议现成：obs/diag/decision/refined/refit 五路发布 + robot_status 10Hz + execute_target/ack/reach 三客户端 | `sim_field_targets.py:660-676`、`:880-894` |
| 4 | skip_reconstruction 旁路现成：`_dispatch_unrefined`（plan_id 加 `:unrefined` 后缀）+ 臂侧 `allow_unrefined_geometry` | `executor_node.py:1322+`；brain.launch.py:69-76 |
| 5 | FULL 干跑 mock 可行：09-22 hollow 网格 10/10、adaptive 10/10（tool 恒关） | testing-log 09-22 续系列 |
| 6 | deny 门真实生效：批次4 unrefined 袋径×D_inner 门，网格 deny 例臂侧真实拦截 completion=2（A-P3-2 闭环） | testing-log 09-23 续九；grid expect 含 `deny_decision` |
| 7 | TEM 配置位置：`trajectory_execution.allowed_execution_duration_scaling: 5.0`（controllers{,_mock}.yaml:5）；moveit_config 非红线区 | 磁盘 grep |
| 8 | FSM 纯核零 ROS 可直接进测试进程：`react(batch_state, event)→Reaction`；`event_for_outcome:404`（1/2→SKIPPED、3→FAILED、4→CANCELED）；`permissions_for:396-400` RECOVERY 态仅 CANCEL+ACK | `harvest_fsm.py` |
| 9 | M1 磁盘真相=18/20：`runs/sim_field_targets_20260923_{153659,173249}.jsonl` 两轮 matched 均 18/20；失配=near_horizontal_1021_1（护栏正确拒，expect 仍 succeed）+ deep_left_low_axis（边界抖动 None）。记忆「复跑 20/20」是 deny 修复前「无 hang」口径（该轮 bag_d100_denied 本身 matched=False），勿混用 | 本轮 python 复核 |
| 10 | 战役门数字（沿用不改）：M1 100%、M2 PREGRASP≥95%、M3 FULL≥90% completion≥6 绕行比≤1.70、5 轮 FULL≥90% 无 300s hang、P1 3σ 门（entry/bottom/neck≤5mm/轴角≤1.5°/袋长≤8mm） | campaign README；2026-09-22 方案 |

---

## 2. 方法论与分层底座

### 2.1 分层总览与阶段映射

```
E4 真机（field_*）          KEEP 最终权威      → 阶段六（前置门）
E3 真相机 mock 臂战役轮      S3/S6/S7          → 阶段五
E2 注入矩阵门 M1–M5         TEM+重定基线+互证  → 阶段一（TEM）+阶段二（收口）
E1 自动化批次链 E2E ★新     launch_testing    → 阶段三（一期）+阶段四（二期）
   纯核 pytest / gtest / 回放塔（不动）
```

- 各层只断言各自能证明的事：E1 证明接线与语义（FSM/门/账本），不证明感知质量与轨迹方向；方向对错以 E3/E4 为权威——「colcon test 绿 ≠ 套袋验收」口径不变。

### 2.2 三方互证方法论（每阶段内在结构，裁定 ⑤）

```
A 解析先验（秒级、零 ROS、不起栈）
  ├ A1 几何先验：replay_oracle / analytic_roll_ladder 对同一案册逐例预报
  │    （接近分类、LIN 可行性、expect 档位、IK 可解性）
  ├ A2 语义先验：harvest_fsm.react 纯核推演期望状态/事件序列
  └ 产物：analysis/<round>/analytic_prior.json（逐例期望+已知解析盲区标注）
B 实跑 + 全程录制（起栈、真路径、三路取证同步开）
  ├ B1 纯核 gtest 层：同一案册喂 peach_arm 真实 C++ 纯核（回放塔交叉对账）
  ├ B2 图上层：mock 栈+注入器 / sim_field_targets 网格 / RunHarvest 全链
  │    └ 在线结果流：runs/<rid>/{ledger.json, *.jsonl, perception_data/}
  ├ B3 会话 bag 全量数据：observability 随栈启停（record_level 分级，MCAP）
  └ B4 rviz2 窗口视频：record_rviz_harvest.sh x11grab → runs/<rid>/rvizwin.mp4
C 三方互证收口（对照报告，阶段门）
  ├ C1 解析↔在线：逐例三分法——可行性与实现一致性
  ├ C2 在线↔bag 离线复算：bag_reader/bag_report/stability_metrics 从 B3 重算
  │    （state/event 时序、关节行程、几何 3σ）对照 B2 在线结论
  └ C3 视频详析（取证定位）：#3 分歧例必看 + 正路径抽检，裁定「回归 vs 解析盲区」
```

**C1 三分法**（沿用回放塔「analytic_ok 只许升不许降=全链下界」不对称语义）：

| # | 解析 A | 实跑 B | 判定 | 处置 |
|---|--------|--------|------|------|
| 1 | 成功 | 成功 | ✅ 一致通过 | — |
| 2 | 失败 | 成功 | ✅ 合规（解析是下界） | 记录频次；长期恒败恒成→解析盲区修 oracle |
| 3 | 成功 | 失败 | ❌ **互证发现** | **进 C3 视频取证 + C2 bag 复算归因**，关闭才过门 |
| 4 | 失败 | 失败 | ⚠️ 一致失败 | expect=succeed 则 **expect/案册过期**，修后重跑（非实现缺陷） |
| 5 | 任意 | 无码失败/hang | ❌ 缺陷 | 直接缺陷；bag 看 hang 期间最后话题活动定位卡点 |

**C2（在线↔bag 复算）**：outcomes/completion 与事件流一致、关节行程与 stage_durations 同量级、关键话题（/joint_states、state/events、observations）无 >2s 缺口。节点自报成功但 bag 侧事件链断裂=半份状态；bag 有活动 ledger 无记录=丢账本。
**C3（视频详析）**：全部 #3 分歧例取证 + 每轮 ≥1 例正路径抽检 + C2 发现点可视复核；时间戳标注进 cross_validation.md。

**录制档位（B3/B4）与留存纪律**：

| 阶段 | B3 bag | B4 视频 | 说明 |
|------|--------|---------|------|
| 三/四 E1-CI | `record_level:=std` 轻录（或按 CI 磁盘预算禁录） | **不可达**（headless） | CI=C1+轻量 C2（bag_reader 纯 Python 用例内复算）；C3 留本地轮 |
| 四 E1 本地全量轮 | std（分歧例 all） | 每轮录 | 三方齐备的人读对照样 |
| 二 E2 矩阵轮 | std | 每轮录（run_round.sh 已编排） | 沿用战役 runbook |
| 五 E3 战役轮 | **all**（100G 预算） | 每轮录 | runbook 现行（--level all --rviz-dur） |

留存照现行：mp4 不入库（.gitignore+台账记时长）；bag 分析后 `purge_analyzed_bags.py` 回收、`bags.md` 留索引；四件套（在线 jsonl+ledger / bag / 视频 / perception_data）经 `collect_round.py` 归一。

**资产与三个缺口**：A 段引擎全现成（`replay_oracle`+冻结基线、`analytic_roll_ladder`、`harvest_fsm.react`）；B3/B4 现成（observability 分级录制 `params.py:121/196`、`record_rviz_harvest.sh`）；C 段现成（`collect_round`/`bag_report`/`stability_metrics`/`bag_reader`）。缺口一=回放塔↔peach_arm gtest 交叉对账未接线（B1，`replay_oracle` docstring「阶段 2」）→ 阶段三实施；缺口二=案册 seed 三处不同源（梯子 20260922/60、实跑默认 20260910/100、回放塔 stratified_200=20260911）→ 阶段二建 `corpora.yaml` 注册表单源；缺口三=C2/C3 无统一载体 → `cross_validate.py` 只做 C1 薄层、引用既有归一产物（**不重写 collect_round/bag_report**）。

**边界**：joint_travel/PTP 绕行/IK 自碰不属 C1（C2 bag 可量实际行程作补充）；感知质量归 C2 的 3σ；真机接触归 E4。视频是取证材料不是自动断言源，自动化门由 C1/C2 承担。

---

## 3. 阶段一：地基与缓解（先行小改动轮）

**目标**：清掉阻断后续阶段的四个地基项，产出可评审的小 PR 面。
**前置**：无（本轮即第一轮）。

**测试内容**：
1. **TEM 关闭**：`aubo_e5_moveit_config/config/controllers.yaml:5` 与 `controllers_mock.yaml:5` 的 `trajectory_execution` 段加 `execution_duration_monitoring: false`（参数名以 MoveIt 2 Jazzy 文档核定；UR Driver 主流做法）。**关 ≠ 修**：A-P3-1 根因另行跟，TEM off 后 `boundedExecute`（execute_timeout_s=90s）仍是超时防线。同轮改 architecture/testing 活文档。
2. **r0_gate.sh 清单补齐**：按 testing-log.md:564 记档全量 diff 门清单 vs 各包 test/ 目录，使 CI 门 ≥ colcon test 收集集。
3. **launch_testing 隔离切换**：`peach_system_tests/CMakeLists.txt` 的 `add_launch_test` → `add_ros_isolated_launch_test`（自动独立 ROS_DOMAIN_ID）；`test_mock_launch.py` 复跑绿。
4. **S0 注入契约尖峰**（半天出结论）：相机关时感知节点是否在 `/peach/perception/target_observations` 静默；`harvest_run_id` 严格度与 `selected_target_id` 调和语义；C22 survey 失败注入手段勘定（photo pose 不可达 / moveit_enabled:=false 变体）。降级路径：双发→注入器高频占优（锁边沿触发 `executor_node.py:998-1016`）→ 感知发布抑制参数。
5. bondpy 前置记档（`sudo apt install ros-jazzy-bondpy`；装后 bond_timeout:=8.0 才可开 Python 死检；各阶段均不依赖）。

**三方互证执行**：本轮无互证（纯地基）；TEM off 后可选手动一轮网格冒烟确认无回归。
**录制档位**：无（不起正式轮）。
**验收门**：build/test/r0_gate 全绿；launch_testing 切换后复跑绿且隔离域生效；尖峰结论落档（含 C22 注入手段决定）。
**产物**：小 PR 面 + 尖峰记档（testing-log 追记）。
**风险与回退**：TEM off 若致行为差异（理论不应有）→ 活文档同轮回滚；尖峰最坏结论=感知需发布抑制参数 → peach_harvester 小改（非红线），排阶段三前完成。

---

## 4. 阶段二：注入矩阵收口（E2 层；三方互证首秀）

**目标**：M1–M5 门过线并重定基线，建立三方互证制度（后续阶段照此运转）。
**前置**：阶段一 TEM off + 尖峰结论（本轮只依赖 TEM）。

**测试内容**：
1. **基建两件**：`campaign/20260922_dual_tool/corpora.yaml` 案册注册表（名字→类型+seed+规模+expect 来源，A/B 驱动统一取参，消除 seed 散落）；`cross_validate.py` 互证对照脚本（C1 判定薄层：输入 analytic_prior.json + 实跑 jsonl + 注册表 → 三分法计数+逐例分歧表，归因栏引用 C2 产物链接与 C3 视频时间戳；不重写 collect_round/bag_report）。
2. **边界例裁定**：`deep_left_low_axis`（解析与实跑双败 #4 → expect 过期，修 expect）；`near_horizontal_1021_1`（解析成/护栏拒 #3 → 护栏与解析模型口径差，人工已定性「护栏正确拒」，expect 改 deny 后归因关闭）。裁定后 M1 才可能 100%。
3. **A 段**：`analytic_roll_ladder.py` 按注册表案册出逐例先验（秒级）。
4. **B 段**：`run_m1_m5.sh` 全量复跑（域 61、无相机、hollow 档；bag std + rviz 视频随轮）。
5. **C 段**：C1 三分法收口 + C2（collect_round/bag_report 核对在线↔bag 一致）+ C3（#3 分歧例视频取证）。

**三方互证执行**：全流程首秀。**已用磁盘数据预演**（解析梯子 seed 20260922 vs 实跑 09-23 网格）：

| 案例 | 解析预报 | 实跑结果 | 三分法 | 处置 |
|------|---------|---------|--------|------|
| `deep_left_low_axis` | 失败 | outcome=None（两轮一致） | #4 一致失败→expect 过期 | 修 expect 重跑，非实现缺陷 |
| `far_no_ik` | 失败 | 按 skip_ik 期望匹配 | #2 合规 | 记录即可 |
| `near_horizontal_1021_1` | **可行** | 护栏拒 outcome=2 | #3 互证发现 | 口径差定性制度化；expect 改 deny 后关闭 |

**录制档位**：B3 std / B4 每轮录（run_round.sh 编排）。
**验收门**：M1 期望分类 100%（裁定后分母）；M2 PREGRASP≥95%；M3 FULL≥90% 且 completion≥6、绕行比≤1.70；失败 100% 有码；与改动前基线（M1r 19/20、M3r 25/29、M4r 7/30）对照**只许升不许降**；#3 分歧清零 + #5 为零 + 四件套齐（录制完整性门：bag 关键话题无 >2s 缺口、视频时长≈轮次 ±10%）。M2 的 3 例 SKIPPED_UNREACHABLE 与 M3 剔除 rand_03 口径在新报告显式声明。
**产物**：`analysis/injection/m1_m5_report_<date>.md`、`cross_validation.md`、台账 README 轮次表更新；bag 分析后 purge 留索引。
**风险与回退**：注册表落地前 random/stratified 档互证只做 grid 20 例同源子集；M2/M3 率仍差 → 按 A 级问题清单归因（A-P3-3 周期级预算独立跟），不放宽门。

---

## 5. 阶段三：自动化批次链一期（E1 层一期）

**目标**：把 supervisor 批次链纳入 colcon/CI——正路径+安全负路径 9 用例（C01–C05、C10–C13）。
**前置**：阶段一全部（尤其尖峰结论与隔离切换）；阶段二基线数字（断言分母引用）。

**测试内容**：
1. **注入器 `perception_sim`**（测试内节点，协议照抄 `sim_field_targets.py:660-676` 并扩展调度链）：

| 通道 | 内容 |
|------|------|
| 订 `/peach_supervisor/state`（latched） | 取 scene_epoch / harvest_run_id / batch_state / target_id |
| 发 `/peach/perception/target_observations`（reliable volatile d10，~2Hz） | 两段式：进度（collecting_count↑、observations=[]）→ 锁定（locked=true+固定 ID 集+逐帧几何） |
| 发 `/peach/reconstruction/grasp_decision`（transient_local d1） | 按夹具 expect 定 allow/deny 与令牌 |
| 发 `/aubo_io_controller/robot_status`（volatile 10Hz） | 仿真就绪态 |
| （FULL）refined/refit 两路 | 与 sim 轮同构，可选 |

几何源=`perception_constraint_grid.yaml`（20 例，deny_decision 例用于 C10）+`field_pregrasp_cases.yaml`。栈=include harvest_system（mock、camera/imu 关、offscreen、临时 AUBO_RUNS_DIR），`running_stack_pids` 前置守卫保留（战役栈在跑时拒测明示，等位不抢）。工具档案=栈默认 adaptive；tool 恒 false。

2. **用例矩阵（一期 9 例）**——批次态：WAITING_READY=0/DISCOVERY=1/RUNNING=2/PAUSE_PENDING=3/PAUSED=4/COMPLETED=6/RECOVERY_REQUIRED=7/INTERRUPTED=8；mode：PREVIEW=0/OBSERVE_ONLY=1/FULL=2/PREGRASP_ONLY=3：

| # | 场景 | 注入/意图 | 核心断言 |
|---|------|-----------|----------|
| C01 | SURVEY_ONLY 只扫 | intent=2，2 目标 | →COMPLETED(SETTLE)；ledger settle；**零次 ExecuteTarget 派发** |
| C02 | 使能关结算 | intent=0，execution_enabled 不开 | LOCK 后 EXECUTION_DISABLED→COMPLETED(RECORD_DISABLED)；零派发 |
| C03 | PREGRASP 单目标停驻 | execute_pregrasp_only=true+skip_reconstruction，typical 例 | unrefined PREGRASP_ONLY（plan_id `:unrefined`）；SUCCEEDED+recovery_required+completion=2；无 SetIO/CUT 段；state 至 RECOVERY_REQUIRED(7) |
| C04 | ACK 回访不再 Begin | 承 C03，ControlTask(6) | ACK 后回 DISCOVERY→Survey 回访；**BeginScene 只调 1 次**；state_seq 消耗 |
| C05 | FULL 干跑双目标 | 运行期 param set execute_pregrasp_only=false+skip | 每颗 SUCCEEDED、completion≥6、grasped=false、tool 恒关；CYCLE_DONE→回访第二颗；ledger 2 outcomes 对齐 claimed |
| C10 | deny 决策臂侧拦截 | 网格 deny_decision 例+decision allowed=false | 门上拒/FAILED、completion=2、有码；跳果继续不挂批 |
| C11 | 批中 CANCEL_NOW | C05 中 ControlTask(4) | goal 取消、账本 CANCELED、乱序 seq 拒；**栈不 wedge、无在途泄漏**（A-P3-1 级联回归锁，TEM off 后必须仍绿） |
| C12 | 使能断流命令门拒 | C03 中 SetEnables 关/停发心跳（>5s 回落本地参数） | 臂侧门拒（armed/enables 类失败码）、跳果；无门直写 |
| C13 | 操作台语义 | PAUSE(0)/RESUME(1)/SKIP_TARGET(5)+乱序 | PAUSE_PENDING→PAUSED→恢复；SKIP 进 rework；过期 state_seq 拒收 |

3. **断言体系**：期望 FSM 序列=测试进程 `import harvest_fsm.react` 推演（判定只在纯核出现一次）；账本逐字段对期望（claimed/outcomes/completion_level/stage_names）；安全门断言（无 SetIO 事件段、enables 心跳、deny completion=2、RECOVERY 态权限表）；`@post_shutdown_test` 退出码；单用例 wall-clock 上限（正路径 ~120s/FULL ~240s），一文件一组共享栈控预算。

**三方互证执行**：A2 纯核推演（=断言内建）+ 夹具几何 oracle 预报；B=注入器实跑（CI 内 record_level:=std 轻录）；C1=实测对照推演序列（分歧即失败）+ C2=bag_reader 用例内复算事件链完整性（可选档）；C3 留阶段四本地轮。
**录制档位**：CI=E1-CI 行（轻录、无视频）。
**验收门**：9 用例 CI 绿；隔离域合规；r0_gate/industrial_ci 覆盖（CI 预算紧张则先跑一期子集，实施轮定）。
**产物**：`peach_system_tests` 注入器+新 launch_testing 文件、CI 覆盖；**回放塔↔peach_arm gtest 交叉对账接线**（B1 层：回放塔案册 field 14/stratified_200/random_100 作 gtest 夹具喂纯核，`grasp_geometry`/`trajectory_guard` gtest 已在，补案册装载与逐例断言）。
**风险与回退**：尖峰降级路径（§3 阶段一）；CI 时长超预算 → 分组 TIMEOUT+子集；C22 手段若尖峰未定 → 该例顺延阶段四。

---

## 6. 阶段四：自动化批次链二期（E1 层二期 + 本地满配轮）

**目标**：故障注入矩阵全量 + 变体A 路径 + 三方满配人读对照样。
**前置**：阶段三用例绿。

**测试内容**：
1. **故障注入用例（6 例）**：

| # | 场景 | 注入手段 | 核心断言 |
|---|------|----------|----------|
| C06 | 变体A 并行路径 | reconstruct_in_trajectory:=true（默认关） | Build 并行不阻塞即派 FULL；不阻塞主链 |
| C20 | 感知断流 | 锁定后注入器停发 observations | 新鲜度/anchor 收口：跳过或 INTERRUPTED，**必有码无 hang**（方案 §8 A1 调度侧表现） |
| C21 | 永不锁定 | 恒发阶段 A 进度 | WAIT_LOCK 超时→INTERRUPTED+lock 类失败码；不无限等待 |
| C22 | Survey 失败整批中止 | 按阶段一尖峰勘定的手段 | SURVEY_FAILED→整批 INTERRUPTED（不发 BeginScene）；账本 survey_failed |
| C23 | build 超时跳果 | skip_reconstruction:=false+无相机（真实重建采帧门自然超时） | build_timeout→target_skipped 批继续；`observe_build_view_race` 分支至少一条可达 |
| C24 | 执行中取消级联回归 | C05 单颗执行中 goal cancel（非 ControlTask） | 取消后 ACK 补发路径（96cdd79 修复）不回归：CANCELED 终局、state_seq 一致、下一颗可派 |

2. **E1 本地全量轮**（手工/夜间）：15 用例全量 + std bag（分歧例 all）+ rviz 视频——三方齐备的人读对照样，#3 分歧在此视频取证归因。
3. CI 全量矩阵（15 用例）。

**三方互证执行**：同阶段三（CI）；本地轮=C1+C2+C3 满配。
**录制档位**：CI=E1-CI 行；本地轮=E1 本地全量行。
**验收门**：15 用例全绿；故障类 100% 有码无 hang；本地轮三方齐（四件套+cross_validation 引用）。
**产物**：全矩阵 CI 覆盖 + 本地轮四件套。
**风险与回退**：CUT_UNCONFIRMED/SetIO 失败负路径 mock 结构性不可达——留 gtest 层，明确不进 E1（勿试 mock 开 tool，C 级红线必挂不撤退）。

---

## 7. 阶段五：真相机战役轮（E3 层；操作员在场）

**目标**：真实感知噪声下的全链稳定；产出 S3 深度 bag（变体B 实验硬前提）；核心门 S6。
**前置**：阶段二基线 + 阶段三/四无未归因回归；操作员在场；战役 runbook（域 61、等位不抢纪律）。

**测试内容与门**：

| 子阶段 | 内容 | 前置 | 验收门 |
|--------|------|------|--------|
| S3 真相机轮 | 真实场景+stereo/percipio，产出**含运动+深度流** session bag；拍照位自洽（先发 intent:2） | 栈起来 | bag 深度/stereo 话题非空（temporal_k 鬼影先导与变体B 硬数据前提）；P1 门采样 |
| S6 五轮 FULL（核心门） | `e2e_full_unrefined_` ≥5 轮×≥3 目标，skip_reconstruction，tool 恒关 | S3 自洽+阶段二基线 | SUCCEEDED 且 completion≥6 占比≥90%；无 300s hang；grasped=false；事件链全、账本零缺口 |
| S7 IMU（adaptive） | I1–I7 七步（真 IMU+TF 对齐+enable/pause+insert_progress） | 操作员 IMU enable 授权 | 七步全过、servo status=0；开运动不经 authorizeStage 的授权语义单独记录 |

S4（survey/pregrasp 批链冒烟）已被 09-22 两次 live 轮覆盖，台账补记不单列。

**三方互证执行（满配）**：A 腿=**事后解析复核**——真实感知无解析模型，但从 bag/ledger 提取实测目标几何作 roll_ladder 事后案册（预报该实测几何下应可行/该拒，再对照实跑），A 从先验变后验，三方结构仍成立；C2=`stability_metrics.py` bag 几何 3σ 复算（P1 门引擎，在线↔离线现成范式）；C3=逐轮全读（目标少）。
**录制档位**：E3 行——bag all（100G 预算）+每轮视频（run_round.sh --level all --rviz-dur）。
**验收门**：上表三门 + P1 感知稳定门（锁定时延 percipio≤8s/stereo≤2s；3σ pos≤5mm/轴角≤1.5°/袋长≤8mm；10min 零 ID 切换；tf_stale<1%；无>2s 帧缺口）+ 四件套齐。
**产物**：runs/ 四件套 + 战役台账/bags.md + per_round_summary。
**风险与回退**：感知侧 A 级问题（A1 断粮/A2 残差废/A3 谓词不一致等，2026-09-22 方案 §8）影响 S6 稳定性——批内目标数 ≤3 规避，本体修复另行立项不在本方案；每轮收尾 MUST 停栈+pgrep（含 bag record/ffmpeg）。

---

## 8. 阶段六：真机前置（E4，不自动化）

**目标**：申请真机授权前的总门。
**前置**：阶段二～五全部门过 + A 级问题清零（A-P3-1 至少有「TEM off+回归用例」缓解闭环）+ 分支推 Gitee。
**内容**：按 `docs/testing.md` 真机授权前检查清单逐项核对；mock 全绿不证明碰撞/接触力/急停回路/SetIO 刀具回路/真机 TF 外参漂移——照旧 KEEP 真机为最终权威。
**门**：清单全过 + 人工授权（口头/书面，非 yaml/按钮）。

---

## 9. 跨阶段验收门总表

| 阶段 | 门 | 数字/口径 |
|------|----|-----------|
| 一 | 地基绿 | build/test/r0_gate 绿；launch_testing 隔离域生效；尖峰落档 |
| 二 | M1–M5 重定基线 + 互证首秀 | M1 100%（裁定后分母）/M2≥95%/M3≥90% completion≥6 绕行比≤1.70/失败 100% 有码；只升不降；#3 清零 #5 零 |
| 三 | E1 一期 9 用例 CI 绿 | C01–C05/C10–C13；A2 推演=断言内建；隔离合规；B1 交叉对账接线 |
| 四 | E1 二期 15 用例全绿 | +C06/C20–C24；故障类 100% 有码无 hang；本地轮三方齐 |
| 二/三/四 | 解析门（A，前置） | 回放塔绿（analytic_ok 只升不降、分母精确）；prior 与实跑同案册同 seed |
| 二/三/四 | 录制完整性门（带录制轮） | 四件套齐；bag 关键话题无 >2s 缺口；视频时长≈轮次 ±10%；分析后 purge 留索引 |
| 五 | S3/S6/S7 | 见 §7 表 + P1 门 |
| 六 | 真机授权前清单 | docs/testing.md 现行 |

## 10. 风险与开放问题（跨阶段）

| # | 项 | 影响 | 处置 |
|---|----|------|------|
| 1 | 注入契约尖峰不确定（双发/run_id/C22 手段） | 阶段三排期 | 阶段一首任务+三级降级；半天出结论 |
| 2 | 「M1 复跑 20/20」与磁盘 18/20口径差 | 阶段二分母 | 阶段二复跑自证；期间一律按磁盘 18/20 |
| 3 | A-P3-1 根因未修（TEM off 是缓解） | 真机若复现仍楔死 | 独立跟；C11/C24 回归锁；阶段六前置要求缓解闭环 |
| 4 | A-P3-3 周期级预算缺失 | hang 仅 GOAL_TIMEOUT 兜底 | 独立事项，不阻塞本方案 |
| 5 | E1 用例时长×CI 预算 | industrial_ci 超时 | 一文件一组共享栈+分组 TIMEOUT；必要时 CI 只跑子集 |
| 6 | 感知侧 A 级问题（A1/A2/A3…） | S6 稳定性 | C20 锁调度侧表现；修复另行立项 |
| 7 | bondpy 缺失争议（dpkg 已装 vs 批次7 勘定） | Python 死检开启 | 各阶段不依赖；开启前实测一次 |
| 8 | mock 开 tool 必挂（C 级红线） | — | E1 全用例 tool 恒 false，禁试 |
| 9 | 案册 seed 三处不同源 | 互证无法逐例 | 阶段二注册表落地前只做 grid 20 例同源子集 |
| 10 | Python oracle 与 C++ 纯核逆移植漂移 | 解析先验失真 | 阶段三 B1 对账接线后有 C++ 真核背书；漂移进 #2/#3 归因 |
| 11 | CI headless 无显示 | B4 视频腿 CI 不可达 | 分层（CI=C1+轻量 C2）；不为此装 Xvfb |
| 12 | 录制磁盘预算与残留 | 100G 占满/违反 MUST | purge 纪律+录屏脚本自带 cleanup+pgrep 复核含 ffmpeg |
| 13 | 录制对实时性扰动 | 主链节拍 | 只读订阅已按会话 bag 设计；std/all 分级；C2 异常时降档对照复跑 |

每轮收尾照 MUST：停栈 + pgrep 复核 + 攒批提交纪律（里程碑才 commit+push）。

---

### 附录 A：E1 用例注入参数速查（阶段三/四）

| 用例 | intent | execute_pregrasp_only | skip_reconstruction | 夹具 | 注入器行为 | ControlTask |
|------|--------|----------------------|--------------------|------|-----------|-------------|
| C01 | 2 | — | true | 2×allow | 进度→锁定 | — |
| C02 | 0 | true | true | 2×allow | 进度→锁定（execution_enabled 不开） | — |
| C03 | 0 | true | true | 1×typical | 进度→锁定+decision allow | — |
| C04 | 0 | true | true | 2×typical | 同上 | ACK(6) |
| C05 | 0 | false（param set） | true | 2×allow | 进度→锁定+decision allow | — |
| C06 | 0 | false | true（变体A 开） | 2×allow | 同 C05+Build 并行 | — |
| C10 | 0 | false | true | deny 例 | 锁定+decision deny（袋径>D_inner） | — |
| C11 | 0 | false | true | 2×allow | 同 C05 | CANCEL_NOW(4) 中途 |
| C12 | 0 | true | true | 1×typical | 同 C03；中途停发 enables/SetEnables false | — |
| C13 | 0 | true | true | 2×typical | 同 C04 | PAUSE/RESUME/SKIP+乱序 seq |
| C20 | 0 | true | true | 1×typical | 锁定后停发 | — |
| C21 | 0 | true | true | — | 恒进度不锁定 | — |
| C22 | 0 | true | true | — | 正常（Survey 阶段即失败） | — |
| C23 | 0 | true | **false** | 1×typical | 正常锁定（重建自然超时） | — |
| C24 | 0 | false | true | 2×allow | 同 C05 | goal cancel（非 ControlTask） |

### 附录 B：关键文件路径索引

- 现有集成测：`src/peach_system_tests/test/test_mock_launch.py`（CMake：`add_launch_test` 待切换）
- 注入协议参照：`scripts/sim_field_targets.py:660-676`（发布/客户端）、`:880-894`（robot_status）
- 解析先验资产（A 段）：`campaign/20260922_dual_tool/scripts/analytic_roll_ladder.py`、`campaign/20260922_dual_tool/analysis/injection/analytic_roll_ladder.json`；`src/peach_system_tests/test/{replay_oracle.py, replay_baselines.json, test_replay_approach.py}`
- 录制与复盘资产（B3/B4/C2/C3）：`src/peach_observability/peach_observability/{params.py:121/196, observability_node.py:302-304, bag_reader.py, bag_report.py}`；`scripts/record_rviz_harvest.sh`；`scripts/collect_round.py`；`campaign/.../scripts/stability_metrics.py`；`scripts/purge_analyzed_bags.py` + `campaign/.../bags.md`
- 夹具：`src/peach_arm/test/fixtures/perception_constraint_grid.yaml`（20 例）、`field_pregrasp_cases.yaml`
- FSM 纯核：`src/peach_harvester/peach_harvester/supervisor/harvest_fsm.py`（react/event_for_outcome/permissions_for）
- 世代与锁定：`src/peach_interfaces/msg/HarvestState.msg:60`、`srv/BeginScene.srv`、`executor_node.py:1105/1662-1667`
- TEM：`src/aubo_e5_moveit_config/config/controllers.yaml:5`、`controllers_mock.yaml:5`
- 战役：`campaign/20260922_dual_tool/{README.md, scripts/{run_m1_m5.sh, m1_m5_report.py, run_5rounds.sh, imu_steps.sh, run_round.sh}}`
- 方案依据：`reports/2026-09-22-real-cam-mock-dual-tool-validation/report.md`、`campaign/20260922_dual_tool/analysis/{injection/m1_m5_report.md, p3_findings_20260923.md}`
- 磁盘基线证据：`runs/sim_field_targets_20260923_{153659,173249}.jsonl`（M1 18/20 两轮一致）
