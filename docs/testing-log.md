# 测试过程记录

本文只记真机/审查轮次，**不写现行怎么跑、不写验收口径**。流程、命名、门与命令：[testing.md](testing.md)。原始 jsonl 在工作区 `runs/`（结构化文本已入库随仓推送，图像/mcap/点云二进制仍只留本地）与 `_archive/runs/`（整体不入库），不要删。

改行为或 yaml 不改本文；补一条实测时同一轮改本文对应小节。约束：[AGENTS.md](../AGENTS.md)。

---

## 怎么读

- **重构后代码**（阶段执行器）：09-01 下午起的 `field_pregrasp_20260901_*`。现行停袋底以 **1757** 为准。
- **重构前代码**（行为树）：08-21～08-31 与 09-01 晨间。失败模式仍可对照，数字不得当现行合格证据。
- 方向/定位对错只认停预抓取后的现场目视。`allowed`/余量/RMSE 只作记录。
- ACK 前自动 `summary.md` 常把 Hold 记成 `unfinished`；以技能 `[SUCCEEDED] PREGRASP_ONLY` 与现场停位为准。

复算归档基线（只读 `runs/` / `_archive/runs/`，不写不删）：

```bash
python3 scripts/replay_metrics.py
```

---

## 量化基线（归档；行为类出自重构前）

硬件事实（相机 ~2.5 FPS 等）与机型无关，继续有效。行为类数字是重构前表现，只作排障对照起点。

| 门 | 归档事实 | 变好长什么样 |
|----|----------|----------------|
| 相机速率 | ~2.4–2.5 FPS（log 2.43；中位间隔 0.4 s） | 新 live hz 覆盖前，设计仍按 2.5 |
| 有效视角 | 接触轮常 4–6；成功对照 15 | 停稳对齐后 skip 不以 `robot_not_static` 为主 |
| 静止跳过 | lin target_1：147 拒中 93 次（63%） | 有效视角/秒接近感知 FPS 的停走子集，而非 ~0.2 |
| 轴门 35° | 完全错轴诊断；套入许可改动态预算 | 不得为提速放过双表面；也不得把 35° 当套入角门 |
| TF | 接触轮 `tf_failures=0` | tf_failures 上升须停 |
| 新鲜度 | 08-24 观察失败 5/10，主因 `selected_target_stale` | 未测得 EMA 时门限 3.0 s；测得后只放宽 |
| MTC | 接近 25–29 s 撞 **12 s** 接触护栏；笛卡尔 0.967；`(0/1)` | 超护栏保持 `skipped_unreachable` |
| 会话 | 批次结束后 events 再写 ~65 min | 结束后不再追加空观测 |
| 工具 | 全程 `tool.enabled=false` | 日志无 SetIO |

数据根：`_archive/runs/root_2026-08-24/`。工作区 `runs/` 可缺失。

解读陷阱：批次 `summary.md` 重建终值常是 IDLE / `captured_views: 0`——用逐目标 `captured_views_max`。GraspDecision 心跳 `not_ready` 占多数 ≠ 精化从未 ACCEPT。感知单帧 ACCEPT 远少于 REOBSERVE，不能当批次成功率。

---

## 重构后：PREGRASP_ONLY

档位与通过判据见 [testing.md](testing.md) §4。下列为轮次事实。

### 2026-09-01 下午（重构后代码首批真机）

`1440`：`target_12/13` 首次走通 观察→重建→融合→再确认→MovePregrasp，卡 MTC `ptp to on-axis pregrasp` 0 解（预抓取半径 0.92/1.07 m 物理超程）；`radial_budget_negative`/`bag_d95_exceeds_tool` 双预算同拒。随后上线 **TCP IK 选果预检**（`CheckReachability`）+ 有效深度窗：`1500` 四颗全窗过滤零浪费；`1540` IK 过滤 0/1/3、选中 `target_2` 真做观察短移，卡 `neighbor_gap`（邻锚 58.5 mm，105 次拒帧）——近距双检。**现场定夺：近距双检先做检测框大的（小框多为叶遮残片/误检）**：选果次序改 priority+框面积降序、串扰门小框豁免（面积比 2.0）。同日随后：观察行程门固化 `observe_max_total_joint_travel_rad=4.0` 进 yaml；监控修三处——终局事件并入 `failure_code`、ACK/暂停/恢复审计事件、summary 加验收门对照与账本互引。

### 09-01 17:04 `field_pregrasp_20260901_1704`（重构后首次 Hold）

入口外 70 mm、再后撤 100 mm。验收门「到预抓取停住 ≥1」✓。`target_2` 观察 2 视过门后精化预抓取半径 ~0.93 m，MTC PTP 0 解 → `skipped_unreachable`（SELECT 用感知入口过 IK，精化入口仍超程）。`target_1` 观察 LIN 0.12 m / 8.8 s → 拍照位 PTP → 预抓取 PTP 5.71 s goal-hold → **HoldPregrasp `SUCCEEDED` + `recovery_required`**。实测 TCP `[0.298, -0.639, 0.367]` 与规划预抓取重合。`allowed=false`（`bag_d95_exceeds_tool`）未拦预抓取。无 SetIO、无 `neighbor_gap` 拒帧。**现场目视 `target_1`：方向与位置良好，轨迹可行。** 账本 `runs/field_pregrasp_20260901_1704/`。

### 09-01 17:57 `field_pregrasp_20260901_1757`（现行 0 后撤、停拟合袋底）

命令见 [testing.md](testing.md) 就绪单。起栈前示教器手动回到 `global_photo_pose`（相对 SRDF 最大 |Δq|=0.0001 rad）。`target_2` SELECT `ik_no_solution`。`target_1` 观察 LIN 0.118 m / 8.28 s → 回拍照位 3.94 s → 预抓取 PTP 5.53 s goal-hold → **HoldPregrasp `SUCCEEDED` + `recovery_required`**。入口=预抓取=`[0.304, -0.614, 0.536]`（相对 1704 TCP 沿袋轴约 +17 cm，与去掉 70+100 mm 后撤一致）。融合轴 `[0.042, 0.136, 0.990]`；到位 TCP Z 与该轴重合（`alignFrameZ` 只转工具 Z、保留拍照位滚转，倾斜约 8°，目视不像大拧腕）。`allowed=false`（`bag_d95_exceeds_tool`）未拦。无 SetIO。ACK 前 summary 计 `unfinished`。**现场目视 `target_1`：方向与定位中上水平，只需微调。** 过程 `runs/field_test_20260901/log.md`；账本 `runs/field_pregrasp_20260901_1757/`。

### 09-03 16:04 `field_pregrasp_20260903_1604`（先 Survey 再 Begin 开窗；未 Hold）

起栈已在拍照位（|Δq|≤0.0001 rad）。档位：调度+技能 `execution`/`grasp` 开，`tool.enabled=false`，`execute_pregrasp_only=true`。链：`surveying` → `photo_pose_reached` → `collecting` → `round_locked`（`scene_epoch=1`）。首轮 SELECT 三颗均 `ik_no_solution`；回访 Survey（不 Begin）后再锁，派 `target_0`/`target_2`/`target_1`。三颗均 `observe_failed`：`target_0` 有效视点 0/1（`neighbor_gap` 79 / `missing_mask` 13）；`target_2`/`target_1` `selected_target_stale`（重建全程 `missing_mask`）。`termination_reason=no_targets_succeeded`；到预抓取停住 0；无 SetIO。结束 TCP 回到拍照位 `[0.302, -0.232, 0.708]`。过程 `runs/field_test_20260903/log.md`；账本 `runs/field_pregrasp_20260903_1604/`。

### 09-03 17:09 `field_pregrasp_20260903_1709`（SELECT IK 改停位几何后首测）

SIGINT 旧栈后用 16:57 编的 `peach_manipulation` 重起；开批前在 `global_photo_pose`（|Δq|≤0.0001）。后撤现行 0.03 m。`CheckReachability` 对入口做后撤 + `alignFrameZ` 再 IK。`target_0` **SELECT 过**（`target_dispatched`，不再 `ik_no_solution`）；观察 2 视 refit ACCEPT。`PREGRASP_ONLY` 卡 MTC `ptp to on-axis pregrasp (0/1)`：`ValidateSolution` `INVALID_MOTION_PLAN`（路径上 tcp 姿态相对目标误差 ~0.48 rad > 容差 0.35 rad / 20°）。`skipped_unreachable`；到预抓取停住 0；无 SetIO；结束回拍照位。回访 `target_1` `out_of_depth_window:2.20m;ik_no_solution`。过程 `runs/field_test_20260903/log.md`；账本 `runs/field_pregrasp_20260903_1709/`。

### 09-03 17:16–17:18 连开两批（第三批 Hold 后停）

`1716`：同 1709。`target_0` SELECT 过、2 视 ACCEPT，MTC `ptp to on-axis pregrasp (0/1)` → `skipped_unreachable`；46.8 s；回拍照位。账本 `runs/field_pregrasp_20260903_1716/`。

`1717`：`target_0` SELECT 过、观察后 **HoldPregrasp `SUCCEEDED` + `recovery_required`**。到位 TCP `[0.388, -0.598, 0.510]`（与 1624 Hold 几乎同点）；姿态相对拍照位小倾（q `[-0.053, 0.110, -0.038, 0.992]`）。`allowed=false`（`bag_d95_exceeds_tool`）未拦。无 SetIO。ACK 前 summary 计 `unfinished`。未发 1718。账本 `runs/field_pregrasp_20260903_1717/`。

`1740`：去掉 PTP 段 20° OrientationConstraint 后，MTC `ptp to on-axis pregrasp` **规划并执行**。关节护栏 `joint_total=11.12` / `joint_max=3.84` 过 12/6.1。TCP 拍照 `[0.302, -0.232, 0.708]` → Hold `[0.384, -0.614, 0.518]`：路径 1.39 m、弦 0.44 m、绕行比 **3.21**、相对弦偏离 0.51 m、途中 z 抬到 **1.065 m**（比拍照位高 35 cm）、相对目标最远回退 0.24 m。Hold 关节相对拍照位跳构型（shoulder `0.425→2.115`，foreArm 反号，wrist1 `1.462→-2.374`）。现场目视：**绕行轨迹不可接受**。精化轴约 23°，`keypoint_cloud_axis_conflict`，`allowed` 未过。无 SetIO。臂停在 Hold，`recovery_required=true`，未 ACK。账本 `runs/field_pregrasp_20260903_1740/`（若目录存在）。随后源码改为接触到预抓取只走 LIN/CIRC，失败不改 PTP，并加笛卡尔绕行审查。

技能方向：定位用感知入口；工具 Z 对齐感知袋轴；滚转不抄感知四元数。详见 architecture 接触段与 `alignFrameZ`。

### 09-10 mock 全链路回放（09-09 现场逐目标坐标；审查轮）

工具：`scripts/sim_field_targets.py`（注入 runs/ 账本记录的 entry/axis/底/颈 → 驱动真实技能节点 `ExecuteTarget PREGRASP_ONLY`，逐用例实测 TCP 弦/路径/绕行比/偏离/回退）与 `scripts/trajectory_watchdog.py`（执行中 15 Hz FK 监测，超门 `~/cancel_cycle` 停轨留档；仅 `running=true` 时累计）。用例集 `src/peach_manipulation/config/field_pregrasp_cases.yaml` `targets_20260909`：09-09 各批 14 个有效目标坐标（另两条感知外参故障期界外坐标 |entry|≈2.1 m 不属本项目作业范围，未参与）。**14/14 `SUCCEEDED` 停预抓取（recovery_required、无 SetIO），实测绕行全部门内、watchdog 零违例**：直连/LIN 类比 1.0–1.42、偏离 ≤0.16 m；两个远斜轴目标走转移级（staging PTP 弧 2.31/0.323/0.101 与 2.41/0.34/0.112，过转移级门 2.5/0.40/0.15）；近伸展目标走直连 PTP 兜底 1.28/0.187/0。行为改动（同轮改 architecture/testing）：LIN/CIRC 全滚转失败后新增转移级（fly-over 走廊 → staging PTP → 直连 PTP 兜底），转移级笛卡尔门 2.5/0.40/0.15（1740 无约束 PTP 3.2/0.51/0.24 仍拒）；MTC 解显示默认关闭（滚转候选在臂动前实时发 RViz 造成轨迹抖动）。mock 复现要点：`/joint_states` 为字母序须按名映射；Hold 构型回拍照位走示教器口径（仿真用 JTC 两段插值复位等价）；期间另有并行决策 0017 参数库迁移的三个在途 bug 一并修复（`rclcpp::ParameterDescriptor` Jazzy 别名、params.py 嵌套类作用域、RULES 单条规则未包装）。记录 `runs/sim_field_targets_20260910_013938.jsonl`、`runs/trajectory_watchdog_20260910_013920.jsonl`。

---

### 09-10 mock 最近距离接近重写（STOMP 轨迹优化；审查轮）

用户定调：接近必须**最近距离/垂直正面**，不得绕行（实际果园枝叶环境绕远必碰枝）。诊断（`scripts/probe_chord.py`）：拍照位→预抓取直弦在远斜轴目标（1437_0/1503_0）**中段 IK 跳支且构型插值 80% 点自碰**——直弦物理不可行，逐滚转笛卡尔 fraction 0.14–0.62，此前的 staging PTP 弧（比值 2.2–2.4）即绕行来源。按主流轨迹优化重写 `grasp_task.cpp` 接近为三层：LIN/CIRC → **STOMP**（Jazzy `moveit_planners_stomp`，直弦关节插值种子+碰撞/平滑/控制代价，最近构型关节目标——位姿目标内嵌单次 IK 在边界位姿 INVALID_GOAL_CONSTRAINTS）→ staging PTP 兜底（非最短，仅直线真不可达）。删除全部历史补丁（CartesianPath 走廊 / fly-over / 弦上换支重播种 / 弦采样 DP——后两者实测死于自碰边界，留档见 git）。两处 launch 挂 stomp 管线（moveit_configs_utils 默认配置；包级覆盖会整段替换管线配置致 move_group 崩溃，已回退默认）。**14/14 全部到位且近直线**：10 例比 1.07–1.15/偏离 ≤0.135 m、1510_2 1.28、1503_0 1.38、1437_0 1.15–1.65（STOMP 随机优化波动，均过门）、1021_1 完美 1.0；回退除 1437_0 ≤0.066 外全 0。watchdog 0 违例。MTC 解显示保持关闭（防候选闪跳）。记录 `runs/sim_field_targets_20260910_043154.jsonl`、`runs/trajectory_watchdog_20260910_043144.jsonl`。

---

### 09-11 mock 接近轨迹形状（typical 包络 30 随机；审查轮）

现行接近已是 staging PTP + 轴向 LIN（G/under 单弦与 STOMP 均已删）。本轮只验 **TCP 路径形状**（不得 1740 式大绕行、不得大拧转），不评方向/定位。工具 `scripts/sim_field_targets.py`，mock `harvest_system`，`PREGRASP_ONLY`，`skip_observation`。默认采样改为 `--envelope typical`（`axis_z≥0.70` 且 `|entry|≤1.02`，对齐现场多数袋）；`--envelope algorithm` 含近水平袋，只作压测。同 seed `20260911`、`--random 30 --velocity 1.0`：

1. **未开笛卡尔/姿态门**（`runs/sim_field_targets_20260911_094313.jsonl`）：全量 20/30、typical 子集 8/9。`rand_10` 绕行比 2.33、途中抬 ~27 cm；成功接近 TCP 测地线 108°–180°。
2. **打开笛卡尔 1.8/0.25/0.08 与 TCP 姿态 90°、滚转只 keep-roll ±30°/±60°**（`runs/sim_field_targets_20260911_100331.jsonl`）：全量 17/30、typical 仍 8/9。从拍照位成功接近绕行比 ≤1.46；108°/90.4° 被姿态门拒发。staging 笛卡尔审查此前因 `planTaskOnly` 误绑接近键而未生效，已修为走 `staging_*`。
3. **typical 采样 + 腕轴加权 IK、当前+4 随机种子取最近 5 候选**（`runs/sim_field_targets_20260911_102546.jsonl`，TCP `runs/idle_20260911_100314/tcp_trajectory.jsonl`）：**26/30 到位**。从拍照位成功接近绕行比 ≤1.70、姿态 ≤71°，无抬到 ~1 m。4 例均规划后护栏拒发，未 silently 执行坏轨：`rand_01`/`rand_23`（同源 1503_0 扰动，−x 斜入）轴向 LIN 加速度超限后兜底 PTP **12.91 / 12.72 rad > 12**；`rand_16`（左入口 +x 轴，现场少见）回退 **0.139 m > 0.08 m**、绕行比 ~3（1740 同类抬弧）；`rand_24` 五条 PTP 弧均 clip 袋囊圆柱（r≈0.10–0.11 < 0.12）。未放宽 12 rad / 8 cm 回退 / 12 cm keepout。`--velocity 1.0` 时脚本 0.6 s 静止切段常把回拍照位与接近并段；失败例 `from_photo=false` 路径常是上一段返程，形状以 `from_photo=true` 与日志 `MTC 接近笛卡尔/姿态审查` 为准。

### 09-11 mock 规划加速 + 感知算法包络 100 随机

同 seed `20260911`、`--random 100 --envelope algorithm --velocity 1.0`。执行路径改为 MTC `plan(1)`（不再为凑满 `mtc_max_solutions` 5 耗尽 `planning_time_s`）；回拍照位 PTP 时限 `photo_ptp_planning_time_s` 0.5 s，失败再 OMPL 3.0 s；`select_goal_joints` 各滚转并行 IK（KDL `setFromIK` 加锁）。**不**并发 100 颗 `ExecuteTarget`（单臂）。护栏未放宽。

`runs/sim_field_targets_20260911_151609.jsonl`：墙钟 **14.5 min**；单例 `elapsed_s` 中位 **4.63 s**（min 1.41 / p90 6.63 / max 9.84）。全量 **54/100** 到位；现场 typical 子集 **28/32**；接近袋囊 keepout 12。从拍照位成功接近绕行比 1.00–1.72。失败主因：MTC 规划 21、staging 无 IK 14、绕腕累计 3、笛卡尔 3、姿态 2、keepout 2、回退 1。近水平（`axis_z<0.40`）仅 3/16。未放宽 12 rad / 8 cm / 12 cm / 90°。

### 09-11 解析覆盖（不执臂；10000 分层位姿）

`python3 scripts/analyze_approach_envelope.py --n 10000 --seed 20260911`。复刻感知上半球 / `|entry|≤1.15` / 拍照弦 ≤0.80，以及 TCP 测地线 90°、轴向 LIN keepout。主路径 PTP 的累计行程与弧绕行**不**在闭式里。分层：typical 40%、中斜 20%、近水平 20%、远入口 10%、长弦 10%，另钉 14 个现场袋。

`runs/analyze_approach_envelope_20260911_154345.jsonl`（1.8 s，10014 行）：闭式 **9961/10014**；typical **4246/4246**。失败 53：近水平轴对拍照位 tool-Z 测地线 >90° 共 52；现场 `1639_0` `|entry|=1.158>1.15` 计 perception。拍照→staging 直连弦穿囊 2379/10014（LIN 对照，不计 analytic_ok——这就是删 G 弦、改 staging PTP 的几何原因）。axis_z / |entry| / 弦长分箱无空档。

随后把姿态门与算法包络对齐（感知允许水平）：绝对上限 **110°**（keep-roll γ≈90°、±60° 滚转闭式 γ(90,60)≈105°），相对起止余量 **20°**（仍拦对轴只需 20° 却中途拧到 108° 的 PTP）；算法 `|entry|` 上限 **1.16**（纳入 1639_0）。不放宽 12 rad / 8 cm / 12 cm。重跑同 seed 见下条。

### 09-11 约束对齐后再解析 10000

同 seed `20260911`。`runs/analyze_approach_envelope_20260911_155716.jsonl`：闭式 **10014/10014**（含现场 14/14、近水平 2000/2000）。弦穿囊对照仍约 2362（不计 analytic_ok）。mock 执臂须重启 harvest 才加载新二进制。

### 09-14 接近约束四层（果实胶囊 / octomap / 近果降速 / 接触纯核；审查轮）

决策 0019。mock `harvest_system` `hardware_mode:=mock camera_enabled:=false`，未授权真机。工具半径改为有限圆柱（无端球）；octomap 订 `/camera/depth_registered/points`，规划系 `world`。

- 解析不变量：`python3 scripts/analytic_constraints.py --n 10000 --seed 20260911` → **0/8280**（field 5000 / extended 3000 / grid 280）。I1 轴向 LIN 工具圆柱不触果。
- 接触纯核：`python3 -m pytest src/peach_manipulation/test/test_contact_monitor.py` PASS（关/短基线/摩擦缓升/尖峰/斜率）。
- octomap：move_group `Listening to '/camera/depth_registered/points' using message filter with target frame 'world '`；无 `No sensor plugin specified`。mock 无点云=空地图。
- mock typical 100：`python3 scripts/sim_field_targets.py --random 100 --seed 20260910 --velocity 1.0`。`runs/sim_field_targets_20260914_114320.jsonl`：全量 **89/100** 到位。从拍照位成功 70/70，绕行比 1.27–1.65、回退 0，脚本后检果实胶囊 0。11 失败均 `from_photo=false`（与 09-11 切段口径相同，不当接近形状）：果实胶囊 1、回退 4、偏离 1、绕腕 1、staging IK 2、MTC 规划 2。未放宽 12 rad / 8 cm / 绕行三门。接触检测默认关。无 SetIO。

---

## 重构前：PREGRASP_ONLY 里程碑

**2026-08-28：** 轮次 A `field_pregrasp_20260828` 重建 ndarray `or` 崩溃（已修）。轮次 B `field_pregrasp_20260828b` 重建未崩：`target_0` 技能锁定集未跟上拒 OBSERVE；`target_4` 约 2 机位/4 帧，圆柱 RANSAC 与关键点轴冲突 → `allowed=false` / `refined_quality_not_allowed`，未进 `MovePregrasp`/`HoldPregrasp`。无 SetIO。随后改为：包络否决不拦预抓取、融合几何与接触许可拆开、入口侧向贴体积、独立剪切参考。轮次 C `field_pregrasp_20260828c`：六节点 Active 后开批，`target_0` 观察约 15 s 有效视点 0/1 → `observe_failed` / `insufficient_views`，仍未到预抓取，无 SetIO。轮次 D–D6 只看 Debug Image：上半球约束后 `target_1` 箭头朝左略上。轮次 E `field_pregrasp_20260828e`：开执行/抓取后 Survey 过，`target_0` `build_start_timeout`，`target_1` `build_rejected`（取消 Build 后未等结束就派下一颗），未进预抓取，无 SetIO。源码已改为取消后等待。轮次 F `field_pregrasp_20260828f`：`target_1` 观察约 17 s 有效视点 0/1 → `insufficient_views`，仍未到预抓取，无 SetIO。根因：袋融合后写 `geometry.jsonl` 对 `cut_pose` ndarray 用了 Python `or`，被当成 TSDF 积分失败并回滚体积，故 RViz 无 TSDF Cloud、技能有效视点 0。已修：写点不用 `or`；融合失败不回滚已积分体积。轮次 G `field_pregrasp_20260828g`：两颗均积分（各 2 视、TSDF ~2000 点、`refit ACCEPT`），观察门过；`PREGRASP_ONLY` 发了，`ptp to on-axis pregrasp` MTC 0/1，未到位、无 SetIO。批次结束后体积复位，RViz TSDF Cloud 会空，须在观察/重建进行中看。全图与逐门实测：`runs/field_test_20260828/`。

**2026-08-31：** 拍照位再最短路径。`field_pregrasp_20260831_1554`：发现 3、尝试 3、成功 0，无 SetIO、未 HoldPregrasp。`target_1` 观察 2 视 refit ACCEPT，PTP 回拍照位 goal-hold，随后单段 PTP 到预抓取 10.79 rad / 单轴 4.23 > 10 / 3.2 → `skipped_unreachable`。随后护栏改为 **12 / 6.1**。16:33 / 16:36 重测（`1633`/`1636`）三颗均 `observe_failed`（EMA 预测收口砍掉 8 cm 第二机位）。17:00 `field_pregrasp_20260831_1700`：观察停准则改为覆盖/次数用尽、步长 0.15 m。`target_1` 短移 0.117 m、基线 10.82°、PTP 预抓取 10.96 / 4.16 过 12 / 6.1，HoldPregrasp `SUCCEEDED`+`recovery_required`，无 SetIO。ACK 时调度因 `is_service_ready` 崩溃（已改为 `service_is_ready`）；另两颗未派。方向只在现场评。账本 `runs/field_pregrasp_20260831_1700/`；综述 `runs/field_test_20260831/log.md`。

**2026-09-01 晨间：** 全程 PREGRASP_ONLY，未 Hold。`field_pregrasp_20260901_0900`：两颗 `observe_failed`（锁定集竞态；0.15 m 观察 LIN 累计 **2.63–3.70 rad** 被默认 2.5 拒）。本会话运行时 `observe_max_total_joint_travel_rad=4.0`（随后固化进 yaml）。`0907`：`target_1` 观察 LIN 约 8.3 s / 0.15 m 过 4.0，到位后 `selected_target_stale`；重建全程 `missing_mask`（有效深度 EMA≈0.27）。无 SetIO。过程 `runs/field_test_20260901/log.md`。

---

## 重构前：接触与观察（工具 IO 关）

过程结论：`_archive/runs/root_2026-08-24/web_runs/field_test_20260821/log.md`；综述 `reports/process-data-analysis-2026-08-24.md`。

### 08-21

整栈 Active，相机 ~2.4 fps，手眼 TF。Survey → 并行 Build+OBSERVE → FULL(`skip_observation`)。静止采帧。35° 轴门拦住 ~49°。opt3：`target_0` 4 视角轴 ~11° `allowed=true`，再确认「未获得新鲜观测」但跟踪 OBSERVED → `skipped_quality`；`target_1` 5 视角轴 ~49°，MTC 预计 29 s > 12 s → `skipped_unreachable`。无 SetIO。接触验收门当时未通过。

### 08-24

再确认改为 OBSERVED 用当前锚点 + `updated_s`。`grasp`：`target_1` 插入 0.967，当时 `min_fraction=1.0` → skip。`grasp4`：两目标 4 视角，接近预计 25 s / 27 s > 12 s → `skipped_unreachable`。未下发接触轨迹。

当日后源码已改为：当前位采帧 + 最多一次 12° 短 PTP；接触沿检测轴短程 LIN（未对轴则 PTP 到预抓取点），禁止接触段 OMPL；`min_views=2`；`min_fraction=0.95`；拍照位过 `transit_max_*`；不预填 EMA。**未再宣称抓取成功。**

### 08-25

开抓取关工具。许可后 9 s 与 12.6 s 的正常接近曾被 12 s / 4–8 rad 当成绕行拒掉。护栏改为 **20 s / 10 rad / 单轴 3.2**（仍拒 40 s 爬行与 4.5 rad 绕腕）后，18:53 `field_full_20260825_1851`：`target_1` 接近/插入/撤离均 goal-hold（12.4 s / 3.0 s / 4.2 s），工具关、无 SetIO；`target_0` 仍 `insufficient_angular_baseline`。方向准不准以现场目视为准。带工具采摘未做。实验室两果均应走到接触干跑。

### 08-31（接触护栏与预抓取）

`execute_pregrasp_only=true`，开抓取关工具。时长门已关（`*_max_duration_s=0`）。同日早班：晨间 LIN 远移+slerp `NO_IK` / 工作空间；1347 观察硬可达过滤后无候选（随后去掉硬过滤）；1351 `target_0` LIN 规划过、IK 过、轴 ~11.7°，被当时 20 s 时长门拒（行程 8.2 rad，慢直线不是绕行）；1405 从观察 look-at 出发 LIN `NO_IK`。15:54 起见上节 PREGRASP_ONLY。

### 09-01 代码审查修复（P0 安全底线，未上真机）

全链路逻辑审查后先行修复四处高危 + 两处索引错位，行为回到文档既有口径，无需改验收门：

1. **身份分配死循环**：手写 Munkres 调整步方向写反（加错行集合，搜索区净变化为零，造不出新零点），密集果簇下感知 worker 线程静默挂死。改用 apt `scipy.optimize.linear_sum_assignment`（禁止边仍以大有限代价参与、结果按有限性过滤，语义同旧接口）；离线 500 密集门控用例无挂死、200 矩形用例与暴力枚举同最优。
2. **plan-only 泄漏真机运动**：plan-only 预览失败经 BT Fallback 落入 `AcquireReconstructionViews`（execute 硬编码 true）会真的移动机械臂。`btAcquireViews` 入口加 plan-only 安全门（只规划、终态 PLAN_READY）。P2 重写后由模式 switch 结构性保证。
3. **手动周期旧目标钉残留**：`start_cycle` 不清上一 action 周期的 `cycle_target_id_`，会按已 HARVESTED 目标的陈旧锚点执行。手动分支补清。
4. **切断假确认**：删除 `confirmFeedback(false)` 伪调用；反馈未接线前 `tool.enabled=true` 终局保持 FAILED/CUT_FEEDBACK_TIMEOUT（有意保守侧）。
5. **球内点索引错位**：果线球拟合 inliers 是「法线有效子集」下标，改用同一子集取点（原全量索引典型场景约 1/3 内点为离群）。
6. **圆柱抛光抽稀错样本**：内点 >800 时改为对内点下标等距抽稀（原对全点云采样，混入外点污染袋轴）。

### 09-15（真 IMU servo 跟随，mock 臂）

真 IMU（CH343 `1a86:55d3`，串口 `/dev/imu`，udev 规则补 55d3 后主路径通）+ mock 臂全链路 servo 跟随验证：bringup mock → 拍照位 → `serial_imu.launch.py use_rviz:=false tf_parent_frame:=tcp align_to_parent:=true` → `imu_follow_servo.launch.py` → enable → `motion.enabled=true`。启动 145ms 自动采对齐，清掉上电残差 rpy (-3.97°, 0.56°, 34.74°)（无磁 yaw 占大头）；`/imu/data` 与 tcp 姿态一致。开门后静置关节 5 s 不动、`command_twist` 全 0（死区+位置保持工作）；`/moveit_servo/status`=0（echo 须 BEST_EFFORT 才收得到）；`/joint_states` echo 的 position 顺序是字母序，与冻结关节序无关，核对时勿误读。用户手动转动 IMU，RViz 臂随动确认可行。真机臂运动未动（mock），工具未碰。备注：独立起 serial_imu 接臂必须带 tcp+align 两参（默认 world/不对齐，README §5 有载），当轮先漏带后纠正。

### 09-16（相机深度帧率根因分析，PS800-E1，台架）

问题：彩色+深度+点云三路全开时整机 2.43 fps，为何开深度就把帧率从彩色单路 15 fps 拉下来。全程仅台架相机测试，未动臂、未 SetIO；所有实验只经 launch 参数与 install 拷贝临时改动，结束时 `install/` 参数文件逐字节还原、`src/` git 干净、无残留进程。原始数据与一次性探针工具在 `/tmp/percipio_fps_test/`（重启即失，结论已录此处）。

**相机身份**：Percipio PS800-E1（SN 207000152740），散斑结构光双目 + RGB，tycam R3.6.49，SDK camport4 4.2.10，走 legacy GigE2.0 协议路径。

**根因结论（官方规格背书）**：PS800-E1 官方标称深度帧率 **0.8 fps @ 全部深度分辨率**（1280×960 / 640×480 / 320×240 同值，en.percipio.xyz 产品页），是"精度换帧率"的产品设计（Z 精度 0.51mm@500mm）。实测 2.43 fps（411.5 ms/帧，设备端日志 `got one frame` 间隔 411ms 证实节拍在相机内部）已优于标称口径。彩色流单开 15.00 fps 是 RGB 传感器自身能力；深度一开，`TYFetchFrame` 按深度+彩色整组同步出帧，节拍被深度拖住。

**机理与仪器证据**（camport4 直读，一次性 C++ 探针）：深度组件 `image number`=18、`match window height`=5，恰好满足官方算力约束 `(image number+1)/2 × match window height < 48`（47.5<48，质量拉满档）；LeftIR/RightIR 曝光 990（只读探针，流式中写会被 `system busy -1016` 拒）；Laser power=50 auto=1。每帧深度需完整散斑图案序列采集+板上 SGBM，周期与分辨率无关（320×240 实测同为 2.42），瓶颈是图案数×单幅时间，不是像素量也不是 GigE 带宽（千兆全双工、MTU 1500、CPU 96% 空闲）。

**排除项（每项有实测）**：主机侧加工/点云生成/深度配准无关（零订阅者时同样 2.43；关点云关配准不变）；`frame_rate`/`frame_rate_control` 是上限请求不加速；`parameters.xml` 的 `DepthSgbmImageNumber=5` 对本机无效——0x1610 寄存器空闲态可写且读回一致（探针验证），驱动启动流程时序正确（设备默认 JSON 在 open 时载入、xml 下发在其后），xml=5 真实生效于流式期间帧率仍 2.43 → 按官方参数文档语义 image number 是"深度计算融合的 IR 图像数"（质量项），本机散斑采集序列固定，缩短不了采集周期；SGPM 相位数同理无效。

**遗留问题一（仓库漂移）**：`src/.../launch/parameters.xml`（HEAD=SGBM 2）与 `install/`（SGBM 5，09-15 构建）不一致，launch 实际读 install 拷贝——`parameters.xml` 的 `camera_parameter` 是文件内容注入参数，src 改动不重建则不生效。另该参数在本机属无效配置（写 5 写 18 帧率同），建议后续清成空值防误导（本轮未改 src，留决策）。

**遗留问题二（投递层，真实可修项）**：生产 2.43 fps 但 RELIABLE 大图投递坍塌——同函数先后发布的 `camera_info`（小消息）稳定 2.33 Hz，921KB 彩色 raw 大图 0.3–1.3 Hz 且有 3–7s 停顿，`depth_registered/points`（约 4MB）反而 2.2 Hz；感知节点恰以 RELIABLE 订阅 raw 图（代码注释自述与驱动对齐），属现行系统实际缺陷。`image_raw/compressed`（JPEG 小消息）实测 2.432 Hz、间隔抖动 5ms 完美贴合生产帧率。修复路径：感知改订 compressed 或修 DDS 大消息传输（FastDDS SHM+大缓冲快速 profile 一轮反更差，未深调，非本轮范围）。

**产品口径影响**：launch 默认 `frame_rate=5.0` 对本机深度不可达；停走式"每停取一帧"应按 ~2.4 fps（一帧 ~411ms 停留）排节拍。若未来需要连续高帧率深度，本机型选型不满足（同厂 F 系列标称 5fps@各分辨率，属换硬件决策）。

### 09-16 续（"本地计算提帧率"可行性核查，PS800-E1）

问题：采集 fps 的官方/SDK 口径到底是什么；把深度计算搬到本机能否提帧率。方法：官方页+SDK 头文件核对，台架 IR 流序列探针（raw 与 compressed 双路径，含 JPEG 解码帧差分）。

**采集 fps 口径闭环**：IR 传感器裸采集实测 **14.79 fps**（曝光 990 时），14.79÷18 ≈ 0.82 ≈ 官方深度标称 0.8fps——官方口径 = IR 采集率 ÷ image number（图案数），本机实测 2.43 优于标称。另实测曝光 100 时 IR 流速率 24.98 fps：**传感器硬件本身有 ≥25fps 余量，被固件深度管线锁在图案数除法里**。彩色单流 15.00 fps 不变。

**散斑图案判定（深度+IR 同开，IR 真实输出 mean=47 std=27，compressed 帧差分）**：相邻 IR 帧平均绝对差 **15–26**（满量程 6–10%），图案逐帧变化——轮播/随机散斑，非静态图案时域平均；AI 图判读确认散斑点阵完整、场景轮廓可辨、曝光正常。

**"本地计算"结论：当前固件/SDK 下不可行**，三道闸门：
1. **素材闸（致命）**：真实散斑 IR 图只在深度组件开启时输出、且跟随深度帧组 2.43 fps——本机匹配素材上限即 2.43 对/秒，无增益。IR 单流模式（深度关）输出 **冻结白板**（mean=254.6/std=10.0 逐像素相同；跨 990→100 十倍曝光变化输出完全不变，重启期 xml 写入亦无效）——不是实时传感器图像，25fps 只是搬运节奏。固件未开放"raw 图案流"模式。
2. **算法闸**：camport4 SDK 无本地立体匹配 API（`TYImageProc.h` 仅散斑滤波/修补/增强等后处理），本机计算需自建 OpenCV SGBM+极线校正+标定链路，数周级工程。
3. **质量闸**：单对图案立体匹配的深度噪声对标 18 图案融合（Z 0.51mm@500mm）显著劣化，套袋工艺精度是否兜得住需实测定标，质量责任自担。

**残余路径**：①向图漾发函确认 PS800-E1 固件是否有 raw 图案流/高速档（用户存储 JSON 由 PercipioDC 工具写入，驱动 open 时加载，是厂商工具链的合法入口）；②确需高帧率深度则换型（官方页同厂高帧档：FM854-E1 26fps@640×480、FM855-E1 19fps、FM815-IX-E1 5fps、GM 系列 ToF 30fps、TM 系列 25fps）。

环境复原：install 参数文件与备份逐字节一致；本轮未改 `src/`；探针与帧样本均在 `/tmp/percipio_fps_test/`（重启即失）。

### 09-16 再续（参数可调性判定 + 深度模式曝光实验）

用户问"当前参数能否调整"。实时读回 75 项与此前一致（image number=18、match window 5×5、IR 曝光 990、flash light intensity=19/enable=0、laser power=50）。补测最后一个未验证的帧率杠杆：**深度模式下 IR 曝光**（Left/Right `ExposureTime` 经 install xml 启动期下发，990→300→100 三档）——帧率恒定 2.4283/2.4209/2.4177（±0.5% 噪声级），**IR 曝光寄存器不参与深度采集周期**（对比 IR 单流模式下曝光确实改变传率 14.8↔25fps，深度管线不吃这个口）。至此 PS800-E1 全部 SDK 可及参数对深度帧率的效应实测完毕：**没有任何参数能提高深度采集帧率**。Viewer 调参走同一套寄存器，结论相同；Viewer 的剩余价值=Expert/Guru 级参数树浏览、GUI 质量对比、参数集基线管理。环境复原核验通过（install 逐字节一致、src 干净、无残留）。

### 09-16 三续（重大修正：独立 IR 原始散斑数据可获得，本地计算素材闸不存在）

用户追问"原始数据都拿不到？"——裸 SDK 复测**推翻上一条"素材闸（致命）"结论**。此前 IR 单流白板的根因不是固件锁死，而是**投射器（Laser 组件）的自动控制在无深度组件时不点亮**，IR 输出的是非光信号底噪（跨 3.3 倍曝光亮度恒 12.0→12.2，但帧率随曝光 20.5↔14.7fps 响应——传感器时序活着、无光学内容；此前 ROS 路径的 254.6 白板疑为投射器泛光态差异）。

**解锁序列（空闲态，SDK 直调）**：`TYSetBool(LASER, TY_BOOL_LASER_AUTO_CTRL, false)` + `TYSetInt(LASER, TY_INT_LASER_POWER, 100)` → 开 IR 流：
- 左 IR 单流：真实散斑图像 mean≈80/std≈73、13.2fps、帧间差 2.6（活画面）；AI 图判读确认散斑密度/对比度满足立体匹配（场景即测试台植物枝叶）。
- **双目 L+R 同帧组 5/5 硬件同步 ≈ 14.5 对/秒**——立体对素材硬性要求满足。L/R 亮度差约 4 倍（R mean21/std14，需单独拉 R 曝光平衡后匹配）。
- 曝光寄存器裸 SDK 读写正常（300↔990 可设，>990 被拒 -1013）；深度模式下曝光仍不影响采集节拍（前条结论不变）。

**持久化行为（重要）**：laser 设置跨连接自动复位（复测 auto=1/power=50）；曝光值跨连接**残留**（fresh open 读到前轮 xml 写的 100）——改设备设置后必须核验/复位。

**修正后的本地计算可行性**：素材闸不存在，剩余成本=①自建 OpenCV SGBM + 极线校正（SDK 无算法，标定可经 `TY_STRUCT_CAM_CALIB_DATA` 读）；②单图案精度验证（对套袋定位可能够用）；③质量-帧率档位在主机侧连续可调（1 对≈14.5fps 最糙，k 对时域融合换质量）——这正是固件不给的 image number 自由度。连续满功率激光的热管理未评估，长时间运行需观察。原始探针与帧样本在 `/tmp/percipio_fps_test/`。

### 09-16 四续（单图案 SGBM 深度快速验证：通过）

方法：同一静态场景两遍采集——①解锁激光后双目 IR 30 对（L/R 曝光均 990，标定自 `TY_STRUCT_CAM_CALIB_DATA`，**基线 62.2mm、f≈1098px，右相机外参即相对左 IR**）；②设备端 18 图案深度 30 帧（独立连接，laser 自动复位为 auto）。离线 `cv2.stereoRectify + StereoSGBM(MODE_HH, 256disp)`，中央 1/4 ROI 统计。极线验证：ORB 比值 0.7 筛选后 71 对特征 **|dy| 中位 0.00px**（裸 BFMatcher 在散斑图上全错配，中位 180px 的假象勿重蹈）。AI 图判读：深度图结构分层合理、空洞仅遮挡边缘、无噪声斑。

**结果（场景中位 ~800mm，额定范围边缘）**：

| 指标 | SGBM 单图案 | SGBM k=5 均值 | SGBM k=10 | 设备 18 图案 |
|------|------------|--------------|-----------|-------------|
| ROI 有效率 | **74.1%** | — | — | 63.9% |
| 时域噪声中位 | 1.49mm | 0.96mm | 0.65mm | 1.13mm |
| 噪声 p90 | 4.54mm | 3.35mm | 2.60mm | 5.05mm |

场景中位深度 803 vs 800mm（差 4mm，量级一致）。方法论交叉验证：官方 Z 精度 0.51mm@500mm 按平方律外推 @800mm ≈ 1.3mm，与实测设备噪声 1.13mm 吻合。等效速率：单对 ~14.5fps、k=5 ≈2.9fps、k=10 ≈1.45fps。

**判定：单图案立体精度对套袋用途足够**——1.49mm@800mm 按平方律折算 @500mm ≈ 0.6mm，且感知重建管线自带多点拟合进一步平滑；**~14.5fps = 设备深度 6 倍速率**，需要质量时 k 融合旋钮兜底（k=5 时 0.96mm 已优于设备单帧）。这是"现有硬件提速"路线的可行性证据。

**遗留工程项（立项前必做）**：①实时化——MODE_HH 离线约 2-3s/帧，需 MODE_SGBM_3WAY/半分辨率匹配压到 <100ms；②驱动集成——独立模式解锁序列（laser 手动+双 IR 流）要进 percipio_camera 包成一个采集模式；③真值标定（本次仅噪声+与设备一致性，无地面真值）；④田间动态场景与连续满功率激光热管理验证。采集/分析工具与 60 帧样本在 `/tmp/percipio_fps_test/`（重启即失）。

### 09-16 五续（实时彩色深度演示验证通过）

实时演示工具 `stereo_live`：解锁激光双目 + 半分辨率 MODE_SGBM_3WAY + JET 伪彩单窗口，色阶按场景 5%-95% 分位自动拉伸（保底 150mm 色带），`a` 键实时切 k=1/2/4/8 时域融合。实测：**显示 14.7-14.8 fps（相机双目对速率跑满）、处理仅 12-13ms/帧、有效率 64-65%、左边缘 ~15-20% 视差盲区（numDisparities 固有，正常）**。首版"一片深蓝看不到效果"的根因：**TY 标定结构体是 float32 内存，直接以 `cv::Mat(...,CV_64F, ptr)` 包装会位型错读 → stereoRectify 输出 NaN → 全图无效**；必须 `Mat(...,CV_32F,ptr).convertTo(K,CV_64F)`。此坑对未来把该链路集成进 percipio_camera 驱动同样适用。

**工具固化**：本轮全部评测工具（feature_dump/write_test/laser_check/raw_ir_test/stereo_grab/stereo_live/sgbm_eval.py + build.sh + README）已入 `src/percipio_camera/scripts/ps800_eval/`，独立 g++ 构建（不进 colcon），用法与安全注意见该目录 README。/tmp 下的采集数据（双目对/深度帧）为临时件，未入库。

### 09-16 六续（相机能力 × peach 感知/抓取需求 评估）

**感知侧硬门槛**（peach_harvester/config/*.yaml，键名即出处）：深度 uint16×0.25mm 或 32FC1(m)、RGB-D 须配准对齐、sync_slop 0.05s、min_points 100（位姿）/300（ICP）、min_mask_depth_ratio 0.35、TSDF 体素 3mm、refit RMSE 门 5mm、ICP 精配准门 7mm/RMSE 8mm、min_views 2（推荐 5、实测基线角 ~9.5°）、收齐窗口按帧数计（min_collect 10 + settle 5 + 3）、config 注释自述"现场感知约 2.5 FPS"。视点距离：拍照 ~0.66m、环绕半径 0.32–0.40m（view_planner）。

**逐项判定**：①精度——管线按 3–8mm 尺度设计，设备深度 1.13mm 与主机单图案 1.49mm（k=1）/0.96mm（k=5）@800mm 均有 2–5 倍裕度，**两条路线都过**；②节拍——所有窗口按帧数计，14.5fps 使收齐窗口 7.4s→1.2s、min_views 两机位采集时间减半，**收益真实但上限受感知自身算力钳制**（YOLO+MobileSAM 逐帧推理，BoundedWorker capacity=1 丢帧兜底，实际增益须实测推理耗时）；③RGB-D 配准——设备端现成，主机路线**缺主机侧配准**（SGBM 深度在左 IR 系，外参可读、可行待写）；④**彩色与 IR 双目同组采集未验证**（带宽 3.1MB/组×14.5≈45MB/s 千兆内可行；帧组行为与时间戳对齐是明天关键实验，sync_slop 50ms 要求同组 HW stamp）；⑤近距——**现行系统固有风险**：环绕 0.32m 低于相机额定 0.4m 下限（靠深度占比门滤），主机 SGBM 理论下限 f·B/numDisp≈0.27m 反而可能覆盖该档（光学近距质量未验）。

**结论**：精度维度两条路线都满足，节拍维度主机路线有 1.5–6 倍真实收益（下限取决于感知推理速率），代价是配准与同组采集两块工程缺口。不构成"必须迁移"的结论——若停走节拍按 2.43fps 排已够，维持现状；若要缩短每停时间/多机位环绕提速，主机路线值得做明天的两项关键实验后再立项。

### 09-17（peach_stereo 包落地 + 真机冒烟通过）

**关键前提实验**（rgbd_probe，裸 SDK）：彩色 640x480 yuyv + 双目 IR 同帧组 **13.6–13.7 组/秒、30/30 组全含三路、时间戳逐微秒相同**（TIME_SYNC=HOST 后即纪元微秒）。彩色不设档位时默认 2560x1920 yuyv（9.8MB/帧）会把组率拖到 2.5——档位适配是硬前提。

**新包 `src/peach_stereo/`**（ament_cmake，C++17）：`stereo_camera_node` 实现"解锁激光 → RGB+L+R 同组采集 → stereoRectify+SGBM(半分辨率) → `TYMapDepthImageToColorCoordinate` 配准 → 发布"全链路，话题与 percipio_camera 同构（`/camera/color/image_raw`、`/camera/depth/image_raw` uint16×0.25mm 已配准、`camera_info`、静态 TF 同名链、2Hz JET 调试流）；参数 nav2 式 yaml（`sgbm.*`/`avg_k`/`laser_power` 等）；camport4 SDK 跨包引用 percipio_camera 源码树（头文件未随包安装）。三个 SDK 级坑（均已在代码注释标记）：`TYUpdateInterfaceList()` 不调则 ETH 接口不枚举；`TYUpdateDeviceList(iface)` 不调则设备数为 0；标定 float32 必须 convertTo(CV_64F)。

**真机冒烟（隔离域 77）**：节点产流 **13.5 gps、发布率 100%**；compressed 通道投递干净（深度时间戳间隔 67ms 整 ≈14.9Hz 节奏）；**深度/彩色同组时间戳差 0.0ms**；深度内容合理（640×480 uint16、有效率 51%、中位 824mm 与场景一致）；AI 目验 RGB-D 配准"无可见偏移/错位"。raw RELIABLE 大图投递坍塌依旧（hz 3–5Hz）——投递层调整方案见下。消费侧新坑：jpeg 插件编不了 mono16（`/compressed` 发空载荷），深度要走 `compressedDepth`（PNG 前有 12 字节容器头：格式码+两个量化 float，消费端要跳过）或 `zstd`。环境复原：冒烟节点已停、laser auto=1/power=50 复验。

### 09-17 续（感知 A/B：peach_stereo vs percipio 设备深度，同场景台架）

**推理基准（venv torch 2.13+cu130，GPU 在位）**：YOLO(best.pt) 稳态 **4ms/帧**、MobileSAM 框提示 **21ms/框**（全帧自动分割 1427ms 非感知路径）——**感知算力非瓶颈，此前"YOLO+SAM 可能钳制帧率收益"的保守判断作废，相机 13.5fps 可被全额利用**。

**栈级 A/B**（域 77，同场景同光照同静态 TF base_link→camera_link；scene_perception 全参数默认）：注意无 TF 时感知每帧双次 0.5s 超时查询会把它拖到 0.87fps——台架测感知必须先补 TF。

| 指标 | peach_stereo（主机立体） | percipio（设备 18 图案深度） |
|------|--------------------------|------------------------------|
| 相机帧率 | 13.5 gps | 2.43 fps（日志核实） |
| 感知处理节奏（masks 头时间戳间隔中位） | **133ms ≈ 7.5Hz** | 600ms ≈ 1.7Hz |
| 感知 Active → 目标集锁定 | **≈2.8 s** | **≈48 s**（含一迟确认的第 3 目标反复重置 settle，放大了低帧率下的墙钟惩罚；纯帧数口径 18 帧也差 3 倍） |
| 锁定目标数 | 2（conf 0.99） | 3（conf 1.0；多出的第 3 目标疑低帧率下窗口拉长引入） |
| 同一目标 camera_distance | **0.611 m** | **0.616 m**（两条独立深度链差 **5mm**，单图案精度的交叉验证） |

**发现的 bug（未改，报给 harvester 维护轮）**：`scene_perception.launch.py` autostart:=true 路径节点收到**双重 activate**（已 active 再收 transition 3）未捕获异常直接进程退出；harvest_system 主路径 autostart:=false + 手动/manager 驱动不受影响。台架绕法：autostart:=false + 单次手动 `ros2 lifecycle set configure`（launch 的 activate 处理器是无条件的，configure→inactive 会自动激活）。

**参数调整（已落地）**：①`scene_perception.yaml` `tentative_ttl_frames` 8→**20**——按帧计的 TTL 随前端帧率缩短墙钟（2.43fps×8≈3.3s vs 7.5fps×8≈1.1s，遮挡/闪检目标会被过快弃置），上调维持 ~2.7s 语义，percipio 前端下偏保守无害；②`target_reconstruction.yaml` `max_views` 注释更新（帧密度 3 倍、view_filter 去重兜底、值不动）。维持不动的依据：收齐窗口参数已按实测帧率自适应（好设计，A/B 中 2.8s 锁定即其兑现）；recommended_views 留待真机轮；感知无需步进参数（GPU 推理 4–50ms/帧量级）。

### 09-17 再续（bug 修复 + 调整方案落地 + 规格档案）

**autostart 双 activate 崩溃修复（双层）**：launch 侧 `OnStateTransition` 加 `start_state='configuring'` 过滤（只匹配 configuring→inactive 一次）；节点侧 main() 对 spin 的生命周期重复转换异常捕获后继续 spin（状态机已在目标态，重复请求无副作用）。验证：autostart:=true 连跑两轮 died=0、均达 Active。

**调整方案落地**：①视点距离——`peach_arm.yaml scan.observation_radius_m` 0.40→**0.45**、`minimum_radius_m` 0.32→**0.42**（PS800-E1 额定下限 0.4m；stereo 前端近档待近距标定后再评估放开），view_planner.py 纯核默认值同步；②percipio launch `frame_rate` 默认 5.0→**2.5**（诚实值：5.0 不可达且无加速作用）；③harvest_system 新增 `camera_frontend:=percipio|stereo`（默认 percipio）与 `camera_ip` 参数——stereo 时向只读的 aubo bringup 传 camera_enabled:=false 压掉 percipio、由 harvest_system 直起 peach_stereo（launch 语法与 PythonExpression 引号坑：LaunchConfiguration 求值是裸字符串，比较须加引号）。

**规格档案**：PS800-E1 全量实测规格表（型号/SN/固件/深度原理与 18 图案约束/各流帧率/基线 62.2mm/量程精度/激光行为/时间戳/深度口径/已知限制/无效参数清单）入 `src/peach_stereo/README.md`；architecture.md（停走式相机模型条目）与 io.md（相机前端二选一）同轮更新。

**遗留（下一轮）**：感知端 compressed 订阅改造（投递层 #3——A/B 中感知自身 7.5Hz 未受阻，优先级降为"多订阅方（RViz/录包）改善"）；stereo 前端真机轮（harvest_system 切换后端到端采摘节拍对比 + 激光温升）。

### 09-17 三续（标定唯一性整理 + E2E 首轮排障记录）

**E2E 首轮（mock 臂 + 真相机 percipio 前端）**：RunHarvest 两轮均 `survey_failed, discovered:0` 且 **12ms 即 ABORT**（瞬时失败，非窗口超时）——待查 supervisor 的 survey 前置条件（感知注册表明明在册 2 目标持续命中；另见重建 worker"队列已满"自栈起 6s 持续）。**点云位置不对的根因**：手眼标定 active.yaml 只存于 `_archive/runs/hand_eye/`（被归档），extrinsics_publisher 回退名义值 [0,0,0.02]+identity，与现场标定差 ~11cm+旋转；期间一次误恢复（放 `runs/hand_eye/`，实际查找路径是 `src/aubo_hand_eye_calibration/hand_eye/`，见 storage.py 优先级）已纠正。

**标定唯一性整理（落地）**：
1. **根因**：`src/aubo_hand_eye_calibration/hand_eye/` 曾被 .gitignore 整目录忽略 → 标定仅存本机 → 工作区清理后只剩归档副本。现改为只忽略 `candidates/`（会话产物），**active.yaml 入库随仓**（clone 即得）。
2. **在用标定唯一事实源**：手眼外参=`src/aubo_hand_eye_calibration/hand_eye/active.yaml`（改值或覆盖 yaml 后重启 extrinsics_publisher 生效；目录 README 载明流程与边界，`_archive` 副本为历史不读取，`AUBO_HAND_EYE_DIR` 仅限特殊部署）；彩色内参=`src/percipio_camera/config/color_camera_info.yaml`（**percipio 与 peach_stereo 两前端共用**——peach_stereo 新增 `color_camera_info_file` 参数，launch 默认注入同一文件，语义与 percipio 一致：保留流分辨率、标定字段整组取自文件）；IR/深度内外参=设备内直读。architecture.md 增 7a 条目。
3. **验证**：外参——发布器日志 `Published active camera extrinsic` + 实时 TF `wrist3→camera_link=[0.045,0.108,0.002]+标定四元数` 逐位吻合；内参——peach_stereo 发布的 K 与 yaml `camera_matrix` 逐位一致（466.17/465.56/326.07/244.79）。

### 09-17 四续（E2E 前端 A/B 完整轮 + 两缺陷一修复）

**跑法**：域 77，`hardware_mode:=mock` + 真相机，`scene_key=e2e_ab` 同场景；两前端各自起整栈后 `SetEnables(execution=true)` + `set_execution_armed`，`RunHarvest intent=PICK_ALL`。账本：`runs/e2e_percipio_121647|121954/`、`runs/e2e_stereo_123809|123937/`（123809 为参数未设时的门拒绝轮）。

**发现 1（缺陷，绕行未根治）：mock 模式 survey 必被安全门拒绝。** `SurveyScene → goToPhotoPose → safety_gate_` 无条件要求 robot_status，而 `aubo_io_controller`（唯一发布者）按 bringup 设计仅 real 模式 spawn——mock 下 `/aubo_io_controller/robot_status` 零发布者，门报 `拍照位姿安全门未通过: robot_status_missing`，12-19ms 即 ABORT。这也解释了上午"survey_failed 12ms 即 ABORT"的另一半（当时域内同时有 real 栈在发 robot_status，掩盖了该缺陷）。本轮绕行：两前端统一在线 `ros2 param set /peach_arm execution.require_robot_status false`（对等）。**遗留**：mock 栈需要等价 robot_status 数据源（mock 假状态发布器或 bringup mock 分支），否则 e2e/联调每次都要手动关参数。

**发现 2（缺陷，已修）：`camera_frontend:=stereo` 从未真正通过 harvest_system 起过 stereo 相机。** jazzy launch 语义：`IncludeLaunchDescription` 把 `launch_arguments` 落成全局 `SetLaunchConfiguration` 且**不回滚**（源码 `include_launch_description.py` visit() 返回 `[SetLaunchConfiguration…] + [描述]`）。aubo include 传入的 `camera_enabled`（stereo 时压成 `'false'`）覆盖 CLI 原值，排在其后的 stereo include 条件 `camera_enabled=='true'` 恒假 → `stereo_camera_node` 不启动（进程号连续无缺口佐证）。最小复现（同条件 LogInfo / IncludeLaunchDescription 均正常触发）+ 真实 launch 进程 cmdline 参数无误，三重定位。**修复**：stereo include 移到 aubo include **之前**（彼时读到的仍是 CLI 原值），注释载明该 launch 语义坑；重建后文档单命令验证——`stereo_camera_node-1` 随整栈首启、13.4gps 满速产流、五节点 Active、感知正常消费。

**发现 3（集成缺口，未改，报 harvester/重建维护轮）：两前端在观察→重建收口门同一处失败，但失败机理互补。** 账本均记 `observe_build_view_race: views=1 < min_views=2`（build_view_count=1）：
- percipio（2.0fps，race 窗自适应 15.1s）：观察段臂动，深度帧位姿滞后 → `target_drift` 63→200mm > 40mm 门限连续拒采；同帧重复 `same_stamp`；另有一次 MoveGroup plan aborted。
- stereo（13.4gps，race 窗 4.4s）：①`missing_mask` 占主导——重建自动采帧要求与深度帧**同时间戳的掩膜**，深度 13.4fps ≫ 感知掩膜节奏，精确 stamp 配对近乎必失配；②相机静止时 `near_duplicate`（平移 0.0mm/旋转 0.00°）拒收 → VIEW_FAST 单视策略下静态相机只能积 1 视，min_views=2 必须依赖补视移动落入 race 窗内（4.4s 内规划+执行难达成）；③13.4fps 下重建 worker 队列持续打满拒帧。
- 即：percipio 败于"帧太慢+漂移门"，stereo 败于"掩膜-深度 stamp 配对 + 单视去重与 min_views=2 互斥"。**上午台架 A/B 的 2.8s vs 48s 是发现锁定（registry lock）口径，不覆盖本收口段。** 遗留：掩膜按最近邻 stamp 容差配对或感知掩膜流提频；VIEW_FAST 的 build 收口窗口/补视触发与 min_views 联动需重审。

**附带观察**：stereo 深度质量显著优（采帧有效深度占比 0.95 vs percipio 0.64-0.84）；raw RELIABLE 大图投递坍塌在 stereo 前端依旧（hz 工具测深度 0.07-1.49s 抖动，投递层遗留不变）。恢复项：两栈已拆、进程清零（bringup 预检两次拦下我方漏杀进程，工作正常）；`execution.require_robot_status` 为节点运行时参数不跨栈，无需恢复。

**同日裁定（用户）**：仿真/mock 测试对 robot_status 门**可以忽略**——不建 mock 假状态发布器、不改默认值；mock 联调以在线 `ros2 param set /peach_arm execution.require_robot_status false` 为既定做法（真机默认 true 的门不受影响）。发现 1 的"遗留"就此关闭。

### 09-17 五续（e2e25 分阶段战役：RViz 颜色闭环 + 双前端阶段矩阵 + observe 链 mock 伪影定性）

**基线提交**：`2b3f74d`（stereo include 顺序 + executor TF/logger 修复入库）+ `3af90db`（颜色一键 + 测试门三件解锁），三包 colcon test 全绿（198 tests / 0 failures）。

**1. RViz 机械臂颜色回归闭环（用户报"没有之前的颜色"）。** 根因三层：真彩色来自独立 `RobotModel` 显示读 `/robot_description` 渲染 .dae 内嵌材质（灰身+橙臂；URDF 链接是匿名 material 灰），302b1b0 误判——rviz `Display::load` 先读 `Value` 再被 `Enabled` 键覆盖，只改 `Value` 无效；38dc129 起 topic 被清空后该显示从未恢复。工作区已备好 3/4（topic `/robot_description` + Transient Local + 场景机器人 `Robot Alpha: 0` 隐身防灰叠影），`moveit.rviz` RobotModel `Enabled: false→true` 一键闭环。验证：`xwd -id <窗口ID>` 直抓 RViz（窗口在别的 workspace 时 root 截图抓不到），3D 视图区橙色像素呈同列两簇（竖直臂链上/下关节装饰环特征，352px）且无灰色叠影。

**2. 测试门解锁三件（均为 AGENTS「缺头补头/修 setup」路线，非关 lint）。** ① peach_harvester `setup.py` `tests_require`→`extras_require={'test':['pytest']}`：colcon 的 pytest 探测只认后者，legacy 写法静默退回 unittest 且 0 测试判失败（与 serial_imu 同款坑，peach_bringup 已是正确写法）；18 用例恢复执行。② `moveit.launch.py` 补 BSD-3-Clause 版权头（ament_copyright 模板原文）。③ aubo_e5_moveit_config `CMakeLists` 删除无效的 `set(ament_cmake_copyright_FOUND TRUE)` 跳过——Jazzy `ament_lint_auto_find_test_dependencies` **无条件** `find_package`，预置 `_FOUND TRUE` 从未生效（该包 copyright 门其实一直在跑且一直失败）。

**3. 双前端阶段矩阵（域 77，mock 臂 + 真相机 PS800-E1，`scene_key=e2e25/e2e25s`）**：

| 阶段 | percipio | peach_stereo |
|------|----------|--------------|
| T0 栈冒烟（四节点 Active/joint_states/TF 链） | ✓ | ✓ |
| T0.5 RViz 颜色 | ✓（本轮修复后截图验证） | —（同一修复） |
| T1 相机链（编码/内参/fps） | bgr8+16UC1、k[0]=466.174635 与共用 yaml 逐位一致、驱动 2.0fps | bgr8+16UC1（配准到 `camera_color_optical_frame`）、内参同源逐位一致、驱动 13.4gps |
| T2 begin_scene→锁定 | 11.0s | 12s（收齐窗自适应，不随帧率线性） |
| T4 grasp_decision 默认门 | `allowed=false, reason=reconstruction_not_ready` ✓ | 同 |
| T5 check_reachability | 服务通（camera 系 (0,0,0.62)→`no_ik`，包络判定合理） | 同 |
| T9a SURVEY_ONLY | 18.7s SUCCEEDED | —（未重跑） |
| T9 PICK 批 | 五轮剥离见下 | 有界批（per_target_timeout 45s）发出后随栈被外部终止（`claimed=[]` 无 outcomes） |
| fire_step | PHOTO ✓；VIEWPOINT/BUILD 未接线（显式拒绝）；批次取消后（INTERRUPTED）PHOTO 亦失败（MoveTo 失败 + 臂门 `selected_target_stale`） | — |

感知命中流：两前端 `events.jsonl` 均以驱动节奏流式落 `frame_observations`；masks 目录批外为空（掩膜在批内观察段才落盘，已知行为）。

**4. observe→build 链五轮剥离与 mock 伪影定性（取代四续发现 3 的机理猜测）。** percipio 批内逐层：
- 第 1 轮（原参数）：`observe_build_view_race` 15.75s，views=1。
- 小修① `observe_build_grace_s 3.0→12.0`（yaml 入库）：第 2 轮 23.5s 仍 race——`target_drift` 门 0.04 连拒（漂移 40.9→222mm）。**在线 `ros2 param set` 对重建门配置不生效**（configure 期冻结 dataclass，实证：drift 改 0.25 后仍按 40mm 拒），yaml 注释已载明该边界。
- 小修② `capture.max_target_drift_m 0.04→0.25`（yaml 入库，重启生效）：第 3 轮换 `missing_mask`——视点移动后感知把同一果实**重注册成新 ID**（注册表 1→2 个目标、原 ID 命中冻结），重建绑定的旧 ID 永远等不到同 stamp 掩膜。
- 小修③ supervisor `reconstruction_min_views 2→1`（仅在线，未入 yaml）：第 4 轮过 race 但 `build_timeout:reconstruction` 180s（第二视点会话吊着 COLLECTING 不 finalize）。
- 第 5 轮 VIEW_CONSERVATIVE：`observe_failed: plan_id mismatch: preview != execute`（0.019s 即拒）——技能侧检查点/令牌双路的**新缺陷信号**，报 harvester 维护轮。
- **定性（用户裁定）**：相机物理上固定台架，但 TF 链按眼在手上（外参挂腕）——mock 臂一动 TF 假装相机跟着动，base 系目标位置随之错位（实测偏移量与视点位移吻合，~220mm）→ 漂移门拒收、身份断裂、锚点失效，**全部是 mock 伪影**；真机眼在手上时相机真实随动、base 系位置自洽，该链不存在此问题。**多视点观察/重建及依赖模型的下游（接近/PREGRASP/FULL）在 mock 下原理性不可验证**——e2e 下游阶段留待真机。

**附带发现（本轮新增，报维护）**：① 批次跳过后、空转选果期间 supervisor 状态发布 ~115Hz（state_seq 单批累计 23.5 万+；on-change 发布器空转刷状态，goal feedback 单批 1.4 万条）——DDS 无谓洪泛。② `scan.protected_zones 盒#0 存在 min>=max 的轴（退化盒）已丢弃` 警告在成功轮次同样出现（保护区配置 wart，非阻塞）。③ `ros2 topic hz` 探针一接入 RELIABLE 大图即把投递压塌（color 0.18Hz、depth 0.07-1.49s 抖动，与四续"raw RELIABLE 大图投递坍塌"同象）——帧率以驱动日志为准，勿用 hz 探针测大图链。

**未跑项**：T6 接近/PREGRASP_HOLD、T7 使能门负测试、T8 FULL 单目标（均被"模型依赖 + mock 伪影"挡）；stereo T9 整批（栈被外部终止）。**同日裁定（用户）：mock/仿真 e2e 作废——相机固定台架而 TF 按眼在手上算的伪影使多视点链原理性不可验证，转真机测试；所有等待/超时硬上限 20s。** 恢复项：栈已停、进程清零；两处 yaml 小修（drift 0.25 / grace 12.0）**随真机轮即刻回调**（0.04 / 3.0），真机证据另立；min_views 仅运行时改过未入 yaml（默认仍 2）。

### 09-17 六续（真机轮：新拍照位 + 全链 pick1 过门 + 四缺陷定性 + octomap 工具豁免修复）

**基线**：e1ed331（SRDF `global_photo_pose` 真机示教更新，wrist2 -0.500→-0.280，示教器到位后 /joint_states 直读；`harvest_stow` 保持旧拍照位，注释标明已分叉）；`.setup_assistant` 升级 Jazzy MSA schema（小写键 + package_settings，修 `invalid node; first invalid key: package_settings` 加载失败；`MoveItConfigsBuilder` 大小写双兼容已核源码）。**MSA 重导出事故与规矩**：对该包全量重导出会把它管辖文件重置回模板（joint_limits 加速度/cartesian_limits 段被删→pilz 加载即崩、pilz 限速 0.25→1.0、sensors_3d octomap 被清空、SRDF 碰撞矩阵砍 7 对工具内互碰、FakeSystem URDF 包装劫持 builder 解析优先级）——本次破坏面已全部还原；**规矩：禁止对 `aubo_e5_moveit_config` 直接 MSA 重导出，确需用 MSA 导出到 /tmp 一次性目录人工 cherry-pick**（MSA 是生成器不是编辑器，本包初始导入后从未重导出过，首轮即全灭属机制必然）。

**真机阶段证据（域 77，percipio 前端）**：T0 四节点 Active + `robot_status drives_powered=1 motion_possible=1`（真机门原生通过，无需 mock 绕行参数）；fire_step PHOTO 过命名状态（新拍照位 ✓）；感知锁定 9.4s；内参 k[0]=466.174635 与共用 yaml 逐位一致；CheckReachability 服务正常（camera 系 (0,0,0.62)→`no_ik` 合理判定）；grasp_decision 无模型默认 `allowed=false`。**pick1 批（显式 target_ids=['target_1']，grasp=false）8.56s 全链过门**：观察→Pilz LIN 视点移动（真机执行 7.5s）→重建 3 帧/2 机位/重叠 p95=5.8mm/**refit ACCEPT**→接触几何解算（entry/axis/travel 0.089m）→**READY_FOR_GRASP**（grasp 门正确拒绝接触）。

**缺陷链定性（按因果序）**：
1. **感知（P1）**：贴图像左下边缘滑动的噪声检测块（bbox_x=0、质心跨帧漂 80px、出现率 18%）可被确认进锁定集，且选果「priority 主序、同级面积降序」让它优先于 81% 出现率的稳定真果被绑定 → 绑定后 missing_mask 断流 → 采帧失败。本轮以显式 `target_ids` 选稳定真果绕行（每个世代 ID 会变，需先采样确认）。修法：确认/选果加贴边门与稳定性序。
2. **octomap 工具自碰死锁（已修）**：眼在手上时工具永远在相机正下方，点云 updater 的 self-filter 漏收工具点云 → `<octomap>×tool_body_link` 接触 → 臂停在任意视点位后**所有**规划（Pilz PTP/OMPL）在 CheckStartStateCollision 死。修复：`acm_policy` F10 回退（`allowToolVersusWholeOctomap()=true`，防撞主力臂连杆+camera_body 保持受查）+ `GraspTask::applyWholeOctomapToolExemption` static 入口在 `on_activate` 后台线程一次应用（周期级生效，Survey/观察/接近全覆盖；服务等待不占激活回调）。重启后从驻留视点规划/执行恢复 ✓。self-filter 修复后可再收紧。
3. **收口体系设计矛盾（P1）**：view_policy FAST「好单视即收」在掩膜流畅时**不再移动** → 机位永远 1 个；supervisor `reconstruction_min_views=2` 与重建 finalize 基线门（`minimum_baseline_deg 8.0`，同位帧近重复正确去重、单站基线恒 0）形成三层耦合，单站永不收口。讽刺闭环：掩膜缺失时反而因「获取性移动」凑出双机位（pick1 即此路径）。运行时 `min_views=1` 解不开（基线门仍拦）。修法：三处门槛语义统一（好单视时 finalize 放行单站，或策略始终补一移）。
4. **视点候选 LIN 穿奇异（P2）**：候选 2/3 的笛卡尔 LIN 段要求 foreArm 4.12/13.98 rad/s（限 2.5964，transit 缩放 0.1 已生效仍超——近奇异/解支翻转），Pilz 整条拒（观察只 LIN、禁止 PTP 绕行是 stages.cpp:533 刻意设计）。修法：候选生成时雅可比预估关节速度可行性，剔除必死候选。
5. **桌面模型水平保守度（P2）**：`table_link×upperArm_Link` 在视点位形接触（物理未撞，z 高度 2026 已修不再动；近臂 25cm 内网格顶点全在 z∈[-0.10,0] 即桌面平板本身，覆盖 1.5m×0.83m）——待台面实测尺寸校核水平范围。

**本轮运行时参数**：`reconstruction_min_views 2→1`（在线，未入 yaml，随栈消亡）。**未跑**：stereo 前端真机对比、T7 使能门负测试、FULL 单目标（均被缺陷 3 挡在接近段之前）。账本：`runs/e2e25r_{percipio_163254,percipio_fast2_163445,pick1_170449,pregrasp2_170647,pregrasp3_171710,pregrasp4_171833,pregrasp5_171948}`。

### 09-18 真机预抓取轮（nanoseconds 崩溃修复 + octomap/桌面碰撞定性 + 点云统计 + 回放架适配）

**批次**：`field_pregrasp_20260918_{1029,1039,1055}`（域 77，adaptive_cylinder_v1，全程 tool=false 无 SetIO，web 8090 监控在线；详细见 `runs/field_test_20260918/log.md`）。

- **1029 崩溃（已修）**：`supervisor/executor_node.py:1317` `get_clock().now().nanoseconds()` 把 rclpy **属性**当方法调用 → run_harvest 回调 `TypeError: 'int' object is not callable` → ABORTED 且 `termination_reason=''`、summary 全 0。修复：`.nanoseconds`（随本轮工作区）。
- **1039（octomap 在）**：接近 12 次重试全 0 解，FCL 实证 `<octomap>×wrist2_Link`。点云统计：袋 XY 15cm 走廊 z0.70–1.01 有 ~1.8k **真实**点（相机距≈0.55m，非 1.01/2.02 鬼影带）=袋上方枝叶；单位自检 ✓（果 0.53m 落 0.50–0.75 档，无 4× 单位错）；鬼影带占比 2.5%/≈0%；远端离群超窗（p99 z=3.92m，octomap max_range=2.0 已裁）。
- **1055（octomap 关）**：用户指令「碰撞先不开」→ `sensors_3d.yaml` `sensors: []`（**临时态须恢复**）。审查门全过（候选 3/5/5/5 均解出 163–168 点轨迹、短路径/胶囊/笛卡尔/姿态门全绿）后 **Pilz 轴向 LIN goal IK NO_IK_SOLUTION**；伴随 `table_link×upperArm`、`foreArm×wrist2` 仅出现在边界构型采样。与 09-17 缺陷 5（桌面模型水平保守度 1.5×0.83m 待实测）叠加解释：正常构型不碰、边界构型蹭（可能含桌面模型偏大幻影）。
- **袋位出包络**：今日袋底 r=0.732/z=0.641（肩到预抓取 0.846m）vs 成功参考 1757 r=0.707/z=0.561（0.786m）+ adaptive TCP +17.6mm ≈ 多要 8cm 伸展。**门缝**：CheckReachability 单点停位 IK 放行 ≠ Pilz LIN goal IK 可行。
- **果胶囊门复核（用户质询）**：果侧 ✓ 已用拟合直径+0.01 膨胀（回退 0.12 仅无效时+告警）；工具侧两处债：实心圆柱模型（D_inner=0.116 空心筒）+ 常量硬编码未走档案单一事实源（当前两档 D_outer/L 相同故数值未脱钩）。1639_1（−24.6mm，轴距≈7.5cm）空心环模型仍拒=真侧贴；12rad/0.25m 门为 09-11 收紧设计值。
- **回放架回归（已修）**：09-18 身份元组门把 `sim_field_targets.py` 打成 0/14 全拒——goal 未填 model_revision/tool_profile_id/calibration_revision/config_revision 四字段；decision 缺 `valid_until` → model_not_executable。已补（goal+decision 同套身份串、valid_until=now+30s），**adaptive 档 7/14**：3 行程门、1 弦偏离、1 筒体压胶囊、1 规划 0 解、1 有效期竞态。
- **新缺陷**：①CANCEL_NOW 终局空（termination_reason=''/summary 全 0，复现 2 次，P1）；②批次收尾停滞 5min 无臂活动无日志、per_target_timeout 未兜住（P2）；③web「柜侧硬件」光照字段 `[object Object]`（P2 展示）。
- **用户裁定**：暂时不减速——运行期 `ros2 param set /peach_arm moveit.approach_near_velocity_scaling 1.0`（仓库默认 0.05 不动）。

### 09-18 续二（三门标定：1/30→30/30 复盘与回填）

**用户两次质疑推动定位**（「不可能碰撞」「100 例 80% 都能去」均成立）。回归链全貌：

- **回归实锤**：hollow 同 seed 20260911 30 例 **1/30**（历史 26/30；纯几何回放塔 25 测绿=核心数学未回归）。拍照位二分排除（旧位 e1ed331^ 也 1/30）；adaptive/hollow 外径同（D_outer 0.120）排除；09-18 未提交改动全是门/心跳、不改轨迹生成，排除。
- **根因=笛卡尔三门阈值失配**：基线提交 cb9e5a1（09-10，66/100 时代）该三门全为 **0=不查**；526eb11（09-11）收紧为 1.8/0.25/0.08 后 100 例 66→**54**（在案）；09-14 胶囊化+09-16 重写后路径族漂移，现行阈值恰压在合法簇边缘——**无门 30 例实测：合法簇 24/30（比≤1.90、偏≤0.258、退≤0.075，其中 rand_28 偏 0.258>0.25、rand_06 退 0.075≈0.08 正被旧值压住），游荡簇 6/30（比≥4.37、偏≥0.42、退≥0.17）**，两簇间隔巨大。
- **随机性注记**：staging 候选扫描含 4 随机种子，同案逐次路径不同——开门跑里连「本有干净候选」的案子也常死（换候选后超阈值）；历史 26/30 本身带抽签方差。
- **隔离实验**：三门运行期置 0 → **30/30** 全过且全部果实胶囊外（回放架 model_revision 改每案唯一串修掉有效期假失败）。
- **标定与回填**：新值 **绕行比 2.6 / 弦偏离 0.32 / 回退 0.12**（合法簇 max×余量，仍拦游荡簇）。运行期验证 28/30；回填 `arm_parameters.yaml` 默认+`config/peach_arm.yaml` 部署两处并重建后，新拍照位+hollow 终验 **30/30**（旧值同配置 1/30）。
- **残余观察**：终验「未从拍照位返程旗标」9/30（无门跑 3/30）——返程记账口径待查，非失败；游荡候选（袋上方悬停被实心胶囊模型拦 −62mm 一类）与规划随机方差仍是 0-2 例/轮的残差来源。
- **流程修正（写进本节供后续轮次遵守）**：回归排查第一动作=**对参**（当前阈值 vs 最后全绿基线提交同名键）+ **门关/开 A/B 对照**；报错信息已含门名/实测值/阈值三要素，缺的是聚合视图——mock 回放汇总应输出逐门拒发直方图。

本轮文件：`runs/sim_field_targets_20260918_{113507,114404,115140,121238,123455}.jsonl`（1/30→1/30→30/30→28/30→30/30 五段证据链）。

### 09-18 续三（harvest_system 真机 + camera_frontend:=stereo 冒烟）

整栈 `hardware_mode:=real camera_enabled:=true camera_frontend:=stereo`（autostart 关）。`peach_stereo` 打开 PS800-E1 `169.254.10.110` status=0，组率 **13.5 gps**；无 `percipio_camera` 节点。lifecycle 四节点 Active；透传/JSB/IO 控制器 active；柜 `AuboE5Hardware` on_activate OK。`execution.enabled` / `execution_enabled` / `grasp.enabled` / `tool.enabled` 均 false，未发 `RunHarvest`、未 SetIO。MoveIt RViz 已起。

**刷屏根因与修复**：Jazzy `image_transport` 加载全部插件且无 `enable_pub_plugins`；observability catch-all 订 `/compressed` 与 `/compressedDepth` 后，jpeg 编 16UC1、compressedDepth 编 bgr8，每帧 ERROR。已改为只发 raw `sensor_msgs/Image`（感知本来订 raw）。复验无 `cv_bridge` / `CompressedPublisher` / `compressed_depth_image_transport` 报错。剩余 MoveIt 噪声：`occupancy_map_monitor` 无 3D 插件、RViz `/recognize_objects` 不可用（非本前端）。

### 09-18 续四（peach_stereo 话题与 percipio_camera 同构）

对照 harvest 用 `percipio_camera.launch.py` 默认面（`color_point_cloud_enable:=true`、`left_ir_enable:=false`、`point_cloud_enable:=false`）。`peach_stereo` 去掉自研 `depth/debug_color`，补发 `depth/camera_info` 与 `depth_registered/points`（配准彩色点云，frame=`camera_depth_optical_frame`，字段 xyz+rgb，与驱动 `publishColorPointCloud` 同构）。真机 `camera_frontend:=stereo` 复验图上只有：

`/camera/color/image_raw`（bgr8）、`/camera/depth/image_raw`（16UC1）、`/camera/{color,depth}/camera_info`、`/camera/depth_registered/points`；无 `debug_color`。未恢复 image_transport 插件后缀（会再次交叉编码刷 ERROR）。未发 `RunHarvest`、使能仍关。

### 09-18 续五（stereo 深度/点云抽帧）

整栈再起 `camera_frontend:=stereo`（使能关）。10 帧同戳：`depth/image_raw` 16UC1 640×480，有效约 **49%**（零值左侧配准空洞，无 65535）；`depth_registered/points` 点数 = 有效像素（约 15.1 万），xyz+rgb，frame 均为 `camera_depth_optical_frame`。Z 中位 **0.73 m**，额定带 0.40–0.80 m 约占有效点 60%。配准后彩色图左约 **26%** 无深度（左 IR→彩色 `TYMap` 基线空洞）；近袋在空洞外、中袋在有效区内。远点噪声：约 1.2% Z>2 m、个别到 ~12 m（SGBM 小视差）。生产组率 ~13.5 gps；`ros2 topic hz` RELIABLE 大消息约 3–5 Hz（已知 FastDDS 坍塌）。感知在册 1 个目标。

### 09-18 续六（真机 Percipio vs stereo 感知 10s 对比视频）

同场景静态腕相机、`hardware_mode:=real`、autostart 关、`execution.enabled=false`，未发 `RunHarvest` / Survey / SetIO。两前端互斥串行：先 `camera_frontend:=percipio` 再 `stereo`。各录 `/peach/perception/debug_image` 与 RViz 3D 视口（Perception Markers 圆柱）10 s 墙钟，拼 2×2。

| 前端 | debug 帧/10s | 锁定 | 目标 | 相机距 | 观测 conf | debug 叠加 conf |
|------|-------------|------|------|--------|-----------|-----------------|
| percipio（设备 18 图案） | 21 | 是 | target_1 | 0.553 m | 0.54 | 0.84 |
| peach_stereo（主机 SGBM） | 37 | 是 | target_0 | 0.565 m | 0.986 | 0.81 |

两前端都只锁中间那颗袋；左侧悬挂袋未进确认集。距离交叉验证差 **12 mm**。感知 worker `capacity=1 drop_oldest`，stereo 相机 13.5 gps 并未变成 13.5 Hz 感知——墙钟帧数 37 vs 21（约 1.8×）。产物：`runs/camera_ab_20260918/comparison_2x2.mp4`（及 debug/rviz 左右拼接）。
## 2026-09-20（下午）外围五包质量轮 W9–W16（ce7f68b→本轮）

**范围**：W0–W8 主链深审后，本轮覆盖外围五包（observability/vegetation/bringup/common/system_tests）+ Round1 遗留清扫 + UNWIND 收敛（用户裁定：bond/composition/diagnostics 全做；PF-1 推迟相机轮）。全程 mock/单测门，不动真机、不动 peach_stereo 用户工作区。

**W9 安全网**：test_perf_baseline 补 CMake 注册（25→29 测）；replay_oracle `_mat_to_quat` m22 分支 w 分量笔误修复（移植时误抄，插桩证实三层语料 1640 调用零触发、基线数值逐数复核不变，json 附 reverified 记录）；observability 死参数 record.bag_topics/debug_token 删除；bringup 参数校验统一 peach_common.param_rules；旧 shim 补 snapshot 再导出。

**W10 observability 热路径**：build_harvest_job 按快照代数记忆化（原每次 snapshot 全量折叠、Web/轨迹以 ~10Hz 重复算）；bag 写队列有界化 record.queue_depth=512（drop-oldest+计数+task_done 销账防 close 挂死）；catch_all stop() 改真 destroy_subscription（rclpy 强引用，原清列表不停回调）；订阅表驱动化；_task_executor_callback 拆分；TCP markers RViz/HTTP 共享缓存；configure 期 retention 后台线程。+4 单测。

**W11 架构收敛**：observability.yaml git mv 随包走+跨包 import 拆除（AGENTS 单向依赖违例清零）；bag_report 手写四元数→scipy Rotation（TF 链既有测作回归门）；ensure_active 双份 workaround→peach_common 单源；manifest 校验 consumer blob 缓存。

**W12 vegetation**：GPU 推理专用回调组+双线程执行器（原单线程长帧饿死 diagnostics）；订阅 QoS→传感档 BEST_EFFORT（peach_common 新增 sensor 工厂）；cv2 死回退路径删除。

**W13 Round1 遗留**（双实施代理+回放塔字节门）：harvester 死码批（tracking/observation/visualization shim/MODE_LABELS/ForegroundMode）、kernel 缓存、4 处 WARN 节流、pose_pipelines 前奏抽取+_failed 合并（全并模板方法按风险降级并互注）、executor 9 处失败收尾收敛+30 docstring；arm params_bridge 四转换归桥、view_planner 三死参数全链删、axisConsistencyGate 诊断字段化、timed join、十个公有头 /// 补齐。回放塔 3 passed、arm 211 测 0 失败、uncrustify 0 违规。

**W14 bond 接线**（UNWIND 收敛-1）：四托管节点 `/bond` 心跳——peach_arm bondcpp 生效（on_activate 起、deactivate 断）；Python 三节点守卫式 bondpy（**本机未装 ros-jazzy-bondpy 且无 sudo：apt 装上+launch bond_timeout:=8.0 即开 nav2_lm 进程死检，未开启期死检由 supervisor HeartbeatWatchdog 承担**）；launch bond_timeout 参数化默认 0。launch_testing 新用例锁「激活后 ≥3 条 id=peach_arm 心跳」。实测：本机 bondcpp 心跳 1Hz、无 sister ~10s ConnectTimeout 停发；ReadyToTest 提前 2s 赶爆发窗；WaitForTopics 收不到 /bond（机制未明，改直接 rclpy 订阅+对照 /joint_states）。活栈嗅探 reliable/best_effort 双路各收 10 条。

**W15 diagnostics+composition 落档**（UNWIND 收敛-2）：observability /diagnostics 双轨（session_recorder 队列/丢帧 + ingest_liveness 摄入活度，对齐 arm W5 做法）；composition 核实为**平台阻断**——Python 无组件容器、ComponentManager 零生命周期处理（源码 grep 证实）、臂侧非高带宽无零拷贝收益——architecture 偏离表记证据，进程隔离+bond 为当前可达上限。

**终验门**（见 W16 提交）：colcon test 全绿（数量以提交记录为准）、r0_gate 绿、manifest ok（54+4）、mock launch 含 bond 用例绿。
## 2026-09-21 E2E 审查修复轮（G1-G5+M1/M2/M3+M11/M13/M15）

按 2026-09-20 端到端审查优先序修复（审查报告 `reports/2026-09-20-e2e-code-review/`）：

- **G1+G3（令牌生命周期同域）**：`decision.validity_s` 参数化默认 120s（原硬编码 5s 与接近链时长错配——真机单 LIN 7.5s、全链 30-60s；0022「冻结不续签」语义原样，窗口=第二道界）；model_revision 增进程级单调 finalize 计数（`tid:views:N`，reset 不清零），同目标同机位数重 Build 不再让臂持过期快照。supervisor 装配处补过期 WARN（按 target+revision 去重，不改 FSM）。TargetModel.valid_until 第二处硬编码 5s 同参数收口。
- **G2（preview 绑定语义）**：只 PREVIEW 档 goal 记 preview 绑定（observe 是采数据且模型建好前带不了三修订——旧实现把 OBSERVE 记成 preview 是 conservative 档 FULL 必拒根因）；FULL/PREGRASP_ONLY 终局清绑定；受理即拒带 `failure_code=PLAN_MISMATCH(20)`（static_assert 与 IDL 钉死）。新增 3 例 gtest：observe→FULL 不拒 / PREVIEW 改身份仍拒 / 过期分级。
- **M1**：`cancel_requested_` 各动作终局 `!running_` 守卫自动清——单果取消不再拖死后续 MoveTo/补视（已录「批次取消后 PHOTO 亦失败」的机理修复）。**M2**：Survey/MoveTo/预览 worker 三处无限 join 统一 2s 有界（packaged_task 模式）。**M3**：失败码断链修复（plan mismatch→20、recovery 拒→9、锚点缺→1）；授权拒绝分级纯核 `stage_denial`——许可过期→SKIPPED_QUALITY（可重派）、明确不允许→FAILED。
- **G4**：bag_report 原子写（tmp+rename）+ observability SIGTERM 窗 60s + join_report 55s 对齐——停栈报告不再出半份，超窗留「无新报告」可 CLI 复跑。**G5**：preflight 名单补 brain 进程（exec=peach_harvester，三节点合进程不进 argv——残留旧脑双 supervisor 共存的缺口）+ flag_bridge/autostart_client/stereo_camera_node。
- **速赢**：M11 诊断 JSON allowed 与类型化消息同源派生（model_contract 单源）；M13 SceneSnapshot 落盘订阅改 transient_local（晚启动不再丢单发快照）；M15 DepositResult.msg 保留（0012 卸果站预留）但头注释改口为预留现状。

**验证**：colcon test 七包 0 失败（interfaces 9/common 37/bringup 4/harvester 188/arm 216/observability 34/vegetation 16）+ system_tests pytest 三件套 25 过；manifest ok（54+4）；r0_gate 242 纯核过。mock launch 冒烟因用户相机栈在跑按 preflight 设计跳过（arm 侧 12 例接触级 gtest 覆盖），相机空闲后可补跑。**未做**：其余中危 M4-M10/M12/M14/M16-M19 与低危清单（审查报告跟踪）；G1 窗口 120s 与 G2 语义需真机验收。
