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


### 09-20（peach_stereo 配准几何审查 → 修复 → 实测推翻 → 终态回退，相机连机全链验证）

**起点**：对 `src/peach_stereo` 做数学审查（图像/深度/点云/配准）。Z 链（stereoRectify→SGBM→z=f·B/d→0.25mm 量化）、camera_info/TF/点云发布与 percipio 同构性全部核对无误；疑点一处：SGBM 的 z 位于 OpenCV 校正网格（R1/P1），而 SDK 配准按所传标定的针孔解释输入网格（libtyimgproc 1.1.0 反汇编证实：`TYMapDepthImageToPoint3d`/`TYMapPoint3dToDepthImage` 只读内参区、**忽略畸变**、内参按 `imageW/intrinsicWidth` 折算——半分辨率深度合法）。

**设备标定直读**（自写 `gridprobe` 系列探针，非交互按 IP 打开，绕开 selectDevice 交互崩溃）：左 IR K=(1103.57/1104.22, cx=647.40, cy=490.49) 有理畸变 k1=0.14…；右 IR 外参 R 含 **4.2° 绕 Y 会聚**、T=(-62.19, 0.19, 1.72)；**DEPTH_CAM 自有标定：fx=fy=1044.93、cx=605.31、cy=491.63、畸变全零、外参单位阵**（零畸变校正网格，与左 IR 标定完全不同）；彩色(2560×1920) 折算 640×480 后与共用 yaml 差 dcy≈-4.1px（两源小差异，两前端同在，不单独修）。

**第一轮修复（后被推翻）**：按纯针孔推理把 z 用 `initInverseRectificationMap` 反校正回原始左 IR 网格再喂 calibL。base vs fixed 同场景位移实测 **+19.1px**，与解析预测（R1=2.64°绕Y→+19.6px）吻合——warp 实现本身正确。但同帧组实测推翻其方向：

**决定性方法（gridprobe2/3：同帧组同时抓 彩色+双IR+设备深度，TIME_SYNC=HOST 同微秒，消除场景漂移伪影）**——期间发现纯 live 背靠背比对不可靠（本台架枝叶数分钟内漂 3~16px、54% 特征外点；percipio 自一致对照才 ±4px 噪声底）。真 SDK 三方裁决（z_rect+calibL / z_raw+calibL / z 重投影到 calibD 网格+calibD，各自过真 `TYMapDepthImageToColorCoordinate`，与设备深度配准 D=dev∘calibD 比）三组同帧组一致：

| 变体 | 相对 vendor 链位移 |
|------|--------------------|
| **A=z_rect 直接喂 calibL（原实现）** | **+7~+9px（最贴）** |
| B=z 反校正回原始左 IR 网格喂 calibL | +22~+26px |
| C=z 重投影到 calibD 网格喂 calibD | +25~+26px |

**结论与物理解释**：设备把校正旋转折进了 DEPTH_CAM 标定的主点/焦距（cx 605 vs 647 ≈ -42px ≈ R1 2.64°×f），我的校正网格∘calibL 与设备网格∘calibD 本就近似同构；"反校正到原始左 IR"与"calibD 网格移植"两个"更纯"方案都反而偏离。设备真实内部网格不可从标定推导（C 变体 +26px 证明 calibD 针孔≠设备网格）。**手眼/感知按 percipio 链标定——与 vendor 同构即系统正确**。已回退 warp（代码注释载明缘由与"勿再修"），保留三项真修复：

1. **avg_k 有效值均值**：原实现把无效 0 计入 k 帧均值分母（2/5 有效→800mm 变 320mm 仍过 200mm 门，产生偏近幻影面）；改为累加值+逐像素有效计数、按计数求商（计数 0 处=0 保持无效）。k=4 冒烟：近距(200,450)mm 占比 9.09% 与 k=1 的 8.97% 一致（均为真实近结构边缘），无新增幻影；发布节奏按 k 下降符合预期。
2. **color_mode 失配拒启**：原 WARN 后 fallback 640×480 默认值，若设备实际给 2560×1920 则配准/camera_info/点云全链错位；现 FATAL 拒启（启动期非法即拒绝启动）。
3. **SDK 返回值检查**：`TYMapDepthImageToColorCoordinate` 失败丢帧+节流告警（原静默发全零深度）。

**验证与终态**：回退版输出与 09-20 上午基线逐位一致（dx=0 dy=0 |ΔZ|中位 4.8mm，n=14.6 万）；13.6gps 无性能回归；新增 `rectify: R1=…deg` 诊断日志。**遗留（记录在案）**：peach_stereo 与 percipio 配准输出存在 **+8px 系统差**（≈10mm@0.6m，随距离线性）——两前端切换采果前应重做一次手眼标定（旋转样系统差可被手眼吸收）或接受 ~1cm 偏差；勿在节点内再加 warp 修补。lint：本轮新增代码零违规（uncrustify 193 行 diff 与 cpplint 行宽均为提交前存量债，与本轮无关）。

**过程沉淀**：① camport4 预编译 `stereo_grab` 系无 tty 必崩（selectDevice 交互），非交互操作照抄节点 openDevice 流程自写探针；② 稀疏深度图比对用「全局位移扫描+中位 |ΔZ| 谷」而非 tile 匹配（设备深度 10~16% 有效时 tile 全灭）；③ 彩色边-深度边对齐裁判在本台架（枝叶稠密纹理）不可用（重合率平台 ~56%，峰值仅 +2%，选择偏差把任何变体拉向 0）——**同帧组对照是唯一可信仪器**。环境复原：全部探针/节点已停、激光 auto=1/power=50 复验、进程清零。

### 09-20 续（SGBM 参数优化轮：同帧组扫描 → uniqueness 10→6，minDisp>0 证伪）

**方法**：gridprobe4（gridprobe3 参数化派生，标定缓存后纯离线）对 /tmp/simul 3 组同帧组跑真 SDK A 链 vs 设备 D 链；评分=覆盖率优先，三门=精度（谷底中位|ΔZ|≤基线×1.10）/鬼影带不升/轮廓位移≤2px。**两个过程坑（防复发）**：① OpenCV SGBM `minDisparity>0` 时输出视差是相对索引，真实视差=输出+minDisp（dadd 对照：A/D 比值 1.45→1.02）；② gridprobe4 初版 `cv::Mat(640,480)` 行列建反致扫描数据错乱作废重跑——gridprobe3 用 Size 重载无损，+8px 裁决不受影响。

**裁决**：`minDisparity=16` 覆盖 +28pp 全为幻觉（远背景墙强制错配成中距，谷底精度 219~232mm=基线 25 倍；RealSense 式视差窗平移在有远背景入画场景失效）；P2=3200 优于 800（覆盖 +6~8pp）；`numDisparities=128` 最优性证明（0.3m 下限 ⇒ d≥113.8 ⇒ 112 档 Z_min=305mm 会切目标）。**赢家 uniqueness_ratio 10→6**：覆盖 44.55→47.28%（+2.73pp）、精度 9.0mm 门内、鬼影 +0.18pp 经 8 邻域连贯性检验为真墙面非伪影；备选 b7_u6 全门过但覆盖低（记录不落地）。

**落地**：`stereo_camera_node.cpp` 暴露 `sgbm.uniqueness_ratio`（默认 6）+ yaml 键；构建绿（uncrustify/cpplint 失败项=提交前存量债，本轮新增行零违规）。**live 验证**：同场景窗内覆盖 48.45→50.4%（4 帧一致）；帧率 A/B uniq10=13.5gps / uniq6=13.4~13.5gps 无回归；深度中位 736→729mm 无漂移。报告 `reports/2026-09-20-camera-image-analysis/sgbm-sweep.md`。环境：节点/探针全停进程清零；感知演示栈（/demo/* + rviz）应要求保留运行。

### 09-20 续二（学习式立体匹配零样本实测：RAFT-Stereo/IGEV 9 档全灭，better-algorithms 假设闭环）

**动机**：better-algorithms.md §2 遗留"学习式对本机散斑 IR 的泛化是未验证假设，先测再定"。本轮把 GitHub 零样本口碑最好的两族真权重拉到同款三门仪器上裁决。

**方法与仪器等价性**：Python 复现 A 链（rectify→SGBM→Z→0.25mm 量化→ctypes 直调 libtyimgproc `TYMapDepthImageToColorCoordinate`）先对 /tmp/sweep/base 验证——D 链位级一致（diffD=0×3 组）、A 链窗内中位差 0.000mm、覆盖率差 ≤0.05pp、谷底 8.25/8.00/8.50 vs 基线 8.50mm。学习式视差走同一尾部（`/tmp/lstereo/run_model.py`），评分仍是 `sweep_metrics.py` 三门。校正是节点同几何（前 8 畸变、alpha=0），灰度平铺 3 通道（两仓库训练同约定）。

**候选**：RAFT-Stereo（eth3d/middlebury/realtime 权重）+ IGEV（eth3d/middlebury），半分辨率 640×480（与现行可比）与全分辨率 1280×960（IGEV max_disp=256）。环境：yolo_env（torch 2.12.1+cu130@3090）；IGEV 钉 timm==0.5.4，以符号链接法隔离进 /tmp/lstereo/igev_env（yolo_env 的 timm 1.0.27 未动）。权重 gdown 经代理取自两仓库官方 Google Drive。

**裁决（3 组同帧对）**：9 档全部撞毁精度门——谷底 29~84mm vs 基线 8.5mm（3.4~9.9 倍），RAFT 半分覆盖 71%（+24pp）但 A/D 共同域 P25 |ΔZ| 已 19mm、P75 达 172~206mm；A/D 中位比值 1.026~1.036（~3% 深度尺度偏差）+ 大面积局部形变；幻觉面多落在 0.3–1.5m **抓取窗内**（鬼影门反而 0.00 全过——比窗外鬼影更危险）。realtime 变体 28ms/帧（35fps、272MB）是唯一生产帧率档，精度同样全灭。全分辨率把 IGEV 谷底从 80→30mm 但仍差 3.4 倍——域差距是主矛盾，不是分辨率。

**终态**：零样本学习式路线就此关闭；现行 SGBM uniq=6 保持全部已测候选（经典 + 学习式共 14 档）帕累托最优。剩余严肃路线只有域内微调（设备链 18-pattern 深度当伪 GT，同帧组现成），代价未评估。报告 `reports/2026-09-20-camera-image-analysis/learned-stereo-live-test.md`（better-algorithms.md §0/§2/§3 已同步改写）。**第三个"覆盖幻觉"实例**（继 WLS、minDisp>0），再次自证同帧组三门是唯一可信仪器。环境：无节点/相机进程起停（纯离线数据）；/tmp/lstereo 与 /tmp/sweep/{raft_*,igev_*} 易失，表格数字已固化进报告。

### 09-20 续三（点云质量轮：广域调研 + 有效值中值落地 + mono 基础模型探针判负）

**调研**（三路并行，要点与 URL 归档在 `reports/2026-09-20-camera-image-analysis/pointcloud-quality.md`）：厂商滤波栈（librealsense/Orbbec/Percipio/Luxonis：视差域滤波、补洞=假数据官方明言、DQT 四指标、物理层曝光/增益/激光优先）；算法开源（ximgproc 只去噪用法、Open3D 点云 SOR、多帧中值>均值、置信度 PointField 实践）；基础模型（DA-V2 metric/DepthPro/MoGe-2 全景 + Stereo Anywhere/MonSter"mono 裁判"范式）。

**新指标**：局部平面粗糙度（3×3 平面拟合残差中位，mm）——与 DQT Plane Fit RMS / Orbbec Spatial Precision / ISO 10360-13 同族，设备链参考 1.08mm。

**滤波实测（3 组同帧对，三门+粗糙度）**：med3（3×3 有效值中值）全过且粗糙度 1.17→**1.03mm（−12%，低于设备链）**、谷底 8.75 优于 9.00、覆盖不变；med5 鬼影 +0.08 超门；联合双边谷底 11.25 撞精度门（跨边缘平滑，WLS 教训重现）；引导滤波纹理拷入（粗糙度 4.5× 劣化，文献预警命中）。**机理**：中值保边输出真实观测值，线性/核平滑在边缘造中间值。

**大模型探针（判负）**：DA-V2 Metric Indoor/Outdoor Small-hf（33ms/帧@3090）对 0.3–1.5m 工作窗无信号——0.3–0.6m 桶预测 26m、0.6–0.9m 桶 20m（倒挂），ρ≈0.01。"mono 当裁判"文献成立但本内容（近距+植被）双重 OOD；与零样本立体证伪互证。MoGe-2 为唯一未测第二意见。

**落地**：`median_ksize` 参数（0/3/5，非法拒启，默认 3）+ computeDepth 有效值中值（nth_element 无分配）+ yaml 键 + rectify 日志 `med=`。构建绿；lint 新增行零违规（存量 4 项经 stash 对照确认：copyright/include_order/2×行宽）；**live 冒烟过**：med=3 生效、13.5–13.6 gps 无回归、点云/彩色话题在发（域 77，演示栈已恢复运行——旧 13.5gps 节点为换新二进制主动重启）。**域偏差认知（不改行为）**：librealsense 结论平滑应在视差域做；avg_k（Z 域均值）存域偏差属存量债（默认 k=1 未生效），中值是序统计量与单调变换可交换故免疫——后续路线见报告 §5（物理层标定/avg_k 视差域化/置信度字段）。

### 09-20 续四（端到端实测对比轮：12 变体矩阵 vs 原驱动 → sgbm.mode=hh4 落地）

**方法**：`/tmp/lstereo/e2e_matrix.py`——变体轴 scale(0.5/1.0)×mode(3WAY/HH/HH4)×时域(无/视差域 k3 中值/均值)，A 链位级复现（repro_a ctypes 真 SDK 配准）vs 设备 18 图案 D 链，三门+粗糙度+匹配计时；帧组=/tmp/simul（09:45）+ /tmp/simul2（15:54 新鲜复验，gridprobe4 live 重抓）。**前置静态性检查过**（D 链组间谷底 0.75–1.25mm @dx=0），时域融合评估方法论成立。

**裁决**：**`sgbm.mode: hh4` 端到端帕累托最优并落地**——谷底 8.75→7.50mm（新鲜组 9.00→8.00mm）、鬼影 1.65→1.48%（1.32→1.05%）、粗糙度 1.03→0.94mm（低于设备链 1.08）、覆盖 −0.34pp 门内、匹配 10.9→54ms 仍装进相机 73ms 帧间隔。HH4 为本轮关键发现：HH 一半时间拿接近质量（54 vs 116ms，OpenCV 4 路径向量化变体）。**端到端硬约束=发布率**：视差域 k3 中值覆盖 +4.5pp（47→51.5）且门内，但批式语义发布率÷3（1.9–2.8Hz≈设备链 2.43fps——09-17 轮 percipio 曾因此败 observe_build_view_race）→ 记档不落地；全分辨率档（s100 系谷底 5.5–6.5mm 最优）粗糙度 1.4–2.2mm 劣化 + HH 匹配 386–990ms → 记档不落地。k3 视差域**均值**撞精度/边缘门（均值对野值敏感，中值稳健再证）。

**落地**：`stereo_camera_node.cpp` `sgbm.mode` 参数（3way/sgbm/hh/hh4 非法拒启，默认 hh4）+ rectify 日志 `mode=`；yaml 键 + 注释。构建绿；lint 新增行零违规（uncrustify 171 行 diff 全为存量 brace 风格，新增标识符零出现）。**live 冒烟过**：`mode=3` 生效、**13.7gps 700/700 无丢帧**（相机节拍限速，处理链 <73ms 帧间隔）、camera_info 正确、话题在流（hz 订户抖动=已知 raw RELIABLE 大图坍塌缺陷）。演示栈已恢复（域 77 新默认档运行）。报告 `reports/2026-09-20-e2e-stereo-optimal/`（含完整矩阵与记档未落地项：滑窗时域融合、全分辨率特写档、HH 档）。

**与原相机对比分析（round5b，fig6）**：同帧组双链补充测量——D 链逐帧时域一致性 **1.50mm** vs hh4 9.00mm（base 10.25mm）——**单图案逐帧抖动是主机链本征代价**（18 图案融合 vs 单图案），由 med3/avg_k>1(k=5 时 0.96mm 反超设备 1.13mm)/下游多视 TSDF 融合三点吸收；D 窗内覆盖 50.31% vs hh4 47.40%（hh4+k3med 52.2% 反超）；粗糙度 hh4 0.97–1.00mm **优于** D 1.08–1.14mm。停走节拍下端到端仍是 hh4 占优（帧率 5.6×：感知锁定 2.8s vs 48s；双链目标互证差 5mm）。对比表与图见报告 §7/§5.5。

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

### 09-21（percipio 深度崩塌根因定位与修复 + 双前端健康档重测）

**根因终章**：09-20 下午起 percipio 原驱动深度反复崩塌（窗内 valid 5–12%、mdr 0.33–0.55），先后排除配准环节/laser_power/软触发/IR 组件/闲置冷却/SIGKILL 与优雅停组合/激光预置（干净与异常断连）/SDK 固件复位（`TYCloseDevice(h,true)` 仅 0.06→0.12）/整机断电——一夜未愈。09-21 判别实验（install 副本 parameters.xml 改名→官方 launch 无 XML 下发）一击恢复 **0.474/0.977**：真凶为 `percipio_camera/launch/parameters.xml` 调参残留值 `DepthSgbmImageNumber=2` 被 launch 无条件读取下发（percipio_camera.launch.py:13/99），设备 18 图案 SGBM 被砍成 2 幅。**修复**：源码该值清空（=设备默认）+ 注释记录结论 + `colcon build --packages-select percipio_camera`；官方配置复测 **0.477/0.986**。此前"切换毒害/激光状态/热衰减"假设全部修正为排查路径；教训：该 XML 是无条件下发通道，实验值残留即生产事故（本仓纪律同步：percipio_camera=官方驱动+仅本机 IP/分辨率调整）。

**双健康档重测**（主机与相机 08:42 断电重启后，同场景同感知链各 70s/~140 帧）：hh4 vs percipio 修复档——覆盖 0.500 vs 0.478、mdr 1.000 vs 0.985、检出 3/确认 1 一致；**entry std z 0.58 vs 3.31mm（hh4 拟合点稳 5–8×）**、帧间跳变 0.40 vs 1.85mm；percipio ROI 级更稳（掩膜中位 std 0.00 vs 0.32、场景中位 0.34 vs 0.54）；帧源 13.8gps vs 2.43fps（5.7×）。两档均远低于 pregrasp 3mm 门。注意 percipio entry std 昨日曾 0.25mm——随场景/目标构成波动，不作档位优劣断言。**运维新增**：反复强杀容器积累 FastDDS SHM 死锁（/dev/shm/fastrtps_*，实测 166 个）致新订户失连，主机重启或手动清理即除；驱动深度健康口径=首 30 帧 valid（<10% 异常）。

**归档**：peach_stereo/e2e_compare_20260920/（精简后：对比分析文档+实测报告+最新图/视频 7 件+核心脚本 12+数据 2 份）；本轮源码改动=parameters.xml 单值清空。演示栈恢复 stereo hh4（mode=3，13.8gps）。

**因果终裁（bag A/B）**：为排除"一夜重启才是恢复原因"，同日同会话同设备只翻转 XML 值各录 20s ros2 bag（/home/mu/Pictures/video/xml_value2 vs xml_empty，color+depth+camera_info）：值 2→**valid 0.096**（78 帧/20s，帧率反常高=2 幅图案生效佐证）；空值→**valid 0.473（0.470–0.477，39 帧稳定）**。值 2 当日重装即崩、清空即愈——**XML 残留值因果坐实，重启假说排除**。

### 09-21 续（temporal_k 落地轮：滑窗时域中值 + 置信度 PointField + yaml 键匹配 bug 修复）

**temporal_k 实现**：`stereo_camera_node.cpp` 新增 `temporal_k`（1/3/5，非法 FATAL）——配准后彩色网格深度上 k 帧逐像素有效中值（环形缓冲 `treg_`），**每帧照常发布不除率**（区别于 avg_k 批式 Z 域均值）。中值=序统计量、域无关（round4 已证），顺带稳定配准闪烁。同步在 PointCloud2 加 `confidence` 字段（float32：采样数占比×窗内取值一致性，temporal_k=1 时恒 1）。

**live A/B（同场景同感知链各 45s/~85 帧）**：

| 指标 | tk=1（对照） | **tk=3** | tk=5 |
|---|---|---|---|
| entry std (x,y,z) [mm] | 0.32/0.35/0.41 | **0.01/0.11/0.21** | 0.11/0.15/0.25 |
| 帧间跳变 \|Δ\| [mm] | 0.31/0.32/0.30 | **0.01/0.06/0.11** | 0.05/0.10/0.21 |
| 袋半径 [mm] | 34.1±0.27 | 32.9**±0.14** | 32.9**±0.06** |
| 场景深度中位 std [mm] | 0.82 | **0.43** | 0.67 |
| 窗内覆盖 | 0.4974 | **0.5015** | **0.5129** |
| mdr | 0.9999 | 1.0000 | 1.0000 |
| gps | 13.7 | **13.7** | 13.7 |

**裁定**：tk=3 落地为默认档——entry std z **−48%**（0.41→0.21mm）、半径 std **−48%**（0.27→0.14mm）、场景中位 std **−48%**、覆盖 +0.4pp，**13.7gps 无回归**（滑窗在采集线程内 ~3ms 额外开销）。tk=5 半径更优（±0.06mm）但 tk=3 已远超门限且保守。

**yaml 键匹配 bug（存量修复）**：yaml 顶层键 `peach_stereo_camera_node:` 与 launch namespace=`camera` 的节点全名 `/camera/peach_stereo_camera_node` **从不匹配**——此前所有 yaml 部署值实际从未生效（节点全用代码默认值，碰巧一致）。改为 `/**:` 通配后验证 `temporal_k=3` 正确加载。**这是本仓 launch 的存量 bug，所有参数默认值碰巧等于 yaml 值所以未暴露**。

## 2026-09-21 E2E 审查修复轮（G1-G5+M1/M2/M3+M11/M13/M15）

按 2026-09-20 端到端审查优先序修复（审查报告 `reports/2026-09-20-e2e-code-review/`）：

- **G1+G3（令牌生命周期同域）**：`decision.validity_s` 参数化默认 120s（原硬编码 5s 与接近链时长错配——真机单 LIN 7.5s、全链 30-60s；0022「冻结不续签」语义原样，窗口=第二道界）；model_revision 增进程级单调 finalize 计数（`tid:views:N`，reset 不清零），同目标同机位数重 Build 不再让臂持过期快照。supervisor 装配处补过期 WARN（按 target+revision 去重，不改 FSM）。TargetModel.valid_until 第二处硬编码 5s 同参数收口。
- **G2（preview 绑定语义）**：只 PREVIEW 档 goal 记 preview 绑定（observe 是采数据且模型建好前带不了三修订——旧实现把 OBSERVE 记成 preview 是 conservative 档 FULL 必拒根因）；FULL/PREGRASP_ONLY 终局清绑定；受理即拒带 `failure_code=PLAN_MISMATCH(20)`（static_assert 与 IDL 钉死）。新增 3 例 gtest：observe→FULL 不拒 / PREVIEW 改身份仍拒 / 过期分级。
- **M1**：`cancel_requested_` 各动作终局 `!running_` 守卫自动清——单果取消不再拖死后续 MoveTo/补视（已录「批次取消后 PHOTO 亦失败」的机理修复）。**M2**：Survey/MoveTo/预览 worker 三处无限 join 统一 2s 有界（packaged_task 模式）。**M3**：失败码断链修复（plan mismatch→20、recovery 拒→9、锚点缺→1）；授权拒绝分级纯核 `stage_denial`——许可过期→SKIPPED_QUALITY（可重派）、明确不允许→FAILED。
- **G4**：bag_report 原子写（tmp+rename）+ observability SIGTERM 窗 60s + join_report 55s 对齐——停栈报告不再出半份，超窗留「无新报告」可 CLI 复跑。**G5**：preflight 名单补 brain 进程（exec=peach_harvester，三节点合进程不进 argv——残留旧脑双 supervisor 共存的缺口）+ flag_bridge/autostart_client/stereo_camera_node。
- **速赢**：M11 诊断 JSON allowed 与类型化消息同源派生（model_contract 单源）；M13 SceneSnapshot 落盘订阅改 transient_local（晚启动不再丢单发快照）；M15 DepositResult.msg 保留（0012 卸果站预留）但头注释改口为预留现状。

**验证**：colcon test 七包 0 失败（interfaces 9/common 37/bringup 4/harvester 188/arm 216/observability 34/vegetation 16）+ system_tests pytest 三件套 25 过；manifest ok（54+4）；r0_gate 242 纯核过。mock launch 冒烟因用户相机栈在跑按 preflight 设计跳过（arm 侧 12 例接触级 gtest 覆盖），相机空闲后可补跑。**未做**：其余中危 M4-M10/M12/M14/M16-M19 与低危清单（审查报告跟踪）；G1 窗口 120s 与 G2 语义需真机验收。

### 09-21（文档同步轮：把修复轮与相机轮的已实施行为落进三份活文档）

**改了什么（只动 docs，零源码）**：architecture——技能节点「含什么」补 M1 取消旗标收口 / M2 三线程 2s 有界回收 / G2 预览绑定只由 PREVIEW 写入且执行周期终局清复位 / M3a-b 受理期拒单与 onStart 拒绝落码，授权分级句改为 M3c 口径（EXPIRED→SKIPPED_QUALITY 可重派）；决策 0011 补 M3c 细化追记、新增决策 0025（修复轮全清单 + percipio `parameters.xml` 残留清空 + peach_stereo hh4/uniq6/med3 档指档）；整栈入口预检句补 G5 名单依据；观测 bag 固定订阅补 M13 `scene_snapshot` transient_local；相机前端 13.5→13.7 FPS（hh4）。io——`Clearance` 行补装配过期 WARN 与 `decision.validity_s` 指针、相机前端行同步 hh4/13.7 与 parameters.xml 纪律、会话 bag 订阅集补 `scene_snapshot` 落盘 QoS。G1/G3/G4 在本轮之前已由修复轮追记进 0022②/重建节/观测节。

**复跑验证**：`scripts/r0_gate.sh` 绿（vision 8 过 + supervisor 10 过 4 skip + common 19 过 + manifest 54+4）；最近一轮 colcon test（09-21 09:12）七包全绿（arm 199 计 0 失败、harvester 186 过 2 skip、observability 34、common 37、bringup 4）。mock launch_testing 复跑仍被在跑演示相机栈（percipio launch 拉起的 `component_container`）按 preflight 设计拒测——与上条「相机空闲后可补跑」同状态，未清用户栈。

**缺口（记档不修）**：① `scripts/r0_gate.sh` 文件清单未含新增 `test_reconstruction_decision_validity.py`（colcon pytest 会收集并已过；CI peach-core 门不跑该文件）；② peach_stereo lint 存量债（copyright/cpplint 2×行宽/include_order + uncrustify 211 行 diff）仍红——用户工作区既有债，09-20 轮已记档。

**追补（同日 temporal_k/confidence 文档同步，零源码）**：io.md `/camera/depth_registered/points` 表行与相机前端行补 `confidence` FLOAT32 字段（point_step 24：rgb@16、confidence@20；初版 offset16/step20 与 byString 的 rgb 槽重叠、线上颜色被置信度覆写，同日修正 @20/step24；消费方按字段名读——graspnet 旁路 `read_points_numpy(field_names=['x','y','z'])`、move_group octomap、RViz 均兼容）与 `temporal_k` 档（部署=3，A/B 数字见本文件上方「09-21 续（temporal_k 落地轮）」表）；architecture 前端条目 4 与决策 0025 追记同轮，并记 yaml 顶层键 `/**:` 存量修复。PointCloud2 类型不变、manifest 不动；`src/peach_stereo/README.md` 已补 temporal_k/confidence/yaml 键三处条目（规格表行+工作原理+话题节布局段+使用节键名警告）。

### 09-21 终（重启验证 + 旧录制清理重录 + XML 因果现场重测 + 影响评估）

**重启验证挖出四件事并闭环**：① confidence 偏移 bug 线上坐实与修复——重启后 `ros2 topic echo` 见 `rgb@16==confidence@16`，源码改 `confidence@20/point_step=24` + rebuild，字节级复核（中点 rgb=0x76805D 与 conf=0.997 相互独立）；与 percipio 自家点云 rgb@16/step20 布局对齐。② **4 个挂死 `ros2 bag record`（-d 30/-d 10 挂 50–60 分钟，其一写已删除 mcap）以 4 个死订户占住 `/camera/{depth,color}/image_raw`，新订户（topic hz/感知）0 帧**——按 PID 清杀 + `/dev/shm/fastrtps_*` 清零 + daemon 重启 + 栈重启后恢复（30 帧 valid 0.506）；沉淀新规则 `.cursor/rules/test-program-cleanup.mdc`（测试进程 timeout 包裹+收尾 pgrep 复核+清理后复验数据流）。③ 感知叠加图无框无掩膜（用户报告）：生产 yaml `publish_debug_image: false` 所致，采集器 collect.py 本地置 True（生产默认不动），/demo/debug 与视频恢复框/掩膜/entry 轴。④ `ros2 topic hz` CLI 恒 0 帧（echo/rclpy 订户正常）——工具层怪癖记档，健康验证以 30 帧探针为准。

**XML 因果现场重测**（原证据 bag 已清理，按用户要求重验；同场景同会话只翻 `DepthSgbmImageNumber`，源码→rebuild→重启→30 帧探针）：值 2 → valid **0.053**/点云 16,470 点/fps 反常 2.0；空值 → valid **0.466**/154,985 点。因果再坐实；测后 XML 恢复空值 + rebuild。

**全量重录（旧录制清理后）**：同场景同感知链各 55s——hh4 107 帧 / percipio 104 帧；hh4 entry std z **1.20 vs 4.31mm（3.6×）**、半径 **±0.06 vs ±0.24（4×）**、密度 1.000 vs 0.975、粗糙度 0.33 vs 0.44、valid 0.507 vs 0.467；percipio 场景中位 std 0.45 vs 0.65 与左缘覆盖（缺口 59.7% vs 67.0%）仍占优；entry 均值逐轴差 ≤4.1mm（+8px≈10mm@0.6m 系统差内）；确认数 hh4 3/2 vs percipio 3/1（密度差利于过确认门，单轮数据点）。bag hh4 10s（~455MB）/percipio 30s、双 30s 窗口视频与 8 件图全量重生成归档。

**工程补丁（同轮）**：`ros2 bag record -d N` 自停不可靠（收尾极慢、metadata 延迟落盘）→ 一律 `timeout -s INT -k 10` 包裹；make_fig_f.sh `scale=600:-1` 奇数高被 yuv420p/x264 拒 → 改 `-2`；fig_e 改自窗口视频抽帧（make_fig_e.py，替代截图法）。

**归档**：e2e_compare_20260920 更新为重录轮定版——README 索引、report.md（事件链+新数字）、新增 **analysis/peach_project_impact.md（peach 项目影响评估：集成面已接线感知零改动/收益/切换清单/运维风险）**、run_analysis.sh 总跑器入 scripts/。演示栈恢复 stereo hh4+tk3（13.5gps、valid 0.511、字段布局正确）。

### 09-21 终二（testing.md 冒烟节补全：三份活文档对源码的最后一处漂移收口，零源码）

**核对结论**：temporal_k/confidence/parameters.xml/yaml `/**:` 等源码行为此前已由「文档同步轮+追补」落进 architecture（条目 4 + 决策 0025 追记）、io（点云 confidence 字段布局 + 相机前端段落）与本文件；逐条对照 `stereo_camera_node.cpp`/`stereo_camera.yaml`/`parameters.xml` 复核无剩余漂移，唯一漏网是 **testing.md**（c242de3 与追补轮均未触及）：其冒烟节帧率口径只写了 percipio 2.43，且未收录「09-21 终」沉淀的四条运维口径。本轮补齐四句：① 帧率按前端计——stereo 前端 ~13.7 fps（相机节拍限速，hh4+tk3 处理链 <73 ms 帧间隔）；② 驱动深度健康门=头 30 帧 valid <10% 即异常（percipio 崩塌事故验收口径）；③ 本机 `ros2 topic hz` CLI 恒 0 帧怪癖——帧率/健康以 30 帧探针为准；④ 订户全线 0 帧排查路径=挂死 `ros2 bag record`（`timeout -s INT -k 10` 包裹）+ `/dev/shm/fastrtps_*` 残留。

**发现记档不修（用户工作区）**：`src/peach_stereo/config/stereo_camera.yaml` 的 `sgbm.mode: hh4` 行注释写「全率发布 ~8gps」，与其自引报告（`reports/2026-09-20-e2e-stereo-optimal/`：live 13.7gps、700/700 无丢帧、组率由相机节拍决定）及本文件 09-20 续四/live 冒烟记录矛盾——疑为落地前预估未随实测更新。三份活文档与 README 均为 13.7，以实测为准；yaml 属用户工作区未改，留用户处理。

### 09-21 末（test/ 精简归档 + 带标注 30s 终录 + target_1 答复）

**目录迁移（用户主导）**：`e2e_compare_20260920/` 整体迁入 `src/peach_stereo/test/`（analysis/data/report/scripts）并删除旧档；本轮把脚本改自定位 `SD=$(dirname $0)`（run_collect/run_rviz/run_analysis 不再依赖 /tmp 副本与旧档路径），make_fig_e.py/make_fig_f.sh 收进 scripts/（run_analysis 全自洽），verify_stack.py（重启验证探针）入档。旧 bag（hh4_10s/percipio_30s）清除——终录轮不录 bag（精简集=视频+图+jsonl）。

**target_1 深度答复**：上轮 hh4 档 `target_1`=远距小目标——掩膜中位 **758.2±0.43mm**、entry z 761.4±3.23mm（103 帧）；**注意 tid 是会话内轨迹号非稳定物理 ID**（本轮 percipio 的 target_1=近袋 550mm），跨轮比较按物理目标对齐。

**终录轮（带检测框/掩码，60s×2）**：hh4 116 帧 / percipio 115 帧，同场景 3 检 1 确（确认为同一近袋，tid 两档不同）；hh4 entry std (0.18/0.44/0.58)mm vs percipio (0.56/1.75/3.26)mm（**z 5.7×**）、跳变中位 4.6×、密度 1.000 vs 0.975、粗糙度 0.33 vs 0.44、valid 0.509 vs 0.464；场景中位 std 本轮 hh4 反超（0.66 vs 0.74）；左缘覆盖 percipio 略好（59.7% vs 67.6%）；entry 均值 z 差 6.0mm（系统差内）。连续两轮方向一致（z 3.6×→5.7×，量级随场景波动）。30s×2 窗口视频（标注直显）+8 图+jsonl 归档 test/，report.md/README 更新为终版。演示栈恢复 stereo（13.9gps、valid 0.510）。

### 09-21 终三（"录制视频不对"返工：composite 全尺寸窗 + 自检门，145/146 帧终版）

**用户判上一轮视频不合格——复核属实**：rviz 内嵌 Image 面板在 1228×866 窗里被缩到 ~300px 宽且渲染偏暗，2px 框线/掩膜轮廓经视频缩放后不可见（静态分析图 fig_a/cyl/pcq 与 jsonl 数据本身正常，问题纯在视频路径）。**修复**：① collect.py 增 `E2E_GUI=1` composite 全尺寸窗（感知叠加图|彩色|深度JET 各 640×480 横排，cv2.imshow 固定位置供 x11grab 直录）；② `record_session.sh` 一条链=起采集→等帧→**后**起 rviz（闩锁首帧即带标注）→`check_frame.py` 截图自检门（绿/橙框+红掩膜轮廓像素计数，与 debug_draw.py 绘制色对齐；不过门拒绝录制）→30s 双窗并行录制→收尾 pgrep 复核；③ make_fig_f 四源拼接（每前端 composite 上+rviz 3D 下竖叠，两前端横排）。坑修：`set -u` 与 ROS setup 冲突、percipio2 tag 文件名对齐。

**终版轮（75s×2，双档自检门均一次过）**：hh4 145 帧 / percipio 146 帧，3 检 1 确；hh4 entry std **0.39/0.53/1.92mm** vs percipio **2.39/2.88/4.37mm**、跳变中位全线 2–4×、密度 1.000 vs 0.971、粗糙度 0.33 vs 0.44、场景中位 std 0.44 vs 0.83、valid 0.510 vs 0.465；左缘覆盖 percipio 60.2% vs 67.5%；entry 均值逐轴差 0.6/6.0/0.7mm（系统差内）。**连续三轮 entry std z：3.6×/5.7×/2.3×——方向稳定，量级随场景波动**。视频目检：composite 三联全尺寸上检测框/掩膜轮廓/ID 置信度文字清晰可见（终检合格）。归档 test/report/ 增 4 件源视频（{hh4,percipio2}_{composite,rvizwin}.mp4）+ 拼接 fig_f；report.md/README 终版。演示栈恢复 stereo（13.6gps、valid 0.507、无测试残留进程）。

**追记（同日，percipio 覆盖/细节优势定量——用户图上观察复核）**：用户从 fig_pcq 指出"原相机驱动能看清更多细节、范围更好"——量化证实（frame_0141）：**左缘 15% 列覆盖 hh4 0.000（结构性全盲）vs percipio 0.035；远端 1–1.5m 覆盖 0.017 vs 0.024（+40%）；细节密度（全有效 3×3 局部 std）12.8 vs 20.7mm（percipio +62%，含真实细结构与少量噪声纹理——hh4 半分辨率+med3+tk3 平滑链以细节换拟合稳定）**；hh4 的全图 valid 更高（0.510 vs 0.462）是中心区+时域中值贡献，空间分布上 percipio 更连片。report.md §2/§4、README 速查、影响评估 §2（增"代价"节）§3（视点居中升为硬要求+新增覆盖/细节损失条）§6 同轮改口：选型结论不变（停走节拍用 hh4），但 percipio 明确为覆盖/细节敏感场景的正当前端（双前端按场景并存）。

### 09-21 收口（文档同步终检：README 档案链接随 test/ 迁移修正 + reports 全路径，零源码）

**逐条复核**：三份活文档对 temporal_k/confidence/parameters.xml 清空/yaml `/**:` 已由「文档同步轮+追补+终二」收口，与 `stereo_camera_node.cpp`/`stereo_camera.yaml`/`parameters.xml` 现行源码逐条对上。本轮剩余漂移全是 09-21 末「目录迁移（用户主导）」的下游：① `src/peach_stereo/README.md` 端到端对比档案链接仍指旧路径 `e2e_compare_20260920/`（目录已迁 `test/`，链接悬空）——改指 `test/README.md`；② README 两处报告简写 `reports/sgbm-sweep.md`、`reports/pointcloud-quality.md` 实际位于 `reports/2026-09-20-camera-image-analysis/` 下——补全路径；③ README「使用」节 `ros2 topic hz` 快验行补本机 CLI 恒 0 帧怪癖指引（「终」轮沉淀，见 testing.md 冒烟节），条目日期链补全为 09-16/17/18/20/21。architecture「命名与文件树」peach_stereo 条目补 `test/` 一行（档案索引 test/README.md）。testing-log 历史条目中的旧路径按只追加原则保留原样。**仍在用户侧（终二已记档）**：yaml `sgbm.mode: hh4` 行注释「~8gps」与实测 13.7gps 不符，留用户处理。

### 09-21 深度（影响评估深度版：消费者地图/帧率逐层兑现/真实门限考据/真机验证矩阵）

**产出**：`test/analysis/peach_project_impact.md` 重写为深度版。核心增量结论：

1. **帧率优势逐层兑现分解**：相机源 5.6× → 感知层 **3.1×**（BoundedWorker capacity=1 drop_oldest 丢旧保新，scene_perception_node.py:196；09-17 实测 stereo 端感知 ~7.5fps vs percipio 被源限 2.43fps）→ 身份锁定 ~3×（testing-log:278 纯帧数口径 18 帧）→ **重建收口层 0×：09-17 stereo mock E2E 的 observe→build FAIL（views=1<min_views=2）三连缺陷仍在代码**——掩膜按**纳秒精确 stamp** 查表（capture.py:724 `masks.get(stamp_ns)`，无容差配对）+ near_duplicate 静态拒收与 min_views=2 互斥 + 13.4fps 重建队列打满。2.8s vs 48s 宣传口径澄清：48s 含病理 settle 重置，对等口径 ~3×。
2. **"pregrasp 3mm 门"考据为讹传**（沿自早期对比报告并进了本轮所有文档——已全量改口）：仓库真实门限=重建精配准体素 fine_voxel **3mm**、pregrasp 偏置 **30mm**（grasp_standoffs.yaml）、采集漂移门 **40mm**。按真门重算尾部（按稳定目标分组）：近袋 hh4 P95 4.5/MAX 5.5mm、percipio P95 9.0mm（多视均值按 std/√N 收敛入体素内，偏置裕度 3.3–5.5×）；**远目标 0.95m 上 hh4 P95 19.5/MAX 29mm 逼近 30mm 偏置预算**（percipio 额定 0.4–0.8m 根本不确认该目标——两种"范围"语义：hh4 量程远但精度退化，percipio 覆盖连片但量程截止）。
3. **真机验证状态矩阵**：percipio 真机 observe→refit→READY 全链已过（pick1 8.56s）；**stereo 从未真机闭环 observe→build**——切换硬门槛新增【阻断】stamp 配对修复（最近邻容差或掩膜流提频）+ mock 复测 + 真机闭环一次，排序在手眼重标之前。
4. 消费者地图（file:line）：相机话题唯一生产消费者=感知节点（臂/调度/观测只经 IDL）；跨前端耦合参数表（tentative_ttl_frames 20/max_views 24/race 窗自适应 15.1s↔4.4s/sync_slop）入档。
5. 同轮改口：report.md/README/对比分析文档中全部"3mm 门"表述替换为真实门限口径；README 索引标注影响评估为深度版。

### 09-21 收口二（基于源码的文档终检：ps800_eval 工具表补 temp_probe + stereo yaml 过期注释修正，零行为改动）

**逐条核对结论**：对 `stereo_camera_node.cpp` 当前工作区版本做全量源码↔文档对照——参数面（`sgbm.uniqueness_ratio` 默认 6、`sgbm.mode` 3way/sgbm/hh/hh4 非法 FATAL、`median_ksize` 0/3/5 默认 3、`temporal_k` 1/3/5 非法 FATAL、代码默认 1/部署 yaml=3）、confidence 布局（rgb@16/confidence@20/point_step 24、temporal_k=1 时恒 1.0）、注册失败丢帧+5s 节流 WARN、color_mode 失配 FATAL 拒启、avg_k 有效值均值（z_sum/z_cnt）——与 architecture 条目 4+决策 0025 追记、io.md 相机前端段落、testing.md 冒烟节、`src/peach_stereo/README.md` 规格表逐条一致；README 引用的 test/README.md 与两个 reports/ 路径均存在。此前「文档同步轮+追补+终二+收口」的收口声明属实，无剩余漂移。

**本轮修两处**（仅文档/注释，零行为）：

1. `src/percipio_camera/scripts/ps800_eval/README.md`：build.sh 已加 `temp_probe` 构建但「工具一览」缺行——按源码头注释补行（只读温度探针：不开流、不点激光，三路探测 SDK 温度可读性，09-20 激光热管理可行性验证用）。
2. `src/peach_stereo/config/stereo_camera.yaml`：`sgbm.mode: hh4` 行注释「全率发布 ~8gps」是落地前预估残留，与同文件 temporal_k 注释、e2e 报告（live 13.7gps、700/700 无丢帧、组率由相机节拍决定）及本文件 09-20 续四/09-21 终二记录矛盾——终二曾记档「留用户处理」，本轮按用户文档同步指令改为实测口径（匹配 ~54ms 仍装进相机 73ms 帧间隔→全率发布实测 13.7gps）。yaml 注释改动不参与运行时行为；下一轮 percipio_camera/peach_stereo 无需重建。

### 09-21 收口三（文档同步：test/ 精简归档悬空引用清理 + branch_analysis 工具入索引，零源码/零行为）

**逐条复核结论**：承接「收口二」，对工作区源码（`stereo_camera_node.cpp`/`stereo_camera.yaml`/`parameters.xml`/launch/ps800_eval）与三份活文档、`src/peach_stereo/README.md` 重做全量对照——无新增漂移（temporal_k/confidence 布局/yaml `/**:`/parameters.xml 清空口径均已在档）。剩余漂移集中在 test/ 档案自身：①「终三」轮曾归档 4 件源视频（`{hh4,percipio2}_{composite,rvizwin}.mp4`），其后用户精简归档只保留 rvizwin 两件，composite 已不在库，但 `test/README.md` 目录树与 `report/report.md` §3 清单未跟着删（悬空引用）；② 13:51 新增 `test/scripts/branch_analysis.py` 完全未入索引。

**本轮修三处**（仅文档）：

1. `test/README.md`：目录树删 composite 行、rvizwin 行注明 composite 未随归档保留（`fig_f_compare.mp4` 即其合成终版，重录经 record_session.sh 再生成）；scripts 索引补 `branch_analysis.py` 与 `probe_raw_depth.py`/`run_probe.sh`（09-20 评估工具，原索引漏收）。`.cursor/rules/test-program-cleanup.mdc` 等其余引用复核存在，无悬空。
2. `test/report/report.md`：§3 同 composite 口径修正；§6 复现节补 `branch_analysis.py <profile> [n_frames]` 入口（需 stereo 前端与 peach_vegetation 分割同跑，**stdout SUMMARY 须 tee 落盘**）。
3. `branch_analysis.py` 记录：peach_vegetation 枝掩膜 × 相机深度质量分析（避障口径：cov/cov_thick/cov_thin、detail_mm 枝上 3×3 局部 std、mad；掩膜带源图 stamp 与深度就近配对容差 0.3s）。14:00–14:04 已跑两档 `p1_hh4_base`（15 帧）/`p2_hh4_detail`（12 帧），**stdout（逐帧 JSON+SUMMARY 数字）未落盘、产物图仅在 /tmp/e2e_live/branch/**——无数字可入档；两档标签对应的参数组也无落盘记录。后续比较须重跑并 tee 保存 stdout、同步记档参数组。

### 09-21 避障轮（hh4 细化四档实测：peach_vegetation 枝掩膜×深度——"能否更细化"终裁）

**参数组与数字全部落档**（`test/report/branch/branch_p{1..4}.log` + 代表图八件 + `fig_branch_{depth,overlay}.png`；配对容差已放宽 0.6s）：

| 档位（参数） | 枝覆盖 | 细枝覆盖 | 细节密度 | 帧率 |
|---|---|---|---|---|
| p1 hh4 现行（yaml 部署 med3+tk3+scale0.5） | **0.604** | **0.652** | 12.9mm | 13.5gps |
| p2 hh4 med0+tk1 | 0.585 | 0.629 | 13.1mm | 13.5gps |
| p3 hh4 全分辨率（+scale1.0） | 0.493 | 0.544 | 15.1mm | **4.3gps** |
| p4 percipio（640x480 官方档，n=7） | 0.540 | 0.622 | **16.0mm** | ~1–2fps |

**终裁**：①避障口径下 **hh4 现行档已是覆盖最优**——tk3 时域中值补细枝深度闪烁（此前"percipio 覆盖更好"是全图连片口径，枝掩膜×有效深度口径 hh4 反超）；**为避障改档是负收益**（p2：+2% 细节 −3% 覆盖）。②细化上限结构性受限：p3 换 +18% 细节但覆盖 −18%、帧率 −68%、numDisp=128@全分辨率 z_min≈0.53m 侵入 0.3–0.8m 工作区——坏交易不采用；细枝几何精度敏感的单帧场合用 percipio。影响评估新增 §4a、report 新增 §2a、README 图件行同步。

**peach_vegetation 两缺陷记档（待修）**：①**换相机节点后订阅楔死**（收流不吐掩膜，进程 2% CPU 空闲）——每次前端/档位切换后必须重启 veg（本轮三次复现）；②launch autostart 事件与 main 自激活（ensure_active）确定性互杀——launch 路径不可用，`ros2 run` 直起勿发 lifecycle 命令（误发 activate 即打死节点）。运维顺带：分析器须 nohup 分离跑（同命令块内前台跑会被组信号误杀）。演示栈恢复 stereo 现行档（13.6gps、valid 0.507）。

### 09-21 注释同步轮（避障轮结论落进代码注释与文档，零行为改动）

按用户核定把本轮沉淀写进源码注释（全部 comment-only，无行为变化、无需重建生效）：

1. `stereo_camera.yaml`：`num_disparities` 补「勿与 scale=1.0 组合（128@全分辨率 z_min≈0.53m 侵入工作区）」；`processing_scale` 补全分辨率四档实测坏交易（细节 +18%/覆盖 −18%/4.3gps）；`temporal_k` 补避障轮结论（tk3 补细枝闪烁、为避障关滤波负收益）。
2. `target_reconstruction/capture.py`：精确 stamp 查表处（原 :724）补已知缺陷注释——高帧源下近乎必失配、stereo 前端 observe→build 阻断项、修法（最近邻容差/掩膜提频）与影响评估 §3 指针。
3. `peach_vegetation`：节点 docstring 补两条运维注意（launch 互杀勿用/相机重启后须重启本节点）；launch `autostart` 参数描述补互杀警告。
4. `peach_stereo/README.md` 规格表新增「避障（枝细结构）」行（现行档覆盖全档最优/全分辨率坏交易/percipio 细节 16.0mm）。
5. yaml 的 `sgbm.mode` 行 "~8gps" 过时注释此前已由并行轮修正为 13.7gps（收口二记档的"留用户处理"项已闭）。

### 09-21 几何拟合性能轮（preemptive RANSAC 打分 + 球先验抽稀 + Powell 单起点救援，行为契约不变）

**范围**：`vision/common/geometry.py` + `vision/common/bag_landmarks.py` 纯核拟合热路径——感知估计（袋/果位姿）与重建 refine 共用。动机：近距大掩膜点云（N 可达数万）时 RANSAC 打分与 LM 抛光成本随点数线性膨胀；无对外契约/IDL/参数变化，三份活文档对 geometry 只有模块级口径（architecture「拟合共用」行），零活文档改动，本轮只落档。

**三处改动**（依据见源码注释）：

1. **RANSAC 打分两段式（preemptive）**：`ransac_sphere`/`ransac_cylinder` 全部 max_iter 个假设先在固定种子子样本（`_PREEMPT_SCREEN_N=512`，超限才抽、N≤512 原样返回零 rng 消耗）上粗排，仅前 `_PREEMPT_FINALISTS=16` 名回全点集复核取最优。打分成本 O(iter×N) → O(iter×screen + finalists×N)。Nistér 2003 preemptive RANSAC 同构；PCL「先降采样云再拟合」同一实践。假设生成的 rng 消费序与旧版逐字一致（抽样本在循环前一次性发生）；并列按（子样本计数降序、迭代序升序）先到先得，等价旧 `>` 语义；N≤512 时逐数等价单段版本。**语义注意**：大点云下 finalist 选择由子样本粗排决定，与全量打分为近义（固定种子可复现），不承诺逐数一致。
2. **`polish_cylinder_axis` 单起点+救援**：RANSAC 提示起点先跑，仅结果非有限（提示落入退化盆）才补跑三个 canonical 起点（原版四起点无条件全跑）。实测定点数据上四起点收敛到同一 G 极小（hint 与 canonical 差 <10%、轴差在优化容差内），常态下三起点是纯冗余——scipy Powell 每起点 ~5ms 封装成本，热路径每次柱轴抛光省三跑。
3. **球先验拟合抽稀**：`bag_landmarks.estimate_bag_landmarks` 的 `fit_sphere_robust` 输入 >1200 点时固定种子均匀抽 1200（球 RANSAC 打分与 LM 抛光均随点数线性）。球先验是辅助量（诊断 + 多帧融合后的禁切包络先验，非主几何），先验经 refine 多帧融合平滑，单帧抽稀噪声不进主几何。

**验证**：`test_vision_geometry` 8 过；refine/ICP 纯核（`test_reconstruction_refine_result`/`test_reconstruction_icp_cache`）17 过 2 skip（本轮 15:00 后复跑，PYTHONPATH 直指源码树，未起 ROS 图）。全量 colcon 绿基线仍为当日 09:12 七包 0 失败——本轮改动晚于该轮，全量复跑并入下次构建轮。无端到端计时数字（收益为分析性：复杂度式与 Powell 单起点成本，非实测墙钟）。

### 09-21 感知真实数据迭代轮（stereo 活流调参闭环 + 成熟库对比，零行为改动）

**范围**：几何优化（上条）之后，用户接通相机要求真实数据+RViz 迭代取证。全程 stereo hh4 活流（域 77，相机栈为用户数据源保留）；感知单节点调参模式（/tmp launch：output_frame 空=相机系 T=I、gravity fixed、TUNE_DEBUG 开关），不动 yaml 部署默认。细节与数据见 `reports/2026-09-21-perception-live-tuning/report.md`。

**结论**：

1. **性能**：live 2 目标 total 127–145ms、有效 7.5–8fps（容量 1 worker 稳态，源 13.7gps）；geometry 85–99ms≈42–50ms/目标，与微基准一致。昨晚 350–600ms 确证争用放大非代码退化。debug 发布开销实测≈噪声级——**订户门控裁定不做**（PF-3 已兑现零拷贝）。
2. **稳定性**：60s/485 帧身份零抖动（idset_changes=0，双目标全程在册）；r0 debug 图双目标框/掩膜/轴合理、零误检（runs/tune_20260921/r0/）。
3. **发现（未改行为）**：单帧门 ACCEPT 生产/调参均 0 触发（refine 有 non-REJECT 兜底=良性但偏好分支死码）；`low_valid_depth` 按检测框 ROI 均值计分母致生产 56/56 恒命中；`travel_too_short` 同恒命中。三项记遗留，与真机验收轮同改。
4. **成熟库对比（先评估后换）**：identity 匈牙利已是 scipy；pyransac3d 圆柱拟合否决（轴误差 15.9° vs 自研 0.051°、8× 慢、无种子确定性）；Open3D voxel 降采样破坏确定性不采纳。自研拟合栈三轴均优，保留。
5. perf_baseline.json 增 `live_tuning_stereo_2targets_ms` 段。清理：调参节点已停 pgrep 零残留。

### 09-21 感知第二轮调参迭代（单帧门三处语义修正，live 重取证绿）

**范围**：`pose_pipelines.py`（门控/信息 flag 分离 + valid_ratio 目标级口径 + 长度一致性门）、`inference.py`（build_masks 返回 sam_yield）、新测试 `test_vision_gating.py` 7 测；io.md §3.1 单帧门条目同轮改口。全量 **peach_harvester 201 测 0 失败**。

**三处修正与证据**：

1. **valid_ratio 目标级口径**：build_masks 新增返回 sam_yield=|SAM∩valid|/|SAM|，经 estimate_modes→estimate(`target_valid_ratio`) 驱动 low_valid_depth 门/confidence/σ/遮挡分类；None 回退旧 ROI 均值（外部直调兼容）。live 复测：良态目标 confidence **0.14→0.977**，`low_valid_depth` 消失。注：取证中更正一个误判——175240 三帧掩膜下深度 p50=0（87% 零洞），该帧目标真实产率本就 0.131，新旧口径同判差帧；口径修正在**健康帧**上兑现收益。
2. **travel_too_short 复核**：公式 travel=min(袋长−margin_neck, insert_length) 本身正确；生产 56/56 恒命中根因是**袋长塌缩**（隐含 4-5.5cm vs 物理 ~15cm，疑地标/重力极性在该 rig 失准）。处置=阈值不动（健康场景 live 复测 travel 0.055-0.070 不再命中）+ 新增 `axis_length_inconsistent` 门（纯函数 `axis_length_consistent`）把塌缩显式化，根因修复留真机验收轮。
3. **门控/信息分离→ACCEPT 可达**：`_INFORMATIONAL_FLAGS`（taper_*/polarity_*/axis_from_pca/fruit_prior_auxiliary/*_from_band/gravity_defaulted/axis_from_profile_sign）不再压状态；`unbagged_display_only` 留门控（果线仅显示语义不变）。合成良态锥形袋走完整链 **status=ACCEPT、gating=[]、信息 flag 在列**（test 锁定）；refine ACCEPT 偏好分支复活。live 场景两目标仍 REOBSERVE=**error_budget_exceeded**（6.6cm 袋/14mm 径向余量 vs 20° 轴不确定度预算 24mm——真实几何警示，应当门控；圆柱 RANSAC 赢下时 θ→2-5° 即放行）。

**live 重取证（stereo 活流，调参模式）**：60s/462 帧 `idset_changes=0`（身份零抖动保持）；r1 debug 图视觉复核无退化（runs/tune_20260921/r1/）；timing total 166ms/6.6fps（同机负载波动区间，geometry 111ms 为 EMA 未稳+场景方差，无系统性回归）。清理照旧：本轮结束留调参栈运行待用户 RViz 确认后停。

### 09-21 过程可视化 + 分割显示官方风格定版（用户验收轮）

**用户要求**：过程图像可视化（点云+圆柱拟合）+ 分割按官方可视化显示。

1. **RViz 调参配置**（/tmp/perception_tune.rviz + run_rviz_tune.sh，域 77、DISPLAY=:0、Fixed Frame=camera_color_optical_frame）：SceneCloud（/camera/depth_registered/points）+ TargetCloud（/peach/perception/single_cloud 方块）+ CylinderFitting（/peach/perception/markers：轴/入口/绿黄红三态）+ DebugImage + TF。已在用户屏幕拉起，调参栈留运行动态观看。
2. **debug 掩膜改官方 ultralytics plot 风格**（debug_draw.py）：半透明逐实例色填充（alpha 0.40，字节数组和稳定取色）+ 同色轮廓，替代 09-01 起的纯描边（用户本轮明确改口；纹理可透见性保留）。r2 取证 runs/tune_20260921/r2/debug.png：填充/轮廓/箭头/剪切线正常，timing 无回归（total 159ms/7.0fps 同负载区间）。peach_harvester 201 测绿。

### 09-21 袋长 2D+3D 融合（检测框限幅，用户定向）——尺寸对拍驱动

**动机**：用户问「圆柱拟合的尺寸对吗」。活流对拍（/tmp/run_sizecheck.sh：拟合值 vs 掩膜像素×深度/焦距独立反推）：直径 6.7 vs ≥7.8cm（−14%，径向 P95 稳健可接受）；**长度 9.0 vs ≥12.6cm（−29%）系统性偏短**——点云只覆盖有有效深度的可见段，分位带（P10/P90/P98）再截一截。

**实现**（pose_pipelines + inference，与暂停会话遗留的 raw_mask 半成品合并为一套）：`mask_axial_length_px` 取原始 SAM 剪影沿投影轴两端展程（1/99 分位）；build_masks 第 4 返回值透传 raw SAM ROI（未经深度门控）→ estimate_modes 袋线注入 `raw_mask`（果线签名不收，球拟合不受剪影影响）；estimate 内两端只延不缩：掩膜剪影优先、检测框角点沿轴像素范围硬限幅，另设 0.35m 绝对/3×相对护栏。flag=`length_extended_from_2d`（信息类，不压门）。

**验证**：合成深度空洞测试（上半段深度置 0，剪影完整）锁融合恢复袋长；**peach_harvester 202 测 0 失败**；live 复测 target_0：fit_len 0.090→**0.110m**、len_err **−29%→−12%**（剩余=掩膜分位裁剪+投影近似，且掩膜反推本身是物理下界）、travel 0.075→0.095m；直径/耗时无回归（r3 total 102ms/9.9fps）。取证 runs/tune_20260921/r3/。io.md §3.1 同轮。

**边界说明**：融合后 travel 变长会让「误差预算门」（(standoff+travel)·sinθ vs 径向净空）更易触发——这是诚实几何（杆臂长了指向误差放大），不是回归；ACCEPT 合成测试已改用短袋场景保持「信息 flag 不压门」命题独立。

### 09-21 mock 臂 + 真立体相机：跳过重建验证接触链（审查轮，不评方向）

**档位**：`hardware_mode:=mock` `camera_frontend:=stereo` `skip_reconstruction:=true` `autostart:=false`；`SetEnables(execution+grasp, tool=false)`；默认 `execute_pregrasp_only=true`。不放松 `min_views` / `max_target_drift_m`。未授权真机动臂/SetIO。

**接线问题与同轮修复**：① mock 无 `aubo_io_controller`，`require_robot_status` 仍 true → Survey ~40 ms `robot_status_missing`；`harvest_system` mock overlay 置 false。② `publish_debug_image` 须默认 true（空 Debug Image）。③ 不开 skip 时 PICK_ALL 卡 `observe_build_view_race`（views=1，`min_views=2`；mock TF 动、相机不跟）。④ `skip_reconstruction` 根级 launch overlay 打不进 brain 进程里的 `peach_supervisor`，须 `peach_supervisor.ros__parameters`；本轮先热设再补 overlay。

**走通**：`e2e_unrefined_20260921T185527` Survey→锁→无 Build→`ExecuteTarget PREGRASP_ONLY`（reconfirm 0.43 s + approach_insert 13.0 s）→ ACK → 回拍照位再 Survey → `completed`。ledger `target_1` outcome=0、`completion_level=2`、`geometry_source=scene_observation`。全程无 SetIO。重建节点仍 WARN 队列满，不挡接触。跑法见 [testing.md](testing.md)「实验室端到端」。

### 09-21 续：skip 路径 FULL 许可钉住（未融合 GraspDecision 冲门修复，未实跑 FULL）

**缺陷**：skip_reconstruction 路径重建节点不关、持续发布未融合 GraspDecision（常 `allowed=false`），FULL 有两条路径被它冲掉套入许可——① 臂侧 `TargetCache.updateGraspDecision` 把 `promoteUnrefinedGeometry` 钉住的 `quality.grasp_allowed` 冲回 false，CONTACT 复检拒；② 调度装配 ExecuteTarget goal 时把该决策装成 CONTACT 令牌，臂侧令牌复检 `allowed=false` 拒。PREGRASP_ONLY 轮不读 `grasp_allowed` 故未暴露（`e2e_unrefined_20260921T185527` 只有预抓取）。

**修复**（`peach_arm/target_cache` + `peach_harvester/executor_node`）：`promoteUnrefinedGeometry` 同时钉 `grasp_allowed=true` 与决策目标 ID；`unrefined_hold_` 期间 `updateGraspDecision` 直接放行不落账（心跳/决策不续签语义不变，有效期仍只经 replaceModelSnapshot）；调度 skip 档 decision 置空整体不装令牌（走快照回退=钉住后的臂侧 quality）。`test_target_cache` 锁定钉住+拒绝决策不冲门，peach_arm 12 测绿（本机复跑）；调度装配分支属节点层，现行测试塔无覆盖（缺口记下）。

**文档同轮**：architecture 档位/FSM 段、io.md `allow_unrefined_geometry` / `skip_reconstruction` / Clearance 行、testing.md e2e_unrefined 行补档位说明并新增「跳过重建完整接触（套入干跑，不开刀）」一节。该 FULL 干跑**尚未实跑**；验收断言已逐一对照源码核过（tool 关跳 ActuateCutter/VerifyCut、撤离成功即清 recovery、`VerifyHarvestOutcome` 在 tool 关时不拦、不记采摘成功）。

### 09-22 mock 臂 + 真立体相机：skip 路径 FULL 套入干跑（不开刀）

**档位**：`hardware_mode:=mock` `camera_frontend:=stereo` `skip_reconstruction:=true` `autostart:=false`；运行期 `execute_pregrasp_only:=false`；`SetEnables(execution+grasp, tool=false)`。未授权真机动臂/SetIO。launch 后 `/peach_supervisor skip_reconstruction` 仍为 False（brain 一进程三节点，`Node()` 默认名 `peach_harvester`，嵌套 `peach_supervisor.ros__parameters` overlay 打不进），热设 True 后核过；臂侧 `quality.allow_unrefined_geometry` 已 True。深度探针 30 帧 valid_mean=0.496、13.6 fps；手眼平移 `[0.045, 0.108, 0.002]`。

**走通到哪**：`e2e_full_unrefined_20260922T094345` Survey→锁→无 Build→`ExecuteTarget` **FULL**（再确认 0.36 s + 接近 12.9 s 到预抓取，检查点 CK_AT_PREGRASP）→ CONTACT 授权过（未再被未融合 `GraspDecision.allowed=false` 拒）→ `previewFullContact` 失败。账本 `target_1` outcome=2、`failure_code=skipped_unreachable`、`completion_level=2`、阶段仅 `reconfirm`+`approach_insert`。ACK 后回访 Survey，已 claim 不再选 → `HarvestState.batch_state=completed`。全程无 SetIO；`tool.enabled` 保持 false。

**套入失败原因（不是许可门）**：MTC `sleeve linear along bag axis (0/1)`。感知 `d95_m≈0.068`、袋底 `base_link` ≈`[0.434, -0.611, 0.610]`、轴 ≈`[0.296, -0.521, 0.800]`；预抓取 |p|≈0.94 m（E5 工作空间边缘，IK 仍到）；沿轴插入到袋颈 |p|≈1.05 m，笛卡尔直线无解。`promoteUnrefinedGeometry` 未拷贝 `bag_diameter_upper_m`（观测侧也未入 `CachedTarget`），胶囊回退 0.12 m——与本轮 Cartesian 0/1 无关，skip FULL 几何钉住仍缺口。重建节点仍 WARN `missing_mask`/队列，不挡接触。报告 `runs/e2e_full_unrefined_20260922T094345/round_report.md`；会话 bag `runs/session_20260922_094104/bag/`（停栈后 observability 未出 `bag_report`）。

### 09-22 续：skip overlay / 直径钉住 + mock FULL 套入干跑走通（不开刀）

**缺口补上（同轮源码+活文档）**：① brain unnamed `Node` 上嵌套 `peach_supervisor.ros__parameters` dict 打不进进程内调度节点；改为 OpaqueFunction 写成 `peach_supervisor` 键 ParameterFile。本轮 mock 起栈后 **未热设** `ros2 param get /peach_supervisor skip_reconstruction` → True，臂侧 `quality.allow_unrefined_geometry` True。② 观测 `bag_diameter_upper_m` 进 `CachedTarget` / Selected·Locked update，`promoteUnrefinedGeometry` 拷进精化；gtest `PromoteUnrefinedFromLockedAndHoldsAgainstDiagnostics` 断言 0.068。③ `sim_field_targets.py --mode full`：goal 须 `PROFILE_FULL`（默认 profile=0 会把 `mode=FULL` 盖成停预抓取）。

**软件接触链（无相机、在达几何）**：`hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true autostart:=false imu_enabled:=false`；`scripts/sim_field_targets.py --mode full --case 1757 --velocity 1.0`（tool 保持关，直发 `ExecuteTarget` 不经 `RunHarvest`）。`runs/sim_field_targets_20260922_100400.jsonl`：outcome=0、`completion_level=6`（`LEVEL_RETREAT_CONFIRMED`）、`grasped=false`、`recovery_required=false`、`pregrasp_passed=true`。检查点 2 预抓取 → 3 套入预规划 → 4 套入到位 → `tool.enabled=false` 跳过 SetIO → 8 原路撤回 → 9 `harvest_stow` → `SUCCEEDED`。无「感知果实直径无效」回退（注入直径 0.06 m）。脚本 `detour_flag` 是 `--velocity 1.0` 切段把返程并进接近（`from_photo=false`），不当套入形状。实验室上午那袋 |p|≈1.05 m 仍不可达，未重跑相机 FULL。

**停栈**：launch SIGINT 后 observability 挂死（PID 144945），TERM 2 s 仍在则 KILL；`pgrep` 无 peach/ros2 残留；`/dev/shm/fastrtps_*` 已清。会话 bag `runs/session_20260922_100001/bag/`（KILL 后无 `bag_report`）。`test_target_cache` 12 测绿。未授权真机动臂/SetIO。

### 09-22 续：SELECT 套入终点 IK + mode 权威；mock 复跑 1757 FULL

**档位**：`hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true autostart:=false imu_enabled:=false`。launch overlay 后 `/peach_supervisor skip_reconstruction` True、`quality.allow_unrefined_geometry` True、四托管 Active。未授权真机动臂/SetIO。`SetEnables(execution)` 后 `SurveyScene` 到拍照位再探 IK（与 SELECT 种子一致）。

**CheckReachability（拍照位种子）**：

| 入口 | travel | require_sleeve | 结果 |
|------|--------|----------------|------|
| 1757 `[0.304,-0.614,0.536]` \|e\|=0.870 \|s\|=0.903 | 0.06 | false / true | 均 `reachable=true` |
| 实验室上午袋 `[0.434,-0.611,0.610]` 轴 `[0.296,-0.521,0.800]` \|e\|=0.966 \|s\|=1.044 | 0.08 | false / true | **均 true**（单点 IK 仍有解） |
| 同上 | 0.20 | true | `sleeve_no_ik`（\|s\|=1.161） |
| `[1.20,-0.40,0.50]` \|e\|=1.36 | 0.08 | false | `no_ik` |
| 径向前插 `[0.70,-0.50,0.40]` travel 0.20 \|s\|=1.145 | — | false / true | 预抓取 true / `sleeve_no_ik` |

上午那袋 travel≈0.08 时 SELECT 套入终点 IK **仍会放行**；当时失败是 MTC 沿轴 LIN `(0/1)`，不是终点无 IK。门挡住的是「终点已经无 IK」的袋，不是全部笛卡尔空洞。

**软件接触链**：`SetEnables(execution+grasp, tool=false)` + `scripts/sim_field_targets.py --mode full --case 1757 --velocity 1.0`。`runs/sim_field_targets_20260922_103933.jsonl`：outcome=0、`completion_level=6`、`grasped=false`、`recovery_required=false`、`pregrasp_passed=true`、阶段 reconfirm 0.26 s / approach_insert 12.1 s / retreat 3.3 s。检查点 2→3→4 → `[ACTUATE_TOOL] tool.enabled=false，跳过末端 IO` → 8→9 → `[SUCCEEDED]`。脚本 `detour_flag=true`（`from_photo=true` 段事后胶囊审查）不当套入失败。

**停栈**：launch 父进程已退、子进程成孤儿；按 PID TERM，observability 282612 2 s 后 KILL；`pgrep` 无 peach/ros2 节点；`/dev/shm/fastrtps_*` 已清。会话 `runs/session_20260922_103652/bag/`（KILL 后无 `bag_report`）。

### 09-22 续：本轮不开自适应；hollow mock FULL 网格 9/10

**档位**：`hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true tool_profile:=hollow_cylinder_v1 imu_enabled:=false autostart:=false`（`ROS_DOMAIN_ID=43`）。未授权真机动臂/SetIO。本轮**不起** `adaptive_cylinder_v1` / `imu_follow` / Servo。

**隔离**：`/peach_arm tool.profile_id=hollow_cylinder_v1`；无 `imu_follow` / `servo_node` 进程与节点；`ros2 service list` 无 `/imu_follow/*`。空心栈若仍 `create_client(/imu_follow/enable)`，FastDDS 图上会挂出服务名（无服务端）——已改为仅自适应档案建客户端。

**网格** `scripts/sim_field_targets.py --grid --mode full --velocity 1.0 --tool-profile hollow_cylinder_v1` → `runs/sim_field_targets_20260922_120255.jsonl`：**9/10 matched**。在达 5 例（`typical_1757` / `tilt_1639_1` / `travel_min` / `travel_max` / `info_length_extended`）outcome=0、`completion_level=6`、`grasped=false`、无 SetIO。SELECT 正确 skip：`lab_oos_20260922`=`sleeve_no_cartesian`、`far_no_ik`=`no_ik`、`bbox_edge`、`tool_clearance_failed`。`near_horizontal_1021_1` 套入/撤回完成（completion=6）后 `ReturnHarvestStow` PTP「累计关节行程 6.91 rad > 6 rad」→ outcome=3（`transit_max` 护栏，不是套入失败）。脚本 `detour_flag` 仍是胶囊事后审查，不当套入失败。

**停栈**：launch SIGINT 后按 preflight PID TERM；observability 2 s 后 KILL。当时预检名单只有节点名 `peach_lifecycle_manager`、匹配不到 nav2 可执行名 `lifecycle_manager`，漏杀两只 lm（433358/449158），已按 PID 清并补名单。`pgrep` 无 peach/ros2 残留。会话 `runs/session_20260922_120244/bag/`（后补 `peach_bag_report`）。`peach_arm` 18/18、`peach_bringup`/`peach_system_tests` 本包测绿。

### 09-22 续：bag/RViz 对照后 stow 两跳；hollow 网格 10/10

**证据**：`session_20260922_112338` / `120244` 作业票同一句：`返回 harvest_stow 失败: 拍照位姿拒绝绕行轨迹: 累计关节行程 7.57/6.91 rad > 6 rad`，入口 `[0.518,-0.695,0.521]`=`near_horizontal_1021_1`。套入/撤回已完成（jsonl completion=6）。根因不是 `transit_max` 过严：`harvest_stow` 与 `global_photo_pose` 已分叉（wrist2 −0.50 vs −0.28，Δ≈0.22 rad），`stageReturnHarvestStow` 直调 `goToPhotoPose(stow)` 对不上接近轨迹起点，跳过原路返程去新规划 PTP。本轮网格无 RViz 录像（当时 `QT_QPA_PLATFORM=offscreen`，rviz2 崩）。09-18 `camera_ab_*_rviz.mp4` 是感知 A/B，不含本网格接触。

**调优（未抬 6 rad 门）**：`harvestStowNamedHops(photo, stow)` → 先 `goToPhotoPose(photo)`（可倒放已过门接近）再短 PTP `harvest_stow`。gtest `HarvestStowNamedHopsViaPhotoThenStow`。architecture / io / testing / `peach_arm.yaml` 同轮。

**复跑**：`tool_profile:=hollow_cylinder_v1 imu_enabled:=false`，无 `/imu_follow`。`runs/sim_field_targets_20260922_134716.jsonl`：**10/10**。`near_horizontal_1021_1` outcome=0、completion=6、grasped=false。RViz `runs/grid_hollow_stowfix_20260922/rvizwin.mp4`（4:00，1418×815）：t=5/60/150 空心筒口对袋轴、白线为接近/套入；Debug Image 无帧（本网格 `camera_enabled:=false`）。预检补 `lifecycle_manager` 与 `rviz2`（本轮曾漏杀 rviz 494433，已按 PID 清）。会话 `runs/session_20260922_134706/`。未授权真机动臂/SetIO。柜 IP 169.254.10.98 本轮 ping 不通；相机 169.254.10.110 通，live 仍 hollow。

### 09-22 续：接近改为果平面折线 LIN

**证据**：`runs/sim_field_targets_20260922_134716.jsonl` 接近段 path/chord 常 1.5–2.65、弦偏离 0.31–0.61 m；RViz t60/t150 白线从拍照位抡出大弧。根因是拍照位→staging 的关节 PTP 不约束 TCP 面，侧向扫枝。

**复跑**：`tool_profile:=hollow_cylinder_v1 imu_enabled:=false`。`runs/sim_field_targets_20260922_141241.jsonl`（会话 `session_20260922_141232`）：**10/10**。`typical_1757` 走通果平面折线（三跳 LIN ratio 1.00）；tilt/travel_min/travel_max/near_horizontal 折线规划失败后 PTP 兜底仍到位。未授权真机动臂/SetIO。

### 09-22 续：接近不再用近果档；可视化只画已走到

**证据**：折线末跳（AlongAxis）套了 `approach_near_velocity_scaling` 0.05，把赶路拖成接触速度；每个滚转候选都 `plan()` 折线，`DisplayMotionPath` 把护栏拒掉的解发到 `/display_planned_path`。observability `tcp_chord` 把会话首末点直连，穿过未走到的空间；`planned_views` 把最多 24 个未走视点画成箭头。

**调优**：接近折线三跳与 PTP 兜底沿轴 LIN 走 `velocity_scaling` 0.10；`approach_near` 0.05 只套入/撤退。折线 `plan()` 每趟至多一次。Pilz/OMPL 去掉 `DisplayMotionPath`（对齐 Nav2：只可视化正在跟随/已执行的路径）。删 `tcp_chord`。`planned_views` 只在观察短移成功后画该视点。RViz Planned Path `Show Robot Visual=false`。pytest `test_tcp_markers_draw_sampled_path_not_unreached_chord`。未抬 `transit_max`。本轮不开自适应、未 `hardware_mode:=real`。

### 09-22 续：接近改面内斜插；网格点位拉开

**证据**：矩形两段 LIN（先落到果面再横收 0.4 m）把一次接近拆成三次规划，路程是直角边之和；`--grid` 六个 succeed 里四个入口都是 1757，轨迹看起来永远同一条。枝间需要垂直进果，但不需要走满矩形。

**调优**：未齐先原地对齐；面内一跳斜插到轴上 staging，再沿轴垂直进入（gtest `PlanarApproachHopsAisleThenPerpendicular` 断言第一跳同时有侧向与轴向、比直角两段短）。折线 LIN 加速度仍封顶 0.10。网格在达点改为现场不同簇（1757 巷中 / 1639_1 深左斜 / 1113_1 近左 travel_min / 1740 中偏 travel_max / 1113_0 右巷 flag / 1021_1 近水平）。打断的 mock 栈已按 PID 清。未 `hardware_mode:=real`。

### 09-22 续：斜插 keep-roll，不叠刀口 Rz

**模型**：空心圆筒 TCP `Rx(-90°)`，Z=开口、XY=刀口；零位开口朝世界 +Z。SRDF `global_photo_pose` 的 TCP 四元数近单位阵（xyzw `[-0.005,-0.010,-0.037,0.999]`），开口已近 +Z。

**感知**：悬挂袋轴亦近 +Z（`typical_1757` axis `[0.042,0.136,0.990]`）。拍照位 TCP Z 与袋轴夹角 **8.4°**（与 09-01 现场「倾斜约 8°、目视不像大拧腕」一致）。圆筒绕 Z 对称，笛卡尔斜插不需要 ±30°/±60° 刀口滚转。

**调优**：`planarApproachHops` 只用 `alignFrameZ` keep-roll；笛卡尔折线不再 `applyPlanarApproachOrientation` 叠候选滚转（滚转只给 PTP 兜底）。小对轴仍走 **长斜插**——把 8.4° 挪到 0.13 m 沿轴段后 typical wrist1 到 −7.18（与零位移拧腕同类），已收回。gtest `PlanarApproachHopsKeepsPhotoRollWhenZAlreadyNearAxis`。`peach_arm` 18/18。

**网格** `runs/sim_field_targets_20260922_154317.jsonl`：**10/10**（hollow mock FULL；未 `hardware_mode:=real`）。日志对轴 8.4°/16.8°/23.5°/28.2°/31.4°/75° keep-roll。近水平 75° 是袋轴本身接近水平，不是刀口 Rz。停栈 leftover 0。

### 09-22 续：keep-roll 长斜插复核；保持拍照姿态撤回

**证据** `runs/sim_field_targets_20260922_155057.jsonl`：**10/10**（hollow mock FULL；`ROS_DOMAIN_ID=55`；未 `hardware_mode:=real`）。`near_horizontal` outcome=0 completion=6，stow 不再 `transit_max`。typical 对轴 8.4° keep-roll 走长斜插：wrist1 −3.28 / 限 3，降档 −3.12，改 PTP；执行接近比 1.33–1.56（typical 1.56、travel_min 1.46、travel_max 1.48、info 1.33、近水平 1.36）。tilt 当时比 34.6 是 photo→photo 切段误把整周期合成一段，不是套入失败。

**保持拍照姿态（撤回）**：对轴 <15° 时 hop 不 `alignFrameZ`，typical wrist1 恶化到 −7.15 / 降档 −5.51（残差落到 0.13 m 沿轴，与 delay-align 同类）。已收回，仍 keep-roll 走长斜边。刀口 ±30/±60 仍不叠进笛卡尔。

**切段**：`sim_field_targets` 把「起止都在拍照位、路径≫弦」的段按离拍照最远点拆成出程/返程。keep-photo 半轮 tilt 比从 34.6 变成 1.13。

`peach_arm` 18/18。未 `hardware_mode:=real`。

### 09-22 续：空心 live SURVEY（mock 臂 + 真立体相机）

柜 `169.254.10.98` 不通，未 `hardware_mode:=real`。相机 `169.254.10.110` PS800-E1 在。`camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true tool_profile:=hollow_cylinder_v1 imu_enabled:=false`。彩色探针 30 帧 / 2.37 s ≈ **12.6 FPS**。`SetEnables(execution=true)` 后 `RunHarvest` `e2e_survey_20260922T160318` intent=2：**SUCCEEDED** `termination_reason=completed`，discovered=1 attempted=0，账本 `claimed=[]`。感知 `target_1`。使能已关。停栈 leftover 0。panda_sort_gazebo 未动。

### 09-22 续：自适应 mock FULL 网格（与空心隔离）

`tool_profile:=adaptive_cylinder_v1`，`imu_enabled:=false`（无 USB），`imu_follow` 仍随档案 Include。`runs/sim_field_targets_20260922_161623.jsonl`：**10/10**。succeed 六案 completion=6、接近比 1.34–1.61、胶囊外。套入阶段日志「IMU 跟随接管、跳过 MTC CartesianPath」；`waitImuFollowTravel` 按 0.01 m/s 墙钟等，不是真跟随位移。贴边/净空 skip_select、超程 `no_ik`、实验室袋 `sleeve_no_cartesian` 仍命中。未 `hardware_mode:=real`。

### 09-22 续：空心 live SURVEY + PREGRASP（真立体相机 + mock 臂）

柜不通，未 `hardware_mode:=real`。`tool_profile:=hollow_cylinder_v1 imu_enabled:=false`，图上无 `imu_follow`/`servo`。彩色 30 帧 / 4.88 s ≈ **6.1 FPS**。`e2e_survey_20260922T162215` SURVEY_ONLY completed、discovered=1。锁定采样 8 拍全是 `target_1` confirmed、`target_set_locked=True`（ID 不闪）。`e2e_unrefined_20260922T162302` skip_reconstruction 无 Build：账本 outcome=0、`completion_level=2`、阶段 `reconfirm`+`approach_insert` 10.2 s，`[SUCCEEDED] PREGRASP_ONLY` 停预抓取、无 SetIO。过早 ACK 被拒「周期运行中不能确认恢复」；Hold 后命令 6 才 `accepted`。使能已关。停栈 leftover 0。live FULL 未发：实验室袋 SELECT 仍可能 `sleeve_no_cartesian`。

### 09-22 续：对照 bag/RViz 视频收可视化

**证据**：`runs/grid_hollow_stowfix_20260922/{t5,t60,t150}.png` 白线三角 + 1 m TF 名 `tool_axis` 挡住筒口；`e2e_adaptive_grid_20260922T1616/rvizwin.mp4` 多数帧是叠在 RViz 上的 IDE（x11grab 录屏幕像素）。live 帧 `adapt_t350` Debug Image 有 `target_0` 掩膜，检测点云过碎、TSDF/重建 markers 在 skip_reconstruction 下仍开。8090 固定 X–Y 俯视把沿轴升程压扁。

**调优**：RViz TF 轴 0.15 m、不显示名字/箭头；Detection Cloud 开（8 mm 方块）；TSDF/重建 markers/TCP Path 默认关。套入轴只画预抓取→入口，换 run/目标清路标。8090 按跨度最大两轴投影并按相位着色。录像脚本先 `wmctrl` 前置 MoveIt 窗。未 `hardware_mode:=real`。

### 09-22 续：过程数据压缩

`runs/` 只留空心网格 `155057`、自适应网格 `161623`、live SURVEY `e2e_survey_20260922T162215`、live PREGRASP `e2e_unrefined_20260922T162302`、stowfix RViz 关键帧。历史 idle/session/harvest 目录、MCAP、被挡住的自适应录像、根目录自行车草稿已删。旧 `reports/`（09-17～09-20）与 `summary_2026-09-14` 归档 `_archive/`。`_archive/runs` 与 `_archive/caches` 清空。结论仍以本文件为准。


## 2026-09-22 战役 P0 缺口修复（双末端验证战役准备轮）

- `sim_field_targets.py`：`--tool-profile` 启动时与 `/peach_arm tool.profile_id` fail-fast 比对（档案切换须整栈重启，错配即拒跑）；注入 GraspDecision 按档案 D_inner×袋径复算径向预算（复用 `tool_budget.evaluate_sleeve_cut`，allowed=sleeve_ok），工具体常量从 aubo_description 档案装载。**发现 A 级候选：axial_margin 在当前档案误差常数下结构性为负**（blade_capture 8mm < axial_safety 4mm + blade_plane/robot/motion 固定误差 7mm），真实 refined 链同式同判——这正是 skip_reconstruction 接触链存在的原因之一；sim 决策只执行径向门保臂链可测，轴向口径待 P3 裁定。
- `perception_constraint_grid.yaml` 10→20 例：袋径 0.05/0.068/0.10（0.10=hollow 预算拒）、短袋 0.03/长袋 0.18、行程 0.24 夹紧、occluded 信息类 flag、右巷斜轴/深左低轴簇；`expect` 新增 `deny_decision`（FULL 模式下注入 allowed=False，臂侧拒接触 completion<3）。test_constraint_grid 同轮扩矩阵覆盖断言。
- `scene_perception_node`：P0-4 暴露 BoundedWorker.dropped（只在计数增长时 WARN；drop_oldest 下 submit 恒真，此前丢帧完全静默）。
- rviz：`moveit_campaign.rviz` 战役定版副本（关键 Display 全开核对），防战役期间 GUI 改动污染基础配置。
- 战役基建：`campaign/20260922_dual_tool/`（README 轮次台账/bags 索引/analysis/scripts/videos）+ 三脚本（run_round.sh 编排、per_round_summary.py 轮次门、stability_metrics.py P1 指标含 bag 回放 3σ）；e2e_unrefined_20260922T162302 实数据离线验证过。
- 门：r0_gate 绿；peach_harvester/peach_arm colcon test 绿（lint 首轮抓出 docstring 格式 3 处，已清）。


## 2026-09-23 动作通道有界执行防线（TEM stop 事件风暴根因落地）

- M1 tilt_1639_1 300 s hang（96cdd79 记 A 级候选；sim 侧 goal 超时取消已先行修复，污染 jsonl 留证）当日根因定位：move_group TEM 异常——`transit_max` 触发的 stop 事件风暴（8.2 万行日志不停）使同步 `execute` 永久阻塞且不理取消，动作通道一次即永久卡死，后续目标级联失败。
- 防线：新 GPL 参数 `peach_arm moveit.execute_timeout_s`（默认 90 s，校验 >1；部署值 `config/peach_arm.yaml`）。`motion.cpp` 全部 execute 收口 `boundedExecute`：async 派发 + 等待环先到先收，取消探针（节点注入 `cancel_requested_`）命中或超时即 `move_group_->stop()`，10 s 宽限仍不返回则放弃等待（线程滞留一次换通道可用）。新公有 `MoveItMotionInterface::stopExecution()`（MGI::stop 打节点级停止服务，与发起执行的接口实例无关）。MTC 侧 `Task::execute` 同样 async 先到先收，`GraspTaskConfig.execution_stop` 由节点注入 `stopExecution()` 兜底，超时 reason=`MTC execution timeout (stop issued)`。
- 附带日志归因：`TargetCache::updateRefinedPose/updateRefinedFitting` 增 `reject_reason` 出参（`unrefined_hold`/`target_mismatch`），臂侧 WARN_THROTTLE 10 s——unrefined_hold 是 skip_reconstruction 批的 latched 0.5 s 心跳常态，原无节流 WARN 单轮刷 8 万行淹没真信号。
- 文档：architecture 决策 0027 + peach_arm 节点「执行有界等待」段同轮。零 IDL/话题/QoS 变化。无真机运动。

### 09-23 续：接近轨迹定型「斜直线+垂直进入」，删 staging PTP 绕角与多级兜底（用户裁定）

- **定型**：接近主路径 = 斜直线（面内一跳到预抓取下方轴上，keep-roll 对轴）+ 沿轴垂直进入预抓取；套取后原路撤退回来。**唯一兜底 = 同形降速重试**（斜插 0.20/0.10，腕轴超限再 0.10/0.04）。
- **删除**：staging PTP 绕角（L 形矩形折线）及其 5 候选×滚转扫描、`StagingCandidateSelector` 纯核与 `staging.*` GPL 参数族（seeds/wrist_weight/roll_penalty/top_n）、`staging_ik_env_` 环境池、`makeStagingSequence/Task`、planToPregrasp 的 STAGING 多级兜底链（LIN 直连+滚转扫描对 STAGING 档不再触发）；`kQuickIkProbeTimeoutS` 迁 motion.hpp（选果预检仍用）。预览与执行同形（都斜直线）。
- 语义：不满足（无当前 TCP / 跳长超限 / 扫掠触果囊）即失败收口 `skipped_unreachable`，不再绕行。`approach_staging_standoff_m` 保留（斜直线落点定义）。
- 门：colcon build/test 绿、r0_gate 绿。**改动前基线已留档**（M1r 19/20、M3r 25/29、M4r 7/30，campaign/analysis/injection/），改动后须复跑注入矩阵对照。

### 09-23 续二：轨迹定型三版 v3「PTP 关节空间+垂直进入」，Z 滚转放开 ±60，仿真改解析迭代（用户裁定）

- v2 斜直线-only 全链证伪（M2 2/30、M3 1/30、M4 0/30——keep-roll 斜插 LIN 腕限位大面积失败）。
- v3 定型：PTP（Pilz 关节空间）→ 轴上 staging（alignFrameZ + 滚转梯子 {0,±30°,±60°}，每档当前种子+随机重启≤3）→ 沿轴垂直 LIN 预抓取 → 套入/撤退原路。无候选 zoo、无降速档。
- Z 滚转分析修订：功能几何冗余（回转对称筒+圆环刀口）但 **IK 解多样性必要**（解析 0° 档 25% vs −60° 档 55%，用户放开正确）。
- 解析评估工具 `campaign/.../analytic_roll_ladder.py`（/compute_ik 双路点、秒级、不起周期）：网格 18/20、typical 随机 63%（单种子下界）。仿真链太慢的裁定下，参数迭代走解析，最终率一轮短 mock 确认。

### 09-23 续三：v4 入冠垂直段（用户裁定）

- 接近序列改三段：PTP 关节空间→中段点正下方（树冠外）→ 世界垂直 LIN 入冠 0.05（伸进果树里）→ 沿轴 LIN 0.05 对轴到预抓取。斜袋时进冠铅垂、末段顺轴。
- 参数：`approach_final_axial_m` 0.05 / `approach_canopy_entry_m` 0.05（替 `approach_staging_standoff_m`）。
- 解析：57/80=71.3% vs v3 56/80=70%，入冠段零可达性代价。门：build/test/r0_gate 绿。

### 09-23 续五：v4d 平滑连续（sequence 混合），零成功率损失定版

- 冠内两段 LIN（垂直入冠→沿轴对轴）改走 Pilz sequence（`/plan_sequence_path`，blend_radius 0.02）：corner 圆滑、速度连续一条轨迹到底；PTP 段保留冠口停点（树冠外安全观察位）；sequence 失败自动降级回未混合 MTC LIN（形状同）。
- 两个根因修复：①`planAndMaybeExecute` 收尾 reset `active_task_` 后再取 `getRobotModel()` 的空指针 SIGSEGV（模型改在任务移交前取）；②**Pilz sequence 的逐 item 规划走 move_group 默认管线、不读 item.pipeline_id**——默认 ompl 时 LIN 退化为 OMPL 关节规划、端点不接、混合必拒；默认管线切 pilz（moveit.launch.py，MTC 各 stage 自带 pipeline 不受影响）。
- 终验：网格 **18/20 与 v4c 持平**（失配同两例：near_horizontal 算法包络边缘例护栏正确拒 + deep_left 边界抖动），sequence 走廊 13 次全走通零降级，91 段轨迹全部执行完成。

### 09-23 续六：重构批次0/1（P0 计划获批执行）

- 批次0：baseline_inventory.json 快照（385 参数/54 接口/1324 函数）+ schema 守卫入 colcon；harvester 209 重封。
- 批次1：**F1 收口**——StrictMaskGate 精确 ns 查表改最近邻容差配对（`capture.mask_stamp_tolerance_s` 0.08s，<掩膜流周期之半；stereo 13.6fps 前端解锁路径打通），5 单测覆盖窗内/窗外/零容差/多候选；D3 throttle 单位误解 4 处（1000.0→1.0s，rclpy 实为秒）；D2 未 configure 即 destroy 的 worker 空引用守卫。D1 BeginScene 线程项并入批次6 调度重构。

### 09-23 续七：重构批次2（轴向预算 G1 收口）

- **维度拆分**：refine.py sig_p 不再双喂——横向 6mm 下限只进径向；袋颈轴向误差独立（多视颈点轴向坐标 MAD + `axial_neck_floor_m` 3mm 下限）。
- **常数治理**：七项预算常数迁 target_reconstruction.yaml `tool.budget.*` 带 provenance（M0 台架前保守包络）；`axial_safety_margin` 默认 0（与 fruit_safety_clearance 历史重复扣减，G1 根因③），旧值 0.004 可恢复对拍。
- **新语义**：零感知误差轴向余量 +1mm（旧恒负为缺陷行为，契约测试重写）；默认常数态（颈下限 3mm）余量 −2mm → `axial_budget_structurally_unsatisfiable` 待 M0 标定（结构性旗=工具侧固定常数吞捕获带，与"袋不好"分辨）。
- 反例测试落位：增大任一误差项不得提高许可。

### 09-23 续八：重构批次3（剪切证据链 G2/G3 软件闭环）

- **ToolState.msg**：刀/保持/载荷三轴（各自 UNKNOWN 优先），/peach_arm/tool_state latched on-change；manifest 55 接口。
- **新事件门（§12.2）**：ToolActuator.confirmFeedback 须见**本命令后闭合新上升沿**（早已卡高→`no_new_di_edge` 拒）——单测抓出并修复了"沿门只在回调层、执行器本体可被 hardware_ok=true 直通"的语义洞。
- **保守收口（L89）**：stageVerifyCut 有界等沿 2.0s（`tool.feedback_timeout_s` GPL），超时 CUT_UNCONFIRMED——不撤退/不重发/不 resume，FAILED 待人工；确认后 CK_CUT_CONFIRMED+LEVEL_CUT_CONFIRMED 可达（此前结构性不可达）。
- **G3 最小闭环**：stageReleasePayload 在 harvest_stow 后开刀释放（commandToolOpen，TOOL 级授权同 close）；载荷真证据（光电/力）M0 后接，现 PAYLOAD_UNKNOWN 诚实态。
- tool_actuator.cpp 移入 _core 纯核库；6 例 gtest（沿确认/卡高/开沿三轴/双门幂等/复位归 UNKNOWN/双证据）。

### 09-23 续九：重构批次4（分级许可 P0-2 + A-P3-2 闭环）

- **分级档位门**：authorizeStage 令牌路径 CONTACT 看 radial_margin>0、TOOL 看 axial_margin>0+pregrasp_verified；快照路径 refined 按 sleeve/cut 能力三态分档（unrefined 交袋径门）。单条 allowed 降汇总位（msg 注释同轮）。
- **unrefined 袋径门**：promoteUnrefinedGeometry 拒绝袋径>工具内径的自授权（d_inner 档案 launch 注入：hollow 0.104/adaptive 0.116，`tool_profile_d_inner_params` 单源）——A-P3-2（unrefined 链全旁路预算）闭环。
- **假门清理**：domain/budget ready_ok 恒 True → d95 证据派生。
- **e2e 证据**：网格 18/20 保持；deny 例臂侧真实拦截（`快照套入能力非 VALID`→FAILED@CONTACT、completion=2 未进套入）；tool 开启负路径 SetIO 失败→TOOL_STATE_UNKNOWN（stages 无 retreat/stow，grasped=false，保持待人工）。DI 超时路径（CUT_UNCONFIRMED）因 mock set_io 拒收不可达，单测层覆盖。

### 09-23 续十：重构批次5（套入实测行程判据）

- `waitImuFollowTravel` 计时死等 → **FK 实测行程**：wait 起点 TCP 沿锁定轴向位移投影（推进正向/撤退反向），容差 5mm；FK 不可用回退 `/imu_follow/insert_progress`（imu_follow 同轮发布 `_insert_travel` 目标积分——弱一等，注明）；时间只作截止（名义+2s，§11.3 到位证明只能是实测）；停滞窗 3s 内增益 <1mm → UNKNOWN 收口。
- `insert_progress.hpp` 纯核（DONE/CONTINUE/STALLED/DEADLINE 四态）+ 5 gtest（含"行程更小绝不得 DONE"反例）。
- 撤退路径同判据（原路收回参考=反向投影归零）。

### 09-23 续十一：重构批次6（变体A 一期 + 先导实验前提勘定）

- **一期（默认关）**：`peach_supervisor.reconstruct_in_trajectory` true 时 DISPATCH 启动 Build 进 COLLECTING 后不再阻塞 `_wait_build_after_observe`，立即派 FULL（FULL goal 既有 skip_observation=true）——重建与接近并行；臂侧 FinalizeAndValidate 有界等精化兜底；周期收口取消并等 Build 结束（单槽约束）。yaml 键/规则/参数三源同轮。
- **先导 bag 实验勘定**：本地全部 session bag 无深度/stereo 话题（近期为无相机注入轮）——temporal_k 运动鬼影定量实验**数据前提缺失**，待 S3 真相机轮（含运动+深度）产出后执行，变体B（连续采集）生死据此判定。
- **二期记档**：冠口检查点分段（MovePregrasp 拆 MoveToCanopy→Finalize→corridor，采集点=冠口静止位）为变体A 完整形态，待一期真相机数据后实施。

### 09-23 续十二：重构批次7（收口批）

- deactivate 收口四线程（worker/action/survey/move_to）无限 join → **2s 有界**（pthread_timedjoin_np，超时 detach + WARN）——TEM 风暴等挂死不再卡 lifecycle。
- ID-1：空 request_id 兜底 'harvest' → `auto_<时间戳>`（账本目录不复用，断点恢复误跳过收口）。
- **bond 勘定**：launch 实测 `缺 ros-jazzy-bondpy` WARN（此前"已装"记录不实）——默认维持 0.0；默认 8.0 实测会因 Python 节点无心跳拆栈。**前置：sudo apt install ros-jazzy-bondpy 后 bond_timeout:=8.0 即开**。
- P0 八批次全数收口；先导 bag 实验（变体B 判定）与 I6 现场步、M0 台架标定为待现场窗口项（均已记前置条件）。

### 09-24 文档同步：回放塔 v4 重封口径追平（43be561 欠账）

- 43be561（09-23）重封 `replay_baselines.json`：v4 三路点构造（staging 纯轴向 0.13 → mid−ẑ·0.05 世界垂直，replay_oracle 同轮换公式）；stratified `lin_chord_fail` 49→51、random_100 8→6；json 增 `revised` 字段。当时未留 testing-log 条目，本条补记。
- ddd9136 标**历史 v1 口径**的两脚本是 `scripts/analytic_constraints.py` 与 `scripts/sim_approach_probe.py`（`analyze_approach_envelope.py` 仍现行，v4 docstring）；testing.md 回放塔节不再写「与 scripts 逐数一致」，逐数权威=基线 json 的 provenance/reverified/revised 链。
- 同轮：architecture/io/testing 追平批次 4–7（档位门/行程判据/reconstruct_in_trajectory/deactivate 2s/ID-1 兜底）与 v4d（sequence blend、move_group 默认 pilz）措辞；护栏现行值统一 2.6/0.32/0.12。

### 09-24 E2E 阶段一：地基与缓解（方案 reports/2026-09-24-e2e-test-plan v2.2）

- **TEM 关闭（决策 0029①）**：`aubo_e5_moveit_config` 双 `controllers{,_mock}.yaml` 声明 `trajectory_execution.execution_duration_monitoring: false`。参数名核自 Jazzy `trajectory_execution_manager.hpp:336`（与 `allowed_execution_duration_scaling` 同命名空间）；UR Driver 主流（monitoring 与透传控制器打架）。A-P3-1 stop 风暴的**缓解**（根因另行跟）；超时防线仍是 0027 `boundedExecute`（90s）。活文档同轮：architecture §执行有界等待+决策表 0029、testing.md 门描述、AGENTS.md 执行监测行。
- **r0_gate 清单补齐（②）**：全量 diff 各包 test/ 后 +13 文件（vision `test_vision_{gating,ransac,sam_fallback,scene_params}`/`test_reconstruction_mask_gate_f1`/harvester `test_yaml_params`/common `test_lifecycle`/observability `test_{params,pipeline}`/system_tests `test_{perf_baseline,baseline_inventory,replay_approach}`/bringup `test_params`+新增 peach_sim 组 `test_{params,scene}`、PYTHONPATH 补 `src/peach_sim`）。**有据排除 3 个非零 ROS 文件**（实跑门暴露）：`test_vision_estimator`（pipeline→msg_builders→geometry_msgs）、`test_tcp_trajectory`（geometry_msgs 直 import）、`test_hotpaths`（imports rclpy/rosbag2）——仍由 colcon test 覆盖。**testing-log:564 漂移记档勘误**：`test_reconstruction_decision_validity` 不在门内是**正确**行为（其 docstring 自述 import 链含 ROS msg 属 colcon 层），:564 对该文件的暗示系误报。补齐后门全绿（最后组 29 过/2 过/21 过 + manifest ok 55+5）。
- **launch_testing 隔离域（③）**：Jazzy `launch_testing_ament_cmake` 无 `add_ros_isolated_launch_test`（读 `/opt/ros/jazzy/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake` 证实：宏参数只有 TARGET/TIMEOUT/PYTHON_EXECUTABLE/ARGS/LABELS，未解析参数透传 `ament_add_test`）——以 `add_launch_test(... ENV "ROS_DOMAIN_ID=89" "ROS_LOCALHOST_ONLY=1")` 等效实现（域 89 避开战役 61 与并发会话曾用 33/46/77）。
- **bondpy 勘定反转（④）**：dpkg `ros-jazzy-bondpy 4.2.0-1noble.20260902` 在装，且 `source /opt/ros/jazzy/setup.bash && python3 -c "import bondpy"` **成功**——批次7「本机缺 bondpy」为误报（当时测试未 source overlay）。`bond_timeout` launch 默认仍 0.0；开 nav2_lm Python 侧进程死检只差置 8.0（E2E 各阶段不依赖）。
- **S0 注入契约尖峰四问定论（⑤，E1 注入器实施依据）**：① 感知静默性=**成立**——观测发布纯帧驱动（`scene_perception_node.py` `_on_rgbd`(message_filters 同步)→`_process_rgbd`→`_publish_frame`→`_publish_target_observations`，无 timer），相机关时感知在 `/peach/perception/target_observations` 静默→注入器单发布者方案成立，无需降级路径。② 锁判定比方案预期更宽松：`_lock_set_ready`（executor_node.py:1659-1667）只查 `epoch 相等∧epoch>0∧target_set_locked`，**不查 harvest_run_id/selected_target_id**；`_on_obs`(:610-621) 无过滤缓存任意到达数组——注入器只需从 latched `/peach_supervisor/state` 对齐 `scene_epoch`。③ C22 注入手段=`moveit_enabled:=false` 变体栈：`initializeMoveIt`（manipulation_skills_node.cpp:374-380）MGI 构造 wait_for_servers 有界 5s（A8 注释：服务器缺席时规划调用**快速失败并报错**），栈存活、Survey 必败→整批 INTERRUPTED 不发 BeginScene。④ C26–C28 断言口径：派发前 `_wait_target_in_locked_set(target_id, 2.5s)`（executor_node.py:1133）不在锁定集→`observe_failed: target_not_in_locked_set` 码 SKIPPED_QUALITY；臂侧 `onTargets`（manipulation_skills_node.cpp:808+）`observed = OBSERVED ∧ candidate≠REJECT ∧ ¬anchor_from_memory` + `safety_gate` 观测超龄判定（记忆锚点可用但不刷新新鲜度）——帧级闪烁 vs 真丢失的判定锚点齐备。
- **验证**：`colcon build`（aubo_e5_moveit_config+peach_system_tests）绿；`bash scripts/r0_gate.sh` 绿。**遗留**：`test_mock_launch` 隔离域复跑待机器空闲（本时段并发会话/外项目栈连续占用——peach_sim orchard 后 panda_sort_gazebo，按等位不抢纪律未清；空闲后 `colcon test --packages-select peach_system_tests --ctest-args -R test_mock_launch` 复核）。M1–M5 注入矩阵在 TEM off 后的重定基线属阶段二，未在本轮。

### 09-24 E2E 阶段二（免起栈部分）：案册注册表 + 互证脚本 + 边界例裁定 + 首份 C1 产物

- **corpora.yaml 案册注册表**（`campaign/20260922_dual_tool/`）：grid（20 固定例）/field_pregrasp（targets_20260909 14 例）/stratified_200（seed 20260911）/random_100（seed 20260910）/m4_algo_30（seed 20260911 algorithm）/ladder_random_60（seed 20260922 **标注历史遗留不同源**）——seed/规模/expect 来源单源，互证同源校验依据。
- **cross_validate.py**（campaign scripts C1 薄层）：三分法对照解析先验↔实跑 jsonl，`--gate`=#3 归因清零∧#5=零；不重写 collect_round/bag_report。实跑暴露并修正一个分类假阳性：`skipped_select`（资格门预期跳过）单列「门内一致」判定，不算 #5。
- **首份互证产物** `analysis/injection/cross_validation_20260924.md`（09-23 两轮 grid 数据 vs 解析梯子 seed 20260922）：20 例= #1×13 / #3×3（**全部归因关闭**：near_horizontal=护栏正确拒、bag_d100_denied=梯子不建模袋径×D_inner 决策门（设计内拦截）、lab_oos=any-roll 口径差）/ #4×2（deep_left expect 过期、far_no_ik 预期 skip_ik）/ #5×0 / 资格跳过×2——**门 ✅ 过**。互证法首次端到端运转即把两个人工定性项制度化为产出（方案 §4.3 预演结论兑现）。
- **边界例裁定落地（夹具+词表）**：`deep_left_low_axis` expect succeed→**skip_cartesian**（双败一致，sleeve_no_cartesian 码两轮一致）；`near_horizontal_1021_1` expect succeed→**deny_guardrail**（新词表档：MTC short-path 护栏拒，类型化判据=failure_code SLEEVE_PLAN_FAILED=5，不串匹配 reason）——sim_field_targets matched 分支、夹具头注释、test_constraint_grid 词表断言三处同轮；夹具 4 测绿。M1 分母随裁定更新（20 例口径不变，期望分类重封）。
- **待机器空闲（panda_sort_gazebo 外项目栈仍占，等位不抢）**：阶段一遗留 `colcon test --packages-select peach_system_tests --ctest-args -R test_mock_launch`；阶段二 B 段=M1 网格复跑（TEM off+新词表，预期 20/20）→ M2–M4 → cross_validate 复收口 + 基线重封。

### 09-24 E2E 阶段二 B/C 段：M1–M4 复跑（TEM off 栈）+ 互证复收口 + 基线重封

- **阶段一遗留闭环**：机器空闲（panda_sort_gazebo 退场）后 `test_mock_launch` 隔离域（ROS_DOMAIN_ID=89）完整绿跑——rc=0、34 测 0 失败、收尾 pgrep 无残留。阶段一验收门全闭。
- **可视化**：战役栈 `launch_stack.sh` 的 `DISPLAY=:0` 假定与本机 X server（`:1`）不符——栈内 RViz 曾静默夭折；按用户要求开观察面：`moveit_campaign.rviz` 于 `:1`（域 61）+ 8090 过程页（HTTP 200）。**勘定：战役脚本 DISPLAY 假定需随会话核**（未改脚本，运行时覆盖）。
- **M1 三轮（TEM off + deny_guardrail 词表）**：19/20、19/20、19/20——**失配例逐轮漂移**：R1 tilt_1639_1（折线↔兜底绕行翻转，绕行后返程行程 7.10 rad 超 6 rad 门，code=8）、R2/R3 deep_left_low_axis（sleeve 笛卡尔可达性随 IK 种子翻转：09-23 两轮+R1 不可达 / R2 R3 可达，同栈内翻转=非确定性实证）。near_horizontal_1021_1 新裁定三轮全 matched（deny_guardrail 判据=failure_code 5 稳定）。**TEM off 无不良**：无 hang（历史 tilt 300s hang 案干净收口）、失败 100% 有码。
- **M2/M3/M4（seed 20260922 与梯子同源——corpora.yaml 已修正 m2/m3/m4 条目）**：M2 25/30=83%（门 95% 未过；3 例新失败=护栏/入冠 LIN/对轴 LIN 规划边界，1 例翻好）、M3 20/30=67%（门 90% 未过；4 例返程行程门 code=8 completion=6 + 6 例接近段护栏/规划失败）、M4 11/30（压测升，护栏拒主导+滚转梯子 IK 无解 4 例）。全部失败有码、零挂起、绕行比 0.23–0.40 远低于 1.70。
- **互证复收口门 ✅**（cross_validation_20260924_rerun.md，50 例=M1 最好轮+M3 对梯子先验）：#1×27 / #2×6 / #3×11 **全部族级归因**（F1 护栏预算 6 例、F2 返程行程门 4+tilt、F3 决策/滚转口径 2）/ #4×3 / #5×0 / 资格跳过×2 / **可达翻转×1（新判定档：skip 期望案当轮变可达=expect 模型缺口，非系统失败）**。结论：全部分歧可归因到解析先验职责外三族，无未解释失败。
- **基线重封**：`m1_m5_report_20260924.md`（M1 95%→门 100% 未过、M2 83%、M3 67%；无码 0/挂起 0 过）；m*_*.log 覆写为今日轮（旧版在 git 历史）。
- **门语义遗留（待用户裁定，未放宽任何门）**：tilt/deep_left 与 M2/M3 边界族为逐轮翻转非确定性案——单轮二值门需要口径修订（双分支 expect / 多数轮 / N 轮统计），候选已在报告与 cross_validation 产物记档。

### 09-24 勘正：阶段一 bondpy 结论不完整（审计 F4 采纳）

- 本文件 09-24 阶段一条目④「裸 `import bondpy` 成功→只差置 bond_timeout:=8.0」**不成立**：包 `__init__` 为 0 字节，守卫实际用的 `from bondpy import Bond`（`lifecycle.py:39`）仍 ImportError——**置 8.0 会因 Python 三节点无心跳被 nav2_lm 拆栈，勿照做**。裸 import 成功只是误导性部分证据。真修=守卫改 `from bondpy.bondpy import Bond`（实测可用）后再置 8.0。决策 0029④ 已同轮改为修正版口径。

### 09-24 E2E 阶段二收尾+阶段三 kickoff：M1 双分支词表 + E1 注入链首例打通

- **M1 双分支词表（方案 A 落地）**：`flaky_cartesian`（deep_left：不可达∧sleeve_no_cartesian 或 可达∧执行成功）/`flaky_transit`（tilt：全程成功 或 RETREAT_FAILED=8∧completion≥6 有码收工）——夹具/sim matched/schema 测三处同轮；`test_succeed_entries_are_spread` 守卫改按执行族计（succeed∪flaky_*∪deny_guardrail，唯一点 6/x 跨 0.311 守卫保持）。**验证轮结论**：马拉松栈（连跑 7 轮）8/20 塌方=返程拒把臂留在非拍照位→后续级联（rand_03 级联族实证）；净栈重启 17/20——**travel_min/travel_max/info_length 也是翻转族**（09-22 日志早记 travel 类兜底）。双分支不再扩：更宽的抖动族属门语义裁定（推荐 N 轮统计口径），今日 5 轮 M1 证据链完整（19/19/19/8[劣化]/17）。
- **E1 注入链首例打通（阶段三）**：`perception_sim` 注入器（S0 契约：单发布者/epoch 对齐/两段式锁定/SetEnables）+ `test_e1_supervisor_chain.py`（launch_testing，域 90，CMake ENV 隔离）。**C01 SURVEY_ONLY 全链 PASS 且稳定复现**（RunHarvest→Survey→Begin→注入进度→锁定→COMPLETED 结算→账本 claimed 校验→零 RUNNING 派发）。迭代勘定：①注入器状态轨迹订阅漏接线（首版 states 恒空）；②**Survey=TRANSIT 类运动需 execution 使能**（SetEnables 单旋钮同时置调度 execution_enabled——操作台契约实测确认）；③brain 激活即装 YOLO/SAM，自动化直发首批撞模型加载（稳定窗 12s 缓解）。
- **F-E1-1（新缺陷，exit 门红）**：跑过批次后 `peach_arm` 停栈 **SIGABRT(-6)**——mock_launch 空跑不复现；与 09-24 审计 F3（批次7 detach 后 releaseResources UAF 窗）同族嫌疑，待对账。
- **F-E1-2（语义发现，C02 转 skip 待裁定）**：批中 param 关 `execution_enabled` **不拦派发**（受理期开→Begin 后、锁定前关→仍 RUNNING 派发）。EXECUTION_DISABLED-at-SELECT 的真实可达路径待产品裁定（受理期即关会挡 Survey；或 enables 应改选择期活查）。
- **observability 停栈挂死再现**：战役栈拆除后 observability 孤儿进程卡 SIGTERM 窗不退（需 kill -9）——09-22 起根因未明案例再现一次，E1 预检两次被它挡。
- 机器状态：战役栈/E1 栈均拆净（pgrep 复核无残留）。

### 09-24 方案校对 v2.3 + E1 C03 打通：注入协议五契约钉死

- **方案校对（reports/2026-09-24-e2e-test-plan v2.2→v2.3）**：bondpy 口径勘正（置 8.0 会拆栈，真修=from bondpy.bondpy import Bond）；隔离宏勘正（Jazzy 无 add_ros_isolated_launch_test，ENV 等效实现）；S0 标已定论；附录 A 补新词表与使能前置；新增 §1.4 执行状态节（各阶段快照+待裁定两项）；风险表 +2（bondpy/抖动族宽于预期）。
- **C03 PREGRASP 停驻 PASS（五次迭代，每轮钉死一条注入契约）**：① Survey=TRANSIT 需 execution 使能（SetEnables 单旋钮，前轮已勘）；② 相机关时臂侧取相机位姿需 **camera_link→camera_depth_optical_frame 静态 TF 桥**（sim_field_targets 同款，注入器已补——缺它 reason=无法取得当前相机位姿）；③ **PREGRASP 属接触规划类还需 grasp 使能**（只开 execution 会「未执行接触动作」空成功 completion=0）；④ 合成几何会撞 MTC short-path 护栏——注入目标必须用 **grid 夹具 known-good 例**（typical_1757）；⑤ **袋方向约定：bag_bottom=entry、bag_neck=entry+axis·length**（sim_field_targets.load_grid_cases 同源；entry 在袋中段则工具起点压果胶囊，护栏间隙 -0.1）。
- **E1 一期进度**：C01 PASS（稳定）+ C03 PASS（停驻语义全断言：RECOVERY_REQUIRED(7) 保持 5s 无 ACK 不回 DISCOVERY、completion=2、无 cut 段、账本对齐）；C02 skip（F-E1-2 待裁定）；exit_codes 红=F-E1-1（peach_arm 批次后停栈 SIGABRT，真缺陷待修）。C04/C05/C10–C13 待做（注入器契约已齐，后续用例纯增量）。
