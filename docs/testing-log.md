# 测试过程记录

本文只记真机/审查轮次，**不写现行怎么跑、不写验收口径**。流程、命名、门与命令：[testing.md](testing.md)。原始 jsonl 在工作区 `runs/`（gitignore）与 `_archive/runs/`，不要删。

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

技能方向：定位用感知入口；工具 Z 对齐感知袋轴；滚转不抄感知四元数。详见 architecture 接触段与 `alignFrameZ`。

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
