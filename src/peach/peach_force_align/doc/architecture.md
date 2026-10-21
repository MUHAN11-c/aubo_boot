# peach_force_align 任务分析与实施计划

2026-10-21。数据侧本轮到此：`/force/points`、`/force/travel`、`/force/ad`、
`/force/status` 已稳定；6.0 mm/370 g 不再误判，raw AD 保留；20 s 手动五点每点都能
到满行程，但**五点同时**、**1.2 mm 预压/350 g/5 mm**、**绝对力**仍需台架。

本文件是后续任务的**分析/计划**，不是已实现代码。

## 0. 一句话

在 `peach_arm` 的 CONTACT 阶段，用一个新的 `ForceSeat` 动作替换现在的
`stageSleeveLinear`（MTC 固定 LIN）：从预抓取位开始，力触发接触 → 再进 5 mm
（可调）→ 停、判 5 点 → 绕 TCP 旋转 ≤5° 让 CoP 回中 → 保持/再套 → 交给现有
`stageActuateCutter`。力/CoP 只用于**相对** 5 点，绝对力按 ±10% 容差。

## 1. 现有栈的挂点

- `peach_arm`：`stages.cpp::executeCycle` 顺序
  `MTC_APPROACH_INSERT`（到预抓取）→ `stagePlanSleeveAndReverseRetreat` →
  `stageSleeveLinear`（`grasp_task::sleeveLinear`，MTC 沿轴 LIN）→
  `stageActuateCutter`（TOOL）→ `stageVerifyCut` →
  `stageExecuteReservedReverseRetreat` → `stageReturnHarvestStow`。
- `stageSleeveLinear`：CONTACT 级 `requireStageAuthority`；`grasp_task_->sleeveLinear`；
  `setContactAcm(ContactAcmStage::Sleeve)`；`startContactGuard()`；
  `contactAbortSuspected()`；`sleeve_partial`；`contact_recovery_required`。
- `stagePlanSleeveAndReverseRetreat`：`previewFullContact` 一次规划
  approach+sleeve+reverse，`CTX.sleeve_planned` 供 `sleeveLinear` 使用。
- **关键矛盾**：MTC LIN 是固定轨迹，不能“力触发停 + 5 mm + 旋转后重套”；
  `previewFullContact` 预规划的 **reverse retreat** 在力对准后失效。
- 控制器：`aubo_passthrough_trajectory_controller`（FJT action，200 Hz 更新，
  6 关节）；mock 下是 `joint_trajectory_controller`。MoveIt/MTC 与力控不能同时写 JTC。
- 授权：CONTACT 令牌（sleeve/radial_margin）；TOOL 令牌（cut/axial + pregrasp_verified）；
  `motionOutputAllowed`、`safetyReady`、`robot_status`、`grasp_enabled`、`valid_until`。
  力控不得绕过 `authorizeStage`。
- 传感器：`/force/points`（5 N）、`/force/travel`（mm，力等效）、`/force/ad`（raw）、
  `/force/status`（OK/SAT/OPEN）；6.0/370 SAT；OPEN 阈值 AD≥3000。

## 2. 目标执行流程

```
预抓取停稳（现有 stageVerifyPregrasp）
  ↓ CONTACT 授权
ForceSeat.init          传感器健康、5 点零位、pregrasp/轴/目标有效
ForceSeat.approach      沿插入轴慢进（到 contact 前），力>阈值 + 3 帧去抖 → 接触位
ForceSeat.seat          从接触位再进 seat_stroke（默认 5 mm），同时盯
                        ΣF、每点、状态；到 5 mm 或力上限先停
ForceSeat.assess        记录 5 点 F/AD/mm/status；算 CoP、总力、spread；
                        判 SAT/OPEN（6.0 是饱和，不是断线）
ForceSeat.correct       若 CoP/spread 超限：绕 TCP 旋转（≤5° 累计），
                        方向 = 让 CoP 回中；需要时在 5 mm 预算内再套
ForceSeat.hold          保持 0.2–0.5 s，确认稳定；力 cap/姿态/状态
  ↓ 返回 ForceSeat result
stageActuateCutter（现有 TOOL 令牌、SetIO、切断确认）
stageVerifyCut
retreat                  从最终位姿重新规划（不能用预规划的 reverse）
stageReturnHarvestStow
```

**5 mm 的定义**：接触后、机械臂沿插入轴**再走的位移**（可调 `seat_stroke_m`），
不是 `/force/travel` 的 mm；传感器 mm 是力/66，仅显示。5 mm 是**预算上限**，
力 cap 先到就先停。

## 3. 包/接口

### 3.1 新包
```
src/peach/peach_force_align/
  CMakeLists.txt / package.xml          # ament_cmake
  include/peach_force_align/core.hpp    # 纯核：CoP/旋转/状态机/阈值
  src/core.cpp                          # 无 ROS、无 HAL
  src/force_seat_node.cpp               # action server + 订阅 + JTC + TF
  src/joint_streamer.cpp                # JTC 20 Hz 单点流
  test/                                 # 纯核 + 节点 fake
  config/force_align.yaml
  launch/force_align.launch.py
  doc/architecture.md
```
- 纯核零 ROS；节点只 decode/调核/publish/action；与 `peach_arm` 分层一致。
- **推荐**：独立节点 + 纯核库。`peach_arm` 作为 action client；力控节点只在
  ForceSeat 期间拥有 JTC；`peach_arm` 的 MTC/MoveIt 不同时写。
- 备选：把核链进 `peach_arm`（同一进程/同一 authority），少一个 action，但
  力控/JTC 代码进 `peach_arm`，包边界差。二选一，需定。

### 3.2 IDL（`peach_interfaces/action/ForceSeat.action`）
Goal：
- `string request_id/run_id/cycle_id/target_id`
- `peach_interfaces/Clearance clearance`（CONTACT 令牌，节点侧再查 target/valid_until）
- `geometry_msgs/PoseStamped pregrasp_pose`
- `geometry_msgs/Vector3 insert_axis`（单位向量；tcp +Z/袋轴）
- `float64 planned_insert_m`（预抓取→入口/袋内的计划量，来自 ctx.travel_m）
- `float64 seat_stroke_m`（默认 0.005）
- `float64 target_total_force_n`、`max_force_n`
- `float64 max_rotation_deg`（默认 5）
- `float64 cop_tolerance_m`、`float64 force_spread_tol`
- `builtin_interfaces/Duration timeout`
- `string tool_profile_id`

Feedback：`phase`、`float64[5] force_n`、`total_force_n`、`cop_x/cop_y`、
`rotation_deg`、`stroke_m`、`uint8[5] status`、`elapsed`。

Result：`success`、`failure_code`、`reason`、`float64[5] final_force_n`、
`final_cop_x/y`、`final_rotation_deg`、`final_stroke_m`、`uint8[5] final_status`、
`float64 spread`、`uint32 frames`。

- 加 `peach_interfaces/config/interface_manifest.yaml`；`peach_arm/doc/io.md`；
  `peach_force_align/doc/architecture.md` 同轮更新。
- 失败码：新增 `FORCE_SEAT_FAILED` / `FORCE_CAP` / `FORCE_NO_CONTACT` /
  `FORCE_ROTATION_LIMIT` / `FORCE_SENSOR_INVALID`（`FailureCode.msg` + manifest）。

### 3.3 `peach_arm` 集成
- 参数 `force_align.enabled`（默认 false）、`force_align.action`、
  `force_align.timeout_s`、`force_align.seat_stroke_m`、`force_align.max_force_n`、
  `force_align.max_rotation_deg`。
- `stageSleeveLinear` 分支：
  ```cpp
  if (force_align_enabled_) return stageForceSeat(ctx);
  ```
- `stageForceSeat`：`requireStageAuthority(CONTACT)` → 组 goal（带 `ctx.clearance`、
  `ctx.refined`、`ctx.travel_m`）→ action send → feedback 落 `peach_arm.contact` 日志 →
  result；成功 `LEVEL_SLEEVE_COMPLETED`/`CK_SLEEVED`；失败按 `FailureCode`；取消走
  现有 abort + `RobotMoveStop`。
- `stagePlanSleeveAndReverseRetreat`：力控分支**跳过**（不预规划 sleeve/reverse）。
- `stageExecuteReservedReverseRetreat`：力控分支**不执行预规划 reverse**；改为
  `grasp_task_->retreat(axis, retreat_distance, true)` 从最终位姿重新规划（或
  ForceSeat result 里带 retreat；二选一，推荐 `peach_arm` 重规划，保持 MTC 撤退门）。
- `stageActuateCutter` 不变，仍走 TOOL 令牌；力控结束后 `pregrasp_verified`/轴向余量
  复检；旋转后的 cut 点仍在 TCP，5° 限内。
- `ExecuteTarget` 的 `stage_names`/`stage_durations` 加 `FORCE_SEAT`；`Result` 可加
  force 摘要（可选；不加则看 `peach_force_align` 的 diagnostics/bag）。

### 3.4 节点/控制
- JTC：`aubo_passthrough_trajectory_controller` 的 `FollowJointTrajectory` action；
  20 Hz（50 ms）单点 JointTrajectory；6 关节；`joint_states` 反馈；20 Hz 足够
  5 mm/2 mm/s（0.1 mm/步）；力环 100 Hz，命令 20 Hz。
- 需要 IK：从 `pregrasp_pose` + 轴向位移/旋转生成 TCP 位姿；用 MoveIt
  `RobotState`/IK 或 `moveit::core`；碰撞（ACM 接触阶段豁免）。
- 单写者：`peach_arm` 在 ForceSeat 期间不写 JTC；MTC 任务已结束；JTC action
  一次一个 goal；cancel 双向下发。
- 备选 `moveit_servo`：100 Hz 笛卡尔，延迟更低，但当前栈未配置；要先评估
  servo 的 collision/实时性/驱动，再决定。**默认先 JTC 20 Hz**。

## 4. 算法要点

- **接触**：`ΣF > contact_force_n`（建议 0.8–1.0 N，约 80–100 g）且连续 3 帧；
  或 ≥3 点 > 50 g；结合 `status`，SAT 不算接触；OPEN 立即失败。
- **5 mm**：接触位后沿轴 +`seat_stroke_m`；速度 1–2 mm/s；每 50 ms 看力；
  `ΣF > max_force_n` 或任一点 > `per_point_max`（建议 370 g 的 90–100%）先停。
- **CoP**：`F=Σf_i`，`p=Σ f_i r_i / F`；5 点坐标见表；`|p|>cop_tolerance` 触发旋转。
- **旋转**：绕 TCP；轴 = 插入轴 × p 的方向（或等价小角度）；角度步 0.5–2°，
  累计 ≤5°；5° 对应膜片横向约 6.3 mm（72.29 mm × tan5°）；旋转后 CoP 应减小；
  不减小则反向/停止；5 mm 预算内可再套。
- **判据**：5 点 spread（max-min / mean）< `force_spread_tol`（建议 10–15%）、
  `|p| < 2 mm`、总力在 target ±10%；否则重试/失败。
- **相对 vs 绝对**：绝对力未标定，先用相对 5 点；target_total_force 给 ±10% 容差；
  若后续砝码标定，再收紧。
- **SAT/OPEN**：6.0/370 是 SAT（有效上限），不是断线；OPEN 用 AD≥3000；
  力 cap 触发的 SAT 是正常停止，不判 sensor fault。
- **失败**：无接触（行程用完）、力 cap、5° 用完仍超差、OPEN/stale、JTC 失败、
  timeout、cancel → 停止/retreat 到 pregrasp → `contact_recovery_required` → ACK。

## 5. 安全

- 硬件急停不经 ROS；软件 stop 不称 e-stop。
- 力 cap（总/单点）、5° 累计、5 mm 预算、timeout、速度上限；
- `status` stale/OPEN/NaN、JTC 反馈丢失、TF 丢失、IK 失败、碰撞 → 立即停/retreat；
- `peach_arm` 的 CONTACT/TOOL 授权不被绕过；ForceSeat goal 带 `Clearance`，
  节点侧再查 `target_id/valid_until`；
- 取消：ForceSeat cancel + JTC cancel + `RobotMoveStop`；`contact_recovery_required`；
- 失败后禁止 resume 原轨迹；重试须重新授权。

## 6. 数据/标定依赖

| 依赖 | 现状 | 对力控的影响 |
|---|---|---|
| 5 点同时受力 | 未测到（逐点满行程） | CoP/旋转方向/收敛判据没有真实五点数据 |
| 1.2 mm 预压/350 g/5 mm | 未验证 | 5 mm 的力、力 cap、target 不确定 |
| 绝对力 | 未标定 | 只能用相对 5 点；绝对 target ±10% |
| 5° 方向/符号 | 未台架 | 旋转方向需 sphere/桃台架确认 |
| 5 mm 力 | 330 或 409 g 两解 | 决定 target/max_force |
| 断线/SAT | 已有 status | 安全判据可用 |

**台架最小集**：球/桃 + 5 点同时压 + 1/2/5 mm 已知位移 + 5° 旋转 + 力；
录 20 s（web 按钮），分析 CoP/rotation/力。

## 7. 测试/验收

- **纯核单测**：CoP/旋转符号/5°/力 cap/状态机/超时/取消。
- **节点 fake**：fake `/force/*` + fake JTC action；成功/无接触/力 cap/OPEN/取消。
- **台架**：固定球/桃；5 点同时；5 mm；5°；重复 10 次；CoP 收敛；力 spread。
- **集成**：`peach_arm` mock + ForceSeat；MTC 前后；retreat 重规划；TOOL 门；
  取消/故障/ACK。
- **真机**：预抓取→ForceSeat→cut→retreat；E-stop；果；失败注入。
- **验收**：5 点 spread ≤ 目标、CoP ≤2 mm、5° 内、5 mm 内、无力 cap 误停、
  SAT/OPEN 正确、失败必停/retreat、无 JTC 双写。

## 8. 任务分解（建议排期）

| 阶段 | 内容 | 产出 | 估时 |
|---|---|---|---|
| T0 | 包/IDL/纯核/配置/`peach_arm` 分支（默认关）/fake 测试 | 可编译、可仿真、无运动 | 1–2 天 |
| T1 | JTC 20 Hz 单点流 + 接触/5 mm/力 cap；台架固定板 | 受控套入，不带旋转 | 2–3 天 |
| T2 | CoP/旋转/5°/再套；球/桃台架；5 点同时数据 | 5 点收敛 | 2–3 天 |
| T3 | `peach_arm` CONTACT/TOOL、MTC 撤退重规划、取消/故障 | mock 全链 | 3–5 天 |
| T4 | 真机预抓取→ForceSeat→cut→retreat；果；E-stop | 真机闭环 | 3–5 天 |
| T5 | 诊断/bag/文档/验收/参数固化 | 可交付 | 2–3 天 |

关键路径：5 点同时/5 mm 台架 → 旋转方向 → JTC/authority → MTC 撤退重规划 → 真机果。

## 9. 风险

- 5 点旋转方向/符号错 → 越转越偏；台架先定符号，限 0.5–2°/步。
- 5 mm/1.2/350 两解 → 力 cap 早停或过压；先台架定位移-力。
- JTC 20 Hz + 8 点 80 ms → 接触/力 cap 延迟；降速（1–2 mm/s）+ 力 cap 提前量。
- MTC reverse 预规划失效 → 必须重规划；否则撤退撞袋/果。
- 双写 JTC → 轨迹交错；单写者 + action 串行 + cancel。
- 绝对力未标定 → 只用相对；target ±10%；不要宣称计量精度。
- 果的个体差异/袋 → 5 点力分布；先 sphere，再桃，再袋。

## 10. 开放决策（需拍板）

1. 独立节点 vs 链进 `peach_arm`（推荐独立节点 + 纯核）。
2. JTC 20 Hz vs `moveit_servo`（默认 JTC；servo 需另评）。
3. 5 mm 语义：接触后附加 5 mm（本计划）——确认。
4. target/max force：350/370/400 g 哪个作 target/cap；绝对 ±10%。
5. 旋转中心/方向：TCP、5°、轴 = 插入轴 × CoP；符号台架定。
6. 撤退：`peach_arm` 重规划 vs ForceSeat 内撤退（推荐前者）。
7. `ForceSeat` 结果是否进 `ExecuteTarget.Result`（可选）。
8. 失败码/动作名/manifest 的最终命名。

## 11. 立即下一步

1. **台架**：球/桃五点同时 + 1/2/5 mm + 5° + 20 s 记录；这是 T2 的输入。
2. **T0 骨架**：建包/IDL/纯核/`peach_arm` 分支（默认关）+ fake 测试；
   不动真机。
3. **JTC 20 Hz 空载流**：在 mock/真机低速走 10 mm，确认单点流/取消/反馈；
   再接力控。
4. 确认 §10 的 1/3/4/5/6 后，进入 T1/T2。
