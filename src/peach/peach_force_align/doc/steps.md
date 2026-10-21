# peach_force_align 分步实施

2026-10-21。按“一步一步、每步可验收、门不过不往下走”拆。总计划见
[architecture.md](architecture.md)。

## 0. 原则

1. **先无运动，再有运动**：IDL/纯核/fake 测试通过前，不发任何 JTC goal。
2. **默认关**：`peach_arm` 的 `force_align.enabled=false`；显式打开才走力控。
3. **单写者**：ForceSeat 期间只有它写 JTC；`peach_arm`/MTC 不写。
4. **授权不绕过**：`peach_arm` CONTACT/TOOL 令牌仍是主门；ForceSeat goal 带
   `Clearance`，节点侧再查 target/valid_until。
5. **失败必停/撤**：力 cap、5°、5 mm、timeout、OPEN/stale、JTC 失败、cancel
   → 停 + retreat + `contact_recovery_required`。
6. **相对力**：绝对力未标定；5 点用相对，绝对 target ±10%。
7. **5 mm 是机械臂位移**，不是 `/force/travel`。
8. **台架并行**：五点同时 / 1.2 mm / 350 g / 5 mm 的台架不必等代码，尽早做。

## 1. 当前基线

- 传感器：`/force/points`、`/force/travel`、`/force/ad`、`/force/status`；
  6.0/370 SAT、OPEN AD≥3000；20 s web 按钮；逐点满行程已验证；五点同时未测；
  绝对力未标定；1.2 mm/350 g/5 mm 未验证。
- 手臂：`peach_arm` `ExecuteTarget`；`stageSleeveLinear` 是 MTC LIN；
  `previewFullContact` 预规划 reverse；`aubo_passthrough_trajectory_controller`
  是 FJT/JTC 单写者；CONTACT/TOOL 授权；`ContactMonitor` 腕电流默认关。
- 新包/IDL 尚未创建。

## 2. 步骤总表

| 步 | 内容 | 验收 | 门 | 估时 |
|---|---|---|---|---|
| **S0** | 拍板 5 件事 + 冻结接口/数据语义 | 决策记录；action 字段定稿 | G0 | 0.5 d |
| **S1** | `ForceSeat.action` + `peach_force_align` 骨架 + 纯核 + 单测 | `colcon build`；单测过；**无运动** | G1 | 1–2 d |
| **S2** | 节点骨架 + fake `/force/*` + fake JTC + action server | fake 下 action 成功/失败；**无真机** | G2 | 1 d |
| **S3** | JTC 20 Hz 单点流空载 10 mm（mock/真机低速） | 1–2 mm/s、20 Hz、cancel、反馈、无超调 | G3 | 1–2 d |
| **S4** | 力健康 + 接触检测 + 力触发停（不套 5 mm） | 固定板：<1 N 接触、3 帧、停、retreat | G4 | 1–2 d |
| **S5** | 受控套入 5 mm + 力 cap | 接触后 5 mm；cap 先到先停；数据 | G5 | 1–2 d |
| **S6** | 5 点评估/CoP（不旋转）+ 五点同时台架 | 五点同时数据；CoP/总力/spread | G6 | 1–2 d |
| **S7** | 绕 TCP 旋转 5°/再套/收敛（球台架） | CoP 减小；≤5°；spread 达标 | G7 | 2–3 d |
| **S8** | `peach_arm` `stageForceSeat` + action client + CONTACT + 默认关 | mock 下 CONTACT 通过；取消/失败 | G8 | 2–3 d |
| **S9** | MTC 撤退重规划 + TOOL/cut + 全 mock | 预规划 reverse 不再用；cut/retreat 通 | G9 | 1–2 d |
| **S10** | 真机台架（板/球，无果） | 5 mm/5°/力 cap/重复 10 次 | G10 | 2–3 d |
| **S11** | 失败/取消/E-stop/recovery | 每种失败必停/撤/ACK；无双写 | G11 | 1–2 d |
| **S12** | 桃/袋端到端 | 10 次；5 点/切/撤；无过压 | G12 | 3–5 d |
| **S13** | 验收/文档/参数固化/上线 | 验收清单；bag/diagnostics；回滚 | G13 | 2–3 d |

## 3. 每步细节（先写前 6 步）

### S0 拍板/冻结（0.5 d）
- 决策：独立节点 vs 链进 `peach_arm`；JTC 20 Hz vs Servo；5 mm 语义；target/max
  force；retreat 归属。
- 冻结：`ForceSeat` goal/feedback/result 字段；`Clearance`；失败码；参数名；
  `/force/*` 语义（SAT/OPEN）；5 点坐标/通道。
- 产出：本文件 + `architecture.md` + manifest 变更清单。
- G0：5 个决策有记录；接口不再改。

### S1 IDL/骨架/纯核（1–2 d）
- `peach_interfaces/action/ForceSeat.action`；`FailureCode.msg` 新码；
  `interface_manifest.yaml`；`peach_arm/doc/io.md`。
- `src/peach/peach_force_align`：CMake/package/core.hpp/core.cpp/test；
  core 只做：CoP、旋转角/轴、5 点 spread、状态机、阈值、SAT/OPEN 判定。
- 验收：`colcon build --packages-select peach_interfaces peach_force_align`；
  单测覆盖 CoP 零点/偏置、旋转符号、5°/5 mm/cap、SAT/OPEN。
- G1：**无任何 JTC 发布**；纯核可复现。

### S2 节点/fake（1 d）
- `force_seat_node.cpp`：action server；订阅 `/force/*`；TF `tcp/tool_axis`；
  JTC client 只创建不发送（或 fake）。
- fake：`/force/points`/`status` 发布器；fake JTC action server；launch。
- 验收：fake 下 goal 成功、无接触失败、OPEN 失败、cancel；日志/diagnostics。
- G2：不接真机；action 状态机正确。

### S3 JTC 20 Hz 空载（1–2 d）
- `joint_streamer`：从当前 `joint_states` 出发，沿插入轴 10 mm，1–2 mm/s，
  50 ms 单点；IK；JTC `FollowJointTrajectory`；反馈/取消。
- mock 先跑；真机空载低速（无接触）。
- 验收：20 Hz；1–2 mm/s；10 mm 误差 <1 mm；cancel <100 ms 停；无超调；
  单写者（`peach_arm` 不写）。
- G3：JTC 流安全；才允许接力。

### S4 接触/停（1–2 d）
- 力健康：`/force/status`、stale、OPEN、SAT；零位/tare。
- 接触：ΣF>0.8–1.0 N 且 3 帧；或 ≥3 点>50 g；触发后停 JTC；retreat 到 pregrasp。
- 固定板/球台架；**不套 5 mm**。
- 验收：10 次接触位置重复 <0.5 mm；无误触发；OPEN 立即停；cancel/retreat。
- G4：接触停可靠；才允许 5 mm。

### S5 5 mm 受控套入（1–2 d）
- 接触位后 +5 mm（`seat_stroke_m`，可调）；1–2 mm/s；每 50 ms 看力；
  `ΣF>max` 或单点>per_point_max 先停。
- 记录 5 点 F/AD/mm/status；SAT 正常，不判 fault。
- 验收：5 mm±0.2；cap 先到先停；无过压；retreat。
- G5：5 mm 可控；才允许评估/旋转。

### S6 5 点/CoP（1–2 d）
- 5 点同时台架（球/平板）：记录 5 点 F、CoP、总力、spread；不旋转。
- 校验：通道映射（5/1/2/3/7）、SAT/OPEN、CoP 符号；重复 10 次。
- 验收：CoP 重复性；5 点 spread 基线；与 web/手册对应。
- G6：有真实五点数据；才允许旋转。

## 4. 并行台架（不阻塞 S0–S3）

| 台架 | 目的 | 何时要 |
|---|---|---|
| 五点同时压（球/平板）20 s | CoP/基线 | S6 前 |
| 1/2/5 mm 已知位移 + 力 | 1.2 mm 预压/350 g/5 mm | S5/S7 前 |
| 5° 旋转 + 力 | 旋转方向/符号 | S7 前 |
| 砝码/测力计 | 绝对力/重复性 | S7/S10 前 |
| 断线/悬空 | OPEN | S4 前 |

## 5. 立即开始

- 先做 **S0**：确认 5 个决策（默认：独立节点+纯核、JTC 20 Hz、5 mm 接触后附加、
  target 350–370 g/±10%、retreat 由 `peach_arm` 重规划）。
- S0 过了直接做 **S1**（IDL/骨架/纯核），不接真机。
- 台架五点/5 mm 与 S0–S3 并行。
