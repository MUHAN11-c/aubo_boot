# peach 核心目的差距分析与路线图：感知—剪切—抓取 室外套袋水果

- **日期**：2026-09-23
- **性质**：只读分析 + 路线图。**本轮不改任何源码、不动 `docs/` 三份活文档**（无源码改动即无同轮文档欠账）。
- **本轮裁定（用户）**：接触许可走向 **分级许可 + 反馈闭环**（见 §4）。
- **证据口径**：全部结论均带 `文件:行号`，可直接复核；账面数字由 `tool_budget.py` 公式代入实参推出，未跑真机、未授权动臂/SetIO。

---

## 0. 结论速览

栈的骨架是完备的：看→建→动→批→监五段都在，双工具档案、授权矩阵、安全/质量/轨迹四道门、会话 bag 与账本齐全（`docs/architecture.md`）。**缺的不是包，是「剪切抓取」这条主线上四段断点**——任何一段不补，FULL 都走不到 `SUCCEEDED`，产品就永久停在 `PREGRASP_ONLY`：

| # | 断点 | 侧 | 一句话 |
|---|------|----|--------|
| **G1** | **许可账面恒拒** | 感知 | 轴向剪切预算在**零感知误差**下也是负的（最好 −3 mm，实作 −9 mm）→ `cut_capability` 恒 `INVALID` → `allowed` 恒 `false` → 套入/剪切永不授权 |
| **G2** | **剪切无证据** | 执行 | 切断确认 `confirmFeedback` 预留未接线，`ctx.cut_confirmed` 硬编码 `false` → `harvestConfirmed` 恒 `false` → 即便真剪了也判 `CUT_FEEDBACK_TIMEOUT` |
| **G3** | **抓取无保持、无投放** | 执行 | 工具 IO 只有一条 `close`，无 grip/hold/release；「放入收集箱」只是回 `harvest_stow`，剪断后的果子既没有保持动作也没有释放动作 |
| **G4** | **果柄剪切点是代理不是感知** | 感知 | 真实果柄/枝条剪切位置**既无检测器也无消息字段**，现在只有「袋口轴向站位」代理 |

另外三处会静默放行的**假门/旁路**（安全相关，独立于上面四条，见 §5）：`domain/budget.py` 两处 TODO 占位恒放行、`skip_reconstruction` 链无臂侧工具预算门、单周期无周期级预算。

> 现在之所以「能跑通」，靠的是 `execute_pregrasp_only:true` + `tool.enabled=false` 把 G1–G3 全部绕开（只到预抓取、不套入、不 SetIO）。这是**回避**不是**解决**：把档位一放开，四处断点立刻全部命中。

---

## 1. 核心目的拆解 × 现状矩阵

核心目的「感知—剪切—抓取 室外套袋水果」拆成六段。等级：**OK**=可用 / **代理**=用近似量顶替、有系统性偏差 / **空**=没有 / **恒拒**=有实现但账面永远不过。

| 环节 | 需要什么 | 现状 | 证据 | 等级 |
|------|----------|------|------|------|
| ① 套袋果检测 | 袋/果分离、实例 | YOLO `peach_bag`/`peach_nobag` 两类 + MobileSAM | `inference.py:51-166,169-268`；`pipeline.py:313` | OK |
| ② 袋型与姿态 | 袋轴/袋底/袋颈/袋径 | 圆柱 RANSAC 定轴 + 半径剖面辨口底 + 2D/3D 融合 | `pose_pipelines.py:226,311-444`；`bag_landmarks.py:261` | OK（有塌缩，见 §6-P1） |
| ③ **果柄/枝条剪切点** | 柄点 + 枝轴，给出**可剪的实体位置** | **只有「袋口/检测框轴向极限」代理**，无检测器、无 IDL 字段 | `refine.py:932-971`（`_cut_station`）、`TargetModel.msg:20-22`、`GraspDecision.msg` `cut_pose #（袋口/框极限）` | **空** |
| ④ 多视重建与许可 | 袋模型 + 套入/剪切许可 | TSDF+ICP+多视角 Huber 融合齐备；许可公式账面恒负 | `refine.py:1003-1196`；`tool_budget.py:47-80` | 重建 OK / 许可**恒拒** |
| ⑤ 套入（包覆抓取） | 握住袋子/果子 | 只有「筒体套入」包覆，**无夹持闭合与保持力** | `stages.cpp:1106-1197`；`motion.cpp:225-256` 单条 `close` | **代理** |
| ⑥ 剪切 | 剪断果柄 + **证明剪断** | `SetIO{fun=3,pin=0,state=1}` 一次 close；切断确认硬编码 false | `tool_actuator.cpp:26-48,50-62`；`stages.cpp:1199-1252` | 剪令发出 / **无证据** |
| ⑦ 投放收集箱 | 释放入箱 | 只回 `harvest_stow`，**无 open/投放动作** | `stages.cpp:1305` | **空** |
| ⑧ 户外鲁棒 | 光照/风动/遮挡/深度缺失 | 有**检测与阻断**（LightingMeter、swing、五类遮挡、valid_depth），**无补偿与动作响应** | `stream_metrics.py:188`；`identity.py:768-830`；`bag_landmarks.py:63-85` | 半个 |

---

## 2. G1 — 剪切预算结构性为负（最硬的阻断）

### 2.1 账面推演

`tool_budget.py:74-75`：

```
axial_margin = blade_capture_half_width − axial_safety_margin − axial_error95
axial_error95 = neck_position95 + blade_plane_cal95 + robot_axial95 + target_motion95
```

代入 `tool_budget.py:10-24` 默认常数与 `refine.py:1065-1072,1117-1118` 实参：

| 项 | 值 | 来源 |
|----|----|------|
| `blade_capture_half_width` | **+8 mm** | `tool_budget.py:19`，源码未写理由 |
| `axial_safety_margin` | **−4 mm** | `tool_budget.py:20`，语义未写 |
| `blade_plane_calibration_error95` | −2 mm | `tool_budget.py:21` |
| `robot_axial_error95` | −2 mm | `tool_budget.py:22` |
| `target_motion95` | −3 mm | `tool_budget.py:23`，写死（风动不回灌） |
| `neck_position95` = `sig_p` | **−6 mm 起** | `refine.py:1070`：`max(view_sigma, MAD(底), MAD(颈), 0.006)` —— **有 6 mm 下限** |

- **零感知误差（`sig_p` 理论取 0）**：8 − 4 − 7 = **−3 mm**。→ 结构性为负，与感知好坏无关。
- **实作值（`sig_p` 走 6 mm 下限）**：8 − 4 − 13 = **−9 mm**。
- 因此 `cut_ok` 恒 `False`（`tool_budget.py:80`）→ `cut_capability` 恒 `INVALID` → `allowed = geometry ∧ sleeve ∧ cut`（`model_contract.py:47-53`）恒 `False`。
- `GraspDecision.msg` 自己的注释就写着「`cut_capability` 默认档案轴向余量常负 → INVALID」，`cross_field.py:48` 也写「Auto-cut remaining infeasible is the correct F05 result」——**这不是疏忽，是账面自知的死局，只是一直没有被当 P0 处理**。

### 2.2 对比：径向是可判定的，轴向不可判定

`tool_budget.py:22-45`：`radial_available = 0.5·D_inner − 0.5·d_bag95 − 0.002`，`radial_error95 = sig_p + L·sin θ + 8 mm`。

以 `sig_p` 6 mm、`L=0.15 m`、`θ=2°`（≈5 mm）计，误差 ≈ 19 mm：

| 工具 | `D_inner` | 袋 D95 | 可用净空 | 结果 |
|------|-----------|--------|----------|------|
| hollow | 104 mm | 100 mm | 0 mm | 拒（`bag_d95_exceeds_tool`） |
| hollow | 104 mm | 80 mm | 10 mm | 拒（径向负） |
| hollow | 104 mm | 50 mm | **25 mm** | **可过** |
| adaptive | 116 mm | 80 mm | **26 mm** | **可过** |

**结论**：径向门「薄袋/大内径可过」，是可判定的；**轴向门无论给什么输入都拒**。所以问题不在袋、不在感知，在预算模型与工具工艺不匹配。

### 2.3 根因三层（按修的顺序）

1. **`sig_p` 6 mm 下限被当成「袋颈 95% 轴向误差」**（`refine.py:1118` 同一个 `sig_p` 同时喂 `center_lateral95` 与 `neck_position95`）。它是**横向定位**散布的下限，直接当**轴向**误差用，既混了维度又叠加了下限。
2. **固定误差 7 mm 与捕获带 8 mm 同量级**，且 `blade_plane_calibration_error95` / `robot_axial_error95` / `target_motion95` 三个常数**源码未写理由、无标定来源**（AGENTS 三问的「来源/原因」两项都是空的）。`blade_plane_calibration_error95` 尤其应是**逐工具标定残差**，不该是设计常数。
3. **`axial_safety_margin`(4 mm) 与 `fruit_safety_clearance`(12 mm, `tool_budget.py:80` 经 `cut_to_fruit_m` 独立把关) 语义重叠**，疑似重复扣减一次安全余量。

---

## 3. G2 / G3 / G4 — 另外三段断点

### 3.1 G2 剪切无证据链

- `tool_actuator.cpp:50-62 confirmFeedback(hardware_ok, …)` **实现好了但没人调**；`stages.cpp:1236` 直接 `ctx.cut_confirmed = false`，注释自陈「切断确认保持删除态」。
- `stages.cpp:1332` `harvestConfirmed(ctx.cut_confirmed, ctx.retreat_confirmed)`（`tool_actuator.hpp:66`：`cut_evidence && retreat_evidence`）——`tool.enabled=true` 时恒 `false`，终局必落 `FAILED/CUT_FEEDBACK_TIMEOUT`。**FULL 永远拿不到 `SUCCEEDED`，这是设计成「不许谎报成功」的保守收口，代价是也永远报不了真成功。**
- **好消息：反馈源在驱动契约里已经有了**——`aubo_msgs/msg/IOState.msg:10` `tool_io_states`（工具数字输入，2 路 pin 0..1），`aubo_io_controller` 每控制周期发 `~/io_states`。
- **红线**：`IOState.msg:7` 明说板载 DO **无状态接口**（`state` 只是命令回显）。**切断证据只能来自 `tool_io_states`，绝不能用 SetIO ACK 或 DO 回显冒充**——`stages.cpp:1229` 现在的 `CK_CUT_ACCEPTED`「SetIO 受理（≠切断）」标注是正确的，要保持。

### 3.2 G3 抓取无保持、无投放

- `motion.cpp:244-247`：`SetIO{fun=tool.io_fun, pin=tool.io_pin, state=tool.close_state}` **单条 close**，无 open、无独立夹持/松开编排。`ToolActuatorState` 里预留的 `SAFE_OR_HOLDING` 从未进入。
- 全链只有「筒体包住袋子」这一层包覆保持；剪断果柄瞬间到撤退完成之间，**没有一个动作是「抓住」**。
- 「放入收集箱」= `stages.cpp:1305 stageReturnHarvestStow` 走到 `harvest_stow` 就结束，**没有释放/投放**。即使 G1、G2 都补上，剪下来的果子也送不出筒体。

### 3.3 G4 剪切点是「袋口代理」，不是感知出来的果柄

- `refine.py:932-971 _cut_station`：`cut = bottom + length·axis` = **袋口**；docstring 明说「剪切站：袋口 / 分割贴检测框极限」「果距不足只否决 `allowed`，不把刀挪到果–颈中点」。
- 同一代理贯穿到 IDL 与执行端：`TargetModel.msg:20-22` `cut_plane_point/cut_normal`、`GraspDecision.msg` `cut_pose #（袋口/框极限）`、`publish.py:213-244` 写入、`motion.cpp:172` 把 TCP 当物理剪切点。
- 全仓唯一的「柄」概念是裸果线 `_stem_cavity_axis`（`pose_pipelines.py:1123-1176`，梗洼估**果梗方向**，不给剪切点），且 `scene_perception_node.py:113` `enable_fruit=False` → **事实死代码**。
- `peach_vegetation`（Frangi + ExG）**无果柄/枝条连接概念**，生产链零订阅；`ivg_*` 两包是遗留旁路，不辨袋/柄。
- **后果**：袋口≈袋的形变极限，不是柄。套袋桃的柄在**袋口上方的扎口处**，与袋口有系统性偏移且随风摆/纸袋形变漂移。用袋口代理剪切，配合 G1 那个「轴向捕获带 ±8 mm」的账面要求，等于**既要盲切又要切得准**。

---

## 4. 走向裁定：分级许可 + 反馈闭环（设计草案，未实施）

### 4.1 为什么现有契约已经能承载

`GraspDecision.msg` 已经有四枚能力位 `geometry_capability / pregrasp_capability / sleeve_capability / cut_capability`（`VALID/INVALID/UNKNOWN` 三态，`TargetModel.msg:56-59` 同款），`allowed` 只是它们的合取投影（`model_contract.py:47-53`）。**IDL 不用加字段**，改的是「谁看哪一枚」。

### 4.2 分级许可

| 阶段 | 今日闸门 | 改为 |
|------|----------|------|
| TRANSIT / PREGRASP | `Active ∧ robotReady ∧ !cancel ∧ execution` | 不变（`cycle.cpp:76`） |
| CONTACT（套入/包覆） | `… ∧ grasp_enabled ∧ **allowed** 复检` | `… ∧ grasp_enabled ∧ **geometry ∧ sleeve** VALID` |
| TOOL（剪切） | `… ∧ tool_enabled ∧ **allowed** 复检` | `… ∧ tool_enabled ∧ **geometry ∧ cut** VALID **∧ 剪切预备位残差过门**` |
| 汇总 `allowed` | 唯一闸门 | **保留不删**，降级为「完整接触可一次做完」的汇总位，进账本/审计/作业票 |

语义收益：**套入（包覆抓取）不再被「剪切带不可保证」一票否决**；剪切独立判、独立否决、独立收口。这正是 `capability` 三态当初留出来的缝。

风险与护栏：分级后「套入了但剪不了」是新增状态，必须有明确收口——**剪切带不可保证时，只到「剪切预备位」停住并 `SKIPPED_QUALITY` 收口**（可重派），不得空剪、不得半剪半退。

### 4.3 反馈闭环（把「剪断」变成证据）

```
SetIO(close) ──► 柜侧执行 ──► /aubo_io_controller/io_states.tool_io_states[pin]
                                        │
                                        ▼
                    ToolActuator::confirmFeedback(hardware_ok)   ← 现成的，只差接线
                                        │
                                        ▼
                    ctx.cut_confirmed ──► harvestConfirmed(cut, retreat) ──► harvest.grasped
```

- 反馈源唯一性：**只认 `tool_io_states`**（工具数字输入）。板载 DO 无状态接口，SetIO ACK / DO 回显一律不得作切断证据。
- 超时/证据不足：保持现有保守收口 `FAILED/CUT_FEEDBACK_TIMEOUT`，**不宣称 `harvest.grasped`**；保持抓紧、退到安全位、等人工 ACK。
- 双证据原则保持不变（`tool_actuator.hpp:66`）：切断证据 **∧** 撤退证据。

### 4.4 必须靠标定才能定的量（不要拍脑袋填）

| 常数 | 现值 | 怎么得来 |
|------|------|----------|
| `blade_capture_half_width` | 8 mm | **捕获率曲线**：干切标准柄/纸带，扫轴向偏移 ±20 mm，记录切断成功率，取 95% 捕获半宽 |
| `blade_plane_calibration_error95` | 2 mm | 逐工具 TCP↔刀口面标定残差 95%（现为设计常数，应改成标定产物） |
| `robot_axial_error95` | 2 mm | 沿轴 LIN 到位重复精度实测（`scripts/trajectory_watchdog.py` 已能采） |
| `target_motion95` | 3 mm | **应动态**：由 `identity.py:768-830` 的 swing 幅度回灌（现写死） |
| `axial_safety_margin` | 4 mm | 先澄清语义：若与 `fruit_safety_clearance`(12 mm) 重复则删除一项 |
| `neck_position95` | `sig_p`（6 mm 起） | **拆维度**：横向用 `sig_p`，轴向另测（多视角袋颈沿轴散布 + 袋口形变），并去掉 6 mm 横向下限 |

**通过判据**：`capture_half_width − axial_safety_margin − axial_error95 > 0` 能在**实测常数**下对典型袋成立；不成立就说明**工艺要改**（换刀、改捕获带、或改「先夹持固定再剪」降低 `target_motion95`），而不是继续调公式。

---

## 5. 会静默放行的假门与旁路（独立安全项）

| 项 | 位置 | 问题 |
|----|------|------|
| **假门 1** | `domain/budget.py:33-38` | `TODO(W13-A 占位)`：`geometry` 恒 `VALID`——IK/碰撞可达性无独立信号源接入，等于几何能力位**从不拒** |
| **假门 2** | `domain/budget.py:47-51` | `TODO(W13-A 占位)`：`ready_ok` 恒 `True`——消费方若当校验结论用即放行 |
| **旁路 1** | `target_cache.cpp markUnrefinedHold` | `skip_reconstruction` 链**无臂侧工具预算门**，超内径袋无防线，全靠感知 SELECT 的 `tool_clearance_failed` 旗标（战役 A-P3-2） |
| **旁路 2** | 周期级 | 单周期**无周期级预算**，坏目标可拖整批（战役 A-P3-3，`rand_03` 曾 >300 s） |
| 漏杀 1 | `quality_gate.cpp:31-46` | 轴一致性 35° **只诊断不拒**，完全错轴仍可能放行 |
| 漏杀 2 | `contact_monitor.hpp` | 硬接触止损默认 `enabled=false`、阈值 0（`peach_arm.yaml:108-112`） |
| 漏杀 3 | `acm_policy.hpp` | `allowToolVersusWholeOctomap` 恒 `true`，工具撞场景地图不报警 |
| 上游 | move_group TEM | stop 事件风暴使 `execute` 永久阻塞、只能重启栈（战役 A-P3-1，MoveIt 栈内缺陷）——**真机门的现实阻断，不由本仓修复** |

---

## 6. 路线图

### P0 — 解锁主线（可在 mock + 真相机全链验证，不动真机）

| # | 做什么 | 改哪里 | 验收门 |
|---|--------|--------|--------|
| P0-1 | **轴向预算重构**：拆 `sig_p` 横向/轴向维度、去掉 6 mm 下限误用、澄清 `axial_safety_margin` 语义、五个常数迁 yaml（逐工具标定入口） | `tool_budget.py:10-24,47-80`、`refine.py:1065-1118`、`config/target_reconstruction.yaml` | 新增「**结构性不可满足**」诊断：`capture − safety − 固定误差 ≤ 0` 时置 `axial_structural` 旗 + `reason=axial_budget_structurally_unsatisfiable`，**一眼分清「袋不好」和「公式不成立」**；零误差用例账面不再恒负 |
| P0-2 | **分级许可**：CONTACT 取 `geometry∧sleeve`、TOOL 取 `geometry∧cut`+预备位残差；`allowed` 降为汇总位 | `model_contract.py:47-53`、`cycle.cpp:71-140`、`stage_denial.hpp` | 剪切带不可保证时只到剪切预备位 `SKIPPED_QUALITY` 收口，不空剪不半剪；账本/作业票三态如实 |
| P0-3 | **切断证据闭环**：订 `~/io_states` 取 `tool_io_states[pin]` 喂 `confirmFeedback`，去掉 `stages.cpp:1236` 硬编码 | `manipulation_skills_node.cpp:723` 附近、`motion.cpp:225-256`、`tool_actuator.cpp:50-62`、`stages.cpp:1199-1252` | mock 下能真实翻转 `cut_confirmed` 并出 `SUCCEEDED`；证据不足仍 `CUT_FEEDBACK_TIMEOUT`；**绝不以 ACK 冒充证据** |
| P0-4 | **grip → cut → hold → release 编排**：套入后夹持保持、剪切、撤退全程保持、`harvest_stow` 释放入箱 | `motion.cpp:225-256`、`stages.cpp:1067-1345`、`tool_actuator.*`、`arm_parameters.yaml` | 全程任一时刻果子都有保持；释放只在收集箱位且已确认切断；失败路径不丢果 |
| P0-5 | **清假门/旁路**：`geometry/ready_ok` 接真实可达信号；unrefined 链补「袋径×`D_inner`」轻量臂侧门；加周期级预算 | `domain/budget.py:33-51`、`target_cache.cpp`、`cycle_support.hpp` | `deny_decision` 用例在 unrefined 链同样被臂侧拒；单目标超预算即失败收口不拖批 |
| P0-6 | **单测补洞**：ToolActuator FSM、`protected_zones`、`cycle_context` 映射、阶段序列编排 | `peach_arm/test/` | `r0_gate.sh` 绿 + 新增覆盖编排层 |

### P1 — 户外鲁棒性（决定田间能不能用）

| # | 做什么 | 关键点 |
|---|--------|--------|
| P1-1 | **真实果柄/枝条剪切点**（对应 G4，**感知侧最高价值**） | 袋口邻域环形 ROI 内做柄-枝连接局部检测（可复用 `peach_vegetation/frangi.py` 的枝响应 + SAM point prompt）；输出**柄点 + 枝轴**进 `TargetModel` 新字段；**未检出则回退袋口且 `cut_capability=INVALID`**——不得默认当已知 |
| P1-2 | **袋长塌缩根治** | `pose_pipelines.py:127-155` 自述 56 帧 `travel` 0.024–0.040 m（隐含袋长 4–5.5 cm，物理 15 cm+）——直接污染 `travel_m`/`cut_travel_m` 的插入深度。改 SAM 掩膜锥体拟合求袋口顶点，或袋轴/端点时序 EMA |
| P1-3 | **风动回灌** | `identity.py:796` swing 现为布尔；把摆动幅度回灌 `tool_budget.target_motion95`，风大自动收紧剪切预算（配合 P0-1 的参数化） |
| P1-4 | **遮挡驱动动作** | `bag_landmarks.py:63` 五类遮挡分类现在不改变任何行为；`branch_blocked` 应触发补机位/换视角（`capture.py:79` 已有 skip 骨架） |
| P1-5 | **深度置信度利用** | stereo 的 per-pixel `confidence`（`stereo_camera_node.cpp:646-684`）接入感知深度门；`DEPTH_VOID` 目标主动补拍 |
| P1-6 | **果柄方向先验启用或删死码** | `pose_pipelines.py:878-1176` 裸果线约 300 行不可达：要么 `enable_fruit` 进 yaml 启用梗洼定向（对 P1-1 有用），要么整段删除 |

### P2 — 性能与工程健康

| # | 做什么 | 关键点 |
|---|--------|--------|
| P2-1 | 推理提速 | `yolo_half` 现默认关（`scene_perception.yaml:21`）；SAM 只对锁定集（`locked_only_segmentation` 已具备）；降低 `BoundedWorker` 丢帧（`scene_perception_node.py:594-602`，P0-4 drop 计数已暴露但仍会静默丢） |
| P2-2 | 阈值集中化 | `bag_landmarks.py:88-90`、`pose_pipelines.py:321-331,468-498`、`tool_budget.py:10-24` 全部硬编码迁 yaml |
| P2-3 | 收紧漏杀 | `quality_gate` 轴门升级软拒/降速重采；`allowToolVersusWholeOctomap` 待 self-filter 修复后收紧为 `false` |
| P2-4 | 未接线项清账 | `KIND_JOINTS`（`move_to.cpp:136`）、`MAINTENANCE`（`harvest_fsm.py:26`）、`CK_AT_STAGING`（`stages.cpp:923`）——要么接线要么删除 |
| P2-5 | `ivg_*` 两包定位 | README 明写「未接线遗留」或移出工作区，避免被当成感知链路 |

---

## 7. 建议的下一轮切入顺序

1. **P0-1 轴向预算重构 + 结构性诊断**（半天量级，纯核零 ROS，先让账面「可判定」，否则后面全白做）
2. **P0-3 切断证据闭环**（把「剪断」变成事实，是「剪切」二字的本体）
3. **P0-4 grip/cut/hold/release**（把「抓取」二字补全，与 2 同一批改 `stages.cpp` 最省）
4. **P0-2 分级许可**（依赖 1 的新能力位语义；按 §4.2 的护栏收口）
5. **P0-5/P0-6**（清假门 + 补测试，收口本轮）
6. 真机侧另需先解 **A-P3-1 move_group TEM stop 风暴**（上游 MoveIt 缺陷）与 §4.4 的**工具标定**，才谈得上放开 `tool.enabled` / `grasp.enabled` 做 FULL 真机。

> 提醒：P0 全部改动都落在 `peach_harvester`（vision）与 `peach_arm`，按 AGENTS 必须**同轮改 `docs/architecture.md` / `io.md` / `testing.md`**，并在 `docs/testing-log.md` 追加验证轮次。真机运动与 SetIO 仍须逐轮授权，硬件急停不经 ROS。
