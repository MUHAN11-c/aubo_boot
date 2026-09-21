# 端到端代码级审查报告（2026-09-20，HEAD=b0ef3d4）

**范围与方法**：W0–W8（主链逐函数）与 W9–W16（外围包+遗留清扫）两轮之后，本轮沿**数据与控制流的跨包接缝**做端到端静态追踪，不起任何栈/仿真。四路并行：①启动链与批次发起 ②感知→重建链 ③执行链与批次收尾 ④横切契约（IDL 字段/QoS/TF/单位/线程/参数）。所有结论带 文件:行 证据；高危项经主审二次抽验。

**与既往轮的关系**：不重复已裁定项（PF-1 推迟相机轮、mock robot_status 在线参数法、bond Python 侧待 apt、composition 平台阻断、W0–W16 已修项）。

---

## 一、总览

| 级别 | 数量 | 说明 |
|------|------|------|
| 高（会错行为/死锁/数据错） | 5 | G1–G5 |
| 中（边界脆弱/契约漂移/恢复路径受损） | 19 | M1–M19 |
| 低（卫生/文档/死管道） | 14 | 见附录 |

**验证为一致的接缝面**（简述，细节在各段）：launch 参数传递 5 条 Include 全对齐；nav2_lm 名单与四节点名一致、顺序自洽；FSM 状态表与 handler 双向闭合；BeginScene/BuildTargetModel/SurveyScene/ExecuteTarget 的 goal→服务端字段与状态名两侧一致；深度单位单源换算（mm 边界纪律良好，无残留 mm 假设）；ExecuteTarget outcome 枚举在 arm/FSM/ledger/observability 四侧数值闭环；恢复 ACK 全链完整且 plan_id/令牌不复用；TF 帧集合闭合、全部查询两端可达；感知/重建/臂帧名参数驱动；五个 Python 包参数键双向零漂移；线程地图总体到位。

---

## 二、高危发现

### G1 接触许可令牌 5s 有效窗 vs 接近链时长错配（两路独立交叉印证）
- 生产端：`publish.py:111 MODEL_VALIDITY_S=5.0`；`reconstruction_core.py:661-672` 同 revision 沿用首次冻结时刻；Build finalize 后 collector 转 READY 不再采帧（`capture.py:615`），revision 停变 → **valid_until 钉死在 build 收口后首心跳 +5s**。
- 消费点在套入授权（CONTACT 级）：`stages.cpp:987`→`cycle.cpp:81-86`；该检查位于「finalize→reconfirm(2–6s)→回拍照位→PTP+LIN→残差验证(≥0.6s×3)」**之后**；真机单观察 LIN 实测 7.5s（testing-log 09-17 六续）→ 接近链全程必 >>5s。
- 后果链：令牌过期→拒套入→`contact_recovery_required_=true`（`stages.cpp:835-837`）→FAILED+恢复门→**FULL 档每颗都要人工 ACK**。当前默认 `execute_pregrasp_only: true` 掩盖本项。
- 佐证：回放架用 30s 窗仍录得「1 有效期竞态」（testing-log:382）——生产 5s 更不可用。

### G2 conservative 档 observe goal 缺身份元组 → plan 契约必拒；fast 档契约真空
- OBSERVE goal 只填 9 字段、无 model/calibration/config_revision（`executor_node.py:1142-1151`，主审抽验确认）；FULL goal 三修订必非空（`cycle.cpp:142-159` identityComplete）。
- `previewMatchesExecute→identitiesMatch` 全字段相等（`plan_contract.hpp:48-62`）→ `""≠"t0:3"` 恒 mismatch → **FULL 一律 abort**（`cycle.cpp:250-257`）；且 `last_preview_valid_` 无复位点→第二颗起连 OBSERVE 也被拒。
- fast 档（默认）不走 arm 观察动作→`last_preview_valid_` 恒 false→plan 契约整条不生效。即 conservative=必拒、fast=永不校验。
- 单测契约与装配相反：`test_pregrasp_level.cpp:76-93` 假设 observe goal 携带完整身份。
- 症状已录 testing-log:351（09-17「新缺陷信号」），本轮补齐根因。

### G3 model_revision=`{target_id}:{views_count}` 非单调 → 臂持过期模型快照
- `refine.py:1296`；臂侧 ModelSnapshot 换新仅看 `(model_revision, target_id)` 变化（`manipulation_skills_node.cpp:876-892`）。
- 同目标重 Build 且聚类机位数相同 → revision 字符串相同 → 臂不替换快照 → 沿用旧（过期）valid_until 与几何 → 该目标接触恒拒 `model_not_executable`。

### G4 停栈报告竞态：launch 默认 5+5s 拆除窗 vs join_report(300s) + 非原子写
- Jazzy `ExecuteLocal` 默认 `sigterm_timeout=5`/`sigkill_timeout=5`（execute_local.py:85-88，主审抽验）；observability 以普通进程在栈内（无停序安排、不在 lm 名单）。
- 真机大 bag 报告 >5s 必被杀；`bag_report.py:832-833` 直写无 tmp+rename → **留半份报告**。mock 冒烟 bag 小未暴露；`join_report(300)` 在 launch 托管下形同虚设。

### G5 preflight 漏检 brain 主进程（主审修正口径）
- 名单（`preflight.py:6-14`）实际覆盖 peach_arm/peach_observability/ros2_control_node 等多数现行名（代理初报「peach_arm 不在名单」不实，已纠正）。
- **真实缺口**：brain 进程 exec=`peach_harvester`（`brain.launch.py:46` 无 `name=` 参数）→ 场景/重建/调度三节点名不进 argv → 残留旧脑不被拒启 → **双 supervisor/双感知静默共存**（最危险形态）。另漏 `peach_lifecycle_flag_bridge`、`peach_autostart_client`、相机节点。

---

## 三、中危发现（摘要，全部带双侧 文件:行 于各段原始报告）

**执行/恢复路径**
- M1 `cancel_requested_` 全局旗标只在下周期 onStart 复位（cycle.cpp:435 唯一 store(false)）→ 一次单果取消/skip 后一切 MoveTo 被拒、fast 补视静默退化（observe.py:166-174 把拒单当 moved）——testing-log 已录「批次取消后 PHOTO 亦失败」的真正机理，且触发面更广。
- M2 Survey/MoveTo 受理回调仍无限 join 且同默认互斥组（cycle.cpp:523-527、move_to.cpp:154-159；W13-B 只修了 ExecuteTarget）+ 预览 worker 无界 join（cycle.cpp:432-434）→ 卡死时 ACK/取消/订阅全停（恢复死锁最坏形态）。
- M3 失败码断链：plan-mismatch/onStart 拒路径 failure_code=0（PLAN_MISMATCH=20 有码不用，manipulation_skills_node.cpp:1191、cycle.cpp:300-309）；ledger 只落 'full_failed' 笼统串；令牌过期经话题快照回看被误分级 FAILED 而非可重派的 SKIPPED_QUALITY（stages.cpp:242-248）。
- M8 单槽单向门：executeMoveTo 不置 running_、Survey 等帧窗口 running_=false → MoveTo 运动中可受理 ExecuteTarget（双运动流）、Survey 快照可被 MoveTo 挪机位；`~/fire_step` PHOTO 无批忙门是现网触发面。
- M9 MoveTo 失败码语义错用（取消=RECOVERY_REQUIRED、拒=DECISION_REJECTED、失败=SLEEVE_PLAN_FAILED，move_to.cpp:189-259）+ fast 循环不读 `arrived` → 失败补视计成功。

**感知→选果→重建**
- M4 臂侧观测几何不校验 header.frame_id 与 tf_stale（`src/peach_arm` grep frame_id/tf_stale 零命中）：TF 退化帧的相机系/陈旧 TF 几何入缓存并刷新新鲜度门（重建侧有 frame 门、supervisor 侧有 base 门，唯臂缺位）。
- M5 supervisor 选果不消费 out_of_view/anchor_stale/depth_void/tf_stale/tracking_status（batch.py:43-67 vs identity.py:237-247）→ 已判「复扫无益」目标仍被选中空耗 Build。
- M6 IK 未覆盖（相机系 candidate 无 pose）且半径回退 None 时目标在无几何门下直通（batch.py:70-82,119-134）。
- M11 诊断 JSON 的 grasp_decision.allowed 恒 False（reconstruction_core.py:891 从不置 True；类型化消息侧另算）→ events.jsonl 审计轨迹上许可永远呈拒绝。
- M12 BuildTargetModel goal.scene_epoch 无校验 + model.scene_epoch 双源（goal 回显→节点 epoch 覆写，target_reconstruction_node.py:1219-1471）。

**启动/契约/配置**
- M7 `moveit_enabled:=false` 时栈照常"就绪"但每批必 survey_failed（开关只传 aubo include，peach_arm 恒 initializeMoveIt 且 MGI 5s 等待返回值被忽略）。
- M13 SceneSnapshot 落盘订阅 VOLATILE vs 发布 transient_local → 记录节点晚启动则单发快照永不入 bag（observability_node.py:190）。
- M14 重建 diagnostics offered deadline=1.5s 未入 manifest QoS 契约（target_reconstruction_node.py:309-318）。
- M15 DepositResult.msg 死文件+过期头注释（W7 删 deposit 后遗留）。
- M16 executor 观察环帧名硬编码绕过 frames 参数（observe.py:94、executor_node.py:1440 'base_link'/'camera_depth_optical_frame' 裸串）。
- M17 arm GPL 三个执行契约超时（enables_heartbeat/robot_status_timeout/model_max_age）只在 schema 默认值、不入部署 yaml——现场不可调。
- M18 ExecuteTarget Feedback.state/checkpoint 与 Result.verification 块生产无消费（executor 不注册 execute 的 feedback_cb；W7 宣称的「证据单源」只兑现了 harvest 块）。
- M19 闩锁话题 `/peach/lifecycle/managed_nodes_activated` 双生产者（flag_bridge+自研 lifecycle_manager.py:41-44）只入册一个，手启自研件会互相覆盖闩锁。

---

## 四、低危（附录摘要）

IDL 死管道聚簇（TargetModel 几何体整块/BagFitting 十字段/BagGrasp2D 像素几何/候选协方差/match_status/quality_score 恒 0/MoveTo KIND_JOINTS·joints·speed_scaling/SurveyScene Feedback 无生产/Clearance 四字段与 goal.generation 无消费）；注释旧名（HarvestState/CanonicalEvent/ControlTask/RunHarvest 头注释仍写 peach_executor 时代话题）；`*1000` 恒等比较（pose_pipelines.py:339,898）；supervisor `_decision_cache`/`_observations` 无锁换引用（GIL 良性、纪律不一致）；工具连杆名三方手工同步（重建硬编码 vs yaml vs xacro）；感知/重建相机参数双份无交叉校验；plan_updater.py:322 恒真条件；记忆锚点回填不设 candidate.header；operator_skip×SUCCEEDED 账本/事件漂移；`_cmd_full` 超时不等臂收口→窄窗放大整批 INTERRUPTED；末条反馈晚到误闩恢复门；manifest 缺 /peach/perception/diagnostics qos 列；peach_harvester 旧转发 launch 参数面窄（丢 camera_frontend 等）；autostart fire-and-forget 无结果回调。

---

## 五、修复优先序建议（本轮只审不改）

1. **G1+G3**（同域：许可/模型版本生命周期）——valid_until 预算重标定（或改"受理时校验+执行段心跳续签"语义需重审 0022）与 model_revision 单调化（加 finalize 计数/时间戳）一起动，均涉真机验收口径。
2. **G2**（conservative 档功能性断链）——observe goal 补三修订字段即可对齐单测已钉契约；fast 档是否要 preview 记录需产品判断。
3. **G4**（停栈报告）——observability 进 lm 名单或 launch 显式 sigterm_timeout 覆盖 + bag_report tmp+rename 原子写。
4. **G5**（preflight）——名单加 `peach_harvester`（exec 名）/`peach_lifecycle_flag_bridge`/`peach_autostart_client`，或改为按 `ros2 launch peach` 组合匹配。
5. M1/M2/M3（恢复路径与失败码）优先于其余 M 项；IDL 死管道聚簇可与下轮接口清理合并处理。

## 六、方法学声明

四段独立追踪，G1 由两路代理独立命中（S2-H1=S3-F2）互为印证；主审抽验 G1/G2/G4/G5 四条关键证据（G5 修正了代理的一处事实错误）。审查全程未修改任何文件、未起任何栈。

---

# 复审附录（2026-09-21，修复轮 688ea64/8bf001f/c242de3 之后）

方法：双路独立复审（harvester 提交 / arm 提交逐 hunk + 消费点终盘）+ 主审核跨提交接缝（SKIPPED_QUALITY→调度/rework 闭环：`rework_kind('quality')`→`rework_list.json` 人工补采，批内 claimed 排除自动重派——"可重派"语义成立）。

## 判定总表

| 提交 | 修复面 | 判定 | 新回归 |
|------|--------|------|--------|
| 688ea64 | G1 窗口参数化 | **修对**（参数链五环闭合、0022 冻结逐字保真、COLLECTING 归空保真、第二处硬编码收口、fixture 无 5s 残留） | 无高/中；低×3 |
| 688ea64 | G3 revision 单调 | **修对**（锁纪律/RLock 核实、reset 不清零核实、18 文件消费点全不透明串、goal 装配与话题侧同源同缓存必同串） | 无高/中；低：跨进程重启撞串窗口收窄未消除（counter 无持久化，被 120s 长窗放大暴露时长） |
| 688ea64 | M11 allowed 同源 | **修对**（提取与旧消息侧逐字段等价、早退路径数值等价、恒 False 下游无隐藏假设） | 无 |
| 8bf001f | G2 preview 分流 | **修对**（调用点守卫与旧版逐字等价=无放松；conservative 必拒与无复位两缺陷均除） | 低：**契约全线休眠**——debug 桥白名单缺 plan_id/两修订，全树无客户端能发带 plan_id 的 PREVIEW goal（plan_id 成事实死字段；连带发现 debug 桥的 FULL 也因缺修订被受理门拒=手动 FULL 本不可达）——设计取舍未落文档 |
| 8bf001f | M1 取消旗自动清 | **修对**（running_ 全表核实守卫充分；运动停止在置旗时同步生效） | 低：survey 快照等待环可能错过他家取消（有界兜底、终局分类同旧） |
| 8bf001f | M2 三处 timed join | **修对但留缝**（回调死锁确实消除、与 W13-B 同构） | **中（新）**：executeMoveTo 不置 running_（M8 遗留）→ detach 后卡死 MoveTo 线程与新周期可**并发使用 move_group_（MGI 非线程安全，UB）**——限"卡死>2s 且无视取消"病态场景，本质是死锁换 UB 的权衡，注释未提 |
| 8bf001f | M3 失败码 | **大体修对**（PLAN_MISMATCH=20 双钉、EXPIRED 严格限两支、SKIPPED_QUALITY 全链闭环到账本 target_skipped） | 低×3：EXPIRED 路径 failure_code 仍 0（MODEL_EXPIRED=18 闲置）；"可重派"被 recovery 闩锁限定（CONTACT 门在预抓取运动后→仍须 ACK；默认 PREGRASP_ONLY 档不可达该门）；pending code 在 detach 重叠下理论竞争 |
| c242de3 | G4/G5/M13/M15 | **修对**（主审自查：LifecycleNode kwargs 透传 sigterm_timeout ✓、原子写 ✓、preflight argv0/lib 匹配不误伤 ✓、闩锁两端齐 ✓） | 无 |

## 复审新增跟进项（编号续 R）

- **R1（中，新）**：MoveTo detach 并发 MGI 窗口——建议 executeMoveTo 补 running_ 置位（即原 M8 的收口），一并消掉 detach UB 面。
- **R2（低中，预存半修）**：`closeMotionOutputAndCancel`（manipulation_skills_node.cpp:283-296）四处裸 join 仍无界——生命周期/析构路径遇卡死线程仍吊死；detach×releaseResources UAF 暴露面已扩到 survey/move_to/worker。
- **R3（低）**：EXPIRED 路径接 MODEL_EXPIRED=18；G3 revision 并入 boot/世代段或持久化计数；`_decision_expiry_warned` 加上界；debug 桥白名单补 plan_id/两修订（或文档明示 preview 契约休眠+手动 FULL 不可达）。

## 复审结论

七项修复全部达成 2026-09-20 审查原意、无放松性回归；唯一新增中危 R1 仅存在于 M2 本要处理的病态场景（卡死>2s）且是显式权衡；其余为低危残留与文档缺口。修复轮验收通过。
