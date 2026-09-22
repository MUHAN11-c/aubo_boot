# 真实相机 + 仿真控制：双末端感知→抓取验证与分析方案 v1.0

日期：2026-09-22　分支：`test/20260909-field-traj`（含未提交改动，行号以当日磁盘为准）
性质：**方案文档**（本轮只做源码深读 + 计划整理，不动代码、不启栈、不录数据）。
依据：peach 八包源码深读（三份探读报告，关键结论带 file:line）+ 三份活文档（architecture / io / testing）当轮核对。

---

## 0. 结论摘要

1. **目标**：在「真实相机 + mock 控制」形态下，验证两种末端（`hollow_cylinder_v1` 固定圆柱刀 / `adaptive_cylinder_v1` 自适应圆柱+IMU）从感知识别到抓取的全链稳定性；重建（`target_reconstruction`）本轮跳过，只当后续精化件。多组不同感知约束的结果要能跑完整抓取流程，全部通过后才进真机验证。
2. **可行性已经源码证实**：跳过重建有现成机制（`skip_reconstruction:=true` → supervisor 不发 Build/补视 + 臂侧 `quality.allow_unrefined_geometry` 把锁定集场景观测提升为接触几何）；注入式多组仿真有现成工具（`scripts/sim_field_targets.py`，含 `--grid` 感知约束网格与期望分类）；全量录制有现成机制（会话 bag `record.level` + 100G 预算 + `purge_analyzed_bags.py`）——**不需要新造框架，P0 只需小修与补维**。
3. **最大的结构性风险**（详见 §8 A 级清单）：多目标批次里非 selected 目标在执行期被「断粮」（锁定后 SAM 只分割当前目标 → 其余目标 30s `anchor_stale`、120s 被 drop）；预抓取残差修正只认 selected 缓存；调度资格谓词与感知不一致（REJECT/摆动目标可被 claim、`camera_distance_m==0` 绕过深度窗）。这些决定 P2 批次链实验的设计方式，先量化再修。
4. **双末端分工**（按既定裁定）：hollow 档跑完整抓取矩阵；adaptive 档仿真只测 **IMU 跟随**（真实 IMU 数据 + `serial_imu` 对齐 TF 正常），不做套入链。
5. **验收门先量化后定稿**：P0 基线轮跑完，把 §9 的建议阈值标定成正式门，之后 P1–P4 按门放行。

---

## 1. 目标、范围与成功定义

### 1.1 核心目标（按优先级）

| # | 目标 | 判定层 |
|---|------|--------|
| G1 | 感知识别结果稳定（袋检测/分割/袋几何/身份锁定，真实相机、真实光照与遮挡扰动） | P1 |
| G2 | 抓取轨迹无碍：感知约束驱动的多组不同结果，都能走完整 `ExecuteTarget`（接近→预抓取→套入→撤退→回 stow，mock 干跑） | P2-a |
| G3 | 调度批次链无碍：`RunHarvest` 端到端（选果→派发→账本→补采）在真实相机场景下多批可复现 | P2-b |
| G4 | 自适应末端 IMU 跟随链可用（真实 IMU 数据、TF 对齐正常、跟随/刹车/断流保护） | P2-IMU |
| G5 | 过程数据可复盘：全量 bag（100G 上限）+ rviz 视频 + 结构化文本三面齐备，分析产物可留存、原始 bag 可删 | 全程 |

### 1.2 范围与边界

- **包含**：`peach_harvester`（感知+调度）、`peach_arm`、`peach_bringup`、`peach_observability`、`serial_imu`/`imu_follow`（仅 adaptive 的跟随链）、`scripts/` 战役工具。
- **不包含（本轮忽略）**：重建/精化链质量本身（`skip_reconstruction` 跳过；其参数与门不放松）；导航（已归档）；旁路 IVG 三包；真机运动与 SetIO（未授权不动）。
- **mock 证明不了的（真机 KEEP，不装懂）**：碰撞安全、接触力/电流止损（④层）、`RobotMoveStop` 停轨、SetIO 刀具回路、真机 TF/外参漂移。这些在 §9 P4 门里列为真机项。

### 1.3 「成功」的口径（贯穿全文）

- 仿真抓取成功 = `ExecuteTarget` 终局 `SUCCEEDED` 且 `completion_level ≥ LEVEL_SLEEVE_COMPLETED`（撤离后为 6），`harvest.grasped=false`（`tool.enabled=false` 干跑不宣称采摘成功）。`cut_confirmed` 结构性不可达（SetIO ACK≠切断，`stages.cpp:1098-1102`），真机口径照旧，不为本轮改语义。
- 感知稳定 = §9 P1 门全部通过（锁定时延、抖动、身份、TF 新鲜度、丢帧）。
- 每个失败必须有稳定 `failure_code` 归因，不允许 hang、不允许无码失败。

---

## 2. 现行系统事实基线（源码深读结论）

### 2.1 全链一页图

```
真实相机(percipio|stereo 前端, 话题同构 /camera/*)
  → scene_perception(三路 ApproximateTime slop0.05 → YOLO0.35 → 门0.40/IoS0.6
    → SAM(锁定后仅锁定集/当前目标) → 深度窗[0.3,1.5] 袋位姿管线 → χ²身份 match_or_register
    → confirm 5帧 → 锁定集)
  → /peach/perception/target_observations（base_link 系 entry/bottom/neck/axis/diameter/travel + flags）
  → supervisor(FSM: WAITING_READY→DISCOVERY(Survey→Begin→WAIT_LOCK)→SELECT
    [采收率门 ∩ 锁定集 ∩ 非裸果/贴边 ∩ 深度窗0.3-1.6 ∩ CheckReachability IK/半径窗]
    → DISPATCH [skip_reconstruction? 不发Build : Build+OBSERVE并行]
    → PREGRASP_ONLY(默认)/FULL)
  → peach_arm ExecuteTarget（受理门=身份元组7字段+锚点/锁定集 → 阶段序列
    Prepare→Finalize→Reconfirm→MovePregrasp(拍照位→staging PTP+轴向LIN)→VerifyPregrasp
    →[PREGRASP: HoldPregrasp] / [FULL: PlanSleeve→SleeveLinear→SetIO→VerifyCut→Retreat→Stow]）
  → 授权矩阵 authorizeStage（TRANSIT/PREGRASP: Active∧robotReady∧execution;
    CONTACT:+grasp∧令牌/决策复检; TOOL:+tool）→ JTC(mock)/透传(real)
```

### 2.2 感知链关键事实（`src/peach_harvester/peach_harvester/vision/scene_perception/`）

- 输入 `/camera/color/image_raw` + `/camera/depth/image_raw` + `/camera/color/camera_info`，slop 0.05s（`scene_perception_node.py:189-201`；`config/scene_perception.yaml:8-16`）。QoS RELIABLE/VOLATILE depth10。深度 uint16×0.25=毫米。
- 帧处理是容量 1 的 BoundedWorker（drop_oldest，`scene_perception_node.py:196-197`）；丢帧计数 `BoundedWorker.dropped` 存在但**未对外发布**（`runtime.py:95,106,120`）——P1 指标缺口，见 §8 B10。
- 单帧门：`min_detection_conf 0.40`、YOLO 0.35、IoS 0.6/碎片 0.5、深度窗 [0.3,1.5]m、`min_points 100`、`travel_too_short<0.05`、`bag_axis_too_short<0.03`、`axis_2d_mismatch>45°`、`foreground_truncated` 触边比 0.15/3边、`axis_length_inconsistent`、`tool_clearance_failed`（唯一 REJECT 来源）。置信度=valid_ratio/0.65 × n/800 × (1−touch)（`pose_pipelines.py:499-501`，无标准化口径，仅 tie-break）。
- 身份：χ² 门 9.0、`match_radius 0.06`、`confirm_frames 5`、TTL 20 帧、`anchor_max_age 30s`、drop 120s、`target_swinging` 0.03m×3帧（`identity.py:26-29`；yaml:56-67）。贴边帧不攒确认 + `bbox_edge` 选果侧过滤。
- 可视化（rviz 录视频用）：`/peach/perception/debug_image`（确认目标叠加：框/掩膜 alpha0.4/底→颈箭头/剪切紫圈/ID+置信度标签）、`debug_image_raw`（全量真相流，注释明确不进 RViz）、`markers`（袋轴/入口→行程 ARROW/包络圆柱/扫掠体积/先验球/ID 文本）、`single_cloud`、`detections`、`masks`（`debug_draw.py:53-152`；`msg_builders.py:421-505`）。
- 事件落盘：`events.jsonl`（`global_targets_locked`/`frame_observations` 每帧一条/`scene_begin`/`target_dropped`），HarvestDataStore 写队列 512，**队列满在 plan_lock 内同步降级写**（`runtime.py:221-226`）。
- 稳定性指标现状：`stream_metrics` 只产 EMA 统计（fps/分段耗时/光照），经 latched `harvest_state` 暴露——**可直接当 P1 门原料**；逐帧抖动/丢帧率没有现成指标（P0 需补一小段离线分析：从 bag/events 复算）。

### 2.3 调度链关键事实（`supervisor/`）

- FSM 9 批次态 × 13 目标相位，纯核 `harvest_fsm.react`；`RunHarvest` goal 字段 request_id/intent/target_ids/target_harvest_ratio/per_target_timeout_s/sector_timeout_s/view_policy。
- 选果（`batch.py:140-210`）：锁定集 ∧ confirmed ∧ 非裸果 ∧ 非 `bbox_edge` → 深度窗 0.3–1.6m（**`batch.py:172` 在 `camera_distance_m==0` 时跳过检查**）→ 半径窗 0.15–0.88m（CheckReachability 不可用时静默回退，2s 超时）→ priority 升序+同级框面积降序。资格谓词**不含** REJECT/target_swinging/tf_stale（`batch.py:104-114`）——与感知 `_selectable` 不一致（§8 A3）。
- 视点两档：fast（supervisor 直驱补视 ≤3 视）/ conservative（臂 OBSERVE_ONLY，4 重试）。
- 操作台：`SetEnables` → latched `/peach/batch/enables` + 1Hz 心跳；`ControlTask`（PAUSE/RESUME/CANCEL_NOW/SKIP_TARGET/ACK_RECOVERY…）。
- 账本 `runs/<request_id>/ledger.json` 每命令收口+断点恢复；跳过/失败自动入 `rework_list.json`。
- **跳过重建**：`skip_reconstruction`（`peach_supervisor.yaml:14`，launch `harvest_system.launch.py:132-134` 透传，并同时注入臂侧 `quality.allow_unrefined_geometry`，bringup:188-189）。入口 `executor_node.py:1134-1136` → `_dispatch_unrefined`（:1299-1325）：plan_id 加 `:unrefined`、revision=`unrefined:{epoch}:{target_id}`、不装 GraspDecision 令牌，直接 READY_FULL。臂侧 `promoteUnrefinedGeometry`（`target_cache.cpp:471-493`）把场景观测钉成 READY+grasp_allowed 并置 `unrefined_hold_` 锁存（跨批证据短路风险，§8 A6）。

### 2.4 抓取链关键事实（`src/peach_arm/`）

- 受理门（`cycle.cpp:146-207`）：FULL/PREGRASP_ONLY 须身份元组完整（run_id/scene_epoch/target_id/model_revision/tool_profile_id/calibration_revision/config_revision，`model_contract.hpp:52-61`）+ 目标命中锁定集锚点或 selected 缓存。scene_epoch 感知侧恒 0（`cycle.cpp:660-662`）→ 元组该字段形同虚设。
- 阶段序列与检查点：CK_AT_PREGRASP/SLEEVE_PLANNED/SLEEVED/CUT_ACCEPTED/RETREATED/STOWED 已实现；**CK_AT_STAGING 永不标、CUT_CONFIRMED/RETAINED 永不标**。
- 预抓取残差门：1.5°/2.0°/3mm，两次采样隔 200ms，最多 3 次修正（`stages.cpp:859-978`；`pregrasp_residual.hpp`）。**修正回路取 `refinedSnapshot()`（selected 缓存）**，非 selected 目标 ID 不符直接停修（`stages.cpp:925-933`，§8 A1）。
- 接近主路径：拍照位 PTP（0.5s Pilz→3.0s OMPL）→ staging 转移（PTP 到预抓取下方 0.10m + 轴向 LIN）→ 直连 LIN 兜底；套入/撤退 MTC 笛卡尔直线（0.005 步长/0.95 最低完成）。护栏：关节 12/6.1 rad、绕行 2.6/偏离 0.32/回退 0.12（09-18 标定值，勿动）、TCP 姿态 110°+20°、果实胶囊 0.12 回退半径、反爬 +2cm。
- 失败码 0–20 全集见 `FailureCode.msg`；`stage_denial.hpp` 分级（EXPIRED→SKIPPED_QUALITY 可重派，其余 FAILED）。
- mock 差异：JTC 替代透传；`require_robot_status=false` 注入（launch:183-187）；无 SetIO 服务（`tool.enabled=false` 时跳过 IO，周期可走完；**若开 tool 必挂 TOOL_STATE_UNKNOWN 且不自动撤退**）；电流止损不触发；取消=JTC 抢占。

### 2.5 双末端档案（`aubo_description/config/*.yaml` + launch `tool_profile`）

| 项 | hollow_cylinder_v1 | adaptive_cylinder_v1 |
|----|----|----|
| D_inner / D_outer / L_insert | 0.104 / 0.120 / 0.200 | 0.116 / 0.120 / 0.200 |
| TCP 原点 | (0, 47.90, 151.07) mm | (0, 47, 168.66) mm |
| IMU | 无 | 挂 tcp，`serial_imu` 姿态源 |

- 切换=整栈 launch 参数（默认 adaptive），驱动 URDF 帧、感知 `tool.D_inner`、重建 `tool.budget.d_inner`、`tool.profile_id` 标签（`tool_profiles.py`）。peach_arm 审查几何（筒 0.06/0.2）是静态 yaml 不随档案——当前两档案外径相同无实害（§8 C3）。
- 帧名两把共用冻结：`tool_axis/cutting_plane/tcp/sleeve_mouth/tool_body_link`；`L_blade=0` 使三工具帧塌缩同点（§8 C4）。

### 2.6 录制 / 可视化 / 注入现状（含已有战役工具，勿重复建设）

| 能力 | 现状 | 入口 |
|------|------|------|
| 会话 bag | 17 条镜像订阅 + raw（debug_image×2/tsdf_cloud/scene_snapshot/tf/rosout）+ `record.level: std`（相机 raw 限 1Hz）/`all`（不限速，stereo≈50MB/s）/`core` 三档 | `src/peach_observability/config/observability.yaml:33-41`；随栈启停开合 `runs/session_*/bag/` |
| 容量预算 | **`max_total_bag_gb: 100.0` 已配置**；超限停栈自动回收最旧 `session_*/bag`（报告/账本/文本永不删，审计 `retention_audit.jsonl`） | 同上 yaml:40 |
| 分析后删除 | `scripts/purge_analyzed_bags.py`（按 session 或 `--all-reported runs` 删 MCAP 留报告） | 已入库 |
| 停栈报告 | `bag_report.md/json` 自动生成；CLI `peach_bag_report` 可复跑 | observability |
| 单轮归集 | `scripts/collect_round.py <request_id>` → `round_report.md`（账本+感知事件+ros 日志+bag 互链） | scripts |
| 注入仿真 | `sim_field_targets.py`：`--case`（案册）/`--random N --seed --envelope typical|algorithm`/`--grid`（`perception_constraint_grid.yaml`，期望分类 succeed/skip_ik/skip_cartesian/skip_select，现 9 例）/`--mode full`/`--velocity 1.0`/`--tool-profile`（**参数存在但未接进消息体**，两处硬编码 `adaptive_cylinder_v1`：`sim_field_targets.py:744,977`，§8 A4） | scripts |
| rviz | `aubo_e5_moveit_config/rviz/moveit.rviz` 已配 Debug Image/多路点云/感知重建 markers/TCP Path/Imu/RobotModel/MotionPlanning；缺口=抓取目标位姿箭头、果实胶囊 keepout 形体、`/imu_follow/target_pose`、bag 回放专用配置 | §6 |
| 8090 | 过程页/账本直播/调试 POST（`debug.motion_enabled` 门）；web 不显示 debug image（只进 bag+RViz） | observability |
| 战役助手 | `scripts/lab_perception_grasp_campaign.sh`（preflight/imu-tf/grid-cmd 提示器） | scripts |

### 2.7 IMU 跟随链（adaptive 专项）

```
真实 IMU(/dev/imu, CH343) → serial_imu(/imu/data, frame=imu_link, Rx(180°)软旋正)
  → align_to_parent：查 TF base_link→tcp 旋转当残差清零（对齐由 serial_imu 负责）
  → imu_follow(enable 时采 TF base_link→tcp 参考 + 当前 IMU 四元数参考)
  → 每节拍把 IMU 增量经死区/锥限幅(0.35rad)/平滑 → servo delta_twist_cmds(BEST_EFFORT)
  → moveit_servo → JTC(mock 臂跟随)
```

- 前提（testing.md 已验证路径）：先导 `global_photo_pose`（mock 全零参考 IK 无解）；servo 用 `imu_follow_servo.launch.py`；真正下发须 `motion.enabled=true`（mock 自由，真机须授权）；**勿与 peach MTC 执行同时开门**（指令流互踩无仲裁）。
- 自动保护已修好且有双重防护：IMU 断流 0.5s / 关节断流 1.0s / 连续 IK 失败 10 次 → disable；disable 后在途 IK 回包不补发、迟到 FJT goal 立即取消（`follow_node.py:394-399,462-469`）。
- 插入推进 `~/insert_start/stop`：0.01m/s、行程钳 0.20m、锁开口方向——自适应末端套入的人工衔接段，本轮只验证到「推进匀速+可停」。

### 2.8 mock 形态的物理语义（决定实验设计，必须先讲清）

**真实相机装在真臂腕上，臂不动；mock 关节却在动。** 因此 TF `base_link←camera_optical` 随 mock 关节走，而真实相机静止——mock 关节偏离「物理相机实际位姿对应关节值」时，感知算出的 base_link 系几何有伪影（重建侧实测 ~220mm，`target_reconstruction.yaml:31` 注释）。结论：

- **感知（P1）必须让 mock 停在与物理相机一致的位姿**：初始 `harvest_stow` 与拍照位差 0.22rad（wrist2），需先开 execution 做一次 Survey（mock 自由）把 mock 关节对到 `global_photo_pose`，之后物理相机、TF、感知三方自洽。
- **注入仿真（P2-a）完全绕开相机**，注入目标几何直接进 base_link 系，无自洽问题——这是「按感知约束模拟多组结果」的主通道。
- **批次链（P2-b）**：相机在拍照位看真袋 + `skip_reconstruction:=true`，臂在照片位与预抓取之间往返；回访拍照位后感知几何重新自洽。每颗之间回 Survey（FSM 既有行为）恰好满足自洽条件。

---

## 3. 验证总体架构

### 3.1 三条数据面

| 数据面 | 内容 | 保留策略 |
|--------|------|----------|
| 结构化文本 | ledger.json / events.jsonl / sim_field_targets_*.jsonl / bag_report.md / round_report.md / retention_audit.jsonl | **永久**（入库随仓推送） |
| 全量 bag | `runs/session_*/bag/`（MCAP，100G 预算） | 分析完成即 `purge_analyzed_bags.py` 删除，报告留存 |
| rviz 视频 | 录制窗口视频（§6），`runs/<session>/video/` 或 `runs/<request_id>/video/` | 分析完成可删（结论写进文本） |

### 3.2 两种目标来源的分工（本方案的关键设计）

| | 真实相机场景 | 注入合成目标 |
|--|--|--|
| 服务目标 | G1 感知稳定性 + G3 批次链 | G2 多组抓取矩阵 |
| 通道 | 相机→感知→supervisor→ExecuteTarget | `sim_field_targets.py`→话题注入→ExecuteTarget（或 grid 的 SELECT 资格子集） |
| 多样性来源 | 物理摆袋/光照/遮挡（人工布景） | 采样器（位置/倾角/行程/直径/期望分类） |
| 自洽条件 | mock 须停在拍照位（§2.8） | 无（`camera_enabled:=false`） |
| 覆盖门 | 无法穷举 → 定性 + 统计 | 可批量 → 定量门 |

两者结合才同时满足「感知结果稳定」与「多组不同结果完整抓取」：真实相机证明感知链对，注入矩阵证明抓取链对任意合法感知输出都对。

### 3.3 轮次组织

每轮 = 一个 `request_id`（按 testing.md 命名：`e2e_survey_/e2e_unrefined_/e2e_full_unrefined_<YYYYMMDDTHHMMSS>`）+ 一个会话 bag + （可选）rviz 视频 + 当日 `runs/field_test_20260922/log.md` 追记 + testing-log 追加。注入矩阵轮按 seed 命名 jsonl（脚本自动）。**前清后清 pgrep 是 MUST**（09-21 四个挂死 bag record 堵死深度流的教训，见 AGENTS 第 1/9 章）。

---

## 4. 阶段计划

### P0 准备（预计 1 个工作轮，全部是应用包/脚本小改，驱动栈零接触）

| # | 事项 | 类型 | 产出 |
|---|------|------|------|
| P0-1 | `sim_field_targets.py` 把 `--tool-profile` 接进 `msg.tool_profile_id` 与 `goal.tool_profile_id`（消两处硬编码 :744/:977） | bug 修复 | hollow 档网格可跑且账面不污染 |
| P0-2 | 扩充 `perception_constraint_grid.yaml`：补遮挡维度（遮挡类 flag：`mask_unavailable`/`foreground_truncated` 注入）、袋径维度（0.05/0.068/0.10 贴 D_inner 门）、袋长维度（0.03 短袋/0.18 长袋）、直径超 D_inner−2×clearance 的 `tool_clearance_failed` 用例 | 用例补充 | 网格从 9 例扩到 ~20 例，覆盖 §3.2 采样维度 |
| P0-3 | rviz 视频录制脚本（`scripts/record_rviz_video.sh`，ffmpeg x11grab，见 §6） | 新脚本 | 一条命令出 mp4 |
| P0-4 | 暴露 `BoundedWorker.dropped` 到 `harvest_state` JSON（一行镜像）或至少确认 10s 节流日志可检索 | 观测补缺 | P1 丢帧门有数据源 |
| P0-5 | （可选）`moveit.rviz` 增补：抓取目标位姿箭头消费（若感知 markers 已含入口 ARROW 则只补 `/imu_follow/target_pose` 与果实胶囊演示） | 可视化 | 录视频信息量 |
| P0-6 | 基线轮：起栈（mock+相机+skip_reconstruction）→ SURVEY_ONLY 拍照位 → 只扫 → `--grid` 全网格 → `--random 30 --seed 20260922 --velocity 1.0` | 实测 | 标定 §9 阈值；验证 P0-1~5 |
| P0-7 | 处理存量：`runs/session_20260922_094104`（15.5GB）、`100001`、`103652` 三袋先出报告、归档结论、purge | 数据卫生 | 释放磁盘（当前仅剩 171G free，82% 已用） |

### P1 感知稳定性（真实相机；两前端各一轮起）

前置：mock 臂 Survey 到拍照位（§2.8 自洽）；`record.level: std`（长跑）；rviz 视频开录。
场景矩阵（物理布景，每场景 ≥10min 连续流）：

| 场景 | 操控 | 主要看 |
|------|------|--------|
| S1 静态基线 | 1–3 袋摆拍照位视野，不动 | 锁定时延、抖动、ID 稳定、fps |
| S2 光照扰动 | 开关灯/遮光/直射 | lighting.low_quality 触发与恢复、误 REJECT 率 |
| S3 遮挡 | 手/枝叶道具部分遮挡袋体、晃动 | flags（foreground_truncated/axis_2d_mismatch）、锚点保持 |
| S4 动态 | 缓慢移动袋（模拟摇摆）、拿走/放回 | target_swinging 正确触发、LOST→REOBSERVE 回归 |
| S5 多目标 | 4–6 袋不同深度/倾角 | 锁定集收齐、身份不串、深度窗边缘 |
| S6 前端对比 | percipio vs stereo 同场景 | 帧数口径漂移（confirm 5 帧的墙钟差 ~5.5×）、confidence 分布 |

分析（离线，读 bag+events，不占栈）：锁定时延分布、entry/bottom/neck/axis 稳态 3σ 抖动、ID 切换/重复注册次数、`tf_stale` 率、`frame_observations` 缺口（丢帧）、每 flag 触发率与合理性。
产出：`reports/<date>-perception-stability/report.md` + 阈值定稿（回填 §9）。

### P2-a 注入式多组抓取矩阵（`camera_enabled:=false`，G2 主通道）

起栈：`hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true imu_enabled:=false autostart:=false`，使能经脚本 `ensure_enabled`。每档配置一轮（每轮一个 session bag + video）：

| 档 | 命令 | 规模 | 门 |
|----|------|------|----|
| M1 网格回归 | `--grid --mode full --velocity 1.0 --tool-profile hollow_cylinder_v1` | P0 扩充后 ~20 例 | 期望分类全对（succeed/skip_* 与 `expect` 一致） |
| M2 典型随机 PREGRASP | `--random 30 --seed <s1> --envelope typical` | 30 | §9 P2 门 |
| M3 典型随机 FULL | `--random 30 --seed <s1> --mode full --velocity 1.0` | 30 | 同上 + completion_level≥6 |
| M4 算法包络压测 | `--random 30 --seed <s2> --envelope algorithm --mode full --velocity 1.0` | 30 | 记录近水平袋行为分布（不做硬门，先看数据） |
| M5 失败重测 | `--pick`（对 M2–M4 失败例） | 全部失败例 | 归因收敛：每例 failure_code 可解释 |

对照基线（回归参照，来自 testing.md 09-11 轮）：typical 30 例 26 到位、绕行比 ≤1.70、姿态 ≤71°；全链路 66/100；现场包络 39/41。M2/M3 低于对照即触发 P3 修正循环。
自适应末端：**不跑 M2–M5**（裁定：仿真做不了自适应跟随，只测 IMU）。仅保留 `--grid` 在 adaptive 档跑一遍 M1（验证档案注入/几何差异不破坏链路）+ P2-IMU 专项。

### P2-b supervisor 批次链（真实相机 + skip_reconstruction，G3）

起栈：`hardware_mode:=mock camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true autostart:=false`；mock 先 Survey 到拍照位。使能走 `SetEnables`。

| 轮 | 内容 | 门 |
|----|------|----|
| B1 | `e2e_survey_<ts>`（intent 2）只扫 ×3 场景 | photo_pose_reached→round_locked→completed；账本 claimed 空 |
| B2 | `e2e_unrefined_<ts>`（intent 0，默认 PREGRASP_ONLY）≥3 批 × ≥2 目标 | 事件链全（dispatch 无 Build→PREGRASP→recovery_required→ACK→回访）；ledger outcome=0/completion_level=2 |
| B3 | `e2e_full_unrefined_<ts>`（运行期 `execute_pregrasp_only false`，tool 关）≥2 批 | checkpoint 到 CK_SLEEVED/RETREATED/STOWED；grasped=false；无 SetIO |
| B4 | 多目标批次专项（≥3 目标/批）：**量化 §8 A4 断粮效应**（第 2/3 颗的 anchor_stale/drop/记忆锚点使用率） | 不设通过门；产出量化报告驱动 A 级修正 |
| B5 | 异常注入：批中 CANCEL_NOW、SKIP_TARGET、拔相机流恢复 | FSM 迁移正确、无 hang、账本/补采清单正确 |

### P2-IMU 自适应末端跟随专项（真实 IMU；G4）

前置：`imu_enabled:=true`（随整栈起，tf_parent=tcp，align 自动）；mock 臂导到 `global_photo_pose`；**peach 使能全关、无 MTC 执行**；另起 `imu_follow_servo.launch.py`。

| 步 | 操作 | 判定 |
|----|------|------|
| I1 | TF 核对：`tf2_echo tcp imu_link`、`/imu/data` 帧率、静置 `linear_acceleration.z≈+9.6` | 对齐 TF 正常（用户红线） |
| I2 | `enable` → 静置看 `~/target_pose` | 目标=参考，零漂移（死区 0.02rad 内归零） |
| I3 | 手转 IMU/工具（<0.35rad） | mock 臂姿态随动，`/moveit_servo/status`=0，位置零漂移 |
| I4 | `motion.enabled true` 后大幅转动+快速抖动 | 锥钳生效（0.35）、平滑、无奇异甩动 |
| I5 | 拔 IMU 串口/杀 serial_imu | 0.5s 自动 disable + servo 刹车零速 |
| I6 | `insert_start` → 观察 → `insert_stop` | 0.01m/s 匀速推进、姿态仍只跟 IMU、可停 |
| I7 | 对齐专项：转动工具后 `ros2 service call /imu/align_to_parent ...` 再 enable | 新参考下残差清零，跟随方向正确（Rx(180°) 倒装语义不反） |

全程 rviz 录视频（Imu 显示 + 臂运动 + `/imu_follow/target_pose` 若 P0-5 补上）。

### P3 数据分析与修正回归（贯穿，集中两轮）

每轮数据三面归集：`collect_round.py` + bag_report + 本方案 §8 清单核对。发现的 A/B 级问题按 §8 建议修 → **修正后必须重跑受影响的矩阵档**（网格 M1 全量 + 抽 M3 10 例回归），回放塔 `colcon test --packages-select peach_system_tests` 必绿（护栏改动必须过 replay_baselines 判定语义：analytic_ok 只许升）。

### P4 真机验证门（全部通过后申请授权）

见 §9；真机档位、命名、流程沿用 testing.md「预抓取全程测试就绪单」，本方案不重复，只加：双末端分别门（hollow 全链；adaptive 真机也先 IMU 跟随再接触——真机 IMU 对齐在真臂上重做 I1/I7）。

---

## 5. ros2 bag 全量录制方案（上限 100G）

### 5.1 配置

- **预算已是 100G**：`src/peach_observability/config/observability.yaml:40 max_total_bag_gb: 100.0`。超限自动回收最旧 `session_*/bag`（文本/账本/报告永不删，逐条审计 `runs/retention_audit.jsonl`）。
- **档位策略**：长跑（P1 场景轮、P2-b 批次链）用默认 `record.level: std`（相机 raw 限 1Hz，计算话题全量）；**失败短抓/问题复现轮切 `all`**（stereo≈50MB/s，100G≈8.5h 累计，短抓几十分钟内无压力）。切法：yaml 改后重启，或 `ros2 param set /peach_observability record.level all` 后重启节点。
- 会话内无分片机制：一段 bag 一路长到回收线。**分段靠轮次组织**——每轮/每批一个 session（栈启停即开合），不要开一次栈连跑一整天。

### 5.2 生命周期纪律（防 09-21 式堵流事故）

1. 每轮结束停栈 → 自动出 bag_report → `collect_round.py` 归集 → 分析结论写入 `runs/field_test_<日期>/log.md` + reports。
2. **分析完成后立刻 `python3 scripts/purge_analyzed_bags.py runs/session_<该轮>`**（或阶段性 `--all-reported runs`）。磁盘仅 171G free，purge 节奏必须跟上，不能全靠 100G 兜底。
3. 不另起裸 `ros2 bag record`（除非临时诊断，且必须 `timeout -s INT -k 10` 包裹 + 结束 pgrep 复核）——会话 bag 机制已覆盖全量需求。
4. 回放分析在**另一 ROS_DOMAIN_ID** 或停栈后进行，避免回放流污染在线域。

### 5.3 每轮复盘最低完备集（已有机制，汇总在此）

① 会话 bag（含 `/rosout` 全量日志回放）；② `~/.ros/log/<launch 时间戳>/launch.log`；③ `runs/<request_id>/`（ledger + 感知 events）。`collect_round.py` 一键归集成 `round_report.md`。

---

## 6. rviz2 视频录制方案

### 6.1 录什么

`aubo_e5_moveit_config/rviz/moveit.rviz` 现役布局已含：Debug Image（检/分割叠加）、Perception/Reconstruction Markers（袋轴/入口箭头/包络/拟合球/ID）、单目标与整幅点云、TSDF、Planned Views、TCP Path、Imu 盒子、RobotModel+MotionPlanning（规划轨迹）。**布局即证据面**，录制时按轮次打开对应显示组：

| 轮 | 必开显示 |
|----|----------|
| P1 感知 | Debug Image + Perception Markers + single_cloud + Imu |
| P2-a 注入 | MotionPlanning（规划轨迹）+ RobotModel + TCP Path + Perception Markers（注入目标可视化） |
| P2-b 批次 | 上两行的并集（分屏：左 Debug Image 右 3D） |
| P2-IMU | Imu + RobotModel + `/imu_follow/target_pose`（P0-5 后） |

### 6.2 怎么录（P0-3 脚本封装）

```bash
# 原理：ffmpeg x11grab 抓 RViz 窗口；产物 mp4 落轮次目录（二进制，不进 git）
ffmpeg -f x11grab -framerate 30 -video_size 2560x1440 -i $DISPLAY \
  -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  "runs/<request_id>/video/rviz_<label>_$(date +%H%M%S).mp4"
```

- 命名与轮次 label 对齐（如 `P2a_M3_seed20260922`），便于 bag/视频/账本三方互查。
- 结束 `pgrep -af ffmpeg` 复核清理（MUST 同样适用）。
- 备选：窗口管理器级录制（如 `wf-recorder`）画质相同；不引入常驻依赖，脚本封装 ffmpeg 即可。

### 6.3 分析用法

视频与 bag 时间对齐方式：起录前后各说一声触发一个可检索事件（如 `ros2 topic pub --once /peach/observability/job` 或直接记墙上钟 + bag 起止），分析时用 bag_report 时间轴对帧。

---

## 7. 双末端差异执行细则

| 项 | hollow_cylinder_v1（固定圆柱刀） | adaptive_cylinder_v1（自适应+IMU） |
|----|------|------|
| M1 网格 | 全量 | 全量（验证档案注入/几何差异） |
| M2–M5 矩阵 | 全量 | **不跑**（裁定） |
| P2-b 批次链 | 全量 | 不跑（默认档案即 adaptive，B 轮显式传 `tool_profile:=hollow_cylinder_v1`） |
| IMU 跟随 | 不适用 | **P2-IMU 全部 7 步（真实 IMU + TF 对齐）** |
| 真机门 | 全链 | 先 I1/I7 真臂重对齐 → 跟随验收 → 才谈接触 |

档案切换纪律：整栈重启才生效；起栈后核 `ros2 param get /peach_arm tool.profile_id` 与 launch 参数一致（P0-1 修复前，注入脚本在 hollow 档会写错标签——先修再跑）。

---

## 8. 不合理设计与约束修正清单

> 依据三份源码探读报告。分级：**A=妨碍本战役目标正确性，P0/P2 前处理**；**B=影响分析保真度，P3 集中处理**；**C=真机语义/记录在案，本轮不修**。每条给证据与建议方向；动手前按 AGENTS 流程（检索→主流做法→同轮改活文档）。

### A 级（战役正确性）

| # | 问题 | 证据 | 影响 | 建议 |
|---|------|------|------|------|
| A1 | 执行期其余锁定目标被「断粮」：锁定后 SAM 只分割 executor 当前目标，其余目标 30s `anchor_stale`、120s 被 drop | `pipeline.py:283-296`；`yaml:62-63` | 多目标批次结构性劣化（B4 专测量化对象）；P2-b 大批次统计失真 | 短期：实验设计按「批内目标数 ≤3」+ B4 量化；中期：锁定集分割范围放宽为「锁定集∪当前目标」或感知侧对锁定集做低频保活分割 |
| A2 | 非 selected 目标的预抓取残差修正被废：修正回路取 selected 缓存，ID 不符直接停修 | `stages.cpp:925-933` | 批次中第 2+ 颗残差超门只能失败，多组抓取成功率被压低 | 修正回路改取「周期生效目标的锁定集快照」（与 cycleTargetSnapshot 分流一致） |
| A3 | 调度资格谓词与感知不一致：不查 REJECT/swinging/tf_stale；`camera_distance_m==0` 绕过深度窗 | `batch.py:104-114,172` vs `identity.py:237-264` | 坏目标被 claim 后在臂侧才失败，浪费批次时间且污染统计 | 谓词对齐感知 `_selectable`；深度 0（锚点缺失）按超窗处理 |
| A4 | 注入工具多样性不足 + `--tool-profile` 未接线：random 档袋长/行程取模板、袋径固定 0.06、无遮挡维度；profile 两处硬编码 | `sim_field_targets.py:278-351,744,977` | M2–M4 覆盖面不足；hollow 档账面污染 | P0-1 接线；P0-2 扩 grid；random 档加直径/长度采样（±30% 抖动即可起步） |
| A5 | 记忆锚点袋长=直径的伪造几何：LOST 目标回填 `0.5×(diameter or 0.06)` 当半长 | `identity.py:1188-1197` | 断粮后的批次第 2/3 颗用 fabricated 几何进接触，统计不是实测 | 与 A1 同解（保活分割减少 LOST）；短期在 ledger/事件标注 `anchor_from_memory` 占比，分析时分离 |
| A6 | `unrefined_hold_` 锁存跨批短路：skip 模式下同 ID 跨批时新证据被忽略 | `target_cache.cpp:446-495,112-150` | B3 连续批次第 2 批同 ID 目标可能用旧几何 | hold 的解锁面扩到 `updateLockedTargets`/新 scene_epoch |

### B 级（分析保真度，P3 处理）

| # | 问题 | 证据 | 处理方向 |
|---|------|------|----------|
| B1 | reconfirm 参考锚自引用（随最新帧走，注入式跳变宽容） | `stages.cpp:621-624` | 参考锚固定为 finalize 时锚点 |
| B2 | model_snapshot 不续签：同 target 重发新 valid_until 不更新，重派须换 model_revision | `manipulation_skills_node.cpp:890-907` | 实验脚本遵守；或决策同串也更新 valid_until |
| B3 | scene_epoch 感知侧恒 0，元组字段形同虚设 | `cycle.cpp:660-662` | 接 BeginScene 返回值或先记录为已知限制 |
| B4 | 事件写盘队列满时在 plan_lock 内同步降级写 | `runtime.py:221-226` | 降级写移出锁；或 dropped 计数 + 告警 |
| B5 | 帧数口径参数（confirm/TTL/settle/collect）跨前端墙钟差 ~5.5× | `scene_perception.yaml:60-61` | S6 实测后决定是否按帧率自适应 |
| B6 | confidence 无标准化口径（三因子乘积，写死 magic number） | `pose_pipelines.py:499-501` | P1 用实测分布定义工程口径，再决定是否进门 |
| B7 | 恒真退化条件（疑似死代码）：本帧无 payload 就回填记忆锚点 | `plan_updater.py:322,85-98` | 恢复原意「上帧几何退化才回填」 |
| B8 | lighting 门不阻断 | `yaml:69`；`plan_updater.py:268-275` | P1 自定门：`lighting.low_quality` 连续 N 秒冻结派发（调度侧小改） |
| B9 | CheckReachability 2s 超时静默回退半径窗 | `executor_node.py:1778-1799` | 回退时 ERROR 级事件 + ledger 标记 |
| B10 | 感知丢帧不可观测（BoundedWorker.dropped 无人读） | `runtime.py:95-120` | P0-4 暴露 |

### C 级（真机语义 / 记录在案，本轮不修）

| # | 事实 | 证据 | 口径 |
|---|------|------|------|
| C1 | `cut_confirmed` 结构性不可达（无 confirmFeedback 调用点） | `stages.cpp:1098-1102`；`tool_actuator.hpp:47-63` | 仿真成功=completion_level≥SLEEVE_COMPLETED；真机 KEEP |
| C2 | mock 开 `tool.enabled=true` 必挂 TOOL_STATE_UNKNOWN 且不自动撤退 | `stages.cpp:1087-1092` | 仿真永远 tool=false；⚠️ 勿在 mock 试开 |
| C3 | 臂侧审查几何（0.06/0.2）不随档案注入 | `peach_arm.yaml:121-124` | 当前两档案外径同，无实害；加新档案时必须同步 |
| C4 | `L_blade=0` → cutting_plane/sleeve_mouth/tcp 塌缩同点，「刀口对颈」残差是冗余测量 | `tcp.xacro:9,20-39`；`pregrasp_residual.hpp:66-79` | 真机刀档案落地时重审 |
| C5 | staging 首段 PTP 不查反爬（拍照位在果上方的先验） | `grasp_task.cpp:1009` | M4 近水平/低位袋压测时人工看首段弧 |
| C6 | robot_status 门 mock 关闭、电流止损 mock 无效 | launch:183-187；`motion.cpp:86-88` | KEEP；真机门 P4 重验 |

### 明确保留的约束（不是缺陷，勿在战役中「顺手改掉」）

停走式相机节拍（产品模型）；滚转表 ±30°/±60°；果实胶囊回退 0.12m；接近三门 2.6/0.32/0.12（09-18 标定）；关节门 12/6.1；`PREGRASP_ONLY` 默认；仿真 `tool.enabled=false`；`require_robot_status` 真机 KEEP true；`min_views=2`/`max_target_drift_m` 不放宽（testing.md 明文）。

---

## 9. 验收门汇总（建议值，P0-6 基线后定稿）

### P1 感知门（每前端 × 每场景）

| 门 | 建议阈值 | 数据源 |
|----|----------|--------|
| 锁定时延 | percipio ≤8s / stereo ≤2s（confirm 5 帧 + 收齐窗） | events `global_targets_locked` 时间差 |
| 稳态抖动（3σ） | entry/bottom/neck 位置 ≤5mm；轴角 ≤1.5°；袋长 ≤8mm | bag 回放复算 |
| 身份 | 10min 零 ID 切换/重复注册；`target_swinging` 静态误报 0 | events + markers |
| 新鲜度 | `tf_stale` 率 <1%；`anchor_stale` 误判 0 | harvest_state |
| 连续性 | `frame_observations` 无 >2s 缺口；丢帧计数（P0-4）=0 或解释 | events + BoundedWorker |
| 光照 | S2 场景 low_quality 触发后恢复 ≤10s；无错误几何进锁定集 | harvest_state.lighting |

### P2 抓取门（hollow 档）

| 门 | 阈值 |
|----|------|
| M1 网格 | 期望分类 100% 一致 |
| M2 PREGRASP | ≥95% `SUCCEEDED`+recovery_required（对照 09-11：26/30=87%，门收紧前先达对照） |
| M3 FULL | ≥90% completion_level≥6；绕行比 ≤1.70、姿态 ≤71%（对照不变） |
| 失败归因 | 100% 有 failure_code；无 300s 超时 hang；`--pick` 重测后不可解释例=0 |
| M4 压测 | 产出分布报告；近水平袋（axis_z<0.3）不进入 P4 范围声明 |
| B2/B3 批次 | 事件链完整、账本零缺口、ACK 语义正确、rework 清单正确 |
| 回归 | 回放塔绿；`analytic_ok` 不降；修 A 级问题后 M1 全量 + M3 抽 10 例重跑 |

### P2-IMU 门

I1 TF 对齐正常（帧率/静止比力/对齐残差清零）；I2 静置零漂移；I3 跟随误差收敛且 servo status=0；I4 锥钳/平滑生效无甩动；I5 断流 0.5s 自动 disable + 刹车零速；I6 推进匀速可停；I7 重对齐后方向正确。

### P4 真机前置门（全部满足才申请授权）

P1（至少 stereo 前端）全过；P2-a M1–M3 全过且失败例归因清零；P2-b B1–B3 全过；P2-IMU 全过；遗留 A 级问题要么已修要么在报告中有量化结论与真机风险声明；§8 C 级已在真机计划中列对应检查项；工作分支已提交推送（Gitee 备份，09-22 修复链勿再单副本）。

---

## 10. 运行纪律、风险与边界

- **安全**：本轮全程 mock/仿真，无真机运动、无 SetIO；`tool.enabled` 全程 false（C2：mock 开 tool 会挂死且不撤退）；adaptive 的 `motion.enabled` 只在 P2-IMU 且无 peach 执行时开。
- **进程卫生**：每轮前后 `pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'`；残留按 PID 清；ffmpeg 一并纳入复核。
- **磁盘**：171G free / 82% 已用——100G 预算贴着上限，P0-7 先清存量，P1 起每轮 purge。
- **单副本风险**：改动只在工作分支时及时提交推 Gitee（既有教训）。
- **20s 阻塞纪律**：批量矩阵跑长轮用后台任务+通知；单命令探针 ≤18s peek。
- **文档同轮**：P0-1/P0-2 等改动落地时，若行为变化触及三份活文档口径（如注入工具成为正式系统测入口），同一轮改 architecture/testing 对应行；testing-log 追加轮次记录。

---

## 11. 命令速查

```bash
# ── 通用前清/后清（MUST）─────────────────────────────────────────
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe|ffmpeg'

# ── P1 感知轮（mock+真相机，先 Survey 到拍照位）──────────────────
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true autostart:=false
ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
  "{execution: true, grasp: false, tool: false, reason: 'P1 perception'}"
ros2 action send_goal /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'e2e_survey_YYYYMMDDTHHMMSS', scene_key: 'lab', profile_id: 'default', intent: 2}"

# ── P2-a 注入矩阵（无相机）───────────────────────────────────────
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=false skip_reconstruction:=true imu_enabled:=false \
  tool_profile:=hollow_cylinder_v1 autostart:=false
python3 scripts/sim_field_targets.py --grid --mode full --velocity 1.0 --tool-profile hollow_cylinder_v1
python3 scripts/sim_field_targets.py --random 30 --seed <s1> --mode full --velocity 1.0

# ── P2-b 批次链 ──────────────────────────────────────────────────
ros2 param set /peach_supervisor execute_pregrasp_only false   # 仅 FULL 干跑轮，测完改回
ros2 action send_goal -f /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'e2e_full_unrefined_YYYYMMDDTHHMMSS', scene_key: 'lab', profile_id: 'default', intent: 0, view_policy: 0}"

# ── P2-IMU（peach 使能全关，另起 servo）──────────────────────────
ros2 launch imu_follow imu_follow_servo.launch.py
ros2 service call /imu_follow/enable std_srvs/srv/Trigger
ros2 param set /imu_follow motion.enabled true        # mock 自由
ros2 service call /imu_follow/disable std_srvs/srv/Trigger

# ── 归集与回收 ───────────────────────────────────────────────────
python3 scripts/collect_round.py <request_id>
ros2 run peach_observability peach_bag_report runs/session_*/bag
python3 scripts/purge_analyzed_bags.py runs/session_<该轮>
```
