# peach 感知→抓取全链路逐函数深度审查与优化方案 v1.0

- 日期：2026-09-20　分支：test/20260909-field-traj
- 范围：peach_harvester（vision 两节点 + supervisor）、peach_arm、peach_interfaces（ExecuteTarget）、peach_vegetation、peach_observability（消费端）；不含驱动九包与 peach_stereo（其两处未提交修复原样保留，本文不涉及）。
- 方法：主链关键文件全文亲读（scene_perception_node 836 行、target_reconstruction_node 2119 行、manipulation_skills_node.cpp 1218 行、cycle.cpp 559 行、stages.cpp / grasp_task.cpp 关键区段、ExecuteTarget.action 全文、yaml_params 三副本逐字节 diff）+ 四路并行逐函数盘点（感知纯核 / 重建纯核 / supervisor / 臂侧余量），全部结论带 文件:行号 证据。
- 前提：**核心算法零改动**——staging 三层接近、果胶囊门、令牌双路命令门、harvest_fsm 查表、RANSAC/ICP/TSDF/refit/匈牙利/YOLO/SAM 数学、全部自适应超时公式数值、三段式锁协议、话题/服务/动作名、参数键名、QoS 语义。
- 主流对照（提交说明引用依据）：Nav2 节点/纯核分工 + nav2_common + diagnostic_updater；UR Driver GPL 参数单源；Eigen::Quaterniond::angularDistance；cv_bridge desired_encoding；Autoware 接口清单双向核对。

---

## 0. 结论摘要

问题收敛为六类，全部有逐条对应解决内容（见 §2–§7）：

| 类 | 量化 | 解决方案落点 |
|---|---|---|
| A 节点内嵌业务 | recon 节点 ~700 行、scene 节点 ~230 行、arm 主节点 ~460 行、executor_node ~600 行 | §2 各节点下沉纯核模块 |
| B 配置转抄 | arm params_→50 成员→两 Config 三遍 ≈350 行；recon `_session_metadata` 94 行手抄 | toXxxConfig 单点 + params snapshot() 自省 |
| C 手写重复设施 | yaml_params ×3（零 diff）、param_rules ×4、QoS 手写 ≥6、深度反投影 ×3（数学同构、距离窗语义差异见 §3.4）、ACM 函数 ×2、夹角函数 C++ ×4、target_cache 调和 ×2（90% 重复） | peach_common 单源 + 库替换 |
| D 接口债 | ExecuteTarget 三重镜像 + deposit 恒 false；ledger 白名单与 extra 断链 | §4 六步收敛 |
| E 死代码/死参数 | §5 总清单（≈30 项） | 随各 Phase 清理 |
| F 行为缺陷（新发现） | 高危 7、中危 9、低危若干 | §6 分级修复（先测后修） |

---

## 1. 五条主流程函数级 walkthrough（现状基线）

### 1.1 帧感知流（相机 → target_observations）
1. 三话题 `message_filters.ApproximateTimeSynchronizer`（slop=0.05s）→ `scene_perception_node.py:634 _on_rgbd` → `BoundedWorker(cap=1, drop_oldest)`（runtime.py:76）。
2. worker 线程 `_process_rgbd:725` → `_decode_rgbd:641`：cv_bridge 解码 → `normalize_depth_to_uint16_mm`（geometry.py:575）→ `_lookup_T_out_cam:594`（tf2 精确 stamp→失败回退 latest 打 tf_stale）→ `gravity_camera_from_R` → 组 `SyncedRgbd`。
3. `PerceptionPipeline.process`（pipeline.py:288，~205 行）：`UltralyticsYolo.detect`（inference.py:86）→ `filter_detections`+IoS 去重（inference.py:577）→ 锁定集 `segmentation_bboxes`（pipeline.py:264，含裸 `np.linalg.inv`）→ `MobileSam.segment`（inference.py:175）。
4. 逐检测：`_crop_mask_to_bbox` → `build_masks`（SAM∩有效深度∩膨胀连通域）→ `RobustBagPosePipeline.estimate`（pose_pipelines.py:155）：`_to_points` 反投影#1 → `estimate_normals`（geometry.py:96，内联反投影#3）→ `fit_cylinder_robust` → `estimate_bag_landmarks`（bag_landmarks.py:261）→ 入口/行程（contracts.py:165/187）→ 协方差（identity.py:43）。
5. tf 旗标注入 → `_apply_T_to_grasp3d` 换输出系（pose_pipelines.py:42，节点:38 私有 import）。
6. 身份：`match_or_register_frame`（identity.py:891）→ `assign_detections` → `solve_hungarian`（scipy）→ `_commit_match`（EMA/摆动/确认）。
7. 逐结果消息组装（visualization.py `_to_*`，pipeline 反向私有 import）→ `PerceptionResult`。
8. 节点 `_publish_frame:742`（纯 ROS I/O，形态良好）→ `_publish_target_observations:395`（167 行业务在节点，问题见表 §2.1）→ `_publish_harvest_state`。

### 1.2 重建流（采帧 → finalize → refit → 发布/Build）
- 采帧：`_on_rgbd:580` → worker(cap=3 不丢) → `_process_rgbd:589` → `session.process`（深度门）→ 帧环 `_push_frame_ring` → `_auto_drive`（AutoControllerMixin，capture.py:860）→ 三段式门禁 `_gated_capture_begin/query_tf/finish`（node:997/1024/1045，锁内-锁外-锁内收口）→ `_auto_capture_commit` → `_prepare_frame`（构云 `build_cloud_base` + ROI 裁剪 + `BoundedIcp.refine` 粗细两层）→ `_commit_prepared_frame`（锁内入库 + `_integrate_tsdf` + `IcpTargetCache` 增量复用/全量刷新）→ 锁外 `_publish_all`。
- finalize：三入口（Trigger 服务 / 自动满栈 / Build 主体）→ `_finalize_now:1288`（机位门 `summarize_view_coverage` → `collector.finalize` → overlap → `_run_tsdf:1364`）→ `_run_refit:1394`（`select_refitter` → Cylinder/SphereRefitter → `_collect_bag_views:1494` → `fuse_bag_views`（refine.py:788，34 键裸 dict）→ `_merge_fused_bag_model:1529`（59 行 dict 覆写）→ 成对写 `_refined/_bag_model` + `_log_geometry_row`）→ `_publish_all`（PublisherMixin，闩锁 12 话题 + PublishThrottle 节流）。
- Build：`_on_build_target_model:1655` → `_build_target_model_body`（reset hack :1676 → 强制 COLLECTING → `_wait_min_views:1612`（Event.wait，但 `time.monotonic` 直取 :1616/:1648 违反协议 I3）→ `_finalize_now` → `_fill_target_model:2016`）。

### 1.3 批次调度流（RunHarvest → 选果 → 派发 → 账本）
1. `executor_node.py:512 _goal_if_active`（Active/栈/占用门）→ `:728 _run_harvest`（复位/策略/`react(WAITING_READY,RUN_REQUESTED)`）。
2. 主循环 `:785 _run_harvest_body`（22 分支、180 行）：每轮查取消 → `_wait_pause:1946`（无界设计性）→ SELECT/DISPATCH/EXECUTE_FULL 前过 `_wait_recovery:1982`（ACK 门）。
3. SURVEY → `_survey_body:1660`（SurveyScene + 回访 dwell + `_publish_scene_snapshot`）→ 首轮后 `_restore_ledger:1906` 断点恢复。
4. SURVEY_AT_POSE → `_cmd_begin:975`（BeginScene）→ `_wait_lock:1627`（等锁定 15s）。
5. SELECT：`ratio_reached` → `_query_reachability:1740`（批量 IK 0.1s/pose）→ `next_target`（batch.py:87：preferred 直通→深度窗∩IK→排序）→ `claimed.add` + `TargetDeadline`。
6. DISPATCH → `_cmd_dispatch:990`：BuildTargetModel goal → `_wait_build_started`（2s）→ fast 档 `_fast_observe_loop:1363`（直驱 MoveTo 补视）或 OBSERVE_ONLY×4 → `_wait_build_after_observe` → READY_FULL。
7. EXECUTE_FULL → `_cmd_full:1174`：clearance 装配自 `_decision_cache` → `_send_action` ExecuteTarget（180s 名义）→ 结果入账（§4 消费表）→ `_push_outcome` → `event_for_outcome` → `_react` → 循环回 DISCOVERY。
8. 每命令收口 `_persist_ledger:1916`（batch.py `save_ledger` 原子写）→ 终局 `build_summary` + finally `ReworkList.save`。
- 已知缺陷位：`:719-725 _react` 恒传 standing txn → 终局后事件被 `_stale` 第三条款丢弃（§6 S1 活锁）。

### 1.4 执行周期流（ExecuteTarget → 阶段序列 → 透传）
1. `cycle.cpp:114 onActionGoal`：Active 门 → 模式/占用/recovery → FULL|PREGRASP_ONLY 身份元组（model_contract.hpp:41）→ 锁定集 gateSample / selected 命中。
2. `:185 onActionAccepted`（join 旧线程起新线程）→ `:196 executeAction`：PREVIEW 分流 `previewContact`；其余建 `CycleContext`（clearance 令牌五字段 :263-272，fresh_window=`effectiveTargetMaxAgeS`）→ `onStart:360`（CAS running_、目标快照复核）→ worker 线程 `executeCycle`（stages.cpp:238）。
3. 阶段链：PrepareCycle(:318，goal 钉死校验) → PlanPreview(:376) | AcquireViews(:399，ScanBudget 停准 + waitForFresh/NewStation) → FinalizeAndValidate(:607，质量门+`contactEntryGeometry`) → ReconfirmTarget(:683，ReconfirmPolicy 窗口环) → MovePregrasp(:802，先回拍照位→`grasp_task_->moveToPregrasp`) → VerifyPregrasp(:879，三帧 TF 残差 1.5°/2.0°/0.003 硬编码 :919-920 + 修正回路≤2) → [PREGRASP_ONLY 停驻 :997] → PlanSleeve(:1014) → SleeveLinear(:1037) → VerifyCutHold(:1079 空阶段) → ActuateCutter(:1085，ToolActuator→SetIO) → VerifyCut(:1125) → Retreat(:1139) → Stow(:1166) → VerifyHarvest(:1187) → Complete(:1202)。
4. 授权单点 `authorizeStage`（cycle.cpp:49-112）：TRANSIT/PREGRASP→`authorizeTransit`；CONTACT 令牌双路（allowed/绑定 target_id/valid_until/model_stamp 窗）或 GraspDecision 快照复检；TOOL 叠加 tool_enabled。
5. 终局：executeCycle 四级判定（recovery>取消>成功>失败）→ `fillStageDurations` → `fillExecuteResults`（manipulation_skills_node.cpp:1168，三重镜像填充）→ succeed/canceled/abort。
6. GraspTask 内部：`classifyApproach`（grasp_task.cpp:106 分档）→ `tryStagingTransit:747`（stagingCandidate:729 → 节点侧 `select_goal_joints` lambda :500-606）→ `makeStagingSequence`（Pilz PTP+轴向 LIN）→ `planTaskOnly:962` 四类护栏级联（关节行程 12/6.1rad → 果胶囊逐段 → 笛卡尔绕行 → 姿态行程 cap）→ `executeSolution` → move_group → 透传控制器。

### 1.5 结果回流与 IO 面
- ExecuteTarget Result 消费：executor_node.py:1234-1269（outcome_record 优先、顶层 4 bool 写 extra、deposit 并 reason）；observability/pipeline.py:28-29（ledger 行透传白名单，但 batch.py:294 白名单不写这些键——断链 §6 S4）；debug_actions.py:251（hasattr 泛化）。
- 重建产物：/peach/reconstruction/* 12 话题（闩锁+deadline 1.5s on diag）+ BuildTargetModel TargetModel。
- 账本：runs/\<request_id\>/ledger.json（原子写）+ rework_list.json（非原子 write_text）+ perception_data/sessions/geometry.jsonl 三套落盘。

---

## 2. 逐函数审查与优化对照表

> 格式：`位置 | 函数 | 问题 | 优化内容`。「—」= 审查通过保持现状。仅列有实质内容的行；纯数据类/薄透传且无问题的函数不逐行罗列（其健康度已在 walkthrough 体现）。

### 2.1 scene_perception_node.py（836 行 → 目标 ≈500）
| 位置 | 函数 | 问题 | 优化内容 |
|---|---|---|---|
| :395-561 | `_publish_target_observations` | 167 行业务：帧率自适应窗伸缩 :407-423、OUT_OF_VIEW 预分类 :424-431、plan.update+drop 记账 :432-446、光照统计 :448-468、逐字段组装 :470-553 | 下沉纯核 `plan_updater.py`：`PlanUpdater(pipeline, params, clock).update(header, mask_header, records, payloads, scene_epoch, executor_state) -> PlanUpdateOutcome`；节点只做 token→msg 映射+publish+save_mask 回调 |
| :291-320 | `_harvest_state_dict` | JSON 快照组装在节点 | 移 plan_updater（状态投影纯函数 `harvest_state_dict()`） |
| :360-393 | `_start_harvest_run` | manifest 组装在节点 | HarvestDataStore 领域方法 `start_harvest_run(plan, params, executor_run_id)` |
| :563-592 | `_degenerate_candidate`/`_fill_memory_anchor` | 几何判据+记忆回填在节点；:38 私有跨模块 import `pose_pipelines._rotation_to_quat`；identity.py:1162 lazy import 形成循环依赖 | 回填逻辑归 identity.py（`memory_grasp` 已在此）；`rotation_to_quat` 已在 geometry.py:713，删除私有 import |
| :133-175 | QoS 内联 ×3 | 手写 profile | `peach_common.qos.latched()/stream()` |
| :167-168 | `~/begin_scene` | 服务与同步回调同默认互斥组+单线程 spin | 服务独立 callback group；`MultiThreadedExecutor(2)`（单线程 spin 靠 worker 兜底现状保留为兼容说明） |
| :634-650 | `_on_rgbd` 偏差告警 | 手写二次校验（slop×80%） | 保留（防御式合理），阈值常量命名 |
| :531-541,556 | plan_lock 内 `cv2.imwrite`/events 追加 | 磁盘抖动拖住整帧+BeginScene（§6 V4） | HarvestDataStore 增加有界异步落盘队列（worker 外独立 I/O） |

### 2.2 感知纯核（问题项；其余审查通过）
| 位置 | 函数 | 问题 | 优化内容 |
|---|---|---|---|
| pipeline.py:288 | `process` | ~205 行巨函数；:36-52 跨模块私有 import；debug 双整图 copy :308-309；bbox_touches_image_edge 同帧算两次 :403/:425 | 尾段 records/payloads 组装移 plan_updater 后自然缩短；私有符号升公开 API；debug copy 复用；edge 结果复用 |
| pipeline.py:140 vs inference.py:490 | `_crop_mask_to_bbox`/`_crop_mask` | 裁剪原语两份 | 合一进 geometry（`crop_mask_to_bbox`） |
| pipeline.py:283 | 裸 `np.linalg.inv` | 与 geometry.invert_transform 并存 | 统一走 geometry |
| identity.py:979/985/1039 | 临时 id 拼接 | `frame_index+len(frame_used)` 可撞号（§6 V3） | 单调计数器 |
| identity.py:876-889 | TTL/墙钟淘汰 | 不同步清 `pipeline.bbox_at_edge`（缓慢泄漏 §6 V2） | 淘汰时同步删键或随帧重建 |
| identity.py:730 | `SpatialEmaMatcher` | 名不副实（match 已删） | 改名 MatchRadiusConfig |
| inference.py:208-214 | `MobileSam.segment` | 内部吞异常返回 []，pipeline:339-348 逐框回退成死路径（§6 V1） | 内部记录不吞、异常上抛由上层回退 |
| inference.py:290-308 | FOREGROUND_MODES 注册表 | 单模式过度设计；MODE_LABELS 无消费者 | 压缩为常量 |
| inference.py:483 | kernel 每目标重建 | getStructuringElement 热路径 | 构造一次缓存 |
| inference.py:422/439 | `time.perf_counter` 自计时 | 与注入时钟双轨 | 计时统一注入时钟 |
| pose_pipelines.py:12 | 顶层 import geometry_msgs | 与纯核 docstring 矛盾（§6 V5） | `_rotation_to_quat` 消息包装移 msg_builders |
| pose_pipelines.py:42 | `_apply_T_to_grasp3d` | 点字段手工枚举；私有被 import | 字段名表数据驱动+公开 `apply_transform_to_reference` |
| pose_pipelines.py:437 | `_to_points` | 手写反投影#1；:454 重复算 _valid_depth（valid_roi 已有） | `geometry.backproject` 替换+复用 valid_roi |
| pose_pipelines.py:619 | `_project` | 手写投影 | geometry 补 `project_points` 对偶 |
| pose_pipelines.py:670-941 | 果线 estimate | 与袋线 ~120 行同构脚手架 | 模板方法抽公共骨架（只动结构） |
| pose_pipelines.py:941 | `_failed_fruit` | 与 `_failed` 仅 kind 差异 | 参数化合并 |
| visualization.py 整体 | — | 消息组装与像素绘制混杂；下划线函数被 pipeline/node 反向 import | 拆 `msg_builders.py`（公开 API：to_detection2d/to_candidate/to_candidate_2d/to_fitting/to_markers/xyzrgb_to_cloud_msg）+ `debug_draw.py` |
| visualization.py:35 vs :293 | `_pack_rgb_bgr` 双赋值 | 死代码 | 删 :293 |
| visualization.py:296 | `_bbox_cloud_xyzrgb` | 手写反投影#2 | `geometry.backproject` |
| visualization.py:44 vs identity.py:244 | STATUS_MAP vs _STATUS_REJECT | REJECT=2 两处定义 | 常量单源（peach_common） |
| visualization.py:421/:608 | 两套三态配色 | 并行 | 单源配色表 |
| image_gates.py:52 | Rodrigues 往返 | 可直乘 | 等价简化 |
| image_gates.py:116 vs visualization:324 | bbox 裁剪重复 | 两份 | 复用 clip_bbox |
| geometry.py:121-124 | `estimate_normals` 内联反投影 | 手写反投影#3 | `backproject` 复用 |
| geometry.py（新增） | `backproject(depth_mm, mask, K, *, min_depth_m, max_depth_m, stride) -> (xyz, rgb)` | — | 单一实现：距离窗参数化（感知线传窗、重建线开窗），替换三份 |
| contracts.py:42 | TOOL_GEOMETRY 全局默认 | 与 yaml 双源 | 显式标注"测试默认"，生产必经 yaml |
| contracts.py:211 | s_min=0.8 系数 | 魔法数 | 命名常量 |
| bag_landmarks.py:63 | `classify_occlusion` | 生产调用硬编码 neighbor_gap=1.0/edge=False → NEIGHBOR/贴边分支不可达 | 调用方传真值或裁死分支 |
| stream_metrics.py / pipeline.py:236-247 | EMA α=0.3 散布 | 字面量多处 | peach_common 单常量 |
| runtime.py:267 | save_mask `time.monotonic` | 违反 I3 | 注入 Clock |
| runtime.py:146-165 | runs-root 解析 | 与 supervisor/batch.py:208 双实现；observability 跨包 import 后者 | peach_common.paths 单源 |
| domain/tracking.py:10、observation.py:9 | 生产零引用；tracking 制造反向依赖 | 死代码 | 删（registry 由 scene_perception 导出） |
| domain/budget.py:33/44 | geometry 恒 VALID、ready_ok 恒 True | 占位语义 | 显式 TODO 标注或参数化 |
| domain/evidence.py:28/35 | corridor_* 仅测试引用 | 预留 | 标注预留或删 |

### 2.3 target_reconstruction_node.py（2119 行 → 目标 ≈900）
| 位置 | 函数 | 问题 | 优化内容 |
|---|---|---|---|
| :1086-1269 | `_register_cloud`/`_integrate_tsdf`/`_prepare_frame`/`_commit_prepared_frame` | 三段式锁协议核心 | **保留原位**；仅 `_collect_gate_values` dict 字面量(:974-992)提为 dataclass；:1146-1153 TSDF 回滚补 `_icp_target_cache.invalidate()`+版本递增（§6 R2） |
| :1288-1362/:1364-1392/:1394-1492 | `_finalize_now`/`_run_tsdf`/`_run_refit` | 编排+算法在节点 | 下沉 `refit_orchestrator.py`：`RefitOrchestrator(refitters, refit_config, timing).run(tsdf_cloud, frames, kind_memory, axis_hint, budget_params, keep_last_good, mark_final) -> RefitResult`；成对写入约定注释(:1463-1467)原样随迁 |
| :1494-1527 | `_collect_bag_views` | 纯算法在节点 | 移 refine.py（与 `fuse_bag_views` 同居） |
| :1529-1587 | `_merge_fused_bag_model` | 59 行裸 dict 覆写；N1 axis_angle 覆写；N6 kind 强制 'cylinder' | 移 refine.py 改纯函数；RefitResult 分 `refit_axis_angle_deg`/`fused_axis_angle_deg` 两字段；kind 保留原值、融合信息另置 flag |
| :1612-1653 | `_wait_min_views` | `time.monotonic` :1616/:1648 违反 I3 | 换注入 `self._algo_clock.now()`；Event 机制不动 |
| :1655-1742 | `_build_target_model_body` | :1676 直接构造 Trigger.Request/Response 调 `_on_reset` | reset 核心提为 `_reset_session()` 共享方法，服务 handler 与 action 都调 |
| :1744-1776 | `_on_save_session` | 持 `_state_lock` 全程写盘（多帧 PNG+PLY 秒级，阻塞心跳违约 1.5s deadline，§6 R4） | 锁内快照、锁外写盘（session_recorder 承接） |
| :1791-1884 | `_session_metadata` | 94 行手抄参数快照 | params 层 `snapshot()` 自省（yaml_params 已有全量叶子） |
| :1886-1963 | `_log_geometry_row`/`_log_view_geometry` | 节点直接写 jsonl、在 `_run_refit` 主链锁内 | `session_recorder.py`：`SessionRecorder.geometry_row(RefitResult, fused)/view_row(landmarks, frame)`，追加式单点写 |
| :1965-2014 | `_lookup_tool_frame`/`_pregrasp_verification_msg` | latest TF 豁免散在节点；N4 双路竞态（心跳持锁 vs `_publish_all` 锁外，`_pregrasp_prev` 互冲） | 移 publish.py；`_pregrasp_prev` 读写入锁、TF 查询留锁外；latest 豁免以显式函数名+注释保留 |
| :2016-2083 | `_fill_target_model` | 手拼协方差 :2075-2083 | 移 publish.py（`fill_target_model(model, fused, refined, params, now)`） |
| :834-933 | `_reset_products`/`_refresh_tsdf_outputs` | — | 随 RefitOrchestrator 组合注入（产物缓存归 Core） |
| Mixin 结构 | AutoControllerMixin/FrameStoreMixin/PublisherMixin | 隐式宿主属性契约（盘点：FrameStore 10 个、AutoController 13 个+11 个节点方法、Publisher 25 个+12 publisher，见 §3.3） | `ReconstructionCore` 显式组合对象（构造注入），Mixin 留薄壳继承过渡一个提交期后删 |

### 2.4 重建纯核（问题项）
| 位置 | 函数 | 问题 | 优化内容 |
|---|---|---|---|
| capture.py:80 | `classify_skip_reason` | 中文子串匹配，文案改即失配 | GateDecision 直接带稳定 code（字段已具备） |
| capture.py:889-895 | `_record_auto_skip` 锁外调用 | collector 计数无锁突变（§6 R1） | 移入 with 块统一锁域 |
| capture.py:908 | `_auto_start` | 锁内调 `_publish_all`（重活进锁） | 发布移锁外（COMMIT 路径已有同款先例 node:692-694） |
| capture.py:539/554 | `accumulated_cloud/rgb` | 每次全量 vstack（心跳 1Hz+发布重复） | 随帧入栈增量维护或发布前一次物化 |
| integrate.py:85 | `_make_rgbd` | RGBD 组装重复 | 公共 helper |
| integrate.py:732 | `Open3dCloudBuilder.build` | 单层转发 | 重构期可直接砍（调 build_cloud_base） |
| integrate.py:849 | `BoundedIcp._prepare` | 粗细两层重复降采样估法向 | fine 级缓存 |
| refine.py:284-378 | `_fail/_precheck/_gated_result` | 裸 dict 组装；N11 docstring 谎称写 entry | RefitResult 构造器；修 docstring |
| refine.py:386/409 | `_apply_axis_consistency` | axis_angle_deg 被 merge 覆写（N1） | RefitResult 双字段 |
| refine.py:728 | `_cut_station` neck_margin_m 形参 del | 死参 | 删 |
| refine.py:788 | `fuse_bag_views` | 34 键裸 dict、6 键无消费（cut_normal/fruit_prior_auxiliary/envelope_span_m/envelope_d95_m/detection_conflict_deg/axis_point） | `BagModel` dataclass，落成时裁死键 |
| publish.py:276-400 | `_dump_yaml/_write_ply/_write_triangle_mesh/save_session` | 文件 IO 混在发布模块 | 拆 `session_recorder.py`（对齐方向） |
| publish.py:106 | `PublishThrottle.reset` | 无调用方 | 删或接入 _reset_products |
| publish.py:568 | `build_refined_grasp_markers` | 60+ 行单函数 | 拆小函数（拆分时保持视觉输出逐字节不变） |
| publish.py:776-786 | `_lock_decision_validity` | `_locked_*` 三属性未在宿主 __init__ 声明 | 显式初始化（Core 注入清单项） |
| publish.py:789 | `_publish_all` | 不持锁读 collector.frames/accumulated_cloud（N7） | 帧栈 tuple(frames) 快照入锁后传出 |
| session.py:53-63 | `from_params` refitters 装配 | node:182 双持有同引用；无 ≥2 键断言 | 收敛单持有+启动断言 |
| session.py:78-82 | `process` k[0]/k[4] 硬编码 | 无长度校验（N10） | 校验后 reshape(3,3) |
| node:1300+1497 | coverage 重复计算 | O(V²)×2（N9） | 结果参数透传 |

### 2.5 supervisor（问题项）
| 位置 | 函数 | 问题 | 优化内容 |
|---|---|---|---|
| executor_node.py:719-725 | `_react` | 恒传 standing txn → 终局后事件全丢+CYCLE_DONE 无限重发+高频刷盘（§6 S1 高危） | 每事件生成新 txn id（`{gen}:{event}:{seq}`）或 settled 在下个命令效果到达时清除；配 FSM 单测固化 |
| :1690-1694 | `_send_goal` wait_for_server | 最长 180s 纯阻塞不可取消（§6 S2） | ≤5s 探测+重试+可 `_poke` 打断 |
| :1926-1932 | `_call_service` | 30+30s 不可取消 | 同上 |
| :460-474 | `_on_fire_step` PHOTO | 服务回调内同步 45s（违 20s 仓规）；未过使能门（§6 S3） | 异步化+`_execution_enabled_effective` 预检 |
| :407-425 | `_on_set_enables` | 不校验 tool→grasp→execution 依赖（check_enable_deps 死码在此接入） | 接入 param_rules.check_enable_deps |
| :1363-1370 | `_fast_observe_loop` | move 30s | 传参 min(timeout,18) 对齐仓规 |
| :785-967 | `_run_harvest_body` | 180 行 22 分支巨循环 | 拆命令处理器表（`_COMMANDS: dict[Command, handler]`）；行为不变 |
| :990-1173 | `_cmd_dispatch` | 180 行；fast 路径 OBSERVE goal 组装后不发（死装配） | 拆 `_dispatch_fast/_dispatch_observe`；删死装配 |
| :1440 | `_target_deadline_exceeded` | 仅两个入口检查，动作途中不复检 | 等待循环内并入时限 |
| :1903-1924 | ledger 三函数 | 恢复 details 空 dict/discovered 归零（S7）；异常路径不落盘（S5）；`_outcome_details` 断链 | details 随 ledger 落盘回读；finally 统一 persist |
| batch.py:284-302 | `outcome_to_dict` | 白名单不含 harvest_confirmed/cut_confirmed/retreat_confirmed/completion_level/failure_code_n → `_cmd_full` extra 全丢（S4） | 白名单补 failure_code_n/completion_level；三 bool 键随 IDL 收敛删除（§4） |
| batch.py:87-106 | `next_target` | preferred 直通绕过 eligible 谓词（S6：可选中未确认/裸果） | preferred 也过 `_eligible_locked_items` |
| batch.py:29-30 | `pregrasp_pose_of` | `not (x or y or z)` 误拒合法 (0,0,z) | is None 判定 |
| batch.py:241-246 | `_safe_run_component` | 不滤 `\0`/纯 `.`；非法 id 折叠到固定目录跨批混淆 | 对齐 observability sanitize；拒绝并报错 |
| batch.py:208 | `default_runs_root` | 与 vision 重复（§3） | peach_common.paths |
| harvest_fsm.py:250-260 | `react` 未知组合返回 Command.NONE | 被主循环当 CYCLE_DONE（S1 放大器） | NONE 语义拆分 NOP vs cycle-done（新增 Command::NOP） |
| harvest_fsm.py:263-320 | apply_event/EventHold 与 reducer 暂停分支 | 双份暂停处理 | 收敛一处（reducer 为权威） |
| reducer.py:117-131 | set_paused/set_recovery/begin_session | 节点从未调用 | 删或接线（选删） |
| domain/watchdog.py | 全模块 | supervisor 内无调用方（语义属 arm 侧） | 迁 peach_arm 或删 |
| domain/ledger.py | LedgerIndex | close 返回值/get 无消费者；跨批不重置撞号 | 删（或真正守护 persist 去重） |
| :2040 | `_publish_state` | 每次 _apply 全量 feedback | 限频（100ms） |
| :2019 | `_make_state` | execution_enabled 读本地参数非 override（投影失真） | 统一走 `_execution_enabled_effective` |
| lifecycle_manager.py:98 | `_run_command` | RLock 内全栈 RPC（锁粒度过大） | 缩小临界（自研件按阶段 6 计划删除前不动结构） |
| params attach 于 __init__:220 | 早于 configure | 挂 on-set 回调过早 | 移 on_configure（与 arm C++ 对齐） |

### 2.6 臂侧（问题项）
| 位置 | 函数 | 问题 | 优化内容 |
|---|---|---|---|
| manipulation_skills_node.cpp:291-395 | `loadParameters` | params_→50 扁平成员 | 删镜像成员（hpp:325-372），Params 直传 |
| :397-446 | `rebuildMotionInterface` | 30+ 键再抄 | `toMotionConfig(const Params&)` 单点（motion_config.hpp 纯函数） |
| :448-636 | `rebuildGraspTask` | 40+ 键再抄 + :500-606 `select_goal_joints` 107 行 lambda（5 种子/腕权 2.5/滚转惩罚 4.0/top5 硬编码） | `toGraspTaskConfig(Params)` + `StagingCandidateSelector` 纯核（include/peach_arm/staging_selector.hpp：`Config{seeds,wrist_weight,roll_penalty,top_n}` 进 GPL yaml `staging.*` 默认=原值；IK 回调注入保 KDL 互斥在节点侧） |
| :900-973 | 六个 `effective*`+`trackFrameInterval` | 超时族在节点 | `frame_timeouts.hpp：FrameRateTimeouts{onTargetFrame,frameWaitS,targetMaxAgeS,reconfirmWaitS,refinedWaitS(collecting)}` 公式逐字搬移 |
| :1088-1143 | setState/publishState | 手拼 JSON+callback_timing 同话题 | **status 保留**（web 消费端不动）+ 新增 `diagnostic_updater`（1Hz：TF/缓存新鲜度/队列/回调耗时/接触电流特征），CallbackTimingRegistry 并入诊断任务 |
| :1168-1216 | `fillExecuteResults` | 三重镜像填充 | 纯函数化 + IDL 收敛（§4） |
| stages.cpp:879-995 | `stageVerifyPregrasp` | 残差门 1.5°/2.0°/0.003 硬编码 :919-920；TF 轮询+几何+修正回路混杂 | `pregrasp_residual.hpp：PregraspThresholds{frame_consistent_deg,axis_deg,lateral_m}` 进 yaml `grasp.pregrasp_residual.*` + `ResidualReport evaluate(toolPoses, refined)` 纯核 |
| stages.cpp:355-367 vs 475-501 | ViewContext 组装两份重复 | — | `makeViewContext()` 辅助 |
| stages.cpp:1146-1148 | retreat 模式判定 | ReverseNominal 死分支+恒 WARN 噪音 | 删枚举与 WARN（retreat_policy.hpp 同步） |
| stages.cpp:1079-1083 | `stageVerifyCutHold` | 空阶段 | 删或接保持位判定（删优先，阶段序列注释同步） |
| grasp_task.cpp:349-476 | 两个 ACM 豁免函数 | 90% 重复；octomap_ns 死变量 :451-452；连杆清单三处硬编码 | 合并 `applyToolOctomapExemption(scene, policy)` policy∈{WHOLE_MAP,PER_TARGET}；连杆清单进 tool 档案参数 |
| grasp_task.cpp:185-216 | `tcpPathFromJoints` | planTaskOnly 双重 FK（:1020 全量+:1031 逐段） | 逐段结果复用 |
| grasp_task.cpp:962-1124 | `planTaskOnly` | 四类护栏级联内嵌单函数 | `ApproachInspector` 管道（JointTravel/FruitCapsule/CartesianDetour/OrientationTravel 各自纯函数 report；skip_tail 与 staging 首段旗标参数化）——数学不变 |
| grasp_task.cpp:226 | `setContactAcm` | pending_acm_* 跨调用隐式状态 | 改参数传入 |
| grasp_task.cpp:747 | `tryStagingTransit` | 候选失败原因不区分 | report 细分 IK 无解 vs 护栏拒 |
| motion.cpp:160 vs safety_gate.cpp:25 | staleness 双门双阈值（0.5/1.0s，两参数键并存） | 易漂移 | 单点化到 SafetyGate 或注释钉死分工（保守：先注释钉死+参数键合并入 yaml 注释） |
| motion.cpp:197-227 | `commandToolClose` | 越级 setState(FAILED) 不经 failStage（A1） | 返回原因由 stageActuateCutter 统一落账 |
| motion.cpp:328-410 | `onCheckReachability` | IK 50ms/单种子/滚转档与 staging 选择器不同源 | 共享 StagingCandidateSelector 配置对象 |
| motion.cpp:68-84 | 电流环形缓存 | `erase(begin())` O(n)；注释写 64 实为 128 | std::array 环形或 deque；注释对齐 |
| cycle.cpp:546 | executeSurvey scene_epoch=0 硬编码 | 编排器消费则恒零 | 接真实源（订阅 HarvestState 缓存） |
| cycle.cpp:290-312 | 反馈 200ms sleep 轮询 | 手写轮询线程 | 可换条件变量（低优先，行为等价） |
| cycle.cpp:185-193 | onActionAccepted join 旧线程 | 滞留则阻塞回调 | timed join + WARN |
| 头文件 | ViewPlannerBase/QualityGateBase/SafetyGateBase | 零第二实现虚基类；QualityGateBase 默认体与覆盖不一致（死代码） | 删基类去虚化（将来需要第二实现时按 AGENTS 走 pluginlib 新缝） |
| view_planner.cpp | azimuth_limit_deg/candidate_layers/views_to_minimum_radius 零消费；azimuth_steps 硬编码 1 | 死参数 | 删键或接线（删优先，yaml 同步） |
| quality_gate.cpp:31-38 | axisConsistencyGate 恒过 | 以 GateResult 表达纯诊断误导 | 返回诊断结构 |
| target_cache.cpp:78-180 | updateSelected/updateLocked | ~90% 重复 | 抽 `applyObservation(CachedTarget&, update)` |
| target_cache.cpp:281 | fitting clear 置 0.0 | 破坏 -1=无效约定 | 置 -1 |
| cycle_support.hpp:65-92 | ScanBudget 哑参+BUDGET_EXHAUSTED 不可达 | 接口与实现不符 | 收紧签名（删死分支；stages.cpp:449-460 同步） |
| trajectory_guard.hpp:352 | `quatGeodesicDeg` 手写 acos+π | — | `Eigen::Quaterniond::angularDistance` 替换（等价，gtest 对拍） |
| math_utils.hpp/stages.cpp:38/target_cache.cpp:39/view_planner.cpp:146 | 夹角函数 4 处兜底包装 | — | 单一 `angles.hpp`（兜底语义显式成枚举参数） |
| tool_actuator.hpp:14-38 | ToolProfile 死档案 | 无运行期消费 | 删；IO 三元组已参数化 |
| tool_txn.hpp | 除 harvestConfirmed 外零调用 | 死档案 | harvestConfirmed 迁 tool_actuator 后删文件 |
| acm_policy.hpp:29-41 | 连杆白名单 | 第三份清单 | 随档案参数单源 |
| grasp_geometry.hpp:174-179 | kToolBody* 与 tcp.xacro 手工对齐 | 换档案失同步 | `aubo_description/config/<profile>.yaml` 经 GPL 注入（tool_links/body L/R） |
| contact_monitor.hpp:147 | reason_ 死字段 | — | 删 |
| model_contract.hpp:71 | heartbeatRenewsValidity 恒 false 文档函数 | — | 改 constexpr 注释 |
| stages failStage | 41 处调用仅 13 处带失败码（28 处 code=0 上报） | 观察段/视点/再确认等无码 | 按 FailureCode 预留补码（观察类 UNREACHABLE、硬接触 CONTACT_ABORTED 等），表驱动清点 |
| move_to.cpp:196-202 | speed_scaling 未生效 WARN | — | 已注释说明，保持；或接线 MGI setMaxVelocityScalingFactor（行为变化需真机验证→排除，仅保持） |

### 2.7 vegetation 零风险归一
- yaml_params/param_rules 换 peach_common；`_to_bgr` 手写翻转（vegetation_node.py:207-215）→ cv_bridge `desired_encoding='bgr8'`；丢帧锁 → BoundedWorker(cap=1, drop_oldest)。

---

## 3. 横切设施与 peach_common 详细设计

### 3.1 包结构（ament_python，对齐 Nav2 nav2_common）
```
peach_common/
  peach_common/
    __init__.py
    yaml_params.py      # 原样单源（169 行，三副本已验证零 diff）
    param_rules.py      # 并集：check/min_max/enable_deps/nonempty/one_of/seq_gt
    runtime.py          # BoundedWorker/Clock/ManualClock/ScalarEma 上移
    paths.py            # resolve_runs_root 归一（vision 与 supervisor/batch 双实现合一）
    qos.py              # latched() / stream(depth=10) / reliable(depth=10)
  test/                 # 纯核 pytest（r0_gate 直测）
  package.xml setup.py setup.cfg resource/
```
- 迁移映射：三包 `from peach_harvester.yaml_params import attach` → `from peach_common import attach`；旧模块留 `from peach_common import *` shim 一个提交期后删。
- observability：attach 与 observability.yaml 迁回本包（消除对 peach_harvester.supervisor 的两条隐性 Python 边：params.py:15-16、observability_node.py:23）。
- `scripts/r0_gate.sh` PYTHONPATH 增挂 peach_common 源码树；CI（industrial_ci）非忽略包自动覆盖。

### 3.2 QoS 对齐
manifest 已是单源；代码侧 `qos.py` 工厂替换 ≥6 处手写（scene 两处、recon 两处、bringup 两组件、observability）；`latched()` 与 manifest `reliable+transient_local+depth1` 值对齐，可选在 check_interface_manifest.py 增「代码 QoS 构造与 manifest 一致」的抽查。

### 3.3 ReconstructionCore 注入接口（Mixin 宿主属性盘点结论）
- FrameStoreMixin 依赖：`_state_lock/_frame_ring/_frame_ring_max/_latest_frame/_target_masks/_locked_target_centers/_locked_target_areas/_preferred_target_id/_mask_gate/collector`
- AutoControllerMixin 依赖：上述锁+collector+`params.capture.*`+`get_logger`+`_algo_clock`+`_last_captured_stamp_sec/_target_kind_memory/_bound_axis_hint/_latest_candidates/_harvest_data` + 11 个节点方法（_auto_start/_finalize_now/三段式×3/_best_candidate/_reset_products/_publish_all/_prepare_frame/_commit_prepared_frame/_record_auto_skip）
- PublisherMixin 依赖：25 个状态属性 + 12 个 publisher + 7 个 params 叶子 + 6 个组装方法
→ Core 构造签名：`ReconstructionCore(collector, mask_gate, kind_memory, params, algo_clock, logger, timing, throttle, icp_target_cache)`；节点持有 Core 并把 Mixin 薄壳转发一个提交期。

### 3.4 深度反投影统一（三份实现差异实测）
integrate.build_cloud_base（open3d 官方，无距离窗，base 系）/ pose_pipelines._to_points（手写，含 [min,max] 距离窗）/ visualization._bbox_cloud_xyzrgb（手写，无窗，bbox 采样）。→ `geometry.backproject(depth_mm, mask, K, *, min_depth_m=None, max_depth_m=None, stride=1)`：数学同构已证实，距离窗参数化是唯一语义差异；三处调用替换后以现有纯核 pytest 数值对拍。

---

## 4. ExecuteTarget 接口收敛（六步 + 全消费端表）

**改动**：Result 删顶层 `cut_command_accepted/cut_confirmed/retreat_confirmed/harvest_confirmed`（:63-66）与 `deposit`（:70）；保留 outcome/completion_level/failure_code/reason/recovery_required/stage_durations/stage_names/harvest/verification/outcome_record/pregrasp。

**消费端逐点改造**：
| 消费端 | 现读 | 改为 |
|---|---|---|
| executor_node.py:1248-1257 | 顶层 harvest_confirmed/cut_confirmed/retreat_confirmed、deposit.reason | `harvest.grasped`/`verification.harvest_confirmed`；deposit.reason 并入顶层 reason 或删 |
| executor_node.py:1258-1264 | deposit.deposited | 删（未标定语义进 reason） |
| observability/pipeline.py:28-29 | ledger 行白名单 cut/retreat/harvest_confirmed | 随 batch.py:294 白名单同步改 failure_code_n/completion_level |
| manipulation_skills_node.cpp:1176-1215 | 填充端三重镜像 | 只填 harvest/verification/outcome_record |
| stages.cpp:327-330 等 | ctx 维护 cut_command_accepted 等 | ctx 字段保留（内部状态），只删消息镜像 |

**顺序**：IDL 注释 → 接口 README+manifest → io.md → 先编 peach_interfaces → 改三消费端+填充端 → `check_interface_manifest.py` 绿 + observability 26 测 + supervisor 测回归。

---

## 5. 死代码/死参数/死键清理总清单（随 Phase 执行）
1. Python：visualization:293 别名、domain/tracking.py、domain/observation.py、domain/evidence.corridor_*（标注）、refine axis_point 死字段、fused 6 死键、PublishThrottle.reset、_cut_station 死参、reducer 三死动词、domain/watchdog.py（迁或删）、LedgerIndex、batch.target_artifact_dir、runtime.default_harvest_root（迁移完成后）
2. C++：ToolProfile、tool_txn.hpp（harvestConfirmed 迁移后）、octomap_ns 死变量、ScanBudget 死分支+哑参、retreat ReverseNominal、stageVerifyCutHold 空阶段、ContactMonitor.reason_、view_planner 三死参数、QualityGateBase 默认体
3. yaml：arm_parameters.yaml view_planner 死键删除（部署值 config/peach_arm.yaml 同步）

---

## 6. 新发现缺陷清单（分级；修复先于或伴随对应 Phase，均需先补测试）

### 高危（行为缺陷）
| # | 位置 | 缺陷 | 修复 |
|---|---|---|---|
| S1 | executor_node.py:719-725 + reducer.py:55-63 | 终局后 settled-txn 活锁：standing txn 使后续事件命中 _stale 第三条款被丢，主循环无限重发 CYCLE_DONE 并高频 _persist_ledger/_publish_state（只读脚本已复现 effects=[]） | 事件级新 txn id；harvest_fsm NONE 语义拆分；FSM 单测固化 |
| S2 | :1690/:1926/:460/:1363 | 四处阻塞等待违 20s 仓规（180/30+30/45/30s，不可取消） | 有界探测+可打断 |
| S3 | :407-425/:460 | SetEnables 不校验使能依赖（可越级开刀）；FireStep 不过使能门 | 接入 check_enable_deps；使能预检 |
| R1 | capture.py:889-895 | `_record_auto_skip` 锁外落账竞态 | 移入锁内 |
| R2 | node:1146-1153 | TSDF 回滚漏 icp_target_cache.invalidate+版本递增 | 补齐 |
| R3 | node:1975-2014 + publish 调用路径 | `_pregrasp_verification_msg` 双路并发竞态（_pregrasp_prev） | 读写入锁 |
| R4 | node:1748 + :1892-1963 | 持锁写盘（save_session/geometry.jsonl）阻塞心跳违约 1.5s deadline | 锁内快照锁外写（session_recorder） |

### 中危
| # | 位置 | 缺陷 | 修复 |
|---|---|---|---|
| S4 | batch.py:284-302 | ledger 白名单断链：_cmd_full 写的 extra 4 键永丢，observability 按存在设计 | 白名单补齐（随 IDL 收敛同步） |
| S5 | executor_node.py:769-783 | 异常路径 finally 不补 _persist_ledger | finally 统一 persist；ReworkList.save 改原子写 |
| S6 | batch.py:104-106 | preferred 直通绕过 eligible（可选中裸果/未确认） | preferred 过谓词 |
| S7 | :1906-1914/:827/:740 | 断点恢复丢 details/discovered | details 随 ledger 回读 |
| V1 | inference.py:208-214 | SAM 吞异常→逐框回退死路径 | 异常上抛 |
| V2 | identity.py:876-889 | bbox_at_edge 淘汰不同步（缓慢泄漏） | 同步清理 |
| V3 | identity.py:979/985/1039 | 临时 id 撞号 | 单调计数 |
| N1/N6 | node:1569/:1547 | axis_angle 覆写；kind 强制 cylinder | RefitResult 双字段；kind 保留 |
| A1 | motion.cpp:206-223 | commandToolClose 越级 setState | stage 统一落账 |
| A2 | stages 28 处 failStage 无码 | 失败码缺口 | 表驱动补码 |

### 低危：V4（plan_lock 内 IO）、V5（pose_pipelines 顶层 ROS import 违纯核守卫）、S9 双钟兜底 identity.py:902、batch `_safe_run_component` 口径、`pregrasp_pose_of` 零值误拒、`_make_state` 投影失真、`_discovered` 无锁自增、Survey scene_epoch=0、电流环注释不符、`_publish_state` 风暴。

---

## 7. 实施映射（已批准 P0–P5 的扩展）

| Phase | 内容 | 新增并入 |
|---|---|---|
| P0 安全网 | arm gtest（grasp_geometry/trajectory_guard/view_planner/quality+safety_gate/reconfirm_policy/StagingCandidateSelector 对拍）；fixtures 迁 test/；harvester golden pytest（观测组装/_merge_fused_bag_model/fuse_bag_views 输出冻结）；**FSM txn 语义单测**（S1 前置） | 补 target_cache/applyObservation 对拍 |
| P1 peach_common | 建包+迁移+shim+observability 归位+r0_gate | S9/B 项 |
| P1.5 缺陷修复批（先行提交，独立可回滚） | S1/S2/S3/R1/R2/R3 + V1/V2/V3（每项一测一提交） | — |
| P2 感知侧 | scene（plan_updater/visualization 拆分/QoS/服务组/backproject/crop 合一）→ recon（refit_orchestrator/session_recorder/RefitResult/Mixin→Core/snapshot/时钟修复 R4） | V4/V5/N1/N6/N7/N9/N10 |
| P3 臂侧 | 参数单点/StagingSelector/FrameRateTimeouts/ResidualChecker/ACM 合并/档案注入/angles.hpp/删虚基类/诊断双轨/死码清单 | A1/A2/FK 复用/target_cache 合一/Inspectors 管道 |
| P4 IDL | 六步流程（§4） | S4/S5/S7 同轮 |
| P5 文档+提交 | architecture（包表+peach_common+UNWIND 改口：诊断已用/参数副本已收敛）、io.md、testing.md；分 Phase 提交，每提交一句主流检索依据 | supervisor 结构拆分（命令表/dispatch 拆分/S6/S8）并入 P4 前置或独立小提交 |

---

## 8. 等价性验证矩阵
golden pytest 消息级对拍（感知观测/重建 refit 输出）｜回放塔 `test_replay_approach` 基线逐字节（护栏数学）｜`test_mock_launch` 冒烟（lifecycle Active/关节序/TF 无叠帧）｜yaml 化门限默认值=原硬编码值核对表｜`check_interface_manifest.py`｜observability 26 测｜`colcon test` 全绿｜C++ 改后 Clangd restart。真机验收明确不在本轮。

## 9. 明确排除（逐条理由）
倒放轨迹 TOTG 重参数化（改真机时间参数→单独真机轮）；Python entry_points/pluginlib 注册表（dict 2 实现，AGENTS 明文留原模块）；composition/bond（本轮不新增生命周期节点）；GPL for Python（0024 已裁定）；peach_stereo 工程化及未提交修复；驱动九包；supervisor 自研 lifecycle_manager 重构（阶段 6 计划删除，仅修锁粒度注释）。

---
*附：本文与已批准重构方案（同日 ExitPlanMode 版）配套——该方案定 Phase 骨架与门，本文提供逐函数证据与全部优化条目；实施以两文合并口径为准。*
