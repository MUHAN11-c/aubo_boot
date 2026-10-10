# `peach_harvester` io（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/io.md` 拆写入包：本包四节点（scene_perception / target_reconstruction / supervisor / lifecycle_manager）与独立进程 `peach_scene_obstacles` 的话题、服务、动作与参数。相机契约（/camera/* 前端选择）与跨包边界在 [peach_bringup/doc/io.md](../../peach_bringup/doc/io.md)；字段级 IDL 在 [peach_interfaces/README.md](../../peach_interfaces/README.md)。**源码与本文互相更新，改接口须同一轮改另一边**。

一包两节点。不发运动、不选下一颗、不写 `ledger.json`（可向 `runs/<request_id>/perception_data/` 追加 `HarvestDataStore` 事件）。感知几何优先 stamp TF，失败 latest 并 `tf_stale`；重建积分必须图像时刻精确 TF，禁止 latest。

### 3.1 `peach_scene_perception_node`（看）

消费：RGB-D；`HarvestState`；精确或 latest TF。Lifecycle 非 Active：同步回调直接 return，不积分、不 `BeginScene`。

```mermaid
flowchart TB
  act{"Lifecycle Active?"}
  act -->|否| idle["回调直接 return"]
  act -->|是| src{"入口"}
  src -->|BeginScene| clr["重启收齐窗 scene_epoch++；换场才清身份"]
  src -->|HarvestState| lock["作业 target_id 只作锁定显示 不选下一颗"]
  src -->|RGB-D 同步| pipe["一帧管线"]
  pipe --> out["observations / initial_pose / diagnostics / debug_image"]
  clr --> wait["等下一帧"]
```

**读图：** 节点三入口。`BeginScene` 重启收齐窗；换场才清身份。`HarvestState` 只告诉「当前作业是哪颗」，不在这里选果。真正产出在 RGB-D 回调（下图）。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_scene_perception_node/begin_scene` | service | 重启收齐窗、推进 `scene_epoch`；换场才清身份 | peach_scene_perception | peach_supervisor |
| `/peach/perception/target_observations` | topic | 全量观测（锁定前 `observations[]` 空）：稳定 ID、跟踪态、掩膜、`scene_epoch`。调度选果须世代对齐且已锁定；重建对齐、技能新鲜度 | peach_scene_perception | peach_supervisor, peach_target_reconstruction, peach_arm, peach_observability |
| `/peach/perception/initial_pose` | topic | 单帧袋入口/轴初值。重建当起点；不授权运动 | peach_scene_perception | peach_target_reconstruction |
| `/peach/perception/diagnostics` | topic | 单帧拟合诊断（直径/RMSE/内点） | peach_scene_perception | peach_target_reconstruction |
| `/peach/perception/harvest_state` | topic | 感知侧计划 JSON（锁定集镜像，给监控） | peach_scene_perception | peach_observability |
| `/peach/perception/debug_image` | topic | 检/分割叠加（confirmed-only，进 RViz） | peach_scene_perception | peach_observability |

清单未列、源码仍发（可视化豁免）：

| 名字 | 含义 |
|------|------|
| `/peach/perception/detections` | 本帧检测框（真相流，不进 RViz） |
| `/peach/perception/debug_image_raw` | 筛选前叠加，与 `debug_image` 成对落盘 |
| `/peach/perception/masks` | SAM 像素掩膜 |
| `/peach/perception/single_cloud` | 检测框深度反投影，对 TF/深度，不是重建 |
| `/peach/perception/markers` | 锁定目标 3D：绿 ACCEPT、黄 REOBSERVE、红 REJECT |
| `/peach/perception/axis` | 袋轴 Vector3Stamped 可视化（RViz） |

```mermaid
flowchart TB
  sync["同步 RGB-D + 彩色 K + TF"] --> yolo["YOLO conf 0.35"]
  yolo --> filt["min_detection_conf 0.40"]
  filt --> dedup["IoS 0.6 去重"]
  dedup --> sam["MobileSAM 最多 16 框"]
  sam --> fg["hybrid_dilated = SAM ∩ 膨胀深度连通域；SAM 缺失 → mask_unavailable"]
  fg --> bag{"袋圆柱 / 果球"}
  bag --> gate["单帧 ACCEPT / REOBSERVE / REJECT"]
  gate --> tfw{"世界系 TF?"}
  tfw -->|unavailable| skip["几何留相机系 不注册"]
  tfw -->|stale| staleid["几何可变到 output_frame；身份不提交 打 tf_stale+target_untracked"]
  tfw -->|ok| id["整帧 χ²门+匈牙利 ID + EMA"]
  id --> obs["/peach/perception/target_observations"]
  gate --> init["/peach/perception/initial_pose"]
```

**读图：** 一帧相机数据从左到右变成「有哪些桃」。检测框 → 分割掩膜 → 袋/果几何 → 单帧门（只给画面，不授权运动）→ **仅精确 stamp TF（ok）** 才登记世界系身份。右边两条话题：观测给调度选果和技能；初值位姿给重建当起点。

- YOLO 异常：整帧跳过，不炸 worker。无框不补假框。
- SAM 只出像素掩膜。截断超 16 框的目标常变 OCCLUDED。前景几何只用 `hybrid_dilated`；SAM 缺失或交后像素 < `min_mask_points` 给 `mask_unavailable` / REOBSERVE，**不**走深度-only 几何回退。`depth_fallback` 只出现在构造 hybrid 时的深度连通域标签。
- 单帧门（2026-09-21 第二轮起）：`status` 只看**门控类** flag（净空/行程/误差预算/有效深度/截断/2D 校验/长度一致性等）；过程类 flag（`taper_*`/`polarity_*`/`axis_from_pca`/`fruit_prior_auxiliary`/`*_from_band`/`gravity_defaulted`/`length_extended_from_2d`）只进 `diagnostic_flags` 供追溯、不再压状态——修正前生产 56/56 恒 REOBSERVE（过程 flag 全帧在场）、refine 的 ACCEPT 偏好分支死码。`valid_depth_ratio` 同轮改**目标级口径** |SAM∩valid|/|SAM|（旧 ROI 均值被窗外背景稀释一个量级，连带压垮 confidence/σ/遮挡分类）；袋长塌缩新增显式 `axis_length_inconsistent` 门（3D 长度 vs 框 2D 展程×深度/焦距粗测）。**袋长 2D+3D 融合**（09-21 第三轮）：点云只覆盖有深度的可见段，袋长以原始 SAM 剪影沿投影轴的展程为准、被检测框限幅（`raw_mask` 经 build_masks 透传），两端只延不缩——修掉活流对拍 len −29% 的系统性偏短；直径不融合（径向 P95 稳健且直接进净空安全门）。
- 身份：整帧一次全局 1-1 分配（`identity.assign_detections`：同类 + 马氏 χ² 门 9≈3σ + 歧义比 1.2）。节点不传 `covariance`，σ=`match_radius/3`×`recovery_scale`。`SpatialEmaMatcher` 欧氏两段匹配**不**走帧路径（只提供半径属性）。`tf_unavailable` 与 `tf_stale` 均不改权威身份。确认 `confirm_frames=5`；贴边帧不攒确认。摆动连续 3 帧残差 >0.03 m → `target_swinging`（不可选）。
- 跟踪 token：OUT_OF_VIEW / LOST / OCCLUDED / DEPTH_VOID / OBSERVED。
- 锁定：`CollectLockPolicy`；`tf_stale` / `tf_unavailable` / `target_untracked` / `target_swinging` / `bbox_edge` 不可选。锁定后新 ID 不入集。`observations[]` 只发锁定 ID；锁定前数组空。

作业参数（部署值 `config/scene_perception.yaml`；`ScenePerceptionParams.attach`）：

| 参数 | 含义 |
|------|------|
| `yolo_conf` / `min_detection_conf` | 检测两级置信度。过低闪框，过高逆光漏检 |
| `yolo_model_path` / `sam_model_path` | 权重路径。yaml 写 `$(find-pkg-share peach_harvester)/model/…`；空串在参数层拒绝启动；节点不回落、不在编排层判空 |
| `confirm_frames` | 累计命中才转正进锁定/候选；短暂闪现不占 ID |
| `target_memory.match_radius_m` | 同类且距离内复用同一 `target_id` |
| `harvest.min_collect_frames` / `lock_settle_frames` / `max_collect_s` | 收齐窗口：最少帧、无新 ID 稳定帧、超时兜底 |
| `pipeline.min_depth_m` / `max_depth_m` | 位姿管线有效深度窗（现行 0.3 / **1.5** m） |
| `pipeline.bag_impl` / `fruit_impl` | 袋/果线映射名（`PIPELINES_BY_IMPL`）；果线另受 `from_params(..., enable_fruit=False)`（改 `True` 才构造） |
| `depth_scale_unit` | uint16：raw × 本值 = 毫米（Percipio 0.25） |
| `sync_slop_s` | RGB-D 近似同步允差（0.05 s） |
| `tool.entry_d_tool` / `entry_d_s` | 入口相对袋底。由 `grasp_standoffs.yaml` 注入，勿只改这里 |
| `tool.D_inner` | 工具内径（径向走廊门）。整栈由 launch `tool_profile` 档案注入覆盖（`aubo_description/config/<profile>.yaml` 单一事实源；三把剪切手 bite/shear/adaptive = 0.104/0.080/0.120，基础值=adaptive_shear_v1 0.120；bite 0.104 为 2026-09-30 裁定——几何权威在 URDF/机械设计，按设计意图登记，机械定型后实测核回） |
| `tool.L_insert` / `tool.L_blade` | 最大插入深度 / TCP 到剪切平面轴向距离。整栈由档案注入覆盖（三把 0.030/0.030/0.090 与 0.037/0.030/0.079，基础值=adaptive 档案；2026-09-29 扩注入面） |

### 3.2 `peach_target_reconstruction_node`（建）

消费：同一套 RGB-D；感知观测；`HarvestState.target_id`；`BuildTargetModel`。

```mermaid
flowchart TB
  act{"Lifecycle Active?"}
  act -->|否| idle["不积分 不受理 Build"]
  act -->|是| src{"入口"}
  src -->|BuildTargetModel| slot{"已有 Build 在跑?"}
  slot -->|是| rej["拒绝 单槽"]
  slot -->|否| bind["绑定 target_id COLLECTING"]
  src -->|HarvestState 换目标| drop["放弃旧会话 不把半成品续绑"]
  src -->|RGB-D 且已绑定| cap["采帧门"]
  cap -->|过| icp["有界 ICP"]
  icp -->|过| tsdf["LocalTsdf 在线积分"]
  tsdf --> cov{"机位 >= min_views?"}
  cov -->|否| wait["继续收帧"]
  cov -->|是| fin["finalize 提云 + 袋融合"]
  fin --> pub["grasp_decision / refined_* / tsdf_cloud"]
  bind --> wait
```

**读图：** `BuildTargetModel` 一次只绑一颗。RGB-D 过门才积分；机位够了才 finalize 出发许可话题。换目标丢旧会话。finalize 后的几何/许可分岔见下图。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_target_reconstruction_node/build_target_model` | action | 绑 `target_id`，收满机位后 finalize 出模型 | peach_target_reconstruction | peach_supervisor |
| `/peach/reconstruction/diagnostics` | topic | 结构化心跳：绑定态、机位数、基线、TF 失败。**QoS=latched+DDS deadline 1.5 s**（A10：心跳 1 Hz，超期即 offered-deadline-missed WARN——执行器卡滞的 DDS 层补强，只记日志不改行为；请求方 deadline ≥1.5 s 或不设均兼容） | peach_target_reconstruction | peach_arm, peach_observability |
| `/peach/reconstruction/diagnostics_debug` | topic | 调试 JSON 明细（TSDF/ICP/逐机位），不进决策 | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/status` | topic | 短状态 String（MCAP 白名单用这个，不是 diagnostics） | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/grasp_decision` | topic | 融合入口/轴/预抓取/剪切 + `allowed` 汇总位（分档复检语义见 §5 `GraspDecision` 行） | peach_target_reconstruction | peach_arm, peach_supervisor（闩锁令牌缓存）, peach_observability |
| `/peach/reconstruction/pregrasp_verification` | topic | 重建侧残差观测；技能 VerifyPregrasp 用工具 TF，未订本话题 | peach_target_reconstruction | （观测） |
| `/peach/reconstruction/refined_pose` | topic | 融合后袋位姿（精化候选） | peach_target_reconstruction | peach_arm, peach_scene_obstacles, peach_observability |
| `/peach/scene/obstacles_refresh` | topic | Survey 快照建图触发（Empty，transient_local；Survey 成功=拍照位刚拍完，此刻点云帧即障碍快照源，0035） | peach_supervisor | peach_scene_obstacles |
| `/peach/reconstruction/refined_axis` | topic | 融合袋轴，给监控三维 | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/refined_diagnostics` | topic | 精化拟合诊断 | peach_target_reconstruction | peach_arm, peach_observability |
| `/peach/reconstruction/tsdf_cloud` | topic | 绑定目标 TSDF 表面（批次结束会复位变空） | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/local_cloud` | topic | 当前目标局部点云（latched；清单可视化豁免） | peach_target_reconstruction | （可视化/调试） |
| `/peach/reconstruction/markers` | topic | 相机轨迹与精化示意 | peach_target_reconstruction | （可视化） |
| `/peach/reconstruction/shape_hypothesis` | topic | 形状假说（契约预留，未当批次门） | peach_target_reconstruction | （无消费方） |

重建调试 Trigger（`std_srvs/Trigger`，已进清单；Web 调试面/人工用，调度主路径不调；改状态的三口过 Active 门，query 只读无门）：

| 名字 | 含义 |
|------|------|
| `/peach_target_reconstruction_node/reset_reconstruction` | 清空帧栈与绑定目标，回 IDLE（换目标重积） |
| `/peach_target_reconstruction_node/finalize_reconstruction` | 手动提前 finalize 当前会话出模型 |
| `/peach_target_reconstruction_node/save_session` | 显式落盘会话工件（`runs/session_*/`） |
| `/peach_target_reconstruction_node/query_reconstruction_state` | 只读返回当前重建与数据关联 JSON |

采帧门（`target_reconstruction/capture.py`，自动失败=skip）：满栈 → 无帧 → 掩膜 → 同 stamp → 帧龄>2 s → 静止（`/joint_states` 最大 `|vel|`>0.03 rad/s）→ 空 frame_id → **精确 TF**。

```mermaid
flowchart TB
  obs["同戳掩膜 + RGB-D"] --> gate["采帧门"]
  gate -->|过| icp["有界 ICP"]
  icp -->|拒| skip["跳帧 不硬套"]
  icp -->|过| tsdf["LocalTsdf"]
  tsdf --> fin["finalize 提云"]
  fin --> landmarks["宽头=底、窄头=口；轴袋底→袋口且不朝下；对打否决该帧"]
  landmarks --> geom["入口侧向贴体积 + 剪切参考=袋口/框极限"]
  geom --> pre["PREGRASP_ONLY 真机评方向定位"]
  geom --> envelope["TSDF 包络主方向否决"]
  envelope -->|病态跳过12°| budget["动态径向/轴向预算"]
  envelope -->|夹角>12°| veto["keypoint_cloud_axis_conflict"]
  envelope -->|一致| budget
  veto --> gd{"GraspDecision.allowed 只拦套入"}
  budget --> gd
```

**读图：** 重建把多机位收成「这一颗怎么套」。过采帧门才积分；ICP 拒了就丢这一帧，不硬套进模型。下面分岔：左边融合几何给预抓取评方向；右边包络轴只否决、不授权。最下菱形 `allowed` **只决定能不能套入/剪切**，关了仍可去预抓取。

**两层三态不要混：**

| 层 | 字段 | 谁消费 |
|----|------|--------|
| 感知单帧 | `BagGraspCandidate.status` ACCEPT/REOBSERVE/REJECT | 初值、可视化。不发运动 |
| 融合几何 | `GraspDecision` 入口/轴/预抓取/剪切参考（融合成功即填；后撤由 `grasp_standoffs.yaml` 注入，现行入口 0=拟合袋底、预抓取相对入口 0.03 m） | `PREGRASP_ONLY` 到预抓取停住；RViz/监控目视。方向定位对错以真机预抓取实测为准。停袋底对照轮次 1757（历史在 git，原 testing-log 已删） |
| 接触许可 | `GraspDecision.allowed` | 只授权套入/剪切；禁止降级接触 |

融合成功时 entry/axis/pregrasp/cut_pose 有效，即使 `allowed=false`。无几何时入口/轴填零，只信 `reason` / `failure_code`。常见 reason：`reconstruction_not_ready`、`refined_geometry_unavailable`、`bag_model_unavailable`、`dynamic_budget_negative`、`keypoint_cloud_axis_conflict`（包络轴与关键点轴 >12° 且包络有长径比）、`cut_plane_fruit_clearance` / `cut_band_unavailable`。`envelope_axis_ill_conditioned` / `envelope_too_few_slices` 只诊断，不单独关 `allowed`。>35° 只打 `diagnostic_axis_mismatch`，不单独把 `allowed` 打成 false。通过接触：`dynamic_budget_accept` / `refined_geometry_accept`。软件预算与夹角不代替预抓取位的真机精度评定。

`allowed=true` 之后仍可能 `skipped_quality`（再确认/预抓取残差）或 `skipped_unreachable`（MTC 护栏）。心跳里大量 `not_ready` 不等于精化从未 ACCEPT；看逐目标 max，不要看收工 IDLE。

作业参数（部署值 `config/target_reconstruction.yaml`；`TargetReconstructionParams.attach`）：

| 参数 | 含义 |
|------|------|
| `capture.min_views` | finalize 所需独立机位数（默认 2 = 当前位+一次短移） |
| `capture.max_views` | 帧栈上限，不是节拍 |
| `capture.require_robot_static` | `/joint_states` 最大 \|vel\| 超门则跳帧 |
| `capture.min_neighbor_gap_m` / `neighbor_gap_area_ratio` | 邻锚过近拒帧；小框面积比豁免，近距双检不互锁 |
| `tf_timeout_sec` | 按深度 stamp 精确查 TF；失败跳帧，禁止 latest |
| `refit.entry_standoff_m` / `refit.pregrasp_standoff_m` | 入口相对袋底、预抓取相对入口。只改 `grasp_standoffs.yaml` |
| `tool.budget.d_inner` | GraspDecision 动态预算许可的工具内径。整栈由 launch `tool_profile` 档案注入覆盖（三把剪切手 bite/shear/adaptive = 0.104/0.080/0.120，基础值=adaptive 0.120；bite 0.104 为 2026-09-30 裁定值，与纯核 `ToolBudgetParams` 历史默认同级） |
| `tool.profile_id` | 档案标签（`GraspDecision`/`PregraspVerification`/`TargetModel` 的 `tool_profile_id`）。整栈由 launch `tool_profile` 档案注入 |
| `refitter.cylinder_impl` / `sphere_impl` | 柱/球精化映射名（`REFITTERS_BY_IMPL`）；其余算法直接构造 |

---

### peach_scene_obstacles（场景障碍快照，0035）

独立进程（`peach_harvester` 包 console_script，不进 lifecycle 名单、不随 brain 进程）；`harvest_system` 在 `camera_enabled:=true` 时拉起（须位于 aubo include 之前，stereo 分支参数泄漏坑同见 launch 注释）。无状态快照服务：订阅 `/camera/depth_registered/points`（**RELIABLE** VOLATILE depth 5——两前端发布端均 RELIABLE，BE 订户大帧假性丢包（09-29 勘定），缓存最近一帧）、`/peach/reconstruction/refined_pose`（latched，累积已精化目标胶囊）、`/joint_states`+`/robot_description`（自身滤除 FK/mesh）、`/peach/scene/obstacles_refresh`（触发）；经 `/apply_planning_scene` 写 BOX 对象 `peach_scene_obstacles`（首写 ADD、此后同请求 REMOVE+ADD 原子替换）。参数 `config/scene_obstacles.yaml`（0024 直读 attach：体素 0.06、自身余量 0.05、胶囊余量 0.10/0.10、半径 1.5、上限 3000、帧龄门 2s；**滤除在体素中心上做，阈值=余量+体素半对角**——保证方块含角点不切机器人表面，0036）。触发后处理链在工作线程单飞（Open3D 突发不占 executor）；新目标精化后同帧重写；作业期冻结。**只写 world.collision_objects，不碰 ACM**（ACM 归 peach_arm，分域防覆盖；peach_arm 激活即 3 次幂等重试应用 26 条豁免并打 INFO）。纯核 `vision/scene_obstacles/core.py`（零 ROS，pytest 19 例）。

---

## 5. `peach_harvester`（supervisor）

调度与 lifecycle 管理器同包。8090 实现在 `peach_observability`；参数 yaml 也在 `peach_observability/config/observability.yaml`（W11 起随包走），节点 `ObservabilityParams.attach`。调度是批次唯一所有者；lifecycle 不发 `RunHarvest`；监控只读。

### 5.1 `peach_harvester`（supervisor）（批）

```mermaid
flowchart TB
  rh["RunHarvest"] --> flag{"managed_nodes_activated?"}
  flag -->|否且 require_managed_stack| rej["拒绝开批"]
  flag -->|是| nav["_cmd_navigate 直通 NAV_OK"]
  nav --> survey["SurveyScene 复核关节"]
  survey -->|失败| fail["整批 survey_failed"]
  survey -->|成功| begin["BeginScene 重启收齐窗"]
  begin --> waitLock["WAIT_LOCK 世代对齐且锁定"]
  waitLock --> intent{"intent / execution_enabled?"}
  intent -->|SURVEY_ONLY 或执行关| settle["结算 不选果"]
  intent -->|PICK| sel["SELECT 深度窗 ∩ CheckReachability"]
  sel -->|无候选| empty{"empty_survey_limit?"}
  empty -->|否| revisit["SurveyScene 回访 不 Begin"]
  empty -->|是| settle
  sel -->|有| disp["并行 BuildTargetModel + ExecuteTarget OBSERVE"]
  disp --> full{"execute_pregrasp_only?"}
  full -->|true 默认| pg["ExecuteTarget PREGRASP_ONLY"]
  full -->|false| fl["ExecuteTarget FULL"]
  pg --> recov{"recovery_required?"}
  recov -->|是 未 ACK| hold["不 Survey 不派下一颗"]
  recov -->|ACK| ledger["写 ledger.json"]
  fl --> ledger
  ledger --> revisit
  revisit --> sel
  ctl["ControlTask PAUSE/SKIP/CANCEL/ACK"] -.-> recov
```

**读图：** 调度是唯一动作客户端。开批先看 lifecycle 旗标（软件就绪，不是柜侧上电）；到位无导航动作。**先到拍照位再开收齐窗**；锁定后才选果。默认停预抓取，须 ACK 才再 Survey（回访不 Begin）。状态机细节见 [architecture.md](architecture.md) 图 C。虚线是人工 `ControlTask`，监控不发。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_supervisor/run_harvest` | action | 显式开一批；不自动发 | peach_supervisor | 人工 |
| `/peach_supervisor/control` | service | 暂停/跳过/取消/ACK；须带对的 `state_seq` | peach_supervisor | 人工 |
| `/peach_supervisor/state` | topic | 批次快照：`target_id`、档位、`recovery_required`、permissions | peach_supervisor | peach_scene_perception, peach_target_reconstruction, peach_observability |
| `/peach_supervisor/events` | topic | 可检索事件（拍照到位/锁定/派发/终局/过滤/ACK） | peach_supervisor | peach_observability |
| `/peach_supervisor/scene_snapshot` | topic | WAIT_LOCK 或回访 dwell 后的锁定集快照 | peach_supervisor | （无订阅方；账本/MCAP） |

客户端（仅本节点）：`BeginScene`、`SurveyScene`、`BuildTargetModel`（与 OBSERVE_ONLY 并行）、`ExecuteTarget`、`CheckReachability`、`MoveTo`（`/peach_arm/move_to`，fast 档观察/回拍照位，2026-10-08 起）。批次6 `reconstruct_in_trajectory`（默认关）：true 时 DISPATCH 起 Build 后不再阻塞等 Build 完成，立即派 FULL（goal 自带 `skip_observation=true`，身份 revision=`pending:*` 占位，不装 GraspDecision 令牌）——重建与接近并行，臂侧 FinalizeAndValidate 有界等精化兜底，周期收口取消并等 Build 结束（单槽约束）。账本：`runs/<request_id>/ledger.json`。

---

### 5.2 `peach_lifecycle_manager`（管）

```mermaid
flowchart TB
  boot["节点起来 定时器踢一次 STARTUP"] --> cfg["正向 configure"]
  cfg --> act["正向 activate"]
  act --> flag["managed_nodes_activated"]
  flag --> wait["等人 ManageLifecycleNodes"]
  wait -->|PAUSE| deact["逆向 deactivate → 旗标 false"]
  wait -->|RESUME| react["正向 activate → 旗标"]
  wait -->|RESET| rst["逆向 deactivate+cleanup 再 configure/activate"]
  wait -->|SHUTDOWN| td["逆向 teardown → 旗标 false"]
```

**读图：** 名单感知 → 重建 → 技能 → 调度；observability 不进名单。PAUSE 是节点 Inactive，不是批次暂停。不管柜侧上电，不发 `RunHarvest`。**整栈由 `nav2_lifecycle_manager` 承载**（节点名同为 `peach_lifecycle_manager`，bond_timeout=0，名单硬编码在整栈 launch；下表闩锁在整栈由 `peach_lifecycle_flag_bridge` 发出）；本包内管理器保留独立 launch（`peach_harvester/launch/lifecycle_manager.launch.py`），服务语义不变。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_lifecycle_manager/manage_nodes` | service | STARTUP/PAUSE/RESUME/RESET/SHUTDOWN 整栈；不发 RunHarvest | peach_lifecycle_manager | 人工 |
| `/peach_lifecycle_manager/is_active` | service | nav2_lifecycle_manager 自带只读口：查询某托管节点当前生命周期态（调试用） | peach_lifecycle_manager | 人工/调试 |
| `/peach/lifecycle/managed_nodes_activated` | topic | 名单节点是否都 Active；调度开批闸门 | peach_lifecycle_manager（独立）/ peach_lifecycle_flag_bridge（整栈） | peach_supervisor |

名单默认：场景感知 → 重建 → 技能 → 调度。observability **不进名单**。

| 参数 | 含义 |
|------|------|
| `node_names` | 有序 configure/activate 名单 |
| `startup_timeout_s` | 单次状态转换等待上限 |
