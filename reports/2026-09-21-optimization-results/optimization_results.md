# peach 全量优化结果报告（函数级，2026-09-20/21）

**范围**：W0–W8（前轮，感知→抓取主链）之后的本会话三轮——① 外围五包质量轮 W9–W16（8 提交 ce7f68b→b0ef3d4）；② 端到端代码审查（5 高危/19 中危/14 低危）；③ 修复轮（3 提交 688ea64/8bf001f/c242de3）+ 独立复审验收。分支 test/20260909-field-traj，共 **11 个未推送提交**。

**总体数字**：
- 测试：本轮起点 503 → 终态 **~535 例全绿**（interfaces 9 / common 37 / bringup 4 / harvester 188 / arm 216 / observability 34 / vegetation 16 / system_tests 31）；净增 ~32 例，全部为语义锁定型（非凑数）。
- 代码量：11 提交合计约 **+1,950/−860 行**（新增大头是测试与注释，删除大头是死码/复制粘贴/手写数学）。
- 缺陷闭环：审查发现 5 高危 **全部修复**；19 中危修复 6 项（M1/M2/M3/M11/M13/M15）；复审确认七项修复面全部「修对、无放松性回归」。

---

## A. 感知管线（peach_harvester/vision/scene_perception）

| 函数/模块 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| `inference.CandidateEstimator.build_masks` | 每个检测目标每帧重建 `cv2.getStructuringElement` 膨胀核 | 按核边长的 dict 缓存（`_dilate_kernel_cache`），构造一次复用 | W13 |
| `inference` 模式注册表 | `ForegroundMode` dataclass + `FOREGROUND_MODES` 单模式注册表 + `MODE_LABELS`（三样均零消费者） | 压缩为常量 `MODE_IDS = ('hybrid_dilated',)` | W13 |
| `reconstruction_core` 拒帧告警 | 构云/ICP 拒帧逐帧无节流 WARN（持续故障刷盘） | 补 1s 节流（`throttle_duration_sec=1000.0`，rclpy 毫秒单位） | W13 |
| `scene_perception_node` 分辨率告警 | 两处逐帧 WARN | 同上 1s 节流 | W13 |
| `pipeline.process` | 裸 `np.linalg.inv(T_out_cam)` | 走 `geometry.invert_transform` 单源 | W13 |
| `pose_pipelines._project` | 手写单点投影 | 薄委托新 `geometry.project_point`（backproject 的对偶，语义逐字一致） | W13 |
| `pose_pipelines` 双 `estimate`（袋 241 行/果 216 行同构） | 26 行×2 重复前奏（裁框→尺寸门→掩膜→点云→离群→点数门→重力） | 抽公共 `_prepare_estimate_inputs` + `_EstimateInputs` 上下文；`_failed/_failed_fruit` 合并为单实现（按 `self.kind` 推导）；中段/尾部深度纠缠按「逐字节保持」硬门降级未全并，两法 docstring 互注 | W13 |
| `plan_updater.py:322` | 恒真死条件 `degenerate_candidate(_empty_candidate())` | 记录在案（行为与 W3 前逐字等价，留待接口清理轮） | 审查记录 |
| 死码 | `domain/tracking.py`、`domain/observation.py`、`visualization.py` 过渡 shim | 删除（全仓 grep 零生产引用） | W13 |

## B. 重建管线（peach_harvester/vision/target_reconstruction）

| 函数/模块 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| **许可有效期**（`publish.MODEL_VALIDITY_S`） | 硬编码 5.0s——与接近链时长错配（真机单 LIN 实测 7.5s、FULL 全链 30-60s）→ FULL 档每颗令牌在套入授权点过期→恢复门→人工 ACK（**G1 高危**） | 参数 `decision.validity_s` 默认 **120s**（yaml→attach 规则→`publish.decision_validity_s` 单点取窗→冻结处/消息兜底/TargetModel 三处收口）；0022「同 revision 冻结不续签」语义逐字保真；运行期 `ros2 param set` 可调 | 688ea64 |
| `refine.merge_fused_bag_model` 的 `model_revision` | `f'{tid}:{views}'` 非单调——同目标同机位数重 Build 字符串不变→臂侧 ModelSnapshot 不换新→沿用过期 valid_until/几何（**G3 高危**） | `f'{tid}:{views}:{finalize_counter}'`，计数进程级单调（RLock 下串行、reset 不清零、失败轮跳号不影响单调）；18 个消费文件逐一验证均按不透明串 | 688ea64 |
| `_grasp_decision` 诊断 JSON 的 `allowed` | 初始化 False 后从不置 True——events.jsonl/diagnostics_debug 审计轨迹上许可永远呈拒绝（**M11**） | 与类型化消息同源派生：`model_contract.capabilities_from_decision/allowed_from_decision` 单源，dict 侧/消息侧同函数 | 688ea64 |
| `_lock_decision_validity` | 窗口硬编码 | 取 `decision_validity_s(self.params)`；冻结判据逐字未动 | 688ea64 |
| （调度侧配套）`executor_node` 令牌装配 | 过期令牌照发、无感知 | `_warn_decision_expiry`：过期 WARN 一次（target+revision 去重、零值与臂同语义跳过），不改 FSM | 688ea64 |

## C. 调度（peach_harvester/supervisor）

| 函数/模块 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| `_cmd_dispatch` | 9 处 `_react→_record_skip→_apply→return` 五行失败收尾复制粘贴 | 收敛单一 `_fail_dispatch(...)`（参数逐点保持） | W13 |
| OBSERVE goal 装配 | fast/conservative 判定前无条件装配（fast 档死装配） | 移入 conservative 分支；`_cycle_plan_id` 跨轮消费点保持在分支外并注释 | W13 |
| docstring | 28 个方法 + 2 个嵌套闭包无 docstring | 全部补齐（中文一行，含 W6-A 后基建） | W13 |
| `_on_fire_step` 五个占位单步 | 语义不明 | docstring 明确 debug-only 占位、未接线 | W13 |
| `budget.py` 占位语义 | `capability_from_ok(True)`/`'ready_ok': True` 硬编码无标注 | TODO 就位标注（占位语义、不得当校验结论） | W13 |
| `identity.py:913` 双钟兜底 | wall/monotonic 混用风险 | 注释钉死「表项年龄只与单调钟比」 | W13 |
| bond | 无进程心跳 | configure 建 /activate 重臂 /deactivate+cleanup 断（守卫式 bondpy，缺包降级 WARN） | W14 |

## D. 技能节点（peach_arm）

| 函数/模块 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| **plan 契约**（`cycle.cpp` 受理） | 任何 goal（含 OBSERVE_ONLY）都记 preview 绑定→observe goal 带不了三修订→conservative 档 FULL 必拒、第二颗起连 OBSERVE 也拒；fast 档契约真空（**G2 高危**） | **只 PREVIEW 模式写绑定**；FULL/PREGRASP_ONLY 终局清绑定；受理即拒不清（保留全字段约束）；比对抽纯核 `executePlanGate`（plan_contract.hpp）。复审证实调用点守卫与旧版逐字等价=无放松 | 8bf001f |
| plan mismatch 失败码 | abort FAILED 但 `failure_code=0`（PLAN_MISMATCH=20 有码不用） | `pending_accept_failure_code_` 成员（每 goal 复位）带入 Result；`static_assert` 与 IDL 双钉 | 8bf001f |
| onStart 拒单码 | 全部 0 | recovery 拒→RECOVERY_REQUIRED(9)、锚点缺失→OBSERVE_FAILED(1)（与 stagePrepareCycle 同条件同码）；无词表项的保持 0+TODO | 8bf001f |
| **取消旗标** `cancel_requested_` | 唯一复位点=下一 ExecuteTarget onStart→一次单果取消/skip 后一切 MoveTo 被拒、fast 补视静默退化（**M1**，「批次取消后 PHOTO 亦失败」的机理） | `clearCancelFlagIfIdle()`（`!running_` 守卫）六处终局自动清；复审以 running_ 全表核实守卫充分 | 8bf001f |
| 无限 join | Survey/MoveTo 受理回调 + 预览 worker 三处裸 join（卡死→默认互斥组吊死→ACK/取消/订阅全停，**M2**） | packaged_task+future 2s 有界 join + WARN + detach（与 W13-B ExecuteTarget 同纪律） | 8bf001f |
| 授权拒绝分级 | 令牌过期经话题快照回看→误记 FAILED（不可重派语义） | 纯核 `stage_denial`：EXPIRED（仅 valid_until 超时/model_stamp 超窗两支）→ **SKIPPED_QUALITY**（进 `rework_list.json` 补采闭环）；明确不允许→FAILED | 8bf001f |
| `loadParameters` | ViewPlanner/QualityGate/SafetyGate/contact_detect 25 行逐字段手抄 | `params_bridge` 新增 4 个 to*Config 转换归桥 | W13 |
| `view_planner` | 三个死参数（azimuth_limit_deg/candidate_layers/views_to_minimum_radius）全链携带 | hpp/GPL yaml/部署 yaml/loadParameters 全删 | W13 |
| IK 超时字面量 | motion.cpp 0.05 与 staging 两处各自硬编码 | `staging_selector.hpp` 共享常量（0.1 深搜/0.05 预检，值不变） | W13 |
| `axisConsistencyGate` | 恒过门（两 return 均 allowed=true，语义不明） | 误差进诊断字段 `axis_angle_deg/axis_mismatch`（allowed 语义显式化，令牌逐字保留） | W13 |
| `onActionAccepted` | 无限 join 旧线程 | packaged_task 2s timed join + WARN | W13 |
| 公有头注释 | 10 个头 `///` 稀疏（trajectory_guard 415 行 0 条） | 全部补齐（对齐 model_contract.hpp 风格，重点 report struct 全字段） | W13 |
| bond | 无进程心跳 | bondcpp：on_activate 建（LifecycleNode 构造器，原生 publisher）/deactivate+releaseResources 断 | W14 |

## E. 观测（peach_observability）

| 函数/模块 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| **`ObservabilityState.snapshot`** | 每次 snapshot 全量跑 `build_harvest_job`（157 行折叠），被 Web 轮询/RViz 5Hz/路标以 30–50Hz 重复调用、结果多被丢弃 | 按 `_revision` 记忆化（每代只算一次、多调用方共享只读缓存）+ `job()` 窄访问器；`topic_ages()` 诊断访问器 | W10 |
| `Recorder` 写队列 | `queue.Queue()` 无界——'all' 档 ~50MB/s 盘速掉队时内存无上界 | 有界 `record.queue_depth`（默认 512）drop-oldest + 丢帧计数 + 10s 节流告警；被丢条目补 `task_done` 防 `close()` 永挂 | W10 |
| `CatchAllRecorder.stop()` | 只清 Python 列表——rclpy 节点持订阅强引用，deactivate 后通配回调**继续向记录器入队** | 逐个 `destroy_subscription` + `destroy_timer` + 清发现态（幂等、支持重激活重发现） | W10 |
| `bag_reader.read_bag` | 类型化转换表在消息循环内逐条重建（5 键 dict + 跨模块查找/条） | 提出循环外构造一次 | W10 |
| `_create_subscriptions` | 101 行 20 路订阅平铺注册 | 核心表 + bag raw 表两张订阅表驱动（新增订阅=加一行） | W10 |
| `_task_executor_callback` | 一个回调做 5 件事 | 拆 `to_task_executor_state`（下沉 state.py 转换器族）+ `_track_run_context` + `_track_fsm` | W10 |
| TCP 可视化 | RViz 5Hz 与 HTTP /api/trajectory 各自重复 downsample+marker 构建 | `_tcp_viz_bundle` 签名缓存共享一份 | W10 |
| `retention.sweep` | configure 期同步 rglob 全部 bag（20GB 库秒级阻塞生命周期） | 后台线程 + cleanup join | W10 |
| **手写四元数/TF 合成** | `_quat_mul`/`_quat_rotate`/`_compose_chain` 手写哈密顿积（AGENTS 禁手写四元数运算） | 删两个手写函数，链合成改 `scipy.spatial.transform.Rotation`（xyzw 同 ROS 序；TF 链既有测试作回归门） | W11 |
| **跨包依赖** | params.py `from peach_harvester.supervisor import ...`（AGENTS 单向依赖违例）+ yaml 寄居他家 | yaml git mv 随包走、import 切 peach_common、package.xml 去 harvester 依赖、launch/manifest/两包测试同轮改 | W11 |
| `job.py` 失败判级 | 子串散落两个函数内 | `_FAIL_STAGE_TOKENS`/`_BATCH_FAIL_TOKENS` 带优先级注释的表（保行为） | W11 |
| **`/diagnostics` 双轨** | 健康只在自研 HTTP 报（AGENTS 反模式） | Updater 5s 两任务：`session_recorder`（队列水位/丢帧/目录）+ `ingest_liveness`（最热年龄 ≤10s OK/≤60s WARN/更久 STALE） | W15 |
| **停栈报告**（G4 高危） | launch 默认 5+5s 拆杀 vs `join_report(300s)` + `write_report` 非原子直写→真机停栈报告大概率被杀留半份 | 原子写（tmp+`os.replace`）+ launch `sigterm_timeout=60` + `join_report(55)` 对齐——最坏留「无新报告」（CLI 可复跑）不再出半份 | c242de3 |
| SceneSnapshot 落盘 | VOLATILE 订阅吞掉单发闩锁——记录节点晚启动则快照永不入 bag（M13） | 落盘订阅改 transient_local | c242de3 |
| `ensure_active` 自转换 | 与 vegetation 复制粘贴两份 | `peach_common.lifecycle.ensure_lifecycle_active` 单源 | W11 |

## F. vegetation

| 函数/模块 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| GPU 推理回调 | Frangi 数百 ms 帧推理跑在默认互斥组→推理期间 diagnostics/生命周期回调全部饿死 | 专用 `MutuallyExclusiveCallbackGroup` + `MultiThreadedExecutor(2)`（保留 try-lock 丢帧） | W12 |
| 图像订阅 QoS | RELIABLE（相机侧将来切 SensorDataQoS 会静默断流） | 传感档 BEST_EFFORT（`peach_common.qos.sensor` 新工厂；与 RELIABLE 发布端兼容不破现有连接） | W12 |
| cv2 死回退 | `_hsv_u8` 30 行 numpy HSV 分支 + `_dilate_bool` sliding-window 分支（cv_bridge 硬依赖保证 cv2 必在） | 删除，直走 cv2 | W12 |

## G. 基础设施（bringup / common / interfaces / system_tests）

| 项 | 优化前 | 优化后 | 提交 |
|---|---|---|---|
| **preflight 拒启名单**（G5 高危） | 漏 brain 进程（exec=`peach_harvester`、三托管节点名不进 argv）→残留旧脑双 supervisor 静默共存；另漏 flag_bridge/autostart/stereo | 名单补 4 名（argv[0] basename + /lib/ 路径匹配，不误伤编辑器/colcon）+3 测试对账 | c242de3 |
| bringup 参数校验 | 第三套 `_POSITIVE` 手写风格 | 统一 `peach_common.param_rules.check` 规则表 | W9 |
| `check_interface_manifest` | consumer blob 接口数×consumer 数重复读盘；正向/反向判定同构两份 | 按 consumer 缓存 + 复用 `_name_in_blob` | W11 |
| `peach_common.qos` | 无传感档工厂 | `sensor(depth)`（BEST_EFFORT+VOLATILE） | W12 |
| `peach_common.lifecycle` | 无 | `create_bond/break_bond`（守卫式 bondpy）+ `ensure_lifecycle_active` | W11/W14 |
| replay_oracle `_mat_to_quat` | m22 主元分支 w 分量误抄 `(m[0][1]-m[0][1])/s` 恒 0（潜伏数学缺陷） | 修复为 `(m[1][0]-m[0][1])/s`；插桩证实三层语料 1640 调用零触发、基线数值逐数不变 | W9 |
| test_perf_baseline | **从未被 CMake 注册**（colcon 永不运行） | 注册（25→29 测） | W9 |
| observability 死参数 | `record_bag_topics`（零消费）、`debug_token`（鉴权已删） | 删除 + 文档改口 | W9 |
| DepositResult.msg | 头注释宣称「技能写入 Result.deposit」（W7 后不实） | 保留（0012 卸果站预留）+ 注释改口预留现状（M15） | c242de3 |
| launch `bond_timeout` | 硬编码 0.0 | 参数化（默认 0；开启前置 apt bondpy，描述写明） | W14 |

## H. 架构 UNWIND 收敛

| 项 | 结果 |
|---|---|
| lifecycle bond | 四托管节点接线：arm bondcpp **生效**（/bond 1Hz 实测）；Python 三节点守卫式 bondpy（**待用户 `sudo apt install ros-jazzy-bondpy` 后 `bond_timeout:=8.0` 即开**）；launch_testing 用例锁心跳 |
| diagnostic_updater | peach_arm（W5）/ **peach_observability（W15 新增）**/ serial_imu 在用；感知/重建/调度仍缺口 |
| composition | 核实**平台阻断**落档：Python 无组件容器；ComponentManager 零生命周期处理（源码证据）；臂侧非高带宽无零拷贝收益——进程隔离+bond 为可达上限 |

## I. 复审验收（2026-09-21）

七项修复面（G1/G3/M11/G2/M1/M2/M3）双路独立复审 + 主审跨提交接缝核验：**全部「修对」、无放松性回归**；SKIPPED_QUALITY→`rework_list.json` 补采闭环确认。复审新增跟进项：
- **R1（中）**：`executeMoveTo` 不置 `running_`（M8 遗留）→ M2 detach 后卡死 MoveTo 线程与新周期可并发用 move_group_（MGI 非线程安全）——修法=补 running_ 置位（M8 收口）。
- R2（低中）：`closeMotionOutputAndCancel` 四处裸 join 仍无界（生命周期路径半修）。
- R3（低）：EXPIRED 接 MODEL_EXPIRED=18；G3 revision 跨重启撞串（建议并入 boot/世代段）；`_decision_expiry_warned` 上界；**preview 契约全线休眠**（debug 桥白名单缺 plan_id/两修订，全树无客户端可激活）——补白名单或文档明示。

## J. 遗留清单（按优先序）

1. **真机验收**：G1 的 120s 窗口、G2 preview 语义、令牌过期 SKIPPED_QUALITY 分级（FULL 档真机首验重点）。
2. R1（MoveTo/M8 收口）→ R2 → R3。
3. 原审查剩余中危：M4（臂侧观测无 frame/tf_stale 校验）/M5/M6（选果门）/M7（moveit_enabled:=false 假就绪）/M8/M9（MoveTo 码错用+不读 arrived）/M10/M12/M14/M16-M19。
4. 低危聚簇（IDL 死管道等）留接口清理轮。
5. mock launch 冒烟待相机空闲补跑；PF-1 推迟相机轮。
