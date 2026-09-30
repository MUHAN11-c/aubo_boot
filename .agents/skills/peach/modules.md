# 模块与文件

改某处先打开对应文件，再扩读。测试在各包 `test/`。

## peach_interfaces

IDL 唯一契约包，不跑节点。`config/interface_manifest.yaml` + `scripts/check_interface_manifest.py`。

## peach_common

不跑节点。`yaml_params.py`（attach）、`param_rules.py`、`qos.py`、`paths.py`、`lifecycle.py`（bond 守卫）、`ros_log_bridge.py`（stdlib→/rosout）、`event_meter.py`。各 peach 包旧路径留 shim。

## peach_bringup

| 文件 | 作用 |
|------|------|
| `launch/harvest_system.launch.py` | 整栈入口：预检（磁盘+旧实例+域 ID）、启动事实 JSON、Include 顺序 |
| `preflight.py` | 拒叠 RSP / extrinsics；磁盘余量 |
| `autostart_client.py` | 仅 autostart=true；硬等 selfcheck_passed |
| `lifecycle_flag_bridge.py` | is_active → managed_nodes_activated |
| `config/bringup.yaml` | 部署参数 |

不含业务。应用 launch Include `aubo_e5_bringup`，不复制 RSP。

## peach_harvester · 大脑

`brain.py`：感知+重建+调度三节点同一进程。`launch/brain.launch.py`：unnamed Node（勿 `name=` 重映射）。`peach_scene_obstacles` **不在** brain 内。

### supervisor/

| 文件 | 作用 |
|------|------|
| `executor_node.py` | 调度 Lifecycle；命令循环；Survey 后发 obstacles_refresh；订 grasp_decision 作令牌 |
| `harvest_fsm.py` | 事件→命令纯核表 |
| `domain/reducer.py` | 三维状态 + 世代/事务去重 |
| `batch.py` | next_target / 资格谓词 / 账本 / summary |
| `observe.py` | fast 档视点信号纯核 |
| `params.py` + `config/peach_supervisor.yaml` | 参数 |

`cycle_core/`：`batch_policy.py`、`view_policy.py`、`view_planner.py`（与臂 C++ / peach_sim 补视几何对拍）。

### vision/scene_perception/

`scene_perception_node.py` 壳；`pipeline.py` 门面；`inference.py` YOLO+SAM；`pose_pipelines.py` 袋/果；`identity.py` 注册表；`image_gates.py`；`plan_updater.py`；`msg_builders.py`。

### vision/target_reconstruction/

`target_reconstruction_node.py` 壳+Build；`reconstruction_core.py` 帧环宿主；`capture.py` 采帧门；`integrate.py` ICP+TSDF；`refine.py` refitter dict；`refit_orchestrator.py`；`publish.py` GraspDecision 组装；`session.py` / `session_recorder.py`。

### vision/scene_obstacles/

| 文件 | 作用 |
|------|------|
| `node.py` | 独立进程壳；订点云/refined_pose/refresh；ApplyPlanningScene |
| `core.py` | 纯核滤除链（体素先行） |
| `params.py` + `config/scene_obstacles.yaml` | 对象 id 须与臂 ACM 同名 |

### vision/common/

`geometry.py`、`tool_budget.py`、`runtime.py`（BoundedWorker、HarvestDataStore）、`ros/clock_adapter.py`、`tool_profiles.py`。感知 **不写** `ledger.json`，可写 `perception_data`。

`config/`：`scene_perception.yaml`、`target_reconstruction.yaml`、`grasp_standoffs.yaml`、`scene_obstacles.yaml`。

## peach_arm

| 文件 | 作用 |
|------|------|
| `src/main.cpp` | 技能节点 + MoveIt 伴随节点 |
| `src/manipulation_skills_node.cpp` | Lifecycle / staging IK 梯子 / ACM 激活重试 |
| `src/cycle.cpp` | 动作受理、authorizeStage、Survey、executeAction |
| `src/stages.cpp` | executeCycle 阶段序列 |
| `src/grasp_task.cpp` | MTC；**tryStagingTransit = v4 接近真源** |
| `src/motion.cpp` | MoveIt 接口、拍照位、接触检测接线 |
| `src/move_to.cpp` | MoveTo + authorizeTransit + enables 心跳 |
| `src/target_cache.cpp` | selected/locked/refined/decision |
| `src/view_planner.cpp` | 观察候选 |
| `src/tool_actuator.cpp` | SetIO；默认关 |
| `src/safety_gate.cpp` `quality_gate.cpp` | 纯核门 |
| `include/peach_arm/acm_policy.hpp` | ③层豁免条目纯核 |
| `include/peach_arm/grasp_geometry.hpp` | 胶囊/反爬/`stagingWaypoints`/`usesImuFollowContact` |
| `include/peach_arm/cycle_context.hpp` | CycleState / CycleContext |
| `config/peach_arm.yaml` | GPL 参数（`obstacle_guard_*`） |

测试：`test/test_*`.cpp / `.py`（含 `test_acm_policy.cpp`）。

## peach_observability

`observability_node.py`：8090（`http_server.py`、`debug_actions.py`）+ 会话 bag + **订 `/diagnostics` 聚合进 8090** + 启动自检。`selfcheck/`：探针、闩锁 `/peach/observability/selfcheck_passed`、`startup.json`。只读。调试 POST 运动须 `debug.motion_enabled`，仍过臂侧命令门。

观测四层：L1 节点日志 → L2 CanonicalEvent/HarvestState/ledger → L3 `/diagnostics` → L4 `runs/session_*`。

## peach_vegetation

独立 launch。Frangi/叶掩膜 2D。**不写 PlanningScene，不进 harvest_system**。

## peach_stereo

默认相机前端（`camera_frontend:=stereo`）。话题与 Percipio 同构，点云多 `confidence`。与 percipio **互斥**；include 必须在 aubo bringup **之前**。

## peach_sim

离线 Blender 果园重建，**不进 harvest_system**。入口 `src/peach_sim/reconstruction/`（`blender_orchard/` 已并入）。只桥 `/clock` 与 `/joint_states`。纯核与 `cycle_core/view_policy.py` 补视几何对拍。

## peach_system_tests

isolated mock launch_testing + 接近回放塔（`replay_oracle.py` 纯几何）。`test_e1_supervisor_chain.py`；`test_tool_profile_smoke.py`（三把末端，域 91/92/93）；`test_scene_obstacles.py`。脚本：`scripts/drive_harvest.py` / `probe_topics.py` / `check_planning_scene.py`。

## 驱动（MUST 只读）

`aubo_e5_hardware` / `aubo_e5_controllers` / `aubo_dashboard`（bringup 不起）/ `aubo_e5.ros2_control.xacro` / bringup / controllers.yaml。透传：FollowJointTrajectory → Passthrough → `AuboE5Hardware::write`。关节序冻结六名。手眼：`src/aubo_hand_eye_calibration/hand_eye/active.yaml`。工具档案：`aubo_description/config/{shear,bite_shear,adaptive_shear}_v1.yaml`。

## 非 peach、勿混进核

IVG 三包、`imu_follow`（仅 `adaptive_shear_v1` Include）、`peach_navigation`（已归档）。
