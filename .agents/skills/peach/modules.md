# 模块与文件

改某处先打开「读这个」列，再扩读。测试在各包 `test/`。

## peach_interfaces

IDL 唯一契约包，不跑节点。`config/interface_manifest.yaml` + `scripts/check_interface_manifest.py`。

## peach_common

不跑节点。`yaml_params.py`（attach）、`param_rules.py`、`qos.py`、`paths.py`、`lifecycle.py`（bond 守卫）。各 peach 包旧路径留 shim。

## peach_bringup

| 文件 | 作用 |
|------|------|
| `launch/harvest_system.launch.py` | 整栈入口、预检、Include 顺序 |
| `preflight.py` | 拒叠 RSP / extrinsics |
| `autostart_client.py` | 仅 autostart=true |
| `lifecycle_flag_bridge.py` | is_active → managed_nodes_activated |
| `config/bringup.yaml` | 部署参数 |

不含业务。应用 launch Include `aubo_e5_bringup`，不复制 RSP。

## peach_harvester · 大脑

`brain.py`：三节点进同一进程。`launch/brain.launch.py`：unnamed Node（勿 `name=` 重映射）。独立入口仍可分进程起三节点。

### supervisor/

| 文件 | 作用 |
|------|------|
| `executor_node.py` | 调度 Lifecycle；命令循环；**唯一**批次动作客户端 |
| `harvest_fsm.py` | 事件→命令纯核表 |
| `domain/reducer.py` | 三维状态 + 世代/事务去重 |
| `batch.py` | next_target / 账本 / summary |
| `observe.py` | fast 档视点信号纯核 |
| `params.py` + `config/peach_supervisor.yaml` | 参数 |

`cycle_core/`：`batch_policy.py`（采收率/时限/view_policy）、`view_policy.py`、`view_planner.py`（与臂 C++ 视点规划对拍的 Python 口）。

### vision/scene_perception/

| 文件 | 作用 |
|------|------|
| `scene_perception_node.py` | Lifecycle 壳 |
| `pipeline.py` | process(frame) 门面 |
| `inference.py` | YOLO + MobileSAM |
| `pose_pipelines.py` | 袋圆柱 / 果球体 |
| `identity.py` | 注册表、锁定窗 |
| `image_gates.py` | 深度/贴边/分割框 |
| `plan_updater.py` | 帧级计划 |
| `msg_builders.py` | IDL 组装 |

### vision/target_reconstruction/

| 文件 | 作用 |
|------|------|
| `target_reconstruction_node.py` | Lifecycle 壳 + Build 动作 |
| `reconstruction_core.py` | 帧环/自动机/发布宿主 |
| `capture.py` | 采帧门、状态 IDLE/COLLECTING |
| `integrate.py` | ICP + TSDF |
| `refine.py` | refitter dict |
| `refit_orchestrator.py` | 融合→决策 |
| `publish.py` | GraspDecision / TargetModel 组装 |
| `session.py` / `session_recorder.py` | 会话与落盘 |

### vision/common/

`geometry.py`、`tool_budget.py`、`runtime.py`（BoundedWorker、HarvestDataStore）、`ros/clock_adapter.py`。感知 **不写** `ledger.json`，可写 `perception_data`。

`config/`：`scene_perception.yaml`、`target_reconstruction.yaml`、`grasp_standoffs.yaml`。

## peach_arm

| 文件 | 作用 |
|------|------|
| `src/main.cpp` | 技能节点 + MoveIt 伴随节点 |
| `src/manipulation_skills_node.cpp` | Lifecycle / 订阅 / 状态 JSON |
| `src/cycle.cpp` | 动作受理、authorizeStage、Survey、executeAction |
| `src/stages.cpp` | executeCycle 阶段序列 |
| `src/grasp_task.cpp` | MTC 接近/套入/撤退 |
| `src/motion.cpp` | MoveIt 运动接口、拍照位、接触检测接线 |
| `src/move_to.cpp` | MoveTo + authorizeTransit + enables 心跳 |
| `src/target_cache.cpp` | selected/locked/refined/decision 三索引 |
| `src/view_planner.cpp` | 观察候选 |
| `src/tool_actuator.cpp` | SetIO；默认关 |
| `src/safety_gate.cpp` `quality_gate.cpp` | 纯核门 |
| `include/peach_arm/cycle_context.hpp` | CycleState / CycleContext |
| `include/peach_arm/grasp_geometry.hpp` | 胶囊/反爬 |
| `include/peach_arm/trajectory_guard.hpp` | 绕行护栏 |
| `include/peach_arm/contact_monitor.hpp` | 腕轴电流，默认关 |
| `config/peach_arm.yaml` | GPL 参数 |

测试：`test/test_*`.cpp / `.py`（gtest 允许）。

## peach_observability

`observability_node.py`：8090 HTTP（`http_server.py`、`debug_actions.py`）+ 会话 bag（`recorder.py`、`catch_all_recorder.py`）+ 停栈 `bag_report`。只读。调试 POST 运动须 `debug.motion_enabled`，仍过臂侧命令门。

## peach_vegetation

独立 launch。Frangi/叶掩膜 2D。**不写 PlanningScene，不进 harvest_system**。

## peach_stereo

可选相机前端（`camera_frontend:=stereo`）。话题与 Percipio 同构，点云多 `confidence`。规格见包 README。与 percipio **互斥**（相机连接独占）；stereo include 必须在 aubo bringup **之前**（launch 参数全局沉降）。

## peach_sim

Gazebo/外观重建离线场景，**不进 harvest_system**。只桥 `/clock` 与 `/joint_states`。当前 Blender 入口 `src/peach_sim/reconstruction/`。GT 清单不消费系统 ID。

## peach_system_tests

isolated mock launch_testing + 接近回放塔（`replay_oracle.py` 纯几何，不消费系统 ID）。`test_e1_supervisor_chain.py` 调度链；`test_tool_profile_smoke.py` 三把末端逐档冒烟（`PEACH_LT_TOOL_PROFILE` 三注册，域 91/92/93：TF `wrist3_Link→tcp`==档案 ±2 mm、`/peach_arm tool.profile_id` 对齐、imu_follow 按名单起/不起）。

## 驱动（MUST 只读）

`aubo_e5_hardware` / `aubo_e5_controllers` / `aubo_dashboard`（bringup 不起）/ `aubo_e5.ros2_control.xacro` / bringup / controllers.yaml。透传：FollowJointTrajectory → Passthrough → `AuboE5Hardware::write`。关节序冻结六名。手眼：`src/aubo_hand_eye_calibration/hand_eye/active.yaml`。

## 非 peach、勿混进核

IVG 三包、`imu_follow`（仅 `adaptive_shear_v1` Include）、`peach_navigation`（已归档）。`blender_orchard/` 已并入 `peach_sim/reconstruction/`（2026-09-28，4ba0631），不再独立存在。
