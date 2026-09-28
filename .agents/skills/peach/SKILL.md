---
name: peach
description: >-
  Peach harvest stack source map: RunHarvest FSM, vision (YOLO/SAM/TSDF),
  peach_arm ExecuteTarget stages, GraspDecision gates, IDL/QoS/TF, package
  file tree. Use when editing src/peach_*, tracing harvest flow, wiring
  topics/actions, changing harvest_fsm, scene perception, reconstruction,
  manipulation_skills, GraspDecision, FailureCode, or peach_interfaces.
---

# Peach 采摘栈 · AI 源码地图

本技能是 **peach 源码逻辑索引**，给改代码的 AI 用。不是第四份活文档。

| 权威 | 路径 | 写什么 |
|------|------|--------|
| 怎么改 | [`AGENTS.md`](../../../AGENTS.md) | MUST / DEFAULT / KEEP / UNWIND |
| 现在跑什么 | [`docs/architecture.md`](../../../docs/architecture.md) [`docs/io.md`](../../../docs/io.md) [`docs/testing.md`](../../../docs/testing.md) | SNAPSHOT；改行为同轮改这三份 |
| 图名/QoS | [`interface_manifest.yaml`](../../../src/peach_interfaces/config/interface_manifest.yaml) | **高于 IDL 文件头注释**（头注释常残留 `peach_executor`） |
| 本技能 | 本目录 | 读哪几个文件、谁调谁、禁区 |

冲突时：MUST > 源码+清单 > 三份活文档 > 本技能。IDL 注释与清单打架以清单为准。

## 先读哪一份

| 任务 | 打开 |
|------|------|
| 一批怎么跑、FSM、选果 | [harvest-flow.md](harvest-flow.md) |
| 一帧怎么变成观测 / 一颗怎么重建 | [vision.md](vision.md) |
| ExecuteTarget、授权门、接近路径 | [arm.md](arm.md) |
| 改话题/动作/失败码/TF/身份 ID | [contracts.md](contracts.md) |
| 「这个文件干什么」 | [modules.md](modules.md) |

## 产品一句话

固定座 AUBO E5 + Percipio（或 `peach_stereo`）套袋桃采摘。现行干跑默认 **PREGRASP_ONLY**：到预抓取停住，不套入、不 SetIO、不回 `harvest_stow`。套入许可唯一权威是 **`GraspDecision.allowed`（分档：CONTACT 看 sleeve/径向余量，TOOL 看 cut/轴向余量+预抓取残差）**。感知 ACCEPT 只当初值/可视化。

## 进程与节点（不要用包名当图名）

整栈入口：`peach_bringup/launch/harvest_system.launch.py`（先预检拒旧实例）。

```
harvest_system
  aubo_e5_bringup          # 驱动只读；mock=GenericSystem+JTC
  serial_imu               # 不进 lifecycle
  imu_follow_servo         # 仅 tool_profile=adaptive_cylinder_v1
  peach_harvester.brain    # 一进程三节点（图名不变）
      peach_scene_perception_node
      peach_target_reconstruction_node
      peach_supervisor
  peach_arm                # Lifecycle；MoveIt 伴随节点同 executor
  peach_observability      # 8090 + 会话 bag；不发运动
  nav2_lifecycle_manager 名 peach_lifecycle_manager
      名单：场景 → 重建 → 技能 → 调度
  peach_lifecycle_flag_bridge  → /peach/lifecycle/managed_nodes_activated
```

`peach_vegetation` / `peach_sim` / `peach_stereo`（默认前端仍 percipio）/ IVG **不进** harvest 业务核。导航包已归档。

## 数据单向（R2）

```
RGB-D → 感知(事实) → 观测/初值
                 ↓
              重建(模型+GraspDecision)
                 ↓
调度(选果/FSM/账本) --ExecuteTarget/Survey--> 臂(运动)
                 ↑ CheckReachability 是唯一反向 IK 查询
```

禁止：感知发运动；技能写 `ledger.json`；技能调重建 Trigger；调度订 `initial_pose` / `grasp_decision` 当选果源（选果只订 `target_observations`）。

## 改代码纪律（peach 特有）

1. **批次态只经** `harvest_fsm.react` / reducer / `_apply`。节点禁止 `self._batch_state = …`。
2. **跨包只走 IDL**。改名字：IDL → manifest → `docs/io.md` → 各端；先编 `peach_interfaces`。
3. **图名用相对名+launch remap**，禁止 `declare_parameter("image_topic")` 当主接线。
4. **重建禁止 latest TF**（积分必须精确 stamp）。深度是 uint16 **毫米**，边界换成米。
5. **新算法缝默认 pluginlib**；现存 `PIPELINES_BY_IMPL` / `REFITTERS_BY_IMPL` 是 UNWIND，不要再扩 dict。
6. **纯核零 ROS**：FSM / 选果 / 位姿管线 / SafetyGate。节点只 decode / 调纯核 / publish。
7. **驱动九包只读**。未授权不动真机、不 SetIO。软件停不得称 e-stop。
8. **测完清进程**（见 AGENTS 第 9 章）。

## 身份四层（改 ID 前必看 contracts.md）

`target_id`（感知注册表）→ `request_id/run_id/cycle_id`（批次）→ `plan_id`+模型七元组 → `clearance`（复用 target_id，不铸新 ID）。`valid_until` **冻结不续签**。

## 参数单源

- Python：`peach_common.yaml_params.attach` + `config/<节点>.yaml`
- C++ `peach_arm`：GPL `peach_arm.yaml`
- 停位几何：`grasp_standoffs.yaml` 注入 scene/recon
- 工具档案：`tool_profiles.py`（`hollow_cylinder_v1` / `adaptive_cylinder_v1`）

## 验证

受影响包 `colcon test --packages-select …`。改 IDL/接线再跑 `python3 src/peach_interfaces/scripts/check_interface_manifest.py`。停栈后 `pgrep` 无残留。
