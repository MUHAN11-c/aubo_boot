# 契约：名字、QoS、身份、失败码

真源：`src/peach_interfaces/config/interface_manifest.yaml`。核对：`python3 src/peach_interfaces/scripts/check_interface_manifest.py`。字段目录：`src/peach_interfaces/README.md`。活文档：`docs/io.md`。

## 命名陷阱

IDL 文件头仍写 `/peach_executor/…`、生产方 `peach_executor`。现行图名：

| 别信注释 | 实际 |
|----------|------|
| `/peach_executor/run_harvest` | `/peach_supervisor/run_harvest` |
| `/peach_executor/state` | `/peach_supervisor/state` |
| 节点 `peach_executor` | `peach_supervisor`（包仍是 peach_harvester） |
| 技能进程 `manipulation_skills` | 图名 `peach_arm` |

改字段流程：IDL 注释（单位/frame/stamp）→ manifest → io.md → 各端 pub/sub → **先编接口包**。

## 调度只订这两条跨包流

`/peach/perception/target_observations`、`/peach/lifecycle/managed_nodes_activated`。不要把 `initial_pose` / `grasp_decision` 写成调度订阅。

## 关键通道

| 名字 | 类型 | QoS 要点 | 谁→谁 |
|------|------|----------|-------|
| `/peach_supervisor/state` | HarvestState | RELIABLE + transient_local d1 | 调度 → 感知/重建/观测 |
| `/peach_supervisor/events` | CanonicalEvent | RELIABLE + TL d50 | 调度 → 观测 |
| `/peach/perception/target_observations` | PeachTargetObservationArray | RELIABLE volatile d10 | 感知 → 调度/重建/臂/观测 |
| `/peach/perception/initial_pose` | BagGraspCandidateArray | TL d1 | 感知 → **仅重建** |
| `/peach/reconstruction/grasp_decision` | GraspDecision | TL d1 | 重建 → 臂/观测 |
| `/peach/batch/enables` | Enables | TL d1 | 调度 SetEnables 后广播 → 臂命令门 |
| `/peach_arm/execute_target` | ExecuteTarget | action | 臂 ← 调度 |
| `/peach_arm/survey_scene` | SurveyScene | action | 臂 ← 调度 |
| `/peach_arm/check_reachability` | CheckReachability | service | 臂 ← 调度 |
| `/peach_scene_perception_node/begin_scene` | BeginScene | service | 感知 ← 调度 |
| `/peach_target_reconstruction_node/build_target_model` | BuildTargetModel | action | 重建 ← 调度 |

传感图像/点云：BEST_EFFORT + volatile。命令/lifecycle：RELIABLE。`/tf` volatile；`/tf_static` transient_local。RELIABLE 订户收不到 BEST_EFFORT 发布。

预留（无生产方）：`NavigateToWorksite`、`HarvestTargetReport`、`VehicleState`、`HarvestOperationStatus`。

## FailureCode（禁止解析自由文本）

见 `FailureCode.msg`：NONE=0 … PLAN_MISMATCH=20。事件、账本、GraspDecision、ExecuteTarget.Result 写常量。常见：OBSERVE_FAILED、BUILD_FAILED、DECISION_REJECTED、PREGRASP_RESIDUAL、SLEEVE_PLAN_FAILED、EXACT_TF_MISSING、DYNAMIC_BUDGET_NEGATIVE、DEGRADED_CONTACT_FORBIDDEN、MODEL_STALE/EXPIRED、MODEL_IDENTITY_INCOMPLETE。

ExecuteTarget.outcome：SUCCEEDED=0、SKIPPED_QUALITY=1、SKIPPED_UNREACHABLE=2、FAILED=3、CANCELED=4。completion_level 是到达档，采摘成功另看 `HarvestResult.grasped`（切断+撤退均确认）。

## 身份

| ID | 铸造 | 失效 |
|----|------|------|
| `target_id` | `TargetRegistry._commit_match` → `target_{N}` | clear 不复位计数；贴边不攒确认 |
| `scene_epoch` | BeginScene +1 | 选果须观测世代==调度值且>0。臂 SurveyScene result **恒填 0**（ID-4） |
| `request_id`/`run_id` | RunHarvest；空则 `auto_<时间戳>` | 账本目录名，不复用 |
| `cycle_id` | `{run_id}:{target_id}` | 单颗 |
| `plan_id` | `{request_id}:{target_id}:{generation}` | 批流不发 PREVIEW，plan 契约在批流 inert（ID-2） |
| 七元组 | run/epoch/target/model/tool/calib/config revision | 受理只查非空 |
| `clearance` | 不铸新 ID | 绑 target_id；valid_until 冻结 |

感知 `harvest_run_id`（µs+snapshot）是臂锁定缓存生命周期边界，与调度 `run_id` 不是同一个字符串。

## TF（合法帧集；多出来=旧实例，预检拒启）

动态 `/tf`：base→shoulder→upperArm→foreArm→wrist1/2/3（关节序 MUST）。

静态：world→base/table；wrist3→{camera_body,camera_link,tool_axis}；tool_axis→{tcp,sleeve_mouth,cutting_plane,tool_body}；camera_link→{color,depth}→optical；`extrinsics_publisher` **只**发 wrist3→camera_link，其余 URDF。手眼事实源：`src/aubo_hand_eye_calibration/hand_eye/active.yaml`。

光学系 `*_optical`（REP-103：z 前 x 右 y 下）。禁止第二套 StaticTransformBroadcaster 叠同一 child。

## 深度与单位

Percipio 深度 uint16 毫米（KEEP 产品例外）。IDL 新字段优先米。体轴 x 前 y 左 z 上。
