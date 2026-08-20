# peach_scene_perception

套袋桃 RGB-D 感知：**Lifecycle**，Active 后才处理帧并受理 `BeginScene`。只发观测，**不发运动指令**。

总览：[docs/flow.md](../../docs/flow.md)。参数：`config/peach_pose.yaml`。

## 从哪读

| 文件 | 职责 |
|------|------|
| `peach_pose_node.py` | Lifecycle 外壳：同步 RGB-D、`BeginScene`、订 `HarvestState` |
| `_on_rgbd` → `_decode_rgbd` → `_process_rgbd` | 一帧入口 |
| `peach_pose/pipeline.py` | 袋/果深度几何、入口、行程建议 |
| `peach_pose/inference.py` | YOLO 检测 + MobileSAM 分割（`segmenter.impl` 可插拔） |
| `peach_pose/assignment.py` + `target_registry.py` | 匈牙利 + 马氏门控身份 |
| `peach_pose/harvest_plan.py` | 收齐窗口与锁定集（**不是**当前作业目标） |
| `peach_pose/anchor_memory.py` | 退化候选时的记忆锚 |
| `peach_pose/fitting.py` | 再导出 `peach_common_py.fitting` |
| `conversions.py` / `visualization.py` | 消息与 RViz |

当前作业目标以 **`/peach_task_executor/state.target_id`** 为准（覆盖观测 `selected` 与锁定后 SAM 焦点）。批次账本不在本包。

## 谁调谁

| 方向 | 内容 |
|------|------|
| 被执行器调 | `~/begin_scene`：清身份表、推进 `scene_epoch` |
| 订阅 | 相机三件套；闩锁 `HarvestState` |
| 发布 | `/peach/perception/*`（见下表） |
| 消费方 | 执行器、重建、技能、观测 |

身份只在 TF 可用（ok/stale）时注册；`tf_unavailable` 保留帧内序号并打 `target_untracked`。

## 流程

```
同步 RGB + 深度 + CameraInfo
  → YOLO（宽进）+ IoS 去重
  → MobileSAM 整帧批量分割
  → 袋/果深度管线
  → TF：优先精确 stamp，失败 latest 并 tf_stale
  → 本帧锚点一次匈牙利分配（歧义 AMBIGUOUS，不强制合并）
  → 发布观测与全局快照
```

## 启动

```bash
ros2 launch peach_scene_perception peach_pose.launch.py
```

节点：`peach_scene_perception_node`。解释器：`aubo_py3.12`。

## 订阅 / 发布 / 服务

订阅：`/camera/color/image_raw`、`/camera/depth/image_raw`（须与彩图配准）、`/camera/color/camera_info`。

| 话题 | 内容 |
|------|------|
| `target_observations` | 稳定 ID、确认、锁定集、2D/3D 候选 |
| `initial_pose` | 抓取候选数组 |
| `axis` | 当前帧最优平移轴 |
| `detections` / `masks` / `diagnostics` / `markers` | 检测、掩膜、拟合、RViz |
| `single_cloud` | 框内点云 |
| `debug_image` | 叠加图 |
| `harvest_state` | 计划 JSON（闩锁） |

服务：`~/begin_scene`；`~/query_harvest_state`。没有批次完成/清记忆服务。

输出系默认 `base_link`。`depth_scale_unit` 默认 `0.25`（Percipio）；毫米回放设 `1.0`。
