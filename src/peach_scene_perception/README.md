# peach_scene_perception

套袋桃 RGB-D 感知：检测 → 分割 → 深度几何 → 世界系身份。只发观测，**不发运动指令**。

## 流程

```
同步 RGB + 深度 + CameraInfo
  → YOLO 检测（宽进）+ IoS 去重
  → MobileSAM 整帧批量分割
  → 袋/果深度管线（圆柱/球、入口、行程建议）
  → 精确时间戳 TF 变到 output_frame（默认 base_link）
  → 本帧全部锚点一次匈牙利分配（马氏门控；歧义 AMBIGUOUS）
  → 发布 /peach/perception/* 与全局快照
```

身份只在 TF 可用（ok/stale）时注册；`tf_unavailable` 保留帧内序号并打 `target_untracked`，避免相机系污染世界系表。

`BeginScene` 清身份表并推进场景世代。锁定窗口、优先级排序仍在本节点的 `harvest_plan`（批次选择权在 `peach_task_executor`，执行器按确认目标自己选）。

## 启动

```bash
ros2 launch peach_scene_perception peach_pose.launch.py
```

节点：`peach_scene_perception_node`。参数权威源：`config/peach_pose.yaml`。解释器由 setup.py 指到 `aubo_py3.12`。

## 订阅

| 话题 | 内容 |
|------|------|
| `/camera/color/image_raw` | RGB |
| `/camera/depth/image_raw` | 深度（须与彩图配准） |
| `/camera/color/camera_info` | 内参 K |

## 发布（`/peach/perception/*`）

| 话题 | 内容 |
|------|------|
| `target_observations` | 稳定 ID、确认、锁定集、2D/3D 候选 |
| `initial_pose` | 抓取候选数组 |
| `axis` | 当前帧最优平移轴 |
| `detections` / `masks` / `diagnostics` / `markers` | 检测、mono16 标签掩膜、拟合、RViz |
| `single_cloud` | 框内点云 |
| `debug_image` | 叠加图（默认关） |
| `harvest_state` | 计划 JSON（闩锁） |

## 服务

- `~/begin_scene`（`BeginScene`）：新场景，清记忆
- `~/query_harvest_state`（`Trigger`）：查询当前集合与数据目录

没有 `reset_global_targets` / `complete_selected_target` / `clear_target_memory`；批次账本在执行器。

## 要点

- 输出系默认 `base_link`，依赖手眼静态 TF。
- `depth_scale_unit` 默认 `0.25`（Percipio）；真毫米回放设 `1.0`。
- 生产档 `publish_debug_image: false`。
- 分割器可插拔（`segmenter.impl`），当前默认 MobileSAM。
