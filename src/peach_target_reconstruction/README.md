# peach_target_reconstruction

单目标局部重建：精确时刻 TF 作初值，有界 ICP 修小误差，合格帧进 TSDF，再精化袋/果几何。

## 流程

```
IDLE
  → 绑定目标（执行器 HarvestState.target_id 优先，否则感知 selected）
  → COLLECTING：每帧按 depth.stamp 查 base←camera；失败跳帧，不用 latest TF
  → 掩膜/漂移/邻目标间距门；拒绝帧不进 TSDF
  → ICP 相对已有表面（±小范围）；越界丢弃
  → BuildTargetModel 或 ~/finalize_reconstruction：视角够则提取网格并 refit
```

默认 `capture.auto_mode: true`：有候选就采。技能 BT 观察扫描时仍会调 `reset` / `finalize` Trigger。批次 `RunHarvest` 当前先派 `ExecuteTarget`（内含观察），不把空会话的 BuildTargetModel 挡在抓取前面。

## 启动

```bash
ros2 launch peach_target_reconstruction reconstruction.launch.py
```

节点：`peach_target_reconstruction_node`。参数：`config/reconstruction.yaml`。入口用 `MultiThreadedExecutor`，避免 `BuildTargetModel` 等待时堵住 RGB-D。

## 订阅

RGB-D 三件套（与感知相同话题）、`/peach/perception/initial_pose`、`target_observations`、`/joint_states`、闩锁 `/peach_task_executor/state`。

## 发布（`/peach/reconstruction/*`，多路闩锁）

`local_cloud`、`tsdf_cloud`、`status`、`diagnostics`、`diagnostics_debug`、`grasp_decision`、`markers`、`refined_pose` / `axis` / `diagnostics`。

## 动作 / 服务

- 动作 `~/build_target_model`：绑定 `goal.target_id`，等到 `min_views` 后 finalize
- Trigger（人工或技能 BT）：`start_reconstruction`、`capture_frame`、`remove_last_frame`、`reset_reconstruction`、`finalize_reconstruction`、`save_session`、`query_reconstruction_state`

## 要点

- `min_views` 默认 4；拒绝帧永不积分。
- 重建输出 `frame_id` 为 `base_link`。
- Session 落盘目录默认工作区 `peach_sessions/`（gitignore）。
