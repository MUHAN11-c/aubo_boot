# peach_target_reconstruction

单目标局部重建：**Lifecycle**，Active 后才积分并受理 `BuildTargetModel`。

总览：[docs/flow.md](../../docs/flow.md)。参数：`config/reconstruction.yaml`。

## 从哪读

| 文件 | 职责 |
|------|------|
| `reconstruction_node.py` | 节点：绑定目标、RGB-D 回调、`BuildTargetModel` |
| `_accept_frame` | `_crop_for_icp` → `_register_cloud` → `_integrate_tsdf` |
| `_wait_min_views` | 事件等待视角数（不 `sleep` 轮询堵 executor） |
| `capture_gate.py` | 三段门：锁 → **精确 stamp TF** → 重校验 |
| `icp_refiner.py` / `tsdf_volume.py` / `cloud_builder.py` | ICP、TSDF、点云 |
| `geometry_refiner.py` | finalize 后重拟合（用 `peach_common_py.fitting`） |
| `mask_gate.py` / `view_coverage.py` / `frame_collector.py` | 掩膜、视角覆盖、帧缓存 |
| `publishers.py` / `session_io.py` | 话题与 session 落盘 |

入口 `MultiThreadedExecutor`：等待 `min_views` 时仍能收 RGB-D。

## 谁调谁

| 方向 | 内容 |
|------|------|
| 被执行器调 | `~/build_target_model`（绑定 `goal.target_id`，等到 `min_views` 后 finalize） |
| 订阅 | 与感知相同的 RGB-D；`initial_pose`、`target_observations`；闩锁 `HarvestState`（`target_id` 优先于感知 selected） |
| 发布 | `/peach/reconstruction/*`（精化位姿/轴/诊断、`shape_hypothesis`、抓取许可） |
| 技能 | **不再**调本包 `reset`/`finalize`；人工 Trigger 仍保留 |

积分 **禁止 latest TF**：`depth.stamp` 查不到 `base←camera` 则跳帧。

## 流程

```
IDLE
  → 绑定（执行器 target_id 优先）
  → COLLECTING：门控 → ICP（±小范围）→ TSDF；拒绝帧永不积分
  → BuildTargetModel：reset、等 min_views、finalize + refit
```

默认 `capture.auto_mode: true`。批次里执行器先发 Build，再并行观察扫描。

## 启动

```bash
ros2 launch peach_target_reconstruction reconstruction.launch.py
```

节点：`peach_target_reconstruction_node`。

## 动作 / 服务

- 动作 `~/build_target_model`
- Trigger：`start_reconstruction`、`capture_frame`、`remove_last_frame`、`reset_reconstruction`、`finalize_reconstruction`、`save_session`、`query_reconstruction_state`

`min_views` 默认 4。输出 `frame_id` 为 `base_link`。Session 默认 `peach_sessions/`（gitignore）。
