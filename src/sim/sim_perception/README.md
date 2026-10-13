# sim_perception

peach 视觉感知栈的**快照复制**（用户裁定 2026-10-13，P3）：`scene_perception` +
`target_reconstruction` 两 lifecycle 节点 + `vision/{common,domain}` 纯核 + 两份
config，跑在仿真相机流上（`sim_bringup/sim.launch.py` + `perception.launch.py`）。

## 快照口径

- **来源**：`src/peach/peach_harvester`（复制时 import 前缀 `peach_harvester.`
  已重写为 `sim_perception.`）。peach 组持续演进，**改进需人工同步到本包**。
- **不复制**：`peach_interfaces`（防 IDL 双定义）、`peach_common`
  （yaml_params/qos/paths/lifecycle，exec_depend 复用）、模型权重 best.pt /
  mobile_sam.pt（config 路径仍指向 peach_harvester 安装数据）。
- **节点名 / 话题名保持 peach 原样**（`peach_scene_perception_node` /
  `peach_target_reconstruction_node`，`/camera/*`、`/peach/perception/*`）：
  仿真独立 ROS domain 运行，无冲突；console 脚本名加 `sim_` 前缀避免 overlay 撞名。
- 不含 supervisor / scene_obstacles（P4 采摘执行与 MoveIt 场景注入后续轮）。

## 运行

```bash
# 1) 统一机体仿真（含腕载 rgbd 与相机桥）
ros2 launch sim_bringup sim.launch.py gui:=false
# 2) 感知栈（感知→重建 lifecycle 顺序 + managed_nodes_activated 闩锁桥）
ros2 launch sim_bringup perception.launch.py
# 3) 驱动采集：begin_scene → target_observations → build_target_model
```

接口契约（相机五话题 RELIABLE / 精确 stamp TF / AUBO 六关节 /joint_states /
use_sim_time）见 `sim_bringup` README 与 `doc/architecture.md`。
许可 BSD-3-Clause（承袭 peach_harvester）。
