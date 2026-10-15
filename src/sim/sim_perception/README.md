# sim_perception

peach 视觉感知栈的**快照复制**（用户裁定 2026-10-13，P3）：`scene_perception` +
`target_reconstruction` 两 lifecycle 节点 + `vision/{common,domain}` 纯核 + 两份
config，跑在仿真相机流上（`sim_bringup/sim.launch.py` + `perception.launch.py`）。

## 快照口径

- **来源**：`src/peach/peach_harvester`（复制时 import 前缀 `peach_harvester.`
  已重写为 `sim_perception.`）。peach 组持续演进，**改进需人工同步到本包**。
- **不复制**：`peach_interfaces`（防 IDL 双定义）、`peach_common`
  （yaml_params/qos/paths/lifecycle，exec_depend 复用）、模型权重 best.pt /
  mobile_sam.pt。
- **节点名 / 话题名保持 peach 原样**（`peach_scene_perception_node` /
  `peach_target_reconstruction_node`，`/camera/*`、`/peach/perception/*`）：
  仿真独立 ROS domain 运行，无冲突；console 脚本名加 `sim_` 前缀避免 overlay
  撞名（`sim_scene_perception_node` / `sim_target_reconstruction_node`）。
- 不含 supervisor / scene_obstacles（P4 采摘执行与 MoveIt 场景注入由
  `orchard_harvest` 编排 CLI 承接演示闭环）。

## launch 组成（`sim_bringup/perception.launch.py`）

两个 LifecycleNode（参数 = 本包 share 两份全量 yaml，`ParameterFile
allow_substs` 装入）+ `nav2_lifecycle_manager`（名 `peach_lifecycle_manager`，
顺序 **感知 → 重建**，autostart；`bond_timeout` 默认 0，与现行真机栈同口径）。
验收路径：`~/begin_scene` → `/peach/perception/target_observations` 非空 →
`~/build_target_model` 出 refined_pose（`min_views=1` 单视收口）。

## sim 分叉调参（2026-10-14 P4b，键名与真机一致、数值分叉）

- **检测阈值**：gz-ogre2 对重建袋的渲染与 YOLO 训练分布有残余域差——纸袋
  albedo 烘回后可检出但置信度保守（0.17-0.19 vs Blender 渲染 0.7+；纹理修复
  前为全零）。`yolo_conf` / `min_detection_conf` 降至 **0.12** 采信这些正确检出，
  误检由 SAM 掩膜质量门 + 深度窗 + 几何门兜底；真机 peach_harvester 保持
  0.35/0.4。
- **锁集窗口**：gz 到帧 ~3.4 Hz、YOLO+SAM worker 仅 ~0.3-0.5 fps，真机 10 帧
  窗口（≈2.8 s）worker 一帧未处理完即锁空集（实测）；放大到 **60 帧（≈17 s
  墙钟）**，worker 可处理 5-8 帧。
- config 是 nav2 式全量清单（键名冻结、真机命令零破坏）：逐帧读取键
  `ros2 param set` 热生效；模型/管线等构造期捕获键改后须重启。

## 运行

```bash
# 1) 统一机体仿真（含腕载 rgbd 与相机桥）
ros2 launch sim_bringup sim.launch.py gui:=false
# 2) 感知栈（感知→重建 lifecycle 顺序）
ros2 launch sim_bringup perception.launch.py
# 3) 驱动采集：begin_scene → target_observations → build_target_model
```

接口契约（相机五话题 RELIABLE / 精确 stamp TF / AUBO 六关节 `/joint_states` /
use_sim_time）见 `sim_bringup` README 与 `doc/architecture.md`。

## 测试

`test/` 移植自 `peach_harvester` 纯核行为测试（快照 fork 的回归门）：
**基线 = fork 点（P3, 9e30b415）+ 已回灌到本包的融合加固对应用例**；fork 后
母包新增行为的测试不移植（本包代码未含该行为），peach 侧后续演进同步代码时
随轮同步测试。运行：

```bash
colcon test --packages-select sim_perception
# 或直跑（须先 source Jazzy 与工作区）：
python3 -m pytest src/sim/sim_perception/test -q
```

open3d 缺失环境按测跳过（不影响其余收集）；模型权重不进单测——推理链路属
launch_testing / e2e 范畴。

**权重依赖**：两份 config 的 `yolo_model_path` / `sam_model_path` 指向
`$(find-pkg-share peach_harvester)/model/`（launch 期展开）——单建 sim 组时须
同装 peach_harvester 或改指本地权重，否则拒启。

许可 BSD-3-Clause（承袭 peach_harvester，见 LICENSE）。
