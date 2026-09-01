# peach_perception

四个能力包之一：**视觉算法**。一包两节点：场景身份与锁定集；当前目标局部模型与 `GraspDecision`。不发运动、不选下一颗、不写账本。重建积分只用精确 stamp TF。
节点之间、以及与技能/执行器之间，只走 [`peach_interfaces`](../peach_interfaces/README.md)，不互相 import 业务模块。详细作用见 [docs/architecture.md](../../docs/architecture.md) §3 `peach_perception`。

总览：[docs/architecture.md](../../docs/architecture.md)。契约：[docs/io.md](../../docs/io.md)。

文件树按 ROS 2 `ament_python` 惯例：`launch/` `config/` `resource/` 在包根，Python 模块与功能包同名。

```
peach_perception/
  peach_perception/          # import peach_perception.*
    common/                  # 拟合、深度、时钟、runs/
    scene_perception/        # peach_scene_perception_node
      offline/               # bag_baseline 等离线脚本
    target_reconstruction/   # peach_target_reconstruction_node
  config/  launch/  model/
  resource/  test/
  package.xml  setup.py  setup.cfg
```

## 节点与契约

| 角色 | 节点 | 输入 | 输出 | 不做什么 |
|------|------|------|------|----------|
| 看 | `peach_scene_perception_node` | RGB-D；`HarvestState`；`BeginScene` | `/peach/perception/*` | 不重建、不选下一颗、不运动 |
| 建 | `peach_target_reconstruction_node` | 同一套 RGB-D；感知观测；`HarvestState.target_id`；`BuildTargetModel` | `/peach/reconstruction/*` | 不检测、不写账本、不用 latest TF 积分 |

作业目标只认执行器 `~/state.target_id`。几何库在 `peach_perception.common`（拟合/深度/时钟），无话题。

## 从哪读

| 文件 | 读什么 |
|------|--------|
| `peach_perception/scene_perception/scene_perception_node.py` | 感知外壳：`_on_rgbd` → `_process_rgbd` |
| `peach_perception/scene_perception/pipeline.py` | 检测、分割、前景、袋/果位姿 |
| `peach_perception/scene_perception/identity.py` | 世界系身份、锁定窗、记忆锚点 |
| `peach_perception/target_reconstruction/target_reconstruction_node.py` | 重建外壳：`_accept_frame`、`BuildTargetModel` |
| `peach_perception/target_reconstruction/capture.py` | 锁 → 精确 stamp TF → 重校验 |
| `peach_perception/common/geometry.py` | 球/柱 RANSAC、深度单位、TF 纯函数 |

参数：`config/scene_perception.yaml`、`config/target_reconstruction.yaml`。

## 启动

```bash
ros2 launch peach_perception scene_perception.launch.py
ros2 launch peach_perception target_reconstruction.launch.py
```

整栈由 `peach_executor/harvest_system.launch.py` include，`autostart:=false`。
