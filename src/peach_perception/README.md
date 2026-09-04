# peach_perception

四个能力包之一：**视觉算法**。一包两节点——`peach_scene_perception_node`（看：身份与锁定集）、`peach_target_reconstruction_node`（建：当前目标局部模型与 `GraspDecision`）。不发运动、不选下一颗、不写账本；重建积分只用精确 stamp TF。

现行设计、文件树与「从哪读源码」：[docs/architecture.md](../../docs/architecture.md) §3。话题/参数契约：[docs/io.md](../../docs/io.md) §3。怎么跑与验收门：[docs/testing.md](../../docs/testing.md)。

```bash
ros2 launch peach_perception scene_perception.launch.py
ros2 launch peach_perception target_reconstruction.launch.py
# 整栈：ros2 launch peach_executor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false
```

离线评估脚本（含 `bag_baseline`）已归档 `_archive/offline_2026-09/`，不随包安装。
