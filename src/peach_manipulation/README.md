# peach_manipulation

四个能力包之一：**机械臂执行**。单节点 `peach_manipulation_node`（Lifecycle）：拍照、主动视点、质量/安全门、预抓取验证、沿轴套入、刀具 GPIO、原路撤退。不写账本、不调重建 Trigger、不当 `BeginScene` / `RunHarvest` 客户端；仅 Active 允许运动类入口。

现行设计、文件树与「从哪读源码」（先 `stages.cpp` 的 `executeCycle`，接触进 `GraspTask`）：[docs/architecture.md](../../docs/architecture.md) §3。动作/服务/参数契约：[docs/io.md](../../docs/io.md) §4。怎么跑与验收门：[docs/testing.md](../../docs/testing.md)。

```bash
ros2 launch peach_manipulation peach_manipulation.launch.py
# 整栈由 harvest_system.launch.py include，autostart:=false，lifecycle 拉起
```

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 全关；真运动须与调度 `execution_enabled` 同时开并经人工授权。工具几何权威在 `aubo_description`（`hollow_cylinder_v1`）。
