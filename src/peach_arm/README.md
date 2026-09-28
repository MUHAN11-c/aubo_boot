# peach_arm

四个能力包之一：**机械臂执行**。单节点 `peach_arm`（Lifecycle）：拍照、主动视点、质量/安全门、预抓取验证、沿轴套入、刀具 GPIO、原路撤退。不写账本、不调重建 Trigger、不当 `BeginScene` / `RunHarvest` 客户端；仅 Active 允许运动类入口。

现行设计、文件树与「从哪读源码」（先 `stages.cpp` 的 `executeCycle`，接触进 `GraspTask`）：[docs/architecture.md](../../docs/architecture.md) §3。动作/服务/参数契约：[docs/io.md](../../docs/io.md) §4。怎么跑与验收门：[docs/testing.md](../../docs/testing.md)。

```bash
ros2 launch peach_arm peach_arm.launch.py
# 整栈由 harvest_system.launch.py include，autostart:=false，lifecycle 拉起
```

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 全关；真运动须与调度 `execution_enabled` 同时开并经人工授权。工具几何权威在 `aubo_description` 工具档案（整栈 `tool_profile` 参数，默认 `adaptive_shear_v1`）。自适应 FULL 在预抓取→套入→回预抓取窗内调 imu_follow；非 IMU 末端走 MTC LIN 且不建 `/imu_follow` 客户端。接近主路径是果平面折线 LIN（面内斜插到轴上 staging + 沿轴垂直进入；斜插 keep-roll 对轴，不叠 ±30/±60 刀口滚转；斜插 0.20/0.10，wrist1 超限再 0.10/0.04，两档失败才 PTP staging）。节点回调只抽字段委托 `cache_`；周期走 `executeCycle`。
