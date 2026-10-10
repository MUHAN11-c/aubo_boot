# peach_arm

四个能力包之一：**机械臂执行**。单节点 `peach_arm`（Lifecycle）：拍照、主动视点、质量/安全门、预抓取验证、沿轴套入、刀具 GPIO、原路撤退。不写账本、不调重建 Trigger、不当 `BeginScene` / `RunHarvest` 客户端；仅 Active 允许运动类入口。

现行设计、文件树与「从哪读源码」（先 `stages.cpp` 的 `executeCycle`，接触进 `GraspTask`）：[doc/architecture.md](doc/architecture.md)（系统级：[peach_bringup/doc](../peach_bringup/doc/architecture.md)）。动作/服务/参数契约：[doc/io.md](doc/io.md)（系统级：[peach_bringup/doc/io](../peach_bringup/doc/io.md)）。怎么跑与验收门：[v1 测试指导](../peach_system_tests/doc/testing.md)。

```bash
ros2 launch peach_arm peach_arm.launch.py
# 整栈由 harvest_system.launch.py include，autostart:=false，lifecycle 拉起
```

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 全关；真运动须与调度 `execution_enabled` 同时开并经人工授权。工具几何权威在 `aubo_description` 工具档案（整栈 `tool_profile` 参数，默认 `adaptive_shear_v1`）。自适应 FULL 在预抓取→套入→回预抓取窗内调 imu_follow；非 IMU 末端走 MTC LIN 且不建 `/imu_follow` 客户端。接近主路径（v4）是 PTP 关节空间到中段点正下方（树冠外，滚转梯子 {0,±30°,±60°} IK）+ 世界垂直 LIN 入冠 + 沿轴 LIN 对轴进入预抓取（冠内两段 LIN 优先 Pilz sequence blend 0.02 平滑连续）；已在果下方且直连不穿囊的短修正走直连 LIN（滚转梯子扫刀口）。节点回调只抽字段委托 `cache_`；周期走 `executeCycle`。
