# peach_arm

四个能力包之一：**机械臂执行**。单节点 `peach_arm`（Lifecycle）：拍照、主动视点、质量/安全门、预抓取验证、沿轴套入、刀具 GPIO、原路撤退。不写账本、不调重建 Trigger、不当 `BeginScene` / `RunHarvest` 客户端；仅 Active 允许运动类入口。

现行设计、文件树与「从哪读源码」（先 `stages.cpp` 的 `executeCycle`，接触进 `GraspTask`）：[doc/architecture.md](doc/architecture.md)（系统级：[peach_bringup/doc/architecture.md](../peach_bringup/doc/architecture.md)）。动作/服务/参数契约：[doc/io.md](doc/io.md)（系统级：[peach_bringup/doc/io.md](../peach_bringup/doc/io.md)）。怎么跑与验收门：[v1 测试指导](../peach_system_tests/doc/testing.md)。

```bash
ros2 launch peach_arm peach_arm.launch.py
# 整栈由 harvest_system.launch.py include，autostart 压 false，lifecycle 拉起
# （本 launch 独立跑时 autostart 默认 true；mock 下另须覆盖
#  execution.require_robot_status:=false，否则 Survey 直接 robot_status_missing）
```

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 全关；真运动须与调度 `execution_enabled` 同时开并经人工授权。工具几何权威在 `aubo_description` 工具档案（整栈 `tool_profile` 参数，默认 `bite_shear_v1`）。三档末端统一走 MTC LIN；自适应 IMU 跟随方法已随 `imu_follow` 删除（2026-10-19，历史在 git）。接近仅两段：直接转移至轨迹净空点 A，再单条轴向套入到 F。令 E 为原入口、u 为单位套入轴、S 为共享视觉后撤、C 为 `moveit.approach_clearance_m`、L 为原切断行程，则视觉预抓取 N=E−uS 保持不变，A=N−uC，F=E+uL；套入/撤退名义长度为 S+C+L。N 只在轴向路径上经过，不另停。C 默认 0.05 m，有限且在 [0,0.20]；C=0 回到视觉停位。远处用受护栏约束的直接 PTP，已对轴且几何允许时可单条 LIN；无解明确拒绝，没有中间走廊或三段回退。只有尚未运动、未取消且未因授权拒绝时才继续候选搜索。场景障碍快照避障（部署 `moveit.obstacle_guard_enabled=false`，树冠太密；true 才让相机连杆受查，见 doc/io.md）。节点回调只抽字段委托 `cache_`；周期走 `executeCycle`。测试：`colcon test --packages-select peach_arm`（仅官方 ament lint；行为核对使用日志与测试指导的手工步骤）。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。
