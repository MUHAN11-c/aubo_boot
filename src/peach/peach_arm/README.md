# peach_arm

四个能力包之一：**机械臂执行**。单节点 `peach_arm`（Lifecycle）：拍照、主动视点、质量/安全门、预抓取验证、沿轴套入、刀具 GPIO、原路撤退。不写账本、不调重建 Trigger、不当 `BeginScene` / `RunHarvest` 客户端；仅 Active 允许运动类入口。

现行设计、文件树与「从哪读源码」（先 `stages.cpp` 的 `executeCycle`，接触进 `GraspTask`）：[doc/architecture.md](doc/architecture.md)（系统级：[peach_bringup/doc/architecture.md](../peach_bringup/doc/architecture.md)）。动作/服务/参数契约：[doc/io.md](doc/io.md)（系统级：[peach_bringup/doc/io.md](../peach_bringup/doc/io.md)）。怎么跑与验收门：[v1 测试指导](../peach_system_tests/doc/testing.md)。

```bash
ros2 launch peach_arm peach_arm.launch.py
# 整栈由 harvest_system.launch.py include，autostart 压 false，lifecycle 拉起
# （本 launch 独立跑时 autostart 默认 true；mock 下另须覆盖
#  execution.require_robot_status:=false，否则 Survey 直接 robot_status_missing）
```

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 全关；真运动须与调度 `execution_enabled` 同时开并经人工授权。工具几何权威在 `aubo_description` 工具档案（整栈 `tool_profile` 参数，默认 `bite_shear_v1`）。三档末端统一走 MTC LIN；自适应 IMU 跟随方法已随 `imu_follow` 删除（2026-10-19，历史在 git）。接近先在几何允许直连时验证完整LIN（含必要对轴）；仅在未运动、未取消且未因授权拒绝时回退。树冠外路径是 PTP 关节空间到中段点正下方（树冠外，滚转梯子 {0,±15°,±30°,±45°} IK）+ 世界垂直 LIN 入冠 + 沿轴 LIN 对轴进入预抓取（冠内两段 LIN 优先 Pilz sequence blend 0.02 平滑连续）；已在果下方且直连不穿囊的短修正走直连 LIN（滚转梯子扫刀口）。场景障碍快照避障（部署 `moveit.obstacle_guard_enabled=false`，树冠太密；true 才让相机连杆受查，见 doc/io.md）。节点回调只抽字段委托 `cache_`；周期走 `executeCycle`。测试：`colcon test --packages-select peach_arm`（gtest 纯核套件 + 回放塔语料在 `test/fixtures/`，流程见 v1 测试指导）。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。

接近轨迹优先完整直线，必要时仅使用原有冠内两段LIN之间2cm blend平滑连接；没有新增CIRC或OMPL接触兜底。PTP是关节空间路径，不能声称TCP走精确圆弧。fallback只允许在尚未运动、未取消且未因授权拒绝时继续。
