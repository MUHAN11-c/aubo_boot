# aubo_driver_ros2

AUBO 协作机械臂 ROS2 Humble 驱动包。

## 架构

两套并存，通过启动脚本选择:

| 架构 | 启动 | 轨迹插值 | 关节状态 | SDK 功能 |
|------|------|---------|---------|---------|
| 旧框架 | `start_IVG_...legacy.sh` | Python simulator | 轮询 getCurrentWaypointInfo | 部分 |
| **新框架** | `start_aubo_new_driver.sh` | **C++ 预计算** | **SDK 回调推送** | **Dashboard 20服务** |

### 新框架控制链路

```
MoveIt2 → FollowJointTrajectory Action
  → joint_trajectory_controller
    ├─ handleAccepted: 预计算 200Hz 插值
    ├─ sendLoop: ROS1 风格自适应批量发送
    └─ update: 安全检查 + 目标容差

并行:
  aubo_state_broadcaster → joint_states (RoadPoint+JointStatus 回调)
  aubo_dashboard_node    → 20 ROS2 服务
```

## 可执行文件

| 可执行文件 | 节点名 | 功能 |
|-----------|--------|------|
| `aubo_driver_ros2` | `aubo_driver` | 旧驱动 (兼容) |
| `joint_trajectory_controller` | `joint_trajectory_controller` | FollowJointTrajectory Action + 预计算 + 发送 |
| `aubo_dashboard_node` | `aubo_dashboard` | LifecycleNode, 20 服务 |
| `aubo_state_broadcaster` | `aubo_state_broadcaster` | 回调驱动状态广播 |
| `aubo_callback_monitor` | `aubo_callback_monitor` | 调试: 回调 vs 轮询对比 |

## MoveIt2 集成

启动:
```bash
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
```

控制器配置 (`config/moveit_controllers.yaml`):
```yaml
controller_names: [joint_trajectory_controller]
joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
```

## 话题/服务

| 名称 | 类型 | 方向 | 发布者 |
|------|------|------|--------|
| `/joint_states` | `sensor_msgs/JointState` | 发布 | state_broadcaster |
| `/aubo/feedback_states` | `FollowJointTrajectory_Feedback` | 发布 | state_broadcaster |
| `/robot_status` | `RobotStatus` | 发布 | state_broadcaster |
| `/aubo_driver/rib_status` | `Int32MultiArray` | 发布 | state_broadcaster |
| `/aubo/startup` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/shutdown` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/brake_release` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/stop` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/fast_stop` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/collision_recover` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/move_joint` | `MoveJoint` | 服务 | dashboard |
| `/aubo/move_line` | `MoveLine` | 服务 | dashboard |
| `/aubo/teach_start` | `TeachStart` | 服务 | dashboard |
| `/aubo/teach_stop` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/set_collision_class` | `SetCollisionClass` | 服务 | dashboard |
| `/aubo/set_payload` | `SetPayload` | 服务 | dashboard |
| `/aubo/set_tool_kinematics` | `SetToolKinematics` | 服务 | dashboard |
| `/aubo/set_tool_voltage` | `SetToolVoltage` | 服务 | dashboard |
| `/aubo/set_io` | `SetRobotIO` | 服务 | dashboard |
| `/aubo/get_fk` | `GetFK` | 服务 | dashboard |
| `/aubo/get_ik` | `GetIK` | 服务 | dashboard |
| `/aubo/get_robot_info` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/get_joint_status` | `std_srvs/Trigger` | 服务 | dashboard |
| `/aubo/get_safety_config` | `std_srvs/Trigger` | 服务 | dashboard |
| `/joint_trajectory_controller/follow_joint_trajectory` | `FollowJointTrajectory` | **Action** | controller |

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `server_host` | `169.254.10.98` | 机器人控制器 IP |
| `server_port` | `8899` | 控制器端口 |
| `motion_command_hz` | `200.0` | 插值频率 |
| `collision_class` | `6` | 碰撞等级 |

## 构建

```bash
cd ~/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_driver_ros2
source install/setup.bash
```

## 已知问题与修复 (2026-06-10)

### RIB 过冲 (Fix14)

**现象**：`sendLoop` 在轨迹执行期间 RIB 峰值可达 1158 (容量仅 400)，导致控制器丢点、实际执行轨迹与规划轨迹不一致。

**根因**：`diag_iv=120ms` 的 RIB 查询间隔远大于 `sendLoop` 的发送周期 (~7ms)，本地 `rib` 变量在 120ms 盲区内完全过期，门控 `rib>=300` 失效。

**修复**：`joint_trajectory_controller.cpp:137` — `diag_iv` 从 `120` 改为 `0`，有数据待发时每轮都查 RIB。

### feedback_states 补全

**问题**：`/aubo/feedback_states` 中 `desired.positions` 和 `error.positions` 永远为空。SDK `JointStatusCallback` 同时推送了 `jointTagPosJ`(目标) 和 `jointPosJ`(实际)，但 `AuboStateBroadcaster` 丢弃了目标位置数据。

**修复**：`aubo_state_broadcaster.cpp` — 保存 `fb_tgt_pos_[]`，发布时填充 `desired.positions` + 计算 `error = tgt_pos - pos`。

> 详见 `doc/移植修复记录.md` 第 13 节 Fix14。

> 详见 [MoveIt2集成指南.md](doc/MoveIt2集成指南.md) — MoveIt2 完整接入方案与 JTC 改造计划

## AUBO SDK 版本

当前驱动使用的 SDK 版本及参考 SDK 对比：

| 项目 | 当前使用 | 参考 SDK |
|------|---------|----------|
| **路径** | `lib/lib64/aubocontroller/` | `doc/references/aubo_sdk/` |
| **库名** | `libauborobotcontroller.so.1.3.1` | `libaubo_sdk.so.2.5.3` |
| **SDK 大版本** | v1.x | v2.x（兼容 v1.x 协议） |
| **RobotType** | 8 种（i5~i10S） | 18 种（**超集**：含全部 8 种旧型号，值 0~7 与 v1.x 完全相同） |
| **编译时间** | 旧版 | 2025-12-05 |
| **API 规模** | ~80 个方法 | ~150+ 个方法（**超集**：含所有 v1.x 方法 + 力控/传送带跟踪/焊缝跟踪等） |
| **验证状态** | ✅ 真机验证通过 | ⚠️ 协议层已验证兼容，未在本机机械臂上实测 |

### 通信协议兼容性验证（确定结论）

通过对比两个 SDK 的导出符号（`nm -D`），确认以下事实：

| 对比项 | v1.3.1 (libauborobotcontroller) | v2.5.3 (libaubo_sdk) | 结论 |
|--------|------|------|------|
| **ProtoRequestLogin** | `aubo::robot::communication::ProtoRequestLogin` | `aubo::robot::communication::ProtoRequestLogin` | ✅ 完全相同 |
| **RobotCommunicationClient** | 相同方法签名、相同 buffer 常量 | 相同方法签名、相同 buffer 常量 | ✅ 逐符号一致 |
| **TCP 帧格式** | 4B SOF + 4B LEN + 4B CRC + 4B END | 4B SOF + 4B LEN + 4B CRC + 4B END | ✅ 完全相同 |
| **RobotType 枚举值** | 0=I5, 1=I7, 2=I10_12, 3=I3S, 4=I3, 5=I5S, 6=I5L, 7=I10S | **0~7 值与 v1.x 完全相同**，8~20 为新增 | ✅ 旧型号值保持不变 |
| **Protobuf 命名空间** | `aubo::robot::communication` + `aubo::robot::common` | `aubo::robot::communication` + `aubo::robot::common` | ✅ 完全相同 |

> **确定结论**：v2.5.3 与 v1.3.1 的**通信协议层 100% 兼容**。两个 SDK 发送完全相同的 protobuf 消息（`ProtoRequestLogin`/`JointVersion`/`ProtoJointStatus` 等）通过完全相同的 TCP 帧格式，机械臂控制器**无法区分**客户端使用的是哪个 SDK 版本。v2.5.3 可以连接旧机械臂，不存在协议被拒的风险。

### 为什么不替换

虽然协议兼容，但**不建议替换**，原因如下：

1. **零收益**：当前驱动只使用了 `ServiceInterface` 的基础运动 API（`robotServiceLogin`、`robotServiceJointMove`、`robotServiceRobotFk` 等），这些 API 在两个 SDK 中完全一致。v2.5.3 新增的力控/传送带跟踪/焊缝跟踪等功能，当前驱动代码完全不使用
2. **改造成本高**：
   - 库名不同：`-lauborobotcontroller` → `-laubo_sdk`（CMakeLists.txt 需改）
   - 头文件路径不同：v2.5.3 有 30 个头文件，当前驱动只 include 了 3 个
   - 头文件命名空间可能有细微差异（v1.3.1 的 `AuboRobotMetaType.h` 用了 `extern "C"` 包裹，v2.5.3 没有）
3. **编译风险**：libaubo_sdk.so.2.5.3 链接 protobuf 3.6.1，当前驱动环境是 protobuf 9.0.1，可能存在 protobuf ABI 不兼容
4. **维护成本**：v1.3.1 已验证稳定，换成 v2.5.3 后如果出现问题需要重新排查

> **总结**：参考 SDK v2.5.3 适合作为 API 参考文档查阅（了解 AUBO SDK 的完整能力面），不适合作为本机旧机械臂驱动的 SDK 替换。如果真的想用新 SDK 的新功能（力控等），需要在备用机械臂上完成连通性验证后再做迁移喵~

## 依赖

- ROS 2: rclcpp, sensor_msgs, std_msgs, std_srvs, geometry_msgs, tf2_ros
- ivg_interfaces (GetFK, GetIK, SetRobotIO, MoveJoint, MoveLine, etc.)
- AUBO SDK: `libauborobotcontroller.so` v1.3.1（预编译，位于 `lib/lib64/aubocontroller/`）
- lifecycle_msgs (AuboDashboardNode LifecycleNode)

## 文档

- `doc/架构设计.md` — 架构 + 新旧对比 + 实测数据
- `doc/AUBO驱动参考手册.md` — SDK 完整参考
- `doc/SDK冲突规则.md` — SDK 冲突规则
- `doc/移植修复记录.md` — ROS1→ROS2 移植记录
- `doc/references/aubo_sdk/` — AUBO SDK v2.5.3 参考（升级候选，**不可直接替换当前驱动**）
