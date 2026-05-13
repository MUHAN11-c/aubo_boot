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
| `/aubo_driver/robot_status` | `RobotStatus` | 发布 | state_broadcaster |
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

## 自定义 srv (demo_interface)

| srv | 请求 |
|-----|------|
| `MoveJoint.srv` | `float64[6] joints, float64 velocity` |
| `MoveLine.srv` | `float64[6] joints, float64 velocity` |
| `TeachStart.srv` | `int32 joint, bool direction` |
| `SetCollisionClass.srv` | `int32 grade` |
| `SetToolKinematics.srv` | `float64[3] position, float64[4] orientation` |
| `SetToolVoltage.srv` | `int32 voltage_type` |

## 文档

- `doc/ARCHITECTURE.md` — 架构 + 新旧对比 + 实测数据
- `doc/AUBO_ROS2_DRIVER_REFERENCE.md` — SDK 完整参考
- `doc/SDK_CONFLICT_RULES.md` — SDK 冲突规则
- `doc/PORTING_MOTION_FIX.md` — ROS1→ROS2 移植记录
