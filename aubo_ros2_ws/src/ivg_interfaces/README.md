# ivg_interfaces — IVG 统一 ROS 2 接口包

IVG 项目所有自定义消息 (msg) 和服务 (srv) 统一定义于此。**17 msg + 35 srv = 52 接口类型**。

## 设计原则

- **单一真相源**: 所有包间通信统一走此包的接口定义，废弃各包独立接口
- **旧包已 COLCON_IGNORE**: `aubo_msgs`, `demo_interface`, `tool_changer_interface` 等已弃用
- **命名**: 全小写下划线分隔 (snake_case)，服务名为动词+名词

## 消息 (17 msg)

### 机器人状态
| 消息 | 用途 | 发布者 | 频率 |
|------|------|--------|------|
| `RobotStatus` | 机械臂末端位姿+RPY+运行模式 | `aubo_state_broadcaster` | ~50Hz |
| `RobotStateRTMsg` | SDK 实时数据 (UR TCP 兼容) | `aubo_state_broadcaster` | ~125Hz |
| `RobotModeDataMsg` | 机器人模式数据 | `aubo_state_broadcaster` | ~10Hz |
| `RobotIOStatus` | 机器人 IO 状态 | `aubo_state_broadcaster` | ~10Hz |
| `ToolIOStatus` | 工具端 IO 状态 | `aubo_state_broadcaster` | ~10Hz |
| `JointTrajectoryFeedback` | 轨迹执行反馈 | `joint_trajectory_controller` | 200Hz |

### 工具与传感器
| 消息 | 用途 | 发布者 | 频率 |
|------|------|--------|------|
| `ToolChangerStatus` | 快换工具状态 (tool_id/名称/类型/连接) | `gripper_swap_worker` | 0.2Hz (每5秒) |
| `ToolDataMsg` | 工具数据 | `aubo_state_broadcaster` | ~10Hz |
| `CameraStatus` | 相机状态 | `percipio_camera` 节点 | 事件驱动 |
| `ImageData` | 图像数据 (自定义格式) | `camport_ros2` | ~30Hz |

### 运动与控制
| 消息 | 用途 | 发布者 | 频率 |
|------|------|--------|------|
| `CartesianPosition` | 笛卡尔位姿 (VPE 推理输出) | `visual_pose_estimation` | ~5-10Hz |
| `IOStates` | 聚合 IO 状态 | `aubo_state_broadcaster` | ~10Hz |
| `Analog` | 模拟量 | `aubo_state_broadcaster` | ~10Hz |
| `Digital` | 数字量 | `aubo_state_broadcaster` | ~10Hz |
| `MasterboardDataMsg` | 主板数据 | `aubo_state_broadcaster` | ~10Hz |

### 系统
| 消息 | 用途 | 发布者 | 频率 |
|------|------|--------|------|
| `NodeStatus` | 节点运行状态 | `system_monitor_node` | ~5Hz |
| `SystemLog` | 系统日志 | 多个节点 | 事件驱动 |

## 服务 (35 srv)

### AUBO 仪表板 (20 srv)
| 服务 | 提供者 | 用途 |
|------|--------|------|
| `/aubo/startup` | `aubo_dashboard_node` | 上电+松刹车+碰撞等级 |
| `/aubo/shutdown` | `aubo_dashboard_node` | 下电 |
| `/aubo/brake_release` | `aubo_dashboard_node` | 松刹车 |
| `/aubo/stop` | `aubo_dashboard_node` | 停止运动 |
| `/aubo/fast_stop` | `aubo_dashboard_node` | 急停 |
| `/aubo/collision_recover` | `aubo_dashboard_node` | 碰撞恢复 |
| `/aubo/move_joint` | `aubo_dashboard_node` | 关节空间运动 |
| `/aubo/move_line` | `aubo_dashboard_node` | 笛卡尔直线运动 |
| `/aubo/teach_start` | `aubo_dashboard_node` | 开启力控示教 |
| `/aubo/teach_stop` | `aubo_dashboard_node` | 停止力控示教 |
| `/aubo/set_collision_class` | `aubo_dashboard_node` | 碰撞灵敏度等级 |
| `/aubo/set_payload` | `aubo_dashboard_node` | 设置末端负载 |
| `/aubo/set_tool_kinematics` | `aubo_dashboard_node` | TCP 偏移设置 |
| `/aubo/set_tool_voltage` | `aubo_dashboard_node` | 工具电压 (0V/12V/24V) |
| `/aubo/get_fk` | `aubo_dashboard_node` | 正运动学 |
| `/aubo/get_ik` | `aubo_dashboard_node` | 逆运动学 |
| `/aubo/get_robot_info` | `aubo_dashboard_node` | 机器人信息 |
| `/aubo/get_joint_status` | `aubo_dashboard_node` | 关节状态 |
| `/aubo/get_safety_config` | `aubo_dashboard_node` | 安全配置 |
| `/aubo/set_io` | `aubo_dashboard_node` | 通用 IO 设置 |

### 应用层服务 (15 srv)
| 服务 | 提供者 | 用途 |
|------|--------|------|
| `ReplayLatteTrajectory` | `latte_imitation_node` | 拉花轨迹回放 (preview/debug/action) |
| `RunGripperSwap` | `gripper_swap_worker` | 夹爪快换 |
| `ChangeTool` | `gripper_swap_worker` + `scene_attach_worker` | 换刀+场景附着 |
| `GetCurrentTool` | `gripper_swap_worker` | 查询当前工具 |
| `MoveToPose` | `move_to_pose_server` | 笛卡尔位姿运动 |
| `ExecuteGraspPose` | `execute_grasp_pose_worker` | 执行单次抓取 |
| `PlanTrajectory` | `plan_trajectory_server` | MoveIt2 轨迹规划 |
| `ExecuteTrajectory` | `execute_trajectory_server` | MoveIt2 轨迹执行 |
| `GetCurrentState` | `get_current_state_server` | 获取当前机器人状态 |
| `SetRobotPose` | `set_robot_pose_server` | 设置机器人位姿 |
| `SetRobotEnable` | `set_robot_enable_server` | 使能/禁用 |
| `SetSpeedFactor` | `set_speed_factor_server` | 速度因子 |
| `SetCollisionClass` | `aubo_dashboard_node` | 碰撞等级 |
| `ReadRobotIO` | `read_robot_io_server` | 读取 IO 状态 |
| `SetRobotIO` | `aubo_dashboard_node` | 设置 IO |

### 视觉/VPE 服务 (7 srv)
| 服务 | 提供者 | 用途 |
|------|--------|------|
| `EstimatePose` | VPE 节点 | 6-DoF 姿态估计 |
| `EstimatePose2D` | VPE 节点 | 2D 姿态估计 |
| `StandardizeTemplate` | VPE 节点 | 模板标准化 |
| `ListTemplates` | VPE 节点 | 列出模板 |
| `UpdateParams` | VPE 节点 | 更新 VPE 参数 |
| `VisualizeGraspPose` | VPE 节点 | 可视化抓取姿态 |
| `ProcessDebugStep` | VPE 节点 | 调试步骤 |

### 相机 (3 srv)
| 服务 | 提供者 | 用途 |
|------|--------|------|
| `SetCameraParameters` | `percipio_camera` | 设置相机参数 |
| `SoftwareTrigger` | `percipio_camera` | 软件触发 |
| `SetIO` | `aubo_dashboard_node` | IO (含 tool_power) |

### 已废弃 (标注保留)
| 服务 | 状态 | 说明 |
|------|------|------|
| `Movel` | @deprecated | 无 .cpp 实现, 仅保留接口定义供向前兼容 |
| `TeachStart` | @deprecated | 功能并入 `/aubo/teach_start` |
| `SetPayload` | @deprecated | 功能并入 `/aubo/set_payload` |
| `SetToolKinematics` | @deprecated | 功能并入 `/aubo/set_tool_kinematics` |
| `SetToolVoltage` | @deprecated | 功能并入 `/aubo/set_tool_voltage` |
| `SetSpeedSliderFraction` | @deprecated | 功能并入 `/set_speed_factor` |

## 依赖

- `rosidl_default_generators` (构建时)
- `geometry_msgs`, `std_msgs`, `sensor_msgs`, `std_srvs`, `trajectory_msgs`
- `moveit_msgs`, `control_msgs`
- `rcl_interfaces` (参数服务类型)

## 使用

```cmake
# C++
find_package(ivg_interfaces REQUIRED)
#include <ivg_interfaces/msg/robot_status.hpp>
#include <ivg_interfaces/srv/replay_latte_trajectory.hpp>
```

```python
# Python
from ivg_interfaces.msg import RobotStatus
from ivg_interfaces.srv import ReplayLatteTrajectory
```

```typescript
// TypeScript (rosbridge)
const msgType = 'ivg_interfaces/msg/RobotStatus'
const srvType = 'ivg_interfaces/srv/ReplayLatteTrajectory'
```
