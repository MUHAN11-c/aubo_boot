# aubo_ros2_driver 功能包集合

AUBO 机械臂在 ROS2 Humble 下的驱动、MoveIt 配置、轨迹执行与仿真（全 ROS2 栈）。

## 子包一览

| 子包 | 职责 |
|------|------|
| **aubo_driver_ros2** | 新框架 C++ 驱动：JointTrajectoryController + AuboDashboardNode(20 服务) + AuboStateBroadcaster |
| **aubo_moveit_config** | MoveIt2 配置与统一 launch (`aubo_new_driver.launch.py`)：自动 TCP 探测 → 真机/仿真 |
| **demo_driver** | 应用层 C++ 服务节点：move_to_pose / plan_trajectory / get_current_state / set_speed_factor 等 |
| **aubo_description** | 机器人模型 (URDF/STL mesh)，含 5 种末端工具 |

> 旧架构 (`aubo_ros2_trajectory_action`、`aubo_robot_simulator_ros2`、`demo_interface`、`aubo_msgs`) 已 `COLCON_IGNORE` 废弃。接口统一为 `ivg_interfaces` 喵~

## 新框架架构

```
MoveIt2 → FollowJointTrajectory Action
  → joint_trajectory_controller (C++ 预计算 200Hz + sendLoop RIB 流控)
  → AuboHardwareInterface → SDK → 真实机械臂

并行:
  aubo_state_broadcaster (回调+RIB 50Hz) → /joint_states, /robot_status
  aubo_dashboard_node (LifecycleNode, 仅真机) → 20 ROS2 服务
```

真机模式使用 AUBO 自定义驱动；仿真模式自动回退 `ros2_control` + `mock_components/GenericSystem`。

## 末端工具

| 工具 ID | 类型 | 所属包 | 用途 |
|---------|------|--------|------|
| **gripper0** | 气动夹爪 φ40 | tool_changer | 工件抓取，通过快换对接 |
| **gripper1** | 电动夹爪 A | latte_imitation | 咖啡拉花载体，不参与快换 |
| **gripper2** | 电动夹爪 φ60 | tool_changer | AI 抓取，通过快换对接 |

### IO 引脚分配

所有 IO 通过 `/aubo_driver/set_io`（`io_type=digital_output`）：

| 逻辑引脚 | 硬件引脚 | 用途 |
|---------|---------|------|
| 6 | 38 | 夹爪开/关 (⚠️ 语义因工位而异) |
| 7 | 39 | 快换盘锁紧/释放 |
| 2 | 34 | 打花开关 (DO2) |
| 4 | 36 | 咖啡开关 (DO4) |

## 编译

```bash
cd ~/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_driver_ros2 aubo_moveit_config demo_driver aubo_description
source install/setup.bash
```

AUBO SDK 预编译库 (`libauborobotcontroller.so`) 在 `colcon build` 时自动安装到 `install/aubo_driver_ros2/lib/` 喵~

## 运行

```bash
# 一键启动全栈 (自动检测真机/仿真)
./start_aubo_new_driver.sh

# 或手动启动
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98

# 跳过编译/录包/RViz
SKIP_BUILD=1 SKIP_ROSBAG=1 SKIP_RVIZ=1 ./start_aubo_new_driver.sh
```

## 文档

| 文档 | 说明 |
|------|------|
| `aubo_driver_ros2/README.md` | 驱动包节点/服务/参数 |
| `aubo_moveit_config/README.md` | MoveIt 配置概览 |
| `aubo_moveit_config/doc/` | 故障排除 / ros2_control 集成 / RViz 配置 / 工作空间限制 |
| `demo_driver/README.md` | 应用层服务节点文档 |
| `aubo_driver_ros2/doc/` | 驱动架构 / SDK 参考 / 移植记录 / SDK 冲突规则 |

## 参考

- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [MoveIt 2 Humble 文档](https://moveit.picknik.ai/humble/index.html)
- [ros2_control 文档](https://control.ros.org/humble/index.html)
- CLAUDE.md §1 AUBO SDK 双连接架构, §2 JointTrajectoryController, §13 仿真模式
