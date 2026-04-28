# tool_changer — 快换与手爪管理

gripper0 ↔ gripper2 双向自动快换，工具状态发布与查询。

## 节点

### gripper_swap_worker_node

| 属性 | 值 |
|------|-----|
| 功能 | 夹爪快换 Worker：gripper0 ↔ gripper2 双向自动快换 |
| 依赖 | tool_changer_interface, demo_interface (SetRobotIO), moveit |
| 多线程 | MultiThreadedExecutor(2)：回调线程执行快换，spin 线程处理 MoveIt/IO 响应 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/run_gripper_swap` | tool_changer_interface/srv/RunGripperSwap | 执行快换，direction: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| `/change_tool` | tool_changer_interface/srv/ChangeTool | 按 tool_id 切换工具 |
| `/get_current_tool` | tool_changer_interface/srv/GetCurrentTool | 查询当前工具状态 |

### 话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/tool_changer_status` | tool_changer_interface/msg/ToolChangerStatus | 快换盘状态（工具 ID / 名称 / 类型 / 连接状态） |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_velocity_scaling` | 0.7 | 关节速度缩放 |
| `joint_acceleration_scaling` | 0.3 | 关节加速度缩放 |
| `home_velocity_scaling` | 0.7 | 回安全位速度缩放 |
| `home_acceleration_scaling` | 0.3 | 回安全位加速度缩放 |
| `gripper_io_index` | 7 | 快换 IO pin 号 |
| `joint_cartesian_switch_delay_sec` | 0.05 | 关节↔笛卡尔切换延时 |

## 运行

```bash
# 需要先启动机械臂驱动（move_group + aubo_driver_ros2 等）
ros2 run tool_changer gripper_swap_worker_node
```

### 工具定义

| 工具 ID | 类型 | 描述 |
|---------|------|------|
| **gripper0** | 气动夹爪 φ40 | 行程 40mm, 夹持力 120N, 重量 0.8kg（视觉工件抓取） |
| **gripper2** | 电动夹爪 φ60 | 行程 60mm, 夹持力 250N, 重量 1.5kg（AI 抓取） |
