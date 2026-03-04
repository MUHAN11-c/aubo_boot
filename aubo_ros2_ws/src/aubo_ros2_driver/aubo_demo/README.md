# demo_driver 调用示例（C++版本）

本目录包含了 `demo_driver` 包中所有服务和话题的调用示例代码（C++实现）。

## 目录结构

### 服务调用示例（客户端）

| 文件名 | 服务名称 | 功能描述 |
|--------|---------|---------|
| `move_to_pose_client.py` | `/move_to_pose` | 移动到目标位姿 |
| `plan_trajectory_client.py` | `/plan_trajectory` | 规划轨迹 |
| `execute_trajectory_client.py` | `/execute_trajectory` | 执行轨迹 |
| `get_current_state_client.py` | `/get_current_state` | 获取当前状态 |
| `set_speed_factor_client.py` | `/set_speed_factor` | 设置速度因子 |
| `set_robot_enable_client.py` | `/set_robot_enable` | 设置机器人使能状态 |
| `set_robot_io_client.py` | `/set_robot_io` | 设置机器人IO |
| `read_robot_io_client.py` | `/read_robot_io` | 读取机器人IO |

### 话题订阅示例（订阅者）

| 文件名 | 话题名称 | 功能描述 |
|--------|---------|---------|
| `robot_status_subscriber.py` | `/robot_status` | 订阅机器人状态 |
| `robot_io_status_subscriber.py` | `/robot_io_status` | 订阅机器人IO状态 |

### 综合示例

| 文件名 | 功能描述 |
|--------|---------|
| `comprehensive_example.py` | 完整工作流程演示，包含多个服务的组合使用 |

## 使用方法

### 1. 编译工作空间

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build
source install/setup.bash
```

### 2. 启动服务节点

在运行示例之前，需要先启动相应的服务节点。可以使用 launch 文件启动：

```bash
# 启动所有服务节点（推荐）
ros2 launch demo_driver demo_driver.launch.py

# 或者单独启动某个服务节点
ros2 launch demo_driver move_to_pose_server.launch
ros2 launch demo_driver robot_status_publisher.launch
# ... 等等
```

### 3. 运行示例

#### 服务调用示例

```bash
# 移动到目标位姿
ros2 run aubo_demo move_to_pose_client

# 规划轨迹
ros2 run aubo_demo plan_trajectory_client

# 执行轨迹
ros2 run aubo_demo execute_trajectory_client

# 获取当前状态
ros2 run aubo_demo get_current_state_client

# 设置速度因子
ros2 run aubo_demo set_speed_factor_client

# 设置机器人使能
ros2 run aubo_demo set_robot_enable_client

# 设置机器人IO
ros2 run aubo_demo set_robot_io_client

# 读取机器人IO
ros2 run aubo_demo read_robot_io_client
```

#### 话题订阅示例

```bash
# 订阅机器人状态
ros2 run aubo_demo robot_status_subscriber

# 订阅机器人IO状态
ros2 run aubo_demo robot_io_status_subscriber
```

#### 综合示例

```bash
# 运行完整工作流程示例
ros2 run aubo_demo comprehensive_example
```

## 服务接口说明

### MoveToPose

移动到目标位姿服务。

**请求参数：**
- `target_pose` (geometry_msgs/Pose): 目标位姿
- `use_joints` (bool): 是否使用关节空间规划
- `velocity_factor` (float32): 速度缩放因子 (0.0-1.0)
- `acceleration_factor` (float32): 加速度缩放因子 (0.0-1.0)

**响应：**
- `success` (bool): 是否成功
- `error_code` (int32): 错误码
- `message` (string): 消息

### PlanTrajectory

规划轨迹服务。

**请求参数：**
- `target_pose` (geometry_msgs/Pose): 目标位姿
- `use_joints` (bool): 是否使用关节空间规划

**响应：**
- `success` (bool): 是否成功
- `trajectory` (trajectory_msgs/JointTrajectory): 规划的轨迹
- `planning_time` (float32): 规划时间（秒）
- `message` (string): 消息

### ExecuteTrajectory

执行轨迹服务。

**请求参数：**
- `trajectory` (trajectory_msgs/JointTrajectory): 要执行的轨迹

**响应：**
- `success` (bool): 是否成功
- `error_code` (int32): 错误码
- `message` (string): 消息

### GetCurrentState

获取当前状态服务。

**请求参数：** 无

**响应：**
- `success` (bool): 是否成功
- `joint_position_rad` (float64[]): 关节位置（弧度）
- `cartesian_position` (geometry_msgs/Pose): 笛卡尔位姿
- `velocity` (float64[]): 关节速度
- `message` (string): 消息

### SetSpeedFactor

设置速度因子服务。

**请求参数：**
- `velocity_factor` (float32): 速度缩放因子 (0.0-1.0)

**响应：**
- `success` (bool): 是否成功
- `message` (string): 消息

### SetRobotEnable

设置机器人使能状态服务。

**请求参数：**
- `enable` (bool): True=使能，False=禁用

**响应：**
- `success` (bool): 是否成功
- `error_code` (int32): 错误码
- `message` (string): 消息

### SetRobotIO

设置机器人IO服务。

**请求参数：**
- `io_type` (string): IO类型 ("digital_output", "analog_output", "tool_io", "tool_analog_output")
- `io_index` (int32): IO索引
- `value` (float64): IO值

**响应：**
- `success` (bool): 是否成功
- `error_code` (int32): 错误码
- `message` (string): 消息

### ReadRobotIO

读取机器人IO服务。

**请求参数：**
- `io_type` (string): IO类型 ("digital_input", "digital_output", "analog_input", "analog_output", "tool_io")
- `io_index` (int32): IO索引

**响应：**
- `success` (bool): 是否成功
- `value` (float64): IO值
- `message` (string): 消息

## 话题说明

### /robot_status

机器人状态话题，发布频率默认 10 Hz。

**消息类型：** `demo_interface/RobotStatus`

**字段：**
- `header` (std_msgs/Header): 消息头
- `is_online` (bool): 在线状态
- `enable` (bool): 使能状态
- `in_motion` (bool): 运动状态
- `planning_status` (string): 规划状态
- `joint_position_rad` (float64[6]): 关节位置（弧度）
- `joint_position_deg` (float64[6]): 关节位置（度）
- `cartesian_position` (geometry_msgs/Pose): 笛卡尔位姿

### /robot_io_status

机器人IO状态话题。

**消息类型：** `demo_interface/RobotIOStatus`

**字段：**
- `header` (std_msgs/Header): 消息头
- `digital_inputs` (bool[]): 数字输入状态
- `digital_outputs` (bool[]): 数字输出状态
- `analog_inputs` (float32[]): 模拟输入状态
- `analog_outputs` (float32[]): 模拟输出状态
- `tool_io_status` (ToolIOStatus): 工具IO状态
- `is_connected` (bool): 连接状态

## 注意事项

1. 在运行示例之前，确保已启动相应的服务节点。
2. 某些服务需要 MoveIt 和机器人驱动正在运行。
3. IO操作需要确保机器人驱动已启动并连接。
4. 速度和加速度因子应在 0.0-1.0 范围内。
5. 位姿的坐标单位是米（m），姿态使用四元数表示。

## 许可证

BSD

