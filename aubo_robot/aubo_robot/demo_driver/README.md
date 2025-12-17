# demo_driver

机器人驱动功能包，提供机器人状态发布、运动控制和IO控制等功能。

## 依赖

- `demo_interface`: 消息和服务定义包
- `aubo_msgs`: Aubo机器人消息包
- `industrial_msgs`: 工业机器人消息包
- `sensor_msgs`: 传感器消息包
- `geometry_msgs`: 几何消息包
- `trajectory_msgs`: 轨迹消息包
- `moveit_core`: MoveIt!核心库
- `moveit_ros_planning_interface`: MoveIt!规划接口

## 节点说明

### 1. robot_status_publisher_node

**功能**：发布机器人完整状态信息，包括在线状态、使能状态、运动状态、规划状态、关节位置和笛卡尔位姿。

#### 发布的话题

- `/demo_robot_status` (demo_interface/RobotStatus) - 默认话题名称，可通过参数 `robot_status_topic` 配置
  - **header** (std_msgs/Header): 标准消息头（时间戳和帧ID）
  - **is_online** (bool): 机器人与上位机通信状态（心跳），在线为true
  - **enable** (bool): 机器人使能状态，已使能为true（独占控制）
  - **in_motion** (bool): 机器人运动状态，运动中为true
  - **planning_status** (string): MoveIt规划状态（idle/planning/executing/error）
  - **joint_position_rad** (float64[6]): 6个关节的当前角度值（弧度）
  - **joint_position_deg** (float64[6]): 6个关节的当前角度值（度）
  - **cartesian_position** (geometry_msgs/Pose): 当前工具中心点（TCP）的笛卡尔位姿（位置单位：m，姿态：四元数）

**注意**：此节点发布到 `/demo_robot_status` 话题以避免与 `aubo_driver` 发布的 `industrial_msgs/RobotStatus` 消息（话题 `/robot_status`）冲突。

#### 订阅的话题

- `joint_states` (sensor_msgs/JointState): 关节状态
- `robot_status` (industrial_msgs/RobotStatus): 工业机器人状态
- `trajectory_execution_event` (std_msgs/String): 轨迹执行事件

#### 使用的服务

- `/aubo_driver/get_fk` (aubo_msgs/GetFK): 正运动学计算服务

#### 参数

- `publish_rate` (double, default: 10.0): 发布频率（Hz）
- `base_frame` (string, default: "base_link"): 基础坐标系
- `planning_group_name` (string, default: "manipulator_e5"): 规划组名称
- `robot_status_topic` (string, default: "/demo_robot_status"): 发布机器人状态的话题名称

---

### 2. robot_io_status_publisher_node

**功能**：发布机器人IO状态信息，包括数字输入/输出、模拟输入/输出、工具IO状态和连接状态。

#### 发布的话题

- `/robot_io_status` (demo_interface/RobotIOStatus)
  - **header** (std_msgs/Header): 标准消息头（时间戳和帧ID）
  - **digital_inputs** (bool[]): 数字输入点状态数组
  - **digital_outputs** (bool[]): 数字输出点状态数组
  - **analog_inputs** (float32[]): 模拟输入点状态数组
  - **analog_outputs** (float32[]): 模拟输出点状态数组
  - **tool_io_status** (demo_interface/ToolIOStatus): 工具IO状态
    - **digital_inputs** (bool[]): 工具数字输入点状态数组
    - **digital_outputs** (bool[]): 工具数字输出点状态数组
    - **analog_inputs** (float32[]): 工具模拟输入点状态数组
    - **analog_outputs** (float32[]): 工具模拟输出点状态数组
  - **is_connected** (bool): IO接口连接状态

#### 订阅的话题

- `/aubo_driver/io_states` (aubo_msgs/IOState): aubo驱动器的IO状态

#### 参数

- `io_states_topic` (string, default: "/aubo_driver/io_states"): IO状态话题名称
- `base_frame` (string, default: "base_link"): 基础坐标系

---

### 3. move_to_pose_server_node

**功能**：提供移动到目标位姿服务，支持关节空间和笛卡尔空间规划，可设置速度和加速度缩放因子。

#### 提供的服务

- `/move_to_pose` (demo_interface/MoveToPose)
  - **Request**:
    - **target_pose** (geometry_msgs/Pose): 目标位姿（位置单位：m，姿态：四元数）
    - **use_joints** (bool): True表示使用关节空间规划，False表示笛卡尔空间规划
    - **velocity_factor** (float32): 速度缩放因子（0.0-1.0）
    - **acceleration_factor** (float32): 加速度缩放因子（0.0-1.0）
  - **Response**:
    - **success** (bool): 执行结果，成功为true
    - **error_code** (int32): 错误代码，成功为0
    - **message** (string): 执行结果的详细信息

#### 使用的服务

- `/aubo_driver/get_ik` (aubo_msgs/GetIK): 逆运动学计算服务（可选）

#### 参数

- `planning_group_name` (string, default: "manipulator_e5"): 规划组名称
- `base_frame` (string, default: "base_link"): 基础坐标系

---

### 4. plan_trajectory_server_node

**功能**：提供轨迹规划服务，规划到目标位姿的轨迹但不执行，返回规划的轨迹和规划时间。

#### 提供的服务

- `/plan_trajectory` (demo_interface/PlanTrajectory)
  - **Request**:
    - **target_pose** (geometry_msgs/Pose): 目标位姿
    - **use_joints** (bool): True表示使用关节空间规划，False表示笛卡尔空间规划
  - **Response**:
    - **success** (bool): 规划成功为true
    - **trajectory** (trajectory_msgs/JointTrajectory): 规划的轨迹
    - **planning_time** (float32): 规划耗时（秒）
    - **message** (string): 规划状态信息

#### 参数

- `planning_group_name` (string, default: "manipulator_e5"): 规划组名称
- `base_frame` (string, default: "base_link"): 基础坐标系

---

### 5. execute_trajectory_server_node

**功能**：提供执行轨迹服务，执行给定的关节轨迹。

#### 提供的服务

- `/execute_trajectory` (demo_interface/ExecuteTrajectory)
  - **Request**:
    - **trajectory** (trajectory_msgs/JointTrajectory): 要执行的轨迹
  - **Response**:
    - **success** (bool): 执行成功为true
    - **error_code** (int32): 错误代码，成功为0
    - **message** (string): 执行结果信息

#### 参数

- `planning_group_name` (string, default: "manipulator_e5"): 规划组名称
- `base_frame` (string, default: "base_link"): 基础坐标系

---

### 6. get_current_state_server_node

**功能**：提供获取当前状态服务，返回当前关节位置、笛卡尔位姿和关节速度。

#### 提供的服务

- `/get_current_state` (demo_interface/GetCurrentState)
  - **Request**: 无
  - **Response**:
    - **success** (bool): 获取成功为true
    - **joint_position_rad** (float64[]): 当前关节角度（弧度）
    - **cartesian_position** (geometry_msgs/Pose): 当前笛卡尔位姿
    - **velocity** (float64[]): 当前关节速度（rad/s）
    - **message** (string): 状态信息

#### 订阅的话题

- `joint_states` (sensor_msgs/JointState): 关节状态

#### 使用的服务

- `/aubo_driver/get_fk` (aubo_msgs/GetFK): 正运动学计算服务

#### 参数

- `planning_group_name` (string, default: "manipulator_e5"): 规划组名称
- `base_frame` (string, default: "base_link"): 基础坐标系

---

### 7. set_speed_factor_server_node

**功能**：提供设置速度因子服务，设置机器人的整体速度缩放因子。

#### 提供的服务

- `/set_speed_factor` (demo_interface/SetSpeedFactor)
  - **Request**:
    - **velocity_factor** (float32): 速度缩放因子（0.0-1.0）
  - **Response**:
    - **success** (bool): 设置成功为true
    - **message** (string): 设置结果信息

#### 参数

- `planning_group_name` (string, default: "manipulator_e5"): 规划组名称
- `base_frame` (string, default: "base_link"): 基础坐标系

---

### 8. set_robot_enable_server_node

**功能**：提供设置机器人使能状态服务，使能或禁用机器人。

#### 提供的服务

- `/set_robot_enable` (demo_interface/SetRobotEnable)
  - **Request**:
    - **enable** (bool): 使能状态，true为使能，false为禁用
  - **Response**:
    - **success** (bool): 执行结果，成功为true
    - **error_code** (int32): 错误代码，成功为0
    - **message** (string): 执行结果的详细信息

#### 发布的话题

- `robot_control` (std_msgs/String): 机器人控制命令（powerOn/powerOff）

#### 参数

- `robot_control_topic` (string, default: "robot_control"): 机器人控制话题名称

---

### 9. set_robot_io_server_node

**功能**：提供设置机器人IO服务，设置指定IO点的状态。

#### 提供的服务

- `/set_robot_io` (demo_interface/SetRobotIO)
  - **Request**:
    - **io_type** (string): IO类型（digital_output/analog_output/tool_io/tool_analog_output）
    - **io_index** (int32): IO点索引
    - **value** (float64): 要设置的值（数字IO：0.0/1.0，模拟IO：实际数值，tool_io输入模式：-1.0）
  - **Response**:
    - **success** (bool): 设置成功为true
    - **error_code** (int32): 错误代码，成功为0
    - **message** (string): 设置结果信息

#### 使用的服务

- `/aubo_driver/set_io` (aubo_msgs/SetIO): aubo驱动器的IO设置服务

#### 参数

- `aubo_set_io_service` (string, default: "/aubo_driver/set_io"): aubo_driver的IO设置服务名称

---

### 10. read_robot_io_server_node

**功能**：提供读取机器人IO服务，读取指定IO点的当前状态。

#### 提供的服务

- `/read_robot_io` (demo_interface/ReadRobotIO)
  - **Request**:
    - **io_type** (string): IO类型（digital_input/digital_output/analog_input/analog_output/tool_io/tool_analog_input）
    - **io_index** (int32): IO点索引
  - **Response**:
    - **success** (bool): 读取成功为true
    - **value** (float64): 读取到的值（数字IO：0.0/1.0，模拟IO：实际数值）
    - **message** (string): 读取结果信息

#### 订阅的话题

- `/aubo_driver/io_states` (aubo_msgs/IOState): aubo驱动器的IO状态

#### 参数

- `io_states_topic` (string, default: "/aubo_driver/io_states"): IO状态话题名称

---

## 使用方法

### 编译

```bash
cd ~/aubo_ws
catkin_make
source devel/setup.bash
```

### 运行节点

#### robot_status_publisher_node

```bash
rosrun demo_driver robot_status_publisher_node
# 或
roslaunch demo_driver robot_status_publisher.launch
```

查看发布的状态：
```bash
rostopic echo /robot_status
```

#### robot_io_status_publisher_node

```bash
rosrun demo_driver robot_io_status_publisher_node
# 或
roslaunch demo_driver robot_io_status_publisher.launch
```

查看发布的IO状态：
```bash
rostopic echo /robot_io_status
```

#### move_to_pose_server_node

```bash
rosrun demo_driver move_to_pose_server_node
# 或
roslaunch demo_driver move_to_pose_server.launch
```

调用服务示例：
```bash
rosservice call /move_to_pose "target_pose:
  position:
    x: 0.4
    y: 0.3
    z: 0.3
  orientation:
    w: 1.0
    x: 0.0
    y: 0.0
    z: 0.0
use_joints: false
velocity_factor: 0.5
acceleration_factor: 0.5"
```

#### plan_trajectory_server_node

```bash
rosrun demo_driver plan_trajectory_server_node
# 或
roslaunch demo_driver plan_trajectory_server.launch
```

调用服务示例：
```bash
rosservice call /plan_trajectory "target_pose:
  position:
    x: 0.4
    y: 0.3
    z: 0.3
  orientation:
    w: 1.0
    x: 0.0
    y: 0.0
    z: 0.0
use_joints: false"
```

#### execute_trajectory_server_node

```bash
rosrun demo_driver execute_trajectory_server_node
# 或
roslaunch demo_driver execute_trajectory_server.launch
```

调用服务示例：
```bash
rosservice call /execute_trajectory "trajectory:
  joint_names: ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}"
```

**典型工作流程**：
1. 调用 `/plan_trajectory` 服务规划轨迹
2. 获取返回的轨迹
3. 调用 `/execute_trajectory` 服务执行轨迹

#### get_current_state_server_node

```bash
rosrun demo_driver get_current_state_server_node
# 或
roslaunch demo_driver get_current_state_server.launch
```

调用服务示例：
```bash
rosservice call /get_current_state
```

#### set_speed_factor_server_node

```bash
rosrun demo_driver set_speed_factor_server_node
# 或
roslaunch demo_driver set_speed_factor_server.launch
```

调用服务示例：
```bash
# 设置速度因子为 0.5（50%速度）
rosservice call /set_speed_factor "velocity_factor: 0.5"
```

**注意**：速度因子范围：0.0 - 1.0，设置后会影响后续的所有运动规划。

#### set_robot_enable_server_node

```bash
rosrun demo_driver set_robot_enable_server_node
# 或
roslaunch demo_driver set_robot_enable_server.launch
```

调用服务示例：
```bash
# 使能机器人
rosservice call /set_robot_enable "enable: true"
# 禁用机器人
rosservice call /set_robot_enable "enable: false"
```

#### set_robot_io_server_node

```bash
rosrun demo_driver set_robot_io_server_node
# 或
roslaunch demo_driver set_robot_io_server.launch
```

调用服务示例：
```bash
# 设置数字输出点0为高电平
rosservice call /set_robot_io "io_type: 'digital_output'
io_index: 0
value: 1.0"

# 设置模拟输出点0为5.0V
rosservice call /set_robot_io "io_type: 'analog_output'
io_index: 0
value: 5.0"

# 设置工具IO点0为输出模式，高电平
rosservice call /set_robot_io "io_type: 'tool_io'
io_index: 0
value: 1.0"
```

**支持的IO类型**：
- `digital_output`: 数字输出（值：0.0=低电平，1.0=高电平）
- `analog_output`: 模拟输出（值：实际电压值）
- `tool_io`: 工具数字IO（值：0.0/1.0=输出模式并设置值，-1.0=输入模式）
- `tool_analog_output`: 工具模拟输出（值：实际电压值）

#### read_robot_io_server_node

```bash
rosrun demo_driver read_robot_io_server_node
# 或
roslaunch demo_driver read_robot_io_server.launch
```

调用服务示例：
```bash
# 读取数字输入点0
rosservice call /read_robot_io "io_type: 'digital_input'
io_index: 0"

# 读取模拟输入点0
rosservice call /read_robot_io "io_type: 'analog_input'
io_index: 0"

# 读取工具IO点0
rosservice call /read_robot_io "io_type: 'tool_io'
io_index: 0"
```

**支持的IO类型**：
- `digital_input`: 数字输入（返回值：0.0=低电平，1.0=高电平）
- `digital_output`: 数字输出（返回值：0.0=低电平，1.0=高电平）
- `analog_input`: 模拟输入（返回值：实际电压值）
- `analog_output`: 模拟输出（返回值：实际电压值）
- `tool_io`: 工具数字IO（返回值：0.0=低电平，1.0=高电平）
- `tool_analog_input`: 工具模拟输入（返回值：实际电压值）
