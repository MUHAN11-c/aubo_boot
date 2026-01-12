# MoveIt 控制流程详解

基于 `moveit_planning_execution.launch` 文件的完整控制流程分析

## 📋 目录
1. [系统架构概览](#系统架构概览)
2. [启动组件详解](#启动组件详解)
3. [数据流分析](#数据流分析)
4. [控制流程步骤](#控制流程步骤)
5. [关键话题和服务](#关键话题和服务)
6. [配置参数说明](#配置参数说明)

---

## 系统架构概览

```
┌─────────────────────────────────────────────────────────────┐
│                    MoveIt 控制架构                          │
└─────────────────────────────────────────────────────────────┘

用户/应用层
    │
    ├─> RViz (可视化界面)
    │   └─> /move_group (Action/Service)
    │
    └─> MoveIt API (C++/Python)
        └─> /move_group (Action/Service)

MoveIt 核心层
    │
    ├─> move_group 节点
    │   ├─ 运动规划 (OMPL)
    │   ├─ 轨迹执行管理
    │   └─ 规划场景监控
    │
    └─> 控制器管理器
        └─> moveit_simple_controller_manager

执行层
    │
    ├─> aubo_joint_trajectory_action (Action Server)
    │   └─> /follow_joint_trajectory (Action)
    │
    ├─> aubo_robot_simulator (仿真器)
    │   └─> 接收轨迹命令
    │
    └─> aubo_driver (真实机器人驱动)
        └─> 与机器人控制器通信

硬件层
    │
    └─> AUBO E5 机器人控制器
```

---

## 启动组件详解

### 1. 规划上下文加载 (`planning_context.launch`)

**功能**: 加载机器人描述和规划配置

**加载内容**:
- **URDF**: 机器人物理描述 (`aubo_e5.xacro`)
- **SRDF**: 语义描述 (`aubo_e5.srdf`) - 定义规划组、碰撞对等
- **关节限制**: `joint_limits.yaml` - 速度、加速度、位置限制
- **运动学配置**: `kinematics.yaml` - 逆运动学求解器参数

**参数命名空间**:
- `/robot_description` - URDF 内容
- `/robot_description_semantic` - SRDF 内容
- `/robot_description_planning` - 关节限制
- `/robot_description_kinematics` - 运动学参数

### 2. 仿真控制器节点

#### `aubo_robot_simulator`
- **类型**: Python 脚本节点
- **功能**: 
  - 接收轨迹命令并模拟执行
  - 发布关节状态反馈
  - 提供仿真环境，无需真实机器人

#### `aubo_joint_trajectory_action`
- **类型**: C++ 可执行文件
- **功能**: 
  - 提供 `FollowJointTrajectory` Action 服务器
  - 接收 MoveIt 的轨迹执行请求
  - 将轨迹发送给仿真器或真实驱动
  - 监控执行状态并反馈

**关键话题**:
- `/follow_joint_trajectory/goal` - 接收轨迹目标
- `/follow_joint_trajectory/feedback` - 发布执行反馈
- `/follow_joint_trajectory/result` - 返回执行结果
- `/joint_path_command` - 发布轨迹命令（发送给仿真器/驱动）

### 3. 机器人驱动节点 (`aubo_driver`)

**功能**:
- 连接 AUBO 机器人控制器（TCP/IP）
- 发布关节状态 (`/joint_states`)
- 接收轨迹命令并执行
- 发布机器人状态和 IO 状态

**关键话题**:
- `/joint_states` - 发布关节状态（位置、速度、力矩）
- `/feedback_states` - 发布轨迹执行反馈
- `/aubo_driver/io_states` - 发布 IO 状态
- `/robot_status` - 发布机器人状态
- `moveItController_cmd` - 订阅 MoveIt 轨迹命令

### 4. TF 发布节点

#### `robot_state_publisher`
- **功能**: 根据 URDF 和关节状态发布机器人各连杆的 TF 变换
- **输入**: `/robot_description` 参数 + `/joint_states` 话题
- **输出**: TF 树（base_link → 各关节 → end_effector）

### 5. MoveIt 核心节点 (`move_group`)

**功能**: MoveIt 的核心节点，负责运动规划、执行和场景监控

**主要能力**:
- **运动规划**: 使用 OMPL 规划器生成无碰撞轨迹
- **轨迹执行**: 通过控制器管理器执行轨迹
- **场景监控**: 监控规划场景变化（碰撞对象、附加对象等）
- **状态管理**: 维护机器人当前状态和规划场景

**关键 Action/Service**:
- `/move_group/move_action` - 规划并执行运动
- `/move_group/execute_trajectory` - 执行已规划的轨迹
- `/move_group/plan_kinematic_path` - 仅规划不执行
- `/move_group/get_planning_scene` - 获取规划场景

### 6. RViz 可视化节点

**功能**: 可视化机器人模型、规划场景和运动轨迹

**显示内容**:
- 机器人模型（根据当前关节状态）
- 规划轨迹预览
- 碰撞检测可视化
- 交互式标记（用于手动设置目标）

---

## 数据流分析

### 规划到执行的完整数据流

```
1. 用户请求 (RViz/MoveIt API)
   │
   └─> /move_group/move_action (Action Goal)
       │
       ├─ 目标位姿/关节值
       ├─ 规划约束
       └─ 规划选项

2. move_group 节点
   │
   ├─> 规划阶段
   │   ├─ 读取当前关节状态 (/joint_states)
   │   ├─ 获取规划场景
   │   ├─ 调用 OMPL 规划器
   │   ├─ 碰撞检测
   │   └─ 生成轨迹 (moveit_msgs::RobotTrajectory)
   │
   └─> 执行阶段
       ├─ 轨迹验证
       ├─ 时间参数化
       └─ 发送到控制器管理器

3. 控制器管理器 (moveit_simple_controller_manager)
   │
   └─> 转换为 FollowJointTrajectory Action Goal
       │
       └─> /follow_joint_trajectory/goal

4. aubo_joint_trajectory_action
   │
   ├─> 验证轨迹
   ├─> 发布轨迹命令
   │   └─> /joint_path_command (trajectory_msgs::JointTrajectory)
   │
   └─> 监控执行状态

5. aubo_robot_simulator / aubo_driver
   │
   ├─> 接收轨迹命令
   ├─> 执行轨迹（仿真或真实）
   └─> 发布反馈
       ├─> /joint_states
       └─> /feedback_states (FollowJointTrajectoryFeedback)

6. 反馈循环
   │
   ├─> aubo_joint_trajectory_action 接收反馈
   ├─> 检查执行状态
   ├─> 发布 /follow_joint_trajectory/feedback
   └─> move_group 监控执行进度
```

### 关节状态流

```
aubo_driver / aubo_robot_simulator
    │
    └─> /joint_states (sensor_msgs::JointState)
        │
        ├─> robot_state_publisher
        │   └─> TF 变换树
        │
        ├─> move_group
        │   └─> 规划场景更新
        │
        └─> RViz
            └─> 可视化更新
```

---

## 控制流程步骤

### 步骤 1: 系统初始化

1. **加载机器人描述**
   - URDF → `/robot_description`
   - SRDF → `/robot_description_semantic`
   - 关节限制 → `/robot_description_planning`
   - 运动学配置 → `/robot_description_kinematics`

2. **启动控制器节点**
   - `aubo_robot_simulator` 启动
   - `aubo_joint_trajectory_action` 启动并等待连接
   - `aubo_driver` 连接机器人控制器（如果使用真实机器人）

3. **启动 MoveIt 节点**
   - `move_group` 加载配置
   - 初始化规划器（OMPL）
   - 初始化控制器管理器
   - 启动规划场景监控

### 步骤 2: 运动规划请求

1. **接收规划请求**
   ```
   用户 → /move_group/move_action (Action Goal)
   ```

2. **获取当前状态**
   ```
   move_group → /joint_states (订阅)
   ```

3. **执行规划**
   - 设置起始状态（当前关节值）
   - 设置目标状态（目标位姿/关节值）
   - 调用 OMPL 规划器
   - 碰撞检测和路径优化
   - 生成轨迹点序列

4. **轨迹后处理**
   - 时间参数化（添加速度、加速度）
   - 轨迹采样和插值
   - 验证轨迹可行性

### 步骤 3: 轨迹执行

1. **轨迹验证**
   ```
   move_group → 验证轨迹
   ├─ 起始状态匹配检查
   ├─ 关节限制检查
   └─ 碰撞检查
   ```

2. **发送到控制器**
   ```
   move_group → 控制器管理器
   └─> /follow_joint_trajectory/goal
   ```

3. **控制器处理**
   ```
   aubo_joint_trajectory_action
   ├─ 接收 Action Goal
   ├─ 验证关节名称匹配
   ├─ 发布轨迹命令
   │   └─> /joint_path_command
   └─ 开始监控执行
   ```

4. **执行轨迹**
   ```
   aubo_robot_simulator / aubo_driver
   ├─ 接收轨迹命令
   ├─ 按时间序列执行轨迹点
   └─ 发布执行反馈
   ```

### 步骤 4: 执行监控

1. **状态反馈**
   ```
   aubo_driver/simulator → /joint_states
   aubo_driver/simulator → /feedback_states
   ```

2. **反馈处理**
   ```
   aubo_joint_trajectory_action
   ├─ 接收反馈
   ├─ 检查执行进度
   ├─ 检查错误状态
   └─ 发布 Action Feedback
   ```

3. **完成通知**
   ```
   aubo_joint_trajectory_action → /follow_joint_trajectory/result
   move_group → /move_group/move_action/result
   用户 ← 执行结果
   ```

---

## 关键话题和服务

### 话题 (Topics)

| 话题名称 | 消息类型 | 发布者 | 订阅者 | 说明 |
|---------|---------|--------|--------|------|
| `/joint_states` | `sensor_msgs::JointState` | aubo_driver<br>aubo_robot_simulator | robot_state_publisher<br>move_group<br>RViz | 关节状态（位置、速度、力矩） |
| `/feedback_states` | `control_msgs::FollowJointTrajectoryFeedback` | aubo_driver | aubo_joint_trajectory_action | 轨迹执行反馈 |
| `/joint_path_command` | `trajectory_msgs::JointTrajectory` | aubo_joint_trajectory_action | aubo_robot_simulator<br>aubo_driver | 轨迹执行命令 |
| `/aubo_driver/io_states` | `aubo_msgs::IOState` | aubo_driver | - | IO 状态信息 |
| `/robot_status` | `industrial_msgs::RobotStatus` | aubo_driver | - | 机器人状态 |
| `/move_group/monitored_planning_scene` | `moveit_msgs::PlanningScene` | move_group | RViz | 监控的规划场景 |

### Action 接口

| Action 名称 | Action 类型 | 服务器 | 客户端 | 说明 |
|------------|------------|--------|--------|------|
| `/move_group/move_action` | `moveit_msgs::MoveGroupAction` | move_group | RViz<br>MoveIt API | 规划并执行运动 |
| `/move_group/execute_trajectory` | `moveit_msgs::ExecuteTrajectoryAction` | move_group | MoveIt API | 执行已规划轨迹 |
| `/follow_joint_trajectory` | `control_msgs::FollowJointTrajectoryAction` | aubo_joint_trajectory_action | move_group | 关节轨迹执行 |

### Service 接口

| Service 名称 | Service 类型 | 服务器 | 说明 |
|------------|------------|--------|------|
| `/move_group/plan_kinematic_path` | `moveit_msgs::GetMotionPlan` | move_group | 仅规划不执行 |
| `/move_group/get_planning_scene` | `moveit_msgs::GetPlanningScene` | move_group | 获取规划场景 |
| `/aubo_driver/set_io` | `aubo_msgs::SetIO` | aubo_driver | 设置 IO 状态 |
| `/aubo_driver/get_ik` | `aubo_msgs::GetIK` | aubo_driver | 逆运动学求解 |
| `/aubo_driver/get_fk` | `aubo_msgs::GetFK` | aubo_driver | 正运动学计算 |

---

## 配置参数说明

### 轨迹执行参数 (`trajectory_execution.launch.xml`)

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 4.0    # 允许的执行时长缩放因子
  allowed_goal_duration_margin: 0.5         # 允许的目标时长余量（秒）
  allowed_start_tolerance: 0.01              # 允许的起始位置容差（弧度）
  moveit_manage_controllers: true           # 是否由 MoveIt 管理控制器
```

### 控制器配置 (`controllers.yaml`)

```yaml
controller_list:
  - name: aubo_e5_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - shoulder_joint
      - upperArm_joint
      - foreArm_joint
      - wrist1_joint
      - wrist2_joint
      - wrist3_joint
```

### 规划器配置 (`ompl_planning.yaml`)

- **规划算法**: RRTConnect, RRT, PRM 等
- **规划时间限制**: 通常 5-10 秒
- **采样分辨率**: 影响规划质量和速度

### move_group 参数

```yaml
move_group:
  allow_trajectory_execution: true          # 允许轨迹执行
  max_safe_path_cost: 1.0                   # 最大安全路径代价
  jiggle_fraction: 0.05                     # 抖动分数（用于路径优化）
  publish_monitored_planning_scene: true    # 发布监控的规划场景
```

---

## 执行状态机

```
IDLE (空闲)
  │
  ├─> 接收规划请求
  │
PLANNING (规划中)
  │
  ├─> 规划成功 → EXECUTING (执行中)
  │   │
  │   ├─> 执行成功 → SUCCEEDED (成功)
  │   ├─> 执行失败 → FAILED (失败)
  │   ├─> 被抢占 → PREEMPTED (抢占)
  │   └─> 超时 → TIMED_OUT (超时)
  │
  └─> 规划失败 → FAILED (失败)
```

---

## 错误处理机制

### 常见错误类型

1. **规划失败**
   - 无碰撞路径不存在
   - 目标不可达
   - 规划时间超时

2. **执行失败**
   - 起始状态不匹配
   - 关节限制违反
   - 控制器通信失败
   - 执行超时

3. **安全停止**
   - 紧急停止触发
   - 保护停止触发
   - 碰撞检测触发

### 错误恢复

- **自动重试**: 某些错误可自动重试
- **状态重置**: 失败后重置到安全状态
- **错误反馈**: 通过 Action Result 返回详细错误信息

---

## 性能优化建议

1. **规划性能**
   - 调整 OMPL 规划器参数
   - 使用合适的规划算法
   - 优化碰撞检测分辨率

2. **执行性能**
   - 调整轨迹采样频率
   - 优化时间参数化
   - 减少不必要的状态更新

3. **系统性能**
   - 合理设置话题队列大小
   - 优化 TF 发布频率
   - 减少不必要的场景更新

---

## 调试技巧

1. **查看规划场景**
   ```bash
   rosrun moveit_ros_planning_scene_monitor print_planning_scene_info
   ```

2. **监控轨迹执行**
   ```bash
   rostopic echo /follow_joint_trajectory/feedback
   ```

3. **检查关节状态**
   ```bash
   rostopic echo /joint_states
   ```

4. **查看 MoveIt 日志**
   - 设置 `output="screen"` 查看 move_group 输出
   - 使用 `--debug` 参数启用调试模式

---

## 总结

MoveIt 控制流程是一个复杂的多层级系统，涉及：
- **规划层**: 生成无碰撞运动轨迹
- **执行层**: 将轨迹转换为机器人控制命令
- **监控层**: 实时监控执行状态和安全性
- **反馈层**: 提供状态反馈和错误处理

通过 `moveit_planning_execution.launch` 文件，所有这些组件被整合在一起，形成一个完整的机器人运动控制系统。

