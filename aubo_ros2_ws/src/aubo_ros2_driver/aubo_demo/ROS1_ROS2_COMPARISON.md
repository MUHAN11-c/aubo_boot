# ROS1 vs ROS2 轨迹动作实现对比

## 📊 架构对比

### ROS1 实现 (`aubo_controller/joint_trajectory_action`)

```
MoveIt (ROS1)
    │
    └─> Action: /aubo_e5_controller/follow_joint_trajectory
         │
         ▼
┌─────────────────────────────────────────┐
│  joint_trajectory_action (Action Server) │
│  - 接收轨迹目标                           │
│  - 验证轨迹有效性                         │
│  - 发布完整轨迹到Topic                    │
└─────────────────────────────────────────┘
    │
    └─> Topic: /joint_path_command (JointTrajectory)
         │
         ▼
┌─────────────────────────────────────────┐
│  aubo_robot_simulator (轨迹插值器)        │
│  - 接收完整轨迹                           │
│  - 5次多项式插值 (200Hz)                 │
│  - 发布高频率轨迹点                       │
└─────────────────────────────────────────┘
    │
    └─> Topic: /moveItController_cmd (JointTrajectoryPoint)
         │
         ▼
┌─────────────────────────────────────────┐
│  aubo_driver (机器人驱动)                 │
│  - 接收高频率轨迹点                       │
│  - 发送到真实机器人                       │
└─────────────────────────────────────────┘
```

### ROS2 实现 (`aubo_ros2_trajectory_action`)

```
MoveIt2 (ROS2)
    │
    └─> Action: /aubo_i5_controller/follow_joint_trajectory
         │
         ▼
┌─────────────────────────────────────────┐
│  aubo_ros2_trajectory_action             │
│  - 接收轨迹目标                           │
│  - 验证轨迹有效性                         │
│  - 5次多项式插值 (100Hz, 0.01s)          │
│  - 直接发布高频率轨迹点                   │
└─────────────────────────────────────────┘
    │
    └─> Topic: /aubo_robot/moveit_controller (JointTrajectoryPoint)
         │
         ▼
┌─────────────────────────────────────────┐
│  aubo_driver (ROS2驱动，假设存在)         │
│  - 接收高频率轨迹点                       │
│  - 发送到真实机器人                       │
└─────────────────────────────────────────┘
```

---

## 🔄 主要差异

### 1. **轨迹处理方式**

| 特性 | ROS1 | ROS2 |
|------|------|------|
| **轨迹发布** | 发布完整轨迹 (`JointTrajectory`) | 直接发布插值后的轨迹点 (`JointTrajectoryPoint`) |
| **插值位置** | 在 `aubo_robot_simulator` 中 | 在 `aubo_ros2_trajectory_action` 中 |
| **插值频率** | 200Hz (0.005s) | 100Hz (0.01s) |
| **发布话题** | `/joint_path_command` → `/moveItController_cmd` | `/aubo_robot/moveit_controller` |

**ROS1流程：**
```
Action → 完整轨迹 → 插值器 → 高频率点 → 驱动
```

**ROS2流程：**
```
Action → 插值处理 → 高频率点 → 驱动
```

### 2. **话题接口**

#### ROS1 话题

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|------|
| `/joint_path_command` | `trajectory_msgs::JointTrajectory` | Action → Simulator | 完整轨迹命令 |
| `/moveItController_cmd` | `trajectory_msgs::JointTrajectoryPoint` | Simulator → Driver | 高频率轨迹点 (200Hz) |
| `/feedback_states` | `control_msgs::FollowJointTrajectoryFeedback` | Driver → Action | 执行反馈 |
| `/robot_status` | `industrial_msgs::RobotStatus` | Driver → Action | 机器人状态 |
| `/trajectory_execution_event` | `std_msgs::String` | User → Action | 执行事件 ("stop") |
| `/joint_states` | `sensor_msgs::JointState` | Driver → System | 关节状态 |

#### ROS2 话题

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|------|
| `/aubo_robot/moveit_controller` | `trajectory_msgs::msg::JointTrajectoryPoint` | Action → Driver | 高频率轨迹点 (100Hz) |
| `/aubo_robot/fjt_feedback` | `control_msgs::action::FollowJointTrajectory_Feedback` | Driver → Action | 执行反馈 |
| `/aubo_robot/moveit_execution` | `std_msgs::msg::String` | User → Action | 执行事件 ("stop") |

### 3. **代码结构对比**

#### ROS1 实现特点

```cpp
// 1. 使用 actionlib (ROS1)
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> 
    JointTractoryActionServer;

// 2. 发布完整轨迹
pub_trajectory_command_.publish(current_traj_);  // JointTrajectory

// 3. 订阅反馈
sub_trajectory_state_ = node_.subscribe("feedback_states", ...);

// 4. 在回调中检查完成条件
if (withinGoalConstraints(...)) {
    active_goal_.setSucceeded();
}
```

#### ROS2 实现特点

```cpp
// 1. 使用 rclcpp_action (ROS2)
rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

// 2. 直接发布插值后的轨迹点
moveit_controller_pub_->publish(intermediate_goal_point);  // JointTrajectoryPoint

// 3. 订阅反馈
fjt_feedback_sub_ = this->create_subscription<...>("/aubo_robot/fjt_feedback", ...);

// 4. 在回调中检查完成条件
if (checkReachTarget(...)) {
    active_goal_->succeed(result);
}
```

### 4. **轨迹插值实现**

#### ROS1 (在 aubo_robot_simulator 中)

```python
# Python实现，5次多项式插值
# 更新频率: 200Hz (0.005s)
# 在独立线程中运行
def _motion_worker(self):
    while not self.sig_shutdown:
        current_goal_point = self.motion_buffer.get()
        # 5次多项式插值
        # 发布到 /moveItController_cmd
```

#### ROS2 (在 aubo_ros2_trajectory_action 中)

```cpp
// C++实现，5次多项式插值
// 更新频率: 100Hz (0.01s)
// 在独立线程中运行
void calculateMotionTrajectory() {
    // 5次多项式插值
    // 直接发布到 /aubo_robot/moveit_controller
    moveit_controller_pub_->publish(intermediate_goal_point);
}
```

### 5. **看门狗机制**

| 特性 | ROS1 | ROS2 |
|------|------|------|
| **周期** | 1.0秒 | 2.0秒 |
| **检查** | 控制器是否响应 | 是否有反馈消息 |
| **动作** | 中止目标 | 中止目标 |

**ROS1:**
```cpp
watchdog_timer_ = node_.createTimer(ros::Duration(1.0), ...);
```

**ROS2:**
```cpp
watch_dog_timer_ = create_wall_timer(std::chrono::seconds(2), ...);
```

### 6. **目标完成检查**

#### ROS1

```cpp
// 检查条件：
// 1. 在目标约束内 (withinGoalConstraints)
// 2. 机器人已停止运动 (in_motion == FALSE)
// 3. 等待到轨迹执行到一半才开始检查
if (withinGoalConstraints(...) && 
    last_robot_status_->in_motion.val == FALSE) {
    active_goal_.setSucceeded();
}
```

#### ROS2

```cpp
// 检查条件：
// 1. 位置误差 < 0.002 弧度
// 2. 所有关节都满足条件
if (abs(feedback->actual.positions[i] - 
        traj.points[last_point].positions[i]) <= 0.002) {
    active_goal_->succeed(result);
}
```

### 7. **关节顺序重映射**

#### ROS1
- 不进行重映射，假设关节顺序一致

#### ROS2
```cpp
// 重新映射关节顺序
trajectory_msgs::msg::JointTrajectory remapTrajectoryByJointName(...)
```

### 8. **均匀采样过滤**

#### ROS1
- 不支持

#### ROS2
```cpp
// 可选：使用UniformSampleFilter生成均匀时间间隔的轨迹点
UniformSampleFilter uniform_filter;
if (uniform_filter.update(remap_traj, uniform_filter_traj)) {
    // 使用过滤后的轨迹
}
```

---

## 🔀 桥接需求

要将ROS2的MoveIt2/ros2_control接口桥接到ROS1驱动，需要：

### 1. **Action → Topic 转换**

```
ROS2 Action: /joint_trajectory_controller/follow_joint_trajectory
    │
    ▼ (桥接节点)
ROS1 Topic: /joint_path_command (trajectory_msgs::JointTrajectory)
```

**实现要点：**
- ROS2 Action服务器接收MoveIt2请求
- 将Action目标转换为ROS1的完整轨迹消息
- 发布到ROS1的`/joint_path_command`话题

### 2. **Feedback Topic → Action Feedback 转换**

```
ROS1 Topic: /feedback_states (control_msgs::FollowJointTrajectoryFeedback)
    │
    ▼ (桥接节点)
ROS2 Action Feedback: /joint_trajectory_controller/follow_joint_trajectory
```

**实现要点：**
- 订阅ROS1的`/feedback_states`话题
- 转换为ROS2 Action反馈消息
- 发布到ROS2 Action客户端

### 3. **Joint States 桥接**

```
ROS1 Topic: /joint_states (sensor_msgs::JointState)
    │
    ▼ (桥接节点)
ROS2 Topic: /joint_states (sensor_msgs::msg::JointState)
```

### 4. **Robot Status 监控**

```
ROS1 Topic: /robot_status (industrial_msgs::RobotStatus)
    │
    ▼ (桥接节点)
ROS2 Action: 中止目标（如果检测到异常）
```

---

## 📝 总结

### 关键差异

1. **ROS1**: 分离式设计（Action → 完整轨迹 → 插值器 → 高频率点）
2. **ROS2**: 集成式设计（Action → 插值处理 → 高频率点）

3. **ROS1**: 插值在独立的Python节点中（200Hz）
4. **ROS2**: 插值在Action服务器中（100Hz）

5. **ROS1**: 需要完整的轨迹消息传递
6. **ROS2**: 直接发布插值后的轨迹点

### 桥接策略

1. **简化方案**: ROS2 Action → 完整轨迹 → ROS1 Topic
2. **保持ROS1架构**: 让`aubo_robot_simulator`继续处理插值
3. **反馈转换**: ROS1反馈 → ROS2 Action反馈
4. **状态同步**: ROS1关节状态 → ROS2关节状态

