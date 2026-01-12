# RViz2 Motion Planning插件配置映射指南

本文档详细说明MoveIt配置文件如何与RViz2中Motion Planning插件的各个设置项对应。

---

## 目录

1. [概述](#概述)
2. [Motion Planning插件界面结构](#motion-planning插件界面结构)
3. [配置文件映射关系](#配置文件映射关系)
4. [详细配置项对应表](#详细配置项对应表)
5. [配置修改示例](#配置修改示例)

---

## 概述

RViz2的Motion Planning插件通过ROS 2话题和参数服务器与MoveIt的move_group节点通信。插件界面上显示的设置和选项主要从以下配置文件读取：

- **SRDF文件** (`aubo_i5.srdf`) - 规划组、关节状态、碰撞检测设置
- **moveit_controllers.yaml** - 控制器配置
- **ompl_planning.yaml** - 规划器配置
- **kinematics.yaml** - 运动学求解器配置
- **joint_limits.yaml** - 关节限制配置
- **启动文件参数** - 话题名称、命名空间等

---

## Motion Planning插件界面结构

RViz2 Motion Planning插件主要分为以下几个部分：

1. **Motion Planning面板** - 左侧控制面板
   - Context（上下文）- 规划组选择
   - Planning（规划）- 规划参数设置
   - Commands（命令）- 执行控制
   - Current State（当前状态）- 机器人状态
   - Goal State（目标状态）- 目标设置

2. **Displays面板** - 右侧显示设置
   - Planned Path（规划路径）- 路径可视化
   - Scene Robot（场景机器人）- 机器人显示
   - Planning Request（规划请求）- 规划请求可视化

---

## 配置文件映射关系

### 1. 规划组（Planning Groups）选择

**RViz界面位置：** Context → Planning Group 下拉菜单

**配置文件：** `config/aubo_i5.srdf`

**对应代码：**
```xml
<group name="manipulator">
    <chain base_link="base_link" tip_link="ee_link"/>
</group>
<group name="endeffector">
    <link name="ee_link"/>
</group>
```

**说明：**
- SRDF文件中定义的`<group>`标签会在Motion Planning插件的"Planning Group"下拉菜单中显示
- 用户可以选择不同的规划组进行运动规划
- 每个规划组定义了哪些关节参与规划

---

### 2. 规划器（Planner）选择

**RViz界面位置：** Planning → Planner 下拉菜单

**配置文件：** `config/ompl_planning.yaml`

**对应代码：**
```yaml
manipulator:
  default_planner_config: RRTConnect  # 默认规划器
  planner_configs:                    # 可用规划器列表
    - AnytimePathShortening
    - SBL
    - EST
    - RRTConnect
    - RRTstar
    - PRM
    # ... 更多规划器
```

**说明：**
- `planner_configs` 列表中的规划器会出现在RViz的"Planner"下拉菜单中
- `default_planner_config` 设置默认选中的规划器
- 每个规划组可以有不同的规划器配置

---

### 3. 规划时间（Planning Time）

**RViz界面位置：** Planning → Planning Time 输入框

**配置文件：** 通过ROS参数动态设置，或通过RViz插件内部参数

**启动文件参数：** `aubo_moveit.launch.py` 中的 `ompl_planning_pipeline_config`

**对应代码：**
```python
# 在启动文件中，可以设置默认规划时间
ompl_planning_pipeline_config = {
    "move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "planning_time": 5.0,  # 默认规划时间（秒）
        # ...
    }
}
```

**说明：**
- 在RViz中可以手动输入规划时间（秒）
- 该值会被发送到move_group节点进行规划请求
- 默认值通常为5.0秒

---

### 4. 规划尝试次数（Planning Attempts）

**RViz界面位置：** Planning → Planning Attempts 输入框

**配置文件：** 通过ROS参数或RViz插件内部参数

**说明：**
- 指定规划算法尝试多少次来找到有效路径
- 默认值通常为10次
- 在RViz中可以手动修改

---

### 5. 速度缩放因子（Velocity Scaling Factor）

**RViz界面位置：** Planning → Velocity Scaling Factor 滑块

**配置文件：** `config/joint_limits.yaml`

**对应代码：**
```yaml
default_velocity_scaling_factor: 1.0  # 默认速度缩放因子（0.0-1.0）
default_acceleration_scaling_factor: 0.1  # 默认加速度缩放因子
```

**说明：**
- 在RViz中通过滑块调整（0.0-1.0）
- 该值影响轨迹执行的速度
- `joint_limits.yaml`中的`default_velocity_scaling_factor`设置默认值

---

### 6. 加速度缩放因子（Acceleration Scaling Factor）

**RViz界面位置：** Planning → Acceleration Scaling Factor 输入框

**配置文件：** `config/joint_limits.yaml`

**对应代码：**
```yaml
default_acceleration_scaling_factor: 0.1  # 默认加速度缩放因子
```

**说明：**
- 在RViz中可以手动输入
- 影响轨迹执行的加速度
- 通常设置为较小的值（如0.1）以确保平滑运动

---

### 7. 控制器选择和执行

**RViz界面位置：** Commands → Execute 按钮

**配置文件：** `config/moveit_controllers.yaml`

**对应代码：**
```yaml
controller_names:
  - joint_trajectory_controller  # 可用的控制器列表

joint_trajectory_controller:
  action_ns: follow_joint_trajectory  # Action命名空间
  type: FollowJointTrajectory
  default: true  # 默认控制器
  joints:
    - shoulder_joint
    - upperArm_joint
    - foreArm_joint
    - wrist1_joint
    - wrist2_joint
    - wrist3_joint
```

**说明：**
- `controller_names` 列表定义了可用的控制器
- `default: true` 标记的控制器会被默认使用
- MoveIt通过Action接口（`/joint_trajectory_controller/follow_joint_trajectory`）发送轨迹
- Action服务器由ROS 2 Control的`joint_trajectory_controller`提供

---

### 8. 机器人描述（Robot Description）

**RViz界面位置：** Displays → MotionPlanning → Robot Description 参数

**配置文件：** 启动文件中的`robot_description`参数

**对应代码：**
```python
# aubo_moveit.launch.py
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([
        FindPackageShare(support_package), 
        "urdf/xacro/inc/", 
        robot_xacro_file_to_use
    ]),
    " aubo_type:=", aubo_type,
])
robot_description = {"robot_description": robot_description_content}

# 传递给RViz节点
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    parameters=[
        robot_description,  # ← 这里
        robot_description_semantic,
        # ...
    ],
)
```

**RViz配置：**
```yaml
# config/moveit.rviz
Robot Description: robot_description  # ROS参数名称
```

**说明：**
- RViz从ROS参数服务器读取`robot_description`参数
- 该参数包含完整的URDF/XACRO生成的机器人描述
- 用于在RViz中渲染机器人模型

---

### 9. 规划场景话题（Planning Scene Topic）

**RViz界面位置：** Displays → MotionPlanning → Planning Scene Topic 参数

**配置文件：** 启动文件中的`planning_scene_monitor_parameters`

**对应代码：**
```python
# aubo_moveit.launch.py
planning_scene_monitor_parameters = {
    "planning_scene_monitor_options": {
        "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        # ...
    }
}
```

**RViz配置：**
```yaml
# config/moveit.rviz
Planning Scene Topic: monitored_planning_scene  # 话题名称（相对于move_group命名空间）
```

**说明：**
- RViz订阅该话题来获取规划场景信息
- 用于显示碰撞对象、环境障碍物等
- 默认话题：`/move_group/monitored_planning_scene`

---

### 10. 规划路径话题（Planned Path Topic）

**RViz界面位置：** Displays → MotionPlanning → Planned Path → Trajectory Topic

**配置文件：** RViz配置文件

**对应代码：**
```yaml
# config/moveit.rviz
Planned Path:
  Trajectory Topic: display_planned_path  # 话题名称
```

**说明：**
- move_group发布规划好的轨迹到这个话题
- RViz订阅该话题来可视化规划的路径
- 默认话题：`/display_planned_path`

---

### 11. 运动学求解器配置

**RViz界面位置：** 隐式使用（用于IK求解）

**配置文件：** `config/kinematics.yaml`

**对应代码：**
```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

**说明：**
- 当用户在RViz中拖动交互式标记（Interactive Marker）设置目标位姿时
- MoveIt使用运动学求解器进行逆运动学（IK）计算
- `kinematics_solver_search_resolution`影响IK求解精度
- `kinematics_solver_timeout`设置求解超时时间

---

### 12. 关节限制

**RViz界面位置：** 隐式使用（在规划时检查）

**配置文件：** `config/joint_limits.yaml`

**对应代码：**
```yaml
joint_limits:
  shoulder_joint:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 3.15
  # ... 其他关节
```

**说明：**
- MoveIt在规划时检查关节限制
- 如果目标位姿超出关节限制，规划会失败
- `max_velocity`和`max_acceleration`限制轨迹执行速度

---

### 13. 轨迹执行参数

**RViz界面位置：** 隐式使用（在执行时应用）

**配置文件：** 启动文件中的`trajectory_execution`参数

**对应代码：**
```python
# aubo_moveit.launch.py
trajectory_execution = {
    "moveit_manage_controllers": False,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
}
```

**或者：**

```yaml
# config/moveit_controllers.yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2  # 允许的执行时间缩放
  allowed_goal_duration_margin: 0.5        # 目标时间容差
  allowed_start_tolerance: 0.01            # 起始状态容差
```

**说明：**
- `allowed_execution_duration_scaling`: 允许轨迹执行时间比计划时间长多少倍
- `allowed_goal_duration_margin`: 允许的目标时间误差（秒）
- `allowed_start_tolerance`: 允许的起始位置误差（弧度）

---

### 14. Move Group命名空间

**RViz界面位置：** Displays → MotionPlanning → Move Group Namespace

**配置文件：** RViz配置文件或启动参数

**对应代码：**
```yaml
# config/view_robot.rviz
Move Group Namespace: ""  # 空字符串表示使用默认命名空间
```

**说明：**
- 如果move_group节点运行在非默认命名空间（如`/robot1/move_group`）
- 需要在此处指定命名空间（如`robot1`）
- 空字符串表示使用默认命名空间（`/move_group`）

---

## 详细配置项对应表

| RViz界面位置 | 配置文件/参数 | 配置项路径/键名 | 说明 |
|-------------|--------------|----------------|------|
| **Context → Planning Group** | `aubo_i5.srdf` | `<group name="...">` | 规划组列表 |
| **Planning → Planner** | `ompl_planning.yaml` | `manipulator.planner_configs` | 可用规划器列表 |
| **Planning → Planner (默认)** | `ompl_planning.yaml` | `manipulator.default_planner_config` | 默认规划器 |
| **Planning → Planning Time** | 启动参数/RViz内部 | 动态参数 | 规划时间（秒） |
| **Planning → Planning Attempts** | RViz内部 | 动态参数 | 规划尝试次数 |
| **Planning → Velocity Scaling** | `joint_limits.yaml` | `default_velocity_scaling_factor` | 速度缩放因子 |
| **Planning → Acceleration Scaling** | `joint_limits.yaml` | `default_acceleration_scaling_factor` | 加速度缩放因子 |
| **Commands → Execute** | `moveit_controllers.yaml` | `controller_names`, `default` | 控制器选择 |
| **Displays → Robot Description** | 启动参数 | `robot_description` | URDF描述 |
| **Displays → Planning Scene Topic** | 启动参数 | `monitored_planning_scene_topic` | 规划场景话题 |
| **Displays → Trajectory Topic** | RViz配置 | `display_planned_path` | 规划路径话题 |
| **IK求解（交互式标记）** | `kinematics.yaml` | `kinematics_solver` | 运动学求解器 |
| **关节限制检查** | `joint_limits.yaml` | `joint_limits.*` | 关节限制 |
| **轨迹执行参数** | `moveit_controllers.yaml` | `trajectory_execution.*` | 执行容差 |

---

## 配置修改示例

### 示例1: 添加新的规划器

**需求：** 在RViz的规划器下拉菜单中添加新的规划器

**步骤：**

1. 编辑 `config/ompl_planning.yaml`：
```yaml
manipulator:
  planner_configs:
    - AnytimePathShortening
    - SBL
    - EST
    - RRTConnect
    - RRTstar
    - PRM
    - NewPlanner  # ← 添加新规划器（需要先在planner_configs中定义）
```

2. 在文件顶部定义规划器配置：
```yaml
planner_configs:
  NewPlanner:
    type: geometric::RRTConnect  # 规划器类型
    range: 0.0
    # ... 其他参数
```

3. 重新构建并启动：
```bash
colcon build --packages-select aubo_moveit_config
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py
```

4. 在RViz的"Planner"下拉菜单中即可看到新规划器

---

### 示例2: 修改默认速度缩放因子

**需求：** 降低默认运动速度，使机器人运动更平滑

**步骤：**

1. 编辑 `config/joint_limits.yaml`：
```yaml
default_velocity_scaling_factor: 0.5  # 从1.0改为0.5（降低到50%）
default_acceleration_scaling_factor: 0.05  # 从0.1改为0.05
```

2. 重新构建并启动，RViz中的速度缩放滑块默认值会变为0.5

---

### 示例3: 更改规划场景话题

**需求：** 使用自定义的规划场景话题

**步骤：**

1. 修改启动文件 `launch/aubo_moveit.launch.py`：
```python
planning_scene_monitor_parameters = {
    "planning_scene_monitor_options": {
        "monitored_planning_scene_topic": "/custom/planning_scene",  # 修改话题名称
        # ...
    }
}
```

2. 或者在RViz中手动修改：
   - Displays → MotionPlanning → Planning Scene Topic
   - 输入：`custom/planning_scene`

---

### 示例4: 添加新的规划组

**需求：** 添加一个新的规划组（例如只控制前3个关节）

**步骤：**

1. 编辑 `config/aubo_i5.srdf`：
```xml
<group name="arm_base">
    <chain base_link="base_link" tip_link="foreArm_Link"/>
</group>
```

2. 在 `config/ompl_planning.yaml` 中添加该组的规划器配置：
```yaml
arm_base:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - RRTstar
    - PRM
```

3. 在 `config/kinematics.yaml` 中添加运动学配置（如果需要）：
```yaml
arm_base:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

4. 重新构建并启动，在RViz的"Planning Group"下拉菜单中即可看到新组

---

### 示例5: 配置多个控制器

**需求：** 支持多个控制器（例如主机械臂和末端执行器）

**步骤：**

1. 编辑 `config/moveit_controllers.yaml`：
```yaml
controller_names:
  - arm_controller
  - gripper_controller  # 添加新控制器

arm_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - shoulder_joint
    - upperArm_joint
    - foreArm_joint
    - wrist1_joint
    - wrist2_joint
    - wrist3_joint

gripper_controller:
  action_ns: gripper_action
  type: GripperCommand
  default: false
  joints:
    - gripper_joint
```

2. 在RViz中，MoveIt会根据规划的轨迹自动选择相应的控制器

---

## 实际工作流程

当您在RViz中操作时，配置的使用流程如下：

1. **选择Planning Group** → 从SRDF读取group定义，确定哪些关节参与规划
2. **选择Planner** → 从ompl_planning.yaml读取规划器配置
3. **设置目标位姿** → 使用kinematics.yaml中的运动学求解器计算IK
4. **点击Plan** → move_group使用选定的规划器进行规划，考虑joint_limits.yaml中的限制
5. **点击Execute** → MoveIt通过moveit_controllers.yaml中配置的Action接口发送轨迹
6. **轨迹执行** → ROS 2 Control的joint_trajectory_controller接收并执行轨迹

---

## 关键话题和Action接口

### 输入话题（RViz → MoveIt）

- `/move_group/goal` - 规划请求
- `/move_group/query` - 查询请求
- `/move_group/execute` - 执行请求

### 输出话题（MoveIt → RViz）

- `/display_planned_path` - 规划的路径可视化
- `/move_group/monitored_planning_scene` - 规划场景更新
- `/move_group/display_contacts` - 碰撞接触点

### Action接口（MoveIt ↔ 控制器）

- `/joint_trajectory_controller/follow_joint_trajectory` - 轨迹执行Action
- `/move_group/move_action` - MoveIt移动Action
- `/move_group/execute_trajectory` - 轨迹执行Action

---

## 总结

RViz2 Motion Planning插件的配置主要来源于：

1. **SRDF文件** → 规划组定义、组状态、碰撞检测
2. **OMPL规划配置文件** → 规划器选择、规划器参数
3. **关节限制配置文件** → 速度和加速度限制、默认缩放因子
4. **控制器配置文件** → 轨迹执行、Action接口
5. **启动文件参数** → 话题和命名空间、机器人描述
6. **RViz配置文件** → 可视化设置、显示选项

理解这些对应关系有助于：
- 快速定位配置问题
- 根据需求调整配置
- 优化运动规划性能
- 自定义Motion Planning插件行为
- 调试规划和执行问题

---

**文档版本**: 1.0  
**最后更新**: 2024-12-18  
**维护者**: AUBO Robotics Team

