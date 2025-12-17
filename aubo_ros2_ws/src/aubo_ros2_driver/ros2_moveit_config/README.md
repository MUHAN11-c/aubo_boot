# ROS 2 MoveIt2 配置 - 通过桥接控制真实机械臂

本配置包允许使用 MoveIt2 和 RViz2 通过 `ros1_bridge` 控制 ROS 1 机器人控制器。

## 目录结构

```
ros2_moveit_config/
├── config/
│   ├── aubo_e5.srdf              # MoveIt 语义描述文件
│   ├── controllers.yaml          # MoveIt 控制器配置
│   ├── ros2_controllers.yaml     # ROS 2 控制器配置
│   ├── joint_limits.yaml         # 关节限制
│   ├── joint_names.yaml          # 关节名称
│   ├── kinematics.yaml           # 运动学配置
│   ├── ompl_planning.yaml        # OMPL 规划器配置
│   └── moveit.rviz               # RViz2 配置文件
├── launch/
│   ├── moveit_bridge_control.launch.py  # 主启动文件
│   └── start_ros1_side.launch.py        # ROS 1 端启动参考
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    ROS 2 端 (Foxy)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   MoveIt2    │  │    RViz2     │  │ ros1_bridge  │  │
│  │  move_group  │  │              │  │              │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
│         │                 │                 │          │
│         └─────────────────┴─────────────────┘          │
│                          │                              │
└──────────────────────────┼──────────────────────────────┘
                           │
                    ┌──────▼──────┐
                    │ ros1_bridge │
                    └──────┬──────┘
                           │
┌──────────────────────────┼──────────────────────────────┐
│                    ROS 1 端 (Noetic)                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  demo_driver │  │   MoveIt      │  │  机器人驱动  │  │
│  │   services   │  │  (可选)       │  │              │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
│         │                 │                 │          │
│         └─────────────────┴─────────────────┘          │
│                          │                              │
│                    ┌─────▼─────┐                        │
│                    │  真实机械臂  │                        │
│                    └───────────┘                        │
└─────────────────────────────────────────────────────────┘
```

## 使用方法

### 前置条件

1. **ROS 1 环境**（Noetic）
   - 已安装并配置 ROS 1 Noetic
   - 已编译 `aubo_ws` 工作空间
   - 已编译 `demo_driver` 包

2. **ROS 2 环境**（Foxy）
   - 已安装并配置 ROS 2 Foxy
   - 已编译 `ros2_ws` 工作空间
   - 已编译 `ros1_bridge` 包

3. **机器人连接**
   - 机器人已上电并连接到网络
   - 已知机器人 IP 地址

### 启动步骤

#### 步骤 1：启动 ROS 1 端（终端 1）

```bash
# 启动 ROS 1 环境
source /opt/ros/noetic/setup.bash
source /home/mu/IVG/aubo_ws/devel/setup.bash

# 启动 roscore
roscore
```

#### 步骤 2：启动 ROS 1 机器人驱动和 MoveIt（终端 2）

```bash
# 启动 ROS 1 环境
source /opt/ros/noetic/setup.bash
source /home/mu/IVG/aubo_ws/devel/setup.bash

# 启动 MoveIt 和机器人驱动
roslaunch aubo_e5_moveit_config moveit_planning_execution.launch robot_ip:=<机器人IP>
```

#### 步骤 3：启动 ROS 1 demo_driver 服务（终端 3）

```bash
# 启动 ROS 1 环境
source /opt/ros/noetic/setup.bash
source /home/mu/IVG/aubo_ws/devel/setup.bash

# 启动 execute_trajectory 服务
rosrun demo_driver execute_trajectory_server_node

# 启动 robot_status_publisher（可选，用于状态反馈）
rosrun demo_driver robot_status_publisher_node
```

#### 步骤 4：启动 ROS 2 端和桥接（终端 4）

```bash
# 启动 ROS 2 环境
source /opt/ros/foxy/setup.bash
source /home/mu/ros2_ws/install/setup.bash

# 启动 MoveIt2、RViz2 和桥接
ros2 launch ros2_moveit_config moveit_bridge_control.launch.py robot_ip:=<机器人IP>
```

### 参数说明

- `robot_ip`: 机器人 IP 地址（默认：127.0.0.1）
- `use_rviz`: 是否启动 RViz2（默认：true）
- `use_bridge`: 是否启动 ros1_bridge（默认：true）

### 使用示例

```bash
# 完整启动命令
ros2 launch ros2_moveit_config moveit_bridge_control.launch.py \
    robot_ip:=192.168.1.100 \
    use_rviz:=true \
    use_bridge:=true
```

## 控制流程

### 1. 在 RViz2 中规划路径

1. 打开 RViz2（已自动启动）
2. 在 Motion Planning 插件中：
   - 设置目标位姿（通过拖拽或输入数值）
   - 点击 "Plan" 按钮规划路径
   - 查看规划的路径（绿色显示）

### 2. 执行轨迹

**方式 1：通过 RViz2 执行**
- 在 Motion Planning 插件中点击 "Execute" 按钮
- MoveIt2 会调用 `demo_driver` 的 `/execute_trajectory` 服务
- 服务通过桥接发送到 ROS 1 端执行

**方式 2：通过命令行执行**
```bash
# 调用执行服务
ros2 service call /execute_trajectory demo_interface/srv/ExecuteTrajectory "{trajectory: {...}}"
```

### 3. 监控执行状态

**订阅状态话题**：
```bash
# 查看机器人状态
ros2 topic echo /robot_status

# 查看关节状态
ros2 topic echo /joint_states
```

**查询当前状态**：
```bash
# 调用查询服务
ros2 service call /get_current_state demo_interface/srv/GetCurrentState "{}"
```

## 反馈机制

### 判断是否到达目标点

1. **服务返回值**（推荐）
   - 执行轨迹后，服务返回 `success` 和 `error_code`
   - `success == true` 表示执行成功，已到达目标点

2. **状态话题监控**（实时）
   - 订阅 `/robot_status` 话题
   - 监控 `planning_status` 和 `in_motion` 字段
   - 比较当前位姿和目标位姿

3. **关节状态监控**（实时）
   - 订阅 `/joint_states` 话题
   - 比较当前关节角度和目标角度

## 故障排除

### 问题 1：桥接无法连接

**症状**：ROS 2 端无法看到 ROS 1 的话题和服务

**解决方案**：
1. 确认 `ros1_bridge` 已启动
2. 确认 ROS 1 端 `roscore` 正在运行
3. 检查环境变量：`ROS_MASTER_URI` 应指向 ROS 1 master

```bash
# 检查桥接状态
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

### 问题 2：MoveIt2 无法规划

**症状**：点击 Plan 按钮无响应或报错

**解决方案**：
1. 确认 `/robot_description` 话题有数据
2. 确认 `/joint_states` 话题有数据（通过桥接）
3. 检查规划组配置是否正确

```bash
# 检查话题
ros2 topic list
ros2 topic echo /robot_description
ros2 topic echo /joint_states
```

### 问题 3：执行失败

**症状**：规划成功但执行失败

**解决方案**：
1. 确认 ROS 1 端 `execute_trajectory_server` 正在运行
2. 确认服务可以访问：
   ```bash
   ros2 service list | grep execute_trajectory
   ```
3. 检查服务调用是否成功：
   ```bash
   ros2 service call /execute_trajectory demo_interface/srv/ExecuteTrajectory "{trajectory: {...}}"
   ```

### 问题 4：RViz2 中看不到机器人

**症状**：RViz2 启动但显示空白

**解决方案**：
1. 确认 `robot_state_publisher` 正在运行
2. 确认 `/robot_description` 话题有数据
3. 在 RViz2 中添加 RobotModel 显示
4. 检查 Fixed Frame 设置（应为 `base_link` 或 `pedestal`）

## 高级配置

### 自定义控制器

编辑 `config/ros2_controllers.yaml` 以配置自定义控制器。

### 自定义规划器

编辑 `config/ompl_planning.yaml` 以配置规划器参数。

### 添加传感器

在 `config/sensors_3d.yaml` 中配置 3D 传感器（如果使用）。

## 相关文档

- [ROS 1 Bridge 映射方法论](../ros1_bridge/ROS1_BRIDGE_MAPPING_METHODOLOGY.md)
- [MoveIt 桥接指南](../ros1_bridge/MOVEIT_BRIDGE_GUIDE.md)
- [demo_driver 文档](../../demo_driver/README.md)

## 注意事项

1. **Action 无法桥接**：MoveIt2 的 Action 接口无法直接桥接，因此使用服务接口
2. **实时反馈**：通过订阅状态话题获取实时反馈，而不是 Action feedback
3. **执行确认**：执行完成后通过服务返回值确认是否成功
4. **环境隔离**：确保 ROS 1 和 ROS 2 环境正确隔离，避免冲突

## 许可证

BSD

