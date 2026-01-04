# AUBO 机器人 ROS1/ROS2 系统提醒文档

## IP配置
```
相机 169.254.10.91
PC  169.254.10.11
Arm 169.254.10.98
交换机 169.254.10.5
```

## 1. ROS1 ROS2 Bridge 桥接

### 1.1 桥接编译执行

```bash
cd /home/mu/ros2_ws && \
source /opt/ros/noetic/setup.bash && \
source /home/mu/IVG/aubo_ws/devel/setup.bash && \
source /opt/ros/foxy/setup.bash && \
source /home/mu/IVG/aubo_ros2_ws/install/setup.bash && \
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

启动动态桥接器：
```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test
```

### 1.2 ROS1 ROS2 启动执行

**ROS1 端启动：**
```bash
cd /home/mu/IVG/aubo_ws && \
use_ros1 && \
catkin_make && \
source devel/setup.bash && \
roslaunch aubo_e5_moveit_config aubo_e5_moveit_bridge.launch
```

**ROS2 端启动：**
```bash
cd ~/IVG/aubo_ros2_ws && \
use_ros2 && \
colcon build && \
source install/setup.bash && \
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
```
**ROS2 端接口启动：**
```bash
cd ~/IVG/aubo_ros2_ws && \
use_ros2 && \
colcon build && \
source install/setup.bash && \
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

### 1.3 MoveIt2 通过桥接驱动机械臂

#### 1.3.1 架构说明

MoveIt2（ROS2）通过 `ros1_bridge` 桥接器连接到 ROS1 端的机器人驱动，实现跨版本的机器人控制。整体架构流程如下：

```
MoveIt2 (ROS2)
    │
    └─> Action: /joint_trajectory_controller/follow_joint_trajectory
         │
         ▼
┌─────────────────────────────────────────┐
│  aubo_ros2_trajectory_action (ROS2)     │
│  - 接收 MoveIt2 轨迹目标                 │
│  - 将 Action 转换为轨迹点                │
│  - 发布到 ROS1 话题                      │
└─────────────────────────────────────────┘
    │
    └─> Topic: /moveItController_cmd (ROS1，通过桥接)
         │
         ▼
┌─────────────────────────────────────────┐
│  aubo_driver (ROS1)                      │
│  - 接收轨迹点                            │
│  - 发送到真实机器人                      │
└─────────────────────────────────────────┘
    │
    └─> 真实机械臂

反馈流：
真实机械臂 → aubo_driver (ROS1) 
         → /joint_states, /feedback_states (ROS1)
         → feedback_bridge (ROS2)
         → MoveIt2 (ROS2)
```

#### 1.3.2 完整启动流程

**步骤 1：启动 ROS1 端驱动**

在第一个终端窗口启动 ROS1 端的机器人驱动和桥接支持节点：

```bash
cd /home/mu/IVG/aubo_ws && \
use_ros1 && \
catkin_make && \
source devel/setup.bash && \
roslaunch aubo_e5_moveit_config aubo_e5_moveit_bridge.launch
```

**启动内容：**
- `aubo_driver`：机器人底层驱动，连接真实机器人（IP: 10.8.144.98）
- `aubo_robot_simulator`：轨迹插值器，处理轨迹点
- `aubo_joint_trajectory_action`：ROS1 Action 服务器（桥接场景中作为备用）
- 机器人状态和IO服务节点

**步骤 2：启动 ros1_bridge 桥接器**

在第二个终端窗口启动动态桥接器，连接 ROS1 和 ROS2：

```bash
cd /home/mu/ros2_ws && \
source /opt/ros/noetic/setup.bash && \
source /home/mu/IVG/aubo_ws/devel/setup.bash && \
source /opt/ros/foxy/setup.bash && \
source /home/mu/IVG/aubo_ros2_ws/install/setup.bash && \
source install/setup.bash && \
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test
```

**说明：**
- `--bridge-all-topics`：自动桥接所有兼容的话题
- 关键桥接话题：`/moveItController_cmd`（ROS2→ROS1）、`/joint_states`（ROS1→ROS2）、`/feedback_states`（ROS1→ROS2）

**步骤 3：启动 ROS2 端 MoveIt2**

在第三个终端窗口启动 MoveIt2 和相关桥接节点：

```bash
cd ~/IVG/aubo_ros2_ws && \
use_ros2 && \
colcon build --symlink-install && \
source install/setup.bash && \
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
```

**启动内容：**
- `move_group`：MoveIt2 运动规划核心节点
- `aubo_ros2_trajectory_action`：将 MoveIt2 Action 转换为 ROS1 话题
- `feedback_bridge`：桥接机器人状态反馈（`/feedback_states`、`/joint_states`）到 ROS2
- `robot_state_publisher`：发布机器人TF变换
- `rviz2`：可视化界面

**步骤 4（可选）：启动 ROS2 接口服务**

如果需要使用额外的 ROS2 服务接口（如轨迹规划、执行等服务），可在第四个终端窗口启动：

```bash
cd ~/IVG/aubo_ros2_ws && \
use_ros2 && \
source install/setup.bash && \
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

#### 1.3.3 关键节点说明

| 节点/组件 | 作用 | 所在端 |
|----------|------|--------|
| `move_group` | MoveIt2 运动规划核心，接收规划请求并生成轨迹 | ROS2 |
| `aubo_ros2_trajectory_action` | 监听 `/joint_trajectory_controller/follow_joint_trajectory` Action，将轨迹发布到 `/moveItController_cmd`（ROS1） | ROS2 |
| `feedback_bridge` | 桥接 ROS1 的 `/feedback_states` 和 `/joint_states` 到 ROS2 | ROS2 |
| `ros1_bridge` | 动态桥接 ROS1 和 ROS2 之间的消息和话题 | 桥接层 |
| `aubo_driver` | 机器人底层驱动，连接真实机器人硬件 | ROS1 |
| `aubo_robot_simulator` | 轨迹插值器，处理轨迹点序列 | ROS1 |

#### 1.3.4 数据流说明

**轨迹执行流程（ROS2 → ROS1）：**
1. MoveIt2 规划轨迹后，通过 Action 发送到 `/joint_trajectory_controller/follow_joint_trajectory`
2. `aubo_ros2_trajectory_action` 接收 Action 目标，转换为轨迹点
3. 发布到 `/moveItController_cmd` 话题（ROS2 端）
4. `ros1_bridge` 将话题桥接到 ROS1 端
5. ROS1 端的 `aubo_robot_simulator` 接收并处理轨迹点
6. `aubo_driver` 将轨迹点发送到真实机器人执行

**状态反馈流程（ROS1 → ROS2）：**
1. `aubo_driver` 从机器人获取关节状态和反馈信息
2. 发布 `/joint_states` 和 `/feedback_states` 话题（ROS1 端）
3. `ros1_bridge` 将话题桥接到 ROS2 端
4. `feedback_bridge` 接收并转换反馈消息格式
5. MoveIt2 接收关节状态更新，用于运动规划

#### 1.3.5 注意事项

1. **启动顺序**：必须按照 ROS1 驱动 → 桥接器 → ROS2 MoveIt2 的顺序启动
2. **话题桥接**：确保 `ros1_bridge` 正确桥接以下关键话题：
   - `/moveItController_cmd`（ROS2→ROS1）
   - `/joint_states`（ROS1→ROS2）
   - `/feedback_states`（ROS1→ROS2）
3. **速度控制**：`aubo_ros2_trajectory_action` 默认速度缩放因子为 0.5（50%），可在 launch 文件中调整 `velocity_scale_factor` 参数
4. **机器人IP配置**：ROS1 端 `aubo_driver` 的机器人IP地址在 launch 文件中配置（默认：10.8.144.98）


## 2. ROS1 单独启动

```bash
roslaunch aubo_e5_moveit_config moveit_planning_execution.launch robot_ip:=10.8.144.98
```

## 3. 安装流程

### 3.1 ROS1 (Noetic) 依赖包安装

```bash
sudo apt install git
sudo apt install ros-noetic-industrial-*

# 安装Gazebo相关控制包
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control \
                 ros-noetic-joint-state-controller ros-noetic-position-controllers ros-noetic-joint-trajectory-controller

# 安装MoveIt相关控制器和可视化工具
sudo apt-get install ros-noetic-moveit-simple-controller-manager
sudo apt-get install ros-noetic-rviz-visual-tools
sudo apt-get install ros-noetic-moveit-visual-tools
sudo apt install ros-noetic-moveit-visual-tools
sudo apt install ros-noetic-moveit-ros-planning-interface
sudo apt install ros-noetic-moveit-ros-perception
sudo apt install ros-noetic-moveit-planners-ompl
sudo apt install ros-noetic-moveit-ros-visualization
sudo apt install ros-noetic-moveit-simple-controller-manager
sudo apt install ros-noetic-ompl
sudo apt-get install ros-noetic-moveit-planners-ompl
sudo apt-get update
sudo apt-get install ros-noetic-moveit-ros-visualization
```

### 3.2 ROS2 (Foxy) 依赖包安装

```bash
sudo apt install ros-foxy-eigen-stl-containers
sudo apt install ros-foxy-moveit-common
sudo apt install ros-foxy-graph-msgs
sudo apt install ros-foxy-moveit-core
sudo apt install ros-foxy-moveit-ros-planning
sudo apt install ros-foxy-control-msgs
sudo apt install ros-foxy-moveit-ros-planning-interface
sudo apt install ros-foxy-moveit-ros-perception
sudo apt install ros-foxy-moveit-servo

# 安装MoveIt 2核心包
sudo apt update && sudo apt install ros-foxy-moveit
sudo apt install ros-foxy-xacro
```

### 3.3 安装 ROS2 Foxy

```bash
# 安装 ROS2 Foxy 桌面版
sudo apt install ros-foxy-desktop -y
```

## 4. 安装 ros1_bridge

### Step 1: 创建工作空间并克隆 ros1_bridge

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge.git
```

**说明：**
- `-b foxy` 指定使用与 ROS 2 Foxy 版本匹配的 ros1_bridge 分支
- `~/ros2_ws` 是 ROS2 的 colcon 工作空间

### 启动桥接器（指定节点名，避免冲突）

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test
```

## 5. 启动桥接器脚本

创建启动脚本 `start_bridge.sh`：

```bash
#!/bin/bash

# =================== 设置 =====================
ROS1_SETUP="/opt/ros/noetic/setup.bash"
ROS2_SETUP="/opt/ros/foxy/setup.bash"
BRIDGE_SETUP="$HOME/ros2_ws/install/setup.bash"
# =============================================

# 检查 roscore 是否已运行
if ! pgrep -x "roscore" > /dev/null; then
  echo "启动 roscore (ROS1 Master)..."
  gnome-terminal -- bash -c "source $ROS1_SETUP && roscore"
  sleep 3
else
  echo "roscore 已运行，跳过。"
fi

# 启动 ROS1 Talker 节点
echo "启动 ROS1 talker 节点..."
gnome-terminal -- bash -c "source $ROS1_SETUP && rosrun roscpp_tutorials talker"

# 启动 ros1_bridge
echo "启动 ros1_bridge 动态桥接器..."
gnome-terminal -- bash -c "source $ROS1_SETUP && source $ROS2_SETUP && source $BRIDGE_SETUP && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test"

sleep 5

# 启动 ROS2 Listener
echo "启动 ROS2 listener（监听 ROS1 发布的 /chatter）..."
gnome-terminal -- bash -c "source $ROS2_SETUP && ros2 topic echo /chatter"
```

## 6. 环境别名定义

在 `~/.bashrc` 或 `~/.zshrc` 中添加：

```bash
# 定义别名 use_ros1，用于加载 ROS1（Noetic）环境
alias use_ros1="source /opt/ros/noetic/setup.bash"

# 定义别名 use_ros2，用于加载 ROS2（Foxy）环境
alias use_ros2="source /opt/ros/foxy/setup.bash"
```

## 7. colcon build --symlink-install 核心说明

`colcon build --symlink-install` 是 ROS 2 中用于编译工作空间的核心命令，`--symlink-install` 是关键参数，作用是创建符号链接（软链接）而非复制文件，大幅提升开发效率。

| 特性 | 普通 colcon build | colcon build --symlink-install |
|------|------------------|-------------------------------|
| 文件处理方式 | 编译后将文件复制到安装目录（install/） | 编译后为文件创建软链接到安装目录 |
| 修改源码后 | 需重新编译才能生效 | 修改 Python 脚本、launch 文件、配置文件等无需重新编译，仅需重启节点即可生效 |
| 磁盘空间 | 占用更多磁盘空间（复制文件） | 节省磁盘空间（仅存链接） |

**使用场景：**
1. 高频修改的文件：Python 代码、launch 文件（.launch.py/.launch.xml）、配置文件（.yaml）、URDF/SDF 模型文件、rviz 配置文件等
2. 开发调试阶段：避免每次改代码都重新编译，提升迭代效率
3. 注意：C++ 代码修改后仍需重新编译（因为涉及编译链接过程），但 `--symlink-install` 对 C++ 的安装目录仍有软链接优化



## 8. 深度 z 读取节点 (depth_z_reader)

### 8.1 编译和启动

编译 `depth_z_reader` 包：

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select depth_z_reader
source install/setup.bash
```

启动深度 z 读取节点：

```bash
ros2 launch depth_z_reader depth_z_reader.launch.py
```

### 8.2 功能说明

`depth_z_reader` 节点用于从 percipio 相机获取深度 z 值：

- 订阅深度图像话题：`/camera/depth/image_raw`
- 读取指定像素位置的深度值（默认读取图像中心点）
- 发布深度值到话题：`/depth_z/center`
- 支持自动搜索有效深度值（当中心点深度无效时）

### 8.3 参数配置

可通过 launch 文件参数配置：

- `depth_image_topic`：深度图像话题名称（默认：`/camera/depth/image_raw`）
- `pixel_x`、`pixel_y`：要读取的像素坐标（-1 表示图像中心）
- `depth_scale`：深度缩放因子（默认：`0.00025`，适用于 scale_unit=0.25 的相机）
- `search_valid_depth`：是否在中心点无效时搜索附近有效深度（默认：`true`）
- `search_radius`：搜索有效深度值的半径（默认：`50` 像素）

---

**文档版本**：1.0  
**最后更新**：2025-12-19

