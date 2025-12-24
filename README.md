# AUBO 机器人 ROS1/ROS2 系统提醒文档

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



---

**文档版本**：1.0  
**最后更新**：2025-12-19

