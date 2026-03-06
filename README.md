# AUBO 机器人 ROS1/ROS2 系统提醒文档

## IP配置
```
相机 169.254.10.91
PC  169.254.10.11
Arm 169.254.10.98
交换机 169.254.10.5
启动脚本 ./start_hand_eye_calibration.sh 
roslaunch aubo_e5_moveit_config moveit_planning_execution.launch robot_ip:=169.254.10.98
```

## 点位配置
```
定义	点位	开	关
做咖啡反馈信号灯	VI0	2V	0V
电源反馈信号灯	VI2	3V	3.3V
打花反馈信号灯	VI3	3V	3.3V
咖啡开关	DO2		
打花开关	DO4		
快换盘开环	DO6		
夹爪开关	DO7		
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
humble版本
```bash
cd /home/mu/IVG/ros2_ws && rm -rf build/ros1_bridge install/ros1_bridge && source /home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash && source /home/mu/IVG/ws_moveit/install_isolated/setup.bash && source /home/mu/IVG/aubo_ws/devel_isolated/setup.bash && source /opt/ros/humble/setup.bash && (test -f /home/mu/IVG/aubo_ros2_ws/install/setup.bash && source /home/mu/IVG/aubo_ros2_ws/install/setup.bash || true) && colcon build --symlink-install --packages-select ros1_bridge
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
catkin_make_isolated && \
source devel_isolated/setup.bash  && \
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

## 9. ROS 系统架构（简化）

### 9.1 数据流
```
MoveIt2 (ROS2) → aubo_ros2_trajectory_action → ros1_bridge → aubo_robot_simulator → aubo_driver → 机器人
```

### 9.2 轨迹重采样（笛卡尔路径）
- **ROS2端**: `UniformSampleFilter` 插值重采样（KDL spline），segment_T ≈ 0.095s
- **ROS1端**: `resample_trajectory_min_interval()` 点选择抽稀
- **参数**: `cartesian_resample_threshold=0.095`, `cartesian_min_segment_interval=0.095`

### 9.3 启动
```bash
./start_hand_eye_calibration.sh  # 一键启动所有节点
```

### 9.4 关键话题
- `/joint_path_command`: ROS1轨迹命令
- `/moveItController_cmd`: ROS1高频率轨迹点 (200Hz)
- `/aubo/feedback_states`: 执行反馈
- `/camera/color/camera_info`: 相机内参（ROS2）

### 9.5 相关文档
- ROS1/ROS2对比: `aubo_ros2_ws/src/aubo_ros2_driver/aubo_demo/ROS1_ROS2_COMPARISON.md`
- ROS2 Control: `aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/ROS2_CONTROL_SETUP_GUIDE.md`
- ros1_bridge: `ros2_ws/src/ros1_bridge/ROS1_BRIDGE_MAPPING_METHODOLOGY.md`

---

## 10. 自动手眼标定系统

### 10.1 概述
基于 Web UI 的交互式手眼标定系统，支持 **Eye-in-Hand** 姿态法自动标定（AX=XB方法）。

**核心特性**：
- ✅ 完全自动化（无需手动点选角点）
- ✅ 自动从 ROS2 CameraInfo 话题获取相机内参
- ✅ 运动质量评估和实时反馈
- ✅ 标定结果确认和保存（XML格式）
- ✅ 现代化 Web 界面（http://localhost:8080）

### 10.2 标定方法
**姿态法（Pose-Based）**：使用 AX=XB 算法
- A = 机器人从位置1到位置2的运动变换（基座坐标系）
- X = 相机到末端执行器的固定变换（待求解）
- B = 标定板从观测1到观测2的运动变换（相机坐标系）

### 10.3 操作流程

**步骤1：启动系统**
```bash
./start_hand_eye_calibration.sh  # 一键启动所有节点
```

**步骤2：打开Web界面**
```
http://localhost:8080
```

**步骤3：采集运动数据**（至少2组，建议3-5组）
```
对每组运动：
1. 移动机器人到位置1 → 点击"📡 记录机器人位姿"（自动拍照并提取标定板位姿）
2. 移动机器人到位置2（建议移动50-200mm或旋转10-30°）→ 再次点击"📡 记录机器人位姿"
3. 自动完成1个运动组 ✓
```

**步骤4：执行标定**
```
点击"🚀 开始自动标定" → 查看标定结果和误差统计
```

**步骤5：保存结果**
```
点击"💾 保存标定结果" → 确认结果 → 下载XML文件
```

### 10.4 运动质量评估

**质量评分标准**：
- ⭐⭐⭐ (≥80%): 高质量 - 运动距离50-200mm，旋转5-45°
- ⭐⭐ (60-80%): 中等质量
- ⭐ (<60%): 低质量 - 建议重新采集

**实时反馈**：
- 每次采集后立即显示质量评分
- 数据质量概览卡片显示整体统计
- 智能提示下一步操作

### 10.5 关键功能

**相机内参自动获取**：
- 自动订阅 `/camera/color/camera_info` 话题
- 无需手动加载标定文件
- 实时更新相机参数

**标定结果展示**：
- 误差统计（平均/最大/最小/标准差）
- 变换矩阵（4×4）
- 旋转表示（矩阵/欧拉角/轴角）
- 平移向量详细信息
- 运动质量分析

### 10.6 数据保存格式

**运动数据**（v3.0格式）：
```json
{
  "version": "3.0",
  "calibrationType": "eye-in-hand-pose-based",
  "calibrationMethod": "AX=XB",
  "motionGroups": [
    {
      "pose1": {
        "robot_pose": {...},
        "board_pose": {...}
      },
      "pose2": {...}
    }
  ]
}
```

**标定结果**（XML格式）：
- 变换矩阵（4×4）
- 旋转矩阵和平移向量
- 标定误差统计
- 运动质量分析

### 10.7 相关文档
- 相机内参集成: `aubo_ros2_ws/src/hand_eye_calibration/相机内参ROS2话题集成说明.md`
- 手眼标定工具: `aubo_ros2_ws/src/hand_eye_calibration/README.md`

---

## 11. 视觉姿态估计系统

### 11.1 概述
基于 Python ROS2 的视觉姿态估计功能包，使用**深度图+彩色图混合处理**进行工件识别和姿态估计。

**核心特性**：
- ✅ 深度图预处理（0值插值、二值化、连通域提取）
- ✅ 特征提取（工件外接圆、阀体外接圆、标准化角度）
- ✅ 模板标准化（旋转到标准方向）
- ✅ 姿态估计（模板匹配、2D对齐、3D姿态计算）
- ✅ Web UI 交互界面（http://localhost:8088/index.html）

### 11.2 处理流程
```
深度图 → 二值化生成掩模 → 从彩色图抠出工件 → 在彩色工件图上提取特征 → 模板匹配 → 姿态估计
```

**关键步骤**：
1. **深度图预处理**：0值插值、深度阈值二值化、连通域提取
2. **彩色图工件提取**：使用掩模从彩色图抠出工件区域
3. **特征提取**：工件外接圆（大圆）、阀体外接圆（小圆）、标准化角度
4. **模板匹配**：与模板库中的模板进行匹配
5. **姿态计算**：2D对齐 + 3D姿态转换到机器人基座坐标系

### 11.3 使用方法

**步骤1：启动节点**
```bash
# 启动相机节点
ros2 launch percipio_camera percipio_camera.launch.py

# 启动姿态估计节点
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

**步骤2：创建模板库**（首次使用）
```bash
# 1. 触发拍照（同时获取深度图和彩色图）
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 2. 标准化模板（创建第一个姿态模板）
ros2 service call /standardize_template interface/srv/StandardizeTemplate \
  "{workpiece_id: '3211242785'}"

# 3. 移动机器人到不同姿态，重复步骤1-2创建多个模板（建议5-10个）
```

**步骤3：姿态估计**
```bash
# 1. 触发拍照
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 2. 估计姿态（自动使用触发拍照获取的最新图像）
ros2 service call /estimate_pose interface/srv/EstimatePose \
  "{object_id: '3211242785'}"
```

### 11.4 ROS2 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/estimate_pose` | `EstimatePose` | 姿态估计（返回抓取位置、置信度等） |
| `/list_templates` | `ListTemplates` | 列出所有可用模板 |
| `/standardize_template` | `StandardizeTemplate` | 标准化模板（创建新模板） |
| `/software_trigger` | `SoftwareTrigger` | 触发相机拍照 |

### 11.5 Web UI

**访问地址**：`http://localhost:8088/index.html`

**功能**：
- 图像上传和拖拽
- 实时姿态估计
- 结果可视化（检测结果、置信度、位姿信息）
- 模板管理（浏览、选择、标准化）
- 参数调试（深度阈值、连通域筛选等）

**启动Web UI**：
```bash
cd aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui
./start_web_ui.sh
```

### 11.6 关键配置

**订阅话题**：
- `/camera/depth/image_raw`: 深度图（16位）
- `/camera/color/image_raw`: 彩色图（BGR8）

**配置文件**：
- `web_ui/configs/debug_thresholds.json`: 深度阈值、连通域筛选参数
- `web_ui/configs/hand_eye_calibration.yaml`: 手眼标定文件
- `web_ui/configs/camera_intrinsics.yaml`: 相机内参

**模板目录结构**：
```
templates/
└── {工件ID}/
    ├── pose_1/
    │   ├── original_image.jpg
    │   ├── image.jpg
    │   ├── mask.jpg
    │   └── metadata.json
    └── pose_2/...
```

### 11.7 注意事项

1. **输入要求**：深度图和彩色图缺一不可，必须同时提供
2. **图像同步**：推荐使用触发拍照模式确保图像同步
3. **模板数量**：建议为每个工件创建5-10个不同姿态的模板
4. **深度阈值**：根据实际场景调整 `binary_threshold_min/max` 参数

### 11.8 相关文档
- 详细文档: `aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/README.md`
- Web UI文档: `aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/README.md`
- 目录结构: `aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/DIRECTORY_STRUCTURE.md`

---

## 12. 重要修复记录

### 12.1 笛卡尔空间路径运动异响问题修复

**问题**：执行笛卡尔空间路径运动时，机械臂发出持续的异响，而关节空间路径运动正常。

**解决方案**：使用插值重采样方法，将笛卡尔路径的点间隔统一到目标值（0.095s），使其与关节空间路径的 segment_T 对齐。

**参考实现**：
- ROS Industrial UniformSampleFilter（插值重采样）
- MoveIt 时间参数化算法
- Universal Robots 驱动设计

**详细文档**：参见 `.cursor/CARTESIAN_PATH_STUTTERING_FIX.md`

**修复日期**：2026-02-05

#### 12.1.1 代码实现位置

**ROS2 端（C++）** - 使用插值重采样（推荐）：
- 文件：`aubo_ros2_ws/src/aubo_ros2_driver/aubo_ros2_trajectory_action/src/aubo_ros2_trajectory_action.cpp`
- 核心函数：`UniformSampleFilter::update()` - 使用 KDL spline 插值
- 调用位置：`JointTrajectoryAction::publishTrajectory()` 第218-250行
- 实现方式：每隔 `target_interval`（0.095s）插值一个新点，保证均匀时间间隔

**ROS1 端（Python）** - 使用点选择抽稀：
- 文件：`aubo_ws/src/aubo_robot/aubo_robot/aubo_controller/script/aubo_controller/trajectory_speed.py`
- 核心函数：`resample_trajectory_min_interval()` - 点选择方法
- 调用位置：`aubo_robot_simulator` 的 `trajectory_callback()` 第902行
- 实现方式：选择间隔 >= `target_interval` 的点，跳过密集点

#### 12.1.2 配置参数

**ROS2 Launch 文件**：
```xml
<node name="aubo_ros2_trajectory_action" pkg="aubo_ros2_trajectory_action" type="aubo_ros2_trajectory_action">
  <!-- 笛卡尔路径重采样参数 -->
  <param name="cartesian_resample_threshold" value="0.095"/>      <!-- 检测阈值：点间隔 < 0.095s 认为是密集笛卡尔路径 -->
  <param name="cartesian_min_segment_interval" value="0.095"/>   <!-- 目标间隔：重采样后的点间隔 -->
</node>
```

**ROS1 Launch 文件**：
```xml
<group ns="aubo_controller">
  <param name="cartesian_resample_threshold" value="0.095"/>
  <param name="cartesian_min_segment_interval" value="0.095"/>
</group>
```

#### 12.1.3 工作原理

1. **自动检测**：计算轨迹中最小点间隔，如果 < `cartesian_resample_threshold`（0.095s），判定为密集笛卡尔路径
2. **ROS2 端（插值重采样）**：
   - 从时间 0 开始，每隔 `target_interval`（0.095s）插值一个新点
   - 使用 KDL `VelocityProfile_Spline` 保证位置、速度、加速度连续性
   - 保证均匀的 segment_T，与关节空间对齐
3. **ROS1 端（点选择）**：
   - 选择间隔 >= `target_interval` 的点
   - 跳过密集点，减少轨迹点数量
   - 简单高效，但不如插值重采样平滑

#### 12.1.4 效果

- **修复前**：笛卡尔路径 segment_T = 0.0748~0.2326s（不均匀），边界频率 6.4~10.0/s，**持续异响**
- **修复后**：笛卡尔路径 segment_T ≈ 0.095s（均匀），边界频率 ≈ 10.5/s（与关节空间对齐），**异响消失**

---

## 环境与故障排查记录

以下为 IVG 环境与启动脚本相关问题的处理记录，便于复现与排查。

### roslaunch 报错 `ModuleNotFoundError: No module named 'defusedxml'`

- **原因**：ROS1 的 `rosmaster` 依赖 `defusedxml`，当前 Python 未安装。
- **处理**：安装后即可正常 `roslaunch`。
  ```bash
  pip3 install --user defusedxml
  # 或
  sudo apt install python3-defusedxml
  ```

### roslaunch 报错 `Invalid <param> tag: Cannot load command parameter [rosversion]: no such command`

- **原因**：`roscore.xml` 通过执行 `rosversion` 设置参数，自编译 `ros_catkin_ws` 未装 `rosbash`，无 `rosversion` 命令。
- **处理**：已改为固定参数，不再依赖 `rosversion`。
  - 修改文件：`ros_catkin_ws/src/ros_comm/roslaunch/resources/roscore.xml` 及 `install_isolated` 下对应 `roscore.xml`。
  - 将 `command="rosversion roslaunch"` / `command="rosversion -d"` 改为 `value="1.16.0"` / `value="noetic"`。
- **注意**：若使用其他 distro（如 melodic），可把 `rosdistro` 的 value 改为对应名称。

### ros2 run 报 `Package 'ros1_bridge' not found`

- **原因**：`ros2_ws` 的 `install/` 中未包含 `ros1_bridge`（此前未完整编译该包）。
- **处理**：在**干净终端**中先完整编译桥接工作空间，再只 source ROS2 环境运行桥。
  ```bash
  # 编译（需先 source ROS1 链 + ROS2 链，见 ros2_ws/README_BRIDGE.md）
  /home/mu/IVG/ros2_ws/build_bridge.sh
  # 运行桥（仅 ROS2 环境，避免 ROS_DISTRO 混用）
  source /opt/ros/humble/setup.bash
  source /home/mu/IVG/ros2_ws/install/setup.bash
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test
  ```

### start_IVG.sh 使用说明与修改

- **ROS 环境**：已统一为 IVG 路径。ROS1 链：`ros_catkin_ws` → `ws_moveit` → `aubo_ws`（使用 `catkin_make_isolated` 与 `devel_isolated/setup.bash`）；ROS2：Humble；`ros2_ws` 为 `/home/mu/IVG/ros2_ws`。
- **运行方式**：请用 **bash** 运行（`./start_IVG.sh` 或 `bash start_IVG.sh`），勿用 `sh start_IVG.sh`；若已用 `sh` 启动，脚本会自动用 `bash` 重新执行。
- **terminator**：若 PATH 中未找到 `terminator`，脚本会再检查 `/usr/bin/terminator`。

### hand_eye_calibration 报 `ModuleNotFoundError: No module named 'flask'`

- **原因**：手眼标定节点依赖 Flask 提供 Web 界面，当前 Python 环境未安装。
- **处理**：安装后无需重新 colcon build，直接再次启动即可。
  ```bash
  pip3 install --user flask flask-cors
  ```
- **包内**：`hand_eye_calibration/setup.py` 已加入 `install_requires=['setuptools', 'flask', 'flask-cors']`，便于 pip 安装时自动拉取依赖。

---

**文档版本**：1.6  
**最后更新**：2026-02-25

