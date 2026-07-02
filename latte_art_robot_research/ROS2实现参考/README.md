# ROS2 实现参考

## 一、系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      感知层 (Perception)                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐ │
│  │ RGB 摄像头│  │ 深度相机  │  │ 力传感器  │  │ 温度传感器    │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────┬───────┘ │
│       │              │             │               │         │
├───────┴──────────────┴─────────────┴───────────────┴─────────┤
│                      规划层 (Planning)                        │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  latte_art_trajectory (拉花轨迹生成)                   │   │
│  │  ├── 图案参数化 → 2D/3D 轨迹点                         │   │
│  │  ├── 抗晃荡速度剖面                                     │   │
│  │  └── → CartesianPath 请求                              │   │
│  └─────────────────────┬────────────────────────────────┘   │
│                        ▼                                     │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  MoveIt2 (运动规划)                                    │   │
│  │  ├── MoveGroupInterface                              │   │
│  │  ├── CartesianPathService                            │   │
│  │  ├── 逆运动学 (Trac-IK / KDL / IKFast)               │   │
│  │  └── 碰撞检测 (FCL / Bullet)                          │   │
│  └─────────────────────┬────────────────────────────────┘   │
├─────────────────────────┴───────────────────────────────────┤
│                      控制层 (Control)                         │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  ros2_control                                         │   │
│  │  ├── JointTrajectoryController                        │   │
│  │  ├── 力控/力矩控制 (admittance control)                │   │
│  │  └── 硬件接口 (RobotHW)                               │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## 二、关键 ROS2 包和工具

### MoveIt2 核心
```bash
# 安装 MoveIt2
sudo apt install ros-${ROS_DISTRO}-moveit

# 生成 MoveIt 配置包
ros2 run moveit_setup_assistant moveit_setup_assistant

# 在代码中加载
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
```

### 笛卡尔路径规划
```python
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import Pose

def plan_cartesian_path(move_group, waypoints, eef_step=0.001):
    """
    笛卡尔空间直线/曲线运动规划
    eef_step: 末端执行器步长 (m) — 拉花建议 1mm
    """
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        eef_step=eef_step,
        jump_threshold=0.0,  # 禁用跳跃, 保证平滑
    )
    return plan, fraction
```

### 逆运动学选择
```python
# 推荐 Trac-IK (比默认 KDL 更快更稳定)
# 安装: sudo apt install ros-${ROS_DISTRO}-trac-ik
# 在 moveit_config/config/kinematics.yaml 中:
# kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
```

### ros2_control 配置
```yaml
# controller_manager 配置示例
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz, 拉花建议 100Hz 以上

joint_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
      - velocity      # 拉花需要速度控制
    state_interfaces:
      - position
      - velocity
    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.01
```

### 视觉检测包
```bash
# 液面高度检测
sudo apt install ros-${ROS_DISTRO}-cv-bridge
sudo apt install ros-${ROS_DISTRO}-image-pipeline

# 可选: AprilTag 用于杯子定位
sudo apt install ros-${ROS_DISTRO}-apriltag-ros
```

## 三、推荐的项目结构

```
latte_art_robot/
├── latte_art_description/       # 机械臂 URDF/XACRO 模型
│   ├── urdf/
│   │   └── robot.urdf.xacro
│   └── launch/
│       └── display.launch.py
├── latte_art_moveit_config/     # MoveIt2 配置包
│   └── config/
│       ├── kinematics.yaml
│       ├── joint_limits.yaml
│       └── controllers.yaml
├── latte_art_trajectory/        # 拉花轨迹生成包 (本项目的核心)
│   ├── __init__.py
│   ├── trajectory_generator.py  # 图案参数化 + 轨迹生成
│   ├── pattern_library.py       # 预设图案库
│   ├── anti_sloshing.py         # 抗晃荡约束
│   └── waypoint_publisher.py    # ROS2 节点: 发布轨迹
├── latte_art_perception/        # 视觉感知
│   ├── cup_detector.py          # 杯子/液面检测
│   └── milk_flow_monitor.py     # 奶泡质量实时监测
├── latte_art_control/           # 执行控制
│   ├── pouring_action_server.py # 拉花 Action Server
│   └── force_admittance.py      # 力控导纳控制
├── latte_art_bringup/           # 启动文件
│   └── launch/
│       ├── simulation.launch.py    # Gazebo 仿真
│       ├── real_robot.launch.py    # 真机
│       └── latte_art_action.launch.py
├── config/                      # 全局配置
│   └── latte_art_params.yaml
└── package.xml
```

## 四、ROS2 Action 接口设计

```python
# 自定义 Action: PourLatteArt.action

# Request
string pattern_type       # "heart", "rosetta", "tulip", "swan", "custom"
float64 cup_center_x
float64 cup_center_y
float64 cup_surface_z
float64 cup_radius
string custom_image_path   # 自定义图案图像 (仅 custom 类型)

# Result
bool success
float64 cartesian_fraction  # 路径规划成功率

# Feedback
float64 progress_percent    # 0-100
string current_stage         # "mixing", "drawing", "finishing"
```

## 五、仿真环境搭建

### Gazebo (Ignition) + ROS2
```bash
# 安装 Gazebo
sudo apt install ros-${ROS_DISTRO}-ros-gz

# 启动仿真
ros2 launch latte_art_bringup simulation.launch.py

# 发送拉花轨迹
ros2 action send_goal /pour_latte_art latte_art_msgs/action/PourLatteArt \
  "{pattern_type: 'rosetta', cup_center_x: 0.3, cup_center_y: 0.0, cup_surface_z: 0.15, cup_radius: 0.04}"
```

### CoppeliaSim (可选)
```bash
# CoppeliaSim 有更好的物理引擎 (液体模拟)
# 通过 ROS2 接口连接
# https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm
```

## 六、性能要求与硬件建议

| 组件 | 最低要求 | 推荐配置 |
|------|---------|---------|
| 机械臂 | 6-DOF, 重复定位精度 <0.5mm | 7-DOF 冗余臂 (如 xArm7, Franka) |
| 控制频率 | 100 Hz | 250-500 Hz |
| 相机 | 640x480@30Hz RGB | 1280x720@60Hz RGB-D |
| 计算平台 | Intel i5 + 8GB RAM | Intel i7 + 16GB RAM + GPU (如果跑VLA) |
| 力传感器 | 可选 | 6轴力矩传感器 (用于液面接触检测) |

## 七、关键指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 绝对定位精度 | < 1mm | 笛卡尔空间 |
| 轨迹跟踪误差 | < 0.5mm | 成形阶段 |
| 路径规划成功率 | > 95% | MoveIt2 CartesianPath |
| IK 求解时间 | < 5ms | 逆运动学 |
| 端到端周期 | 3-10s | 一杯拉花的总时间 |
| 拉花成功率 | > 80% | 图案清晰可辨识 |
