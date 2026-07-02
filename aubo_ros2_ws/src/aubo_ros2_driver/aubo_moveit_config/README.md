# aubo_moveit_config

AUBO 机械臂 **MoveIt2** 配置包：SRDF、控制器、规划器、RViz 配置与各类 launch（含与 `demo_driver` 联动）。

## 概述

本包负责 AUBO E5 机械臂在 MoveIt2 下的完整配置，包括运动学、碰撞检测、规划管线、控制器和 RViz 可视化喵~

## 关键文件

| 文件 | 用途 |
|------|------|
| `config/aubo_e5.srdf` | 碰撞矩阵 (ACM)、规划组、位姿定义 |
| `config/aubo_e5.urdf.xacro` | 机器人 URDF 模型 (含 5 种工具宏) |
| `config/kinematics.yaml` | KDL 运动学求解器参数 |
| `config/joint_limits.yaml` | 关节限位和速度/加速度限制 |
| `config/ros2_controllers.yaml` | ros2_control 控制器配置 (仿真) |
| `config/moveit_controllers.yaml` | MoveIt 控制器映射 |
| `config/moveit.rviz` | RViz2 默认布局 |
| `launch/aubo_new_driver.launch.py` | 主启动文件 (自动检测真机/仿真) |
| `scripts/limit_workspace.py` | 工作空间边界限制脚本 |

## 构建

```bash
cd ~/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_moveit_config
source install/setup.bash
```

## 运行

```bash
# 真机 (需机械臂在线)
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98

# 仿真 (自动回退，无需参数)
ros2 launch aubo_moveit_config aubo_new_driver.launch.py
```

## 依赖

- ROS 2 Humble: moveit_core, moveit_ros_move_group, moveit_ros_planning_interface
- ros2_control + controller_manager (仿真)
- aubo_description (URDF/XACRO/mesh)
- demo_driver (应用层服务)
- rviz2 (可视化)

## 详细文档

| 文档 | 内容 |
|------|------|
| `doc/TROUBLESHOOTING.md` | 故障排除：常见错误代码 (ERR001-007)、诊断流程图、调试技巧 |
| `doc/ROS2_CONTROL_INTEGRATION.md` | ros2_control 集成：控制器参数、spawner 脚本、关节名称对齐、TF 和 MoveIt 配置 |
| `doc/RVIZ_CONFIG.md` | RViz Motion Planning 插件：配置映射、启动文件解析、moveit.rviz 详细说明 |
| `doc/WORKSPACE_LIMIT.md` | 工作空间限制脚本：命令行参数、YAML 配置、边界墙启用/禁用 |
| `doc/TIMEOUT_ROOT_CAUSE.md` | MoveIt 超时根因分析 |
| `aubo_driver_ros2/doc/` | 驱动层文档：架构、SDK 参考、冲突规则 |
| `../demo_driver/README.md` | 应用层服务节点文档 |

## 其他文档 (aubo_description)

| 文档 | 内容 |
|------|------|
| `aubo_description/README.md` | 模型说明 |
| `aubo_description/urdf/xacro/inc/` | XACRO 宏定义 (含 5 种末端工具) |

## 参考

- [MoveIt 2 Humble 官方文档](https://moveit.picknik.ai/humble/index.html)
- [ros2_control 官方文档](https://control.ros.org/humble/index.html)
- CLAUDE.md §2 JointTrajectoryController, §5 CurrentStateMonitor, §8 工具切换碰撞
