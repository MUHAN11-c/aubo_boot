# aubo_demo

调用 **`demo_driver`** 所提供服务与话题的 **C++ 示例可执行文件**（`package.xml` 描述与 `CMakeLists.txt` 一致）。另含 `scripts/` 下的 Python 辅助测试脚本。

---

## 目录结构

```
aubo_ros2_driver/aubo_demo/
├── CMakeLists.txt
├── package.xml
├── src/                              # C++ 源码（均安装到 lib/aubo_demo/）
│   ├── move_to_pose_client.cpp
│   ├── plan_trajectory_client.cpp
│   ├── execute_trajectory_client.cpp
│   ├── get_current_state_client.cpp
│   ├── get_current_pose_client.cpp
│   ├── get_current_pose_moveit2.cpp   # 直接通过 MoveIt2 / TF 取位姿
│   ├── set_speed_factor_client.cpp
│   ├── set_robot_enable_client.cpp
│   ├── set_robot_io_client.cpp
│   ├── read_robot_io_client.cpp
│   ├── robot_status_subscriber.cpp
│   ├── robot_io_status_subscriber.cpp
│   └── comprehensive_example.cpp
└── scripts/                          # 可选：位姿校验、批量测试、test_services.py 等
```

---

## 编译

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_demo
source install/setup.bash
```

---

## 运行前准备

需先启动 MoveIt / 机械臂与 **`demo_driver`** 服务，例如：

```bash
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
# 另一终端
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

服务名称与语义以 **`demo_interface`** 中 `.srv` 定义及 **`demo_driver/README.md`** 为准。

---

## 运行示例可执行文件

```bash
ros2 run aubo_demo move_to_pose_client
ros2 run aubo_demo plan_trajectory_client
ros2 run aubo_demo execute_trajectory_client
ros2 run aubo_demo get_current_state_client
ros2 run aubo_demo get_current_pose_client
ros2 run aubo_demo get_current_pose_moveit2
ros2 run aubo_demo set_speed_factor_client
ros2 run aubo_demo set_robot_enable_client
ros2 run aubo_demo set_robot_io_client
ros2 run aubo_demo read_robot_io_client
ros2 run aubo_demo robot_status_subscriber
ros2 run aubo_demo robot_io_status_subscriber
ros2 run aubo_demo comprehensive_example
```

---

## 服务接口摘要

以下仅为快速对照，字段以 `demo_interface` 中消息/服务为准。

| 能力 | 典型客户端可执行文件 |
|------|----------------------|
| 笛卡尔/关节空间移动到目标位姿 | `move_to_pose_client` |
| 规划轨迹 | `plan_trajectory_client` |
| 执行已有轨迹 | `execute_trajectory_client` |
| 查询关节与笛卡尔状态 | `get_current_state_client` |
| 速度缩放 | `set_speed_factor_client` |
| 使能 | `set_robot_enable_client` |
| IO 读写 | `set_robot_io_client`, `read_robot_io_client` |

---

## scripts/（Python）

用于开发联调，例如 `test_services.py`、`validate_moveit_poses.py`、`batch_pose_test.py` 等；运行前请在同一工作空间中 `source install/setup.bash`，并确保 `demo_driver` 已启动。

---

*接口变更时请同步更新 `demo_interface` 与 `demo_driver/README.md`。*
