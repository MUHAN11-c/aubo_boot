# ros2_ws 桥接工作空间

使用 **aubo_ws**（ROS1 Noetic）和 **aubo_ros2_ws**（ROS2 Humble）编译 ROS1–ROS2 桥接。

## 环境约定

- **ROS Noetic**：`/home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash`
- **ROS Humble**：`/opt/ros/humble/setup.bash`
- **ROS1 链**：ros_catkin_ws → ws_moveit → aubo_ws
- **ROS2 链**：Humble → aubo_ros2_ws/install/setup.bash（若已编译）

## 一键编译

```bash
/home/mu/IVG/ros2_ws/build_bridge.sh
```

或手动执行：

```bash
# ROS1
source /home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash
source /home/mu/IVG/ws_moveit/install_isolated/setup.bash
source /home/mu/IVG/aubo_ws/devel_isolated/setup.bash

# ROS2
source /opt/ros/humble/setup.bash
source /home/mu/IVG/aubo_ros2_ws/install/setup.bash   # 若存在

# 编译
cd /home/mu/IVG/ros2_ws
colcon build --symlink-install
```

## 使用

**若提示 `Package 'ros1_bridge' not found`**：说明 `ros1_bridge` 尚未被编译进 `install/`。请先在一份**干净终端**里执行一键编译（见上一节），再 `source install/setup.bash` 并运行桥。

运行前注意：

- 先 **只** source ros2_ws 的 `install/setup.bash`（或配合 `use_ros2`），再启动桥，避免混用 Noetic/Humble 路径。
- 若出现 `ROS_DISTRO was set to 'noetic' before...`，说明当前 shell 曾 source 过 ROS1，建议开新终端或先 `unset ROS_DISTRO` 再 source ros2_ws。

```bash
# 仅 ROS2 环境（use_ros2 或下面两行二选一）
source /opt/ros/humble/setup.bash
source /home/mu/IVG/ros2_ws/install/setup.bash

# 启动动态桥（需先单独起 roscore 与 ROS2 daemon）
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test
```

## 已做修改（便于通过编译）

1. **ros1_bridge/src/bridge.cpp**  
   `RCLCPP_INFO(..., "..." + topic_name)` 改为 `RCLCPP_INFO(..., "%s", topic_name.c_str())`，满足格式字符串要求。

2. **ros1_bridge/resource/interface_factories.cpp.em**  
   将“定长数组”分支中的 `static_assert(... .size() ...)` 改为运行时 `assert(...)`，因 `.size()` 非常量表达式。

3. **build_bridge.sh**  
   新建脚本，按上述顺序 source 并执行 `colcon build --symlink-install`。
