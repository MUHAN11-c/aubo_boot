# ws_moveit 源码依赖说明（Ubuntu 22.04）

在 `/home/mu/IVG/ws_moveit/src` 中已添加以下缺失依赖的**源码**，用于在 Ubuntu 22.04 下**仅用** `ros_catkin_ws` 环境（不依赖 `/opt/ros/noetic`）编译 ws_moveit。

**完整构建步骤、代码修改与常见错误处理见 [BUILD_SUMMARY.md](BUILD_SUMMARY.md)。**

## 已加入的源码包

| 包名 | 仓库 | 分支/说明 |
|------|------|-----------|
| eigen_stl_containers | https://github.com/ros/eigen_stl_containers | **master**（catkin，勿用 ros2） |
| random_numbers | https://github.com/ros-planning/random_numbers | **master**（catkin） |
| object_recognition_msgs | https://github.com/wg-perception/object_recognition_msgs | 默认（catkin） |
| octomap_msgs | https://github.com/OctoMap/octomap_msgs | **melodic-devel**（ROS1） |
| graph_msgs | https://github.com/PickNikRobotics/graph_msgs | 默认（catkin） |
| geometry2 | https://github.com/ros/geometry2 | **noetic-devel**（含 tf2_eigen 等） |
| urdfdom_headers | https://github.com/ros/urdfdom_headers | 默认；需**先手动**安装到 install_isolated（见下） |
| pcl_msgs | https://github.com/ros-perception/pcl_msgs | **noetic-devel**（pcl_conversions 依赖） |
| perception_pcl | https://github.com/ros-perception/perception_pcl | **melodic-devel**（含 pcl_conversions、pcl_ros） |
| 其他（warehouse_ros、ros_control、eigenpy、ruckig 等） | 见 BUILD_SUMMARY.md 第 2 节 | — |

## 编译步骤

1. **先安装 urdfdom_headers 到本工作空间**（仅首次或更新该包后需要）：
   ```bash
   cd /home/mu/IVG/ws_moveit/src/urdfdom_headers
   mkdir -p build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/home/mu/IVG/ws_moveit/install_isolated
   make -j8 && make install
   ```

2. **仅用 ros_catkin_ws 环境编译 ws_moveit**（不 source /opt/ros/noetic）：
   ```bash
   source /home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash
   cd /home/mu/IVG/ws_moveit
   catkin_make_isolated --install
   ```

## 当前剩余问题

- **srdfdom** 依赖 **urdf**（来自 ros_catkin_ws）。ros_catkin_ws 中的 urdf 在配置时使用了系统 urdfdom 的路径（如 `/usr/lib/x86_64-linux-gnu/urdfdom/...`），若本机未安装对应系统包，会报错「include dir not found」。
- **建议**：在 Ubuntu 22.04 上安装系统依赖后再编译：
  ```bash
  sudo apt-get update
  sudo apt-get install -y liburdfdom-dev liburdfdom-headers-dev
  ```
  若仍缺其他包（如 bzlib、log4cxx），可再安装：
  ```bash
  sudo apt-get install -y libbz2-dev liblog4cxx-dev
  ```
  更多系统依赖与错误处理见 **BUILD_SUMMARY.md**。

## 分支注意事项

- **eigen_stl_containers**、**random_numbers**：默认克隆为 ROS2 分支时，需切到 **master** 再编译。
- **octomap_msgs**：需使用 **melodic-devel**（ROS1 消息定义）。
