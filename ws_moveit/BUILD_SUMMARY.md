# ws_moveit 构建总结（Ubuntu 22.04）

本文档总结在 **仅使用 `ros_catkin_ws` 环境**（不依赖 `/opt/ros/noetic`）下编译 ws_moveit 的配置、已加入的源码依赖、代码修改及系统依赖。

---

## 1. 构建环境与命令

- **环境**：仅 source ros_catkin_ws 的 setup，不 source `/opt/ros/noetic`。
- **编译**：

```bash
source /home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash
cd /home/mu/IVG/ws_moveit
catkin_make_isolated --install
```

缺的依赖一律在 **ws_moveit/src** 中以源码形式加入，与本工作空间一起编译。

---

## 2. 已加入的源码包（ws_moveit/src）

| 包名 | 仓库 | 分支/说明 |
|------|------|-----------|
| eigen_stl_containers | ros/eigen_stl_containers | **master**（catkin） |
| random_numbers | ros-planning/random_numbers | **master** |
| object_recognition_msgs | wg-perception/object_recognition_msgs | 默认 |
| octomap_msgs | OctoMap/octomap_msgs | **melodic-devel** |
| graph_msgs | PickNikRobotics/graph_msgs | 默认 |
| geometry2 | ros/geometry2 | **noetic-devel**（含 tf2_eigen 等） |
| urdfdom_headers | ros/urdfdom_headers | 默认；需**先手动**安装到 install_isolated（见第 4 节） |
| warehouse_ros | ros-planning/warehouse_ros | 默认 |
| ros_control | ros-controls/ros_control | **noetic-devel**（含 controller_manager_msgs 等） |
| pybind11_catkin | 相应仓库 | 默认 |
| ruckig | picknikrobotics/ruckig | **v0.9.2**；需**先手动**安装到 install_isolated |
| eigenpy | stack-of-tasks/eigenpy | 默认；cmake 下含 jrl-cmakemodules 源码 |
| control_toolbox | ros-controls/control_toolbox | **noetic-devel** |
| realtime_tools | ros-controls/realtime_tools | **noetic-devel** |
| control_msgs | ros-controls/control_msgs | **kinetic-devel**（内层包在 src/control_msgs） |
| rosparam_shortcuts | PickNikRobotics/rosparam_shortcuts | **melodic-devel** |
| perception_pcl | ros-perception/perception_pcl | **melodic-devel**（含 pcl_conversions、pcl_ros） |
| **pcl_msgs** | ros-perception/pcl_msgs | **noetic-devel**（pcl_conversions 依赖） |

更多细节见 **BUILD_DEPENDENCIES.md**。

---

## 3. 代码与 CMake 修改

### 3.1 Boost 弃用头 / 占位符（Boost 推荐方式）

- **BOOST_ALLOW_DEPRECATED_HEADERS**  
  消除 `#pragma message: This header is deprecated. Use <iterator> instead.` 等提示。  
  依据：`/usr/include/boost/config/header_deprecated.hpp` 中，定义该宏后 `BOOST_HEADER_DEPRECATED(a)` 展开为空。

  已添加 **add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)** 的包：
  - **eigenpy**（target_compile_definitions，PRIVATE）
  - moveit_ros_planning_interface
  - warehouse_ros
  - pcl_ros
  - pcl_conversions（仅测试分支）
  - control_toolbox
  - geometry2/tf2、tf2_ros
  - random_numbers
  - moveit_setup_assistant
  - moveit_ros_visualization
  - moveit_ros_warehouse
  - moveit_ros_robot_interaction
  - srdfdom
  - rviz_visual_tools
  - moveit_visual_tools

- **BOOST_BIND_GLOBAL_PLACEHOLDERS**  
  在 **moveit_ros_planning_interface/CMakeLists.txt** 中已添加，用于消除 Boost.Bind 占位符弃用提示。

### 3.2 C++ 标准（log4cxx / std::shared_mutex）

- **pcl_ros**  
  `std::shared_mutex` 在 C++17 才进入标准库，系统 log4cxx 的 `boost-std-configuration.h` 会用到。  
  已将 **CMAKE_CXX_STANDARD** 从 14 改为 **17**（对应 `add_compile_options(-std=c++17)` 的 fallback）。

- **rosparam_shortcuts**  
  因 log4cxx 对 `std::shared_mutex` 的需求，已将 **add_compile_options(-std=c++14)** 改为 **-std=c++17**。

### 3.3 其他

- **moveit_ros_planning_interface/CMakeLists.txt**  
  在 `include_directories` 之后增加 **add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)**（见上）。

---

## 4. 需手动先安装到 install_isolated 的包

### 4.1 urdfdom_headers

```bash
cd /home/mu/IVG/ws_moveit/src/urdfdom_headers
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/mu/IVG/ws_moveit/install_isolated
make -j8 && make install
```

### 4.2 ruckig（v0.9.2）

在主分支需要 C++20，故使用 **v0.9.2**，并在其目录中按常规 CMake 流程配置安装到 **/home/mu/IVG/ws_moveit/install_isolated**。

---

## 5. 系统依赖（apt）

编译前建议安装以下开发包，避免缺少头文件或库：

```bash
sudo apt-get update
sudo apt-get install -y \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  liblog4cxx-dev \
  libbz2-dev
```

- **liburdfdom-dev / liburdfdom-headers-dev**：urdf、srdfdom 等依赖。
- **liblog4cxx-dev**：ROS 日志（rosconsole 等）。
- **libbz2-dev**：提供 **bzlib.h**，rosbag 的 `chunked_file.h` 会包含，pcl_ros 等用到 rosbag 的包编译时需要。

若仍缺其他包，可再按需安装，例如：

```bash
sudo apt-get install -y libpoco-dev libogre-1.9-dev freeglut3-dev
```

---

## 6. 常见编译错误与对应处理

| 现象 | 处理 |
|------|------|
| `Could NOT find pcl_msgs` | 在 **ws_moveit/src** 中克隆 **pcl_msgs**（noetic-devel），与本工作空间一起编译。 |
| `'shared_mutex' in namespace 'std' does not name a type` | 将 **pcl_ros**（及 rosparam_shortcuts）的 C++ 标准改为 **C++17**。 |
| `bzlib.h: 没有那个文件或目录` | 安装系统包：`sudo apt-get install libbz2-dev`。 |
| Boost 头文件弃用 `#pragma message` | 在对应包的 CMakeLists.txt 中增加 **add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)**。 |

---

## 7. 单包重编示例

仅重编 pcl_ros：

```bash
source /home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash
cd /home/mu/IVG/ws_moveit
catkin_make_isolated --install --pkg pcl_ros
```

其他包将 **--pkg pcl_ros** 换成对应包名即可。

---

*文档生成后可根据实际仓库路径与分支再微调表格；上述修改均已在 ws_moveit 源码中完成。*
