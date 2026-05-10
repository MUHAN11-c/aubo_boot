# 编译注意事项

本文档记录了 IVG2.0 项目编译过程中遇到的问题及解决方案。

## 1. AUBO SDK 共享库链接

**问题**：`lib64/aubocontroller/` 和 `lib64/config/` 中的 `.so` symlink 被删除，链接器回退到不完整的静态库 `.a`，导致大量 undefined reference。

**根因**：`libauborobotcontroller.a` 缺少 `toolCoordinateCalibLib`、`setDhParaLen`、`ComputeFk_JYH`、`ComputeIkLib`、`userCoordinateCalibLib` 等内部函数实现。而 `libauborobotcontroller.so.1.3.1` 是完整的共享库，所有符号已内部解析。

**修复**：从 `.so.1.3.1` / `.so.11.0.2` 实体文件**复制**（非 symlink）为链接器期望的文件名：

| 目录 | 源文件 | 复制为 |
|------|--------|--------|
| `lib64/aubocontroller/` | `libauborobotcontroller.so.1.3.1` | `libauborobotcontroller.so` `.so.1` `.so.1.3` |
| `lib64/config/` | `libconfig.so.11.0.2` | `libconfig.so` `libconfig.so.11` |
| `lib64/config/` | `libconfig++.so.11.0.2` | `libconfig++.so` `libconfig++.so.11` |

**严禁使用 symlink**：复制给别人使用时 symlink 会断裂，必须用真实文件副本。

## 2. OMPL cmake 配置错误

**问题**：`moveit_ros_planning_interface` configure 时报错：
```
File or directory /usr/local/lib/x86_64-linux-gnu referenced by variable OMPL_LIBRARY_DIRS does not exist
```

**根因**：CMake `configure_package_config_file()` 模板生成的 `PACKAGE_PREFIX_DIR` 是公共变量名（无命名空间前缀），`omplConfig.cmake` 调用 `find_dependency(Eigen3 REQUIRED)` 后，`/usr/local/share/eigen3/cmake/Eigen3Config.cmake` 将该变量覆盖为 `/usr/local`，导致后续 `OMPL_LIBRARY_DIRS` 指向不存在的路径。

**修复**：`moveit_ros_planning_interface/CMakeLists.txt` line 108，添加：
```cmake
set(BUILD_TESTING OFF CACHE BOOL "Disabled to work around OMPL config bug" FORCE)
```
跳过触发该 bug 的 `ament_lint_auto_find_test_dependencies()` 调用。

## 3. RobotWebTools 构建

**问题**：`build_robotwebtools.sh` 检测到 `node_modules` 目录存在就直接复用，但目录内容不完整（缺 vite 等工具），导致构建失败。

**修复**：优化 `ensure_offline_dependencies()` 函数：
1. `node_modules` 存在时，从 `package.json` scripts 提取用到的工具名，检查 `.bin/` 是否存在
2. 缺失则自动清理并重新 `npm install`
3. 优先联网安装，失败则回退离线缓存

## 4. RViz2 工具快换加载失败

**问题**：工具快换时 RViz2 报错 `parameter name must not be empty`，导致 PlanningScene 模型重建失败，末端夹爪不显示。

**根因**：`planning_scene_display.cpp:345` `onRobotDescriptionTopic()` 回调中，`robot_description_property_->getStdString()` 在属性未初始化时返回空串，`rclcpp::Parameter("", data)` 抛异常。

**修复**：`planning_scene_display.cpp` 两处添加空值保护（line 357, line 563），当 `param_name` 为空时 fallback 为 `"robot_description"`。

**注意**：修复后需确保编译产物覆盖到 RViz2 实际加载的库路径。当前环境 RViz2 加载 `ws_moveit` 下的库，因此需要：
```bash
cp IVG2.0/aubo_ros2_ws/install/moveit_ros_visualization/lib/libmoveit_planning_scene_rviz_plugin_core.so.2.5.9 \
   ws_moveit/install/moveit_ros_visualization/lib/libmoveit_planning_scene_rviz_plugin_core.so.2.5.9
```

## 环境要求

- ROS 2 Humble (`/opt/ros/humble`)
- MoveIt 2 (编译安装于 `~/ws_moveit`)
- Node.js >= 20 + npm (用于 RobotWebTools)
- 编译前需 source: `source /opt/ros/humble/setup.bash`
