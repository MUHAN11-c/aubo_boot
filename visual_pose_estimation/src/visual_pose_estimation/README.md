# Visual Pose Estimation

基于C++的ROS2姿态估计节点，提供核心算法实现。

## 项目结构

```
visual_pose_estimation/
├── CMakeLists.txt                # CMake构建配置
├── package.xml                   # ROS2包定义
├── src/                          # 源代码目录
│   ├── main.cpp                  # 主节点入口
│   ├── ros2_communication.cpp     # ROS2通信实现
│   └── config_reader.cpp         # 配置读取实现
├── include/                      # 头文件目录
│   └── visual_pose_estimation/  # 命名空间目录
│       ├── ros2_communication.hpp
│       └── config_reader.hpp
├── launch/                       # Launch文件
│   └── visual_pose_estimation.launch.py
└── configs/                      # 配置文件
    └── default.yaml
```

## 编译

```bash
cd ~/RVG_ws
colcon build --packages-select visual_pose_estimation
source install/setup.bash
```

## 运行

```bash
ros2 launch visual_pose_estimation visual_pose_estimation.launch.py
```

## 参数说明

- `config_file`: 配置文件路径（默认: `configs/default.yaml`）
- `calib_file`: 标定文件路径
- `template_root`: 模板库根目录（默认: `/home/nvidia/RVG_ws/templates`）
- `debug`: 是否启用调试模式（默认: `false`）

## 算法实现

当前版本为基础框架，算法实现将在后续版本中添加：
- 姿态估计算法
- 模板创建和管理
- 特征提取
- 姿态转换

## 依赖

- ROS2 (Humble/Iron)
- OpenCV
- yaml-cpp
- jsoncpp
- interface (ROS2接口包)

