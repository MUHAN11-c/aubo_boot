# GraspNet ROS2 包编译指南

## 文件架构说明

本包的结构符合 ROS2 Python 包标准：

```
graspnet_ros2/
├── graspnet_ros2/              # Python 模块目录
│   ├── __init__.py
│   ├── utils.py                # 工具函数（路径查找）
│   ├── image_saver.py          # 图像保存节点
│   ├── graspnet_node.py        # 实时抓取预测节点
│   └── graspnet_demo_node.py   # Demo 节点（从文件读取）
├── graspnet-baseline/          # GraspNet 基线代码（作为数据文件安装）
│   ├── models/                 # 模型定义
│   ├── utils/                 # 工具函数
│   ├── dataset/               # 数据集处理
│   ├── graspnetAPI/           # GraspNet API
│   └── ...
├── launch/                     # Launch 文件
│   └── graspnet_demo.launch.py
├── resource/                   # ROS2 资源文件
│   └── graspnet_ros2
├── test/                       # 测试文件
├── package.xml                 # ROS2 包配置
├── setup.py                    # Python 包配置
└── setup.cfg                   # Setuptools 配置
```

## 编译步骤

1. **确保依赖已安装**：
```bash
# ROS2 依赖
sudo apt-get install ros-<distro>-rclpy ros-<distro>-sensor-msgs \
  ros-<distro>-visualization-msgs ros-<distro>-geometry-msgs

# Python 依赖
pip install torch open3d scipy Pillow numpy
```

2. **编译包**：
```bash
cd ~/aubo_ros2_ws
colcon build --packages-select graspnet_ros2
```

3. **source 环境**：
```bash
source install/setup.bash
```

## 路径查找机制

包使用 `graspnet_ros2.utils` 模块来查找 `graspnet-baseline` 目录：

1. **开发环境**：从源码目录查找 `graspnet-baseline/`
2. **安装环境**：从 ROS2 share 目录查找 `share/graspnet_ros2/graspnet-baseline/`
3. **环境变量**：从 `GRASPNET_BASELINE_DIR` 环境变量查找

## 文件安装

`setup.py` 会自动收集并安装 `graspnet-baseline` 目录下的所有文件（排除 `.git`, `__pycache__` 等）。

安装后的文件位于：
```
install/graspnet_ros2/share/graspnet_ros2/graspnet-baseline/
```

## 验证安装

编译后验证文件是否正确安装：

```bash
# 检查文件是否安装
ls install/graspnet_ros2/share/graspnet_ros2/graspnet-baseline/

# 测试节点是否能找到路径
ros2 run graspnet_ros2 graspnet_demo_node --ros-args -p data_dir:=/path/to/data
```

## 常见问题

### 1. 找不到 graspnet-baseline 目录

**原因**：文件未正确安装或路径查找失败

**解决**：
- 检查 `graspnet-baseline` 目录是否存在
- 重新编译包：`colcon build --packages-select graspnet_ros2`
- 检查安装目录：`ls install/graspnet_ros2/share/graspnet_ros2/`

### 2. 导入错误

**原因**：Python 路径未正确设置

**解决**：
- 确保已 source：`source install/setup.bash`
- 检查 `graspnet_ros2.utils` 模块是否正常工作
- 设置环境变量：`export GRASPNET_BASELINE_DIR=/path/to/graspnet-baseline`

### 3. 模型文件未找到

**原因**：模型权重文件路径不正确

**解决**：
- 检查模型文件是否存在：`ls graspnet-baseline/logs/log_kn/checkpoint-rs.tar`
- 使用参数指定路径：`--ros-args -p model_path:=/path/to/model.tar`

## 开发模式

在开发模式下，可以直接修改源码，无需重新安装：

```bash
# 在源码目录中测试
cd ~/aubo_ros2_ws/src/graspnet_ros2
python3 -m graspnet_ros2.graspnet_demo_node
```

## 生产模式

在生产模式下，使用 ROS2 安装的版本：

```bash
# 编译并安装
colcon build --packages-select graspnet_ros2
source install/setup.bash

# 运行节点
ros2 run graspnet_ros2 graspnet_demo_node
```
