# GraspNet ROS2 测试指南

## 概述

本包提供了完整的测试功能，用于验证 GraspNet ROS2 包的安装和配置是否正确。

## 测试内容

测试节点会执行以下测试：

1. **路径查找测试**：验证能否找到 `graspnet-baseline` 目录
2. **模型加载测试**：验证能否加载 GraspNet 模型权重
3. **数据读取测试**：验证能否读取数据文件（color.png, depth.png, workspace_mask.png, meta.mat）
4. **抓取预测测试**（可选）：验证完整的抓取预测流程

## 使用方法

### 1. 编译包

```bash
cd ~/aubo_ros2_ws
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

### 2. 运行基本测试

```bash
# 运行所有基本测试（路径查找、模型加载、数据读取）
ros2 launch graspnet_ros2 test.launch.py
```

### 3. 运行完整测试（包括预测）

```bash
# 运行完整测试（需要 GPU）
ros2 launch graspnet_ros2 test.launch.py test_prediction:=true
```

### 4. 指定模型和数据路径

```bash
ros2 launch graspnet_ros2 test.launch.py \
    model_path:=/path/to/checkpoint-rs.tar \
    data_dir:=/path/to/data
```

### 5. 启用 RViz2 可视化

```bash
ros2 launch graspnet_ros2 test.launch.py use_rviz:=true
```

### 6. 直接运行测试节点

```bash
# 使用默认参数
ros2 run graspnet_ros2 graspnet_test_node

# 指定参数
ros2 run graspnet_ros2 graspnet_test_node --ros-args \
    -p test_model_load:=true \
    -p test_data_read:=true \
    -p test_prediction:=false \
    -p model_path:=/path/to/model.tar \
    -p data_dir:=/path/to/data
```

## 参数说明

### Launch 文件参数

- `test_model_load` (bool, default: true): 是否测试模型加载
- `test_data_read` (bool, default: true): 是否测试数据读取
- `test_prediction` (bool, default: false): 是否测试抓取预测
- `model_path` (string, default: ''): 模型权重文件路径（留空使用默认路径）
- `data_dir` (string, default: ''): 数据目录路径（留空使用默认路径）
- `use_rviz` (bool, default: false): 是否启动 RViz2
- `rviz_config` (string): RViz2 配置文件路径

## 测试输出示例

```
[graspnet_test_node]: ============================================================
[graspnet_test_node]: GraspNet ROS2 测试节点启动
[graspnet_test_node]: ============================================================

[graspnet_test_node]: [测试 1] 路径查找功能
[graspnet_test_node]:   ✓ 找到 baseline 目录: /path/to/graspnet-baseline
[graspnet_test_node]:   ✓ 目录存在
[graspnet_test_node]:   ✓ Python 路径设置成功
[graspnet_test_node]:   ✓ 模型路径: /path/to/logs/log_kn/checkpoint-rs.tar
[graspnet_test_node]:   ✓ 数据目录: /path/to/doc/pose_1

[graspnet_test_node]: [测试 2] 模型加载
[graspnet_test_node]:   模型路径: /path/to/logs/log_kn/checkpoint-rs.tar
[graspnet_test_node]:   ✓ 模型文件存在
[graspnet_test_node]:   使用设备: cuda:0
[graspnet_test_node]:   ✓ 模型创建成功
[graspnet_test_node]:   ✓ 模型权重加载成功

[graspnet_test_node]: [测试 3] 数据读取
[graspnet_test_node]:   数据目录: /path/to/doc/pose_1
[graspnet_test_node]:   ✓ 找到文件: color.png
[graspnet_test_node]:   ✓ 找到文件: depth.png
[graspnet_test_node]:   ✓ 找到文件: workspace_mask.png
[graspnet_test_node]:   ✓ 找到文件: meta.mat
[graspnet_test_node]:   ✓ 读取 color.png: (480, 640, 3)
[graspnet_test_node]:   ✓ 读取 depth.png: (480, 640)
[graspnet_test_node]:   ✓ 读取 workspace_mask.png: (480, 640)
[graspnet_test_node]:   ✓ 读取 meta.mat
[graspnet_test_node]:     intrinsic_matrix: (3, 3)
[graspnet_test_node]:     factor_depth: 1000.0

[graspnet_test_node]: ============================================================
[graspnet_test_node]: 测试结果汇总
[graspnet_test_node]: ============================================================
[graspnet_test_node]:   path_finding: ✓ 通过
[graspnet_test_node]:   model_load: ✓ 通过
[graspnet_test_node]:   data_read: ✓ 通过
[graspnet_test_node]: ------------------------------------------------------------
[graspnet_test_node]: 总计: 3 个测试
[graspnet_test_node]: 通过: 3 个
[graspnet_test_node]: 失败: 0 个
[graspnet_test_node]: ============================================================
[graspnet_test_node]: 🎉 所有测试通过！
```

## 常见问题

### 1. 找不到 baseline 目录

**错误信息**：
```
✗ 未找到 baseline 目录
```

**解决方法**：
- 确保已编译包：`colcon build --packages-select graspnet_ros2`
- 确保已 source：`source install/setup.bash`
- 检查 `graspnet-baseline` 目录是否存在
- 设置环境变量：`export GRASPNET_BASELINE_DIR=/path/to/graspnet-baseline`

### 2. 模型文件不存在

**错误信息**：
```
✗ 模型文件不存在: /path/to/checkpoint-rs.tar
```

**解决方法**：
- 检查模型文件是否存在
- 使用参数指定路径：`model_path:=/path/to/model.tar`
- 确保模型文件路径正确

### 3. 数据文件缺失

**错误信息**：
```
✗ 缺少文件: color.png
```

**解决方法**：
- 确保数据目录包含所有必需文件：
  - `color.png`
  - `depth.png`
  - `workspace_mask.png`
  - `meta.mat`
- 使用参数指定数据目录：`data_dir:=/path/to/data`

### 4. 导入错误

**错误信息**：
```
ImportError: No module named 'models.graspnet'
```

**解决方法**：
- 确保路径设置正确，测试节点会自动设置路径
- 检查 `graspnet-baseline` 目录结构是否完整
- 重新编译包

## 测试脚本示例

### 快速测试脚本

创建 `test_quick.sh`：

```bash
#!/bin/bash
cd ~/aubo_ros2_ws
source install/setup.bash
ros2 launch graspnet_ros2 test.launch.py
```

运行：
```bash
chmod +x test_quick.sh
./test_quick.sh
```

### 完整测试脚本

创建 `test_full.sh`：

```bash
#!/bin/bash
cd ~/aubo_ros2_ws
source install/setup.bash

echo "运行完整测试..."
ros2 launch graspnet_ros2 test.launch.py \
    test_model_load:=true \
    test_data_read:=true \
    test_prediction:=true \
    use_rviz:=false
```

## 集成到 CI/CD

测试节点返回退出码：
- `0`: 所有测试通过
- `1`: 至少一个测试失败

可以在 CI/CD 中使用：

```bash
ros2 run graspnet_ros2 graspnet_test_node
if [ $? -eq 0 ]; then
    echo "测试通过"
else
    echo "测试失败"
    exit 1
fi
```

## 下一步

测试通过后，可以：

1. 运行 Demo 节点：
   ```bash
   ros2 launch graspnet_ros2 graspnet_demo.launch.py
   ```

2. 运行实时节点：
   ```bash
   ros2 run graspnet_ros2 graspnet_node
   ```

3. 查看文档：
   - `README.md`: 包使用说明
   - `COMPILE_GUIDE.md`: 编译指南
