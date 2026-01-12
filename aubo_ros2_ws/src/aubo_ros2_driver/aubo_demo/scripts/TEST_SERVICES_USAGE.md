# test_services.py 使用说明

## 概述

`test_services.py` 是一个用于测试 ROS2 demo_driver 服务的测试脚本，可以测试以下服务：
- `GetCurrentState` - 获取当前机器人状态
- `SetSpeedFactor` - 设置速度因子
- `PlanTrajectory` - 规划轨迹
- `ExecuteTrajectory` - 执行轨迹
- `MoveToPose` - 移动到目标位姿

## 使用方法

### 1. 基本用法

#### 运行所有测试
```bash
cd ~/IVG/aubo_ros2_ws
python3 test_services.py
```

#### 查看帮助信息
```bash
python3 test_services.py --help
```

#### 列出所有可用测试
```bash
python3 test_services.py --list
```

### 2. 运行单个测试

#### 测试获取当前状态
```bash
python3 test_services.py --test get_current_state
```

#### 测试设置速度因子
```bash
python3 test_services.py --test set_speed_factor --speed-factor 0.5
```

#### 测试规划轨迹（笛卡尔空间）
```bash
python3 test_services.py --test plan_trajectory
```

#### 测试规划轨迹（关节空间）
```bash
python3 test_services.py --test plan_trajectory --use-joints
```

#### 测试执行轨迹
```bash
# 会自动先规划轨迹，然后执行
python3 test_services.py --test execute_trajectory
```

#### 测试移动到目标位姿（笛卡尔空间）
```bash
python3 test_services.py --test move_to_pose --velocity-factor 0.3 --acceleration-factor 0.3
```

#### 测试移动到目标位姿（关节空间）
```bash
python3 test_services.py --test move_to_pose --use-joints --velocity-factor 0.3 --acceleration-factor 0.3
```

### 3. 参数说明

#### 主要参数

- `--test` / `-t`: 指定要运行的测试
  - 可选值: `get_current_state`, `set_speed_factor`, `plan_trajectory`, `execute_trajectory`, `move_to_pose`, `all`
  - 默认值: `all`

- `--use-joints` / `-j`: 使用关节空间规划（适用于 `plan_trajectory` 和 `move_to_pose`）
  - 默认: 使用笛卡尔空间规划

- `--velocity-factor` / `-v`: 速度缩放因子 (0.0-1.0)
  - 默认值: 0.3
  - 适用于: `move_to_pose`

- `--acceleration-factor` / `-a`: 加速度缩放因子 (0.0-1.0)
  - 默认值: 0.3
  - 适用于: `move_to_pose`

- `--speed-factor` / `-s`: 速度因子 (0.0-1.0)
  - 默认值: 0.5
  - 适用于: `set_speed_factor`

- `--list` / `-l`: 列出所有可用测试并退出

### 4. 使用示例

#### 示例 1: 完整测试流程
```bash
# 1. 获取当前状态
python3 test_services.py --test get_current_state

# 2. 设置速度因子
python3 test_services.py --test set_speed_factor --speed-factor 0.5

# 3. 规划轨迹
python3 test_services.py --test plan_trajectory

# 4. 执行轨迹（会自动先规划）
python3 test_services.py --test execute_trajectory

# 5. 移动到目标位姿
python3 test_services.py --test move_to_pose --velocity-factor 0.3
```

#### 示例 2: 使用关节空间规划
```bash
# 使用关节空间规划轨迹
python3 test_services.py --test plan_trajectory --use-joints

# 使用关节空间移动到目标位姿
python3 test_services.py --test move_to_pose --use-joints --velocity-factor 0.3
```

#### 示例 3: 调整速度参数
```bash
# 使用较低的速度和加速度
python3 test_services.py --test move_to_pose --velocity-factor 0.2 --acceleration-factor 0.2

# 使用较高的速度因子
python3 test_services.py --test set_speed_factor --speed-factor 0.8
```

### 5. 测试位姿

脚本使用的默认测试位姿：
- Position: x=-0.074, y=-0.209, z=0.953
- Orientation: quaternion (0.7026, -0.0001, -0.0008, 0.7116)

### 6. 注意事项

1. **运行前确保服务已启动**
   ```bash
   ros2 launch aubo_moveit_config demo_driver_services.launch.py
   ```

2. **执行轨迹测试会自动规划**
   - `execute_trajectory` 测试会自动先调用 `plan_trajectory` 获取轨迹

3. **测试超时时间**
   - 大部分服务调用超时时间为 10 秒
   - `plan_trajectory` 和 `execute_trajectory` 超时时间为 60 秒

4. **测试结果**
   - 成功: 显示 `✓` 和 `SUCCESS`
   - 失败: 显示 `✗` 和 `FAILED`，并显示错误信息

### 7. 输出示例

```
============================================================
Testing GetCurrentState service...
✓ GetCurrentState: SUCCESS
  Joint positions (rad): [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
  Cartesian position: x=-0.074018, y=-0.209054, z=0.953270
  Message: Successfully retrieved current state from MoveIt
============================================================
```

### 8. 故障排除

如果测试失败，检查：
1. 服务节点是否正在运行
2. MoveIt2 是否已正确初始化
3. 机器人描述参数是否正确加载
4. 网络连接是否正常

