# 位姿对比工具使用说明

本文档介绍如何使用位姿对比工具来验证 MoveIt2 控制机械臂的精度。

## 目录

- [功能概述](#功能概述)
- [前置要求](#前置要求)
- [工具说明](#工具说明)
- [使用方法](#使用方法)
- [示例](#示例)
- [参数配置](#参数配置)
- [故障排查](#故障排查)

## 功能概述

提供两个位姿对比工具：

1. **compare_pose.py** - 手动对比工具
   - 手动控制机械臂移动
   - 记录移动前后的位姿
   - 计算位姿差异

2. **compare_pose_auto.py** - 自动对比工具
   - 自动调用 MoveIt2 服务移动机械臂
   - 自动等待到位并记录位姿
   - 对比目标位姿与实际到达位姿
   - 可保存对比结果到 JSON 文件

## 前置要求

### 1. 系统要求

- ROS 2 (Humble/Foxy)
- MoveIt2
- Aubo 机械臂驱动
- Python 3.8+

### 2. 启动必要的服务

在运行位姿对比工具之前，需要启动以下服务：

```bash
# 终端 1: 启动机械臂驱动
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
# 根据实际情况启动驱动

# 终端 2: 启动 MoveIt2
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py
# 或其他适合的启动文件

# 终端 3: 启动机器人状态发布器
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
ros2 run demo_driver robot_status_publisher
```

### 3. 验证话题和服务

```bash
# 检查 /demo_robot_status 话题
ros2 topic echo /demo_robot_status

# 检查 /move_to_pose 服务 (仅 compare_pose_auto.py 需要)
ros2 service list | grep move_to_pose
```

## 工具说明

### compare_pose.py - 手动对比工具

**适用场景：**
- 测试机械臂移动前后的位姿变化
- 验证手动操作的精度
- 对比任意两个时刻的位姿

**功能特点：**
- 交互式操作，手动控制流程
- 等待用户确认每个步骤
- 实时监测机械臂运动状态
- 详细的位姿差异分析

### compare_pose_auto.py - 自动对比工具

**适用场景：**
- 测试 MoveIt2 控制精度
- 批量测试多个目标位姿
- 自动化测试和验证

**功能特点：**
- 自动调用 /move_to_pose 服务
- 自动等待机械臂到位
- 对比目标位姿与实际位姿
- 支持保存结果到 JSON 文件
- 可配置运动参数

## 使用方法

### 方法 1: 使用 compare_pose.py (手动对比)

```bash
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
cd src/aubo_ros2_driver/aubo_demo/scripts
python3 compare_pose.py
```

**操作步骤：**

1. 运行脚本后，按 Enter 记录初始位姿
2. 使用 MoveIt2 RViz 或其他方式移动机械臂
3. 等待机械臂到位后，按 Enter 记录目标位姿
4. 查看位姿差异分析结果
5. 可选：对比当前位姿与目标位姿

**示例输出：**

```
==================================================================
  🤖 位姿对比工具
==================================================================

此工具将对比以下两个位姿：
  1. 从 /demo_robot_status 主题获取的当前位姿
  2. MoveIt2 控制机械臂到达目标位置后的位姿

📍 准备记录【初始位姿】，按 Enter 继续...

==================================================================
  初始位姿 (从 /demo_robot_status)
==================================================================

📍 位置 (米):
    x:  -0.074018
    y:  -0.209054
    z:   0.953270

🔄 姿态 (四元数):
    w:   0.702572
    x:  -0.000147
    y:  -0.000800
    z:   0.711612

🎯 请使用 MoveIt2 或其他方式移动机械臂，完成后按 Enter 继续...

==================================================================
  📊 位姿差异分析
==================================================================

📍 位置差异 (米):
    Δx:   0.000123 m
    Δy:  -0.000456 m
    Δz:   0.000789 m
    总距离:   0.000923 m (0.923 mm)

🔄 姿态差异 (欧拉角):
    ΔRoll:    0.000012 rad (  0.687°)
    ΔPitch:  -0.000034 rad ( -1.948°)
    ΔYaw:     0.000056 rad (  3.209°)

✅ 位姿匹配：在容差范围内
```

### 方法 2: 使用 compare_pose_auto.py (自动对比)

```bash
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
cd src/aubo_ros2_driver/aubo_demo/scripts
python3 compare_pose_auto.py
```

**操作步骤：**

1. 运行脚本，等待初始化完成
2. 选择使用默认测试位姿或手动输入目标位姿
3. 选择规划方式（笛卡尔空间或关节空间）
4. 按 Enter 开始自动移动
5. 脚本自动等待机械臂到位
6. 查看位姿差异分析结果
7. 可选：保存结果到 JSON 文件

**示例输出：**

```
==================================================================
  🤖 自动位姿对比工具
==================================================================

此工具将自动：
  1. 使用 MoveIt2 控制机械臂移动到目标位姿
  2. 等待机械臂到位
  3. 从 /demo_robot_status 获取实际到达的位姿
  4. 对比目标位姿与实际位姿的差异

是否使用默认测试位姿? (y/n): y

==================================================================
  目标位姿 (期望到达)
==================================================================

📍 位置 (米):
    x:  -0.074000
    y:  -0.209000
    z:   0.953000

使用关节空间规划? (y/n, 默认为笛卡尔空间): n

按 Enter 开始移动机械臂...

[INFO] 发送移动请求到目标位姿...
[INFO] ✓ 移动成功: 轨迹执行完成
[INFO] 机械臂开始运动
[INFO] ✓ 机械臂已到位并稳定

==================================================================
  📊 位姿差异分析
==================================================================

📍 位置差异 (米):
    Δx:   0.000018 m
    Δy:  -0.000054 m
    Δz:   0.000270 m
    总距离:   0.000278 m (0.278 mm)

✅ 位姿匹配：在容差范围内

是否保存对比结果到文件? (y/n): y
[INFO] ✓ 结果已保存到: pose_comparison_20260108_143025.json
```

### 使用 ROS 2 run 命令

也可以使用 ros2 run 命令运行（需要在 package.xml 中配置）：

```bash
# 手动对比工具
ros2 run aubo_demo compare_pose.py

# 自动对比工具
ros2 run aubo_demo compare_pose_auto.py
```

## 参数配置

### compare_pose.py 参数

```bash
# 使用自定义参数
ros2 run aubo_demo compare_pose.py \
  --ros-args \
  -p position_tolerance:=0.002 \
  -p orientation_tolerance:=0.02
```

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `position_tolerance` | float | 0.001 | 位置容差 (米) |
| `orientation_tolerance` | float | 0.01 | 姿态容差 (弧度) |
| `planning_group` | string | manipulator_e5 | MoveIt2 规划组名称 |
| `base_frame` | string | base_link | 基础坐标系 |

### compare_pose_auto.py 参数

```bash
# 使用自定义参数
ros2 run aubo_demo compare_pose_auto.py \
  --ros-args \
  -p velocity_factor:=0.5 \
  -p acceleration_factor:=0.5 \
  -p position_tolerance:=0.002
```

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `position_tolerance` | float | 0.001 | 位置容差 (米) |
| `orientation_tolerance` | float | 0.01 | 姿态容差 (弧度) |
| `velocity_factor` | float | 0.3 | 速度因子 (0.0-1.0) |
| `acceleration_factor` | float | 0.3 | 加速度因子 (0.0-1.0) |

## 示例

### 示例 1: 快速测试单个位姿

```bash
# 使用默认参数快速测试
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
python3 src/aubo_ros2_driver/aubo_demo/scripts/compare_pose_auto.py
# 选择 'y' 使用默认测试位姿
# 选择 'n' 使用笛卡尔空间规划
# 按 Enter 开始
```

### 示例 2: 高精度测试

```bash
# 使用更严格的容差
ros2 run aubo_demo compare_pose_auto.py \
  --ros-args \
  -p position_tolerance:=0.0005 \
  -p orientation_tolerance:=0.005
```

### 示例 3: 批量测试多个位姿

创建一个 Python 脚本进行批量测试：

```python
#!/usr/bin/env python3
import rclpy
from compare_pose_auto import AutoPoseComparator, create_test_pose
from geometry_msgs.msg import Pose

def main():
    rclpy.init()
    comparator = AutoPoseComparator()
    
    # 定义多个测试位姿
    test_poses = [
        create_test_pose(),  # 位姿 1
        # 添加更多测试位姿...
    ]
    
    results = []
    for i, pose in enumerate(test_poses):
        print(f"\n测试位姿 {i+1}/{len(test_poses)}")
        if comparator.move_to_pose(pose):
            comparator.wait_for_motion_complete()
            actual_pose = comparator.get_current_pose_from_status()
            diff = comparator.compute_pose_difference(pose, actual_pose)
            results.append(diff)
    
    # 分析结果
    print("\n批量测试结果汇总:")
    for i, result in enumerate(results):
        print(f"位姿 {i+1}: 位置误差 {result['position']['distance']*1000:.3f} mm")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 位姿差异解读

### 位置差异

- **Δx, Δy, Δz**: 各轴方向的位置差异（米）
- **总距离**: 三维空间中的欧几里得距离（米或毫米）
- **典型精度**: 工业机械臂通常在 ±0.05mm 到 ±1mm 范围内

### 姿态差异

1. **欧拉角差异 (ΔRoll, ΔPitch, ΔYaw)**
   - 绕 X、Y、Z 轴的旋转差异
   - 单位：弧度或度
   - 注意：欧拉角存在万向锁问题

2. **四元数旋转角度**
   - 更稳定的姿态差异表示
   - 表示从一个姿态旋转到另一个姿态的最短旋转角度
   - 推荐用于姿态精度评估

### 容差判断

- **位置容差 (默认 0.001 m = 1 mm)**
  - 适用于大多数工业应用
  - 精密装配可能需要 0.1 mm 或更小

- **姿态容差 (默认 0.01 rad ≈ 0.57°)**
  - 适用于一般操作
  - 精密操作可能需要 0.1° 或更小

## 故障排查

### 问题 1: 无法接收机器人状态

**症状：**
```
[ERROR] ✗ 等待机器人状态超时
```

**解决方案：**
1. 检查 `/demo_robot_status` 话题是否发布：
   ```bash
   ros2 topic list | grep demo_robot_status
   ros2 topic echo /demo_robot_status
   ```

2. 确认 robot_status_publisher 节点正在运行：
   ```bash
   ros2 node list | grep robot_status
   ```

3. 如果话题名不同，修改脚本中的话题名：
   ```python
   # 在脚本中查找并修改
   self.robot_status_sub = self.create_subscription(
       RobotStatus,
       '/your_robot_status_topic',  # 修改为实际话题名
       ...
   )
   ```

### 问题 2: /move_to_pose 服务不可用

**症状：**
```
[ERROR] ✗ 服务 /move_to_pose 不可用
```

**解决方案：**
1. 检查服务是否存在：
   ```bash
   ros2 service list | grep move_to_pose
   ```

2. 确认相关节点正在运行：
   ```bash
   ros2 node list
   ```

3. 查看服务提供者的日志，检查是否有错误

### 问题 3: MoveIt2 初始化失败

**症状：**
```
[ERROR] MoveIt2 接口初始化失败
```

**解决方案：**
1. 确认 MoveIt2 已启动：
   ```bash
   ros2 node list | grep move_group
   ```

2. 检查规划组名称是否正确：
   ```bash
   ros2 param get /move_group move_group.planning_plugin
   ```

3. 查看 robot_description 参数：
   ```bash
   ros2 param list | grep robot_description
   ```

### 问题 4: 位姿差异过大

**症状：**
```
⚠️  位姿不匹配：超出容差范围
```

**可能原因：**
1. 机械臂标定问题
2. 碰撞检测导致路径偏差
3. 关节限位导致无法到达精确位置
4. 目标位姿在工作空间边缘

**解决方案：**
1. 重新标定机械臂
2. 检查并调整碰撞几何
3. 选择工作空间中心的测试位姿
4. 适当放宽容差阈值

### 问题 5: 等待机械臂到位超时

**症状：**
```
[ERROR] ✗ 等待机械臂到位超时
```

**解决方案：**
1. 增加超时时间（修改脚本中的 timeout 参数）
2. 检查机械臂是否真的在运动
3. 检查 `in_motion` 状态是否正确更新
4. 降低速度和加速度因子

## 输出文件格式

compare_pose_auto.py 可以保存结果为 JSON 格式：

```json
{
  "timestamp": "2026-01-08T14:30:25.123456",
  "target_pose": {
    "position": {"x": -0.074, "y": -0.209, "z": 0.953},
    "orientation": {"w": 0.7026, "x": -0.0001, "y": -0.0008, "z": 0.7116}
  },
  "actual_pose": {
    "position": {"x": -0.074018, "y": -0.209054, "z": 0.953270},
    "orientation": {"w": 0.702572, "x": -0.000147, "y": -0.000800, "z": 0.711612}
  },
  "difference": {
    "position_distance_mm": 0.278,
    "orientation_angle_deg": 0.342,
    "match": true
  }
}
```

## 进阶用法

### 与其他工具集成

这些脚本可以与其他 ROS 2 工具配合使用：

1. **与 rosbag 配合记录数据：**
   ```bash
   # 终端 1: 记录数据
   ros2 bag record /demo_robot_status /joint_states
   
   # 终端 2: 运行对比工具
   python3 compare_pose_auto.py
   ```

2. **与 rqt 配合可视化：**
   ```bash
   rqt_plot /demo_robot_status/cartesian_position/position/x:y:z
   ```

3. **与 Plotjuggler 配合分析：**
   ```bash
   ros2 run plotjuggler plotjuggler
   ```

### 自定义容差

根据应用场景调整容差：

| 应用场景 | 位置容差 | 姿态容差 |
|----------|----------|----------|
| 一般搬运 | 5 mm | 2° |
| 精密装配 | 0.1 mm | 0.1° |
| 打磨抛光 | 1 mm | 1° |
| 焊接 | 0.5 mm | 0.5° |

## 联系与支持

如有问题或建议，请联系开发团队或查看项目文档。

## 版本历史

- v1.0 (2026-01-08): 初始版本
  - 手动位姿对比工具
  - 自动位姿对比工具
  - 支持保存结果到 JSON
