# MoveIt2 位姿验证工具使用说明

## 概述

`validate_moveit_poses.py` 是一个专门用于验证 MoveIt2 控制精度的工具，从 JSON 文件读取位姿数据并进行以下验证：

### 验证内容

1. **MoveIt2 机械臂运动到位判断**
   - 监测运动开始时刻
   - 监测运动结束时刻
   - 记录运动时间和稳定性
   - 分析运动状态变化历史

2. **MoveIt2 位姿精度对比**
   - 对比目标位姿（MoveIt2 期望）与实际位姿（从 `/demo_robot_status` 获取）
   - 计算位置差异（毫米级）
   - 计算姿态差异（度）
   - 判断是否在容差范围内

## 快速开始

### 基本用法

```bash
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
cd src/aubo_ros2_driver/aubo_demo/scripts
python3 validate_moveit_poses.py
```

### 使用自定义 JSON 文件

```bash
python3 validate_moveit_poses.py --ros-args \
  -p json_file:=/path/to/your/poses.json
```

### 使用自定义参数

```bash
python3 validate_moveit_poses.py --ros-args \
  -p json_file:=/path/to/poses.json \
  -p position_tolerance:=0.002 \
  -p orientation_tolerance:=0.02 \
  -p velocity_factor:=0.5 \
  -p motion_stable_count:=15
```

## JSON 文件格式

工具期望的 JSON 文件格式：

```json
{
  "version": "4.0",
  "recordedPoses": [
    {
      "position": {
        "x": 0.473,
        "y": -0.119,
        "z": 0.558
      },
      "orientation": {
        "x": -0.702,
        "y": -0.711,
        "z": -0.010,
        "w": 0.009
      }
    },
    ...更多位姿...
  ]
}
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `json_file` | string | auto_hand_eye_poses*.json | JSON 位姿文件路径 |
| `position_tolerance` | float | 0.001 | 位置容差（米），默认 1mm |
| `orientation_tolerance` | float | 0.01 | 姿态容差（弧度），默认约 0.57° |
| `velocity_factor` | float | 0.3 | 速度因子 (0.0-1.0) |
| `acceleration_factor` | float | 0.3 | 加速度因子 (0.0-1.0) |
| `motion_detection_timeout` | float | 5.0 | 检测运动开始的超时（秒） |
| `motion_stable_count` | int | 10 | 静止状态确认次数 |

## 工作流程

### 1. 初始化阶段

```
[INFO] 等待接收机器人状态...
[INFO] ✓ 已接收到机器人状态数据
[INFO] 等待 /move_to_pose 服务...
[INFO] ✓ 服务 /move_to_pose 已就绪
[INFO] 从文件加载位姿: /path/to/poses.json
[INFO] ✓ 成功加载 20 个位姿
```

### 2. 单个位姿验证流程

```
==================================================================
  测试位姿 #1
==================================================================

  目标位姿:
    位置: x=0.473362, y=-0.119853, z=0.558863
    姿态: w=0.009431, x=-0.702949, y=-0.711101, z=-0.010447

  移动前位姿:
    位置: x=0.450000, y=-0.100000, z=0.600000
    ...

📍 验证1: 调用 MoveIt2 移动到目标位姿...
[INFO] ✓ MoveIt2 返回成功: 轨迹规划和执行成功

🔍 验证1: 监测机械臂运动状态...
[1736328934.123] 🏃 机械臂开始运动
[INFO] ✓ 检测到运动开始 (耗时: 0.234s)
[1736328937.456] ⏸️  机械臂停止运动
[INFO] ✓ 机械臂已到位并稳定
  - 开始运动等待: 0.234s
  - 停止运动等待: 3.333s
  - 总等待时间: 3.567s
  - 静止确认次数: 10

📊 验证2: 对比目标位姿与实际位姿...

  实际到达位姿 (从 /demo_robot_status):
    位置: x=0.473380, y=-0.119845, z=0.558870
    姿态: w=0.009429, x=-0.702951, y=-0.711099, z=-0.010449

  位姿差异:
    位置差异: 0.023 mm
      Δx = 0.018 mm
      Δy = 0.008 mm
      Δz = 0.007 mm
    姿态差异: 0.012°

✅ 验证通过: 位姿在容差范围内
==================================================================
```

### 3. 测试报告生成

```
==================================================================
  📊 验证测试报告
==================================================================

总测试数: 5
通过: 5 ✅
未通过: 0 ⚠️
通过率: 100.0%

平均位置误差: 0.156 mm
平均姿态误差: 0.089°
最大位置误差: 0.423 mm
最大姿态误差: 0.234°
平均运动时间: 3.789 s

报告已保存到: moveit_validation_report_20260108_150234.json
==================================================================
```

## 验证细节

### 验证1: 运动到位判断

工具通过以下方式判断机械臂是否到位：

#### 1.1 运动开始检测

- 监听 `/demo_robot_status` 话题的 `in_motion` 字段
- 当 `in_motion` 从 `false` 变为 `true` 时，记录为运动开始
- 设有超时机制（默认 5 秒），如果超时未检测到运动：
  - 可能已在目标位置，进行静止状态确认
  - 或者报告运动检测失败

#### 1.2 运动结束检测

- 持续监听 `in_motion` 字段
- 当 `in_motion` 变为 `false` 时，开始静止确认
- 需要连续 N 次（默认 10 次）确认静止状态
- 额外等待 0.3 秒确保完全稳定

#### 1.3 运动状态日志

工具会实时记录运动状态变化：

```
[1736328934.123] 🏃 机械臂开始运动
[1736328937.456] ⏸️  机械臂停止运动
```

时间戳精确到毫秒，方便分析运动时序。

### 验证2: 位姿精度对比

#### 2.1 数据来源

- **目标位姿**：从 JSON 文件读取，发送给 MoveIt2
- **实际位姿**：从 `/demo_robot_status` 话题的 `cartesian_position` 字段获取

#### 2.2 差异计算

**位置差异：**
```
Δx = 实际位置.x - 目标位置.x
Δy = 实际位置.y - 目标位置.y
Δz = 实际位置.z - 目标位置.z
总距离 = √(Δx² + Δy² + Δz²)
```

**姿态差异（四元数方法）：**
```
点积 = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
旋转角度 = 2 * arccos(|点积|)
```

这个方法计算的是从一个姿态旋转到另一个姿态的最短旋转角度。

#### 2.3 容差判定

- **位置容差**：默认 0.001 m (1 mm)
  - `位置距离 ≤ 容差` → 位置通过 ✅
  - `位置距离 > 容差` → 位置不通过 ⚠️

- **姿态容差**：默认 0.01 rad (约 0.57°)
  - `旋转角度 ≤ 容差` → 姿态通过 ✅
  - `旋转角度 > 容差` → 姿态不通过 ⚠️

- **综合判定**：位置和姿态都通过才算验证通过

## 测试报告格式

生成的 JSON 报告包含详细信息：

```json
{
  "timestamp": "2026-01-08T15:02:34.567890",
  "summary": {
    "total_tests": 5,
    "passed": 5,
    "failed": 0,
    "pass_rate": "100.0%"
  },
  "validation_criteria": {
    "position_tolerance_mm": 1.0,
    "orientation_tolerance_deg": 0.573
  },
  "statistics": {
    "average_position_error_mm": 0.156,
    "average_orientation_error_deg": 0.089,
    "max_position_error_mm": 0.423,
    "max_orientation_error_deg": 0.234,
    "average_motion_time_s": 3.789
  },
  "test_results": [
    {
      "pose_index": 1,
      "timestamp": "2026-01-08T15:02:35.123456",
      "target_pose": {
        "position": {"x": 0.473, "y": -0.119, "z": 0.558},
        "orientation": {"x": -0.702, "y": -0.711, "z": -0.010, "w": 0.009}
      },
      "moveit_call": {
        "success": true,
        "message": "轨迹规划和执行成功",
        "execution_time": 0.234
      },
      "motion_detection": {
        "complete": true,
        "wait_time": 3.567,
        "details": {
          "motion_started": true,
          "already_at_target": false,
          "start_wait_time": 0.234,
          "stop_wait_time": 3.333,
          "stable_checks": 10
        }
      },
      "actual_pose": {
        "position": {"x": 0.473380, "y": -0.119845, "z": 0.558870},
        "orientation": {"x": -0.702951, "y": -0.711099, "z": -0.010449, "w": 0.009429}
      },
      "pose_difference": {
        "position_distance_mm": 0.023,
        "position_distance_m": 0.000023,
        "orientation_angle_deg": 0.012,
        "orientation_angle_rad": 0.000209,
        "dx": 0.000018,
        "dy": 0.000008,
        "dz": 0.000007
      },
      "position_ok": true,
      "orientation_ok": true,
      "success": true
    },
    ...更多测试结果...
  ]
}
```

## 典型应用场景

### 场景 1: 手眼标定位姿验证

验证手眼标定采集的位姿是否可达且精确：

```bash
python3 validate_moveit_poses.py --ros-args \
  -p json_file:=/path/to/hand_eye_calibration_poses.json \
  -p position_tolerance:=0.0005 \
  -p orientation_tolerance:=0.005
```

### 场景 2: 系统精度评估

评估整个机械臂控制系统的精度：

```bash
# 使用更严格的容差
python3 validate_moveit_poses.py --ros-args \
  -p position_tolerance:=0.0001 \
  -p orientation_tolerance:=0.001
```

### 场景 3: 运动时间分析

分析机械臂运动到位的时间特性：

```bash
# 增加静止确认次数，获得更稳定的到位判断
python3 validate_moveit_poses.py --ros-args \
  -p motion_stable_count:=20
```

### 场景 4: 高速运动测试

测试高速运动下的精度：

```bash
python3 validate_moveit_poses.py --ros-args \
  -p velocity_factor:=0.8 \
  -p acceleration_factor:=0.8
```

## 常见问题

### Q1: 提示 "未检测到运动"

**可能原因：**
1. 机械臂已经在目标位置附近
2. `motion_detection_timeout` 设置太短
3. `/demo_robot_status` 的 `in_motion` 字段更新不及时

**解决方案：**
```bash
# 增加运动检测超时
python3 validate_moveit_poses.py --ros-args \
  -p motion_detection_timeout:=10.0
```

### Q2: 位姿差异总是很大

**可能原因：**
1. 坐标系不一致（JSON 文件和 /demo_robot_status 使用不同坐标系）
2. 机械臂标定问题
3. 目标位姿不可达或在奇异点附近

**解决方案：**
1. 检查坐标系设置
2. 重新标定机械臂
3. 选择工作空间内的合理位姿

### Q3: 验证耗时过长

**可能原因：**
1. `motion_stable_count` 设置过大
2. 速度和加速度因子过小

**解决方案：**
```bash
# 减少静止确认次数，提高速度
python3 validate_moveit_poses.py --ros-args \
  -p motion_stable_count:=5 \
  -p velocity_factor:=0.5
```

### Q4: JSON 文件格式错误

**错误信息：**
```
[ERROR] ✗ JSON 文件格式错误: 缺少 recordedPoses 字段
```

**解决方案：**
确保 JSON 文件包含 `recordedPoses` 字段，格式如下：
```json
{
  "recordedPoses": [
    {
      "position": {"x": ..., "y": ..., "z": ...},
      "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}
    }
  ]
}
```

## 数据分析建议

### 1. 查看位置误差分布

检查报告中每个测试的 `position_distance_mm`，分析：
- 平均误差：反映系统整体精度
- 最大误差：反映最坏情况
- 误差分布：是否有异常值

### 2. 查看姿态误差分布

检查 `orientation_angle_deg`，特别关注：
- 是否存在姿态突变
- 某些方向的姿态误差是否特别大

### 3. 分析运动时间

检查 `motion_detection.wait_time`，分析：
- 不同位姿的运动时间差异
- 是否有异常长的运动时间
- 运动时间与位姿距离的关系

### 4. 运动状态分析

检查 `motion_detection.details`：
- `motion_started`: 是否检测到运动开始
- `already_at_target`: 是否已在目标位置
- `start_wait_time`: 运动启动延迟
- `stop_wait_time`: 实际运动时间

## 进阶用法

### 自定义测试流程

可以修改脚本来实现自定义测试流程，例如：

```python
# 在脚本末尾添加自定义分析
def custom_analysis(results):
    # 分析位置误差在各个轴上的分布
    x_errors = [r['pose_difference']['dx'] for r in results if 'pose_difference' in r]
    y_errors = [r['pose_difference']['dy'] for r in results if 'pose_difference' in r]
    z_errors = [r['pose_difference']['dz'] for r in results if 'pose_difference' in r]
    
    print(f"X轴平均误差: {sum(x_errors)/len(x_errors)*1000:.3f} mm")
    print(f"Y轴平均误差: {sum(y_errors)/len(y_errors)*1000:.3f} mm")
    print(f"Z轴平均误差: {sum(z_errors)/len(z_errors)*1000:.3f} mm")
```

### 与其他工具集成

```bash
# 记录测试过程
ros2 bag record /demo_robot_status /joint_states &
python3 validate_moveit_poses.py
```

## 总结

本工具提供了完整的 MoveIt2 控制验证功能：

✅ **验证1 完成**：
- 监测运动开始和结束
- 记录详细的运动时序信息
- 提供可靠的到位判断

✅ **验证2 完成**：
- 精确对比目标位姿与实际位姿
- 提供详细的误差分析
- 生成完整的测试报告

使用本工具可以系统性地评估 MoveIt2 控制系统的性能和精度。
