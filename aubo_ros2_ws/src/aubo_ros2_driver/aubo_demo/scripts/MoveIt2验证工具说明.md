# MoveIt2 验证工具使用说明

## 功能说明

`validate_moveit_poses.py` 是一个专门用于验证 MoveIt2 控制精度的工具。

### 验证内容

✅ **验证1: MoveIt2 机械臂运动到位判断**
- 实时监测运动开始时刻
- 实时监测运动结束时刻
- 记录运动时间和稳定性
- 通过 `/demo_robot_status` 的 `in_motion` 字段判断

✅ **验证2: MoveIt2 位姿精度对比**
- 对比目标位姿与实际位姿
- 计算位置差异（毫米）
- 计算姿态差异（度）
- 自动判断是否在容差范围内

## 快速使用

### 方法 1: 使用快速启动脚本

```bash
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
cd src/aubo_ros2_driver/aubo_demo/scripts
python3 validate_moveit_poses.py --ros-args \
  -p json_file:=/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/auto_hand_eye_poses_1767855934821.json
./quick_test.sh
# 选择 "4) MoveIt2 验证工具"
```

### 方法 2: 直接运行

```bash
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
python3 src/aubo_ros2_driver/aubo_demo/scripts/validate_moveit_poses.py
```

### 方法 3: 指定 JSON 文件

```bash
python3 validate_moveit_poses.py --ros-args \
  -p json_file:=/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/auto_hand_eye_poses_1767855934821.json
```

## JSON 文件格式

工具会从 JSON 文件读取位姿数据，格式要求：

```json
{
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
    }
  ]
}
```

默认会使用手眼标定的位姿文件：
- `/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/auto_hand_eye_poses_*.json`

## 运行示例

### 完整运行流程

```
$ python3 validate_moveit_poses.py

==================================================================
  🤖 MoveIt2 位姿验证工具
==================================================================

此工具验证：
  1. MoveIt2 机械臂运动到位的判断方法
     - 监测运动开始
     - 监测运动结束
     - 记录运动时间

  2. MoveIt2 目标位姿与实际位姿的对比
     - 对比位置差异
     - 对比姿态差异
     - 判断是否在容差范围内

==================================================================
[INFO] 等待接收机器人状态...
[INFO] ✓ 已接收到机器人状态数据
[INFO] 等待 /move_to_pose 服务...
[INFO] ✓ 服务 /move_to_pose 已就绪
[INFO] 使用默认 JSON 文件: /home/mu/.../auto_hand_eye_poses_*.json
[INFO] ✓ 成功加载 20 个位姿

找到 20 个位姿
要测试的位姿数量 (输入数字，0=全部，默认=5): 3

将测试前 3 个位姿

按 Enter 开始验证测试...
```

### 单个位姿验证输出

```
==================================================================
  测试位姿 #1
==================================================================

  目标位姿:
    位置: x=0.473362, y=-0.119853, z=0.558863
    姿态: w=0.009431, x=-0.702949, y=-0.711101, z=-0.010447

  移动前位姿:
    位置: x=0.450000, y=-0.100000, z=0.600000
    姿态: w=0.010000, x=-0.700000, y=-0.710000, z=-0.010000

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

### 最终报告

```
==================================================================
  📊 验证测试报告
==================================================================

总测试数: 3
通过: 3 ✅
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

## 验证原理

### 验证1: 运动到位判断

**判断方法：**
1. 订阅 `/demo_robot_status` 主题
2. 监听 `in_motion` 字段的变化
3. 当 `in_motion` 从 `false` → `true`：运动开始 🏃
4. 当 `in_motion` 从 `true` → `false`：运动结束 ⏸️
5. 连续确认 10 次静止状态，确保稳定

**时序记录：**
- 精确记录运动开始和结束的时间戳
- 计算运动启动延迟、实际运动时间、总等待时间

### 验证2: 位姿精度对比

**数据来源：**
- **目标位姿**：从 JSON 文件读取，发送给 MoveIt2
- **实际位姿**：从 `/demo_robot_status` 的 `cartesian_position` 获取

**差异计算：**
- **位置差异**: √(Δx² + Δy² + Δz²)，单位：毫米
- **姿态差异**: 四元数旋转角度，单位：度

**容差判定：**
- 位置容差：默认 1 mm
- 姿态容差：默认 0.57°
- 两者都在容差内 → ✅ 验证通过
- 任一超出容差 → ⚠️ 验证未通过

## 自定义参数

### 调整容差

```bash
# 更严格的容差（精密应用）
python3 validate_moveit_poses.py --ros-args \
  -p position_tolerance:=0.0005 \
  -p orientation_tolerance:=0.005
```

### 调整运动参数

```bash
# 更快的运动速度
python3 validate_moveit_poses.py --ros-args \
  -p velocity_factor:=0.5 \
  -p acceleration_factor:=0.5
```

### 调整检测参数

```bash
# 更敏感的到位判断
python3 validate_moveit_poses.py --ros-args \
  -p motion_stable_count:=20 \
  -p motion_detection_timeout:=10.0
```

## 输出文件

测试完成后会生成 JSON 格式的详细报告：

**文件名格式：** `moveit_validation_report_YYYYMMDD_HHMMSS.json`

**报告内容：**
```json
{
  "timestamp": "2026-01-08T15:02:34.567890",
  "summary": {
    "total_tests": 3,
    "passed": 3,
    "failed": 0,
    "pass_rate": "100.0%"
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
      "target_pose": {...},
      "actual_pose": {...},
      "pose_difference": {...},
      "motion_detection": {...},
      "success": true
    },
    ...
  ]
}
```

## 常见问题

### Q: 提示 "未检测到运动"

**原因：** 机械臂可能已在目标位置附近

**解决：** 这是正常的，工具会自动确认静止状态

### Q: 位姿差异很大

**原因：** 
1. 坐标系不一致
2. 机械臂标定问题
3. 目标位姿不可达

**解决：**
1. 检查坐标系设置
2. 重新标定机械臂
3. 选择合理的测试位姿

### Q: 如何选择测试位姿数量？

**建议：**
- 快速验证：3-5 个位姿
- 完整评估：10-20 个位姿
- 全面测试：所有位姿

## 前置要求

运行工具前需要启动：

```bash
# 终端 1: 启动机械臂和 MoveIt2
# （根据实际情况）

# 终端 2: 启动机器人状态发布器
ros2 run demo_driver robot_status_publisher

# 终端 3: 运行验证工具
python3 validate_moveit_poses.py
```

## 应用场景

### 1. 手眼标定验证
验证手眼标定采集的位姿是否精确可达

### 2. 系统精度评估
评估 MoveIt2 控制系统的整体精度

### 3. 运动性能分析
分析机械臂运动到位的时间特性

### 4. 问题诊断
诊断位姿控制中的问题

## 更多信息

详细技术文档：`MOVEIT_VALIDATION_README.md`

## 总结

✅ 本工具完成了两项验证：

1. **验证运动到位判断** - 通过监测 `in_motion` 字段，精确判断机械臂何时到位
2. **验证位姿精度** - 对比目标位姿与实际位姿，量化评估控制精度

使用本工具可以系统地验证和评估 MoveIt2 控制系统的性能！
