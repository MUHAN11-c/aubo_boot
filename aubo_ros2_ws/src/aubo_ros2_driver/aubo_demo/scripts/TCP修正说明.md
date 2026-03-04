# TCP 偏移修正说明

## 修改内容

已成功修改 URDF 和 SRDF 文件，添加 24mm 的 TCP 偏移以匹配手眼标定位姿。

### 修改文件

1. **URDF 文件**: `/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_description/urdf/xacro/inc/aubo_ros2.xacro`
   - 添加了 `tool_center_point` 链接
   - 添加了 `ee_to_tcp_joint` 关节，Z 轴向上偏移 24mm

2. **SRDF 文件**: `/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/config/aubo_i5.srdf`
   - 将 `manipulator` 组的 `tip_link` 改为 `tool_center_point`
   - 将 `endeffector` 组的链接改为 `tool_center_point`
   - 将末端执行器的 `parent_link` 改为 `tool_center_point`
   - 添加了 `tool_center_point` 的碰撞检测禁用规则

### 备份文件

原始文件已备份：
- `aubo_ros2.xacro.backup`
- `aubo_i5.srdf.backup`

## 重新编译工作空间

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select aubo_description aubo_moveit_config
source install/setup.bash
```

## 验证修改

### 1. 检查 URDF 是否正确

```bash
# 查看 URDF 中的链接
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5)"
```

在另一个终端查看 TF 树：
```bash
ros2 run tf2_tools view_frames
# 会生成 frames.pdf，检查是否有 tool_center_point 链接
```

### 2. 启动 MoveIt2 并检查

```bash
# 启动 MoveIt2
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py

# 在另一个终端检查末端执行器链接
ros2 param get /move_group move_group.end_effector_link
# 应该输出 tool_center_point
```

### 3. 重新运行验证测试

```bash
cd /home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_demo/scripts
python3 validate_moveit_poses.py --ros-args \
  -p json_file:=/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/auto_hand_eye_poses_1767855934821.json
```

**期望结果：**
- ✅ 位置误差应该从 ~24mm 降低到 < 1mm
- ✅ 姿态误差应该降低到 < 0.57°
- ✅ 通过率应该接近 100%

## 理解修改

### 修改前的坐标系层级

```
base_link
  └─ shoulder_Link
      └─ ... (其他关节)
          └─ wrist3_Link
              └─ ee_link (末端执行器)
```

MoveIt2 控制 `ee_link` 的位姿，但手眼标定的位姿是相对于实际 TCP（相机或工具中心）的。

### 修改后的坐标系层级

```
base_link
  └─ shoulder_Link
      └─ ... (其他关节)
          └─ wrist3_Link
              └─ ee_link
                  └─ tool_center_point (TCP，Z 轴向上偏移 24mm)
```

现在 MoveIt2 控制 `tool_center_point` 的位姿，与手眼标定的参考点一致。

## 可视化 TCP 偏移

在 RViz 中：
1. 添加 `TF` 显示
2. 查看 `ee_link` 和 `tool_center_point` 的关系
3. 应该看到 `tool_center_point` 在 `ee_link` 上方 24mm

## 如果需要调整偏移量

如果测试后发现偏移量不准确，可以修改 URDF 中的偏移值：

```xml
<!-- 在 aubo_ros2.xacro 中找到这一行 -->
<origin xyz="0.0 0.0 0.024" rpy="0.0 0.0 0.0" />
<!-- 调整 z 值，例如改为 0.025 表示 25mm -->
```

修改后重新编译：
```bash
colcon build --packages-select aubo_description
source install/setup.bash
```

## 如果要恢复原始配置

```bash
cd /home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_description/urdf/xacro/inc
cp aubo_ros2.xacro.backup aubo_ros2.xacro

cd /home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/config
cp aubo_i5.srdf.backup aubo_i5.srdf

cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select aubo_description aubo_moveit_config
source install/setup.bash
```

## 故障排查

### 问题 1: 编译错误

**症状：**
```
xacro: error: ...
```

**解决：**
检查 XML 语法，确保所有标签正确闭合。

### 问题 2: MoveIt2 启动失败

**症状：**
```
[ERROR]: Link 'tool_center_point' is not known to the URDF
```

**解决：**
```bash
# 检查 URDF 是否包含 tool_center_point
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5 | grep -A 5 "tool_center_point"
```

### 问题 3: 验证测试仍然失败

**可能原因：**
1. MoveIt2 未重新启动，仍使用旧的 URDF
2. 偏移量不准确（24mm 是估算值）
3. 还有其他坐标系问题

**解决：**
1. 完全重启所有 ROS 2 节点
2. 根据实际测试结果微调偏移量
3. 检查 TF 树确认所有变换正确

## 下一步

修改完成后，立即进行：

1. ✅ 重新编译工作空间
2. ✅ 重启 MoveIt2
3. ✅ 运行验证测试
4. ✅ 查看改进效果
5. 📝 记录最终偏移值
6. 📚 更新项目文档

## 技术细节

### TCP 偏移的物理意义

- **ee_link**: 机械臂末端法兰面的中心
- **tool_center_point**: 实际工具或相机的控制点
- **24mm 偏移**: 从法兰面到相机光学中心或工具尖端的距离

### 为什么是 Z 轴偏移

根据验证测试结果，所有位姿的 Z 轴都有约 -24mm 的系统性偏差：
- 实际 TCP 比期望位置低 24mm
- 说明 TCP 需要向上移动 24mm
- 因此在 URDF 中设置 `xyz="0.0 0.0 0.024"`

### 坐标系约定

- **Z 轴向上**: 标准机器人坐标系约定
- **正向偏移**: `+0.024` 表示沿 Z 轴向上 24mm
- **右手坐标系**: X 向前，Y 向左，Z 向上
