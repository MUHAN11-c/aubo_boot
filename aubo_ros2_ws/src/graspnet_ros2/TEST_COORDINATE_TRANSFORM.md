# 坐标转换验证测试指南

## 测试目的

验证 GraspNet 到 ROS2 的坐标系转换是否正确，确保：
1. TF 坐标系定义正确
2. RViz Marker 与 TF 坐标轴方向一致
3. 机械臂能以正确姿态执行抓取

## 测试步骤

### 步骤 1: 启动系统

```bash
# 终端 1: 启动完整系统
cd /home/mu/IVG/aubo_ros2_ws
conda activate ros2_env
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py use_open3d:=true
```

### 步骤 2: 在 RViz2 中检查坐标轴

1. **查看 TF 坐标系**
   - 在 RViz2 左侧面板找到 "TF" 显示项
   - 确保显示所有坐标系的坐标轴

2. **检查 `grasp_pose_0` 坐标轴方向**（最重要！）
   ```
   预期方向：
   🔴 红色 X轴: 应指向手指张开方向（width）
   🟢 绿色 Y轴: 应垂直于抓取平面（height）
   🔵 蓝色 Z轴: 应指向手指伸出方向（approach）⬅️ 最关键！
   ```

3. **检查 Marker 夹爪可视化**
   - 夹爪的"手指"应该沿着 **蓝色 Z 轴**方向伸出
   - 如果不一致，说明坐标转换有问题

### 步骤 3: 验证 TF 变换数据

```bash
# 终端 2: 查看 TF 变换信息
ros2 run tf2_ros tf2_echo base_link grasp_pose_0

# 预期输出包含：
# - Translation: [x, y, z] 位置
# - Rotation: 四元数 [x, y, z, w]
```

### 步骤 4: 测试机械臂运动

```bash
# 终端 3: 触发抓取并控制机械臂
ros2 run graspnet_ros2 publish_grasps_client

# 观察输出：
# ============================================================
# 步骤 1: 调用发布服务
# ✓ 发布成功: 成功发布 1 个抓取和点云
# ============================================================
# 步骤 2: 获取抓取位姿
# ✓ 获取到抓取位姿:
#   位置 (position): x=..., y=..., z=...
#   姿态 (orientation): x=..., y=..., z=..., w=...
# ============================================================
# 步骤 3: 控制机械臂运动
# ✓ 运动控制成功: Successfully moved to target pose
```

## 验证要点

### ✅ 成功标志

1. **RViz2 可视化**
   - [ ] Marker 夹爪手指沿蓝色 Z 轴方向
   - [ ] TF 坐标轴与 Marker 姿态一致
   - [ ] 抓取点位置在物体表面

2. **机械臂运动**
   - [ ] 规划成功（Planning succeeded）
   - [ ] 执行成功（Successfully moved to target pose）
   - [ ] 末端执行器姿态正确（Z 轴指向抓取方向）

3. **坐标系一致性**
   - [ ] `grasp_pose_0` 的 Z 轴与手指伸出方向一致
   - [ ] 在 RViz2 中从多个角度观察，姿态合理

### ❌ 失败标志（需要调整）

1. **姿态错误**
   - ✗ 手指横着（应该竖着或沿 Z 轴）
   - ✗ 抓取角度旋转 90 度
   - ✗ TF 坐标轴与 Marker 不一致

2. **运动失败**
   - ✗ 规划失败（Planning failed）
   - ✗ IK 求解失败
   - ✗ 碰撞检测报警

## 调试技巧

### 如果姿态不对

1. **检查手眼标定**
   ```bash
   # 查看手眼标定 TF
   ros2 run tf2_ros tf2_echo wrist3_Link camera_frame
   ```

2. **检查坐标系链**
   ```bash
   # 生成 TF 树图
   ros2 run tf2_tools view_frames
   evince frames.pdf
   
   # 预期链：
   # base_link -> ... -> wrist3_Link -> camera_frame -> grasp_pose_0
   ```

3. **打印详细信息**
   - 修改 `publish_grasp_tf()` 的日志级别为 INFO
   - 查看转换前后的旋转矩阵

### 如果运动失败

1. **降低速度和加速度**
   ```bash
   ros2 run graspnet_ros2 publish_grasps_client \
     --ros-args \
     -p velocity_factor:=0.1 \
     -p acceleration_factor:=0.1
   ```

2. **检查工作空间**
   - 确认目标位置在机械臂可达范围内
   - 检查是否有碰撞

3. **使用 RViz MotionPlanning**
   - 手动设置目标位姿
   - 查看是否能规划路径

## 对比测试

### 修改前后对比

| 项目 | 修改前 | 修改后（预期） |
|------|--------|---------------|
| TF Z轴方向 | 垂直（height） | 手指伸出（approach） |
| Marker 一致性 | 不一致 | 一致 |
| 机械臂姿态 | 错误 | 正确 |
| 抓取成功率 | 低/失败 | 提高 |

## 记录测试结果

```
测试日期: _______________
测试人员: _______________

RViz2 检查:
[ ] TF 坐标轴显示正常
[ ] Marker 与 TF 一致
[ ] 抓取点位置合理

机械臂测试:
[ ] 规划成功
[ ] 执行成功  
[ ] 姿态正确

问题记录:
_________________________________
_________________________________
_________________________________
```

## 需要帮助？

如果测试失败，请提供：
1. RViz2 截图（显示 TF 和 Marker）
2. 终端输出日志
3. `ros2 run tf2_ros tf2_echo` 的输出
4. 具体的失败现象描述
