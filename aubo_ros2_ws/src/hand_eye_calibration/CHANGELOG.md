# 手眼标定系统 - 变更日志

## [2026-01-07] 重投影误差修复与位姿到位优化

### 🎯 主要问题
- 手眼标定误差过大（60-125mm）
- 重投影误差异常高（300-900像素）
- 机械臂移动后立即采集图像，可能存在振动

### ✅ 已修复的问题

#### 1. 重投影误差修复（关键修复）
**问题描述**：
- 重投影误差高达 300-900 像素（正常应 < 1 像素）
- 导致标定板位姿计算不准确

**根本原因**：
- `cv2.projectPoints` 使用了错误的畸变系数配置
- 重投影误差计算时数组形状不匹配

**修复内容**：
```python
# hand_eye_calibration_node.py - 第799-802行
# 修复前：使用 self.calib_utils.dist_coeffs（固定值）
# 修复后：使用 dist_coeffs_to_use（与solvePnP保持一致）
projected_points, _ = cv2.projectPoints(
    obj_points, rvec, tvec,
    self.calib_utils.camera_matrix,
    dist_coeffs_to_use  # 修复：使用与solvePnP相同的畸变系数
)

# 第797行 - 修复数组形状问题
corners_reshaped = corners.reshape(-1, 2)
reprojection_errors = np.linalg.norm(corners_reshaped - projected_points, axis=1)
```

**修复结果**：
- 重投影误差从 **300-900 像素** → **0.2-0.6 像素** ✅
- 证明相机内参和畸变系数准确
- 棋盘格检测质量优秀

#### 2. 相机内参加载优化
**问题描述**：
- 原计划从 ROS2 `CameraInfo` 话题获取内参
- 但话题中畸变系数为零，导致错误

**修复内容**：
- 完全删除 `camera_info_callback` 函数
- 改为在初始化时直接从 `config/calibrationdata/ost.yaml` 加载
- 确保使用准确的相机标定参数

**文件修改**：
- `hand_eye_calibration_node.py`：
  - 删除 `camera_info_callback_DISABLED` 函数（原第2704-2841行）
  - 删除 `camera_info_subscription` 创建代码
  - 在 `__init__` 中直接加载 YAML 文件（第169-190行）

- `camera_calibration_utils.py`：
  - 新增 `load_camera_params_from_yaml` 方法
  - 正确解析 YAML 格式的标定文件

- `setup.py`：
  - 修复 `config/calibrationdata` 子目录安装配置
  ```python
  (os.path.join('share', package_name, 'config', 'calibrationdata'), 
   glob('config/calibrationdata/*.*')),
  ```

#### 3. 机械臂位姿到位优化（新增功能）
**问题描述**：
- 原逻辑：移动服务返回成功后立即采集图像
- 存在问题：机械臂可能还在振动或未完全停稳

**解决方案**：
实现智能等待机制，确保机械臂真正到位且稳定后再采集图像

**新增功能** - `_wait_for_robot_settled` 方法（第2650-2742行）：

**等待逻辑**：
1. 实时检查 `RobotStatus.in_motion` 状态
2. 计算当前位姿与目标位姿的误差
   - 位置误差：`sqrt((dx)² + (dy)² + (dz)²)`
   - 姿态误差：`2 * arccos(|q1·q2|)` （四元数角度差）
3. 验证误差在阈值内：
   - 位置容差：**1mm**
   - 姿态容差：**0.57°** (0.01 rad)
4. 确保稳定持续 **0.3秒**
5. 最大等待时间：**5秒**（超时保护）

**工作流程**：
```
机器人移动服务返回成功
    ↓
检查 in_motion = False
    ↓
验证位姿误差 < 阈值
    ↓
持续稳定 0.3 秒
    ↓
✅ 开始采集图像和提取角点
```

**日志输出示例**：
```
✅ 机器人移动服务返回成功: 位置(-0.389, -0.368, 0.524)
⏳ 等待机器人到位并稳定...
   位姿误差满足要求: 位置 0.25mm, 姿态 0.15°, 等待稳定 0.3秒...
   最终位姿: 位置误差 0.234mm, 姿态误差 0.143°
✅ 机器人已到位并稳定，可以采集图像
```

**文件修改**：
- `hand_eye_calibration_node.py`：
  - 第2619-2631行：修改 `move_to_pose` 响应处理，调用智能等待
  - 第2650-2742行：新增 `_wait_for_robot_settled` 方法

#### 4. 代码清理
**删除内容**：
- 删除所有调试日志代码（`# #region agent log` ... `# #endregion`）
- 删除冗余的深度图接收日志
- 移除 `corners_before_subpix` 临时变量（仅用于调试）

### 📊 修复效果对比

| 指标 | 修复前 | 修复后 | 状态 |
|------|--------|--------|------|
| 重投影误差（平均） | 300-600 像素 | 0.2-0.6 像素 | ✅ 完美 |
| 重投影误差（最大） | 900+ 像素 | < 1 像素 | ✅ 优秀 |
| 相机内参来源 | ROS2话题（畸变系数=0） | ost.yaml文件 | ✅ 准确 |
| 机械臂采集时机 | 移动后立即采集 | 到位稳定后采集 | ✅ 可靠 |
| 代码清洁度 | 包含大量调试日志 | 清洁无冗余 | ✅ 整洁 |

### 🔧 技术细节

#### 相机标定参数（来自 ost.yaml）
```yaml
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [462.966355, 0.0, 328.512101, 
         0.0, 461.16834, 250.752617, 
         0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.008263, -0.014846, 0.002199, 0.002100, 0.000000]
```

#### 棋盘格配置
- 尺寸：**9×6** （54个内角点）
- 方格大小：**15mm**
- 检测方法：`cv2.findChessboardCorners` + `cv2.cornerSubPix` 亚像素优化

### ⚠️ 已知问题

#### 手眼标定误差仍然偏大
- **当前误差**：平均 71.176mm，最大 125.026mm
- **期望误差**：< 5mm

**可能原因**：
1. 机器人位姿数据精度问题（特别是姿态部分）
   - 某些运动组显示：机器人旋转0.84°，但标定板变化9.86°（不匹配）
2. 标定姿态数量不足（当前7组，建议10-15组）
3. 运动范围和角度覆盖不够充分
4. 坐标系定义或单位转换可能存在问题

**建议**：
- 验证机器人正解算精度
- 增加标定姿态数量
- 确保运动覆盖不同的旋转轴和平移方向
- 检查机器人坐标系定义

### 📝 相关文件

**修改的文件**：
- `hand_eye_calibration/hand_eye_calibration_node.py`
  - 核心修复：重投影误差计算
  - 新增：智能位姿等待机制
  - 优化：相机内参加载逻辑
  
- `hand_eye_calibration/camera_calibration_utils.py`
  - 新增：`load_camera_params_from_yaml` 方法
  - 优化：角点检测代码
  
- `setup.py`
  - 修复：config子目录安装配置

**配置文件**：
- `config/calibrationdata/ost.yaml` - 相机内参标定文件

### 🚀 使用方法

1. **启动手眼标定系统**：
   ```bash
   cd /home/mu/IVG/aubo_ros2_ws
   source install/setup.bash
   ros2 launch hand_eye_calibration hand_eye_calibration_launch.py
   ```

2. **观察关键日志**：
   - `✅ 从文件加载相机内参` - 确认内参加载成功
   - `✅ 重投影误差良好：平均 X.XX 像素` - 验证标定板位姿质量
   - `✅ 机器人已到位并稳定` - 确认采集时机正确

3. **标定质量评估**：
   - 重投影误差应 < 1 像素
   - 机器人位姿误差应 < 1mm / 0.5°
   - 手眼标定误差目标 < 5mm

### 📚 参考文档

- OpenCV 相机标定文档：https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- 手眼标定原理：Tsai & Lenz, "A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration"
- ROS2 接口文档：`demo_interface/msg/RobotStatus.msg`

---
