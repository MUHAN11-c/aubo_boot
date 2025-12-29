# 眼在手上（Eye-in-Hand）手眼标定系统

## 目录
1. [系统概述](#系统概述)
2. [标定原理](#标定原理)
3. [数学公式](#数学公式)
4. [算法流程](#算法流程)
5. [多轮标定机制](#多轮标定机制)
6. [操作流程](#操作流程)
7. [数据格式说明](#数据格式说明)
8. [参数配置](#参数配置)
9. [输出结果](#输出结果)
10. [常见问题](#常见问题)

---

## 系统概述

本系统实现了一种**眼在手上（Eye-in-Hand）**的单目相机手眼标定方法。相机安装在机器人末端执行器上，通过采集多个棋盘格角点的数据，求解相机坐标系到末端执行器坐标系的变换矩阵（手眼标定矩阵）。

### 核心特点
- ✅ 支持多轮迭代标定，自动剔除异常点
- ✅ 使用非线性优化算法，精度高
- ✅ 自动选择最佳标定结果
- ✅ 详细的误差评估和可视化

---

## 标定原理

### 坐标系定义

系统涉及以下坐标系：

1. **机器人基座坐标系（Base）**：机器人基座固定坐标系
2. **末端执行器坐标系（Gripper/End-effector）**：安装在机器人末端
3. **相机坐标系（Camera）**：相机光心坐标系

### 标定目标

求解变换矩阵 **T_camera2gripper**，使得：

```
P_gripper = T_camera2gripper × P_camera
```

其中：
- `P_camera`：点在相机坐标系下的坐标
- `P_gripper`：点在末端执行器坐标系下的坐标
- `T_camera2gripper`：4×4齐次变换矩阵，包含旋转矩阵R（3×3）和平移向量t（3×1）

### 标定策略

本系统采用**固定棋盘格**的标定策略：

1. **拍照阶段**：
   - 机器人末端执行器移动到拍照姿态（相机垂直于桌面）
   - 相机拍摄固定在桌面上的棋盘格
   - 记录此时末端执行器的姿态 `T_gripper_shot2base`
   - 记录棋盘格角点在相机坐标系下的3D坐标 `P_camera_shot`

2. **点选阶段**：
   - 机器人末端执行器的标志尖点接触棋盘格角点
   - 记录此时末端执行器的姿态 `T_gripper_pick2base`
   - 标志点位置 `T_gripper_pick2base[:3, 3]` 即为角点在基座坐标系下的实际位置

3. **核心约束**：
   由于棋盘格固定在桌面，角点在基座坐标系下的位置是固定的，因此：

   ```
   P_base = T_gripper_shot2base × T_camera2gripper × P_camera_shot
   P_base = T_gripper_pick2base[:3, 3]  （标志点位置）
   ```

   两者应该相等！

---

## 数学公式

### 齐次变换矩阵

4×4齐次变换矩阵的形式：

```
T = [ R  t ]
    [ 0  1 ]
```

其中：
- `R`：3×3旋转矩阵（正交矩阵，R^T × R = I，det(R) = 1）
- `t`：3×1平移向量

### 坐标变换链

对于每个角点 i，完整的坐标变换链为：

```
P_camera_i → P_gripper_i → P_base_i
```

数学表达式：

```
P_base_i = T_gripper_shot2base × T_camera2gripper × P_camera_shot_i
```

展开为：

```
P_base_i = T_gripper_shot2base × [R_camera2gripper  t_camera2gripper] × [P_camera_shot_i]
                                  [0                1]              [1]
```

### 优化目标函数

对于 N 个角点数据，构建优化问题：

**目标**：最小化残差

```
minimize: Σ || P_base_computed_i - P_base_actual_i ||²
```

其中：
- `P_base_computed_i = T_gripper_shot2base × T_camera2gripper × P_camera_shot_i`
- `P_base_actual_i = T_gripper_pick2base_i[:3, 3]`

**约束**：
- 旋转矩阵 R 必须是正交矩阵（通过轴角表示法保证）
- 所有约束同时考虑 X、Y、Z 三个维度

### 参数化

使用**轴角表示法（Axis-Angle）**表示旋转：

- 旋转轴：单位向量 `n = [nx, ny, nz]`
- 旋转角：标量 `θ`
- 轴角向量：`axis_angle = θ × n`（3维向量）

优化参数：6维向量
- `[axis_angle_x, axis_angle_y, axis_angle_z, t_x, t_y, t_z]`

### 初始估计（SVD方法）

使用**Umeyama算法**（基于SVD）进行初始估计：

1. 将基座坐标系的点转换到末端执行器坐标系：
   ```
   P_gripper_i = T_gripper_shot2base^-1 × P_base_i
   ```

2. 求解两组点云之间的刚体变换：
   ```
   P_gripper_i = T_camera2gripper × P_camera_shot_i
   ```

3. 使用SVD分解协方差矩阵：
   ```
   H = Σ (P_gripper_i - centroid_gripper) × (P_camera_shot_i - centroid_camera)^T
   U, S, Vt = SVD(H)
   R_init = Vt^T × U^T
   t_init = centroid_gripper - R_init × centroid_camera
   ```

---

## 算法流程

### 单轮标定流程

```
1. 数据加载
   ├─ 加载相机内参（cameraParams.xml）
   ├─ 加载拍照姿态（robot_shot_pose.json）
   ├─ 加载点选姿态（robot_status_*.json）
   └─ 加载角点坐标（CornerCoordinates.csv）

2. 数据预处理
   ├─ 统一Z坐标值（处理数据不一致）
   ├─ 建立角点索引映射
   └─ 构建数据对（pick_pose, camera_point）

3. 初始估计
   ├─ 使用SVD方法估计初始旋转和平移
   └─ 转换为轴角表示

4. 非线性优化
   ├─ 构建残差函数 residuals(params)
   ├─ 使用 Levenberg-Marquardt 算法优化
   └─ 输出最优变换矩阵 T_camera2gripper

5. 误差评估
   ├─ 计算每个数据点的误差
   ├─ 统计平均误差、最大误差、标准差
   └─ 生成误差评估报告
```

### 优化算法细节

使用 `scipy.optimize.least_squares` 的 Levenberg-Marquardt 方法：

- **方法**：`method='lm'`（Levenberg-Marquardt）
- **最大迭代次数**：`max_nfev=10000`
- **收敛容差**：`ftol=1e-8, xtol=1e-8`

残差函数 `residuals(params)` 计算每个数据点的3D位置误差：

```python
for each data point i:
    P_base_computed = shot_pose × T_camera2gripper(params) × P_camera_shot_i
    P_base_actual = pick_pose_i[:3, 3]
    residual = P_base_computed - P_base_actual  # 3维向量
    residuals.extend(residual)  # 扩展到残差列表
```

---

## 多轮标定机制

### 设计理念

多轮标定通过迭代剔除异常点，逐步提高标定精度：

1. **第一轮**：使用所有数据点进行标定
2. **后续轮**：根据误差分析，剔除异常点后重新标定
3. **最佳选择**：所有轮完成后，选择平均误差最小的轮作为最终结果

### 剔除策略

每轮标定后，根据以下规则剔除异常点：

1. **误差阈值**：
   ```
   阈值 = 平均误差 + error_threshold_multiplier × 标准差
   ```
   剔除误差超过阈值的点

2. **比例限制**：
   ```
   最大剔除数 = max(阈值以上点数, 总点数 × remove_percentage)
   ```
   确保每轮最多剔除一定比例的点

3. **提前停止**：
   - 如果误差改善不足（`improvement < improvement_threshold`），提前停止
   - 如果剩余数据点不足（`点数 < min_points_per_round`），停止标定

### 多轮标定流程图

```
开始
 │
 ├─→ 第1轮：使用所有数据点（15个）
 │   ├─ 标定 → 评估误差
 │   └─ 保存结果
 │
 ├─→ 第2轮：剔除3个异常点（剩余12个）
 │   ├─ 标定 → 评估误差
 │   └─ 保存结果
 │
 ├─→ 第3轮：剔除2个异常点（剩余10个）
 │   ├─ 标定 → 评估误差
 │   └─ 保存结果
 │
 ├─→ ...（继续迭代）
 │
 └─→ 选择最佳结果（平均误差最小的轮）
```

---

## 操作流程

### 准备工作

1. **硬件准备**：
   - 机器人系统（已校准）
   - 单目相机（安装在末端执行器上）
   - 棋盘格（固定在桌面）
   - 末端执行器带标志尖点

2. **软件准备**：
   - 确保 `code/hand_eye_calibration.py` 文件存在
   - 确保数据目录结构正确（见[数据格式说明](#数据格式说明)）

### 数据采集步骤

#### 步骤1：相机内参标定

首先需要标定相机内参，保存为 `data/cameraParams.xml`（使用OpenCV格式）。

#### 步骤2：拍照姿态设置

1. 操作机器人，使相机**垂直于桌面**
2. 记录此时末端执行器的姿态，保存为 `data/robot_shot_pose.json`

#### 步骤3：采集棋盘格数据

对每个棋盘格姿态（至少3个不同姿态），执行以下操作：

1. **放置棋盘格**：
   - 将棋盘格放置在桌面上的固定位置
   - 确保棋盘格相对机器人基座固定不动

2. **拍照**：
   - 机器人移动到拍照姿态（步骤2中的姿态）
   - 相机拍摄棋盘格
   - 保存角点数据到 `chessboard_pose_X/CornerCoordinates.csv`

3. **点选角点**：
   - 对每个要标定的角点：
     - 操作机器人，使末端执行器的**标志尖点接触角点**
     - 确保末端执行器**垂直于桌面**（与棋盘格垂直）
     - 记录此时末端执行器的姿态，保存为 `robot_status_YYYY-MM-DD_HH-MM-SS.json`
     - 在JSON文件中记录 `corner_index`（对应CSV文件中的角点索引）
   - 建议每个棋盘格姿态点选5个角点

4. **重复**：
   - 改变棋盘格位置（至少3个不同姿态）
   - 重复步骤3.1-3.3

### 数据目录结构

```
data/
├── cameraParams.xml                    # 相机内参
├── robot_shot_pose.json                # 拍照姿态
├── chessboard_pose_1/
│   ├── CornerCoordinates.csv           # 角点坐标（相机坐标系）
│   ├── robot_status_2025-11-03_23-26-38.json
│   ├── robot_status_2025-11-03_23-28-45.json
│   └── ...（更多点选姿态文件）
├── chessboard_pose_2/
│   ├── CornerCoordinates.csv
│   └── ...（点选姿态文件）
└── chessboard_pose_3/
    ├── CornerCoordinates.csv
    └── ...（点选姿态文件）
```

### 运行标定程序

```bash
cd /home/nvidia/RVG_ws/src/hand_eye_calibration_tool
python3 code/hand_eye_calibration.py
```

程序将自动：
1. 加载所有数据
2. 执行多轮标定
3. 保存每轮结果和最终结果
4. 输出误差统计

---

## 数据格式说明

### 1. 相机内参文件（cameraParams.xml）

OpenCV格式的XML文件，包含：
- `camera_matrix`：3×3相机内参矩阵
- `distortion_coefficients`：畸变系数
- `image_size`：图像尺寸（宽×高）

示例：
```xml
<opencv_storage>
  <camera_matrix type_id="opencv-matrix">
    <rows>3</rows>
    <cols>3</cols>
    <dt>d</dt>
    <data>fx 0 cx 0 fy cy 0 0 1</data>
  </camera_matrix>
  ...
</opencv_storage>
```

### 2. 拍照姿态文件（robot_shot_pose.json）

JSON格式，包含末端执行器在基座坐标系下的位置和姿态：

```json
{
  "cartesian_position": {
    "position": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  }
}
```

- `position`：位置（单位：mm）
- `orientation`：四元数（x, y, z, w）

### 3. 点选姿态文件（robot_status_*.json）

JSON格式，与拍照姿态格式相同，但额外包含 `corner_index`：

```json
{
  "cartesian_position": {
    "position": { "x": ..., "y": ..., "z": ... },
    "orientation": { "x": ..., "y": ..., "z": ..., "w": ... }
  },
  "corner_index": 12
}
```

- `corner_index`：对应的角点索引（在CSV文件中查找）

### 4. 角点坐标文件（CornerCoordinates.csv）

CSV格式，包含角点在相机坐标系下的坐标：

```csv
corner_index,u,v,x_cam,y_cam,z_cam
12,123.45,234.56,-19.846,104.271,720.003
77,345.67,456.78,46.993,42.103,720.003
...
```

- `corner_index`：角点索引
- `u, v`：像素坐标
- `x_cam, y_cam, z_cam`：相机坐标系下的3D坐标（单位：mm）

**注意**：同一棋盘格姿态下，所有角点的 `z_cam` 应该相同（相机到棋盘格的距离）。

---

## 参数配置

在 `code/hand_eye_calibration.py` 文件开头修改配置：

```python
CALIBRATION_CONFIG = {
    'num_rounds': 5,                      # 标定轮数（建议3-10轮）
    'error_threshold_multiplier': 2.0,     # 误差阈值倍数
    'remove_percentage': 0.2,              # 每轮剔除的最大比例（20%）
    'min_points_per_round': 3,             # 每轮至少需要的数据点数
    'stop_if_no_improvement': True,        # 误差不再改善时提前停止
    'improvement_threshold': 0.01           # 误差改善阈值（1%）
}
```

### 参数说明

| 参数 | 说明 | 默认值 | 建议范围 |
|------|------|--------|----------|
| `num_rounds` | 标定轮数 | 5 | 3-10 |
| `error_threshold_multiplier` | 误差阈值倍数 | 2.0 | 1.5-3.0 |
| `remove_percentage` | 每轮剔除比例 | 0.2 | 0.1-0.3 |
| `min_points_per_round` | 每轮最少点数 | 3 | 3-5 |
| `stop_if_no_improvement` | 提前停止开关 | True | True/False |
| `improvement_threshold` | 改善阈值 | 0.01 | 0.005-0.02 |

### 参数调优建议

- **数据点较多（>20个）**：
  - `num_rounds = 7-10`
  - `remove_percentage = 0.15-0.2`

- **数据点较少（10-15个）**：
  - `num_rounds = 3-5`
  - `remove_percentage = 0.1-0.15`

- **误差较大**：
  - `error_threshold_multiplier = 1.5-2.0`（更严格）

- **误差较小但想进一步优化**：
  - `error_threshold_multiplier = 2.5-3.0`（更宽松）

---

## 输出结果

### 文件输出

程序会生成以下文件：

1. **每轮结果**：
   - `hand_eye_calibration_round1.xml` ~ `roundN.xml`
   - `calibration_error_evaluation_round1.csv` ~ `roundN.csv`

2. **最终结果**：
   - `hand_eye_calibration.xml`（最佳轮的结果）
   - `calibration_error_evaluation.csv`（最佳轮的结果）

### XML结果文件格式

```xml
<opencv_storage>
  <CalibrationType>Eye-in-Hand</CalibrationType>
  <TransformationMatrix type_id="opencv-matrix">
    <rows>4</rows>
    <cols>4</cols>
    <dt>d</dt>
    <data>R11 R12 R13 tx R21 R22 R23 ty R31 R32 R33 tz 0 0 0 1</data>
  </TransformationMatrix>
  <RotationMatrix>...</RotationMatrix>
  <TranslationVector unit="mm">...</TranslationVector>
  <MeanCalibrationError unit="mm">...</MeanCalibrationError>
  <MaxCalibrationError unit="mm">...</MaxCalibrationError>
  <MinCalibrationError unit="mm">...</MinCalibrationError>
  <StdCalibrationError unit="mm">...</StdCalibrationError>
  <DataPointCount>...</DataPointCount>
</opencv_storage>
```

### CSV误差评估文件格式

```csv
棋盘格姿态,末端json文件,角点索引,相机坐标系X(mm),相机坐标系Y(mm),相机坐标系Z(mm),实际X(mm),实际Y(mm),实际Z(mm),计算X(mm),计算Y(mm),计算Z(mm),实际与计算误差(mm)
chessboard_pose_1,robot_status_2025-11-03_23-26-38.json,12,-19.846,104.271,720.003,-9.645,439.544,-66.182,-10.812,439.470,-65.972,1.188
...
```

### 控制台输出

程序会在控制台输出：

1. **数据加载信息**：加载的数据点数量
2. **每轮标定信息**：
   - 使用的数据点数量
   - 变换矩阵
   - 误差统计
3. **最终对比**：所有轮的误差对比表

示例输出：
```
============================================================
所有轮标定结果对比
============================================================
轮次     数据点数       平均误差(mm)        最大误差(mm)        最小误差(mm)       
------------------------------------------------------------
1      15         16.948958       104.321918      2.130514       
2      12         1.131828        1.869832        0.503361       
3      10         0.967019        1.519395        0.307249       
4      8          0.861759        1.051302        0.592700       
5      7          0.797292        1.188205        0.525852        ← 最佳

总误差改善: 95.30% (从 16.948958 mm 降至 0.797292 mm)
```

---

## 常见问题

### Q1: 标定误差很大（>10mm）

**可能原因**：
1. 数据采集不准确（点选位置偏差大）
2. 拍照姿态不一致
3. 相机内参不准确
4. 棋盘格移动了

**解决方案**：
1. 检查数据采集过程，确保点选准确
2. 确保拍照姿态一致
3. 重新标定相机内参
4. 确保棋盘格固定不动

### Q2: 旋转矩阵接近单位矩阵

**可能原因**：
1. 初始估计不准确
2. 数据点分布不好（共线或共面）

**解决方案**：
1. 确保数据点分布在不同位置和姿态
2. 增加数据点数量
3. 检查SVD初始估计的输出

### Q3: 某些轮次的误差反而增大

**可能原因**：
1. 剔除的点过多，导致数据不足
2. 剩余点分布不好

**解决方案**：
1. 调整 `remove_percentage` 参数（减小）
2. 调整 `error_threshold_multiplier` 参数（增大）
3. 程序会自动选择最佳轮，无需担心

### Q4: 程序提前停止

**可能原因**：
1. 误差改善不足（`improvement < improvement_threshold`）
2. 数据点不足（`点数 < min_points_per_round`）

**解决方案**：
1. 这是正常现象，说明误差已经收敛
2. 如果数据点不足，检查数据采集是否完整

### Q5: CSV文件中的Z坐标不一致

**可能原因**：
1. 数据采集时相机距离不一致
2. 数据文件错误

**解决方案**：
1. 程序会自动检测并统一Z值
2. 检查警告信息，确认数据是否正确

### Q6: 如何判断标定结果是否可用？

**判断标准**：
1. **平均误差**：< 2mm（优秀），< 5mm（良好），< 10mm（可用）
2. **最大误差**：< 5mm（优秀），< 10mm（良好）
3. **标准差**：< 1mm（优秀），< 3mm（良好）

**建议**：
- 如果平均误差 > 10mm，建议重新采集数据
- 如果最大误差 > 20mm，检查是否有异常点

---

## 参考文献

1. **Umeyama, S.** (1991). Least-squares estimation of transformation parameters between two point patterns. IEEE Transactions on Pattern Analysis and Machine Intelligence, 13(4), 376-380.

2. **Tsai, R. Y., & Lenz, R. K.** (1989). A new technique for fully autonomous and efficient 3D robotics hand/eye calibration. IEEE Transactions on Robotics and Automation, 5(3), 345-358.

3. **Scipy Optimization Documentation**: https://docs.scipy.org/doc/scipy/reference/optimize.html

---

## 联系方式

如有问题或建议，请联系开发团队。

---

**最后更新**：2025-11-04

