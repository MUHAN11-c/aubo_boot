# 快速操作指南

## 5分钟快速上手

### 前置条件

1. ✅ 已标定相机内参（`data/cameraParams.xml`）
2. ✅ 机器人系统已校准
3. ✅ 相机安装在末端执行器上
4. ✅ 棋盘格固定在桌面

---

## 步骤1：数据采集（最重要！）

### 1.1 设置拍照姿态

```
操作机器人 → 相机垂直于桌面 → 保存姿态为 robot_shot_pose.json
```

### 1.2 采集数据（至少3个棋盘格姿态，每个5个角点）

对每个棋盘格姿态：

1. **放置棋盘格** → 固定在桌面
2. **拍照** → 机器人移动到拍照姿态 → 保存 `CornerCoordinates.csv`
3. **点选角点** → 用标志尖点接触5个角点 → 每次保存 `robot_status_*.json`（包含 `corner_index`）

**重要提示**：
- 点选时末端执行器必须垂直于桌面
- 标志尖点必须准确接触角点
- 每个JSON文件必须包含正确的 `corner_index`

---

## 步骤2：检查数据目录

确保目录结构如下：

```
data/
├── cameraParams.xml
├── robot_shot_pose.json
├── chessboard_pose_1/
│   ├── CornerCoordinates.csv
│   └── robot_status_*.json (至少5个)
├── chessboard_pose_2/
│   ├── CornerCoordinates.csv
│   └── robot_status_*.json (至少5个)
└── chessboard_pose_3/
    ├── CornerCoordinates.csv
    └── robot_status_*.json (至少5个)
```

---

## 步骤3：运行标定

```bash
cd /home/nvidia/RVG_ws/src/hand_eye_calibration_tool
python3 code/hand_eye_calibration.py
```

---

## 步骤4：查看结果

### 4.1 查看控制台输出

关注以下信息：
- ✅ 数据点数量（应该 ≥ 15）
- ✅ 每轮误差统计
- ✅ 最终最佳结果

### 4.2 查看误差评估文件

打开 `data/calibration_error_evaluation.csv`：
- 检查每行误差（应该 < 5mm）
- 找出误差大的点（可能需要重新采集）

### 4.3 查看标定结果

打开 `data/hand_eye_calibration.xml`：
- 查看变换矩阵
- 查看误差统计

---

## 判断结果是否可用

### ✅ 优秀结果
- 平均误差 < 2mm
- 最大误差 < 5mm
- 标准差 < 1mm

### ⚠️ 可接受结果
- 平均误差 < 5mm
- 最大误差 < 10mm
- 标准差 < 3mm

### ❌ 需要重新标定
- 平均误差 > 10mm
- 最大误差 > 20mm

---

## 常见错误处理

### 错误1：数据点不足

```
错误: 数据点不足，至少需要 3 组数据
```

**解决**：检查数据采集是否完整，确保每个棋盘格姿态至少有5个角点数据。

### 错误2：角点索引未找到

```
警告: 角点索引 XX 在CSV文件中未找到
```

**解决**：检查JSON文件中的 `corner_index` 是否与CSV文件中的索引匹配。

### 错误3：Z坐标不一致

```
警告: 文件中发现多个不同的Z值
```

**解决**：程序会自动统一Z值，但建议检查数据采集过程，确保同一姿态下Z值一致。

---

## 参数调整（可选）

如果标定结果不理想，可以调整 `code/hand_eye_calibration.py` 中的配置：

```python
CALIBRATION_CONFIG = {
    'num_rounds': 5,                      # 改为 7-10（更多轮）
    'error_threshold_multiplier': 2.0,     # 改为 1.5（更严格）
    'remove_percentage': 0.2,              # 改为 0.15（剔除更少）
}
```

---

## 下一步

标定完成后：
1. 使用 `hand_eye_calibration.xml` 中的变换矩阵
2. 在机器人控制程序中使用该矩阵进行坐标转换
3. 验证标定精度（在机器人上测试）

---

## 需要帮助？

查看详细文档：`README.md`

