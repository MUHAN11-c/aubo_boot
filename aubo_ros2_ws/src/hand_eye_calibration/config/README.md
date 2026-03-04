# Config 目录说明

## 📁 位姿配置文件

### 📂 poses/ 目录（自动保存的位姿文件）

**用途**：前端自动保存的位姿配置文件存储目录  
**创建时间**：2026-01-12  
**保存方式**：前端点击"💾 保存"按钮时自动保存到服务器

#### 目录说明
- **路径**：`config/poses/`
- **文件命名格式**：`auto_hand_eye_poses_YYYYMMDD_HHMMSS.json`
- **文件格式**：v4.0格式（JSON）
- **内容**：包含记录的机器人位姿数据（recordedPoses）

#### 功能特点
- ✅ **自动保存**：前端保存位姿时自动保存到服务器
- ✅ **文件列表**：加载时可查看服务器上的所有位姿文件
- ✅ **双重选择**：支持从服务器列表选择或本地文件选择
- ✅ **向后兼容**：保留本地文件选择功能

#### 文件结构示例
```json
{
  "version": "4.0",
  "calibrationType": "eye-in-hand",
  "calibrationMethod": "pose-based-auto",
  "savedAt": "2026-01-12T14:15:08.201Z",
  "recordedPoses": [
    {
      "position": { "x": 0.468, "y": -0.074, "z": 0.518 },
      "orientation": { "x": 0.707, "y": 0.707, "z": 0, "w": 0 }
    },
    ...
  ]
}
```

#### 使用流程
1. **保存位姿文件**：
   - Web界面 → 自动手眼标定 → 记录位姿 → 点击"💾 保存"
   - 文件自动保存到 `config/poses/` 目录

2. **加载位姿文件**：
   - Web界面 → 自动手眼标定 → 点击"📂 加载"
   - 优先显示服务器文件列表，可选择：
     - 从服务器列表选择（显示文件信息）
     - 从本地文件选择（保留原有功能）

#### API接口
后端提供了3个API接口用于位姿文件管理：

| API | 方法 | 说明 |
|-----|------|------|
| `/api/poses/save` | POST | 保存位姿文件到服务器 |
| `/api/poses/list` | GET | 列出服务器上的位姿文件 |
| `/api/poses/load` | POST | 从服务器加载指定的位姿文件 |

---

### ⭐ auto_hand_eye_poses_optimized_10_v2.json（推荐）
- **用途**：10位姿优化配置
- **质量**：100/100（所有位姿）
- **特点**：
  - 避免深度相机盲区
  - 平均平移78.2mm
  - 预期误差<40mm
- **适用**：生产环境使用

### auto_hand_eye_poses_optimized.json
- **用途**：6位姿配置
- **质量**：95/100
- **特点**：
  - 前3个位姿质量优秀
  - 位姿5/6存在深度异常
  - 平均误差49mm
- **适用**：参考和对比

### auto_hand_eye_poses_1767839118323.json
- **用途**：历史成功配置（6位姿）
- **特点**：用户实际运行成功的配置
- **适用**：参考和备份

### auto_hand_eye_poses_generated.json
- **用途**：自动生成的位姿配置
- **适用**：开发测试

---

### 📂 calibration_results/ 目录（自动保存的标定结果文件）

**用途**：前端自动保存的手眼标定结果文件存储目录  
**创建时间**：2026-01-12  
**保存方式**：前端点击"💾 保存标定结果"按钮时自动保存到服务器

#### 目录说明
- **路径**：`config/calibration_results/`
- **文件命名格式**：`hand_eye_calibration_YYYYMMDD_HHMMSS.xml`
- **文件格式**：XML格式（OpenCV标准格式）
- **内容**：包含相机内参、手眼变换矩阵、标定精度评估等完整标定结果

#### 功能特点
- ✅ **自动保存**：前端保存标定结果时自动保存到服务器
- ✅ **双重保存**：同时保存到服务器和提供本地下载
- ✅ **向后兼容**：如果服务器保存失败，自动回退到仅本地下载

#### 文件内容示例
XML文件包含以下内容：
- 相机内参矩阵（CameraMatrix）
- 畸变系数（DistortionCoefficients）
- 手眼变换矩阵（TransformationMatrix）
- 标定精度评估（MeanCalibrationError、MaxCalibrationError等）
- 标定元数据（标定类型、方法、日期等）

#### 使用流程
1. **保存标定结果**：
   - Web界面 → 完成手眼标定 → 点击"💾 保存标定结果"
   - 文件自动保存到 `config/calibration_results/` 目录
   - 同时提供本地下载

---

## 📷 相机标定文件

### camera_calibration_accuracy.json
- **用途**：相机标定精度验证结果
- **更新**：每次在Web界面执行"计算结果"后自动更新
- **内容**：
  - 相邻角点距离误差
  - 对角线距离误差
  - 深度误差统计
  - 相机内参
- **使用**：`analyze_auto_calibration.py`自动读取

### calibrationdata/ost.yaml
- **用途**：相机内参文件
- **内容**：
  - 相机矩阵（fx, fy, cx, cy）
  - 畸变系数
  - 图像尺寸
- **当前值**：
  - fx = 466.174635
  - fy = 465.556589
  - cx = 299.134054
  - cy = 231.242129

---

## 🛠️ 其他配置

### hand_eye_calibration.xml
- **用途**：手眼标定结果
- **内容**：相机到机器人末端的变换矩阵
- **更新**：每次标定成功后保存

### cameraParams.xml
- **用途**：相机参数（旧格式）
- **适用**：兼容性保留

---

## 📊 使用流程

### 1. 相机标定精度验证

```
Web界面 → 相机标定精度验证 → 计算结果
↓
自动保存到 camera_calibration_accuracy.json
```

### 2. 位姿配置分析

```bash
python3 ../analyze_auto_calibration.py config/auto_hand_eye_poses_optimized_10_v2.json
```

自动读取 `camera_calibration_accuracy.json` 并分析。

### 3. 自动手眼标定

```
Web界面 → 自动手眼标定 → 加载位姿配置
↓
选择 auto_hand_eye_poses_optimized_10_v2.json
↓
开始自动标定
↓
成功后保存到 hand_eye_calibration.xml
```

---

## 🔄 文件更新时机

| 文件 | 更新时机 | 更新方式 |
|------|---------|---------|
| `camera_calibration_accuracy.json` | 点击"计算结果" | 自动 |
| `hand_eye_calibration.xml` | 标定成功（旧方式） | 自动 |
| `calibration_results/hand_eye_calibration_*.xml` | 前端保存标定结果 | 自动（前端保存） |
| `ost.yaml` | 相机标定 | 手动 |
| `poses/auto_hand_eye_poses_*.json` | 前端保存位姿 | 自动（前端保存） |
| `auto_hand_eye_poses_*.json` | 位姿设计 | 手动/工具 |

---

## 📚 相关文档

- **快速开始**：`../QUICK_START_10_POSES.md`
- **完整分析**：`../AUTO_CALIBRATION_OPTIMIZATION_REPORT.md`
- **精度集成**：`../CAMERA_ACCURACY_INTEGRATION.md`
- **快速参考**：`../QUICK_REFERENCE.md`

---

## ⚠️ 重要提示

1. **不要手动编辑**：
   - `camera_calibration_accuracy.json`（自动生成）
   - `hand_eye_calibration.xml`（自动生成）

2. **谨慎修改**：
   - `ost.yaml`（相机内参，影响所有标定）
   - `auto_hand_eye_poses_*.json`（位姿配置，建议使用工具）

3. **备份重要文件**：
   - 成功的 `hand_eye_calibration.xml`
   - 验证成功的 `auto_hand_eye_poses_*.json`
   - 当前的 `ost.yaml`

---

**目录版本**：v1.2  
**最后更新**：2026-01-12

---

## 📝 更新日志

### v1.2 (2026-01-12)
- ✨ **新增**：`calibration_results/` 目录用于存储前端自动保存的标定结果文件
- 🔄 **改进**：标定结果保存功能，自动保存到服务器（同时提供本地下载）
- 📝 **文档**：更新README，添加calibration_results目录说明

### v1.1 (2026-01-12)
- ✨ **新增**：`poses/` 目录用于存储前端自动保存的位姿文件
- ✨ **新增**：后端API接口（保存/列表/加载位姿文件）
- 🔄 **改进**：前端保存功能，默认保存到服务器（失败时回退到本地下载）
- 🔄 **改进**：前端加载功能，支持从服务器列表选择或本地文件选择
- 📝 **文档**：更新README，添加poses目录说明和API接口文档

### v1.0 (2026-01-08)
- ✨ 初始版本
