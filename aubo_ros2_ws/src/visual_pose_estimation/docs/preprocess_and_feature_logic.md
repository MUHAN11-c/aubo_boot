# 预处理与特征检测逻辑

本文档描述 `visual_pose_estimation_python` 中**预处理**（Preprocessor）和**特征检测**（FeatureExtractor）的当前逻辑与数据流。

---

## 一、整体数据流

```
深度图 + 彩色图
    ↓
Preprocessor.preprocess()
    ↓ 连通域列表 (filtered_components) + 预处理彩色图
FeatureExtractor.extract_features()
    ↓ ComponentFeature 列表
PoseEstimator (模板匹配 / 暴力匹配)
    ↓ 最佳模板 + 角度 + 抓取位姿
```

---

## 二、预处理（Preprocessor）

**文件**: `visual_pose_estimation_python/preprocessor.py`

### 2.1 主入口：`preprocess(depth_image, color_image, binary_threshold_min, binary_threshold_max)`

**输入**:
- `depth_image`: 深度图（uint16 或 float）
- `color_image`: 彩色图 BGR（可选）
- `binary_threshold_min` / `binary_threshold_max`: 深度二值化阈值（可选，否则用参数中的值）

**输出**:
- `filtered_components`: 筛选后的连通域掩码列表（每个元素为单连通域二值图）
- `preprocessed_color`: 用掩模抠出的彩色工件图（未提供彩色图时为 None）

### 2.2 处理步骤

| 步骤 | 方法 | 说明 |
|------|------|------|
| 1 | `process_depth_image()` | 深度图 0 值插值（补齐法）：行列中个别 0 点用邻域填充，整行/整列 0 不处理 |
| 2 | `create_binary_image()` | 二值化：深度在 `[min, max]` 内且非无效值(0/65535) 为 255，其余为 0 |
| 3 | `_extract_connected_components()` | 8 连通 `connectedComponentsWithStats`，每个标签生成一个掩码 |
| 4 | `_filter_components()` | 按面积、尺寸、长宽比筛选，并限制数量 |
| 5 | `extract_color_workpiece()` | 若有彩色图，用筛选后的连通域掩模合成一张彩色工件图 |

### 2.3 连通域筛选条件（`_filter_components`）

- **面积**: `component_min_area ≤ area ≤ component_max_area`
- **外接矩形宽高**: `w ≥ component_min_width`, `h ≥ component_min_height`
- **长宽比**: `min(w,h)/max(w,h)` 在 `[component_min_aspect_ratio, component_max_aspect_ratio]`
- **数量**: 最多保留 `component_max_count` 个，按面积从大到小取

### 2.4 主要参数（Preprocessor）

| 参数 | 含义 | 默认 |
|------|------|------|
| binary_threshold_min / max | 深度二值化范围 | 0, 65535 |
| enable_zero_interp | 是否做 0 值插值 | 1.0 |
| component_min/max_area | 连通域面积范围 | 1000, 1000000 |
| component_min/max_aspect_ratio | 长宽比范围 | 0.3, 4.0 |
| component_min_width / height | 最小宽高（像素） | 60, 60 |
| component_max_count | 最多保留连通域个数 | 10 |

---

## 三、特征检测（FeatureExtractor）

**文件**: `visual_pose_estimation_python/feature_extractor.py`

### 3.1 主入口：`extract_features(components, color_image)`

**输入**:
- `components`: 连通域掩码列表（来自 Preprocessor 的 filtered_components）
- `color_image`: 预处理彩色图（可选，用于特征内保存引用以便可视化）

**输出**:
- `List[ComponentFeature]`：每个连通域一个特征，包含工件圆、阀体圆、标准化角度等。

### 3.2 内部流程

1. **再次筛选**: `filter_components(components)`  
   使用与 Preprocessor 相同的面积、尺寸、长宽比参数再筛一遍（可剔除 Preprocessor 之后又变更参数的情况）。
2. **多线程**: 按 `max_threads` 和连通域数量开线程池。
3. **单连通域处理**: 对每个连通域调用 `_extract_single_component_feature(component_mask)`。

### 3.3 单连通域特征：`_extract_single_component_feature(component_mask)`

对**一个**连通域掩码依次做：

| 步骤 | 方法 | 说明 |
|------|------|------|
| 1 | `_extract_workpiece_circle(mask)` | **工件外接圆（大圆）**：findContours → 可选合并/过滤小轮廓 → minEnclosingCircle → 圆心、半径 |
| 2 | `_extract_valve_circle(mask)` | **阀体外接圆（小圆）**：腐蚀 → 取最大连通域 → 膨胀 → 外接圆 → 圆心、半径 |
| 3 | `_calculate_standardized_angle(wp_center, valve_center)` | **标准化角度**：atan2(valve - workpiece)，弧度/度 |
| 4 | 组装 | 填满 `ComponentFeature`（工件中心/半径/面积、阀体中心/半径/面积、角度、掩码等） |

### 3.4 工件外接圆（大圆）`_extract_workpiece_circle(mask, combine_contours, min_area)`

- `findContours` 取轮廓。
- 若 `combine_contours` 且轮廓数>1：先按面积≥min_area 过滤，再把剩余轮廓点合并为一个点集，再求外接圆。
- 否则：只保留面积≥min_area 的轮廓，取面积最大的轮廓求 `minEnclosingCircle`。
- 返回：`(圆心, 半径)`。

### 3.5 阀体外接圆（小圆）`_extract_valve_circle(mask)`

- **腐蚀**: 椭圆核 `small_circle_erode_kernel`、迭代 `small_circle_erode_iterations`，去掉边缘，保留中心（阀体）。
- 若腐蚀后为 0：自动降低迭代次数或核大小再试。
- **只保留最大连通域**: `small_circle_largest_cc` 为 True 时，对腐蚀结果做连通域，只保留面积最大的一个。
- **膨胀**: 椭圆核、迭代次数，恢复一定尺寸。
- 对膨胀结果做 `_extract_workpiece_circle(..., combine_contours=False, min_area=0)` 得到圆心和半径。
- 返回：`(圆心, 半径, 处理后的掩码)`。

### 3.6 ComponentFeature 结构

- `workpiece_center`, `workpiece_radius`, `workpiece_area`：工件外接圆与面积
- `valve_center`, `valve_radius`, `valve_area`：阀体外接圆与面积
- `standardized_angle`, `standardized_angle_deg`：工件中心→阀体中心的角度（弧度/度）
- `component_mask`：该连通域掩码（可选）
- `color_image`：彩色图引用（可选，用于 draw_features）

### 3.7 主要参数（FeatureExtractor）

| 参数 | 含义 | 默认 |
|------|------|------|
| component_min/max_area 等 | 与 Preprocessor 一致的连通域筛选 | 同左 |
| big_circle_combine_contours | 大圆是否合并多轮廓 | 1.0 |
| big_circle_min_area | 大圆轮廓最小面积 | 100 |
| small_circle_erode_kernel / iterations | 阀体腐蚀核与迭代 | 11, 1 |
| small_circle_dilate_kernel / iterations | 阀体膨胀核与迭代 | 9, 1 |
| small_circle_largest_cc | 阀体是否只保留最大连通域 | 1.0 |
| max_threads | 特征提取线程数 | 4 |

---

## 四、与姿态估计的衔接

- **预处理** 产出：`filtered_components`、`preprocessed_color`。
- **特征检测** 产出：`List[ComponentFeature]`，每个特征对应一个候选工件（工件圆+阀体圆+角度）。
- **PoseEstimator** 使用这些特征做：
  - 特征距离匹配 或
  - 暴力匹配（按角度旋转模板掩膜与目标掩膜做相似度），选出最佳模板与角度，再结合手眼标定得到抓取位姿。

参数来源：Debug 面板/配置文件通过 `config_reader`、`params_manager` 写入 Preprocessor 与 FeatureExtractor 的 `parameters`，二者共用连通域相关阈值（面积、长宽比、尺寸等），保证预处理与特征阶段条件一致。
