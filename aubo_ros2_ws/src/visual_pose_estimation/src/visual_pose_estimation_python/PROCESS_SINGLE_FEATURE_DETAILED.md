# `_process_single_feature` 方法详细说明文档

## 目录

1. [概述](#概述)
2. [基础概念](#基础概念)
3. [方法签名和参数](#方法签名和参数)
4. [完整执行流程](#完整执行流程)
5. [详细算法说明](#详细算法说明)
6. [数学公式详解](#数学公式详解)
7. [实际应用示例](#实际应用示例)
8. [常见问题](#常见问题)

---

## 概述

### 什么是 `_process_single_feature`？

`_process_single_feature` 是视觉姿态估计系统的核心方法，它的作用是将**图像中检测到的一个工件特征**转换为**机器人可以执行的3D抓取姿态**。

### 简单理解

想象你在工厂里看到一个工件，需要告诉机器人如何抓取它：

1. **你看到工件** → 图像中检测到特征
2. **你回忆类似的工件** → 在模板库中匹配
3. **你计算抓取位置** → 根据模板和当前工件的位置差异计算
4. **你告诉机器人** → 输出3D姿态（位置+方向）

`_process_single_feature` 就是自动完成这个过程的方法。

### 输入和输出

**输入：**
- 图像特征（工件中心、半径、角度等）
- 彩色图像和深度图像
- 工件ID（用于查找模板库）

**输出：**
- 3D抓取姿态（机器人基座坐标系下的位置和方向）
- 准备姿态、预放置姿态、放置姿态（如果有）
- 置信度（匹配质量评分）

---

## 基础概念

在深入细节之前，我们需要理解一些基础概念。

### 1. 坐标系

在机器人视觉系统中，有多个坐标系：

#### 1.1 图像坐标系（像素坐标系）

- **原点**：图像左上角 (0, 0)
- **X轴**：向右（列，u坐标）
- **Y轴**：向下（行，v坐标）
- **单位**：像素（pixel）

```
(0,0) ──────────→ u (列)
  │
  │
  │
  ↓
  v (行)
```

**示例：**
- 图像尺寸：640×480
- 工件中心在图像中的位置：(320, 240) 表示图像中心

#### 1.2 相机坐标系

- **原点**：相机光心
- **X轴**：向右
- **Y轴**：向下
- **Z轴**：向前（深度方向）
- **单位**：米（m）

```
相机坐标系：
      Z (深度)
      ↑
      │
      │
      └──→ X
     ╱
    ╱
   Y
```

**从像素坐标转换到相机坐标：**
```
x_camera = (u - cx) * z / fx
y_camera = (v - cy) * z / fy
z_camera = z (从深度图获取)
```

其中：
- `fx, fy`：相机焦距（像素单位）
- `cx, cy`：相机主点（图像中心，像素单位）
- `z`：深度值（米）

#### 1.3 机器人基座坐标系

- **原点**：机器人基座中心
- **X轴**：通常指向前方
- **Y轴**：通常指向左侧
- **Z轴**：通常指向上方
- **单位**：米（m）

这是机器人控制使用的坐标系，所有最终姿态都转换到这个坐标系。

### 2. 变换矩阵（齐次变换矩阵）

变换矩阵用于在不同坐标系之间转换点或姿态。

#### 2.1 什么是变换矩阵？

一个4×4的矩阵，包含：
- **旋转信息**（3×3旋转矩阵）
- **平移信息**（3×1平移向量）
- **齐次坐标**（最后一行固定为 [0, 0, 0, 1]）

```
T = [R11  R12  R13  tx]  旋转矩阵    平移向量
    [R21  R22  R23  ty]
    [R31  R32  R33  tz]
    [0    0    0    1 ]  齐次坐标
```

#### 2.2 变换矩阵的表示

- `T_A_B`：从坐标系B到坐标系A的变换
- 例如：`T_B_C` 表示从相机坐标系(C)到基座坐标系(B)的变换

#### 2.3 如何使用变换矩阵？

**转换点：**
```python
# 点在相机坐标系中
P_camera = [x, y, z, 1]  # 齐次坐标

# 转换到基座坐标系
P_base = T_B_C @ P_camera  # @ 表示矩阵乘法
```

**组合变换：**
```python
# 如果要从相机坐标系到末端执行器坐标系
T_B_E = T_B_C @ T_C_E
# 意思是：先转换到基座坐标系，再转换到末端坐标系
```

### 3. 特征（Feature）

特征是从图像中提取的工件信息：

```python
ComponentFeature {
    workpiece_center: (u, v)      # 工件中心（像素坐标）
    workpiece_radius: r            # 工件半径（像素）
    valve_center: (u, v)          # 阀体中心（像素坐标，可选）
    valve_radius: r               # 阀体半径（像素，可选）
    standardized_angle: θ          # 标准化角度（弧度）
    standardized_angle_deg: θ_deg  # 标准化角度（度）
    workpiece_area: area           # 工件面积（像素²）
}
```

**示例：**
```
工件中心：(320, 240)
工件半径：50像素
阀体中心：(320, 200)
阀体半径：10像素
标准化角度：45度
```

### 4. 模板（Template）

模板是预先保存的工件姿态信息，包含：
- 工件特征
- 抓取姿态（相机坐标系）
- 准备姿态（相机坐标系）
- 模板图像和掩膜

**模板目录结构：**
```
templates/
  └── {工件ID}/
      └── pose_1/
          ├── image.jpg              # 模板图像
          ├── mask.jpg               # 模板掩膜
          ├── template_info.json     # 特征信息
          ├── camera_pose.json       # 拍摄时的相机位姿
          ├── grab_position.json     # 抓取姿态
          └── preparation_position.json  # 准备姿态
```

### 5. 掩膜（Mask）

掩膜是二值图像，用于标识工件的区域：
- **255（白色）**：工件区域
- **0（黑色）**：背景区域

**用途：**
- 模板匹配时比较两个工件的形状
- 提取工件区域进行特征分析

---

## 方法签名和参数

### 完整方法签名

```python
def _process_single_feature(
    self,
    req_id: str,                    # 请求ID（用于日志追踪）
    idx: int,                       # 特征索引（第几个检测到的特征）
    feature: ComponentFeature,      # 特征对象
    features: List[ComponentFeature], # 所有特征列表
    components: List[np.ndarray],   # 连通域列表（二值掩膜）
    color_image: np.ndarray,        # 彩色图像（BGR格式）
    depth_image: np.ndarray,        # 深度图像（uint16格式）
    object_id: str                  # 工件ID（用于查找模板库）
) -> Optional[PoseEstimationResult] # 返回姿态估计结果，失败返回None
```

### 参数详细说明

#### `req_id: str`
- **作用**：请求的唯一标识符
- **用途**：在日志中追踪同一个请求的所有处理步骤
- **示例**：`"req_20240101_120000"`

#### `idx: int`
- **作用**：当前处理的特征在所有检测特征中的索引
- **用途**：区分多个检测到的工件
- **示例**：如果检测到3个工件，索引为 0, 1, 2

#### `feature: ComponentFeature`
- **作用**：当前要处理的工件特征
- **包含信息**：
  - 工件中心坐标（像素）
  - 工件半径（像素）
  - 阀体中心坐标（如果有）
  - 标准化角度
- **示例**：
  ```python
  feature.workpiece_center = (320, 240)
  feature.workpiece_radius = 50.0
  feature.standardized_angle_deg = 45.0
  ```

#### `components: List[np.ndarray]`
- **作用**：所有连通域的掩膜列表
- **格式**：每个元素是一个二值图像（0或255）
- **用途**：提取当前特征对应的掩膜用于模板匹配
- **示例**：
  ```python
  components[0]  # 第一个连通域的掩膜（形状：H×W）
  components[1]  # 第二个连通域的掩膜
  ```

#### `color_image: np.ndarray`
- **作用**：彩色图像
- **格式**：BGR格式，uint8，形状 (H, W, 3)
- **用途**：特征可视化、工件区域提取
- **示例**：640×480的彩色图像

#### `depth_image: np.ndarray`
- **作用**：深度图像
- **格式**：uint16，形状 (H, W)
- **单位**：原始深度值（需要乘以0.00025转换为米）
- **用途**：计算3D位置
- **示例**：640×480的深度图，每个像素值表示深度

#### `object_id: str`
- **作用**：工件ID
- **用途**：查找对应的模板库目录
- **示例**：`"3211242785"`

### 返回值

#### 成功时返回：`PoseEstimationResult`

```python
PoseEstimationResult {
    template_id: str                    # 使用的模板ID（如 "pose_1"）
    T_B_E_grasp: np.ndarray            # 抓取姿态（4×4矩阵，基座坐标系）
    T_B_E_prep: np.ndarray             # 准备姿态（4×4矩阵，基座坐标系）
    T_B_E_preplace: np.ndarray         # 预放置姿态（4×4矩阵）
    T_B_E_place: np.ndarray            # 放置姿态（4×4矩阵）
    confidence: float                   # 置信度（0.0-1.0）
    bounding_box: Tuple[int, int, int, int]  # 边界框 (x, y, w, h)
    target_center_camera: Tuple[float, float, float]  # 目标中心（相机坐标系）
}
```

#### 失败时返回：`None`

---

## 完整执行流程

### 流程图

```
开始
  ↓
[步骤1] 获取目标掩膜
  ├─ 从连通域列表提取掩膜
  └─ 调整掩膜尺寸匹配彩色图
  ↓
[步骤2] 模板匹配
  ├─ 模式A：特征距离匹配
  │   └─ 计算特征相似度
  └─ 模式B：暴力匹配（如果启用）
      ├─ 对齐中心
      ├─ 裁剪区域
      ├─ 缩放（可选）
      ├─ 角度搜索
      └─ 计算重叠度（IoU）
  ↓
[步骤3] 加载相机位姿
  ├─ 当前相机位姿（T_B_C）
  └─ 模板相机位姿（T_B_C_template）
  ↓
[步骤4] 计算角度差
  └─ 转换为弧度并归一化
  ↓
[步骤5] 姿态估计（核心算法）
  ├─ 获取模板抓取姿态深度
  ├─ 转换模板姿态到基座坐标系
  ├─ 计算模板工件中心（基座坐标系）
  ├─ 计算目标工件中心（基座坐标系）
  ├─ 计算相对偏移
  ├─ 应用旋转和平移
  └─ 生成最终抓取姿态
  ↓
[步骤6] 加载附加姿态
  ├─ 预放置姿态
  └─ 放置姿态
  ↓
[步骤7] 输出结果摘要
  └─ 记录日志
  ↓
结束（返回结果）
```

---

## 详细算法说明

### 步骤1：获取目标掩膜 (`_get_target_mask`)

#### 1.1 目的

从连通域列表中提取当前特征对应的二值掩膜，用于后续的模板匹配。

#### 1.2 详细过程

**步骤1.1：提取掩膜**
```python
# 检查索引是否有效
if idx < len(components):
    # 复制连通域掩膜（避免修改原始数据）
    target_mask = components[idx].copy()
else:
    # 索引超出范围，返回None
    return None
```

**步骤1.2：尺寸匹配**
```python
# 获取原始掩膜尺寸
original_shape = target_mask.shape[:2]  # (高度, 宽度)

# 检查是否需要调整尺寸
if target_mask.shape[:2] != color_image.shape[:2]:
    # 使用最近邻插值调整尺寸（保持二值特性）
    target_mask = cv2.resize(
        target_mask,
        (color_image.shape[1], color_image.shape[0]),  # (宽度, 高度)
        interpolation=cv2.INTER_NEAREST  # 最近邻，保持0和255的值
    )
```

**为什么需要尺寸匹配？**
- 模板匹配需要两个掩膜尺寸相同
- 彩色图用于可视化，掩膜需要与彩色图对齐

#### 1.3 输入输出

**输入：**
- `components[idx]`: 连通域掩膜，形状可能为 (H1, W1)
- `color_image`: 彩色图像，形状为 (H2, W2)

**输出：**
- `target_mask`: 调整后的掩膜，形状为 (H2, W2)，与彩色图尺寸一致

**示例：**
```python
# 输入
components[0].shape = (200, 200)      # 原始掩膜
color_image.shape = (480, 640, 3)     # 彩色图

# 输出
target_mask.shape = (480, 640)        # 调整后的掩膜
```

---

### 步骤2：模板匹配 (`_match_template`)

#### 2.1 目的

在模板库中找到与当前检测特征最相似的模板。

#### 2.2 两种匹配模式

##### 模式A：特征距离匹配（默认）

**原理：** 比较特征的几何属性（半径比、角度）

**详细过程：**

**步骤2.1：遍历所有模板**
```python
best_idx = -1
best_distance = float('inf')  # 初始化为无穷大

for idx, template in enumerate(templates):
    if template.feature is None:
        continue  # 跳过没有特征的模板
    
    # 计算特征距离
    distance = _calculate_feature_distance(feature, template.feature)
    
    # 更新最佳匹配
    if distance < best_distance:
        best_distance = distance
        best_idx = idx
```

**步骤2.2：特征距离计算**

特征距离公式：
```python
def _calculate_feature_distance(feature1, feature2):
    # 1. 计算半径比差
    ratio1 = feature1.valve_radius / feature1.workpiece_radius
    ratio2 = feature2.valve_radius / feature2.workpiece_radius
    radius_diff = abs(ratio1 - ratio2)
    
    # 2. 计算角度差（归一化到0-180度）
    angle_diff = abs(feature1.standardized_angle_deg - 
                     feature2.standardized_angle_deg)
    angle_diff = min(angle_diff, 360 - angle_diff)  # 处理周期性
    angle_diff_norm = angle_diff / 180.0  # 归一化到0-1
    
    # 3. 欧氏距离
    distance = sqrt(radius_diff² + angle_diff_norm²)
    
    return distance
```

**示例计算：**
```python
# 目标特征
feature1.workpiece_radius = 50
feature1.valve_radius = 10
feature1.standardized_angle_deg = 45

# 模板特征
feature2.workpiece_radius = 48
feature2.valve_radius = 9.5
feature2.standardized_angle_deg = 47

# 计算
ratio1 = 10/50 = 0.2
ratio2 = 9.5/48 ≈ 0.198
radius_diff = |0.2 - 0.198| = 0.002

angle_diff = |45 - 47| = 2度
angle_diff_norm = 2/180 = 0.011

distance = sqrt(0.002² + 0.011²) ≈ 0.011
```

**输出：**
- `best_idx`: 最佳模板索引
- `distance`: 特征距离（越小越好）
- `confidence`: None
- `best_angle_deg`: None
- `best_aligned_mask`: None

##### 模式B：暴力匹配（启用时）

**原理：** 通过旋转模板掩膜，找到与目标掩膜重叠度最高的角度。

**详细过程：**

**步骤2.1：预加载模板掩膜**
```python
template_mask_data = []

for template_idx, template in enumerate(templates):
    template_dir = Path(workpiece_template_dir) / template.id
    
    # 尝试加载掩膜文件
    for mask_name in ["mask.jpg", "standardized_mask.jpg"]:
        mask_path = template_dir / mask_name
        if mask_path.exists():
            template_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            template_mask_center = (template_mask.shape[1]/2, 
                                    template_mask.shape[0]/2)
            template_mask_data.append((template_idx, template, 
                                      template_mask, template_mask_center))
            break
```

**步骤2.2：对齐中心**

将模板掩膜的中心对齐到目标掩膜的中心。

```python
# 计算平移矩阵
translate_M = np.array([
    [1.0, 0.0, target_center[0] - template_mask_center[0]],
    [0.0, 1.0, target_center[1] - template_mask_center[1]]
], dtype=np.float32)

# 应用仿射变换
template_mask_aligned = cv2.warpAffine(
    template_mask,
    translate_M,
    (target_mask.shape[1], target_mask.shape[0]),
    flags=cv2.INTER_NEAREST,  # 最近邻插值
    borderMode=cv2.BORDER_CONSTANT,
    borderValue=0  # 背景填充为0（黑色）
)

# 二值化确保掩膜是0或255
_, template_mask_aligned = cv2.threshold(
    template_mask_aligned, 127, 255, cv2.THRESH_BINARY
)
```

**可视化说明：**
```
对齐前：
模板掩膜中心: (100, 100)
目标掩膜中心: (320, 240)
需要平移: (220, 140)

对齐后：
模板掩膜中心: (320, 240)  ← 与目标中心对齐
```

**步骤2.3：裁剪区域**

以目标中心为中心，裁剪一个正方形区域用于匹配。

```python
# 计算裁剪半径（取模板和目标半径的最大值）
template_radius = template.feature.workpiece_radius
target_radius = target_feature.workpiece_radius
crop_radius = max(template_radius, target_radius)
crop_size = int(np.ceil(crop_radius * 2.0))

# 计算裁剪区域
crop_x = int(max(0.0, target_center[0] - crop_radius))
crop_y = int(max(0.0, target_center[1] - crop_radius))
crop_w = int(min(crop_size, target_mask.shape[1] - crop_x))
crop_h = int(min(crop_size, target_mask.shape[0] - crop_y))

# 裁剪掩膜
target_mask_cropped = target_mask[crop_y:crop_y+crop_h, 
                                   crop_x:crop_x+crop_w]
template_mask_cropped = template_mask_aligned[crop_y:crop_y+crop_h,
                                               crop_x:crop_x+crop_w]
```

**为什么需要裁剪？**
- 减少计算量（只比较工件区域，忽略背景）
- 提高匹配精度（避免背景干扰）

**步骤2.4：缩放（可选）**

为了加速匹配，可以缩小掩膜尺寸。

```python
scale_factor = brute_force_angle_matching_scale  # 默认1.0（不缩放）

if abs(scale_factor - 1.0) > 1e-6:
    scaled_w = max(1, int(round(target_mask_cropped.shape[1] * scale_factor)))
    scaled_h = max(1, int(round(target_mask_cropped.shape[0] * scale_factor)))
    
    template_mask_scaled = cv2.resize(
        template_mask_cropped, (scaled_w, scaled_h),
        interpolation=cv2.INTER_NEAREST
    )
    target_mask_scaled = cv2.resize(
        target_mask_cropped, (scaled_w, scaled_h),
        interpolation=cv2.INTER_NEAREST
    )
    
    # 二值化
    _, template_mask_scaled = cv2.threshold(
        template_mask_scaled, 127, 255, cv2.THRESH_BINARY
    )
    _, target_mask_scaled = cv2.threshold(
        target_mask_scaled, 127, 255, cv2.THRESH_BINARY
    )
```

**步骤2.5：角度搜索**

对每个角度，旋转模板掩膜并计算重叠度。

```python
# 计算搜索范围
template_angle = template.feature.standardized_angle_deg
target_angle = target_feature.standardized_angle_deg
relative_angle = target_angle - template_angle

min_angle = relative_angle - 180.0  # ±180度范围
max_angle = relative_angle + 180.0
angle_step = brute_force_angle_step_deg  # 默认1.0度

# 初始化最佳结果
local_best_confidence = 0.0
local_best_angle_deg = 0.0

# 角度搜索循环
angle = min_angle
while angle <= max_angle:
    # 归一化角度到0-360度
    normalized_angle = angle
    while normalized_angle < 0.0:
        normalized_angle += 360.0
    while normalized_angle >= 360.0:
        normalized_angle -= 360.0
    
    # 计算旋转矩阵（以目标中心为旋转中心）
    M = cv2.getRotationMatrix2D(scaled_center, normalized_angle, 1.0)
    
    # 旋转模板掩膜
    rotated_template = cv2.warpAffine(
        template_mask_scaled,
        M,
        (target_mask_scaled.shape[1], target_mask_scaled.shape[0]),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )
    
    # 二值化
    _, rotated_template = cv2.threshold(
        rotated_template, 127, 255, cv2.THRESH_BINARY
    )
    
    # 计算重叠度（IoU）
    confidence = calculate_mask_iou(rotated_template, target_mask_scaled)
    
    # 更新最佳结果
    if confidence > local_best_confidence:
        local_best_confidence = confidence
        local_best_angle_deg = angle
    
    # 如果达到接受阈值，提前退出
    if confidence >= brute_force_acceptance_threshold:
        break
    
    angle += angle_step
```

**重叠度（IoU）计算：**

```python
def calculate_mask_iou(mask1, mask2):
    # 转换为布尔掩膜（0或1）
    m1 = (mask1 > 0).astype(np.uint8)
    m2 = (mask2 > 0).astype(np.uint8)
    
    # 计算交集（两个掩膜都为1的像素）
    intersection = np.sum(m1 & m2)
    
    # 计算并集（至少一个掩膜为1的像素）
    union = np.sum(m1 | m2)
    
    # IoU = 交集 / 并集
    if union == 0:
        return 0.0
    return float(intersection) / float(union)
```

**IoU示例：**
```
掩膜1（模板）:    掩膜2（目标）:    交集:          并集:
[1 1 0]          [1 0 0]          [1 0 0]        [1 1 0]
[1 1 0]          [1 1 0]          [1 1 0]        [1 1 0]
[0 0 0]          [0 0 0]          [0 0 0]        [0 0 0]

交集像素数: 3
并集像素数: 4
IoU = 3/4 = 0.75
```

**步骤2.6：生成对齐掩膜**

使用最佳角度生成完整尺寸的对齐掩膜。

```python
# 归一化最佳角度
normalized_best_angle = local_best_angle_deg
while normalized_best_angle < 0.0:
    normalized_best_angle += 360.0
while normalized_best_angle >= 360.0:
    normalized_best_angle -= 360.0

# 计算完整变换矩阵（旋转+平移）
transform_full = cv2.getRotationMatrix2D(
    template_mask_center, normalized_best_angle, 1.0
)
# 添加平移
transform_full[0, 2] += target_center[0] - template_mask_center[0]
transform_full[1, 2] += target_center[1] - template_mask_center[1]

# 生成对齐掩膜
aligned_mask_full = cv2.warpAffine(
    template_mask,
    transform_full,
    (target_mask.shape[1], target_mask.shape[0]),
    flags=cv2.INTER_NEAREST,
    borderMode=cv2.BORDER_CONSTANT,
    borderValue=0
)
```

**输出：**
- `best_idx`: 最佳模板索引
- `distance`: 1.0 - confidence（置信度越高，距离越小）
- `confidence`: IoU值（0.0-1.0）
- `best_angle_deg`: 最佳匹配角度（度）
- `best_aligned_mask`: 对齐后的完整掩膜

---

### 步骤3：加载相机位姿 (`_load_camera_pose`)

#### 3.1 目的

获取当前相机位姿和模板拍摄时的相机位姿，用于坐标系转换。

#### 3.2 详细过程

**步骤3.1：当前相机位姿（T_B_C）**

```python
# 目前使用单位矩阵（眼在手上系统）
T_B_C = np.eye(4)  # 4×4单位矩阵
```

**单位矩阵的含义：**
```
[1  0  0  0]
[0  1  0  0]
[0  0  1  0]
[0  0  0  1]
```

表示相机坐标系与基座坐标系重合（眼在手上系统）。

**步骤3.2：模板相机位姿（T_B_C_template）**

```python
# 从模板目录加载 camera_pose.json
template_dir = Path(self.template_root) / object_id / best_template.id
camera_pose_file = template_dir / "camera_pose.json"

if camera_pose_file.exists():
    with open(camera_pose_file, 'r') as f:
        camera_pose_data = json.load(f)
        
    # 读取机器人末端位姿（基座坐标系）
    pos = camera_pose_data['cartesian_position']['position']
    ori = camera_pose_data['cartesian_position']['orientation']
    
    # 构建T_B_E（基座到末端）
    T_B_E_camera = np.eye(4)
    T_B_E_camera[0, 3] = pos['x']  # 平移X
    T_B_E_camera[1, 3] = pos['y']  # 平移Y
    T_B_E_camera[2, 3] = pos['z']  # 平移Z
    
    # 四元数转旋转矩阵
    qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
    R = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    T_B_E_camera[:3, :3] = R
    
    # 计算：T_B_C = T_B_E * T_E_C
    T_E_C = self.T_E_C if self.T_E_C is not None else np.eye(4)
    T_B_C_template = T_B_E_camera @ T_E_C
```

**四元数转旋转矩阵公式：**
```python
def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),   1 - 2*(qx*qx + qy*qy)]
    ])
    return R
```

**输出：**
- `T_B_C`: 当前相机位姿（4×4矩阵）
- `T_B_C_template`: 模板相机位姿（4×4矩阵，可能为None）

---

### 步骤4：计算角度差 (`_calculate_angle_difference`)

#### 4.1 目的

计算模板与目标工件的角度差，用于后续的姿态旋转计算。

#### 4.2 详细过程

```python
dtheta_rad = None

if brute_force_matching_enabled and best_angle_deg is not None:
    # 将角度从度转换为弧度
    dtheta_rad = np.deg2rad(best_angle_deg)
    
    # 归一化到[-π, π]范围
    while dtheta_rad > np.pi:
        dtheta_rad -= 2 * np.pi
    while dtheta_rad < -np.pi:
        dtheta_rad += 2 * np.pi
    
    # 保存置信度供后续使用
    self.pose_estimator._last_match_confidence = confidence
else:
    # 如果未启用暴力匹配，返回None
    # 姿态估计会从特征角度计算
    dtheta_rad = None
```

**角度归一化示例：**
```python
# 输入：270度
dtheta_rad = np.deg2rad(270) = 4.712  # 弧度

# 归一化
while dtheta_rad > π (3.14159):
    dtheta_rad -= 2π
# 结果：4.712 - 6.283 = -1.571 弧度 = -90度
```

**输出：**
- `dtheta_rad`: 角度差（弧度），范围 [-π, π]

---

### 步骤5：姿态估计 (`_estimate_pose_for_feature`)

这是最核心的步骤，将2D图像特征转换为3D机器人姿态。

#### 5.1 目的

计算目标工件在机器人基座坐标系下的抓取姿态和准备姿态。

#### 5.2 详细计算过程

##### 5.2.1 获取模板抓取姿态深度

```python
# 从模板的T_C_E_grasp获取Z坐标（相机坐标系下的深度）
T_C_E_grasp_template = best_template.T_C_E_grasp
z_template_camera = T_C_E_grasp_template[2, 3]  # 提取Z坐标
```

**T_C_E_grasp的含义：**
- 从相机坐标系到末端执行器坐标系的变换
- `[2, 3]` 表示第3行第4列，即Z方向的平移（深度）

**示例：**
```python
T_C_E_grasp_template = [
    [R11, R12, R13, 0.1],   # X方向平移0.1米
    [R21, R22, R23, 0.05],  # Y方向平移0.05米
    [R31, R32, R33, 0.8],   # Z方向平移0.8米 ← 这就是深度
    [0,   0,   0,   1  ]
]
z_template_camera = 0.8  # 米
```

##### 5.2.2 转换模板抓取姿态到基座坐标系

```python
# T_B_E = T_B_C * T_C_E
T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template

# 提取位置和姿态
template_grasp_pos = T_B_E_grasp_template[:3, 3]  # [x, y, z]
template_grasp_rot = T_B_E_grasp_template[:3, :3]  # 旋转矩阵
```

**矩阵乘法示例：**
```python
# T_B_C_template (4×4)
# T_C_E_grasp_template (4×4)
# 结果：T_B_E_grasp_template (4×4)

# 例如：
T_B_C_template = [
    [1, 0, 0, 0.5],   # 相机在基座坐标系中X=0.5米
    [0, 1, 0, 0.3],   # Y=0.3米
    [0, 0, 1, 1.0],   # Z=1.0米
    [0, 0, 0, 1  ]
]

T_C_E_grasp_template = [
    [1, 0, 0, 0.1],
    [0, 1, 0, 0.05],
    [0, 0, 1, 0.8],
    [0, 0, 0, 1  ]
]

# 结果
T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template
# 位置 = T_B_C的位置 + T_C_E的位置（考虑旋转）
```

##### 5.2.3 计算模板工件中心在基座坐标系下的位置

**步骤1：从像素坐标转换为相机坐标系**

```python
# 从模板特征获取工件中心（像素坐标）
template_center_u = best_template.feature.workpiece_center[0]
template_center_v = best_template.feature.workpiece_center[1]

# 使用相机内参和深度，将像素坐标转换为相机坐标系
fx = camera_matrix[0, 0]  # X方向焦距
fy = camera_matrix[1, 1]  # Y方向焦距
cx = camera_matrix[0, 2]  # X方向主点
cy = camera_matrix[1, 2]  # Y方向主点

# 像素坐标转相机坐标公式
template_center_x_camera = (template_center_u - cx) * z_template_camera / fx
template_center_y_camera = (template_center_v - cy) * z_template_camera / fy
template_center_z_camera = z_template_camera
```

**公式推导：**

相机成像模型（针孔相机模型）：
```
x_camera / z_camera = (u - cx) / fx
y_camera / z_camera = (v - cy) / fy
```

因此：
```
x_camera = (u - cx) * z_camera / fx
y_camera = (v - cy) * z_camera / fy
```

**示例计算：**
```python
# 输入
template_center_u = 320
template_center_v = 240
z_template_camera = 0.8  # 米
fx = 600  # 像素
fy = 600  # 像素
cx = 320  # 像素
cy = 240  # 像素

# 计算
template_center_x_camera = (320 - 320) * 0.8 / 600 = 0.0  # 米
template_center_y_camera = (240 - 240) * 0.8 / 600 = 0.0  # 米
template_center_z_camera = 0.8  # 米

# 结果：工件中心在相机坐标系中为 (0, 0, 0.8)
```

**步骤2：转换到基座坐标系**

```python
# 转换为齐次坐标
template_center_camera = np.array([
    template_center_x_camera,
    template_center_y_camera,
    template_center_z_camera,
    1.0
])

# 转换到基座坐标系
template_center_base = T_B_C_template @ template_center_camera
template_center_base_pos = template_center_base[:3]  # [x, y, z]
```

##### 5.2.4 计算模板抓取位置相对偏移

```python
# 在基座坐标系中计算偏移
offset_template_base = template_grasp_pos - template_center_base_pos
# 结果: [Δx, Δy, Δz]
```

**示例：**
```python
template_grasp_pos = [0.5, 0.3, 0.9]      # 抓取位置
template_center_base_pos = [0.5, 0.3, 0.8]  # 工件中心
offset_template_base = [0.0, 0.0, 0.1]    # 偏移（Z方向高0.1米）
```

##### 5.2.5 计算目标工件中心在基座坐标系下的位置

**步骤1：从深度图获取实际深度**

```python
# 从目标特征获取工件中心（像素坐标）
target_center_u = target_feature.workpiece_center[0]
target_center_v = target_feature.workpiece_center[1]

# 尝试从深度图获取实际深度
target_center_z_camera = _get_depth_from_image(
    depth_image,
    int(target_center_u),
    int(target_center_v),
    search_radius=5
)
```

**深度获取算法：**

```python
def _get_depth_from_image(depth_image, pixel_u, pixel_v, search_radius=3):
    # 从内到外搜索有效深度值
    for r in range(0, search_radius + 1):
        depths = []
        
        # 在圆形区域内搜索
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                # 检查是否在圆形范围内
                if du * du + dv * dv > r * r:
                    continue
                
                u = pixel_u + du
                v = pixel_v + dv
                
                # 检查边界
                if u < 0 or u >= width or v < 0 or v >= height:
                    continue
                
                depth_value = depth_image[v, u]
                
                # 检查有效性（0和65535表示无效）
                if 0 < depth_value < 65535:
                    depths.append(float(depth_value))
        
        # 如果找到有效值，返回中位数
        if len(depths) > 0:
            median_depth_raw = np.median(depths)
            depth_meters = median_depth_raw * 0.00025  # 深度缩放因子
            return depth_meters
    
    return None
```

**为什么使用中位数？**
- 中位数对异常值更鲁棒
- 如果深度图有噪声，中位数比平均值更可靠

**步骤2：像素坐标转相机坐标**

```python
# 如果深度图获取失败，使用模板深度
if target_center_z_camera is None:
    target_center_z_camera = template_center_z_camera

# 像素坐标转相机坐标
target_center_x_camera = (target_center_u - cx) * target_center_z_camera / fx
target_center_y_camera = (target_center_v - cy) * target_center_z_camera / fy
```

**步骤3：转换到基座坐标系**

```python
# 转换为齐次坐标
target_center_camera = np.array([
    target_center_x_camera,
    target_center_y_camera,
    target_center_z_camera,
    1.0
])

# 转换到基座坐标系
target_center_base = T_B_C @ target_center_camera
target_center_base_pos = target_center_base[:3]  # [x, y, z]
```

##### 5.2.6 计算目标抓取姿态

这是最关键的计算步骤。

**步骤1：计算角度差（如果未提供）**

```python
if dtheta_rad is None:
    # 从特征角度计算
    template_angle = best_template.feature.standardized_angle
    target_angle = target_feature.standardized_angle
    dtheta_rad = target_angle - template_angle
```

**步骤2：构建绕Z轴旋转矩阵**

```python
# 构建绕Z轴旋转矩阵（在基座坐标系中）
cos_dtheta = np.cos(dtheta_rad)
sin_dtheta = np.sin(dtheta_rad)

R_z_base = np.array([
    [cos_dtheta, -sin_dtheta, 0],
    [sin_dtheta,  cos_dtheta, 0],
    [0,           0,          1]
])
```

**旋转矩阵的几何意义：**
```
绕Z轴旋转θ角度：
- X轴旋转到: (cos(θ), sin(θ), 0)
- Y轴旋转到: (-sin(θ), cos(θ), 0)
- Z轴不变: (0, 0, 1)
```

**步骤3：计算目标抓取位置**

```python
# 步骤3.1: 对目标中心应用模板的XY偏移
template_grasp_offset_xy = np.array([
    offset_template_base[0],  # Δx
    offset_template_base[1],  # Δy
    0.0                        # Z方向不偏移
])

intermediate_pos = target_center_base_pos + template_grasp_offset_xy
```

**可视化说明：**
```
模板情况：
  工件中心: (0.5, 0.3, 0.8)
  抓取位置: (0.5, 0.3, 0.9)  ← 偏移 (0, 0, 0.1)

目标情况：
  工件中心: (0.6, 0.4, 0.85)
  中间位置: (0.6, 0.4, 0.85) + (0, 0, 0) = (0.6, 0.4, 0.85)
```

```python
# 步骤3.2: 计算相对目标中心的偏移
intermediate_offset = intermediate_pos - target_center_base_pos
```

```python
# 步骤3.3: 绕Z轴旋转偏移
rotated_intermediate_offset = R_z_base @ intermediate_offset
```

**旋转示例：**
```python
# 假设角度差为90度
dtheta_rad = π/2

# 原始偏移
intermediate_offset = [0.1, 0.0, 0.0]  # X方向偏移0.1米

# 旋转矩阵（90度）
R_z_base = [
    [0, -1, 0],
    [1,  0, 0],
    [0,  0, 1]
]

# 旋转后
rotated_intermediate_offset = [0, 0.1, 0]  # Y方向偏移0.1米
```

```python
# 步骤3.4: 计算最终抓取位置
target_grasp_pos_base = target_center_base_pos + rotated_intermediate_offset
target_grasp_pos_base[2] = template_grasp_pos[2]  # Z坐标使用模板的Z
```

**步骤4：计算目标抓取姿态**

```python
# 获取模板抓取姿态的旋转矩阵
R_template_grasp = T_B_E_grasp_template[:3, :3]

# 应用旋转：R_target = R_z * R_template
R_target_grasp = R_z_base @ R_template_grasp

# 构建完整的变换矩阵
T_B_E_grasp_current = np.eye(4)
T_B_E_grasp_current[:3, :3] = R_target_grasp  # 旋转部分
T_B_E_grasp_current[:3, 3] = target_grasp_pos_base  # 平移部分
```

**完整的抓取姿态矩阵：**
```python
T_B_E_grasp_current = [
    [R11, R12, R13, pos_x],
    [R21, R22, R23, pos_y],
    [R31, R32, R33, pos_z],
    [0,   0,   0,   1   ]
]
```

##### 5.2.7 计算目标准备姿态（如果有）

```python
if best_template.T_C_E_prep is not None:
    # 转换到基座坐标系
    T_B_E_prep_template = T_B_C_template @ best_template.T_C_E_prep
    
    # 计算相对偏移
    prep_pos_template = T_B_E_prep_template[:3, 3]
    offset_prep_base = prep_pos_template - template_center_base_pos
    
    # 应用相同的旋转和平移
    intermediate_prep_pos = target_center_base_pos + offset_prep_base
    intermediate_prep_offset = intermediate_prep_pos - target_center_base_pos
    rotated_prep_offset = R_z_base @ intermediate_prep_offset
    
    target_prep_pos_base = target_center_base_pos + rotated_prep_offset
    target_prep_pos_base[2] = prep_pos_template[2]
    
    # 计算旋转
    R_template_prep = T_B_E_prep_template[:3, :3]
    R_target_prep = R_z_base @ R_template_prep
    
    # 构建变换矩阵
    T_B_E_prep_current = np.eye(4)
    T_B_E_prep_current[:3, :3] = R_target_prep
    T_B_E_prep_current[:3, 3] = target_prep_pos_base
```

##### 5.2.8 计算置信度

```python
if hasattr(pose_estimator, '_last_match_confidence'):
    # 使用暴力匹配的置信度
    result.confidence = pose_estimator._last_match_confidence
elif feature_distance is not None:
    # 基于特征距离计算：confidence = 1 / (1 + distance)
    result.confidence = 1.0 / (1.0 + feature_distance)
else:
    # 默认置信度
    result.confidence = 0.8
```

**置信度含义：**
- **1.0**：完美匹配
- **0.8-1.0**：优秀匹配
- **0.5-0.8**：良好匹配
- **0.0-0.5**：一般匹配

---

### 步骤6：加载附加姿态 (`_load_additional_poses`)

#### 6.1 目的

从模板目录加载预放置和放置姿态（如果存在）。

#### 6.2 详细过程

```python
template_dir = Path(self.template_root) / object_id / best_template.id

# 加载预放置姿态
preplace_json = template_dir / "preplace_position.json"
if preplace_json.exists():
    with open(preplace_json, 'r', encoding='utf-8') as f:
        preplace_data = json.load(f)
    
    # 从JSON构建4×4变换矩阵
    T_B_E_preplace = self._load_pose_json_data(preplace_data)
    if T_B_E_preplace is not None:
        result.T_B_E_preplace = T_B_E_preplace

# 加载放置姿态
place_json = template_dir / "place_position.json"
if place_json.exists():
    with open(place_json, 'r', encoding='utf-8') as f:
        place_data = json.load(f)
    
    T_B_E_place = self._load_pose_json_data(place_data)
    if T_B_E_place is not None:
        result.T_B_E_place = T_B_E_place
```

**JSON格式示例：**
```json
{
  "cartesian_position": {
    "position": {"x": 0.5, "y": 0.3, "z": 0.9},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "euler_orientation_rpy_deg": [0.0, 0.0, 0.0]
  }
}
```

---

### 步骤7：输出结果摘要 (`_log_pose_result_summary`)

#### 7.1 目的

记录姿态估计结果的摘要信息到日志。

#### 7.2 详细过程

```python
# 提取位置
grasp_pos = result.T_B_E_grasp[:3, 3]  # [x, y, z]

# 从旋转矩阵提取欧拉角（RPY）
R = result.T_B_E_grasp[:3, :3]
roll = np.arctan2(R[2, 1], R[2, 2])
pitch = -np.arcsin(R[2, 0])
yaw = np.arctan2(R[1, 0], R[0, 0])

# 输出日志
self.logger.info(f'抓取位置: ({grasp_pos[0]:.4f}, {grasp_pos[1]:.4f}, {grasp_pos[2]:.4f})m')
self.logger.info(f'抓取姿态(RPY): ({np.degrees(roll):.2f}°, {np.degrees(pitch):.2f}°, {np.degrees(yaw):.2f}°)')
self.logger.info(f'置信度: {result.confidence:.4f}')
```

**欧拉角提取公式：**
```
roll  = arctan2(R[2,1], R[2,2])   # 绕X轴旋转
pitch = -arcsin(R[2,0])           # 绕Y轴旋转
yaw   = arctan2(R[1,0], R[0,0])   # 绕Z轴旋转
```

---

## 数学公式详解

### 1. 像素坐标到相机坐标转换

**公式：**
```
x_camera = (u - cx) * z / fx
y_camera = (v - cy) * z / fy
z_camera = z
```

**推导：**

针孔相机模型：
```
u = fx * (x_camera / z_camera) + cx
v = fy * (y_camera / z_camera) + cy
```

反推：
```
x_camera = (u - cx) * z_camera / fx
y_camera = (v - cy) * z_camera / fy
```

### 2. 坐标系转换

**公式：**
```
P_base = T_B_C @ P_camera
```

**展开：**
```
[x_base]   [R11  R12  R13  tx] [x_camera]
[y_base] = [R21  R22  R23  ty] [y_camera]
[z_base]   [R31  R32  R33  tz] [z_camera]
[1     ]   [0    0    0    1 ] [1       ]
```

**计算：**
```
x_base = R11*x_camera + R12*y_camera + R13*z_camera + tx
y_base = R21*x_camera + R22*y_camera + R23*z_camera + ty
z_base = R31*x_camera + R32*y_camera + R33*z_camera + tz
```

### 3. 绕Z轴旋转矩阵

**公式：**
```
R_z(θ) = [cos(θ)  -sin(θ)  0]
         [sin(θ)   cos(θ)  0]
         [0        0       1]
```

**几何意义：**
- 点 (x, y) 绕Z轴旋转θ角度后变为：
  - x' = x*cos(θ) - y*sin(θ)
  - y' = x*sin(θ) + y*cos(θ)
  - z' = z（不变）

### 4. 重叠度（IoU）计算

**公式：**
```
IoU = intersection(A, B) / union(A, B)
```

**其中：**
```
intersection(A, B) = |A ∩ B|  # 交集像素数
union(A, B) = |A ∪ B|         # 并集像素数
```

**范围：**
- IoU ∈ [0, 1]
- 1.0：完全重叠
- 0.0：完全不重叠

### 5. 特征距离计算

**公式：**
```
distance = sqrt(radius_diff² + angle_diff_norm²)
```

**其中：**
```
radius_diff = |ratio1 - ratio2|
ratio = valve_radius / workpiece_radius

angle_diff = min(|angle1 - angle2|, 360 - |angle1 - angle2|)
angle_diff_norm = angle_diff / 180.0
```

---

## 实际应用示例

### 完整示例：从图像到机器人姿态

假设我们有一个工件，需要计算抓取姿态。

#### 输入数据

**图像特征：**
```python
feature = ComponentFeature(
    workpiece_center=(320, 240),      # 图像中心
    workpiece_radius=50.0,             # 50像素
    valve_center=(320, 200),          # 阀体中心
    valve_radius=10.0,                # 10像素
    standardized_angle_deg=45.0        # 45度
)
```

**深度图：**
- 工件中心深度：800mm（0.8米）

**相机参数：**
```python
fx = 600  # 像素
fy = 600  # 像素
cx = 320  # 像素
cy = 240  # 像素
```

**模板数据：**
```python
template = TemplateItem(
    id="pose_1",
    feature=ComponentFeature(
        workpiece_center=(300, 220),
        workpiece_radius=48.0,
        standardized_angle_deg=0.0  # 模板角度为0度
    ),
    T_C_E_grasp=[
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.8],  # 深度0.8米
        [0, 0, 0, 1  ]
    ]
)
```

#### 计算过程

**步骤1：模板匹配**
- 找到最佳模板：`pose_1`
- 角度差：45度

**步骤2：计算工件中心（相机坐标系）**
```python
target_center_x_camera = (320 - 320) * 0.8 / 600 = 0.0
target_center_y_camera = (240 - 240) * 0.8 / 600 = 0.0
target_center_z_camera = 0.8
```

**步骤3：转换到基座坐标系**
```python
# 假设 T_B_C = 单位矩阵（眼在手上）
target_center_base = [0.0, 0.0, 0.8]
```

**步骤4：计算抓取位置**
```python
# 模板抓取位置相对偏移
offset_template_base = [0.0, 0.0, 0.0]  # 假设抓取位置在工件中心

# 应用旋转（45度）
dtheta_rad = π/4
R_z_base = [
    [0.707, -0.707, 0],
    [0.707,  0.707, 0],
    [0,      0,     1]
]

# 最终抓取位置
target_grasp_pos_base = [0.0, 0.0, 0.8]
```

**步骤5：计算抓取姿态**
```python
# 模板姿态旋转矩阵（假设为单位矩阵）
R_template_grasp = np.eye(3)

# 应用旋转
R_target_grasp = R_z_base @ R_template_grasp

# 最终姿态
T_B_E_grasp = [
    [0.707, -0.707, 0, 0.0],
    [0.707,  0.707, 0, 0.0],
    [0,      0,     1, 0.8],
    [0,      0,     0, 1  ]
]
```

#### 输出结果

```python
result = PoseEstimationResult(
    template_id="pose_1",
    T_B_E_grasp=T_B_E_grasp,  # 如上所示
    confidence=0.95,
    ...
)
```

---

## 常见问题

### Q1: 为什么需要深度图？

**A:** 深度图用于将2D像素坐标转换为3D空间坐标。没有深度信息，无法确定工件在空间中的实际位置。

### Q2: 什么是标准化角度？

**A:** 标准化角度是工件相对于标准方向的旋转角度。用于匹配不同角度的工件。

### Q3: 为什么需要模板库？

**A:** 模板库存储了已知工件的特征和抓取姿态。通过匹配当前检测的工件与模板，可以快速计算抓取姿态。

### Q4: 置信度如何计算？

**A:** 
- 如果使用暴力匹配：置信度 = IoU（重叠度）
- 如果使用特征距离：置信度 = 1 / (1 + distance)

### Q5: 坐标系转换为什么重要？

**A:** 图像中的位置是2D像素坐标，机器人需要3D基座坐标。必须通过坐标系转换才能让机器人执行抓取动作。

### Q6: 什么是眼在手上系统？

**A:** 相机安装在机器人末端执行器上，相机坐标系与基座坐标系的关系通过手眼标定确定。

### Q7: 为什么需要旋转矩阵？

**A:** 工件可能以不同角度出现，需要通过旋转将模板姿态调整到目标姿态。

### Q8: 如何处理多个检测到的工件？

**A:** `_process_single_feature` 处理单个工件。对于多个工件，需要循环调用此方法。

---

## 总结

`_process_single_feature` 方法完成了从图像特征到机器人姿态的完整转换：

1. **提取掩膜**：准备模板匹配所需的数据
2. **模板匹配**：找到最相似的模板
3. **加载位姿**：获取坐标系转换信息
4. **计算角度**：确定旋转量
5. **姿态估计**：计算最终的3D姿态
6. **加载附加姿态**：获取完整的操作序列
7. **输出结果**：记录处理结果

整个过程涉及：
- 图像处理（掩膜、裁剪、旋转）
- 坐标系转换（像素→相机→基座）
- 几何计算（旋转、平移、偏移）
- 模板匹配（特征距离或暴力匹配）

理解这些步骤和计算过程，有助于：
- 调试姿态估计问题
- 优化算法性能
- 理解系统工作原理
- 扩展功能

---

## 参考资料

1. 计算机视觉基础：针孔相机模型
2. 机器人学：齐次变换矩阵
3. 图像处理：模板匹配算法
4. 几何学：旋转矩阵和欧拉角

---

**文档版本：** 1.0  
**最后更新：** 2024-01-XX  
**作者：** Visual Pose Estimation Team
