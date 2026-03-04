# 预处理和特征提取参数调试工具

## 简介

`debug_preprocess_feature.py` 是一个交互式调试工具，用于实时调整和测试预处理参数和特征提取参数。该工具基于C++实现逻辑，使用Python和OpenCV实现，提供可视化界面。

## 功能特性

1. **实时参数调整**: 通过滑动条实时调整预处理和特征提取参数
2. **可视化结果**: 实时显示处理结果，包括：
   - 背景去除后的前景掩码
   - 连通域提取结果
   - 工件外接圆（绿色）
   - 阀体外接圆（蓝色）
   - 两圆心连线（黄色）
3. **配置保存**: 可以将调整好的参数保存为YAML配置文件
4. **参数验证**: 自动验证参数范围，确保参数有效性

## 使用方法

### 基本用法

```bash
# 使用默认配置文件
python3 debug_preprocess_feature.py <image_path>

# 指定配置文件
python3 debug_preprocess_feature.py <image_path> <config_path>
```

### 示例

```bash
# 使用默认配置调试图像
python3 debug_preprocess_feature.py /path/to/image.jpg

# 使用指定配置调试图像
python3 debug_preprocess_feature.py /path/to/image.jpg configs/default.yaml
```

## 操作说明

### 键盘快捷键

- **`r`**: 重新处理图像（实时处理，无需按键）
- **`s`**: 保存当前配置到 `debug_config.yaml`
- **`q`** 或 **ESC**: 退出程序

### 滑动条说明

#### 预处理参数

1. **Scale Factor (x10)**: 图像缩放因子 (0.1-1.0)
   - 用于在背景去除前缩小图像以提高处理速度
   - 1.0表示不缩放

2. **Border Ratio (x100)**: 边缘采样区域比例 (0.05-0.30)
   - 用于估计背景色的边缘区域比例

3. **Hue Margin**: HSV颜色空间H通道阈值
4. **Hue Std Mul (x10)**: HSV颜色空间H通道标准差倍数
5. **Sat Margin**: HSV颜色空间S通道阈值
6. **Sat Std Mul (x10)**: HSV颜色空间S通道标准差倍数
7. **Val Margin**: HSV颜色空间V通道阈值
8. **Val Std Mul (x10)**: HSV颜色空间V通道标准差倍数

9. **Lab Threshold (x10)**: Lab颜色空间阈值 (2.0-15.0)

10. **Cleanup Kernel**: 背景掩码形态学清理核大小（奇数）
11. **FG Close Kernel**: 前景掩码闭运算核大小（奇数）
12. **Median KSize**: 中值滤波核大小（奇数，最大5）

13. **Min Noise Area**: 最小噪声面积（像素²）
    - 小于此面积的连通域将被去除

14. **Component Min Area**: 连通域最小面积（像素²）
15. **Component Max Area (x10k)**: 连通域最大面积（像素²）

#### 特征提取参数

1. **Big Circle Min Area**: 工件外接圆提取的最小轮廓面积
2. **Small Erode Kernel**: 阀体外接圆提取的腐蚀核大小（奇数）
3. **Small Erode Iter**: 阀体外接圆提取的腐蚀迭代次数
4. **Small Dilate Kernel**: 阀体外接圆提取的膨胀核大小（奇数）
5. **Small Dilate Iter**: 阀体外接圆提取的膨胀迭代次数

## 输出说明

### 可视化显示

- **绿色圆**: 工件外接圆
- **蓝色圆**: 阀体外接圆
- **黄色线**: 两圆心连线（用于计算标准化角度）
- **半透明叠加**: 连通域掩码

### 信息显示

窗口左上角显示：
- `Components`: 检测到的连通域数量
- `Features`: 提取到的特征数量
- `WP Radius`: 工件外接圆半径（像素）
- `Valve Radius`: 阀体外接圆半径（像素）

## 配置文件格式

保存的配置文件格式与 `default.yaml` 相同：

```yaml
preprocess:
  scale_factor: 1.0
  min_area: 2000
  background:
    border_ratio: 0.08
    hue_margin: 12.0
    # ... 其他参数
  feature_extraction:
    min_component_area: 2000
    big_circle:
      combine_contours: true
      min_area: 100
    small_circle:
      erode_kernel: 11
      erode_iterations: 5
      # ... 其他参数
```

## 注意事项

1. **参数范围**: 某些参数有自动限制（如核大小必须为奇数）
2. **实时处理**: 参数调整后会自动重新处理，无需手动触发
3. **性能**: 如果图像较大，建议先使用 `scale_factor` 缩小图像以提高处理速度
4. **连通域筛选**: 如果检测到太多连通域，可以调整 `Component Min Area` 和 `Component Max Area`

## 故障排除

### 问题：看不到任何连通域

- 检查 `Component Min Area` 是否设置过大
- 检查背景去除参数是否合适
- 尝试调整 `Border Ratio` 和 HSV/Lab 阈值

### 问题：检测到太多连通域

- 增加 `Component Min Area`
- 减小 `Component Max Area`
- 调整 `Min Noise Area` 去除小噪声

### 问题：工件外接圆不准确

- 调整 `Big Circle Min Area`
- 检查连通域是否完整（可能需要调整形态学参数）

### 问题：阀体外接圆检测失败

- 调整 `Small Erode Kernel` 和 `Small Erode Iter`
- 调整 `Small Dilate Kernel` 和 `Small Dilate Iter`
- 检查阀体是否在连通域中可见

## 依赖项

- Python 3.6+
- OpenCV (cv2)
- NumPy
- PyYAML

## 作者

基于 visual_pose_estimation 项目的 C++ 实现逻辑开发
