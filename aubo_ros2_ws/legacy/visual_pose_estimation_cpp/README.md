# Visual Pose Estimation (C++ 旧版)

> **此包为旧版 C++ 实现，核心算法已移植至 `visual_pose_estimation_python`（Python）。**
> 当前不再维护，仅保留供参考。新功能开发和部署请使用 Python 版本。
>
> Python 版文档见：`../visual_pose_estimation_python/README.md`

基于C++的ROS2姿态估计节点（已移植至 Python）。

## 项目结构

包路径：`aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation/`（与仓库中 **`interface`**、**`visual_pose_estimation_python`** 并列于 `visual_pose_estimation/src/`）。

```
visual_pose_estimation/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── main.cpp
│   ├── ros2_communication.cpp
│   ├── config_reader.cpp
│   ├── preprocessor.cpp
│   ├── feature_extractor.cpp
│   ├── template_standardizer.cpp
│   └── （pose_estimation / template_creation 等可按 CMake 注释逐步启用）
├── include/visual_pose_estimation/
│   └── （与上述 .cpp 对应的头文件）
├── launch/
│   └── visual_pose_estimation.launch.py
├── configs/
│   └── default.yaml
└── scripts/                      # 预处理/特征调试等（见文末合并文档）
```

## 编译

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_pose_estimation
source install/setup.bash
```

## 运行

```bash
ros2 launch visual_pose_estimation visual_pose_estimation.launch.py
```

## 参数说明

- `config_file`: 配置文件路径（默认: `configs/default.yaml`）
- `calib_file`: 标定文件路径
- `template_root`: 模板库根目录（默认随 launch，常用：`.../visual_pose_estimation/templates`）
- `debug`: 是否启用调试模式（默认: `false`）

## 算法与模块

当前构建中包含：**预处理**（`preprocessor.cpp`）、**特征提取**（`feature_extractor.cpp`）、**模板标准化**（`template_standardizer.cpp`）等，与 ROS2 通信、配置解析共同组成节点主体。若 CMake 中部分目标仍注释为后续启用，以 **`CMakeLists.txt`** 为准。

## 依赖

- ROS2 (Humble/Iron)
- OpenCV
- yaml-cpp
- jsoncpp
- interface (ROS2接口包)

---

## 预处理/特征调试脚本（原 scripts/README_debug.md）

脚本路径：`scripts/debug_preprocess_feature.py`（与下述文档一致）。

### 简介

`debug_preprocess_feature.py` 是一个交互式调试工具，用于实时调整和测试预处理参数和特征提取参数。该工具基于C++实现逻辑，使用Python和OpenCV实现，提供可视化界面。

### 功能特性

1. **实时参数调整**: 通过滑动条实时调整预处理和特征提取参数
2. **可视化结果**: 实时显示处理结果，包括：
   - 背景去除后的前景掩码
   - 连通域提取结果
   - 工件外接圆（绿色）
   - 阀体外接圆（蓝色）
   - 两圆心连线（黄色）
3. **配置保存**: 可以将调整好的参数保存为YAML配置文件
4. **参数验证**: 自动验证参数范围，确保参数有效性

### 使用方法

#### 基本用法

```bash
# 使用默认配置文件
python3 debug_preprocess_feature.py <image_path>

# 指定配置文件
python3 debug_preprocess_feature.py <image_path> <config_path>
```

#### 示例

```bash
# 使用默认配置调试图像
python3 debug_preprocess_feature.py /path/to/image.jpg

## 使用指定配置调试图像
python3 debug_preprocess_feature.py /path/to/image.jpg configs/default.yaml
```

### 操作说明

#### 键盘快捷键

- **`r`**: 重新处理图像（实时处理，无需按键）
- **`s`**: 保存当前配置到 `debug_config.yaml`
- **`q`** 或 **ESC**: 退出程序

#### 滑动条说明

##### 预处理参数

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

##### 特征提取参数

1. **Big Circle Min Area**: 工件外接圆提取的最小轮廓面积
2. **Small Erode Kernel**: 阀体外接圆提取的腐蚀核大小（奇数）
3. **Small Erode Iter**: 阀体外接圆提取的腐蚀迭代次数
4. **Small Dilate Kernel**: 阀体外接圆提取的膨胀核大小（奇数）
5. **Small Dilate Iter**: 阀体外接圆提取的膨胀迭代次数

### 输出说明

#### 可视化显示

- **绿色圆**: 工件外接圆
- **蓝色圆**: 阀体外接圆
- **黄色线**: 两圆心连线（用于计算标准化角度）
- **半透明叠加**: 连通域掩码

#### 信息显示

窗口左上角显示：
- `Components`: 检测到的连通域数量
- `Features`: 提取到的特征数量
- `WP Radius`: 工件外接圆半径（像素）
- `Valve Radius`: 阀体外接圆半径（像素）

### 配置文件格式

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

### 注意事项

1. **参数范围**: 某些参数有自动限制（如核大小必须为奇数）
2. **实时处理**: 参数调整后会自动重新处理，无需手动触发
3. **性能**: 如果图像较大，建议先使用 `scale_factor` 缩小图像以提高处理速度
4. **连通域筛选**: 如果检测到太多连通域，可以调整 `Component Min Area` 和 `Component Max Area`

### 故障排除

#### 问题：看不到任何连通域

- 检查 `Component Min Area` 是否设置过大
- 检查背景去除参数是否合适
- 尝试调整 `Border Ratio` 和 HSV/Lab 阈值

#### 问题：检测到太多连通域

- 增加 `Component Min Area`
- 减小 `Component Max Area`
- 调整 `Min Noise Area` 去除小噪声

#### 问题：工件外接圆不准确

- 调整 `Big Circle Min Area`
- 检查连通域是否完整（可能需要调整形态学参数）

#### 问题：阀体外接圆检测失败

- 调整 `Small Erode Kernel` 和 `Small Erode Iter`
- 调整 `Small Dilate Kernel` 和 `Small Dilate Iter`
- 检查阀体是否在连通域中可见

### 依赖项

- Python 3.6+
- OpenCV (cv2)
- NumPy
- PyYAML

### 作者

基于 visual_pose_estimation 项目的 C++ 实现逻辑开发


---

## 使用示例与服务接口详解（原 docs/基本使用示例.md）

### 基本使用示例

```bash
## 环境
source ~/.bashrc
cd /home/mu/IVG2.0/aubo_ros2_ws
source install/setup.bash

## 编译
colcon build --packages-select visual_pose_estimation

## 运行
pkill -f visual_pose_estimation_node

ros2 launch visual_pose_estimation visual_pose_estimation.launch.py \
    config_file:=/home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation/configs/default.yaml \
    calib_file:=/home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation/configs/hand_eye_calibration.xml \
    template_root:=/home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/templates \
    debug:=true
```

### 测试示例

```bash
## 测试模板标准化服务
ros2 service call /standardize_template interface/srv/StandardizeTemplate "{workpiece_id: '1211242785'}"
ros2 service call /standardize_template interface/srv/StandardizeTemplate "{workpiece_id: '2211242785'}"
ros2 service call /standardize_template interface/srv/StandardizeTemplate "{workpiece_id: '3211242785'}"

## 测试姿态估计服务（需要提供图像数据）
ros2 service call /estimate_pose interface/srv/EstimatePose \
    "{input_image: {...}, object_id: '211242785'}"


## 查看节点状态
ros2 node list | grep visual_pose_estimation

## 查看系统状态话题
ros2 topic echo /visual_pose_estimation/status

## 查看图像数据话题
ros2 topic echo /image_data

## 查看服务列表
ros2 service list | grep visual_pose_estimation



## 测试模板列表服务
ros2 service call /list_templates interface/srv/ListTemplates \
    "{templates_dir: ''}"

## 测试调试步骤处理服务
ros2 service call /process_debug_step interface/srv/ProcessDebugStep \
    "{session_id: 'test', step_index: 0, step_id: 'original', step_name: '读图并创建副本', input_image: '...', is_redo: false}"

## 测试可视化抓取姿态服务
ros2 service call /visualize_grasp_pose interface/srv/VisualizeGraspPose \
    "{template_image_path: '/path/to/template.jpg', workpiece_id: '211242785', pose_id: '1'}"


```

### 接口说明

#### 话题 (Topics)

**/visual_pose_estimation/status** - 发布系统状态信息 (`std_msgs/msg/String`)
- 发布系统运行状态和日志信息

**/image_data** - 订阅图像数据 (`interface/msg/ImageData`)
- 接收相机采集的图像数据

#### 服务 (Services)

**/estimate_pose** - 姿态估计服务 (`interface/srv/EstimatePose`)

```EstimatePose.srv
# Request
string input_image              # 输入图像（base64编码）
string object_id                # 工件ID
# Response
bool success                    # 执行结果，成功为true
int32 success_num              # 检测到的目标数量
float32[] confidence           # 置信度数组
geometry_msgs/Point[] positions # 相机坐标系下的位置数组
geometry_msgs/Pose[] grab_positions      # 抓取位置数组
geometry_msgs/Pose[] preparation_positions # 准备位置数组
string[] pose_images            # 姿态可视化图像（base64）
string vis_image                # 可视化结果图像（base64）
string error_message            # 错误信息
```

**/list_templates** - 模板列表服务 (`interface/srv/ListTemplates`)

```ListTemplates.srv
# Request
string templates_dir            # 模板目录路径（空字符串使用默认路径）
# Response
bool success                    # 执行结果
int32 count                    # 模板数量
string[] template_ids          # 模板ID数组
string[] workpiece_ids         # 工件ID数组
string[] pose_ids              # 姿态ID数组
string[] image_paths           # 图像路径数组
string[] image_base64          # 图像base64编码数组
string error                   # 错误信息
```

**/process_debug_step** - 调试步骤处理服务 (`interface/srv/ProcessDebugStep`)

```ProcessDebugStep.srv
# Request
string session_id              # 调试会话ID
int32 step_index              # 步骤索引
string step_id                 # 步骤ID
string step_name               # 步骤名称
string input_image             # 输入图像（base64编码）
bool is_redo                   # 是否重做
# Response
bool success                  # 执行结果
string result_image           # 结果图像（base64编码）
string metadata               # 元数据（JSON字符串）
string error                  # 错误信息
```

**/visualize_grasp_pose** - 可视化抓取姿态服务 (`interface/srv/VisualizeGraspPose`)

```VisualizeGraspPose.srv
# Request
string template_image_path     # 模板图像路径
string workpiece_id           # 工件ID
string pose_id                # 姿态ID
# Response
bool success                  # 执行结果
string image_base64           # 可视化图像（base64编码）
string error                  # 错误信息
```

**/standardize_template** - 模板标准化服务 (`interface/srv/StandardizeTemplate`)

```StandardizeTemplate.srv
# Request
string workpiece_id           # 工件ID
# Response
bool success                  # 执行结果
int32 processed_count        # 已处理的姿态数量
int32 skipped_count         # 跳过的姿态数量
string[] processed_pose_ids  # 已处理的姿态ID数组
string[] skipped_pose_ids    # 跳过的姿态ID数组
string error_message         # 错误信息
```

### 参数说明

#### 节点参数

- `config_file`: 配置文件路径（默认: `configs/default.yaml`）
- `calib_file`: 标定文件路径（默认: `configs/calibration.yaml`）
- `template_root`: 模板库根目录（默认随 launch，常用：`.../visual_pose_estimation/templates`）
- `debug`: 是否启用调试模式（默认: `false`）

#### 使用示例

```bash
## 使用默认参数启动
ros2 launch visual_pose_estimation visual_pose_estimation.launch.py

# 指定配置文件
ros2 launch visual_pose_estimation visual_pose_estimation.launch.py \
    config_file:=/path/to/custom_config.yaml

## 启用调试模式
ros2 launch visual_pose_estimation visual_pose_estimation.launch.py \
    debug:=true

## 指定模板根目录
ros2 launch visual_pose_estimation visual_pose_estimation.launch.py \
    template_root:=/path/to/templates
```

### 验证节点运行

```bash
## 检查节点是否运行
ros2 node list | grep visual_pose_estimation

## 查看节点信息
ros2 node info /visual_pose_estimation

## 查看服务类型
ros2 service type /estimate_pose

## 查看话题类型
ros2 topic type /visual_pose_estimation/status

## 查看节点日志
ros2 topic echo /visual_pose_estimation/status
```

### 注意事项

1. **依赖检查**: 确保 `interface` 包已编译
2. **配置文件**: 确保配置文件路径正确
3. **模板库**: 模板库为空时会有警告，这是正常的
4. **算法实现**: 当前版本为基础框架，算法实现将在后续版本中添加



---

## 其他文档（未并入）

本包 `docs/` 下其它文件若后续增加，仍以独立 Markdown 维护；与 `doc/` 目录规则一致时可仅在此索引。
