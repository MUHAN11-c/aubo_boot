# visual_pose_estimation_python

Python ROS2实现的视觉姿态估计功能包

## 功能特性

该包使用Python实现了C++版本的visual_pose_estimation功能，基于**深度图像**进行处理，主要包括：

1. **深度图预处理（Preprocessor）** - 参考trigger_depth.py
   - 深度图0值插值处理（补齐法）
   - 基于深度阈值的二值化
   - 连通域提取和筛选

2. **特征提取（FeatureExtractor）**
   - 工件外接圆提取（大圆）
   - 阀体外接圆提取（小圆）
   - 标准化角度计算
   - 多线程并行处理

3. **模板标准化（TemplateStandardizer）**
   - 模板姿态标准化（旋转到标准方向）
   - 模板保存和加载
   - 元数据管理

4. **姿态估计（PoseEstimator）**
   - 模板库加载
   - 最佳模板匹配
   - 2D对齐计算
   - 3D姿态计算

5. **ROS2通信（ROS2Communication）**
   - EstimatePose服务（姿态估计）
   - ListTemplates服务（列出模板）
   - StandardizeTemplate服务（标准化模板）
   - 图像订阅功能（支持触发拍照模式）

## 深度图+彩色图混合处理

**本实现使用深度图+彩色图混合处理**，完全参考了`trigger_depth.py`的处理流程：

### 处理流程（参考trigger_depth.py的display_preprocessed_image）

```
深度图 → 二值化生成掩模 → 从彩色图抠出工件 → 在彩色工件图上提取特征
```

#### 1. 深度图预处理（生成掩模）
- **0值插值处理**：针对深度图中的个别0值点进行补齐
- **深度阈值二值化**：根据最小/最大深度阈值创建二值掩码
- **无效值处理**：自动识别和处理0值、65535等无效深度值
- **连通域提取**：从二值掩码中提取工件连通域

#### 2. 彩色图工件提取（使用掩模抠图）
- **识别框计算**：基于检测到的边界框放大1.2倍
- **掩模应用**：使用深度图二值掩码从彩色图抠出工件区域
- **形态学处理**：对掩码进行膨胀使边缘更平滑
- **白色背景**：非工件区域设置为白色背景

#### 3. 特征提取（在彩色工件图上）
- **工件外接圆提取**：使用大圆算法提取工件整体轮廓
- **阀体外接圆提取**：使用形态学处理（腐蚀、膨胀）提取阀体小圆
- **标准化角度计算**：从工件中心到阀体中心的角度
- **可视化绘制**：在彩色工件图上绘制特征圆和角度

### 与trigger_depth.py的一致性
所有图像处理算法与`trigger_depth.py`保持完全一致：

**深度图处理**：
- `process_depth_image()` - 深度图处理和0值插值
- `_interp_data()` - 0值补齐算法
- `create_binary_image()` - 基于深度阈值的二值化

**彩色图处理**：
- `extract_color_workpiece()` - 使用掩模从彩色图抠出工件
  - 对应trigger_depth.py的`display_preprocessed_image()`中的掩模应用部分
  - 创建白色背景
  - 识别框放大1.2倍
  - 膨胀处理平滑边缘

**特征提取**：
- `_extract_workpiece_circle()` - 大圆提取
- `_extract_valve_circle()` - 小圆提取
- `draw_features()` - 可视化绘制（绿色工件圆、蓝色阀体圆、青色连线）

## 安装和编译

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select visual_pose_estimation_python
source install/setup.bash
```

## 使用方法

### 完整工作流程

#### 第一步：启动相机节点

```bash
# 启动 Percipio 相机节点（根据实际情况调整）
ros2 launch percipio_camera percipio_camera.launch.py
```

#### 第二步：启动姿态估计节点

```bash
# 使用默认参数启动
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py

# 或自定义参数
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py \
  template_root:=/path/to/templates \
  config_file:=/path/to/config.yaml \
  calib_file:=/path/to/hand_eye_calibration.json \
  debug:=true
```

**默认配置：**
- 模板目录：`/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/templates`
- 配置文件：`configs/default.yaml`
- 订阅话题：
  - 深度图：`/camera/depth/image_raw`（可通过参数 `depth_image_topic` 配置）
  - 彩色图：`/camera/color/image_raw`（可通过参数 `color_image_topic` 配置）

#### 第三步：创建模板库（首次使用）

**重要：深度图和彩色图缺一不可，必须同时提供**

```bash
# 1. 触发拍照（同时获取深度图和彩色图）
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 等待约 0.5 秒让图像发布到话题

# 2. 标准化模板（创建第一个姿态模板）
ros2 service call /standardize_template interface/srv/StandardizeTemplate \
  "{workpiece_id: '3211242785'}"

# 3. 移动机器人到不同姿态，重复步骤 1-2 创建多个姿态模板
# 建议创建 5-10 个不同姿态的模板以提高识别率
```

**模板目录结构：**
```
templates/
└── 3211242785/              # 工件ID
    ├── pose_1/              # 姿态1
    │   ├── original_image.jpg
    │   ├── image.jpg
    │   ├── mask.jpg
    │   ├── standardized_mask.jpg
    │   └── metadata.json
    ├── pose_2/              # 姿态2
    │   └── ...
    └── pose_3/              # 姿态3
        └── ...
```

#### 第四步：姿态估计

```bash
# 1. 触发拍照
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 2. 姿态估计（自动使用触发拍照获取的最新图像）
ros2 service call /estimate_pose interface/srv/EstimatePose \
  "{object_id: '3211242785'}"
```

### 图像输入方式

节点启动后会自动订阅图像话题，使用**触发拍照模式**获取图像：

1. **调用软触发服务** → 相机拍照
2. **图像发布到话题** → 节点自动接收
3. **调用服务** → 使用最新图像进行处理

也可以在服务请求中直接传递图像（深度图和彩色图都必须提供）：
```bash
ros2 service call /estimate_pose interface/srv/EstimatePose \
  "{object_id: '3211242785', image: ..., color_image: ...}"
```

### ROS2 服务详解

#### 1. 软触发服务（拍照）

```bash
# 触发相机拍照，同时获取深度图和彩色图
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 响应示例
# success: True
# message: "拍照成功"
```

#### 2. 标准化模板服务

```bash
# 使用触发拍照获取的图像创建模板（推荐）
ros2 service call /standardize_template interface/srv/StandardizeTemplate \
  "{workpiece_id: '3211242785'}"

# 响应示例
# success: True
# processed_count: 1
# processed_pose_ids: ['pose_1']
```

**注意事项：**
- 使用前需先调用软触发服务拍照
- 每次调用会自动生成新的姿态ID（pose_1, pose_2, ...）
- 建议创建多个不同姿态的模板（5-10个）

#### 3. 姿态估计服务

```bash
# 使用触发拍照获取的图像进行姿态估计（推荐）
ros2 service call /estimate_pose interface/srv/EstimatePose \
  "{object_id: '3211242785'}"

# 响应示例
# success_num: 1
# confidence: [0.95]
# grab_position: [...]
# processing_time_sec: 0.125
```

**响应字段说明：**
- `success_num`: 成功检测到的物体数量
- `confidence`: 每个物体的置信度
- `position`: 图像坐标系中的位置
- `grab_position`: 抓取姿态（机器人基座坐标系）
- `pose_image`: 可视化图像（带特征标注）
- `processing_time_sec`: 处理时间

#### 4. 列出模板服务

```bash
# 列出所有可用的模板
ros2 service call /list_templates interface/srv/ListTemplates \
  "{templates_dir: ''}"

# 指定模板目录
ros2 service call /list_templates interface/srv/ListTemplates \
  "{templates_dir: '/path/to/templates'}"

# 响应示例
# template_ids: ['3211242785', '3211242786', ...]
```

## 完整使用示例

### 场景：首次使用，创建模板并进行姿态估计

```bash
# ============ 1. 启动系统 ============
# 终端1: 启动相机
ros2 launch percipio_camera percipio_camera.launch.py

# 终端2: 启动姿态估计节点
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py

# ============ 2. 创建模板库 ============
# 将工件放置在相机视野内，机器人移动到第一个位置

# 拍照
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 创建模板（姿态1）
ros2 service call /standardize_template interface/srv/StandardizeTemplate \
  "{workpiece_id: '3211242785'}"

# 移动机器人到第二个位置，重复拍照和创建模板
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"
ros2 service call /standardize_template interface/srv/StandardizeTemplate \
  "{workpiece_id: '3211242785'}"

# 重复3-5次，创建多个姿态的模板...

# ============ 3. 验证模板 ============
# 列出已创建的模板
ros2 service call /list_templates interface/srv/ListTemplates "{templates_dir: ''}"

# 应该看到：template_ids: ['3211242785']

# ============ 4. 姿态估计 ============
# 将工件放置在任意位置

# 拍照
ros2 service call /software_trigger percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: '207000152740'}"

# 估计姿态
ros2 service call /estimate_pose interface/srv/EstimatePose \
  "{object_id: '3211242785'}"

# 查看结果：success_num, confidence, grab_position 等
```

### 预期输出

**创建模板时：**
```
[INFO] [visual_pose_estimation_python]: 收到标准化模板请求: 3211242785
[INFO] [visual_pose_estimation_python]: ✓ 使用触发拍照的最新深度图
[INFO] [visual_pose_estimation_python]: ✓ 使用触发拍照的最新彩色图
[INFO] [visual_pose_estimation_python]: ✓ 图像准备完成 - 深度图: (480, 640), 彩色图: (480, 640, 3)
[INFO] [visual_pose_estimation_python]: 深度图预处理完成，提取到 1 个连通域
[INFO] [visual_pose_estimation_python]: 彩色工件图像提取完成
[INFO] [visual_pose_estimation_python]: 特征提取完成，成功提取 1 个特征
[INFO] [visual_pose_estimation_python]: 标准化模板已保存: .../templates/3211242785/pose_1
[INFO] [visual_pose_estimation_python]: 标准化模板完成: 成功
```

**姿态估计时：**
```
[INFO] [visual_pose_estimation_python]: 收到姿态估计请求，工件ID: 3211242785
[INFO] [visual_pose_estimation_python]: ✓ 使用触发拍照的最新深度图
[INFO] [visual_pose_estimation_python]: ✓ 使用触发拍照的最新彩色图
[INFO] [visual_pose_estimation_python]: 深度图预处理完成，提取到 1 个连通域
[INFO] [visual_pose_estimation_python]: 特征提取完成，成功提取 1 个特征
[INFO] [visual_pose_estimation_python]: 特征 0 最佳模板: pose_2, 距离: 0.0234
[INFO] [visual_pose_estimation_python]: 姿态估计完成，成功数: 1, 用时: 0.125s
```

## 配置文件

配置文件位于`configs/default.yaml`，包含：
- **深度图预处理参数**（深度阈值、0值插值、连通域筛选）
- **特征提取参数**（大圆、小圆提取、腐蚀膨胀参数、线程数）
- **姿态估计参数**（匹配阈值、暴力匹配）

### 关键参数说明

```yaml
preprocess:
  # 深度阈值（原始值，单位取决于相机）
  binary_threshold_min: 0        # 最小深度
  binary_threshold_max: 65535    # 最大深度
  enable_zero_interp: true       # 启用0值插值
  
  feature_extraction:
    # 小圆（阀体）提取参数
    small_circle:
      erode_kernel: 11           # 腐蚀核大小
      erode_iterations: 5        # 腐蚀迭代次数
      dilate_kernel: 9           # 膨胀核大小
      dilate_iterations: 1       # 膨胀迭代次数
```

### 订阅的话题

| 话题名称 | 消息类型 | 说明 | 配置参数 |
|---------|---------|------|----------|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图（16位，原始深度值） | `depth_image_topic` |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图（BGR8编码） | `color_image_topic` |

**QoS配置：**
- 可靠性：RELIABLE
- 持久性：VOLATILE
- 历史：KEEP_LAST
- 队列深度：1（只保留最新一帧，触发拍照模式）

**自定义话题（可选）：**
```bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py \
  depth_image_topic:=/custom/depth/topic \
  color_image_topic:=/custom/color/topic
```

## 目录结构

```
visual_pose_estimation_python/
├── visual_pose_estimation_python/  # Python模块
│   ├── __init__.py
│   ├── main.py                     # 主节点
│   ├── config_reader.py            # 配置读取器
│   ├── preprocessor.py             # 预处理器
│   ├── feature_extractor.py        # 特征提取器
│   ├── template_standardizer.py    # 模板标准化器
│   ├── pose_estimator.py           # 姿态估计器
│   ├── ros2_communication.py       # ROS2通信
│   └── debug_visualizer.py         # 调试可视化器
├── launch/                         # 启动文件
│   └── visual_pose_estimation_python.launch.py
├── configs/                        # 配置文件
│   ├── default.yaml
│   ├── hand_eye_calibration.xml
│   └── trigger_depth_thresholds_camera.json
├── test/                           # 核心模块测试
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── examples/                       # 示例文件
│   └── example_usage.py           # 使用示例
├── scripts/                        # 工具脚本
│   └── smart_estimate.py
├── web_ui/                         # Web UI
│   ├── index.html                  # Web UI主页面
│   ├── README.md                   # Web UI文档
│   ├── requirements.txt            # Python依赖
│   ├── configs/                    # Web UI配置文件
│   │   └── debug_thresholds.json
│   ├── scripts/                    # Web UI脚本
│   │   ├── http_bridge_server.py  # HTTP桥接服务器
│   │   ├── params_manager.py      # 参数管理器
│   │   └── import_patch.txt       # 导入补丁
│   ├── docs/                       # Web UI文档
│   │   ├── DEBUG_IMPLEMENTATION_SUMMARY.md
│   │   ├── DEBUG_USAGE.md
│   │   ├── FIX_APPLIED.md
│   │   ├── PROJECT_SUMMARY.md
│   │   ├── QUICK_REFERENCE.md
│   │   ├── TEST_GUIDE.md
│   │   ├── 使用示例.md
│   │   └── 快速开始.md
│   ├── test/                       # Web UI测试
│   │   ├── test_color.py
│   │   ├── test_debug_api.py
│   │   └── test_startup.sh
│   ├── resources/                  # Web UI资源
│   │   └── logo.png
│   ├── start_web_ui.sh            # 启动脚本
│   ├── stop_web_ui.sh             # 停止脚本
│   ├── check_installation.sh      # 安装检查脚本
│   └── start_service_now.sh       # 快速启动脚本
├── resource/                       # ROS2资源文件
│   └── visual_pose_estimation_python
├── package.xml                     # 包配置
├── setup.py                        # Python包配置
├── setup.cfg                       # 配置文件
└── README.md                       # 本文件
```

## 依赖项

### ROS2包依赖
- rclpy
- sensor_msgs
- geometry_msgs
- std_msgs
- cv_bridge
- interface

### Python依赖
- opencv-python (cv2)
- numpy
- PyYAML

## 注意事项

1. **输入要求（重要）**：
   - **深度图和彩色图缺一不可**：必须同时提供深度图和彩色图
   - 深度图用于生成掩模和提取连通域
   - 彩色图用于工件区域提取和特征可视化
   - 缺少任何一个图像都无法正常工作
   
2. **图像同步**：
   - 深度图和彩色图必须是同一时刻拍摄的
   - **推荐使用触发拍照模式**确保图像同步
   
3. **深度阈值**：
   - 需要根据实际场景调整 `binary_threshold_min/max` 参数
   - 默认值 `[0, 65535]` 适用于16位深度图
   - 可在 `configs/default.yaml` 中修改

4. **模板目录**：
   - 默认目录：`/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/templates`
   - 目录结构：`templates/{工件ID}/{姿态ID}/`
   - 首次使用需要创建模板库（使用 standardize_template 服务）
   - 建议为每个工件创建5-10个不同姿态的模板

5. **手眼标定**：
   - 如果有手眼标定文件，通过 `calib_file` 参数传入
   - 标定文件包含：相机内参、畸变系数、手眼变换矩阵
   - 用于将相机坐标系下的姿态转换到机器人基座坐标系

6. **调试模式**：启用debug可以保存中间处理结果

7. **性能优化**：可以通过调整`max_threads`来优化并行处理性能

8. **触发拍照模式**：
   - 节点启动时自动订阅深度图和彩色图话题
   - 调用软触发服务拍照后，直接调用姿态估计服务即可（无需在请求中传递图像）
   - 系统会自动使用触发拍照获取的最新深度图和彩色图
   - 深度图和彩色图都是必需的，缺一不可

## 与C++版本的差异

- **输入格式**：Python版本使用深度图+彩色图混合处理
- **掩模生成**：使用深度图生成掩模，而不是颜色背景去除
- **特征可视化**：在彩色工件图上进行特征可视化
- **算法实现**：完全遵循`trigger_depth.py`的处理流程

## 详细文档

### 核心算法详解

- **[`PROCESS_SINGLE_FEATURE_DETAILED.md`](./PROCESS_SINGLE_FEATURE_DETAILED.md)** - `_process_single_feature` 方法详细说明
  - 面向零基础用户的完整教程
  - 包含基础概念、算法详解、数学公式推导
  - 实际应用示例和常见问题解答
  - 从图像特征到机器人姿态的完整流程说明

## 参考

- C++实现：`visual_pose_estimation`
- 图像处理参考：`trigger_depth.py`

## 许可证

Apache-2.0
