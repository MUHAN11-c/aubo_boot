# visual_pose_estimation_python

Python ROS2实现的视觉姿态估计功能包

## 第一节：配置文件、路径规则与快速使用

这一节优先说明新同学最关心的三件事：

1. 配置文件分别放在哪里、做什么。
2. 模板目录、标定文件、Web 资源是按什么规则解析的。
3. 拿到包之后，最快怎么启动和联调。

### 配置文件说明

当前推荐把运行时配置统一放在 `web_ui/configs/`：

- `web_ui/configs/app_config.json`
  - Web/FastAPI 运行时总配置。
  - 目前主要控制 `template_root` 和 `camera_pose_fixed_orientation`。
- `web_ui/configs/debug_thresholds.json`
  - 深度图阈值、连通域筛选、边缘平滑等调试参数。
  - Web debug 能力和算法调参都依赖它。
- `web_ui/configs/camera_intrinsics.yaml`
  - 相机内参默认文件。
  - 算法节点未显式指定内参时，优先从这里读取。
- `web_ui/configs/hand_eye_calibration.yaml`
  - 手眼标定默认文件。
  - launch 未传 `calib_file` 时，默认先从这里找。
- `web_ui/configs/config_paths.md`
  - 配置与路径约定说明，方便二次开发时查阅。

### 路径规则

路径统一由 `visual_pose_estimation_python/web/resources.py` 解析，避免源码路径、安装路径和本机绝对路径混用。

#### 模板目录 `template_root` 的解析优先级

1. launch 参数 `template_root:=...`
2. 环境变量 `VPE_TEMPLATE_ROOT`
3. `web_ui/configs/app_config.json` 中的 `template_root`
4. 工作空间中的 `visual_pose_estimation/templates`

#### 标定和相机内参的解析规则

- 手眼标定优先顺序：
  - `web_ui/configs/hand_eye_calibration.yaml`
  - `web_ui/configs/hand_eye_calibration*.yaml`
  - `hand_eye_calibration` 包 share 目录中的标定结果
- 相机内参优先顺序：
  - `web_ui/configs/camera_intrinsics.yaml`
  - `web_ui/configs/ost.yaml`
  - `hand_eye_calibration` 包 share 目录中的 `ost.yaml`

#### Web 资源的解析规则

- Python Web 代码在 `visual_pose_estimation_python/web/`
- 前端、文档、配置、脚本在 `web_ui/`
- 运行时优先使用安装后的 `share/visual_pose_estimation_python/web_ui/...`
- 如果当前是源码开发态，再自动回退到源码目录 `web_ui/...`

### 快速使用

#### 1. 编译包

```bash
cd <your_ros2_workspace>
colcon build --packages-select visual_pose_estimation_python
source install/setup.bash
```

#### 2. 只启动算法节点

```bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

#### 3. 只启动 FastAPI Web

```bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py
```

默认访问地址：

- `http://127.0.0.1:8088/`
- `http://127.0.0.1:8088/legacy-ui/index.html`

#### 4. 启动完整系统

```bash
cd <your_ros2_workspace>
bash start_IVG_graspnet_points_fastapi.sh
```

#### 5. 常用自定义启动方式

自定义模板目录和手眼标定：

```bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py \
  template_root:=/path/to/templates \
  calib_file:=/path/to/hand_eye_calibration.yaml
```

自定义 FastAPI 监听地址和端口：

```bash
ros2 run visual_pose_estimation_python visual_pose_estimation_web --host 0.0.0.0 --port 8088
```

使用环境变量覆盖模板目录：

```bash
export VPE_TEMPLATE_ROOT=/path/to/templates
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

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
cd <your_ros2_workspace>
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
  calib_file:=/path/to/hand_eye_calibration.yaml
```

**默认配置：**
- 模板目录：由 launch 参数 `template_root` 指定；为空时按包资源规则自动解析，默认回退到工作空间中的 `visual_pose_estimation/templates`
- 标定：未指定 `calib_file` 时从 `web_ui/configs/` 或 hand_eye_calibration 标准路径查找
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

节点参数来自 **ConfigReader 默认值** 与 **web_ui/configs/debug_thresholds.json**（二值化、连通域等阈值）。标定与相机内参未指定时从 `web_ui/configs/`（标准文件名：**camera_intrinsics.yaml**、**hand_eye_calibration.yaml**）或 hand_eye_calibration 标准路径加载。

### 关键参数（debug_thresholds.json）

- **binary_threshold_min / max**：深度二值化阈值  
- **component_***：连通域筛选（面积、宽高比、数量等）  
- 特征提取与姿态估计的其余默认值在 ConfigReader 中写死，可按需改代码或扩展从文件加载

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

## 文件架构与含义

```text
visual_pose_estimation_python/
├── visual_pose_estimation_python/          # Python 源码层
│   ├── __init__.py                         # 包导出与延迟导入
│   ├── main.py                             # 算法 ROS2 节点入口
│   ├── config_reader.py                    # 算法侧默认配置读取
│   ├── preprocessor.py                     # 深度图预处理
│   ├── feature_extractor.py                # 特征提取
│   ├── template_standardizer.py            # 模板标准化
│   ├── pose_estimator.py                   # 姿态估计核心逻辑
│   ├── ros2_communication.py               # ROS2 服务、订阅、节点协调
│   ├── debug_visualizer.py                 # 调试可视化输出
│   ├── rembg_processor.py                  # 直接 rembg 处理
│   ├── subprocess_rembg.py                 # 子进程 rembg 处理
│   └── web/                                # FastAPI Web 后端
│       ├── app.py                          # FastAPI 应用创建
│       ├── server.py                       # uvicorn 启动入口
│       ├── resources.py                    # 统一路径/资源解析入口
│       ├── runtime_support.py              # Web 运行时辅助逻辑
│       ├── params_manager.py               # Web 参数持久化
│       ├── dependencies.py                 # FastAPI 依赖注入
│       ├── routers/                        # 路由层
│       ├── services/                       # 业务服务层
│       ├── ros_bridge/                     # FastAPI 与 ROS2 bridge
│       └── ws/                             # WebSocket 管理
├── web_ui/                                 # 可分发资源层
│   ├── index.html                          # 兼容旧版 UI 主页面
│   ├── static/                             # FastAPI 根页静态入口
│   ├── assets/                             # logo 等静态资源
│   ├── configs/                            # 配置、标定、阈值文件
│   ├── scripts/                            # 辅助脚本，如 rembg 子进程入口
│   ├── tools/                              # 手工验证工具
│   ├── docs/                               # 学习、迁移、测试文档
│   ├── start_web_ui.sh                     # 启动 Web 服务脚本
│   ├── stop_web_ui.sh                      # 停止 Web 服务脚本
│   ├── check_installation.sh               # 安装检查脚本
│   └── start_service_now.sh                # 快速包装启动脚本
├── launch/                                 # ROS2 启动层
│   ├── visual_pose_estimation_python.launch.py
│   └── visual_pose_estimation_web.launch.py
├── test/                                   # 自动化测试层
│   ├── test_copyright.py
│   ├── test_flake8.py
│   ├── test_pep257.py
│   └── test_web_app.py                     # FastAPI Web 回归主入口
├── resource/                               # ament index 资源登记
│   └── visual_pose_estimation_python
├── package.xml                             # ROS2 功能包元数据
├── setup.py                                # Python 安装与资源分发
├── setup.cfg                               # 脚本安装路径配置
├── DIRECTORY_STRUCTURE.md                  # 目录结构补充说明
└── README.md                               # 当前总说明
```

### 架构分层理解

#### 1. 源码层：`visual_pose_estimation_python/`

这是功能包真正会被 Python 导入的代码层，可以再分成两条主线：

- 算法与 ROS2 主链
  - `main.py`
  - `ros2_communication.py`
  - `pose_estimator.py`
  - `template_standardizer.py`
  - `preprocessor.py`
  - `feature_extractor.py`
- Web 后端主链
  - `web/app.py`
  - `web/routers/*.py`
  - `web/services/native_api.py`
  - `web/ros_bridge/manager.py`
  - `web/ros_bridge/node_runtime.py`

也就是说，算法服务和 FastAPI Web 都已经收口在同一个 `ament_python` 包里。

#### 2. 资源层：`web_ui/`

这一层不负责核心算法执行，而是负责：

- 前端页面和静态资源
- Web/FastAPI 运行配置
- 标定和调试阈值文件
- 开发与验证辅助脚本
- 学习文档与迁移文档

它最终会通过 `setup.py` 被安装到 `share/visual_pose_estimation_python/web_ui/...`，供运行时统一读取。

#### 3. 启动层：`launch/`

- `visual_pose_estimation_python.launch.py`
  - 启动视觉姿态估计 ROS2 节点
- `visual_pose_estimation_web.launch.py`
  - 启动 FastAPI Web 服务

两者分开后，算法和 Web 可以独立调试，也可以在完整系统脚本里一起启动。

#### 4. 测试层：`test/`

自动化测试统一放在包根 `test/`：

- 代码规范测试：`test_flake8.py`、`test_pep257.py`
- Web 行为回归：`test_web_app.py`

依赖运行中服务或人工观察的内容，则放在 `web_ui/tools/`，避免和 pytest 自动测试混淆。

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
   - 可在 `web_ui/configs/debug_thresholds.json` 中修改

4. **模板目录**：
   - 默认目录：工作空间中的 `visual_pose_estimation/templates`
   - 目录结构：`templates/{工件ID}/{姿态ID}/`
   - 首次使用需要创建模板库（使用 standardize_template 服务）
   - 建议为每个工件创建5-10个不同姿态的模板

5. **手眼标定**：
   - 如果有手眼标定文件，通过 `calib_file` 参数传入
   - 标定文件包含：相机内参、畸变系数、手眼变换矩阵
   - 用于将相机坐标系下的姿态转换到机器人基座坐标系

6. **性能优化**：可以通过调整`max_threads`来优化并行处理性能

7. **触发拍照模式**：
   - 节点启动时自动订阅深度图和彩色图话题
   - 调用软触发服务拍照后，直接调用姿态估计服务即可（无需在请求中传递图像）
   - 系统会自动使用触发拍照获取的最新深度图和彩色图
   - 深度图和彩色图都是必需的，缺一不可

## 与C++版本的差异

- **输入格式**：Python版本使用深度图+彩色图混合处理
- **掩模生成**：使用深度图生成掩模，而不是颜色背景去除
- **特征可视化**：在彩色工件图上进行特征可视化
- **算法实现**：完全遵循`trigger_depth.py`的处理流程

## 参考

- C++实现：`visual_pose_estimation`
- 图像处理参考：`trigger_depth.py`
- [目录结构说明](DIRECTORY_STRUCTURE.md)
- [准备/抓取位姿计算说明](GRASP_PREP_POSE_CHECK.md)

## 许可证

Apache-2.0
