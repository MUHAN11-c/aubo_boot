# visual_pose_estimation_python

Python ROS2 实现的视觉姿态估计功能包（主力版本）。

> **最近更新 (2026-04)**：
> - 统一配置：所有算法参数集中至 `default_config.yaml`（替代散落的硬编码默认值）
> - 消除 `spin_once` 重入：切换至 `MultiThreadedExecutor`，删除服务回调内忙等循环
> - 提取共享模块：`math_utils.py`（旋转/四元数/欧拉角）、`path_resolver.py`（路径解析）
> - 修复缺失的 `web/` 模块：创建兼容存根，解除包导入阻断
> - 消除重复代码：连通域筛选、四元数转换统一为共享实现
> - C++ 旧版标记为 legacy，不再维护

---

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
cd /home/mu/IVG2.0/aubo_ros2_ws
bash start_IVG_graspnet_points_fastapi.sh
```

（脚本位于 **工作空间根** `aubo_ros2_ws/`，与 `src/` 同级；若你的路径不同，请替换 `cd` 目录。）

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
│   ├── main.py                             # 算法 ROS2 节点入口 (MultiThreadedExecutor)
│   ├── config.py                           # ★ 统一配置 (default_config.yaml 加载)
│   ├── config_reader.py                    # 配置兼容层 (委托 config.py)
│   ├── path_resolver.py                    # ★ 路径解析 (模板/标定/相机内参)
│   ├── params_manager.py                   # ★ 参数持久化管理
│   ├── math_utils.py                       # ★ 共享数学工具 (旋转/四元数/欧拉角/连通域筛选)
│   ├── preprocessor.py                     # 深度图预处理
│   ├── feature_extractor.py                # 特征提取
│   ├── template_standardizer.py            # 模板标准化
│   ├── pose_estimator.py                   # 姿态估计核心逻辑
│   ├── ros2_communication.py               # ROS2 服务、订阅、节点协调
│   ├── debug_visualizer.py                 # 调试可视化输出
│   ├── rembg_processor.py                  # 直接 rembg 处理
│   ├── subprocess_rembg.py                 # 子进程 rembg 处理
│   └── web/                                # FastAPI Web 兼容存根 (re-export 自新模块)
├── web_ui/                                 # 可分发资源层
│   ├── index.html                          # 兼容旧版 UI 主页面
│   ├── static/                             # FastAPI 根页静态入口
│   ├── assets/                             # logo 等静态资源
│   ├── configs/                            # 配置文件
│   │   ├── default_config.yaml             # ★ 统一 YAML 配置 (所有算法参数)
│   │   ├── debug_thresholds.json           # 旧阈值文件 (已废弃，保留兼容)
│   │   ├── camera_intrinsics.yaml          # 相机内参
│   │   └── hand_eye_calibration.yaml       # 手眼标定
│   ├── scripts/                            # 辅助脚本，如 rembg 子进程入口
│   ├── tools/                              # 手工验证工具
│   ├── docs/                               # 文档
   │   ├── API_CALL_FLOW.md                # ★ 前后端 API 调用流程 + 调度机制
   │   └── ...
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
└── README.md                               # 当前总说明（含「目录结构标准化说明」章节）
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


---

## 目录结构标准化说明（原 DIRECTORY_STRUCTURE.md）

本文档说明当前 `visual_pose_estimation_python` 如何按 ROS2 `ament_python` 功能包边界收敛。

### 当前推荐结构

```
visual_pose_estimation_python/
├── visual_pose_estimation_python/  # Python模块（核心代码）
│   ├── __init__.py
│   ├── main.py                     # 主节点入口
│   ├── config_reader.py            # 配置读取器
│   ├── preprocessor.py             # 深度图预处理器
│   ├── feature_extractor.py        # 特征提取器
│   ├── template_standardizer.py    # 模板标准化器
│   ├── pose_estimator.py           # 姿态估计器
│   ├── ros2_communication.py       # ROS2通信模块
│   └── debug_visualizer.py         # 调试可视化器
│
├── launch/                          # ROS2启动文件
│   ├── visual_pose_estimation_python.launch.py
│   └── visual_pose_estimation_web.launch.py
│
├── test/                            # 自动化测试
│   ├── test_copyright.py           # 版权测试
│   ├── test_flake8.py              # 代码风格测试
│   ├── test_pep257.py              # 文档风格测试
│   └── test_web_app.py             # Web 回归测试主入口
│
├── web_ui/                          # Web UI界面
│   ├── index.html                  # Web UI主页面
│   ├── README.md                   # Web UI文档
│   ├── requirements.txt            # Python依赖
│   ├── static/                     # FastAPI 根入口静态页
│   ├── assets/                     # 前端静态资源
│   │
│   ├── configs/                    # 默认配置目录
│   │   ├── debug_thresholds.json   # 调试阈值配置
│   │   ├── app_config.json         # Web 运行时配置
│   │   ├── config_paths.md         # 配置路径说明
│   │   ├── camera_intrinsics.yaml  # 相机内参（标准名）
│   │   └── hand_eye_calibration.yaml  # 手眼标定（标准名）
│   │
│   ├── scripts/                    # Web UI辅助脚本
│   │   └── rembg_subprocess.py    # rembg 子进程入口
│   │
│   ├── tools/                      # 手工验证工具
│   │   ├── check_fastapi_startup.sh
│   │   ├── generate_color_channel_demo.py
│   │   └── verify_debug_api.py
│   │
│   ├── docs/                       # Web UI文档
│   │   ├── FASTAPI_DOCS_INDEX.md
│   │   ├── FASTAPI_WEB.md
│   │   ├── FASTAPI_BEGINNER_GUIDE.md
│   │   ├── FASTAPI_MIGRATION_GUIDE.md
│   │   ├── FASTAPI_TESTING_GUIDE.md
│   │   ├── FASTAPI_EXTENSION_GUIDE.md
│   │   ├── FASTAPI_INTERFACE_TEMPLATE.md
│   │   ├── FASTAPI_ARCHITECTURE_DIAGRAMS.md
│   │   ├── FASTAPI_END_TO_END_EXAMPLE.md
│   │   └── 使用示例.md
│   │
│   ├── start_web_ui.sh            # 启动脚本
│   ├── stop_web_ui.sh             # 停止脚本
│   ├── check_installation.sh      # 安装检查脚本
│   └── start_service_now.sh       # 快速启动脚本
│
├── resource/                       # ROS2资源文件
│   └── visual_pose_estimation_python
│
├── .gitignore                      # Git忽略文件
├── package.xml                     # ROS2包配置
├── setup.py                        # Python包配置
├── setup.cfg                       # 配置文件
├── README.md                       # 主文档
└── README.md（本文）                 # 含目录结构标准化说明
```

### 目录说明

#### 核心模块 (`visual_pose_estimation_python/`)
包含所有核心Python模块，实现了视觉姿态估计的主要功能。

#### 配置文件 (`web_ui/configs/`)
存储 Web 默认配置、手眼标定、相机内参与调试阈值。

#### 启动文件 (`launch/`)
ROS2启动文件，用于启动节点。

#### 测试文件 (`test/`)
存放自动化测试；依赖运行中服务的脚本不再放在这里。

#### Web UI (`web_ui/`)
完整的 Web 资源层，包含前端、配置、文档和手工验证工具。

#### 资源文件 (`resource/`)
ROS2包所需的资源文件。

### 标准化操作

已完成的标准化操作：

1. ✅ 补齐 `resource/visual_pose_estimation_python`，回归标准 `ament_python` 元数据
2. ✅ 统一 `web_ui/configs`、静态资源、脚本和模板路径解析入口
3. ✅ 新增 `visual_pose_estimation_web.launch.py`，让 Web 服务拥有独立 launch 入口
4. ✅ 保留 `test/test_web_app.py` 作为 Web 自动回归主入口
5. ✅ 将手工验证脚本从 `web_ui/test/` 挪到 `web_ui/tools/`
6. ✅ 创建 `.gitignore`，忽略缓存和运行产物

### 注意事项

- `__pycache__/` 目录已被 `.gitignore` 忽略
- 运行时文件（如 `.web_ui.pid`）已被 `.gitignore` 忽略
- 所有文档已统一整理到 `web_ui/docs/`
- 手工验证脚本统一放到 `web_ui/tools/`

### 维护建议

1. Web UI相关的文档应放在 `web_ui/docs/` 目录
2. 自动化测试放在 `test/`；手工验证脚本放在 `web_ui/tools/`
3. 配置文件应统一放在相应的 `configs/` 目录
4. 避免在根目录直接放置文件，除非是必需的配置文件


---

## 函数调用检查报告（原 FUNCTION_CALL_REPORT.md）

这份报告原本针对旧的 `http_bridge_server.py` 单体实现整理，现已不再适用。

### 当前有效的 Web 调用主链

1. `visual_pose_estimation_python/web/app.py`
   - 创建 FastAPI 应用
   - 注册 `system`、`camera`、`pose`、`templates`、`robot`、`grasp`、`debug` 路由

2. `visual_pose_estimation_python/web/routers/*.py`
   - 暴露 `/api/*` 与 `/ws`
   - 通过依赖注入调用 `NativeWebService`

3. `visual_pose_estimation_python/web/services/native_api.py`
   - 负责模板、机器人、抓取、debug 等核心 Web 业务逻辑
   - 调用 `RosBridgeManager.node`

4. `visual_pose_estimation_python/web/ros_bridge/manager.py`
   - 管理 ROS2 bridge 生命周期
   - 启动 `visual_pose_estimation_python/web/ros_bridge/node_runtime.py` 中的 `ROS2Node`

5. `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`
   - 封装相机触发、姿态估计、模板标准化、机器人控制、夹爪切换、循环抓取控制等 ROS2 交互

6. `visual_pose_estimation_python/ros2_communication.py`
   - 提供底层 ROS2 服务实现
   - 调用 `pose_estimator.py`、`template_standardizer.py`、`preprocessor.py`、`feature_extractor.py`

### 说明

- 旧 `web_ui/scripts/http_bridge_server.py` 已移除
- 旧 `web_ui/scripts/params_manager.py` 已移除
- 当前 Web 入口应以 FastAPI 与 `visual_pose_estimation_python/web/*` 目录为准


---

## 准备 / 抓取位姿计算（原 GRASP_PREP_POSE_CHECK.md）

本文档梳理**准备姿态**（preparation）与**抓取姿态**（grasp）的完整计算链路、公式与潜在问题。

---

### 1. 数据流概览

```
模板加载 (pose_estimator)
  ├─ grab_position.json / preparation_position.json → T_B_E_grasp, T_B_E_prep
  ├─ camera_pose.json → T_B_E_camera
  ├─ T_E_C (手眼标定)
  └─ T_C_E_grasp = inv(T_B_C_template) @ T_B_E_grasp
     T_C_E_prep   = inv(T_B_C_template) @ T_B_E_prep
     (T_B_C_template = T_B_E_camera @ T_E_C)

估计请求 (ros2_communication → pose_estimator.estimate_pose)
  ├─ T_B_C: 当前相机位姿 (基座→相机)
  ├─ T_B_C_template: 模板相机位姿 (来自 camera_pose + T_E_C)
  ├─ 目标特征: workpiece_center (u,v), 深度图 or 模板深度
  ├─ dtheta: 角度差 (暴力匹配 or standardized_angle 差)
  └─ 输出: result.T_B_E_grasp, result.T_B_E_prep
```

---

### 2. 抓取姿态 (T_B_E_grasp) 计算步骤

**代码位置**: `pose_estimator.py` → `estimate_pose`，约 1358–1504 行。

#### 2.1 输入

| 符号 | 含义 | 来源 |
|------|------|------|
| `T_C_E_grasp_template` | 模板抓取位姿（相机→末端） | `best_template.T_C_E_grasp` |
| `T_B_C_template` | 模板拍摄时基座→相机 | `camera_pose.json` → `T_B_E_camera @ T_E_C`，缺省时用 `T_B_C` |
| `T_B_C` | **当前**拍摄时基座→相机 | `_load_camera_pose`，**当前恒为 `np.eye(4)`** |
| `target_feature` | 当前检测到的工件特征 | `workpiece_center` (u,v)、`workpiece_radius` 等 |
| `dtheta` | 绕基座 Z 轴的角度差 (rad) | 暴力匹配 `best_angle_deg` 或 `target_angle - template_angle` |
| `depth_image` | 深度图 | 请求传入，可选 |
| `camera_matrix` | 相机内参 fx, fy, cx, cy | 配置 |

#### 2.2 步骤 1：模板抓取深度

- `z_template_camera = T_C_E_grasp_template[2, 3]`（相机系 Z）
- 若 `z_template_camera <= 0`：退化为直接用模板位姿转基座，不进行目标相对变换。

#### 2.3 步骤 2：模板抓取转基座

- `T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template`
- `template_grasp_pos = T_B_E_grasp_template[:3, 3]`
- 模板抓取**位置、姿态**均在基座系。

#### 2.4 步骤 3：模板工件中心（基座系）

- 像素：`(template_center_u, template_center_v) = best_template.feature.workpiece_center`
- 相机系：  
  `x = (u - cx) * z_template / fx`，  
  `y = (v - cy) * z_template / fy`，  
  `z = z_template`  
  其中 `z_template = z_template_camera`（即抓取深度）。
- `template_center_base = T_B_C_template @ [x,y,z,1]`，取平移部分 `template_center_base_pos`。

#### 2.5 步骤 4：模板抓取相对模板中心的偏移（基座系）

- `offset_template_base = template_grasp_pos - template_center_base_pos`
- XY 偏移：`template_grasp_offset_xy = [offset[0], offset[1], 0]`（Z 置 0）

#### 2.6 步骤 5：目标工件中心（基座系）

- 像素：`(target_center_u, target_center_v) = target_feature.workpiece_center`
- 深度：优先从 `depth_image` 在 `(u,v)` 邻域取深度；失败则用 `z_template_camera`。
- 相机系：  
  `x = (u - cx) * z_target / fx`，  
  `y = (v - cy) * z_target / fy`，  
  `z = z_target`
- **`target_center_base = T_B_C @ target_center_camera`**  
  当前实现中 **`T_B_C = I`**，故 `target_center_base = target_center_camera`（目标中心被当作在“基座”系，实际未做相机→基座变换）。

#### 2.7 步骤 6：目标抓取位置与姿态

- 绕基座 Z 轴旋转矩阵：`R_z_base = Rz(dtheta)`（标准 2D 旋转）。
- 中间偏移：  
  `intermediate_offset = template_grasp_offset_xy`  
  然后 `rotated_offset = R_z_base @ intermediate_offset`。
- 目标抓取位置：  
  `target_grasp_pos_base = target_center_base + rotated_offset`，  
  再 **强制 `target_grasp_pos_base[2] = template_grasp_pos[2]`**（Z 始终用模板抓取高度）。
- 目标抓取旋转：  
  `R_target_grasp = R_z_base @ R_template_grasp`，  
  其中 `R_template_grasp = T_B_E_grasp_template[:3,:3]`。
- 最终：  
  `T_B_E_grasp_current[:3,:3] = R_target_grasp`，  
  `T_B_E_grasp_current[:3,3] = target_grasp_pos_base`，  
  即 `result.T_B_E_grasp`。

#### 2.8 小结（抓取）

- 逻辑：用模板“抓取相对工件中心的 XY 偏移”+ 模板 Z，经 `dtheta` 旋转后平移到**当前目标中心**，得到目标抓取位姿。
- 前提：**目标中心**、**模板中心**均在**同一坐标系**（基座）下表示。当前 **`T_B_C = I`** 破坏了这一点（见下文）。

---

### 3. 准备姿态 (T_B_E_prep) 计算步骤

**代码位置**: 同上，约 1512–1552 行。

#### 3.1 输入

- 与抓取共用 `T_B_C`、`T_B_C_template`、`target_feature`、`dtheta`、`R_z_base`、`target_center_base_pos` 等。
- 额外：`best_template.T_C_E_prep`（模板准备位姿，相机→末端）。

#### 3.2 步骤

- `T_B_E_prep_template = T_B_C_template @ T_C_E_prep`  
  `prep_pos_template = T_B_E_prep_template[:3, 3]`
- `offset_prep_base = prep_pos_template - template_center_base_pos`（**含 Z**）
- `intermediate_prep_pos = target_center_base + offset_prep_base`  
  `intermediate_prep_offset = intermediate_prep_pos - target_center_base = offset_prep_base`  
  `rotated_prep_offset = R_z_base @ intermediate_prep_offset`
- `target_prep_pos_base = target_center_base + rotated_prep_offset`  
  再 **`target_prep_pos_base[2] = prep_pos_template[2]`**（Z 用模板准备高度）。
- `R_target_prep = R_z_base @ R_template_prep`，  
  `T_B_E_prep_current` 由旋转 + 平移构成，即 `result.T_B_E_prep`。

#### 3.3 小结（准备）

- 与抓取类似：用模板“准备相对模板中心的偏移”（含 Z，但最终 Z 被覆盖为模板值），绕 Z 旋转 `dtheta` 后平移到目标中心。
- 同样依赖 **目标中心** 在 **基座系** 下的正确表示。

---

### 4. 潜在问题与检查结论

#### 4.1 【严重】当前 T_B_C 恒为单位阵

**位置**: `ros2_communication.py` → `_load_camera_pose`。

```python
T_B_C = np.eye(4)  # 当前使用单位矩阵作为相机位姿（未来可从机器人获取）
```

- **影响**：  
  `target_center_base = T_B_C @ target_center_camera` 实际等于 `target_center_camera`。  
  即目标工件中心**未被变换到基座**，而是直接当作基座系坐标使用。
- **后果**：  
  模板相关量（中心、抓取、准备）均在**真实基座系**（由 `T_B_C_template` 定义）；目标中心却在**相机系**。两者混用，导致抓取/准备位姿在基座系下**系统性错误**。
- **正确做法**：  
  眼在手上时，应用 **当前** 机器人位姿 + 手眼标定得到 **当前** `T_B_C = T_B_E_current @ T_E_C`，再对 `target_center_camera` 做变换。  
- **建议**：  
  从机器人/控制器获取当前 `T_B_E`（或等效的相机位姿），在 `_load_camera_pose` 中填充 `T_B_C`，不再使用单位阵。

#### 4.2 目标中心深度回退到模板深度

- 深度图在 `(u,v)` 邻域无有效深度时，使用 `z_template_camera`。  
- 若目标高度/倾斜与模板差异大，会带来误差；属设计上的回退策略，需知悉。

#### 4.3 抓取 / 准备 Z 完全沿用模板

- 抓取：`target_grasp_pos_base[2] = template_grasp_pos[2]`  
- 准备：`target_prep_pos_base[2] = prep_pos_template[2]`  
- 即 **高度** 不随当前场景变化。若工作面或工件高度与模板不一致，可能不合适。  
- 若希望随深度或工作面变化，需额外设计（例如用目标中心 Z、或固定平面约束等）。

#### 4.4 角度差 dtheta 的两种来源

- **暴力匹配**：  
  `dtheta_rad = np.deg2rad(best_angle_deg)`，`best_angle_deg` 为匹配得到的最优角度。  
  若启用暴力匹配，则使用该值。
- **特征角度差**：  
  `dtheta = target_feature.standardized_angle - template_angle`。  
  若未启用暴力匹配或未提供 `dtheta_rad`，则用此式。  
- 两者需与模板匹配策略一致，避免混用导致朝向偏差。

#### 4.5 四元数取反与笛卡尔输出

- `_convert_transform_to_cartesian_position` 中，对旋转矩阵得到的四元数 **整体取反** 再写入 `CartesianPosition.orientation`，与 C++ 对齐。
- `q` 与 `-q` 表示同一旋转，但若下游（如 MoveIt、机械臂接口）对四元数符号有约定，需保持一致。

#### 4.6 抓取/准备 关节角为 0

- 估计结果仅输出笛卡尔位姿（位置 + 四元数 / RPY），**不做逆解**。  
- 故 `joint_position_deg` / `joint_position_rad` 对抓取、准备**恒为 0**，属当前设计；preplace/place 若来自模板 JSON，可保留模板关节角（见前序修改）。

---

### 5. 公式速查

- **模板抓取转基座**：  
  `T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template`
- **模板工件中心（基座）**：  
  `p_cam = [(u-cx)*z/fx, (v-cy)*z/fy, z]` → `p_base = T_B_C_template @ p_cam`
- **目标工件中心（基座）**：  
  `p_cam` 同理，深度可用深度图或模板 → **`p_base = T_B_C @ p_cam`**（当前 `T_B_C=I` 即未变换）
- **抓取 XY**：  
  `offset_xy = (template_grasp - template_center)_xy`，  
  `target_grasp_xy = target_center_xy + Rz(dtheta) @ offset_xy`  
  **抓取 Z**：`template_grasp_z`
- **准备**：  
  同思路，偏移用 `prep - template_center`，旋转后加至 `target_center`，**Z 用 `prep_pos_template[2]`**

---

### 6. 建议修改优先级

1. **高**：  
   实现并接入 **当前** `T_B_C`（如从机器人读取 `T_B_E`，再 `T_B_C = T_B_E @ T_E_C`），替换 `_load_camera_pose` 中的单位阵。  
   否则准备/抓取位姿在基座系下始终存在坐标系错误。
2. **中**：  
   评估抓取/准备 Z 是否必须随场景变化；若需要，再设计基于深度或工作面的 Z 修正。
3. **低**：  
   统一并显式约定 dtheta 来源（暴力匹配 vs 特征角）、四元数符号与下游接口的一致性。

---

*文档版本：基于当前 `pose_estimator` 与 `ros2_communication` 实现整理。*


---

## FastAPI Web UI 文档索引

`web_ui/docs/` 下文档体量较大，保持独立文件；入口与总索引见：

- [`web_ui/docs/FASTAPI_ARCHITECTURE_DIAGRAMS.md`](web_ui/docs/FASTAPI_ARCHITECTURE_DIAGRAMS.md)
- [`web_ui/docs/FASTAPI_BEGINNER_GUIDE.md`](web_ui/docs/FASTAPI_BEGINNER_GUIDE.md)
- [`web_ui/docs/FASTAPI_DOCS_INDEX.md`](web_ui/docs/FASTAPI_DOCS_INDEX.md)
- [`web_ui/docs/FASTAPI_END_TO_END_EXAMPLE.md`](web_ui/docs/FASTAPI_END_TO_END_EXAMPLE.md)
- [`web_ui/docs/FASTAPI_EXTENSION_GUIDE.md`](web_ui/docs/FASTAPI_EXTENSION_GUIDE.md)
- [`web_ui/docs/FASTAPI_INTERFACE_TEMPLATE.md`](web_ui/docs/FASTAPI_INTERFACE_TEMPLATE.md)
- [`web_ui/docs/FASTAPI_MIGRATION_GUIDE.md`](web_ui/docs/FASTAPI_MIGRATION_GUIDE.md)
- [`web_ui/docs/FASTAPI_TESTING_GUIDE.md`](web_ui/docs/FASTAPI_TESTING_GUIDE.md)
- [`web_ui/docs/FASTAPI_WEB.md`](web_ui/docs/FASTAPI_WEB.md)
- [`web_ui/docs/使用示例.md`](web_ui/docs/使用示例.md)

配置路径说明见 [`web_ui/configs/config_paths.md`](web_ui/configs/config_paths.md)；Web 概览见 [`web_ui/README.md`](web_ui/README.md)。


## 参考

- C++ 实现包：`visual_pose_estimation`（同工作空间）
- 图像处理参考：`trigger_depth.py`
- 目录结构、调用链与抓取位姿推导：见本文上方各节（由原 `DIRECTORY_STRUCTURE.md`、`FUNCTION_CALL_REPORT.md`、`GRASP_PREP_POSE_CHECK.md` 合并）

## 许可证

Apache-2.0
