# peach_pose_ros2 — PeachPose 桃姿 ROS 2 感知包

订阅 Percipio RGB-D，运行 YOLO + MobileSAM + 实测深度几何管线，发布抓取参考候选
与 RViz Marker。**只发参考位姿，不发送运动指令。**

## 架构与数据流

单节点 `peach_pose_node`，经 `scripts/peach_pose_node` 包装脚本进 venv 运行
（source 式约定同 `AGENTS.md` 第 8 节：`AUBO_PYTHON` 显式解释器 > 已激活的
venv > 自动 `source aubo_py3.12/bin/activate`）。launch 只有一个参数
`python_executable`，非空时作为 `AUBO_PYTHON` 传给包装脚本。

一帧数据的完整管线（`_on_rgbd` 回调）：

```text
RGB(bgr8) + 深度(16UC1) + CameraInfo   ApproximateTime 同步 (slop=sync_slop_s,
  QoS RELIABLE，与数据集回放/相机驱动对齐)
  → cv_bridge 转图；深度 raw × depth_scale_unit → 毫米；尺寸/CameraInfo 一致性校验
  → YOLO 检测（yolo_conf），低于 min_detection_conf 的框丢弃
  → 逐目标 MobileSAM 分割 → hybrid_dilated 前景
      = SAM 掩膜 ∩ 膨胀后的实测深度连通域（SAM 缺失显式 REOBSERVE +
      mask_unavailable，禁止静默深度回退）
  → 按检测类别分流几何管线：
      class 0 袋桃 → RobustBagPosePipeline（圆柱 RANSAC 拟合袋轴）
      class 1 裸桃 → RobustFruitPosePipeline（球拟合 + 梗腔定轴）
  → 刀具几何门控（tool.*：内径/插入深/安全余量）→ 三态
      ACCEPT=0 / REOBSERVE=1 / REJECT=2
  → 几何经 TF 变到 output_frame（默认 base_link，依赖
      hand_eye_extrinsics_publisher；TF 失败退回相机系并告警，不静默用错系）
  → 发布候选 / 2D / 拟合诊断 / 检测 / 掩膜 / Marker / debug 图 / 检测框点云
```

内参始终取本机 `/camera/color/camera_info`（不做 FOV 推导回退，避免与标定
不一致）；几何先在相机光心系求解，再按需变到输出系。

## 依赖（`aubo_py3.12`，**GPU**）

必须保持 `numpy==1.26.4`（与系统 `cv2` / `cv_bridge` 兼容），禁止抬升到
numpy 2.x。YOLO / MobileSAM 走 **CUDA**（本机 RTX 3090，`torch 2.13.0+cu130`）。

依赖全部锁定在仓库根 `requirements.txt`（torch / torchvision / ultralytics /
ultralytics-thop 等）：

```bash
# 从仓库根目录安装
aubo_py3.12/bin/pip install -r requirements.txt

aubo_py3.12/bin/python -c "
import numpy, torch, ultralytics
from cv_bridge import CvBridge
assert numpy.__version__.startswith('1.26')
assert torch.cuda.is_available(), torch.__version__
print(numpy.__version__, torch.__version__, torch.cuda.get_device_name(0))
"
```

注意：ultralytics 会把 `opencv-python` 作为依赖带进 venv（本机已验证与
numpy 1.26.4 / cv_bridge 共存）；**不要手动安装/升级 opencv-python 或
scipy**（会 shadow 系统版）。若 `import cv2` / `cv_bridge` 段错误，按
`requirements.txt` 头注释卸载 venv 内 opencv-python，回退 apt 的
python3-opencv。

图像编解码统一走 **cv_bridge**（`bgr8` / `passthrough` uint16 mm / `mono8`）。

重力参数：`gravity_hint_xyz` 为逗号分隔 `"x,y,z"`；空串表示 `None`（算法默认相机系 +Y）。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_pose_msgs peach_pose_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

### 无相机（数据集回放冒烟）

数据集回放工具已独立为 `tools/peach_dataset_replayer.py`（不随 colcon 构建，
与包内模块无依赖）。用法：

```bash
# 终端 1：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py

# 终端 2：另开终端回放数据集
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <数据集根> [--limit N] [--loop] [--rate 0.5]
```

启动前请确认无残留：`pgrep -af 'peach_pose_ros2|peach_dataset_replayer'`（多实例同时加载 YOLO/SAM 可能导致段错误）。

`--dataset` 缺省为工作区 `src/peach_pose_ros2/data/dataset`（软链到 peach_canopy），
推断失败会报错。回放内参用本机 Percipio（脚本内置 `K_PERCIPIO`，与
`color_camera_info.yaml` 一致）；此时 `depth_scale_unit` 应为 1.0（数据集深度
是真毫米）——节点参数仅在启动时读取，请改 `config/peach_pose.yaml` 后再启动。

### 真相机 RGB-D

```bash
# 终端 1：深度+配准（不改默认 RGB-only launch）
ros2 launch percipio_camera percipio_rgbd.launch.py

# 终端 2：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py
```

或与 bringup 并用。注意 bringup 经 `camera_enabled` 起的是默认
`percipio_camera.launch.py`（RGB-only，深度/点云关）；本包要深度，相机应改由
`percipio_rgbd.launch.py` 提供（bringup 侧给 `camera_enabled:=false`，
避免两个 launch 重复打开同一设备）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
# 另开终端：ros2 launch percipio_camera percipio_rgbd.launch.py
```

## 参数

27 个参数全部带中文 `ParameterDescriptor`（`ros2 param describe /peach_pose_node
<参数>` 可查），权威值在 `config/peach_pose.yaml`（launch 全量加载）。常用：

| 参数 | 默认 | 说明 |
|---|---|---|
| `color_topic` / `depth_topic` / `camera_info_topic` | /camera/color/image_raw 等 | RGB-D 输入（深度须 registration 对齐） |
| `camera_optical_frame` | camera_color_optical_frame | 手眼链所挂光学系；空串用深度图 header.frame_id |
| `output_frame` | base_link | 输出坐标系；空串保持相机系 |
| `tf_timeout_sec` | 0.5 | TF 查询超时 (s) |
| `depth_scale_unit` | 0.25 | raw × 本值 = 毫米（Percipio 常见 0.25）；数据集回放设 1.0 |
| `sync_slop_s` | 0.05 | RGB-D 近似同步允差 (s) |
| `min_detection_conf` / `yolo_conf` | 0.5 / 0.3 | 入管线置信度下限 / YOLO 推理阈值 |
| `publish_debug_image` / `publish_masks` / `publish_detection_cloud` | true | 输出开关 |
| `detection_cloud_stride` | 2 | 检测框点云降采样步长（>1 减轻 RViz 负载） |
| `yolo_model_path` / `sam_model_path` | "" | 空串 = 包内 model/best.pt、model/mobile_sam.pt |
| `model_version` / `calibration_version` | 见 yaml | 随结果发布的模型/内外参版本标识（可追溯） |
| `gravity_hint_xyz` | "" | 重力方向提示 "x,y,z"（相机系）；空串 = 算法默认 +Y |
| `tool.*`（8 个） | 见 yaml | 刀具几何：内径/插入深/刀刃距/入口 standoff/余量/版本号 |

## 话题

| 方向 | 话题 | 类型 |
|------|------|------|
| sub | `/camera/color/image_raw` | `sensor_msgs/Image` bgr8 |
| sub | `/camera/depth/image_raw` | `sensor_msgs/Image` 16UC1 mm（须 registration） |
| sub | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| pub | `/peach_pose_node/grasp_candidates` | `peach_pose_msgs/BagGraspCandidateArray` |
| pub | `/peach_pose_node/grasp_candidates_2d` | `peach_pose_msgs/BagGrasp2DArray` |
| pub | `/peach_pose_node/fitting` | `peach_pose_msgs/BagFittingArray` |
| pub | `/peach_pose_node/detections` | `vision_msgs/Detection2DArray` |
| pub | `/peach_pose_node/masks` | `sensor_msgs/Image` mono8 |
| pub | `/peach_pose_node/markers` | `visualization_msgs/MarkerArray` |
| pub | `/peach_pose_node/debug_image` | `sensor_msgs/Image` bgr8 |
| pub | `/peach_pose_node/detection_cloud` | `sensor_msgs/PointCloud2` xyz+rgb（检测框内深度反投影） |

三态：`ACCEPT=0` / `REOBSERVE=1` / `REJECT=2`。SAM 缺失显式 `mask_unavailable`，禁止静默回退。

## 单测

```bash
cd src/peach_pose_ros2
PYTHONPATH=peach_pose_ros2:$PYTHONPATH \
  /home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python -m pytest test/ -q
```

22 例业务测试（候选/拟合/袋果双管线/球精化/校验）+ flake8/pep257 lint。

## 真机联调前置（本包不运动）

1. 手眼 `hand_eye/active.yaml` 已激活，重启 `extrinsics_publisher`
2. 内参一律本机 Percipio：订阅 `/camera/color/camera_info`（`color_camera_info.yaml` 棋盘标定）；改标定后同步 `inspector/config.py` 的 `K_PERCIPIO` 与 YAML `calibration_version`
3. 深度 `depth_scale_unit:=0.25`（与点云 Z 一致）；registration 打开；本包不发运动

## 模型

`model/best.pt`（约 20 MB）、`model/mobile_sam.pt`（约 41 MB），从
peach_canopy 复制；`yolo_model_path` / `sam_model_path` 留空即按包内
默认路径加载。
