# peach_pose_ros2 — PeachPose 桃姿 ROS 2 感知包

订阅 Percipio RGB-D，运行 YOLO + MobileSAM + 实测深度几何管线，发布抓取参考候选
与 RViz Marker。**只发参考位姿，不发送运动指令。**

## 依赖（`aubo_py3.12`，**GPU**）

必须保持 `numpy==1.26.4`（与系统 `cv2` / `cv_bridge` 兼容）。禁止 `pip install opencv-python`。
YOLO / MobileSAM 走 **CUDA**（本机 RTX 3090，`torch==2.13.0+cu130`）。

```bash
# 从仓库根目录安装（含 cu130 额外索引）
aubo_py3.12/bin/pip install -r requirements.txt
aubo_py3.12/bin/pip install --no-deps 'ultralytics==8.3.0'
aubo_py3.12/bin/pip install pyyaml requests tqdm psutil py-cpuinfo pandas
aubo_py3.12/bin/pip install 'numpy==1.26.4'   # 钉回，防被抬升

aubo_py3.12/bin/python -c "
import numpy, torch, ultralytics
from cv_bridge import CvBridge
assert numpy.__version__.startswith('1.26')
assert torch.cuda.is_available(), torch.__version__
print(numpy.__version__, torch.__version__, torch.cuda.get_device_name(0))
"
```

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
推断失败会报错。
节点经 `scripts/` 包装脚本以 aubo_py3.12 venv 启动；`python_executable` 会作为
`AUBO_PYTHON` 传给包装脚本（空 = venv 默认解释器）。

### 真相机 RGB-D

```bash
# 终端 1：深度+配准（不改默认 RGB-only launch）
ros2 launch percipio_camera percipio_rgbd.launch.py

# 终端 2：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py
```

或与 bringup 并用（相机仍建议用 `percipio_rgbd`，因默认 `camera_enabled` 关深度）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=true
# 另开终端覆盖深度：ros2 launch percipio_camera percipio_rgbd.launch.py
```

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

## 真机联调前置（本包不运动）

1. 手眼 `hand_eye/active.yaml` 已激活，重启 `extrinsics_publisher`
2. 内参一律本机 Percipio：订阅 `/camera/color/camera_info`（`color_camera_info.yaml` 棋盘标定）；改标定后同步 `inspector/config.py` 的 `K_PERCIPIO` 与 YAML `calibration_version`
3. 深度 `depth_scale_unit:=0.25`（与点云 Z 一致）；registration 打开；本包不发运动

## 模型

`model/best.pt`、`model/mobile_sam.pt`（约 59MB，从 peach_canopy 复制）。
