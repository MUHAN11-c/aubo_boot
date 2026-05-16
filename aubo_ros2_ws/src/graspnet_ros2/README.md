# GraspNet ROS2

GraspNet-1Billion 6-DOF 抓取预测的 ROS 2 封装。订阅 `PointCloud2`，推理抓取位姿，发布 `PoseArray` + MarkerArray + 动态 TF。

## IVG 系统位置

```
start_aubo_new_driver.sh
  → [8] ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py \
          launch_camera:=false launch_hand_eye_tf:=true
```

点云由 Percipio 相机栈 (步骤 [3-5]) 通过 `/camera/depth_registered/points` 输入，手眼 TF 由 `hand_eye_static_tf_node` 发布。

## 初次安装

### 1. 系统依赖

```bash
# ROS 2 (humble)
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-visualization-msgs \
  ros-humble-geometry-msgs ros-humble-tf2-ros ros-humble-cv-bridge

# Python 基础包 (用户级 pip)
pip3 install torch torchvision open3d scipy Pillow numpy trimesh
```

### 2. 安装 graspnet-baseline + CUDA 扩展

GraspNet 依赖 4 个独立的 pip 包，源码均在 `graspnet-baseline/` 下，均**不是 ROS 包**，需分别 `pip install -e .` 喵~

推荐在新电脑克隆项目后直接执行安装脚本；脚本会用自身目录定位 `graspnet-baseline`，可从任意目录运行喵~

```bash
cd aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline
./install_graspnet_deps.sh --clean --install-common-deps
```

若只想重编本仓库内的 editable 包与 CUDA 扩展，不改动已有 Python 通用依赖，可省略 `--install-common-deps` 喵~

```bash
./install_graspnet_deps.sh --clean
```

脚本内部执行的等价手工步骤如下，供排查时对照喵~

```bash
GRASPNET_BASELINE="$(pwd)/graspnet-baseline"

# ⓪ graspnet-baseline — models/utils/dataset 包（消除 sys.path 运行时注入）
cd "$GRASPNET_BASELINE"
pip3 install -e .

# ① pointnet2 — PointNet++ CUDA 算子 (ball_query, group_points, sampling 等)
cd "$GRASPNET_BASELINE/pointnet2"
mkdir -p pointnet2 && touch pointnet2/__init__.py
pip3 install -e .

# ② knn — KNN CUDA 算子
cd "$GRASPNET_BASELINE/knn"
mkdir -p knn_pytorch && touch knn_pytorch/__init__.py
pip3 install -e .

# ③ graspnetAPI — GraspGroup / NMS / 碰撞检测（推理必需）
pip3 install trimesh
cd "$GRASPNET_BASELINE/graspnetAPI"
pip3 install -e .
```

> **为什么需要 `mkdir` + `touch __init__.py`？**  
> 上游 `pointnet2/setup.py` 和 `knn/setup.py` 中扩展名如 `pointnet2._ext`，构建产物在
> `build/lib.../pointnet2/_ext.so`。但源码目录下没有 `pointnet2/` 子目录，`pip install -e .`
> 复制 `.so` 时会报 `No such file or directory`。创建内层包目录后即可正常安装喵~
>
> `graspnet-baseline/setup.py` 为 IVG2.0 自建（上游无此文件），`models/__init__.py` 自动处理
> 上游代码的绝对导入兼容性问题，节点侧无需 `sys.path.insert` 喵~
>
> `pointnet2/setup.py` 的普通 C++ 编译参数必须只使用 GCC/Clang 可识别的参数；`-Xcompiler`
> 是 NVCC 专用参数，放入 `extra_compile_args["cxx"]` 会导致 `c++: error: unrecognized
> command-line option '-Xcompiler'`，最终没有生成 `pointnet2._ext` 喵~
>
> `install_graspnet_deps.sh` 会检查 `python3`、`pip`、`c++`、`ninja`、`nvcc`、`torch`、`torch.cuda.is_available()`、
> `pointnet2/setup.py`、`knn/setup.py`、`graspnetAPI/setup.py` 等前置条件；无显示 GPU 编译时需显式设置
> `TORCH_CUDA_ARCH_LIST`，例如 `TORCH_CUDA_ARCH_LIST="8.9" ./install_graspnet_deps.sh --clean` 喵~

### 3. 验证导入链

```bash
LD_LIBRARY_PATH=$HOME/.local/lib/python3.10/site-packages/torch/lib:$LD_LIBRARY_PATH python3 -c "
import models
from models.graspnet import GraspNet, pred_decode
from utils.collision_detector import ModelFreeCollisionDetector
from graspnetAPI import GraspGroup
import pointnet2._ext as _ext
import knn_pytorch.knn_pytorch

print(f'OK — CUDA: {torch.cuda.is_available()}, GPU: {torch.cuda.get_device_name(0)}')
print(f'pointnet2 算子: {[x for x in dir(_ext) if not x.startswith(\"_\")]}')
"
```

### 4. 运行时 LD_LIBRARY_PATH

PyTorch 用户级 pip 安装时动态库 (`libc10.so`, `libtorch.so` 等) 在 `~/.local/lib/python3.10/site-packages/torch/lib/`，不在系统 `ldconfig` 搜索路径。

`start_aubo_new_driver.sh` 的 `launch()` 函数中已自动设置，**无需手动配置**：

```bash
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/torch/lib:$LD_LIBRARY_PATH"
```

### 5. colcon build

```bash
cd aubo_ros2_ws
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

## 节点

### `graspnet_demo_points_node`

核心推理节点。订阅 `PointCloud2`，定时循环执行：点云→推理→碰撞检测→NMS→发布。

| 项目 | 默认值 |
|------|--------|
| 订阅 | `input_pointcloud_topic` (`/camera/depth_registered/points`) |
| 发布 Marker | `marker_topic` (`grasp_markers`) |
| 发布抓取 | `grasp_poses_topic` (`grasp_poses_base`, `geometry_msgs/PoseArray`，`frame_id=base_link`) |
| 动态 TF | `camera_frame` → `grasp_pose_i` |
| 循环间隔 | `compute_interval_sec` (1.0) |
| 采集控制 | `/graspnet_capture_control` (`std_srvs/SetBool`) |
| 模型路径 | `model_path` (`baseline_dir/logs/log_kn/checkpoint-rs.tar`) |
| 碰撞阈值 | `collision_thresh` |
| 体素大小 | `voxel_size` |

### `hand_eye_static_tf_node`

发布手眼标定静态 TF：`ee_frame_id` → `camera_frame` + `camera_frame` → `camera_link`。

| 参数 | 默认值 |
|------|--------|
| `hand_eye_yaml_path` | hand_eye_calibration 包的 `config/hand_eye_calibration_best.yaml` |
| `ee_frame_id` | `wrist3_Link` |
| `child_frame_id` | `camera_frame` |

### `publish_grasps_client`

订阅 `grasp_poses_base`，窗口选优，通过 MoveIt2 执行笛卡尔抓取运动。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grasp_poses_topic` | `grasp_poses_base` | 抓取位姿输入话题 |
| `grasp_window_size` | 5 | 缓存最近 N 组抓取 |
| `min_groups_before_pick` | 3 | 至少累计 M 组后选最优 |
| `prefer_vertical` | True | True=选最垂直抓取，False=用第一组 |
| `height_above` | 0.05 | 抓取点上方安全高度 (m) |
| `grasp_z_offset` | 0.15 | gripper_tip 相对 wrist3_Link 的 Z 偏移 (m) |
| `joint_velocity_scaling` | 0.15 | 关节空间回退速度 (0~1) |

### `publish_grasps_worker_loop_control_client`

循环抓取工作者的 gRPC 控制客户端。在 `demo_driver` 包中配合 `PublishGraspsClientWorker` 使用喵~

## GraspNet → ROS 坐标系转换

GraspNet 与 ROS 工具坐标约定不同：

- **GraspNet** `rotation_matrix` 列：col0 = approach，col1 = width，col2 = height
- **ROS 末端**：X = width，Y = height，Z = approach

转换代码：

```python
R_graspnet = grasp.rotation_matrix  # (3, 3)
R_ros = np.column_stack([
    R_graspnet[:, 1],  # ROS X = width
    R_graspnet[:, 2],  # ROS Y = height
    R_graspnet[:, 0],  # ROS Z = approach
])
```

> 下游控制基于已发布 TF / `PoseArray`，需与此一致。改一处不改另一处会导致 "RViz 对但臂错" 喵~

## Launch 文件

| 文件 | 用途 |
|------|------|
| `graspnet_demo_points_with_tf.launch.py` | **IVG 生产使用**：仅 GraspNet 节点 + 手眼 TF，move_group 由外部启动 |
| `graspnet_demo_points.launch.py` | 独立测试：含 move_group + RViz2 + 相机 + octomap，自包含 |
| `percipio_camera_calibration.launch.py` | Percipio 相机驱动（被其他 launch 引用） |
| `octomap.launch.py` | 可选 OctoMap 可视化 |

## 目录结构

```
graspnet_ros2/
├── graspnet_ros2/                 # Python 模块
│   ├── __init__.py
│   ├── graspnet_demo_points_node.py   # 点云推理节点
│   ├── grasp_motion_controller.py     # 极简运动控制（move_to_pose / run_grasp_approach）
│   ├── publish_grasps_client.py       # 抓取窗口选优 + 运动执行
│   ├── publish_grasps_worker_loop_control_client.py  # 循环抓取控制
│   └── hand_eye_static_tf_node.py     # 手眼静态 TF 广播
├── graspnet-baseline/             # 上游 GraspNet 模型代码 + pointnet2/knn 扩展
│   ├── models/                    # GraspNet + backbone 模型定义
│   ├── pointnet2/                 # PointNet++ CUDA 扩展 (需 pip install -e .)
│   ├── knn/                       # KNN CUDA 扩展
│   ├── graspnetAPI/               # GraspNet API (GraspGroup, eval)
│   ├── utils/                     # 碰撞检测等工具
│   ├── dataset/                   # 数据集加载
│   └── logs/                      # 预训练权重 (checkpoint-rs.tar)
├── launch/                        # Launch 文件
├── config/                        # RViz / 相机内参 / 传感器配置
├── doc/                           # 仿真参考文档
├── resource/                      # ament index marker
├── test/                          # lint 测试
├── package.xml
├── setup.py
└── setup.cfg
```

## 许可证

MIT License。graspnet-baseline 代码和模型版权归 MVIG, SJTU graspnet 团队所有，免费用于非商业用途喵~
