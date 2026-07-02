# IVG2.0 部署文档

本文档记录 IVG2.0 系统在新机器上的完整部署过程，涵盖环境准备、依赖安装、代码获取、编译构建、配置调优和启动验证。

---

## 1. 环境概览

| 项目 | 版本/型号 |
|------|----------|
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| Kernel | 6.8.0-111-generic |
| ROS 2 | Humble Hawksbill |
| MoveIt 2 | 2.5.9 (apt 安装) |
| Python | 3.10.12 |
| CMake | 3.22.1 |
| OpenCV | 4.5.4 |
| Eigen3 | 3.4.0 |
| GPU | NVIDIA GeForce RTX 3090 (GA102, 24GB VRAM) |
| NVIDIA 驱动 | 595.58.03 (CUDA 13.2 驱动层) |
| CUDA Toolkit | 12.6.85 (nvcc 编译器) |
| AUBO 机械臂 | E5 (控制器 IP: 169.254.10.98, 端口: 8899) |
| 知微相机 | Percipio FM830 (depth + color, USB 3.0) |

---

## 2. 系统前置依赖

### 2.1 基础工具

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git curl wget \
  terminator \
  python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-vcstool
```

### 2.2 配置 rosdep

```bash
# 如遇到 raw.githubusercontent.com DNS 污染, 先添加 hosts:
sudo bash -c 'cat >> /etc/hosts << EOF
185.199.108.133 raw.githubusercontent.com
185.199.109.133 raw.githubusercontent.com
185.199.110.133 raw.githubusercontent.com
185.199.111.133 raw.githubusercontent.com
EOF'

sudo rosdep init
rosdep update
```

### 2.3 ROS 2 Humble 桌面完整版

```bash
# 添加 ROS 2 源 (如遇网络问题可换清华镜像)
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2.4 MoveIt 2

```bash
sudo apt install -y ros-humble-moveit
```

### 2.5 相机驱动与视觉依赖

```bash
# OpenCV (percipio_camera 需要 photo 模块)
sudo apt install -y libopencv-dev

# Eigen3 (C++ 坐标变换)
sudo apt install -y libeigen3-dev

# Python 视觉/ML 基础依赖
pip3 install numpy scipy opencv-python Pillow PyYAML trimesh
```

### 2.6 Web 前端栈

```bash
# FastAPI + uvicorn (VPE Web 服务和 Dashboard 网关)
sudo apt install -y python3-fastapi python3-uvicorn

# httpx + websockets (Web Dashboard 网关的 HTTP/WS 代理依赖)
# 注意: colcon build 不会自动执行 pip install，
# 因此 setup.py 中声明的依赖需要手动安装
pip3 install httpx websockets

# Flask (手眼标定 Web UI)
pip3 install Flask

# rosbridge_suite (浏览器 ↔ ROS 2 WebSocket 桥)
sudo apt install -y ros-humble-rosbridge-suite

# tf2_web_republisher (TF 数据 Web 发布)
sudo apt install -y ros-humble-tf2-web-republisher

# web_video_server (MJPEG 视频流)
sudo apt install -y ros-humble-web-video-server

# image_transport + plugins
sudo apt install -y ros-humble-image-transport-plugins
```

### 2.7 仿真用 ros2_control (可选)

```bash
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
```

### 2.8 GPU 驱动与 CUDA 部署

IVG2.0 的 GraspNet（点云 6-DoF 抓取预测）、YOLO OBB（旋转框检测）和 Open3D（点云处理）需要 NVIDIA GPU 加速。不安装驱动则自动回退到 CPU 推理，速度会慢 10-50 倍喵~

#### 2.8.1 确认显卡型号

```bash
lspci | grep -i nvidia
ubuntu-drivers devices | grep -E "driver|recommended"
```

#### 2.8.2 禁用 Nouveau 开源驱动

Nouveau 与 NVIDIA 闭源驱动冲突，必须先禁用：

```bash
sudo tee /etc/modprobe.d/blacklist-nouveau.conf << 'EOF'
blacklist nouveau
options nouveau modeset=0
EOF

sudo update-initramfs -u
sudo reboot
```

> 重启后验证：`lsmod | grep nouveau` 应无输出喵~

#### 2.8.3 安装 NVIDIA 驱动

```bash
sudo apt update
sudo apt install -y nvidia-driver-595
sudo reboot
```

> 验证：`nvidia-smi` 应显示 Driver Version 和 GPU 型号。本项目当前使用 **595.58.03**，自带 CUDA 13.2 驱动层 API 喵~

#### 2.8.4 安装 CUDA Toolkit

GraspNet 的 pointnet2/knn C++ 扩展（`.cu`）需要 nvcc 编译：

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-6
```

在 `~/.bashrc` 末尾添加环境变量：

```bash
echo '
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

source ~/.bashrc
```

> 验证：`nvcc --version` 应显示 release 12.6 喵~

#### 2.8.5 安装 Python GPU 库

```bash
# PyTorch CUDA 版（cu130 = CUDA 13.0 运行时，驱动 13.2 向下兼容）
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130

# Open3D GPU 版
pip3 install open3d

# Ultralytics YOLO（YOLO OBB 推理）
pip3 install ultralytics

# ONNX Runtime GPU 版（可选）
pip3 install onnxruntime-gpu

# matplotlib（YOLO 可视化）
pip3 install matplotlib
```

#### 2.8.6 验证 GPU 加速

```bash
python3 -c "
import torch; print('CUDA:', torch.cuda.is_available(), '| GPU:', torch.cuda.get_device_name(0))
import open3d; print('Open3D CUDA:', open3d.core.cuda.is_available(), '| devices:', open3d.core.cuda.device_count())
import ultralytics; print('ultralytics:', ultralytics.__version__)
import onnxruntime; print('onnxruntime providers:', onnxruntime.get_available_providers())
"
```

预期输出包含 `CUDA: True`、`Open3D CUDA: True`、`TensorrtExecutionProvider` + `CUDAExecutionProvider` 喵~

#### 2.8.7 驱动与项目版本兼容性说明

```
Driver 595 / CUDA 13.2 (底层，nvidia-smi)
  ├── torch 2.12 cu130           ✅ 向后兼容（驱动 ≥ 应用 CUDA 版本即可）
  ├── nvcc 12.6 编译的 .cu 扩展   ✅ 向后兼容
  └── YOLO device=cuda:0         ✅ PyTorch 可用即正常
```

NVIDIA 驱动**严格向下兼容**——驱动支持的 CUDA 版本 ≥ 应用编译版本即可。本项目关键点是 GraspNet 的 pointnet2/knn 扩展需要在当前环境重新 `pip3 install -e .`（详见 [4.1 节](#41-graspnet-模型权重与-cuda-扩展编译)），扩展的预编译产物（`pointnet2/build/` 下）来自旧机器，Python 版本和 CUDA 版本可能不匹配喵~

PyTorch 用户级 pip 安装时动态库 (`libc10.so`, `libtorch.so` 等) 在 `~/.local/lib/python3.10/site-packages/torch/lib/`，不在系统 `ldconfig` 搜索路径。`start_aubo_new_driver.sh` 已自动在 `launch()` 函数中设置 `LD_LIBRARY_PATH`，手动运行时需自行设置喵~

| GPU 使用点 | 组件 | 方式 |
|-----------|------|------|
| GraspNet 推理 | torch 张量运算 | PyTorch CUDA |
| GraspNet pointnet2/knn | .cu C++ 扩展 | nvcc 12.6 编译 |
| YOLO OBB | ultralytics YOLO | PyTorch CUDA, `device=cuda:0` |
| Open3D | 点云处理 | Open3D CUDA |

---

## 3. AUBO SDK 部署

AUBO SDK 的二进制库已随 `aubo_driver_ros2` 包一起管理，位于:

```
aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/lib/
├── lib32/            # 32 位 SDK 库
├── lib64/            # 64 位 SDK 库 + 头文件
│   ├── aubocontroller/   # SDK 动态库
│   ├── config/           # 配置文件
│   ├── deps/             # 依赖库 (protobuf, log4cplus 等)
│   ├── libotgLib.a       # OTG 库
│   └── protobuf/         # protobuf 库
```

**无需手动安装系统级 .deb** — SDK 共享库随 `aubo_driver_ros2` 包的 CMakeLists.txt 自动安装到 colcon install 目录。

### 运行时库路径机制

SDK 的 `.so` 文件（`libauborobotcontroller.so.1`、`liblog4cplus-1.2.so.5`、`libconfig.so.11`、`libprotobuf.so.9` 等）在 `colcon build` 阶段由 CMake `install(DIRECTORY ...)` 规则安装到:

```
install/aubo_driver_ros2/lib/
├── libauborobotcontroller.so.1       # AUBO 控制器 SDK
├── liblog4cplus-1.2.so.5            # 日志库
├── libconfig.so.11 / libconfig++.so.11  # 配置解析库
├── libprotobuf.so.9 / libprotobuf-lite.so.9  # protobuf 序列化
└── ... (含版本化 soname 软链接)
```

运行时 `source install/setup.bash` 会将 `<prefix>/lib` 加入 `LD_LIBRARY_PATH`，驱动节点二进制由此找到所需的共享库。

**双重保障** — `start_aubo_new_driver.sh` 的 `launch()` 函数额外在 `LD_LIBRARY_PATH` 中追加了 SDK 源码头路径（`src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/{aubocontroller,log4cplus,config,protobuf}`），即使未重编译 install 也能直接运行喵~

### 排查验证

```bash
# 验证二进制是否缺少库
source /opt/ros/humble/setup.bash
source install/setup.bash
ldd install/aubo_driver_ros2/lib/aubo_driver_ros2/aubo_dashboard_node | grep "not found"
# 无输出 = 所有库均已找到
```

---

## 4. GraspNet 模型与 YOLO 权重

### 4.1 GraspNet 模型权重与 CUDA 扩展编译

GraspNet 的 VCoT-Grasp 模型权重需放到 `graspnet_ros2/graspnet-baseline/logs/` 目录下。

**CUDA C++ 扩展编译**（graspnet-baseline + pointnet2 / knn / graspnetAPI 四个包）：

```bash
cd aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline

# ⓪ graspnet-baseline — models/utils/dataset 标准包
pip3 install -e .

# ① pointnet2 — PointNet++ CUDA 算子 (ball_query, group_points 等)
cd pointnet2
mkdir -p pointnet2 && touch pointnet2/__init__.py   # 修复上游包结构
pip3 install -e .
cd ..

# ② knn — KNN CUDA 算子
cd knn
mkdir -p knn_pytorch && touch knn_pytorch/__init__.py
pip3 install -e .
cd ..

# ③ graspnetAPI — GraspGroup / NMS / 碰撞检测
pip3 install trimesh
cd graspnetAPI
pip3 install -e .
cd ..

# 验证（无需 sys.path 操作）
LD_LIBRARY_PATH=$HOME/.local/lib/python3.10/site-packages/torch/lib:$LD_LIBRARY_PATH \
  python3 -c "import models; \
              from models.graspnet import GraspNet, pred_decode; \
              from utils.collision_detector import ModelFreeCollisionDetector; \
              from graspnetAPI import GraspGroup; \
              import pointnet2._ext; \
              import knn_pytorch.knn_pytorch; \
              print('OK')"
```

> **为什么需要 `mkdir` + `touch __init__.py`？**  
> 上游 `pointnet2/setup.py` 和 `knn/setup.py` 中扩展名如 `pointnet2._ext`，构建产物在 `build/lib.../pointnet2/_ext.so`。
> 但源码目录下没有 `pointnet2/` 子目录，`pip install -e .` 复制 `.so` 时会报 `No such file or directory`。
> 创建内层包目录后即可正常安装喵~

### 4.2 YOLO OBB 权重

`yolo26n-obb.pt` 权重文件放在工作空间根目录 `aubo_ros2_ws/` 下，由 `vision_perception/yolo_obb.launch.py` 的 `model_path` 参数引用。

---

## 5. 获取代码

```bash
cd /home/mu
git clone https://github.com/MUHAN11-c/aubo_boot.git aubo_boot
cd aubo_boot
git checkout dev
```

LFS 大文件 (如有):

```bash
git lfs pull
```

---

## 6. 网络配置

### 6.1 机械臂连接

AUBO E5 控制器通过以太网直连，默认 IP `169.254.10.98`，端口 `8899`。

```bash
# 验证连接
ping 169.254.10.98
# TCP 端口探测
nc -zv 169.254.10.98 8899
```

启动脚本中的 `_check_robot_reachable()` 会自动进行 TCP 探测:
- **可达** → 真实硬件模式 (AUBO 自定义驱动节点)
- **不可达** → 仿真模式 (ros2_control + mock_components/GenericSystem)

### 6.2 代理配置

本机 Web 服务 (127.0.0.1:8090/8088/8070/8089/9090) 不应走 HTTP 代理。启动脚本已处理:

```bash
# 启动时自动 unset 代理变量 + 设置 NO_PROXY
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
export NO_PROXY="127.0.0.1,localhost,0.0.0.0,::1"
```

---

## 7. 环境变量

启动脚本使用的环境变量及其默认值:

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `AUBO_ROS2_WS` | 脚本所在目录 | 工作空间根路径 |
| `ROS_DISTRO_NAME` | `humble` | ROS 发行版 |
| `AUBO_IP` | `169.254.10.98` | 机械臂控制器 IP |
| `WEB_HOST` | `127.0.0.1` | VPE FastAPI 绑定地址 |
| `WEB_PORT` | `8088` | VPE FastAPI 端口 |
| `WEB_DASH_HOST` | `0.0.0.0` | Web 网关绑定地址 |
| `WEB_DASH_PORT` | `8090` | Web 网关端口 |
| `ROSBRIDGE_PORT` | `9090` | rosbridge WebSocket 端口 |
| `WEB_VIDEO_PORT` | `8089` | MJPEG 视频流端口 |
| `HAND_EYE_PORT` | `8070` | 手眼标定 Web 端口 |
| `IVG_WEB_RELOAD` | `true` | FastAPI 热重载 |
| `IVG_ROSBAG_DIR` | `rosbags/ivg_session` | rosbag 保存目录 |
| `SKIP_RVIZ` | `0` | 设为 1 跳过 RViz2 启动 |
| `SKIP_BUILD` | `0` | 设为 1 跳过 colcon build（快速重启） |
| `SKIP_ROSBAG` | `0` | 设为 1 跳过 rosbag 录制 |

可通过 `export` 覆盖默认值:

```bash
export AUBO_IP=192.168.1.100
export WEB_DASH_PORT=9090
./start_aubo_new_driver.sh
```

---

## 8. 构建

### 8.1 完整构建

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 8.2 选择性构建

修改代码后只需编译相关包:

```bash
# 机械臂驱动相关
colcon build --packages-select aubo_driver_ros2 ivg_interfaces aubo_moveit_config

# 工具快换
colcon build --packages-select tool_changer

# 应用层服务
colcon build --packages-select demo_driver

# 视觉相关
colcon build --packages-select visual_pose_estimation_python graspnet_ros2 hand_eye_calibration

# Web 前端
colcon build --packages-select aubo_ros2_web_dashboard
```

### 8.3 Vue 3 前端构建（新）

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web
npm install
npm run build                     # 产物 → web/dist/
```

FastAPI 网关的静态文件挂载自动指向 `web/dist/`（Vite 构建产物）。

### 8.4 RobotWebTools 前端产物（旧，过渡期保留）

```bash
cd aubo_ros2_ws/src/robotwebtools
bash build_robotwebtools.sh
```

产物输出到 `robotwebtools/runtime_js_assets/`，由 Web Dashboard 的 `robotwebtools_assets_dir` 参数引用。

> **注意**: 新 Vue 3 前端通过 npm 管理 ros3d/roslib 依赖，不依赖 RobotWebTools 产物。旧版原生 JS 页面仍需要此步骤。Vue 3 迁移完成后此步骤可移除喵~

---

## 9. 启动

### 9.1 一键启动 (推荐)

**新框架机械臂 + 全栈**:

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
./start_aubo_new_driver.sh
```

脚本通过 terminator 自动创建标签页，按依赖顺序启动（主动轮询替代固定 sleep）：

| 步骤 | 内容 | 包 | 启动时机 |
|------|------|-----|----------|
| 0 | colcon build 全量构建 | — | 最先执行 |
| 1 | 新框架机械臂 (Controller + Dashboard + StateBroadcaster + MoveIt2) | aubo_moveit_config | 依赖步骤 0 |
| 2 | Demo Driver 基础服务 | demo_driver | 依赖 move_group (30s超时) |
| 15 | IVG Web 网关 (rosbridge + TF + 视频 + FastAPI) | aubo_ros2_web_dashboard | 仅依赖构建产物，与[2]并行 |
| 16 | rosbag 录制 | — | 仅需 ROS 2 运行，与[2]并行 |
| 3-5 | 相机栈: 驱动 + 控制 + 图像桥接 | percipio_camera | 与[6][7]并行 |
| 6-7 | 视觉栈: 手眼标定 + VPE | hand_eye_calibration + VPE | 依赖 /camera/color/image_raw |
| 14 | FastAPI Web | visual_pose_estimation_python | 依赖 /estimate_pose，与[8]并行 |
| 8 | GraspNet 点云抓取预测 | graspnet_ros2 | 依赖相机点云 + 手眼 TF |
| 9-12 | Worker 组: 抓取/快换/拉花/循环 (并行) | demo_driver + tool_changer + coffee_latte | 依赖核心服务就绪 |
| 13 | 综合校验 (ROS 服务 + Web 健康检查) | — | 最后执行 |

> **设计思路**: 步骤 15 (Web Dashboard) 和步骤 16 (rosbag) 仅需构建产物 / ROS 2 运行即可启动，从原末尾位置提前到步骤 2 之后，与相机/视觉栈**并行初始化**。步骤 14 (FastAPI) 提前到步骤 7 之后。这些 Web 服务获得了更充裕的启动时间，health check 超时概率大幅降低喵~ |

### 9.2 分步启动

**仅启动机械臂 + MoveIt2** (不含视觉/Web):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 自动探测: 真机/仿真
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
```

**单独启动 Demo Driver 服务**:

```bash
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

**单独启动工具快换**:

```bash
ros2 launch tool_changer gripper_swap_worker.launch.py
```

### 9.3 Dashboard 生命周期激活

真实硬件模式下，Dashboard 是 LifecycleNode，需手动激活:

```bash
ros2 lifecycle set /aubo_dashboard configure
ros2 lifecycle set /aubo_dashboard activate
```

启动脚本已自动处理此步骤——通过轮询 `/aubo_dashboard` 节点就绪（0.5s 间隔，30s 超时）后再激活，替代了旧版固定 `sleep 3` 的硬编码等待喵~

---

## 10. 访问地址

| 面板 | URL |
|------|-----|
| 门户首页 | `http://127.0.0.1:8090/index.html` |
| 视觉抓取面板 | `http://127.0.0.1:8090/vision_grasp_panel.html` |
| 咖啡拉花面板 | `http://127.0.0.1:8090/coffee_latte_panel.html` |
| TF 监控面板 | `http://127.0.0.1:8090/tf_monitor_panel.html` |
| 设置面板 | `http://127.0.0.1:8090/settings_panel.html` |
| VPE FastAPI | `http://127.0.0.1:8088/` |
| 手眼标定 Web | `http://127.0.0.1:8070/` |

### 局域网设备访问

启动脚本会自动检测本机全局 IPv4 地址并输出局域网访问链接。使用手机/平板等设备时，确保与 PC 在同一 Wi-Fi 下，浏览器打开脚本输出的局域网 URL 即可。

---

## 11. 停止系统

### 关闭所有节点

关闭 terminator 窗口即可停止所有标签页中的进程。或使用以下命令:

```bash
pkill -f 'aubo_moveit_config'
pkill -f 'demo_driver'
pkill -f 'percipio_camera'
pkill -f 'hand_eye_calibration'
pkill -f 'visual_pose_estimation'
pkill -f 'graspnet_ros2'
pkill -f 'gripper_swap_worker'
pkill -f 'latte_io'
pkill -f 'web_dashboard'
pkill -f 'ros2 bag record'
pkill -f move_group
pkill -f rviz2
pkill -f rosbridge_websocket
```

`start_aubo_new_driver.sh` 在步骤 0 已包含旧进程清理逻辑，并通过 `trap cleanup INT TERM` 在 **Ctrl+C 中断时自动终止所有已启动的 ROS 2 进程**，无需手动逐个 pkill 喵~

---

## 12. 验证部署

### 12.1 检查节点

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

预期至少包含: `move_group`, `robot_state_publisher`, `rviz2`, `aubo_dashboard`, `aubo_state_broadcaster`, `joint_trajectory_controller` (真机) 或 `controller_manager` + `ros2_control_node` (仿真)。

### 12.2 检查话题

```bash
ros2 topic list | head -20
```

预期有: `/joint_states`, `/tf`, `/tf_static`, `/aubo_driver/robot_status`, `/aubo_driver/io_states`。

### 12.3 检查服务

```bash
ros2 service list | grep -E "estimate_pose|move_to_pose|run_gripper_swap|change_tool|get_current_tool"
```

### 12.4 检查 TF 树

```bash
ros2 run tf2_tools view_frames
```

预期 TF 链: `base_link` → `...` → `wrist3_Link` → `kuaihuan_Link` → 末端工具。

### 12.5 Web 健康检查

```bash
curl --noproxy '*' -s http://127.0.0.1:8088/health
curl --noproxy '*' -s -o /dev/null -w "%{http_code}" http://127.0.0.1:8090/index.html
```

---

## 13. 常见问题排查

| 现象 | 可能原因 | 检查方法 |
|------|---------|---------|
| 机械臂不可达 | 网线未接或 IP 不正确 | `ping 169.254.10.98 && nc -zv 169.254.10.98 8899` |
| `colcon build` 失败 | 缺少 ROS 2/MoveIt 依赖 | `rosdep install --from-paths src --ignore-src -r -y` |
| `Failed to fetch current robot state` | 单线程 Executor 中长耗时操作阻塞了 jointStateCallback | 使用 MultiThreadedExecutor 或将长耗时服务放入独立的 callback group |
| `bad_weak_ptr` | 构造函数中调用了 `shared_from_this()` | 延迟到回调中首次使用 |
| `Connection refused` | rosbridge 或其他端口被占用 | `ss -tlnp \| grep -E "8090\|8088\|8080\|9090\|8089"` |
| RViz2 不显示机器人模型 | `robot_description` 参数未设置 | `ros2 param get /rviz2 robot_description` |
| GraspNet 无输出 | 相机点云话题不对或模型未加载 | 检查 `/camera/depth_registered/points` 是否有数据 |
| 相机无图像 | 相机未连接或权限不足 | `lsusb`, 检查 udev 规则, 查看 percipio_camera 日志 |
| Web 面板打不开 | 网关未启动或端口被防火墙拦截 | `ss -tlnp \| grep 8090`, 检查防火墙 |
| `exit code 127` / `libauborobotcontroller.so.1: cannot open shared object file` | 未重编译导致 .so 未安装到 `install/lib/`，或未 source `install/setup.bash` | (1) `colcon build --packages-select aubo_driver_ros2` (2) `source install/setup.bash` (3) `ldd install/aubo_driver_ros2/lib/aubo_driver_ros2/aubo_dashboard_node \| grep "not found"` 验证 |
| FastAPI 启动报 `TypeError: types.UnionType` | pydantic 1.8.x 不支持 Python 3.10 的 `X \| Y` union 语法 | 将 FastAPI 路由参数中的 `dict \| None` 改为 `Optional[dict]`（`from typing import Optional`） |
| `ModuleNotFoundError: No module named 'httpx'` | Web Dashboard 依赖未安装 | `pip3 install httpx websockets` |
| FastAPI `/health` 健康检查超时 | 服务在启动阶段崩溃（未启动）或代理干扰 | 检查 terminator 标签页输出日志，确认 `uvicorn` 进程存在; 用 `curl --noproxy '*'` 绕过代理测试 |

---

## 14. 端口分配总览

| 端口 | 服务 | 说明 |
|------|------|------|
| 8090 | IVG Web 网关 (FastAPI) | 静态页 + rosbridge WS 代理 + MJPEG 代理 |
| 8088 | VPE FastAPI | 视觉位姿估计 REST API |
| 8070 | 手眼标定 Flask | 手眼标定 Web UI |
| 8089 | web_video_server | MJPEG 相机视频流 (仅本机网关内部使用) |
| 9090 | rosbridge WebSocket | 浏览器 ↔ ROS 2 桥接 (仅本机网关内部使用) |
| 8899 | AUBO 控制器 | 机械臂 TCP 控制端口 |

---

*最后更新: 2026-05-29 (路径/端口修正 + 包名更新) *
*维护者: muhan11, wangxiaoyun@iscas.ac.cn*
