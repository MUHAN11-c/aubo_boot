# 部署与离线 mock 回放

目标平台为 Ubuntu 24.04 x86_64、ROS 2 Jazzy、Python 3.12、Node.js 22 或更高版本。
仓库包含当前源码、机器人资源、厂商 SDK、Peach 检测/分割模型和一份真实 RGB-D 样例。
机器人驱动二进制及相机 SDK 的许可沿用各自供应方条款；其他第三方目录保留自身许可。

## 1. 获取源码与系统依赖

```bash
git clone --branch codex/github-release-20261106 https://github.com/MUHAN11-c/aubo_boot.git
cd aubo_boot
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install python3-venv python3-pip python3-colcon-common-extensions python3-rosdep
# 初次安装 ROS 的机器先执行 sudo rosdep init；已初始化时不重复。
rosdep update --rosdistro jazzy
mapfile -t peach_package_paths < <(colcon list --packages-up-to \
  peach_system_tests peach_web peach_stereo peach_vegetation --paths-only)
rosdep install --from-paths "${peach_package_paths[@]}" --ignore-src --rosdistro jazzy -y
```

ROS Jazzy 按[官方 Ubuntu deb 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)预装。
Node 用 `node --version` 检查主版本至少 22；构建脚本使用仓内 npm 锁文件。
不要 source 其他工作区的 install；无需从本机复制 build、install、venv 或 runs。

## 2. Python 环境

```bash
python3 -m venv --system-site-packages aubo_py3.12
touch aubo_py3.12/COLCON_IGNORE
PYTHONNOUSERSITE=1 aubo_py3.12/bin/python -m pip install --ignore-installed -r requirements-peach.txt
```

`rclpy`、`cv_bridge`、`message_filters`、`rosbag2_py` 等来自 Jazzy apt；不使用 pip 同名替代品。
NumPy 固定 1.26.4，不能升级到 2.x。PyTorch/torchvision 按锁定版本安装；CUDA 可用性取决于主机 NVIDIA 驱动，首次推理包含模型预热。
本部署档只包含 Peach；IVG/rembg 依赖与功能暂不验收。安装时关闭用户 site，并用 `--ignore-installed` 使第三方依赖实际落入 venv。

## 3. 编译与检查

```bash
bash scripts/build_workspace.sh
source scripts/setup_env.bash
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
npm --prefix src/ros2_web_link run check
colcon test --packages-select peach_common peach_vision_core peach_interfaces \
  peach_scene_perception peach_target_reconstruction peach_scene_obstacles peach_supervisor \
  peach_arm peach_bringup peach_observability peach_stereo peach_web peach_system_tests \
  peach_rviz_plugins peach_vegetation serial_force --return-code-on-test-failure
colcon test-result --verbose
```

构建脚本编译 Peach 的完整25 包依赖闭包，同时恢复 SDK 与 Peach 两套前端发行资源；`PEACH_BUILD_WORKERS=2` 为默认并发。
`colcon test` 不代表采摘功能通过，完整 mock 还须运行下一节。
只读驱动包的既有 copyright/cpplint 检查失败单独记录在发布验收文档中，不修改驱动以绕过它们。

## 4. 使用随仓真实数据

```bash
source scripts/setup_env.bash
(cd src/peach/peach_system_tests/data/rgbd_mock_v1 && sha256sum -c SHA256SUMS)
export ROS_DOMAIN_ID=81
export ROS_LOCALHOST_ONLY=1
ros2 run peach_system_tests run_frozen_grasp.py --rviz
```

默认数据位于安装后的 `share/peach_system_tests/data/rgbd_mock_v1/bag`，不依赖当前目录。
Web 为 http://127.0.0.1:8090/，调试页 `/debug.html`；若端口已占用，先结束所属会话，不停止别人的栈。
runner 仅启动 GenericSystem mock、冻结图像、感知/重建、调度和机械臂软件；不连接相机、不启动 dashboard、不输出真实工具 IO。
首次执行前按[测试指导](../src/peach/peach_system_tests/doc/testing.md)盘点进程。runner 收尾会清理自身进程组。

这是固定机位、单视、tool=false 的 L6 干跑；不是多视重建、物理剪切或真实硬件验收。
其测试 overlay 不得用于 real。视觉预抓取位姿及两段去程使用产品现行实现。
显式替换数据时使用 `--bag /absolute/path --index N` 或 `PEACH_FROZEN_BAG`。

## 5. 范围与资源

- Peach 必需模型在 `src/peach/peach_scene_perception/model/`，机器人模型、工具档案与厂商 SDK 随源码提供。首次推理会预热。
- 硬件 IP、串口、手眼标定必须按本机现场设置；随仓标定只适用于对应采集条件，不能移植为新机器的实测外参。
- IVG 暂不处理，其现有源码保留；Peach 的构建/ROS 依赖安装不包含 IVG。Gazebo 当前暂停。
- 历史实验记录保留本机 `runs/data/`，分析在 `runs/analysis_reports/`；本交付样例独立提供来源、原时间戳及校验和。
- 本次构建、功能结果与局限见 [发布验收](releases/2026-11-06.md)。
