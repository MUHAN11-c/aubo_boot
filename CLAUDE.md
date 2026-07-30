# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **完整 AI 代理说明见 [`AGENTS.md`](AGENTS.md)**（294 行，含架构详解、参数表、部署陷阱、安全注意事项）。
> 本文聚焦高频操作与关键约束，是 `AGENTS.md` 的精简入口。

## 项目定位

AUBO E5 六轴机械臂的 ROS 2 Jazzy（Ubuntu 24.04）驱动工作区。
核心架构：**passthrough（一次性下发）**模式 — MoveIt → 自研控制器 → 硬件插件 → TCP2CAN 透传至接口板。
依赖厂商旧 SDK v1.3.1（二进制 vendor，protobuf 2.6.1）。

## 构建与环境

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws

# 全量构建（Release）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 单包构建
colcon build --packages-select aubo_e5_controllers

source install/setup.bash
```

**Python 环境**：所有 Python 脚本一律用 `/home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python`（Python 3.12 venv，numpy 必须 <2，否则 cv_bridge 段错误）。

## 运行模式（`hardware_mode` 参数切换）

| 模式 | 硬件插件 | 控制器 | 用途 |
|---|---|---|---|
| `mock`（默认） | `mock_components/GenericSystem` | `joint_trajectory_controller` | 标准 ros2_control 回归 |
| `sim` | `AuboE5SimHardware` | passthrough + IO | **passthrough 全链路闭环模拟（主力开发模式）** |
| `real` | `AuboE5Hardware` | passthrough + IO + dashboard | 真机 |

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim                # sim 闭环
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=mock               # mock 回归
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<IP> # 真机
ros2 launch aubo_e5_moveit_config moveit.launch.py                              # 纯 MoveIt+RViz（不带硬件）
```

叠加参数：`moveit_enabled:=true/false`、`camera_enabled:=true/false`、`hand_eye_enabled:=true/false`。

## 测试与验证（sim 模式即主验证手段）

项目仅 `aubo_hand_eye_calibration` 有 unittest（15 例），硬件/控制器代码靠 sim 闭环验证：

```bash
# 唯一单元测试套件
cd src/aubo_hand_eye_calibration
/home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python -m pytest test/ -q

# sim 闭环冒烟（改控制器/硬件代码后必做）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder 3
aubo_py3.12/bin/python tools/passthrough_traj_client.py sine_shoulder   # 压测
aubo_py3.12/bin/python tools/motion_analyzer.py run sine_shoulder       # 结果存 test_results/

# 启动任何 launch/测试前必查残留进程
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

## 核心架构速览

```
src/
├── aubo_msgs/                  # 自定义 msg/srv/action
├── aubo_description/           # URDF + ros2_control xacro（接口契约 + 硬件参数）
├── aubo_e5_hardware/           # ★ 核心：SystemInterface 真机插件 + 板级模拟器 + vendor SDK
├── aubo_e5_controllers/        # ★ AuboPassthroughTrajectoryController（FJT action）+ IO 控制器
├── aubo_dashboard/             # 独立服务节点（上电/断电/停止/FK/IK）
├── aubo_e5_bringup/            # bringup.launch.py（唯一入口）+ controllers.yaml
├── aubo_e5_moveit_config/      # MoveIt 配置（ompl + pilz 双管线）
├── aubo_hand_eye_calibration/  # 手眼标定（含 Web 界面 + unittest）
└── percipio_camera/            # 相机驱动（厂商代码，launch 默认值已项目化）
```

数据流：`MoveIt → FJT goal → AuboPassthroughTrajectoryController（重排关节、首点融合、状态机 6→1→2→…→3→5）→ AuboE5Hardware::write()（SPSC 队列入队）→ 发送线程（4ms，五次重采样为 5ms 点，RIB 流控）→ SetRobotPosData2Canbus → 接口板消费`

## 关键开发约束

- **C++17**；注释以中文为主、密度高，修改行为时同步更新注释和 `docs/` 文档
- **控制器参数用 `generate_parameter_library`**：改参数要改 `src/*_parameters.yaml`，不是手写解析
- **硬件插件用已 deprecated 的旧式接口**（`export_state_interfaces/export_command_interfaces`），用 `-Wno-deprecated-declarations` 屏蔽告警——这是能让硬件回写 passthrough 状态机到 command 接口的写法，**不要"升级"**
- **遵循蓝本语义优先**（aubo_boot / UR passthrough）：一次性下发、goal_hold 判定、RIB 流控不要改成流式
- **权威关节顺序**：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`
- 不要向 `build/`、`install/`、`log/` 提交改动

## 高频陷阱

1. **protobuf 兼容**：旧 SDK 强依赖 protobuf 2.6.1（`libprotobuf.so.9`），vendor 在 `aubo_e5_hardware/vendor/lib64/`。链接用 `DT_RPATH`（`--disable-new-dtags`），传递依赖不走 RUNPATH。手动跑二进制需设 `LD_LIBRARY_PATH` 指向 vendor 目录。
2. **SDK 按 CWD 读配置**：`./config/auborobot.conf` 与 `tracelog.properties`；launch 已设 cwd 到 share 目录。
3. **TCP2CAN 独占**：activate 后 SDK 运动 API 被独占，示教器运动暂停。
4. **numpy 版本**：venv 带 `--system-site-packages`，numpy 必须 <2（`numpy==1.26.4`），否则 cv_bridge 段错误。不要 pip 装 opencv-python/scipy（会 shadow 系统版）。
5. **残留进程**：启动前必查 `pgrep -af 'ros2 launch|component_container'`，旧进程会独占设备和 SDK 通道，且跑的是构建前的旧二进制。
6. **网卡 offload（每次开机）**：`sudo ethtool -K enp130s0 gro off gso off tso off` + `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`，否则 SDK 推送链路 >200ms 停滞。
7. **tf2 静态 TF 不覆盖**：更新外参后须重启 `extrinsics_publisher` 进程，同发布者覆盖不传播。

## 安全

- 真机运动测试速度/加速度缩放先压到 **0.1**，确认行为后再放宽
- `auto_power_on=false`（默认），启动不自动上电
- 急停/防护停由本体安全回路主导，软件侧仅停发清队
- `SDK资料/` 与 vendor 二进制为厂商专有资料，不要外传
- SDK 凭据走 xacro 参数，不要把真实密码提交进仓库
