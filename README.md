# AUBO E5 ROS 2 Jazzy 工作区

面向 AUBO E5 六轴机械臂（老控制器固件，8899 端口旧 SDK v1.3.1）的 ROS 2 Jazzy 驱动。
核心逻辑完全遵循 Humble 实测驱动（aubo_boot）的**一次性下发**模式：整条轨迹一次接收，
硬件侧五次重采样（5ms 点距）→ RIB 水位流控 → TCP2CAN 透传至接口板（5ms/点消费）。
控制器插件为本地自写（参考 UR `PassthroughTrajectoryController` 的写法），经
`trajectory_passthrough` GPIO 契约接入 ros2_control。2026-07-27 起替换原流式 JTC 架构
（旧实现已于 2026-07-29 商业化精简时移除，历史见 git 记录与 `docs/archive/`）。

## 包结构

```
src/
├── aubo_msgs/                  # 自定义消息/服务/action（IOState、RobotStatus、SetIO、
│                               #   GetFK/IK、SetPayload、手眼标定 action/srv）
├── aubo_description/           # E5 工作单元 URDF（table/camera/quick_changer）+
│                               #   ros2_control xacro（接口契约与硬件参数表）
├── aubo_e5_hardware/           # 核心：SystemInterface 真机插件 + 板级模拟器插件 + vendor SDK
├── aubo_e5_controllers/        # AuboPassthroughTrajectoryController + AuboIOController
├── aubo_dashboard/             # 独立服务节点（上电/断电/停止/FK/IK/负载，非运动类）
├── aubo_e5_moveit_config/      # MoveIt 配置与 launch（ompl + pilz 双管线；
│                               #   launch/ 下唯一入口 moveit.launch.py：move_group +
│                               #   rviz2 整体启动，bringup 只 include 它）
├── aubo_e5_bringup/            # launch（唯一入口 bringup.launch.py）+ controllers.yaml
├── aubo_hand_eye_calibration/  # 手眼标定（17 预定义位姿 + 多算法求解 + Web 界面 +
│                               #   unittest 测试套件）
├── aubo_scene_recon/           # 点云场景重建（open3d/TSDF 双后端；venv 节点）
├── peach_pose_msgs/            # 桃子位姿估计的自定义消息
├── peach_pose_ros2/            # 桃子位姿感知（YOLO+MobileSAM+深度几何，venv 节点）
├── peach_reconstruction_ros2/  # 桃子多视角局部重建（自动采帧+Open3D TSDF，venv 节点）
├── peach_gantry_description/   # 【新结构模型，暂不参与】架子式采摘机器人 URDF
├── peach_moveit_config/        # 【新结构模型，暂不参与】架子机 MoveIt 配置
└── percipio_camera/            # 相机驱动（厂商代码；launch 默认值已项目化）
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Python 环境（venv）

工作区统一使用 `aubo_py3.12`（Python 3.12 venv，带 `--system-site-packages`）：

```bash
aubo_py3.12/bin/pip install -r requirements.txt   # 锁定的依赖（numpy<2 等，见文件头注释）
```

ROS 节点分两层运行（约定详见 `AGENTS.md` 第 2/8 节）：

- **纯 ROS 节点**（aubo_hand_eye_calibration）：系统 python3（console_scripts + apt 依赖）
- **torch/open3d 节点**（aubo_scene_recon、peach_pose_ros2、peach_reconstruction_ros2）：
  标准 console_scripts 入口，setup.py 经 `options.build_scripts.executable` 把启动器
  shebang 指向 venv 解释器（构建期解析，换机重建自动适配）；launch 一律标准 `Node()`，
  `ros2 run` 直接可用
- **tools/ 脚本**：一律 `aubo_py3.12/bin/python tools/<脚本>` 运行

注意：venv 内禁止装 opencv-python/scipy（会 shadow 系统版导致 cv_bridge 崩溃），
numpy 必须保持 1.26.x（详见 `AGENTS.md` 第 9 节）。

## 测试

```bash
source /opt/ros/jazzy/setup.bash
colcon test && colcon test-result --verbose          # 全包 lint + 接入的测试（179 项）

# 业务测试（venv 解释器）
cd src/aubo_hand_eye_calibration && ../../aubo_py3.12/bin/python -m pytest test/ -q   # 15+2 例
cd src/aubo_scene_recon && ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/peach_pose_ros2 && PYTHONPATH=peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q   # 42 例
cd src/peach_reconstruction_ros2 && PYTHONPATH=peach_reconstruction_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q   # 44 例
```

## 三种运行模式

通过 `hardware_mode` 切换（xacro 参数 + launch 参数）：

| 模式 | 硬件插件 | 控制器 | 用途 |
|---|---|---|---|
| `mock` | `mock_components/GenericSystem` | `joint_trajectory_controller` | 标准 ros2_control 回归（MoveIt 同样映射到 JTC） |
| `sim` | `aubo_e5_hardware/AuboE5SimHardware` | passthrough + IO | **passthrough 全链路闭环模拟**（无真机） |
| `real`（默认） | `aubo_e5_hardware/AuboE5Hardware` | passthrough + IO + dashboard | 真机 |

```bash
# mock：标准 JTC 链路
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=mock

# sim：板级模拟器，passthrough 闭环
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim

# real：真机（robot_ip 默认 169.254.10.98）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<控制器IP>

# MoveIt(move_group)与 rviz2 默认随 bringup 一起启动；底层调试时可关闭：
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim moveit_enabled:=false

# 只起 MoveIt + rviz2（不起硬件；aubo_e5_moveit_config 的唯一 launch，
# 自带 rsp + joint_state_publisher_gui，纯规划调试/可视化用）：
ros2 launch aubo_e5_moveit_config moveit.launch.py

# 叠加手眼标定（相机与外参静态 TF 默认已随 bringup 启动；Web 界面需显式开）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  hand_eye_enabled:=true hand_eye_web_enabled:=true

# 注意：camera_enabled 默认 true——sim/mock 无相机时建议显式关闭：
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false

# 感知与重建（独立入口，venv 节点 console_scripts 直接启动；详见各自 README）
ros2 launch aubo_scene_recon recon.launch.py            # 点云场景重建（open3d/tsdf）
ros2 launch peach_pose_ros2 peach_pose.launch.py        # 桃子位姿感知（YOLO+SAM）
ros2 run peach_reconstruction_ros2 peach_reconstruction_node   # 桃子多视角局部重建
#   默认全自动：有候选自动开始 → RViz 低速动臂自动采帧 → 满 8 视角自动 finalize，
#   RViz 看 /peach/reconstruction/local_cloud（raw）与 tsdf_cloud（TSDF，RGB8 上色）

# 验证过程数据记录（感知/重建话题 + 相机图 + 点云 PLY + 参数/TF 快照，落盘 validation_runs/）
aubo_py3.12/bin/python tools/peach_validation_recorder.py record --step <步骤名> [--note "备注"]

# 无相机时用离线数据集回放驱动 peach_pose_node 冒烟（独立测试工具，不随构建）
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <数据集根> [--limit N] [--loop]

# 查看任意 launch 的全部参数及中文说明
ros2 launch aubo_e5_bringup bringup.launch.py --show-args
```

sim 插件行为与真机契约一致：传输状态机（0/1/2/3/4/5/6）、五次重采样、虚拟接口板每周期
消费 1 点（200Hz = 5ms/点）、`rib_level`、abort 丢弃队列。执行耗时与真实轨迹时长一致，
可提前暴露 MoveIt 超时/判定问题。

`real` 模式在普通内核直接启动（已取消 RT 预检）；但每次开机需手动执行网卡 offload
关闭 + governor 设置（不持久，命令见 `AGENTS.md` 第 9 节），否则 SDK 推送链路可能出现
>200ms 停滞触发 read() FAULT。

## 轨迹执行链路（real/sim）

```
MoveIt → FollowJointTrajectory goal
  → AuboPassthroughTrajectoryController（remapJointNames 按名重排、blendToFirstPoint
    首点融合、每周期经 GPIO 传 1 个设定点：transfer_state 状态机 6→1→2→…→3）
  → AuboE5Hardware::write()（设定点入 SPSC 队列，回写状态机）
  → 发送线程（非 RT，4ms）：逐段五次重采样为 5ms 点 → RIB 水位流控
    → SetRobotPosData2Canbus 批量透传 → 接口板 5ms/点消费
  → 队列排空 → DONE(5) → 控制器 goal_hold → action succeed
```

停止原语（三分）：正常完成=自然排空；取消/抢占=清双队列 + `RobotMoveStop` 主动丢弃
RIB；急停/防护停=仅停发清队（本体安全回路主导）。

## MoveIt 参数传递

1. **action 连接**（`aubo_e5_moveit_config/config/controllers.yaml`）：
   `controller_names: [aubo_passthrough_trajectory_controller]` +
   `action_ns: follow_joint_trajectory`——MoveIt 不感知背后是 JTC 还是自研控制器。
2. **goal 内动态参数**：`path_tolerance/goal_tolerance/goal_time_tolerance` 可覆盖控制器
   默认容差；关节顺序以 goal 的 joint_names 为准，控制器 remap 到权威顺序。
3. **静态配置**：move_group 侧 `trajectory_execution.*`（蓝本值 5.0/10.0/0.15）；
   控制器侧参数在 `aubo_e5_bringup/config/controllers.yaml`。
4. **速度缩放**：由 MoveIt 时间参数化完成，控制器不做执行期缩放。

## 关键参数

hardware 参数（URDF `<param>`，见 `aubo_description/urdf/aubo_e5.ros2_control.xacro`）：
`send_period_ms=4`、`rib_target=400`、`rib_slowdown_1/2=300/350`、`batch_min/max=2/8`、
`ema_alpha=0.1`、`stop_retry_ms=20`、`auto_power_on=false` 等，默认值与蓝本一致。

控制器参数（`controllers.yaml`）：`goal_tolerance_rad=0.02`、`goal_vel_tolerance=0.01`、
`goal_hold_frames=5`、`goal_check_ms=50`、`blend_threshold_rad=0.01`、`blend_steps=30`。

## 真机分阶段测试（务必按序）

详细命令手册（控制器管理、action/服务示例、分析工具、排障表）见
[docs/usage.md](docs/usage.md)。要点：

1. 现场确认急停/限位/碰撞等级/低速模式；`hardware_mode:=real` 只核对 6 关节名称、方向、
   位置与示教器一致（joint_state_broadcaster），不运动；
2. 上电：示教器手动或 `ros2 service call /aubo_dashboard/startup`（默认
   `auto_power_on=false`，不自动上电）；
3. 速度因子 0.1 的单关节小轨迹；
4. 执行中取消/新 goal 抢占（验证 RIB 被丢弃、余点不继续）；
5. MoveIt 整机轨迹 + trace 分析（tools/motion_analyzer.py）。

## 部署注意事项

- **libprotobuf.so.9**：旧 SDK（libauborobotcontroller.so.1.3.1）强依赖 protobuf 2.6.1，
  已 vendor 在 `aubo_e5_hardware/vendor/lib64/`，RPATH（DT_RPATH）自动解析。
- **SDK 运行时配置**：SDK 按进程 CWD 读 `./config/auborobot.conf` 与
  `tracelog.properties`；launch 已把 `ros2_control_node` 与 `aubo_dashboard_node`
  的 cwd 设为各自 share 目录。
- **TCP2CAN 独占**：激活后 SDK 运动 API 被插件独占，示教器运动暂停；deactivate 后恢复。

制作可迁移的源码归档（排除 `build/`、`install/`、`log/`）：

```bash
./scripts/package_workspace.sh
```

## 参考

- `AGENTS.md` — 开发约定、部署坑点与安全规范（AI 代理向，但内容对人同样适用）
- [docs/usage.md](docs/usage.md) — 完整命令手册与排障表
- 包级 README：[手眼标定](src/aubo_hand_eye_calibration/README.md)、
  [场景重建](src/aubo_scene_recon/README.md)、[桃子位姿](src/peach_pose_ros2/README.md)、
  [桃子多视角重建](src/peach_reconstruction_ros2/README.md)
- [docs/peach_perception_progress.md](docs/peach_perception_progress.md) — 桃子视觉
  三包开发进度台账（Phase 0–7 已做/未做）
- 写法参考：UR `ur_controllers::PassthroughTrajectoryController`
  （GitHub: UniversalRobots/Universal_Robots_ROS2_Driver）
- 行为蓝本：aubo_boot 实测驱动（本机 `/home/mu/Music/e`，不随交付分发）
- 旧架构（流式 JTC）文档见 `docs/archive/`（仅供历史参考）
