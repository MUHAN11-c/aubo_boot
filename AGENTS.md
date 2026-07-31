# AGENTS.md — AUBO E5 ROS 2 Jazzy 工作区

> 面向 AI 编码代理的项目说明。阅读者默认对本项目一无所知。
> 项目文档与代码注释以中文为主，本文沿用中文。

## 1. 项目概述

本仓库是一个 ROS 2 Jazzy（Ubuntu 24.04 / noble）工作区，为 **AUBO E5 六轴机械臂**
提供 ros2_control 驱动。机械臂使用老控制器固件（8899 端口，**旧 SDK v1.3.1**，
二进制 vendor 在仓库内，无源码构建）。

核心架构为 **passthrough（一次性下发）** 模式，于 2026-07-27 替换原"标准 JTC 100Hz
流式打点"架构（旧实现已于 2026-07-29 商业化精简时移除，历史见 git 记录与
`docs/archive/`）。逻辑遵循 Humble 实测驱动（aubo_boot）蓝本，写法参考 UR 的
`ur_controllers::PassthroughTrajectoryController`：

```
MoveIt → FollowJointTrajectory goal
  → AuboPassthroughTrajectoryController（按名重排关节、首点 smoothstep（C1）融合，
    每周期经 trajectory_passthrough GPIO 传 1 个设定点，状态机 6→1→2→…→3）
  → AuboE5Hardware::write()（设定点入 SPSC 队列，回写状态机）
  → 发送线程（非 RT，4ms）：五次重采样为 5ms 点 → RIB 水位流控
    → SetRobotPosData2Canbus 批量透传 → 接口板 5ms/点消费
  → 队列排空 → DONE(5) → 控制器 goal_hold → action succeed
```

停止原语分三类：正常完成=队列自然排空；取消/抢占=清双队列 + `RobotMoveStop`
主动丢弃 RIB；急停/防护停=仅停发清队（由本体安全回路主导）。

## 2. 技术栈

- **语言**：C++17（硬件/控制器/dashboard 插件与节点）、Python（launch、tools 脚本）
- **项目 Python 环境**：整个项目的 Python 环境统一为
  `/home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12`（Python 3.12 venv，已装
  matplotlib/numpy）。所有 Python 脚本（tools/、diagnostics/、以及其他任何需要
  调用 Python 的场景）一律用 `aubo_py3.12/bin/python` 运行，不要使用系统
  `python3`；需要新增 Python 包时装进该 venv。venv 的依赖锁定在工作区根目录
  `requirements.txt`（`aubo_py3.12/bin/pip install -r requirements.txt` 可复现）。
  ROS 节点分两层：纯 ROS 节点（aubo_hand_eye_calibration）走系统 python3
  （console_scripts + apt 依赖）；依赖 torch/open3d 的节点（aubo_scene_recon、
  peach_pose_ros2）经包内 `scripts/` 的 bash 包装脚本进 venv 运行（装到
  `lib/<pkg>`，`Node()`/`ros2 run` 按名引用），解释器可用环境变量
  `AUBO_PYTHON` 覆盖（约定详见第 8 节）。
- **框架**：ROS 2 Jazzy + ros2_control（hardware_interface / controller_interface /
  pluginlib / generate_parameter_library）+ MoveIt 2（ompl + pilz 双管线）
- **构建系统**：colcon + ament_cmake（无 pyproject.toml / package.json / Cargo.toml）
- **外部依赖**：vendored AUBO 旧 SDK v1.3.1（`libauborobotcontroller.so`，
  强依赖 protobuf 2.6.1 / libconfig / log4cplus，均已 vendor）

## 3. 仓库结构

```
src/                            # 参与构建的包
├── aubo_msgs/                  # 自定义 msg/srv/action（IOState、RobotStatus、SetIO、
│                               #   GetFK/IK、SetPayload、手眼标定 action/srv）
├── aubo_description/           # E5 工作单元 URDF（table/camera/quick_changer）+
│                               #   aubo_e5.ros2_control.xacro（接口契约 + 硬件参数表）
├── aubo_e5_hardware/           # 核心：SystemInterface 插件
│   ├── src/aubo_e5_hardware.cpp        # 真机插件（双 SDK 连接：conn_control_ 走
│   │                                   #   TCP2CAN 流控发送线程；conn_status_ 走状态
│   │                                   #   推送/安全 IO/停止原语，IO 异步线程）
│   ├── src/aubo_e5_sim_hardware.cpp    # 板级模拟器插件（ursim 等价物，无 SDK 无真机）
│   ├── vendor/                         # SDK 头文件 + lib64 二进制 + 文档
│   └── config/                         # auborobot.conf、tracelog.properties（SDK 按
│                                       #   进程 CWD 读取 ./config/，装到 share）
├── aubo_e5_controllers/        # AuboPassthroughTrajectoryController（FJT action server）
│                               #   + AuboIOController（IO 状态发布 + set_io 服务）
│                               #   参数由 generate_parameter_library 从
│                               #   src/*_parameters.yaml 生成
├── aubo_dashboard/             # 独立服务节点（上电/断电/停止/FK/IK/负载，非运动类）
├── aubo_e5_moveit_config/      # MoveIt 配置与 launch：launch/ 下只有唯一入口
│                               #   moveit.launch.py（真机/仿真共用的整体启动：
│                               #   move_group + rviz2，standalone 时自带 rsp +
│                               #   joint_state_publisher_gui；bringup 只 include 它）；
│                               #   config/ 下 controllers.yaml 与 controllers_mock.yaml
│                               #   两套控制器映射、joint_limits、kinematics、ompl、pilz
├── aubo_e5_bringup/            # bringup.launch.py（唯一入口）+ config/controllers.yaml
├── aubo_hand_eye_calibration/  # 手眼标定（17 预定义位姿 + MoveIt 位姿约束运动；
│                               #   求解算法可选 auto/tsai/park/horaud/andreff/
│                               #   daniilidis + Huber 精化；Web 界面含机器人状态/
│                               #   末端位姿/关节表/逐帧过程/求解公式与残差图表）
├── aubo_scene_recon/           # 点云场景重建（open3d TSDF/累积后端；节点经
│                               #   scripts/recon_fusion_node 包装脚本进 venv 运行）
├── peach_pose_msgs/            # 桃子位姿估计的自定义 msg
├── peach_pose_ros2/            # 桃子位姿估计（YOLO+SAM，依赖 venv 内 torch；
│                               #   节点经 scripts/ 包装脚本启动，同 scene_recon 模式）
├── percipio_camera/            # 相机驱动（厂商代码，两处项目化改动：
│                               #   ① launch 默认值：device_ip=169.254.10.110、
│                               #   depth_enable=true、point_cloud_enable=false、
│                               #   color_point_cloud_enable=true；② 新增
│                               #   color_camera_info_file 参数，默认用
│                               #   config/color_camera_info.yaml（棋盘自标内参）
│                               #   覆盖设备 Flash 标称内参，传 '' 回退）

tools/                          # 分析/测试脚本（不随 colcon 构建）：
                                #   passthrough_traj_client.py（轨迹测试客户端）
                                #   motion_analyzer.py（运动分析工具，单文件单窗口
                                #     图文报告+CSV+PNG：run=脚本轨迹执行分析，
                                #     --real 启用 RIB 判定；rec=RViz 手动运动被动
                                #     录制，逐段 A/B/C/D 四类量化，多段汇总一窗）
                                #   peach_dataset_replayer.py（PeachPose 数据集回放
                                #     冒烟工具：rgb/+depth/ 成对 PNG → RGB-D+CameraInfo）
                                #   fk_ik_check.py、joint_trace_recorder.py、
                                #   m2_m3_retest.py，以及一组链接 vendored SDK 的
                                #   C++ 检查程序（aubo_sdk_{readonly,motion,full,
                                #   speed,jointmove,coexist}_*.cpp）
test_results/                   # motion_analyzer 默认输出目录（PNG 报告 + CSV，
                                #   文件名带时间戳，运行时自动创建）
hand_eye/                       # 手眼标定结果存储（active.yaml + candidates/*.yaml），
                                #   由 storage.py 相对包定位（AUBO_HAND_EYE_DIR 可覆盖），
                                #   不再用 ~/.ros
diagnostics/                    # SDK 探针（独立 CMake 构建，不参与 colcon）：
                                #   aubo_sdk_{abi,runtime,push,tcp2can}_probe.cpp
                                #   （链接 vendored SDK v1.3.1；runtime=轮询延迟/阻塞，
                                #   push=推送频率/空洞，tcp2can=RIB 水位/写延迟，全部零运动）
                                #   run_tests.sh（一键构建+按序运行+绘图）、
                                #   plot_results.py（读 results/*.csv 出 PNG 曲线，
                                #   用 aubo_py3.12/bin/python 运行）
                                #   live_monitor.py（实时监测：与 bringup/RViz2 并行运行，
                                #   只订阅 /joint_states + /aubo_io_controller/rib_status
                                #   + joint_status 话题，不新增 SDK 连接；实时窗口或
                                #   --no-gui 记录，退出出汇总 PNG）
                                #   build/（cmake 产物）、results/（CSV+PNG，每次运行覆盖最新）
docs/                           # 现行文档：usage.md（命令手册）、
                                #   passthrough_migration.md（架构迁移说明）、
                                #   nic_driver_incident.md（网卡驱动事件运维）、
                                #   ur_motion_evaluation_standards.md（运动评估标准）；
                                #   archive/（旧架构文档 6 篇，仅供历史参考）、
                                #   images/（文档配图）
scripts/package_workspace.sh    # 打包可迁移源码归档（排除 build/install/log）
SDK资料/                        # 厂商原始资料（SDK 包、Noetic 参考、文档）
build/ install/ log/            # colcon 产物，勿手动修改
```

无 `AGENTS.md` 子级文件。全部 11 个项目包已接入 lint 测试（ament_cmake 包经
`ament_lint_auto`：copyright/cpplint/uncrustify/lint_cmake/xmllint；ament_python 包带
官方模板 test_flake8.py/test_pep257.py），`colcon test` 强制风格合规；
`aubo_e5_hardware/vendor/` 与 `aubo_dashboard/vendor/` 内含 `AMENT_IGNORE`（ament
工具链跳过厂商代码，不影响编译）。业务测试：aubo_hand_eye_calibration（unittest
15 例）、aubo_scene_recon 与 peach_pose_ros2（pytest，须用 venv 解释器）；
硬件/控制器代码验证靠 sim 模式闭环 + 真机分阶段流程（见第 6 节）。

## 4. 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release     # 全量构建
colcon build --packages-select aubo_e5_controllers        # 只构建某个包
source install/setup.bash
```

## 5. 运行（三种模式）

通过 launch 参数 `hardware_mode` 切换（透传到 xacro）：

| 模式 | 硬件插件 | 控制器 | 用途 |
|---|---|---|---|
| `mock` | `mock_components/GenericSystem` | `joint_trajectory_controller` | 标准 ros2_control 回归；MoveIt 映射到 controllers_mock.yaml |
| `sim` | `aubo_e5_hardware/AuboE5SimHardware` | passthrough + IO | passthrough 全链路闭环模拟，无真机开发/验证首选 |
| `real`（默认） | `aubo_e5_hardware/AuboE5Hardware` | passthrough + IO + dashboard | 真机；普通内核直接启动（已取消 RT 预检） |

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<控制器IP>
```

可叠加 launch 参数（默认 `robot_ip=169.254.10.98`）：`moveit_enabled`（true，
move_group + rviz2）、`camera_enabled`（true，启动 percipio 相机驱动；sim/mock
无相机时建议显式 false）、`extrinsics_enabled`（true，手眼外参静态 TF
wrist3_Link→camera_link）、`hand_eye_enabled`（false，标定采集/求解流程）、
`hand_eye_web_enabled`（false，标定 Web 界面，仅 `hand_eye_enabled:=true` 时生效）。

只起 MoveIt + rviz2（不起硬件，纯规划调试）：`ros2 launch aubo_e5_moveit_config
moveit.launch.py`（自带 rsp + joint_state_publisher_gui；参数 `controllers_file`、
`standalone_state_publishers`，bringup 集成时后者传 false）。

感知/重建为独立入口（venv 包装脚本启动，约定见第 8 节）：
`ros2 launch aubo_scene_recon recon.launch.py`（点云重建）、
`ros2 launch peach_pose_ros2 peach_pose.launch.py`（桃子位姿，仅
`python_executable` 一个参数，作为 `AUBO_PYTHON` 传入）；无相机冒烟用
`aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <根>`。
任意 launch 的全部参数及中文说明可用 `--show-args` 查看。

sim 插件与真机契约一致：传输状态机、五次重采样、虚拟接口板 200Hz 每周期消费
1 个 5ms 点、`rib_level`、abort 丢弃队列；执行耗时与真实轨迹一致，可提前暴露
MoveIt 超时问题。注意 sim 不模拟板载 IO 写回，`set_io` 返回 success=false 属预期。

## 6. 测试与验证策略

全部项目包已接入 lint 测试（见第 3 节），`colcon test` 强制风格合规；业务测试
须用 venv 解释器手动跑（先 source ROS 环境，否则 lint 测试 import 不到 ament_*）：

```bash
source /opt/ros/jazzy/setup.bash
colcon test && colcon test-result --verbose          # 全包 lint + 接入的测试
cd src/aubo_hand_eye_calibration && ../../aubo_py3.12/bin/python -m pytest test/ -q  # 15 例
cd src/aubo_scene_recon && ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/peach_pose_ros2 && PYTHONPATH=peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q
```

硬件/控制器代码的验证方式为：

0. **运行前先查旧进程**（启动任何 launch/测试/探针之前必做）：
   ```bash
   pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
   ```
   有残留先清掉再启动。背景：① 相机等设备被旧进程独占时新 launch 报
   `Open device fail -1014`，且旧进程跑的是**构建前的旧二进制**，会让
   "改动已生效"的判断失真（2026-07-29 实测踩坑）；② 直接 kill launch
   进程会留下孤儿 component_container/节点进程，必须按 PID 补杀；
   ③ SDK 通道（TCP2CAN）与相机一样独占，重开 bringup 前同样要先清。
1. **sim 闭环冒烟**（改控制器/硬件代码后必做）：
   ```bash
   ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
   aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder 3
   aubo_py3.12/bin/python tools/passthrough_traj_client.py sine_shoulder      # 压测重采样/流控
   aubo_py3.12/bin/python tools/motion_analyzer.py run sine_shoulder   # 数据存 test_results/
   ```
   （camera_enabled 默认 true；无相机时 sim 冒烟建议显式关闭，避免驱动刷屏报错。）
   已验证基准（2026-07-27，Jazzy）：三控制器 active；goal 执行成功（goal-hold
   confirmed）；执行中新 goal 正确抢占（旧 CANCELED、新 SUCCEEDED）；
   sine_shoulder 墙钟/标称比 1.08、终点误差 0、joint_states 200Hz。
2. **mock 回归**：`hardware_mode:=mock` 下标准 JTC goal 成功。
3. **真机分阶段流程**（务必按序，详见 `docs/usage.md` 第 7 节）：
   只核对 joint_states → 上电（`/aubo_dashboard/startup`）→ 低速小轨迹 →
   取消/抢占行为 → MoveIt 整机轨迹 + trace 分析。
   验收指标：执行期点吞吐率 ≈200 点/s；RIB 不饿死（>0）不溢出（<400）；
   墙钟/标称时长比 ≈1.0；终点误差 < 0.02 rad。

完整命令手册（控制器管理、action/服务示例、排障表）见 `docs/usage.md`。

## 7. 关键参数与契约

- **权威关节顺序**（goal 的 joint_names 会被控制器 remap 到此顺序）：
  `shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`
- **hardware 参数**（URDF `<param>`，见
  `src/aubo_description/urdf/aubo_e5.ros2_control.xacro`，默认值与蓝本一致）：
  `send_period_ms=4`、`rib_target=400`、`rib_slowdown_1/2=300/350`、
  `batch_min/max=2/8`、`ema_alpha=0.1`、`stop_retry_ms=20`、`auto_power_on=false`、
  `max_joint_velocity/acceleration` 等。
- **控制器参数**（`src/aubo_e5_bringup/config/controllers.yaml`）：
  `goal_tolerance_rad=0.02`、`goal_vel_tolerance=0.01`、`goal_hold_frames=5`、
  `goal_check_ms=50`、`blend_threshold_rad=0.01`、`blend_steps=30`；
  controller_manager `update_rate: 200`。
- **MoveIt 侧**：`trajectory_execution.*` 蓝本值 5.0/10.0/0.15；速度缩放由 MoveIt
  时间参数化完成，控制器不做执行期缩放。
- **GPIO 契约**：`trajectory_passthrough`（transfer_state 状态机 0–6 + 设定点）、
  `aubo_io`（IO 状态/命令 + RIB 状态）与 `speed_scaling`（speed_scaling_factor，
  恒定 1.0，本驱动不做执行期缩放）三组接口，见各插件 XML 描述与 xacro。
- **Python 节点参数默认值**：以各包 `config/*.yaml` 为权威源，代码内
  `declare_parameter` 默认值已逐一对齐（如 hand_eye `velocity_scaling=0.1`、
  scene_recon `max_range=3.0`/`tf_timeout_sec=0.5`）；改默认值时两边必须同步。

## 8. 代码风格与开发约定

- C++17；注释以中文为主、密度较高，注释常解释"为什么"（尤其是蓝本语义与 SDK 坑），
  修改行为时必须同步更新注释与 `docs/` 相关文档。
- 控制器参数用 **generate_parameter_library** 声明：改参数要改
  `src/aubo_e5_controllers/src/*_parameters.yaml`，不是手写参数解析。
- 硬件插件刻意使用 Jazzy 已 deprecated 的旧式
  `export_state_interfaces()/export_command_interfaces()`——这是 UR 验证过、能让硬件
  把 passthrough 状态机回写进 command 接口的写法，用
  `-Wno-deprecated-declarations` 屏蔽告警，**不要"升级"为新式接口**（见
  `aubo_e5_hardware/CMakeLists.txt` 注释）。
- 遵循蓝本（aubo_boot / UR passthrough）语义优先于自由发挥：轨迹一次性下发、
  goal_hold 判定、RIB 水位流控等核心逻辑不要改成流式。
- 全包代码风格执行 ROS 2 官方标准并已被 `colcon test` 强制：C++ 走
  ament_uncrustify + ament_cpplint（100 列、`*`/`&` 居中对齐、BSD copyright 头）；
  Python 走 ament_flake8（99 列、单引号、import 分组）；CMake 走 ament_lint_cmake。
  改代码后必须保持 lint 全绿（工具均在 /opt/ros/jazzy/bin）。
- venv 依赖节点的启动约定：需要 torch/open3d 的节点（aubo_scene_recon、
  peach_pose_ros2）一律经包内 `scripts/` 下 bash 包装脚本启动（装到 `lib/<pkg>`，
  `Node()`/`ros2 run` 按名引用）。包装脚本内部约定：`AUBO_PYTHON` 显式指定解释器
  优先级最高；操作者已 `source` 某个 venv（`VIRTUAL_ENV` 非空）则直接沿用；
  否则自动 `source` 工作区 `aubo_py3.12/bin/activate` 后用 `python3 -m` 启动。
  launch 文件一律用标准 `Node()`，不得出现解释器路径或 ExecuteProcess 拼
  `python -m`；也不要改回 console_scripts——其 shebang 是系统 python3，
  没有 torch/open3d，入口会实际不可用。
- `aubo_e5_hardware` 公共头 `aubo_e5_hardware.hpp` include 了 vendor SDK 头
  （`AuboRobotMetaType.h`），而 vendor include 路径是 PRIVATE——该头目前不支持
  被下游包 include（现无下游消费者；如未来需要，先把 SDK 类型移入 .cpp
  或改为导出 vendor 头）。
- 不要向构建树（`build/`、`install/`、`log/`）提交改动。

## 9. 部署注意事项（容易踩的坑）

- **libprotobuf.so.9**：旧 SDK 强依赖 protobuf 2.6.1，vendor 在
  `aubo_e5_hardware/vendor/lib64/`。链接强制 `DT_RPATH`（`--disable-new-dtags`），
  因为传递依赖不走 RUNPATH；手动跑二进制需设 `LD_LIBRARY_PATH` 指向
  `install/aubo_e5_hardware/lib/aubo_e5_hardware/vendor`，正常经 launch 启动则无此问题。
- **SDK 按进程 CWD 读配置**：`./config/auborobot.conf` 与 `tracelog.properties`；
  launch 已把 `ros2_control_node` 与 `aubo_dashboard_node` 的 cwd 设为各自 share 目录。
- **TCP2CAN 独占**：插件 activate 后 SDK 运动 API 被独占，示教器运动暂停；
  deactivate 后恢复。
- **RT 实时性**：已取消 RT 要求——real 模式在普通内核直接运行，无
  SCHED_FIFO/lowlatency 内核预检（`docs/realtime_setup.md` 仅供历史参考）。
  注意该机 NVIDIA 595 驱动无 PREEMPT_RT 预编译模块，不能换 RT 内核；
  网卡 r8126 DKMS 驱动曾是 SDK 通道秒级停滞根因（`docs/nic_driver_incident.md`）。
- **venv 的 numpy 必须 <2**：`aubo_py3.12` 带 `--system-site-packages`，ROS jazzy
  二进制（cv_bridge）与 apt 的 python3-opencv / python3-scipy 都是按 numpy 1.x
  编译的。venv 里若装 numpy 2.x，`import cv2` 报
  "numpy.core.multiarray failed to import"、`import cv_bridge` 直接段错误。
  正确组合：venv 内 `numpy==1.26.4`（+ `matplotlib==3.11.1`，见根目录
  `requirements.txt`），cv2/scipy 走系统 dist-packages（4.6.0 / 1.11.4），
  不要在 venv 里 pip 装 opencv-python/scipy（装了会 shadow 系统版）。
  节点级 venv 入口约定（包装脚本 + `AUBO_PYTHON`）见第 8 节。
- **tf2 静态 TF 不接受同发布者覆盖**：`extrinsics_publisher` 的 `~/reload`
  重读后确实重新 sendTransform，但同一 publisher 进程更新 wrist3_Link→
  camera_link 不会传播给 tf2 buffer（后加入的订阅者仍拿到旧值）。激活新外参后
  必须**重启 extrinsics_publisher 进程**（或整个标定 launch）才能让所有
  消费者看到新 TF。
- **每次开机必做（不持久）**：网卡 offload 关闭 + governor 设置，否则 SDK
  推送链路会出现 >200ms 停滞触发 read() FAULT：
  `sudo ethtool -K enp130s0 gro off gso off tso off` +
  `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`
  （原 `configure_realtime.sh` 已随旧 bringup 删除，手动执行上述两条命令即可）。

## 10. 安全注意事项

- **真机运行任何运动测试，速度/加速度缩放必须先压到 0.1**，防止突发碰撞；
  确认行为符合预期后才可逐步放宽。各入口：RViz 速度滑条、MoveIt 请求的
  `max_velocity/acceleration_scaling_factor`、测试脚本同理；手眼标定 server
  的 `velocity_scaling`/`acceleration_scaling` 在
  `aubo_hand_eye_calibration/config/calibration.yaml`（已默认 0.1）。
- **真机操作前**：现场确认急停、限位、碰撞等级、低速模式；首次只核对状态不运动；
  严格按 `docs/usage.md` 第 7 节的分阶段流程推进。
- 默认 `auto_power_on=false`，启动不会自动上电；上电需显式调用
  `/aubo_dashboard/startup` 或示教器手动操作。
- 急停/防护停由本体安全回路主导，软件侧仅停发清队——不要在代码里试图"恢复"
  安全停止状态。
- SDK 用户名/密码走 xacro 参数（`sdk_username`/`sdk_password`），不要把真实凭据
  提交进仓库。
- `SDK资料/` 与 vendor 二进制为厂商专有资料，不要外传或上传到公共仓库。

## 11. 参考文档

- `README.md` — 项目入口说明（与本文互补）
- `docs/usage.md` — 完整命令手册与排障表
- `docs/passthrough_migration.md` — 架构迁移说明与已验证清单
- `docs/archive/` — 旧架构文档（仅供历史参考）
- 写法参考：UR `ur_controllers::PassthroughTrajectoryController`
  （GitHub: UniversalRobots/Universal_Robots_ROS2_Driver）；
  行为蓝本 aubo_boot（本机 `/home/mu/Music/e`，不随交付分发）
