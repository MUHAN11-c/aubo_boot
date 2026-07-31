# aubo_hand_eye_calibration — AUBO E5 自动手眼标定

对腕部 Percipio 相机做 eye-in-hand 标定：棋盘格标定板（默认 11×8 内角点、
20 mm 格宽，可配置），机器人按 17 个预定义位姿自动运动采样，求解
wrist3→camera 外参，版本化存档后经人工确认激活发布。

流程编排借鉴 [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)
的采样/持久化/发布分离；运动走 Jazzy
[MoveGroup action](https://docs.ros.org/en/jazzy/p/moveit_msgs/action/MoveGroup.html)
（可取消，带位置/姿态约束）；棋盘检测用 OpenCV `findChessboardCornersSB`
（逐帧重投影门），单板位姿估计 `SOLVEPNP_SQPNP` + `solvePnPRefineLM` 精化。

## 架构与节点分工

三个节点均为 console_scripts 入口，跑**系统 python3**，依赖全部走 apt
（`python3-opencv` / `python3-scipy` / `python3-numpy` / `python3-yaml`，
见 `package.xml`），不进 venv：

| 节点（进程名） | 职责 |
|---|---|
| `calibration_server`（hand_eye_calibration_server） | 标定状态机：`~/run` action 驱动 预检→规划→逐位姿运动/采样→求解（可取消）；`~/activate` 服务激活候选；发布 `~/status`（TRANSIENT_LOCAL 的 JSON 状态）与 `~/preview/compressed`（检测叠加图） |
| `extrinsics_publisher`（hand_eye_extrinsics_publisher） | `wrist3_Link → camera_link` 静态 TF 的**唯一**发布者；读激活结果，无激活/文件损坏时回退标称外参（`[0, 0, 0.02]` + 单位四元数）并告警；`~/reload` 服务重读文件 |
| `web_gateway`（hand_eye_web_gateway） | 本机 HTTP 网关（默认 `127.0.0.1:8088`，代码强制仅 loopback）：桥接 action/激活服务、候选查询、MJPEG 预览，并按 10 Hz 组包机器人状态（io 控制器 / `/joint_states` / TF / `/diagnostics`） |

三节点合计 38 个参数全部带中文 `ParameterDescriptor`，运行时可
`ros2 param describe <节点名> <参数名>` 查看说明。

## 快速开始

前置：先按 `AGENTS.md` 第 6 节确认无残留进程
（`pgrep -af 'ros2 launch|ros2 run'`）。标定 launch **不起相机、不起机械臂**，
三块各自独立：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/aubo_e5_jazzy_ws/install/setup.bash

# 终端 1：机械臂 + MoveIt（关闭 bringup 自带相机，交给下面的相机 launch
# 单独管；hardware_mode:=sim 仅用于验证启动接线，实际标定必须真机 + 真相机）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=<控制器IP> camera_enabled:=false

# 终端 2：相机驱动（标定只用彩色 + CameraInfo，无需深度）
ros2 launch percipio_camera percipio_camera.launch.py

# 终端 3：标定节点（extrinsics TF 已由终端 1 的 bringup 发布，
# 这里 extrinsics_enabled:=false 避免同一静态 TF 双发）
ros2 launch aubo_hand_eye_calibration hand_eye_calibration.launch.py \
  extrinsics_enabled:=false

# 浏览器打开 http://127.0.0.1:8088
```

也可单 launch 一体启动（bringup 内含标定 include，且已把标定 launch 的
`extrinsics_enabled` 固定为 false，避免外参 TF 双发）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  hand_eye_enabled:=true hand_eye_web_enabled:=true
```

标定 launch 参数：`extrinsics_enabled`（true）、`web_enabled`（true）、
`calibration_config`（默认包内 `config/calibration.yaml`）；
`poses_file` 由 launch 固定注入包内 `config/poses.yaml`。

## 标定流程（状态机）

```text
idle → preflighting → planning → plan_ready (仅预检模式, 不动机器人)
     → moving → settling → capturing → (逐位姿循环)
     → returning → solving → complete / quality_failed / failed
任意运动阶段可取消 → cancelling → cancelled
激活(人工确认, 独立服务) → activated
```

1. **预检并规划**：检查 CameraInfo、棋盘可见、TF 链（base→wrist3、
   camera_link→camera_color_optical）、MoveGroup server，然后对 17 个预定义
   位姿逐个做 plan_only 验证（`return_to_start` 时把当前腕部位姿追加为第 18
   个验证位姿），机器人不动；任一不可规划即失败。
2. **执行自动标定**：一次确认后机器人自动走完所有位姿；每个位姿停稳后
   （关节速度 < `joint_velocity_threshold` 持续 `settle_duration_s`）采集
   `frames_per_pose` 帧有效检测，图像观测与腕部 TF 同帧配对，按重投影 RMS
   做 MAD 剔除后取均值。随时可取消，取消显示为 cancelled 而非 failed。
3. **求解**：质量门（样本数/平移 RMS/旋转 RMS/重投影 RMS/旋转跨度）全部
   通过才允许激活；未过门的候选只存档，UI 不会点亮启用按钮。
4. **人工确认激活**：UI 展示完整外参数值及与当前激活版本的差异，确认后
   写入 `active.yaml` 并调 `~/reload` 热重载 `extrinsics_publisher`。

## 求解与质量门

- `solver_method:=auto`（默认）时 5 个 OpenCV 方法（tsai/park/horaud/andreff/
  daniilidis）全部求解，按**门限归一化残差**打分（平移 RMS 与旋转 RMS 各除以
  对应门限后相加，消除量纲任意性）取最优；指定单一方法时只跑该方法，
  失败直接报错，不静默换方法。
- 离群剔除：按目标位姿一致性残差做 MAD（阈值 3.5）剔样本 → 对内点做
  **Huber 联合精化**（手眼 + 固定标定板位姿，se3 残差，scipy
  `least_squares`，`f_scale=0.01`；不收敛则回退初解）→ 精化后再做一次
  MAD 复核。方法打分/逐样本残差/精化统计全部落盘并推给 UI。
- 质量门：`min_samples`、`max_reprojection_rms_px`、
  `max_translation_rms_m`、`max_rotation_rms_deg`、`min_rotation_span_deg`，
  全部通过才 `complete`，否则 `quality_failed`。

## Web UI

绑定 `127.0.0.1:8088`（仅本机）。页面区块：

- MJPEG 实时预览（棋盘检测叠加 + 可见性徽章 + 帧率）
- 标定流程步骤条 + 进度环（可选求解方法、预检/执行/取消按钮）
- 位姿采样矩阵（逐位姿状态/RMS/失败原因，可展开逐帧观测明细）
- 重投影 RMS 时序图、求解结果（方法打分、逐样本残差、精化统计、
  质量门对照、与激活版本的平移/旋转差异）
- 机器人状态（io 控制器的 RobotStatus + 安全 IO）、末端位姿、关节状态
  （位置/速度/电流/温度/跟随误差）、`/diagnostics`；无数据源时对应区块留空
- 历史候选列表（查看详情、激活通过的候选、标注当前激活版本）

REST 接口：`GET /api/status`、`GET /api/robot`、`GET /api/events`（SSE）、
`GET /api/candidates`、`GET /api/candidates/<id>`、`GET /api/active`、
`GET /api/fps`、`GET /preview.jpg`、`GET /preview.mjpeg`；
POST `/api/preflight`（别名 `/api/plan`）、`/api/run`、`/api/cancel`、
`/api/activate`。

## 坐标约定

```text
T_base_target =
  T_base_wrist3 * T_wrist3_camera_optical * T_camera_optical_target
```

激活的变换只由一个节点发布：

```text
wrist3_Link -> camera_link -> camera_color_frame
                            -> camera_color_optical_frame
```

第一条边来自 `extrinsics_publisher`；其余相机内部边来自驱动。首次通过
标定前发布的是标称安装外参并告警；`active.yaml` 损坏或坐标系不匹配不会
让节点崩溃——回退标称外参。

## 结果存储与激活

- 存储目录优先级：`AUBO_HAND_EYE_DIR` 环境变量 > `<工作区根>/hand_eye/`
  （相对包文件定位，源码树与 install 布局均成立）> 兜底
  `~/.ros/aubo_e5/hand_eye`。布局为 `active.yaml` + `candidates/<id>.yaml`。
- 候选文件（`schema_version: 1`）含三组变换、质量指标、方法打分、逐样本
  残差、精化统计与逐帧观测；写入为 tmp + rename 原子替换。激活即把候选
  复制为 `active.yaml`（仅允许通过质量门的候选）。
- **tf2 静态 TF 不接受同发布者覆盖**：激活后服务端会自动调
  `~/reload` 重发 TF，但已订阅的 tf2 buffer 仍保留旧值。激活新外参后必须
  **重启 extrinsics_publisher 进程**（或整个 launch），所有消费者才能看到
  新 TF（见 `AGENTS.md` 第 9 节）。
- `extrinsics_publisher` 参数：`parent_frame`/`child_frame`（默认
  wrist3_Link/camera_link）、`active_file`（空串按默认目录定位）、
  `nominal_xyz_m`/`nominal_quaternion_xyzw`（标称外参）。

## 配置

平移一律以米计。权威参数源为 `config/calibration.yaml`（launch 全量加载；
代码内 `declare_parameter` 默认值已与文件逐一对齐）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `base_frame` / `wrist_frame` | base_link / wrist3_Link | 机器人基座 / 腕部坐标系 |
| `camera_root_frame` / `camera_optical_frame` | camera_link / camera_color_optical_frame | 外参发布目标 / 求解目标 |
| `move_group` | manipulator_e5 | MoveIt 规划组 |
| `image_topic` / `camera_info_topic` | /camera/color/image_raw 等 | 彩色图与内参话题 |
| `board_columns` / `board_rows` | 11 / 8 | 棋盘内角点 |
| `board_square_size_m` | 0.020 | 格宽，**务必实测** |
| `frames_per_pose` | 5 | 每位姿采样帧数 |
| `min_samples` | 12 | 最少接受样本数 |
| `joint_velocity_threshold` | 0.01 | 静止判定关节速度阈值 (rad/s) |
| `settle_duration_s` / `stable_timeout_s` | 0.5 / 10.0 | 停稳判据 |
| `sample_timeout_s` | 4.0 | 单位姿采集超时 |
| `velocity_scaling` / `acceleration_scaling` | 0.1 / 0.1 | MoveIt 速度/加速度缩放 |
| `planning_attempts` / `planning_time_s` | 5 / 5.0 | MoveIt 规划 |
| `position_tolerance_m` / `orientation_tolerance_rad` | 0.001 / 0.01 | 位姿到达容差（目标约束） |
| `max_reprojection_rms_px` | 1.0 | 重投影质量门 |
| `max_translation_rms_m` / `max_rotation_rms_deg` | 0.003 / 0.5 | 一致性质量门 |
| `min_rotation_span_deg` | 20.0 | 腕部旋转跨度质量门 |
| `solver_method` | auto | 求解方法：auto\|tsai\|park\|horaud\|andreff\|daniilidis（goal 未指定时生效） |

改标定轨迹请编辑 `config/poses.yaml`（17 个腕部位姿，基座系 position +
quaternion）。Action/服务接口见 `aubo_msgs`：`RunHandEyeCalibration.action`
（goal：`plan_only`/`return_to_start`/`method`；feedback 含 stage、pose_index、
accepted_samples、progress）、`ActivateHandEyeCalibration.srv`。

## 测试

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select aubo_hand_eye_calibration && colcon test-result --verbose
# 或直接跑（venv 解释器；15 例业务 unittest + flake8/pep257 lint）
cd src/aubo_hand_eye_calibration && ../../aubo_py3.12/bin/python -m pytest test/ -q
```

真机运行前：核对机器人工作空间与碰撞模型，急停放在手边，低速起步
（`velocity_scaling` 已默认 0.1），先用预检（plan_only）确认全部位姿可
规划，再授权运动。
