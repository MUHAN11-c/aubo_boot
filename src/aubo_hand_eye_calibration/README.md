# aubo_hand_eye_calibration

眼在手上外参标定 + 彩色内参标定落盘：采集标定板、解外参、落盘，并由 `extrinsics_publisher` 发静态 TF；内参用 vendored `camera_calibration` 交互标定后由 `apply_intrinsics` 原子落盘。

感知与重建用**精确时间戳**查 `base_link ← camera`；外参 TF 是这条链的一部分。标定坏了，世界系身份和 ICP 初值都会偏。

## 结果在哪（事实源，2026-09-17 起）

| 路径 | 内容 |
|------|------|
| `hand_eye/active.yaml` | 当前激活外参（**入库随仓**；`candidates/` 会话产物才 gitignore） |
| `hand_eye/candidates/` | 历次求解候选（gitignore，仅本机） |
| `../percipio_camera/config/color_camera_info.yaml` | 彩色内参唯一事实源（percipio 与 peach_stereo 共用；由 `apply_intrinsics` 落盘） |
| `config/calibration.yaml` | 棋盘格、质量门、坐标系名 |
| `config/poses.yaml` | 采集用的标定位姿 |

可用 `AUBO_HAND_EYE_DIR` 覆盖外参目录。无源码树时落到 `~/.ros/aubo_e5/hand_eye`。不要写进 `install/share`（colcon 会盖掉）。

拍照示教位姿（`global_photo_pose` 等）在 [`aubo_e5_moveit_config`](../aubo_e5_moveit_config/README.md) 的 SRDF，不在本包。

## 节点与工具

- `extrinsics_publisher`：读 `hand_eye/active.yaml`，发静态 TF `wrist3_Link→camera_link`；可 `reload`。外参来源上 `/diagnostics`（`extrinsics_source`：active=OK、无文件回退名义=WARN、active 损坏/帧名不匹配=ERROR）
- `calibration_server`：动作/服务跑采集与求解（17 位姿自动采集 + OpenCV 五方法竞赛 + Huber 精化 + 质量门）
- `web_gateway`：`http://127.0.0.1:8088` 调试界面（仅回环；2026-09-29 起界面可选 `pose_source`（poses/auto）与 `solve_target`（hand_eye/joint）档位，非法值 400；显示当前激活外参卡——`import:` 来源标「外部导入 · 非本机标定产物」）
- `apply_intrinsics`：校验 `cameracalibrator` SAVE 产物并原子写入内参事实源
- `intrinsics_calibration.launch.py`：拉起 vendored `cameracalibrator`（交互式 GUI；板参数默认值运行时取自 `config/calibration.yaml` 单源，launch args 可覆盖）

bringup 默认可开外参发布。全标定流程单独 launch，不要在日常采摘里自动跑。无 `active.yaml` 时发名义 TF（平移 2 cm、单位四元数），光学系会偏。`_archive/runs/hand_eye/` 是历史归档，不被读取。

## 内参标定流程（棋盘格）

1. 起相机前端（percipio 或 stereo）；
2. `ros2 launch aubo_hand_eye_calibration intrinsics_calibration.launch.py`（需 X 显示；板参数默认 11x8 内角点 @20mm，与手眼同一块板，可用 launch args 覆盖）；
3. GUI 里采满进度条 → CALIBRATE → SAVE（写出 `/tmp/calibrationdata.tar.gz`；COMMIT 不可用——前端无 `set_camera_info`）；
4. `ros2 run aubo_hand_eye_calibration apply_intrinsics /tmp/calibrationdata.tar.gz` → 校验（分辨率/焦距/主点/畸变系数个数）后原子写入事实源；
5. `colcon build --packages-select percipio_camera`（launch 经 `FindPackageShare` 读 **install 副本**，必须重建才生效）；
6. 重启相机前端；`ros2 topic echo /camera/color/camera_info --once` 核对 K；手动更新 `src/peach_harvester/config/scene_perception.yaml` 的 `calibration_version` 标签。

`apply_intrinsics` 的输入也可以是本包 `hand_eye/active.yaml`（读 auto 档联合标定写入的 `intrinsics` 节）。

## auto 档：自动视点 + 内外参联合标定（2026-09-17 新增）

**相机无关**：所有几何量运行时取自活的 `/camera/color/camera_info`（K/D/宽高）；视点距离按「板宽画面占比」反推（适配任意焦距），不写死 FOV/型号常量。

前置：棋盘格**固定摆放**在臂可达处且当前画面可见；`hand_eye/active.yaml` 存在（初始外参用于定位板，缺失明确报错，不做名义回退）。

入口二选一（2026-09-29 起 Web 与 CLI 等价）：Web 界面「标定流程」卡选 **位姿来源=auto · 自动视点**、**求解目标=joint · 内外参联合** 后按 ①②③ 走；或 CLI：

```bash
ros2 action send_goal /hand_eye_calibration_server/run \
  aubo_msgs/action/RunHandEyeCalibration \
  "{pose_source: auto, solve_target: joint, return_to_start: true}"
```

- 流程：当前帧定位板 → FOV 掩码过滤环绕视点（视点位静态保证棋盘格在带余量画面内；**transit 不承诺**）→ 贪心选出旋转跨度 ≥30° 视点队列 → plan-only 预检跳过不可达/碰撞视点并由冗余候选补位 → settle 后同步采集（每视点帧内 RMS 最小前 2 帧**原始角点**）→ 联合求解 → 产物落 `hand_eye/candidates/`。
- 联合求解三段：`cv2.calibrateCamera` 初始化 K/D（与 launch `-k 2` 同族 plumb_bob）→ 现有五方法 hand-eye 给 X/B 初值 → scipy 20 参数（fx,fy,cx,cy,k1,k2,p1,p2,se3(X),se3(B)）Huber 联合抛光，逐视位姿被链式约束 `T_ct = inv(X) inv(A_i) B` 消元。
- 产物：candidate yaml 增量 `intrinsics` 节（等价 OST 文档）+ `viewpoints` 明细 + joint 指标；action result 增 `camera_matrix`/`distortion_coefficients`/`joint_reprojection_rms_px`。
- 误差门：`joint_max_reprojection_rms_px`（默认 0.8px）+ 沿用平移/旋转一致性与跨度门；逐视 RMS 明细在 `per_view`。
- 内参生效三步：`~/activate` 激活 → `apply_intrinsics src/aubo_hand_eye_calibration/hand_eye/active.yaml` → `colcon build --packages-select percipio_camera` 后重启前端。

## vendored camera_calibration（来源与边界）

- 上游：<https://github.com/ros-perception/image_pipeline> `jazzy` 分支 @ `6c3df3099bc7b1ec92215719215c8eefd0d3aa69`（2026-07-10，版本 5.0.13）；**原样入库、零本地修改**，日后对上游 re-sync 直接覆盖 `src/camera_calibration/` 即可；
- 与 apt `ros-jazzy-camera-calibration` 同名同版：**勿与 apt 版并装**（工作区 overlay 优先，易混淆来源）；
- 依赖 `semver`：本机 venv 钉 `semver==3.0.2`（`requirements.txt`）；CI 由该包 `package.xml` 的 `python3-semver` rosdep 键走 apt；
- 订阅 QoS 自动匹配发布端 profile（`camera_calibrator.get_topic_qos`），percipio / stereo 两前端均兼容。

## 社区对照（选型依据）

- 内参：image_pipeline `camera_calibration` 是 ROS 2 官方棋盘格标定工具（Nav2 Jazzy 教程同款），vendored 集成而非自研 `cv2.calibrateCamera` 节点——成熟库优先；
- 外参：求解核即 OpenCV `calibrateHandEye` 五方法（tsai/park/horaud/andreff/daniilidis）+ MAD 离群 + scipy Huber 联合精化，与社区同源且强于 easy_handeye2 的基础用法；easy_handeye2 与 moveit_calibration 均无 Jazzy apt 发行版（后者官方无 ROS 2 版），不引入。

## 测试

纯核单测（零 rclpy）：`test/test_{transforms,stats,solver,detector,storage,intrinsics,viewpoints,joint_calib,web_goal}.py`——合成 AX=XB 位姿恢复与离群剔除、合成棋盘格渲染→检测→PnP 恢复、候选/激活落盘链、内参产物解析校验、FOV 掩码/视点生成/多样性选择、联合内外参求解合成恢复（观测场景由 viewpoints 模块自生成闭环）、Web 档位参数校验与空串回退。

```bash
colcon build --packages-select aubo_hand_eye_calibration
colcon test --packages-select aubo_hand_eye_calibration && colcon test-result --verbose
```
