# aubo_hand_eye_calibration

眼在手上外参标定 + 彩色内参标定落盘：采集标定板、解外参、落盘，并由 `extrinsics_publisher` 发静态 TF；内参用 apt 提供的 `camera_calibration` 交互标定后由 `apply_intrinsics` 原子落盘。原 vendored 副本已删除；当前依赖未锁定其版本，见下方来源说明。

感知与重建用**精确时间戳**查 `base_link ← camera`；外参 TF 是这条链的一部分。标定坏了，世界系身份和 ICP 初值都会偏。

## 结果在哪（事实源，2026-09-17 起）

| 路径 | 内容 |
|------|------|
| `hand_eye/active.yaml` | 当前激活外参（**入库随仓**；`candidates/` 会话产物才 gitignore） |
| `hand_eye/candidates/` | 历次求解候选（gitignore，仅本机） |
| `../percipio_camera/config/color_camera_info.yaml` | 项目彩色 CameraInfo 的内参覆盖事实源（percipio 与 peach_stereo 共用；由 `apply_intrinsics` 落盘） |
| `config/calibration.yaml` | 棋盘格、质量门、坐标系名 |
| `config/poses.yaml` | 采集用的标定位姿 |

可用 `AUBO_HAND_EYE_DIR` 覆盖外参目录。无源码树时落到 `~/.ros/aubo_e5/hand_eye`。不要写进 `install/share`（colcon 会盖掉）。

拍照示教位姿（`global_photo_pose` 等）在 [`aubo_e5_moveit_config`](../aubo_e5_moveit_config/README.md) 的 SRDF，不在本包。

## 节点与工具

- `extrinsics_publisher`：读 `hand_eye/active.yaml`，发静态 TF `wrist3_Link→camera_link`；可 `reload`。启动来源上 `/diagnostics`（active=OK、缺文件 nominal=WARN、已有文件不可用而 nominal 回退=ERROR）；运行期 reload 失败返回 false 并保留此前 TF。
- `calibration_server`：动作/服务跑采集与求解（17 位姿自动采集 + OpenCV 五方法竞赛 + Huber 精化 + 质量门）
- `web_gateway`：`http://127.0.0.1:8088` 调试界面（仅回环；2026-09-29 起界面可选 `pose_source`（poses/auto）与 `solve_target`（hand_eye/joint）档位，非法值 400；显示当前激活外参卡——`import:` 来源标「外部导入 · 非本机标定产物」）
- `apply_intrinsics`：校验 `cameracalibrator` SAVE 产物并原子写入内参事实源
- `intrinsics_calibration.launch.py`：拉起 `cameracalibrator`（交互式 GUI；板参数默认值运行时取自 `config/calibration.yaml` 单源，launch args 可覆盖）

driver bringup 默认开启外参发布；完整标定流程由 `hand_eye_enabled` 单独控制，默认关闭。独立 `hand_eye_calibration.launch.py` 默认启动服务、外参发布器与 Web，但不会自动发标定 goal，也不启动相机或驱动。无 `active.yaml` 时发名义 TF（沿腕 Z 平移 2 cm、单位四元数），它不表示已经标定。`_archive/runs/hand_eye/` 不参与默认路径查找。

公有端点、QoS、时钟与激活边界见 [doc/io.md](doc/io.md)，模块和流程见 [doc/architecture.md](doc/architecture.md)。固定相机部署可关闭本发布器，使用 bringup 的固定安装 TF；不能把它与腕部外参同时作为同一个 `camera_link` 的父变换。

## 内参标定流程（棋盘格）

1. 起相机前端（percipio 或 stereo）；
2. `ros2 launch aubo_hand_eye_calibration intrinsics_calibration.launch.py`（需 X 显示；板参数默认 11x8 内角点 @20mm，与手眼同一块板，可用 launch args 覆盖）；
3. GUI 里采满进度条 → CALIBRATE → SAVE（写出 `/tmp/calibrationdata.tar.gz`；COMMIT 不可用——前端无 `set_camera_info`）；
4. `ros2 run aubo_hand_eye_calibration apply_intrinsics /tmp/calibrationdata.tar.gz` → 校验（分辨率/焦距/主点/畸变系数个数）后原子写入事实源；
5. `colcon build --packages-select percipio_camera`（launch 经 `FindPackageShare` 读 **install 副本**，必须重建才生效）；
6. 重启相机前端并保持彩色图像订阅；核对 `/camera/color/camera_info` 的 K。Percipio 的 CameraInfo 与图像在同一发布回调，只有 CameraInfo 订阅者时不会单独出帧。手动更新 `src/peach/peach_scene_perception/config/scene_perception.yaml` 的 `calibration_version` 标签。

`apply_intrinsics` 的输入也可以是本包 `hand_eye/active.yaml`（读 auto 档联合标定写入的 `intrinsics` 节）。

该工具只写项目 YAML，不改设备 Flash。Percipio 的 YAML 覆盖只替换彩色 CameraInfo 的 D/K/R/P，设备内部深度配准仍使用设备标定；更换图像分辨率时须重新核对内参，驱动不会自动按 YAML 分辨率缩放 K。

## auto 档：自动视点 + 内外参联合标定（2026-09-17 新增）

**相机无关**：所有几何量运行时取自活的 `/camera/color/camera_info`（K/D/宽高）；视点距离按「板宽画面占比」反推（适配任意焦距），不写死 FOV/型号常量。

前置：棋盘格**固定摆放**在臂可达处且当前画面可见；`hand_eye/active.yaml` 存在（初始外参用于定位板，缺失明确报错，不做名义回退）。

入口二选一（2026-09-29 起 Web 与 CLI 等价）：Web 界面「标定流程」卡选 **位姿来源=auto · 自动视点**、**求解目标=joint · 内外参联合** 后按 ①②③ 走；或 CLI：

```bash
ros2 action send_goal /hand_eye_calibration_server/run \
  aubo_msgs/action/RunHandEyeCalibration \
  "{pose_source: auto, solve_target: joint, return_to_start: true}"
```

- 流程：当前图像定位板 → FOV 掩码过滤环绕视点（静态视点满足带余量的投影检查，transit 不承诺）→ 贪心选出旋转跨度 ≥30° 视点队列 → plan-only 预检跳过不可达/碰撞视点并由冗余候选补位 → settle 后采集（每视点帧内 RMS 最小前 2 帧**原始角点**）→ 联合求解 → 产物落 `hand_eye/candidates/`。当前采集缓冲保存处理时刻与观测，腕部配对读取 latest TF；不是按图像曝光 stamp 同步查询。
- 联合求解三段：`cv2.calibrateCamera` 初始化 K/D（与 launch `-k 2` 同族 plumb_bob）→ 现有五方法 hand-eye 给 X/B 初值 → scipy 20 参数（fx,fy,cx,cy,k1,k2,p1,p2,se3(X),se3(B)）Huber 联合抛光，逐视位姿被链式约束 `T_ct = inv(X) inv(A_i) B` 消元。
- 产物：candidate yaml 增量 `intrinsics` 节（等价 OST 文档）+ `viewpoints` 明细 + joint 指标；action result 增 `camera_matrix`/`distortion_coefficients`/`joint_reprojection_rms_px`。
- 误差门：`joint_max_reprojection_rms_px`（默认 0.8px）+ 沿用平移/旋转一致性与跨度门；逐视 RMS 明细在 `per_view`。
- 内参生效三步：`~/activate` 激活 → `apply_intrinsics src/aubo_hand_eye_calibration/hand_eye/active.yaml` → `colcon build --packages-select percipio_camera` 后重启前端。

## camera_calibration（来源与边界；vendored 副本已删）

- 上游：<https://github.com/ros-perception/image_pipeline> `jazzy` 分支 @ `6c3df3099bc7b1ec92215719215c8eefd0d3aa69`（2026-07-10，版本 5.0.13）；工作区 vendored 副本 `src/camera_calibration/` 已于 2026-10-10 随清理删除（历史可 `git show <提交>:src/camera_calibration/...` 取回，re-vendor 直接覆盖该路径即可）；
- 现行 `intrinsics_calibration.launch.py` 与 `package.xml` 的 `exec_depend camera_calibration` 由 **apt `ros-jazzy-camera-calibration`** 解析；依赖没有版本约束，历史 vendored 的 5.0.13 不能代表目标环境当前版本，标定前核对已安装包；
- 依赖 `semver`：本机 venv 钉 `semver==3.0.2`（`requirements.txt`）；apt 版依赖走 rosdep；
- 相机前端须发布原始彩色 Image；launch 将 `image` remap 到该话题并传 `--no-service-check`。标定器的实际 QoS 行为由目标环境安装的 camera_calibration 版本决定；不要把本包 hand-eye 服务固定的 sensor QoS 误认为内参 GUI 的配置。

## 社区对照（选型依据）

- 内参：image_pipeline `camera_calibration` 是 ROS 2 官方棋盘格标定工具（Nav2 Jazzy 教程同款），官方包集成而非自研 `cv2.calibrateCamera` 节点——成熟库优先；
- 外参：求解核即 OpenCV `calibrateHandEye` 五方法（tsai/park/horaud/andreff/daniilidis）+ MAD 离群 + scipy Huber 联合精化，与社区同源且强于 easy_handeye2 的基础用法；easy_handeye2 与 moveit_calibration 均无 Jazzy apt 发行版（后者官方无 ROS 2 版），不引入。

## 测试

```bash
colcon build --packages-select aubo_hand_eye_calibration
colcon test --packages-select aubo_hand_eye_calibration && colcon test-result --verbose
```

`colcon test` 只跑官方 flake8 / pep257。标定是否收敛，看节点日志里的重投影误差和手眼结果。
