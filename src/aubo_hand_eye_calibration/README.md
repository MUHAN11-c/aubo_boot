# AUBO E5 automatic eye-in-hand calibration

This package calibrates a wrist-mounted Percipio camera with a checkerboard
target (default 11×8 inner corners, 20 mm squares, configurable).

The workflow follows the ROS 2 sampling/persistence/publishing separation used
by [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2), while
using a cancellable ROS action and the Jazzy
[MoveGroup action](https://docs.ros.org/en/jazzy/p/moveit_msgs/action/MoveGroup.html)
for automatic movement. Detection uses OpenCV `findChessboardCornersSB` with a
per-frame reprojection gate, and pose estimation uses `SOLVEPNP_SQPNP` with LM
refinement. The solver compares all five OpenCV hand-eye methods (scored by
threshold-normalized residuals), rejects outliers with MAD statistics — again
after refinement — and jointly refines the hand-eye and fixed target poses with
a Huber loss.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/aubo_e5_jazzy_ws/install/setup.bash
ros2 launch aubo_hand_eye_calibration hand_eye_calibration.launch.py
# Open http://127.0.0.1:8088
```

## 标定流程（状态机）

```text
idle → preflighting → planning → plan_ready (仅预检模式, 不动机器人)
     → moving → settling → capturing → (逐位姿循环)
     → returning → solving → complete / quality_failed / failed
任意运动阶段可取消 → cancelling → cancelled
激活(人工确认, 独立服务) → activated
```

1. **预检并规划**：检查 CameraInfo、棋盘可见、TF 链、MoveIt，然后对全部
   17 个位姿（含可选返回位姿）逐个做 plan_only 验证，机器人不动。
2. **执行自动标定**：一次确认后机器人自动走完所有位姿；每个位姿停稳后
   采集 5 帧有效检测（图像观测与腕部 TF 同帧记录，按重投影 RMS 剔除离群
   帧后取均值）。随时可取消，取消显示为 cancelled 而非 failed。
3. **求解**：质量门（样本数/平移 RMS/旋转 RMS/重投影 RMS/旋转跨度）全部
   通过才允许激活；未过门的候选只存档，UI 不会点亮启用按钮。
4. **人工确认激活**：UI 展示完整外参数值及与当前激活版本的差异，确认后
   写入 `active.yaml` 并热重载 `extrinsics_publisher`。

## Web UI

绑定 `127.0.0.1:8088`（仅本机）。功能：

- MJPEG 实时预览（棋盘检测叠加 + 可见性徽章）
- 流程步骤条 + 总进度条 + 逐位姿采样清单（含每位姿 RMS/失败原因）
- 质量指标与门限对照表、外参数值、与激活版本的平移/旋转差异
- 历史候选列表（查看详情、激活通过的候选、标注当前激活版本）

REST 接口：`GET /api/status`、`GET /api/events` (SSE)、
`GET /api/candidates`、`GET /api/candidates/<id>`、`GET /api/active`、
`GET /preview.mjpeg`；POST `/api/plan`、`/api/run`、`/api/cancel`、
`/api/activate`。

## 坐标约定

```text
T_base_target =
  T_base_wrist3 * T_wrist3_camera_optical * T_camera_optical_target
```

The active transform is published by one node only:

```text
wrist3_Link -> camera_link -> camera_color_frame
                            -> camera_color_optical_frame
```

The first edge comes from `extrinsics_publisher`; the remaining camera-internal
edges come from the driver. Before the first accepted calibration, the
publisher uses the nominal mount transform and logs a warning; a corrupt or
mismatched `active.yaml` no longer crashes the node — it falls back to nominal.

## 配置

All translations are in metres. 主要参数（`config/calibration.yaml`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `board_columns/rows` | 11 / 8 | 棋盘内角点 |
| `board_square_size_m` | 0.020 | 格宽，**务必实测** |
| `frames_per_pose` | 5 | 每位姿采样帧数 |
| `min_samples` | 12 | 最少接受样本数 |
| `settle_duration_s` / `stable_timeout_s` | 0.5 / 10.0 | 停稳判据 |
| `planning_attempts` / `planning_time_s` | 5 / 5.0 | MoveIt 规划 |
| `position_tolerance_m` / `orientation_tolerance_rad` | 0.001 / 0.01 | 位姿到达容差 |
| `max_reprojection_rms_px` | 1.0 | 重投影质量门 |
| `max_translation_rms_m` / `max_rotation_rms_deg` | 0.003 / 0.5 | 一致性质量门 |
| `min_rotation_span_deg` | 20.0 | 腕部旋转跨度质量门 |

Edit `config/poses.yaml` to change the calibration trajectory.

## 测试

```bash
colcon test --packages-select aubo_hand_eye_calibration
# 或直接
python3 -m pytest src/aubo_hand_eye_calibration/test/ -v
```

For a real run, verify the robot workspace and collision model, keep an
emergency stop within reach, start at low speed, and use the plan-only step
before authorizing motion.
