# aubo_hand_eye_calibration

眼在手上标定：采集标定板、解外参、落盘，并由 `extrinsics_publisher` 发静态 TF。

感知与重建用**精确时间戳**查 `base_link ← camera`；外参 TF 是这条链的一部分。标定坏了，世界系身份和 ICP 初值都会偏。

## 结果在哪（本包内，不进工作区根）

| 路径 | 内容 |
|------|------|
| `hand_eye/active.yaml` | 当前激活外参（gitignore，不入库） |
| `hand_eye/candidates/` | 历次求解候选 |
| `config/calibration.yaml` | 棋盘格、质量门、坐标系名 |
| `config/poses.yaml` | 采集用的标定位姿 |

可用 `AUBO_HAND_EYE_DIR` 覆盖。无源码树时落到 `~/.ros/aubo_e5/hand_eye`。不要写进 `install/share`（colcon 会盖掉）。

拍照示教位姿（`global_photo_pose` 等）在 [`aubo_e5_moveit_config`](../aubo_e5_moveit_config/README.md) 的 SRDF，不在本包。

## 节点

- `extrinsics_publisher`：读 `hand_eye/active.yaml`，发静态 TF；可 `reload`
- 标定 server：动作/服务跑采集与求解

bringup 默认可开外参发布。全标定流程单独 launch，不要在日常采摘里自动跑。无 `active.yaml` 时发名义 TF（平移 2 cm、单位四元数），光学系会偏。现场副本在 `_archive/runs/hand_eye/`。
