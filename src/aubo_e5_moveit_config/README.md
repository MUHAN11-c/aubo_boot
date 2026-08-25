# aubo_e5_moveit_config

AUBO E5 的 MoveIt 2 配置。技能的拍照位姿和 MTC 都读这里，不另放工作区根目录。

关节名必须与 URDF 权威六轴一致。规划组与控制器映射跟透传控制器对齐。

bringup 在 `moveit_enabled` 为真时拉起 `move_group` 和 `rviz/moveit.rviz`。Displays 分组 **Peach** 订感知/重建/技能可视化（固定系 `base_link`）。采摘技能依赖 Active 的 MoveIt。

## 本包有什么

| 文件 | 用途 |
|------|------|
| `config/aubo_e5.srdf` | 规划组 `manipulator_e5`；示教命名位姿 |
| `config/kinematics.yaml` | IK |
| `config/ompl_planning.yaml` / `pilz_*.yaml` | 规划器 |
| `config/controllers.yaml` / `controllers_mock.yaml` | 与透传控制器对齐 |
| `config/joint_limits.yaml` | 关节限 |
| `launch/moveit.launch.py` | move_group + RViz |
| `rviz/moveit.rviz` | 显示 |

## 示教位姿（改 SRDF，不要写到工作区根）

`config/aubo_e5.srdf` 的 `group_state`：

| 名字 | 谁用 |
|------|------|
| `home` | 收拢 |
| `camera_pose` | 近景观察位 |
| `global_photo_pose` | 全局拍照；技能 `photo_pose_named_target` 默认指向它 |

示教后把关节角写进对应 `group_state`，技能 `SurveyScene` / `go_to_photo_pose` 按名字去 MoveIt。

## 不在本包

手眼外参（`wrist3_Link → camera_link`）在 [`aubo_hand_eye_calibration`](../aubo_hand_eye_calibration/README.md) 的 `hand_eye/active.yaml`。MoveIt 只消费 TF，不存这份 yaml。
