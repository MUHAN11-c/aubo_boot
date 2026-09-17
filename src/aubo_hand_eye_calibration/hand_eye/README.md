# 手眼标定唯一事实源（2026-09-17 整理）

**`active.yaml` 是全项目唯一在用的手眼外参**（`wrist3_Link → camera_link`），
extrinsics_publisher 启动时从本目录读取（`storage.py: default_storage_directory()`）。

## 如何修改

直接改 `active.yaml` 的值，或用新标定结果**整个覆盖**本文件，然后重启外参发布器
（或整栈）生效：

```bash
pkill -INT -f extrinsics_publisher
ros2 run aubo_hand_eye_calibration extrinsics_publisher \
  --ros-args -r __node:=hand_eye_extrinsics_publisher
ros2 run tf2_ros tf2_echo wrist3_Link camera_link   # 核对数值
```

注意 yaml 内 `frames.*` 必须与发布器 parent/child 一致，否则回退名义值并告警。

## 目录内

- `active.yaml` —— 在用外参（入库随仓，任何机器 clone 即得）
- `candidates/` —— 标定会话候选产物（.gitignore 忽略，仅本机）

## 边界（勿混淆）

- `_archive/runs/hand_eye/active.yaml` 是**历史归档**，不被任何代码读取——
  2026-09-17 之前标定目录曾被 .gitignore 整体忽略，机器清理后只剩该归档副本，
  由此引发过"点云位置不对"的排查（testing-log 09-17）。
- `AUBO_HAND_EYE_DIR` 环境变量可覆盖本目录（特殊部署用，日常勿设）。
- 彩色相机内参的唯一事实源是 `src/percipio_camera/config/color_camera_info.yaml`
  （percipio 与 peach_stereo 两个相机前端共用同一份）。
