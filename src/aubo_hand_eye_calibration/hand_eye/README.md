# 手眼标定默认激活外参

**`active.yaml` 是眼在手上部署的默认激活外参**（`wrist3_Link → camera_link`）。
extrinsics_publisher 按 `storage.py::default_storage_directory()` 定位本目录；
`active_file` 或 `AUBO_HAND_EYE_DIR` 可覆盖。固定相机部署关闭该发布器后，
使用 bringup 的固定安装 TF，不从此文件建立腕部相机链。

## 如何修改

标定候选通过质量门后，用现有 `~/activate` 服务写入 active 并请求发布器 reload。
人工导入外参时须保留完整 schema、frames 与 transforms；修改文件后可通过已有服务
重新读取，无需按进程名批量停止节点：

```bash
ros2 service call /hand_eye_extrinsics_publisher/reload std_srvs/srv/Trigger '{}'
# 检查服务 success/message，再核对同一发布者的 wrist3_Link→camera_link TF。
```

yaml 内 `frames.wrist/camera_root` 必须与发布器 parent/child 一致。
启动时缺文件会发布 nominal 并报 WARN，文件损坏或帧名不符会 nominal 回退并报 ERROR；
运行期 reload 失败则返回 false，保留此前已发布的 TF，不自动改为 nominal。
激活写文件与 reload 不是事务：reload 失败可能已更新 active.yaml，须分别核对磁盘与当前 TF。

## 目录内

- `active.yaml` —— 在用外参（入库随仓，任何机器 clone 即得）；auto 档联合标定激活后另含 `intrinsics` 节（内参 K/D，`apply_intrinsics` 可直接读本文件落盘 percipio 事实源）
- `candidates/` —— 标定会话候选产物（.gitignore 忽略，仅本机）；joint 档候选含 `intrinsics`/`viewpoints`/joint 指标增量节

## 边界（勿混淆）

- `_archive/runs/hand_eye/active.yaml` 是**历史归档**，不参与默认路径查找——
  2026-09-17 之前标定目录曾被 .gitignore 整体忽略，机器清理后只剩该归档副本，
  由此引发过"点云位置不对"的排查（testing-log 09-17）。
- `AUBO_HAND_EYE_DIR` 环境变量可覆盖本目录（特殊部署用，日常勿设）。
- 项目彩色 CameraInfo 的内参覆盖文件为 `src/percipio_camera/config/color_camera_info.yaml`
  （percipio 与 peach_stereo 共用，不改设备 Flash/深度配准标定）。
  重标定工具与流程（`camera_calibration`（vendored 副本已删，现行 apt）+ `apply_intrinsics`）
  见包 README「内参标定流程」与 v1 测试指导标定节（`src/peach/peach_system_tests/doc/testing.md`）。
