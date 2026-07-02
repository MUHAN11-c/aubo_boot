# visual_pose_estimation（视觉位姿估计）

本目录聚合 **视觉位姿估计** 相关的 ROS 2 包，位于 `aubo_ros2_ws/src/visual_pose_estimation/`。

---

## 包版本说明

> **当前主力版本：`visual_pose_estimation_python`（Python）**
>
> `visual_pose_estimation`（C++）为早期实现，已将核心算法移植到 Python 版本后不再维护。
> 新功能开发、调试、部署请基于 Python 版本进行。

---

## 子包一览

| 目录 | 包名 | 状态 | 说明 |
|------|------|------|------|
| `src/interface/` | **interface** | 共用 | 公共 `msg`/`srv`（`ImageData`、`EstimatePose` 等），C++/Python 包共用 |
| `src/visual_pose_estimation_python/` | **visual_pose_estimation_python** | **主力** | Python 估姿节点 + FastAPI Web UI（`web_ui/`） |
| `src/visual_pose_estimation/` | **visual_pose_estimation** | 旧版 | C++ 估姿节点与算法模块（已移植至 Python，仅保留参考） |

---

## 其它顶层目录

| 目录 | 说明 |
|------|------|
| `templates/` | 工件模板数据（由 launch / `template_root` 指向） |
| `docs/` | 补充文档（预处理与特征检测算法说明） |
| `debug_snapshots/` | 调试快照 |

---

## 构建

```bash
cd ~/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash

# 主力版本（Python）
colcon build --packages-select interface visual_pose_estimation_python

# 如需编译旧版 C++ 包（仅参考）
# colcon build --packages-select visual_pose_estimation

source install/setup.bash
```

---

## 文档索引

| 文档 | 说明 |
|------|------|
| `src/visual_pose_estimation_python/README.md` | **Python 版本完整文档**（配置、使用、架构、接口） |
| `src/interface/README.md` | ROS2 接口定义说明 |
| `docs/preprocess_and_feature_logic.md` | 预处理与特征检测算法（Python 版本对应） |
| `src/visual_pose_estimation/README.md` | C++ 旧版文档（仅参考） |
