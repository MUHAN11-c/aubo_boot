# visual_pose_estimation（工作区子目录）

本目录聚合 **视觉位姿估计** 相关的多个 ROS 2 包，位于 `aubo_ros2_ws/src/visual_pose_estimation/`。

---

## 子包一览

| 目录 | 包名 | 说明 |
|------|------|------|
| `src/interface/` | **interface** | 公共 `msg`/`srv`（`ImageData`、`EstimatePose` 等） |
| `src/visual_pose_estimation/` | **visual_pose_estimation** | C++ 估姿节点与算法模块 |
| `src/visual_pose_estimation_python/` | **visual_pose_estimation_python** | Python 估姿 + FastAPI/Web UI（`web_ui/`） |

---

## 其它顶层目录

| 目录 | 说明 |
|------|------|
| `templates/` | 工件模板数据（由 launch / `template_root` 指向） |
| `docs/` | 补充文档 |
| `debug_snapshots/` | 调试快照（若存在） |

---

## 构建示例

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select interface visual_pose_estimation visual_pose_estimation_python
source install/setup.bash
```

各包详细用法见对应子目录内的 **`README.md`**。
