# Web API 调用速览（当前实际面）

前端（`web_ui/index.html` + `scripts/app.js`）→ FastAPI（默认 `127.0.0.1:8088`）→ ROS 2。

## 可用端点

| 端点 | 方法 | 作用 |
|---|---|---|
| `/status`, `/health` | GET | 服务与 ROS 桥状态 |
| `/exit` | POST | 退出 Web 进程 |
| `/api/capture_image` | POST | 软触发（`/camera/soft_trigger`）→ 缓存 depth/color 帧 |
| `/api/estimate_pose` | POST | 3D 位姿估计（调 `/estimate_pose` 服务） |
| `/api/estimate_pose_2d` | POST | 2D 位姿估计（调 `/estimate_pose_2d` 服务） |
| `/api/list_templates` 等 `/api/*template*` | POST/GET | 模板列表 / 读取 / 标准化 / 模板图下载 |
| `/api/debug/*` | POST | 调试图像、参数滑块、阈值持久化 |
| `/api/get_robot_status` 等运动/IO/抓取端点 | POST | **一律 501**（本区不发运动/IO；真机走 harvest 调试面或 `graspnet_ros2`，须授权） |

## 设计要点

- 运动与 IO 不经 Web：旧仓的 `/move_to_pose`、`/set_robot_io`、`/execute_single_grasp`
  等服务在本工作区不存在，对应 HTTP 端点保留但固定返回 501。
- 相机软触发是 `std_msgs/String` 话题 `/camera/soft_trigger`（Percipio 驱动），
  不是旧仓的 `SoftwareTrigger` 服务。
- 调试阈值的事实源是 `web_ui/configs/default_config.yaml`（经 `ConfigReader`）；
  滑块保存的 `debug_thresholds.json` 仅做 Web 侧持久化回显。
