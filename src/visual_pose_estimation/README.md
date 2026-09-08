# visual_pose_estimation

本目录是移植到本工作区的**模板匹配视觉估姿**（非采摘感知）。ROS 2 包在子目录 `visual_pose_estimation_python/`。IDL 走 `ivg_interfaces`，**不走** `peach_interfaces`。不进 `harvest_system.launch.py`、不进 lifecycle。

| 路径 | 说明 |
|------|------|
| `visual_pose_estimation_python/` | 估姿节点 + FastAPI Web（8088） |
| `templates/` | 工件模板（launch / `template_root` / `VPE_TEMPLATE_ROOT`） |

C++ 旧包与旧仓 `interface` 包未移植；服务类型在 `ivg_interfaces`。

构建与接口见 [`visual_pose_estimation_python/README.md`](visual_pose_estimation_python/README.md)；活文档口径见 [`docs/architecture.md`](../../docs/architecture.md) §3 旁路视觉抓取、[`docs/io.md`](../../docs/io.md) §8。
