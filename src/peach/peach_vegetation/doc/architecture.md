# `peach_vegetation` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包。系统级架构在 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。**源码与本文互相更新，改一边须同一轮改另一边**。

#### `peach_vegetation`（枝叶，影子）

- **作用：** GPU 枝/叶二维分割。发布 `/peach/vegetation/leaf_mask`、`branch_mask`、`overlay`、`status`。**不写 PlanningScene、不删 octomap、不发运动。** 独立 `ros2 launch peach_vegetation vegetation.launch.py`，不进 lifecycle 名单、不随 `harvest_system` 起（与 YOLO/MobileSAM 分进程同卡，避免对打）。
- **算法：** 直接构造 `FrangiExgSplitter`（torch Hessian Frangi + Excess Green/HSV）。无卡 `device: auto` 回退 CPU。后续 DeepLab 木类可换同一 `split()` 面，不要先扩 yaml `*.impl` 注册表。
