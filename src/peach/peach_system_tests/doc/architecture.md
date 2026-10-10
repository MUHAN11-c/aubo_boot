# `peach_system_tests` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包（职责短页）。

isolated launch_testing（mock `harvest_system`，域 89–94）+ 回放塔（`test_replay_approach`：现场真袋案册 / 分层 / 随机语料对照冻结基线 `replay_baselines.json`）+ perf 基线 + 冻结相机一次感知抓一颗（`frozen_camera_grasp.launch.py`，域 81/82）。**只进 `colcon test`，不进运行 launch、不进真机。**测试流程指导（怎么跑/命名/验收门）在 [doc/testing.md](testing.md)；文件树见 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。
