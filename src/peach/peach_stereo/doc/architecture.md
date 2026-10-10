# `peach_stereo` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包（职责短页）。

可选相机前端（`camera_frontend:=stereo`，**0036 起默认**；percipio 显式传参备用）：主机单图案立体 RGB-D，~13.5–13.7 FPS（hh4 档，档位相关），话题与 percipio 同构（点云多一个 `confidence` FLOAT32 字段；percipio 另有 `device_event` 话题本前端不发）。launch `respawn=true, respawn_delay=3.0`（2026-09-29 起，掉线自动重连）；诊断任务 `stereo_health` 1 Hz（device_open/激光配置/rate<1.0gps WARN/四类丢弃计数）；激光/曝光/标定写入失败拒启，TIME_SYNC 失败仅 WARN 降级。**整栈 include 顺序坑**：stereo include 必须在 aubo bringup include 之前，否则 stereo 永不启动（见 harvest_system.launch.py 注释）。裸 `ros2 run` 不带 yaml 时 `temporal_k` 用 C++ 默认 1（无时域中值、confidence 恒 1.0），部署 yaml=3。规格档案（参数档 / confidence 布局 / temporal_k / 双前端 A/B 对比）见 [README.md](../README.md)；前端选择与 /camera/* 契约在 [peach_bringup/doc/io.md](../../peach_bringup/doc/io.md)。文件树见 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。
