# `peach_stereo` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包（职责短页）。

可选相机前端（`camera_frontend:=stereo`，**0036 起默认**；percipio 显式传参备用）：主机单图案立体 RGB-D，~13.7 FPS（hh4 档），话题与 percipio 同构（点云多一个 `confidence` FLOAT32 字段）。规格档案（参数档 / confidence 布局 / temporal_k / 双前端 A/B 对比）见 [README.md](../README.md)；前端选择与 /camera/* 契约在 [peach_bringup/doc/io.md](../../peach_bringup/doc/io.md)。文件树见 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。
