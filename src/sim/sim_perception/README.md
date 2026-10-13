# sim_perception（P3 填充）

复制的 peach 感知栈（用户裁定快照复制，2026-10-13）：`scene_perception` +
`target_reconstruction` + `vision/common` + 两份 config。

- **快照来源**：`src/peach/peach_harvester`（peach 组持续演进，改进需人工同步到本包）
- **不复制**：`peach_interfaces`（防 IDL 双定义）、`peach_common`（yaml_params/qos/paths，exec_depend 复用）、
  模型权重 best.pt / mobile_sam.pt（运行时参数指向 peach_harvester 安装数据）
- 节点名 / 话题名保持 peach 原样（仿真独立 ROS domain，无冲突）
- 接入 `sim_bringup/perception.launch.py`，验收门 = `begin_scene` → `target_observations` → `build_target_model`

本文件为占位骨架；P3 落地后替换为正式 README 七要素。
