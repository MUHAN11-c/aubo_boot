# ADR 0001：按能力边界分包

## 决策

`peach_harvester` 拆为 `peach_scene_perception`、`peach_target_reconstruction`、`peach_supervisor`、`peach_scene_obstacles`；共享代码进 `peach_vision_core`/`peach_common`；`peach_harvester` 只保留 metapackage。

## 理由

- 与 Nav2/Autoware/Isaac ROS 的“能力包 + 共享 utils + 单一 IDL + bringup”一致。
- 能力包之间不互相 import，接口只走 `peach_interfaces`。
- 单包多节点导致配置、模型、launch、测试所有权混杂。

## 后果

- 每包独立 launch/config/doc/test。
- `peach_bringup` 只组合。
- `peach_sim` 不在该边界内，保持原状。
