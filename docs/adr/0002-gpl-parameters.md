# ADR 0002：参数使用官方 generate_parameter_library

## 决策

新迁移节点使用 `generate_parameter_library`；已迁移：`peach_scene_obstacles`、`peach_bringup` 辅助节点。`attach`/`param_rules` 逐步退役。

## 理由

- 官方 C++/Python 参数生成；类型、描述、约束、动态回调与 `ros2 param describe` 一致。
- 减少自定义参数层和重复校验。
- 部署 YAML 仍保留为事实源，GPL schema 提供默认值/描述/校验。

## 后果

- 每次迁移必须验证 `ros2 param describe`、动态 set、`failure_code` 等行为。
- 未迁移节点暂保留 `attach`，不 dual-run。

## 已知上游缺陷与处理

- 本机 Jazzy apt 的 `generate_parameter_library` **0.7.6** 对整数 `gt_eq`/`lt_eq` 生成 `IntegerRange.from_value = 1.0`（float），`ros2 param describe` 会触发 `rcl_interfaces` 的 `PyLong_Check` 断言并 abort。
- 处理：schema 不生成整型 range；由节点在 GPL listener 之前注册 `SetParametersResult` 回调校验整数上下界（target/supervisor/scene_obstacles/bringup）。
- 该 workaround 不改变部署 YAML 键名；后续升级 GPL 时重新评估。

## 迁移完成状态

六个生产/观测包已完成：`peach_scene_obstacles`、`peach_bringup` 辅助节点、`peach_supervisor`、`peach_target_reconstruction`、`peach_scene_perception`、`peach_observability`。`peach_vegetation` 不在本轮采摘主链范围，仍保留 `attach`。

## 版本边界

本机是 ROS Jazzy apt 的 0.7.6，不是上游 main 最新（记录时 1.3.1），也不是 ROS REP 核心标准。升级/替换 GPL 时须重新验证 Python 生成器与 `IntegerRange` 行为。
