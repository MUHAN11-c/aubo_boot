# peach_scene_obstacles

退出时若SIGINT已关闭rclpy上下文，允许executor取消息阶段的RuntimeError随关闭流程结束；上下文仍有效时的运行错误继续抛出，不将真实故障伪装成正常退出。

Peach Survey-triggered PlanningScene obstacle snapshot node.

开启 `enabled` 后，每次 Survey 固定触发时的最近一帧点云。触发时还没有点云则等首帧。
节点按点云曝光时间查询相机和各网格连杆到 `base_frame` 的 TF，体素化并滤除自身后缓存。
缺少源时间、坐标系或该时刻的 TF 会使本次构建失败，已有 PlanningScene 保持到下一次成功写入。
本轮后续精化只更新这些体素上的目标胶囊滤除。新 Survey 清除上一轮胶囊。

非空快照通过 `/apply_planning_scene` 的 `CollisionObject.ADD` 替换同名对象。
禁用或快照为空时，先用 `/get_planning_scene` 的 `WORLD_OBJECT_NAMES` 核实对象是否存在。
存在则按 id 删除，包括上一实例遗留的对象。服务超时或对象未能移除会记录失败诊断。
`max_boxes` 的下限为 1，启动和运行时均校验，被拒绝的值不会进入下一份快照。
