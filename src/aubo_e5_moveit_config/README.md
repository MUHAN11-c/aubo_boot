# aubo_e5_moveit_config

AUBO E5 的 MoveIt 2 配置：规划组、SRDF 命名位姿、OMPL、simple controller manager。技能节点的拍照位姿（默认 `global_photo_pose`）和 MTC 都走这里。

关节名必须与 URDF 权威六轴一致。规划组与控制器映射跟透传控制器对齐。

bringup 在 `moveit_enabled` 为真时拉起 `move_group` 和 `rviz/moveit.rviz`。Displays 分组 **Peach** 订感知/重建/技能可视化（固定系 `base_link`）。采摘技能依赖 Active 的 MoveIt。
