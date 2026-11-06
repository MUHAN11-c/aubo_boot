# peach_scene_obstacles 接口

默认节点/console script 为 `peach_scene_obstacles`；没有自定义 service server、action 或生命周期转换。跨包图名对照 [interface_manifest.yaml](../../peach_interfaces/config/interface_manifest.yaml)。

## 输入和 MoveIt 客户端

- `/camera/depth_registered/points`：PointCloud2，RELIABLE / VOLATILE / KEEP_LAST(5)。只有 enabled 时订阅；Survey 取最近一帧，等待首帧时用下一帧。字段 x/y/z 为米，header 必须含源曝光 stamp 和相机 frame。
- `/peach/scene/obstacles_refresh`：Empty，RELIABLE / TRANSIENT_LOCAL / depth 1，由 supervisor 在 Survey 成功后触发。
- `/peach/reconstruction/refined_pose`：BagGraspCandidateArray；`/robot_description`：String URDF。均 RELIABLE / TRANSIENT_LOCAL / depth 1。胶囊取 bag_bottom、bag_neck 和 bag_diameter_upper_m，须同本轮 base_frame，且消息时间不早于 Survey 开始。
- `/apply_planning_scene`：moveit_msgs/srv/ApplyPlanningScene，写 world.collision_objects diff；非空时 CollisionObject.ADD 替换同名对象，清场时 REMOVE。
- `/get_planning_scene`：moveit_msgs/srv/GetPlanningScene，只请求 WORLD_OBJECT_NAMES，用于清场前确认实际对象以及移除失败后的复查。
- TF：点云帧及 URDF mesh link 均按点云 stamp 查询 `base_frame ← frame`，每次查询超时 2 秒，无 latest 回退。

系统健康发布到 `/diagnostics`，任务名 obstacle_snapshot，周期 5 秒；字段包括 last_snapshot_age_s、capsules、cloud_cached，报告未触发/最近成功/最近失败。没有独立障碍点云或 marker 发布口。

## 部署参数

`scene_obstacles_parameters.yaml` 生成声明及基础约束，params.py 补充 max_boxes>=1；scene_obstacles.yaml 提供部署值。当前 enabled=false、object_id=peach_scene_obstacles、base_frame=base_link、voxel_size_m=0.12、max_boxes=3000。自身余量、胶囊余量、工作空间半径随 SnapshotParams 在下次 Survey 冻结。

cloud_max_age_s=2.0 是告警阈值，超龄仍按快照使用；apply_service_timeout_s=5.0 是异步回执等待预算，服务发现另有 2 秒等待。配置值以 YAML 为准。object_id 为空会拒绝写入；改名不会自动删除旧名字对象，还须同步臂侧 ACM 引用并清理旧对象。

独立 launch 只暴露 namespace 和 use_sim_time，不暴露 autostart。业务话题与 MoveIt 客户端为绝对名，不随 namespace 改名；不使用 launch 参数冒充 lifecycle 管理。
