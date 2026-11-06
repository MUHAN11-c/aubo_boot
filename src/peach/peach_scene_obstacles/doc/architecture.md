# peach_scene_obstacles 架构

## 职责与源码地图

普通 Node `peach_scene_obstacles`，由独立 launch 或整栈 camera_enabled 分支启动，不加入 lifecycle 管理。默认 enabled=false；负责将一次 Survey 的源点云冻结成碰撞方块，后续仅随精化目标胶囊重滤同一份体素。

- `node.py`：点云/URDF/胶囊缓存、Survey 世代、精确时间 TF、单飞工作线程、MoveIt 场景读写和诊断。
- `core.py`：URDF 关节/网格解析、几何 FK 辅助、STL 加载、自身/胶囊滤除、体素中心及快照统计；没有 ROS import，Open3D 仅在网格/自身滤除处延迟加载。当前节点为每个 mesh link 查询曝光 TF，不使用辅助 fk_link_transforms 猜缺失关节。
- `params.py`：GPL 实时视图、max_boxes 下限、Survey 滤除参数快照；`config/scene_obstacles_parameters.yaml` 是生成 schema，`scene_obstacles.yaml` 为部署值。
- `launch/scene_obstacles.launch.py`、setup.py、setup.cfg、resource 和 package.xml 定义入口、安装、ament 及依赖；`test/` 仅有 ament flake8/pep257 包装。

## 快照流程

1. enabled 时保留最近 PointCloud2。收到 obstacles_refresh 后清上一轮胶囊，冻结当前点云、base_frame 和滤除参数；尚无点云则等待首帧和 robot_description，不提前消耗刷新请求。
2. 0.5 秒泵启动至多一个工作线程。源点云必须有非零 stamp 和 frame_id，点云相机及每个 mesh link 都按该 stamp 查询到 base_frame 的 TF。源帧超 cloud_max_age_s 只 WARN，仍使用；TF/数据错误使本次失败，已有场景不被替换。
3. `survey_voxels` 先体素去重、裁工作空间，再按网格距离剔除自身。自身余量包含体素半对角，避免保留方块角切入机器人表面。URDF 仅提取 collision mesh；若没有可用 mesh 会告警，当前实现仍继续构建，不能声称已完成自身滤除。
4. `filter_survey` 在冻结体素上剔除精化目标的膨胀胶囊，再按 max_boxes 截断。半径、轴向余量均额外计入体素半对角；没有精化几何的目标没有胶囊。只接受本轮开始之后、同 base_frame、有限且正直径的候选。
5. 非空快照用一个 ADD 对象替换同 id 的世界对象。禁用或空快照先查询实际世界名字，再按非空 id REMOVE，包括上次进程遗留对象；没有对象则无需写入。Apply 失败或超时记录错误，不伪称已移除。

节点只写 PlanningScene.world.collision_objects，ACM 由 peach_arm 独立写 diff；相机/机械臂的碰撞豁免不在本节点设置。新 Survey 更换源帧，旧 worker 在提交前核对当前 Survey 对象；作业期不随每帧相机流持续更新地图。

## 配置与验证

enabled 为实时读值，关闭时泵释放点云订阅并触发清场；重新开启后仍需新的 Survey。SnapshotParams 在 Survey 冻结，滤除参数修改不追改现有快照。object_id、服务超时和帧龄告警阈值是操作时读值，不能把所有参数统称为“重启才生效”。

完整接线见 [io.md](io.md)。静态 lint 不验证 MoveIt 实际替换、清场或自身滤除精度；运行验证与清理规则见 [测试指导](../../peach_system_tests/doc/testing.md)。
