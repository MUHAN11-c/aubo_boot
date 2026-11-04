# 联合仿真接口

本组接口只服务隔离仿真，不向真机发命令或 SetIO。使用 `scripts/sim` 建立 domain/partition 与文件写入边界。

## 作业动作

- `/sim_harvest/run_harvest` — `peach_interfaces/action/RunHarvest`。服务端 `sim_mission`，运行器/CLI 显式发起。要求非空 `request_id`，真值模式支持 PICK_ALL/PICK_SELECTED 与 `target_ids`，视觉模式仅支持 PICK_ALL + 空 target_ids；支持单目标超时/采收比例；以结果 summary 和逐目标结果入账，不承诺未实现的调查 intent。
- `/sim_harvest/pickup` — `moveit_msgs/action/Pickup`。服务端 `sim_grasp`，任务客户端串行派发；要求正确 group 和一个 map 系抓取候选；物理实体最终必须属于本次 manifest。真值模式 target_name 为实体 ID；视觉模式为观测 ID，Grasp.id 是完整模型身份元组的 SHA256，服务端再依据视觉位置严格关联唯一实体。阶段反馈为 `state` 字符串，成功以 `MoveItErrorCodes.SUCCESS` 表示完整采摘落筐与收臂。
- `/navigate_to_pose` — `nav2_msgs/action/NavigateToPose`，Nav2 标准服务端。
- `/move_action` — `moveit_msgs/action/MoveGroup`，MoveIt 标准规划/执行服务端。只有操作适配层派发臂目标，任务不另开轨迹写口。

视觉模式另消费 `/peach_scene_perception_node/begin_scene`（BeginScene）、`/peach/perception/target_observations` 和 `/peach_target_reconstruction_node/build_target_model`（BuildTargetModel）。视觉绑定状态通过 `/peach_supervisor/state` 提供给共用能力，限定在本次仿真 domain。

- `/sim_harvest/survey_scene` — `peach_interfaces/action/SurveyScene`，与 Pickup 共用臂锁及跨进程作业锁；视觉采集前移动并验证仿真观察位。

## 底盘与机械臂互锁

命令链为 Nav2 velocity_smoother → `sim_base_guard` → Nav2 CollisionMonitor → Gazebo。互锁只发布到 `/cmd_vel_interlocked`，CollisionMonitor 仍是最终 `/cmd_vel` 发布者。机械臂动作前关闭底盘门并核验实际停车；未知/过期关节、断时钟、伸臂或恢复态均只输出零速度。导航与臂动作还使用本次目录的 POSIX `operation.lock` 互斥，避免两个公开动作入口同时取得运动权。

`/sim_harvest/arm_busy`、`base_locked`、`recovery_required` 均为标准 Bool、RELIABLE/TRANSIENT_LOCAL。仿真门不替代真实机器人的 ExecutionAuthority 或硬件安全回路。

## 状态与证据

- `/sim_harvest/state` — `peach_interfaces/msg/HarvestState`，RELIABLE、TRANSIENT_LOCAL、depth 1；仿真任务状态/目标/进度。
- `/sim_harvest/targets` — `visualization_msgs/msg/MarkerArray`，RELIABLE、TRANSIENT_LOCAL、depth 1；RViz 目标，完成后删除相应标记。
- `/sim_harvest/events` — `std_msgs/msg/String`，JSON 阶段证据，含墙钟、ROS 时间和阶段字段；同步写本次目录的 `sim_mission.jsonl` / `sim_grasp.jsonl`。
- `/sim_harvest/entity_poses` — `tf2_msgs/msg/TFMessage`，桥接原生 PosePublisher 的 `/sim_harvest/poses`（SceneBroadcaster 不保留实体帧元数据）；仅作为实际机器人/果实位置证据，不发布到 `/tf`。
- `/sim_harvest/<id>/state` — `std_msgs/msg/String`，原生 Gazebo `attached` / `detached`；变化时发布，必须先建立订阅再创建动态果实。
- `/sim_harvest/<id>/attach`、`detach` — `std_msgs/msg/Empty` → Gazebo Empty；动态副本名为 `<id>_picked`、link 为 `bag`。

`attached` 证明固定连接，`detached` 证明释放；它们都不单独证明采摘成功。结果还要求实际接触残差、轨迹执行、释放后果实位姿持续稳定在筐内及收臂后复核。切断后的失败不能重派为普通下一颗任务。

## 标准数据与服务

`/clock`（Clock）、`/joint_states`（JointState）、`/odom`（Odometry）、`/scan`（LaserScan）、`/tf`、`/tf_static`、`/map`、RGB-D `/camera/*` 均使用标准消息。六关节序为 shoulder、upperArm、foreArm、wrist1、wrist2、wrist3 的固定 AUBO 顺序。深度 32FC1 为米；16UC1 在图像边界换算，参数不改变 ROS 消息单位语义。

`/world/peach_orchard/create`（ros_gz_interfaces/SpawnEntity）、`remove`（DeleteEntity）负责仿真实体；`/apply_planning_scene`（ApplyPlanningScene）负责世界/附着碰撞对象；controller_manager 和 lifecycle GetState 用于就绪检查。

## 部署参数与产物

`harvest.launch.py` 的主要参数：`scene=data_orchard|demo|orchard|field`、`perception=ground_truth|vision`（默认 vision）、`runtime_dir`、`gui`、`rviz`、`tool_profile`、`max_targets`、`autostart=false`、`use_sim_time=true`。可显式覆盖 world/map/stations_file/manifest_file/scene_params；覆盖者负责保持同源。旧 `field=true|false` 分别映射 field/orchard。

运行器默认不自动发动作；`--mission` 明确派发，`--cancel-after` 在受理后请求取消，`--duration` 限定整个会话。产物位于 `.runtime/session_*/`：`robot.urdf`、同源物理适配 `robot.sdf`、`bridge.yaml`、`nav2.yaml`、场景/站位、launch.log、summary.json 和阶段 JSONL。`summary.json` 的 counts 是收到的数据条数，不能当作业务成功数；实际业务成功看 action result 与阶段证据。


## 2026-11-02 数据驱动小果园

当前默认 data_orchard / vision / adaptive_shear_v1，用户要求暂不运行 Gazebo，以下为实现契约，未完成动态验收。data_orchard 的 stations 只有树级 id、tree_id、nav_pose，不为视觉任务指定果实；manifest 包含真实数据约束资产、每果独立网格、物理包络和枝干圆柱。

- `/sim_harvest/move_to`：复用 `peach_interfaces/action/MoveTo`，当前只接受 KIND_NAMED + harvest_stow。与 Survey/Pickup 共用同一臂锁/进程锁、底盘停车门和取消链；用于空站观察后收臂。
- `/peach/reconstruction/grasp_decision`：`GraspDecision`，RELIABLE/TRANSIENT_LOCAL。执行侧不从 pose 推断许可；Grasp.id 与该消息的完整身份/有效期相符才可检查对应阶段能力。
- `/sim_harvest/run_harvest`：视觉模式仅 PICK_ALL + 空 target_ids；不支持的指定实体请求明确拒绝。max_targets=0 表示容量范围内不限；当前最多两颗，结果原因明确区分额度/容量/场景耗尽。
- `survey_camera_position`、`survey_look_at`：platform_base_link 系，默认相机 (0.20,-0.15,1.25)m、树冠点 (0,-0.60,1.25)m；从当前工具固定挂接 TF 计算观察 TCP，不查某颗真值。图像/重建仍只用精确曝光 TF。
- `perception_progress.json`：运行探针记录观测消息数、epoch、锁定和候选状态；`summary.json` 仍不能用图像条数替代采收结果。
