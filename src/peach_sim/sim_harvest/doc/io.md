# sim_harvest 接口

- `/sim_harvest/run_harvest`：现有 `peach_interfaces/action/RunHarvest`；视觉模式 PICK_ALL + 空 target_ids；真值模式可指定实体。max_targets 按果数，0表示容量范围内不限，当前总容量2。结果至少采一颗时 success=true，termination_reason 区分 target_limit_reached/basket_capacity_reached/target_ratio_reached/scene_exhausted/no_targets_succeeded。
- `/sim_harvest/pickup`：标准 `moveit_msgs/action/Pickup`。一个 map 系 Grasp；真值 target_name 为实体ID，视觉为观测ID。视觉 Grasp.id 是 run/epoch/目标/模型/工具/标定/配置/有效期的身份哈希，最终动作必须与当前 GraspDecision 一致。
- `/sim_harvest/survey_scene`：现有 SurveyScene，按相机位置/朝向规划观察 TCP，不指定某颗真值。观察完成不代表已经检测到果实。
- `/sim_harvest/move_to`：现有 MoveTo，目前只支持 KIND_NAMED + harvest_stow，验证到位后返回；用于空站后安全换站，不开放通用额外运动源。
- `/peach/perception/target_observations`、BeginScene、BuildTargetModel：视觉发现和重建来源；`/peach/reconstruction/grasp_decision` 提供阶段许可；接口权威仍是现有 IDL。
- `/sim_harvest/events` JSON阶段证据和运行目录 sim_mission.jsonl/sim_grasp.jsonl；实体、连接、筐接触证据同系统接口文档。详见 [系统接口](../../sim_bringup/doc/io.md)。

部署参数 manifest_file、stations_file、run_directory、robot_description_file、tool_profile、perception_mode、use_sim_time；Survey 参数 survey_camera_position、survey_look_at 为平台系米制向量。仿真参数不写入目录外 Peach/驱动配置。

用户当前要求暂不运行 Gazebo；本轮仅离线验证，不宣称这些运行端点已被实测。
