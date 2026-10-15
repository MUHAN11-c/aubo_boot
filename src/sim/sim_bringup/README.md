# sim_bringup

仿真子工作空间（`src/sim/`）的入口包：launch + gz 桥接 + Nav2 配置 + 自主采摘编排。

```bash
# 统一机体主入口（P2）：可驾驶履带 + gz_ros2_control 臂 + MoveIt（默认开、
# 无头）。spawn 三级：spawn_x/spawn_y/spawn_yaw 显式给定 > spawn:=work|corridor
# （默认 work=作业位 0.95 m，感知深度窗/臂展/拍照位都按此标定；corridor=行间
# 走廊中心，程序化世界导航起步位）。
ros2 launch sim_bringup sim.launch.py                       # 无头 + moveit
ros2 launch sim_bringup sim.launch.py gui:=true rviz:=True  # gz GUI + RViz（经 moveit_sim 链带 MoveIt 参数）

# 自主导航采摘全栈（P4 演示入口）：sim + perception + Nav2 一键。
# field:=true（默认）= 重建果园（field_no_grass GLB + generate_field 地图/站位，
# spawn 显式同源）；field:=false = 程序化世界（确定性回归，spawn:=corridor）。
# 采摘编排在独立终端跑（不在 launch 内自动发起，与真机意图纪律一致）：
ros2 launch sim_bringup harvest.launch.py                   # 等 1-2 分钟世界首载 + AMCL 就绪
ros2 run sim_bringup orchard_harvest --rows 2 --max-stations 3
# 每站节拍：harvest_stow 行驶 → NavigateToPose 到站 → global_photo_pose 拍照位
# → begin_scene 锁集 → build_target_model → tcp 沿袋轴接近 0.12 m → 下一站。
# 常用参数：--max-targets（默认 1）、--all-stations（默认只访 manifest
# reachable>0 的有果站）、--stations-file（generate_field 产物；field 模式默认
# 同源 maps/field_stations.yaml）、--nav/--scene/--build-timeout（低 RTF 放宽）。

# 感知闭环（P3，与 sim.launch.py 同生命周期起）：
ros2 launch sim_bringup perception.launch.py
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint], points: [{positions: [0.425083, 0.195177, 1.677740, 1.461739, -0.500161, 0.038621], time_from_start: {sec: 5}}]}}"   # 拍照位
ros2 service call /peach_scene_perception_node/begin_scene \
  peach_interfaces/srv/BeginScene "{request_id: r1, scene_key: sim_orchard}"
# → /peach/perception/target_observations（锁集）→ /peach/perception/initial_pose
# → build_target_model action（min_views=1 单视收口）

# 旧锚定/mobile 两模式（P1 前行为保留）
ros2 launch sim_bringup orchard_sim.launch.py
ros2 launch sim_bringup orchard_nav.launch.py               # Nav2（mobile 底盘）

# 重建整园查看（621M GLB 首载约 1 分钟；P2 起带不可见地面碰撞，P4b 起带
# Sensors(ogre2) 渲染——腕载 rgbd/激光可成像；也是 harvest.launch 的 field 世界）
ros2 launch sim_bringup orchard_sim.launch.py \
  world:=install/sim_worlds/share/sim_worlds/worlds/field_no_grass.sdf mobile:=false
```

## 运行须知

- **感知栈与 gz 同生命周期**：gz 重启会让仿真时钟回零，长活的感知节点
  message_filters 被时间倒流卡死（实测注册表冻结）——重启 gz 必须连带重启
  `perception.launch.py`。
- **spawner 超时**：重世界（368 模型）冷启动时 gz 内 controller_manager 服务
  调用 >10 s，spawner 必须 `--service-call-timeout 60`（sim.launch.py 已配），
  否则 load 重试撞 already-loaded、JSB 卡 unconfigured。
- **odom_tf_relay**（console 脚本，sim.launch.py 自动挂）：gz OdometryPublisher
  的 TF 子帧用模型 canonical link 名（`harvester/base_footprint`）且不可配置，
  本节点订 `/odom` 重发 `odom → platform_base_link`，接续 REP-105 帧树
  （Nav2 官方 tutorial 同类 odom_tf_broadcaster 模式）。
- launch 参数速览：`sim.launch.py` = scene_params / world / gui / moveit / rviz /
  rviz_config / spawn / spawn_x / spawn_y / spawn_yaw / tool_profile；
  `harvest.launch.py` = use_sim_time / field / world / map / slam / nav2_params /
  gui / moveit / rviz。
- `config/sim_bridge.yaml` — 统一机体桥（clock/odom/tf/scan 入、cmd_vel 出、
  腕载 rgbd 四路 → `/camera/*` 感知契约 + `/camera/color/image_raw/camera_info`
  显示端别名（RViz Jazzy 按 image_transport 约定派生，仅显示端消费）；
  `/joint_states` 由 JSB 原生发布不经桥）。
- `config/nav2_params_field.yaml` / `config/nav2_params.yaml` — harvest.launch 的
  field / 程序化预设各自 Nav2 参数；`config/orchard_bridge.yaml` /
  `config/gz_gui_ogre1.config` / `config/sim_harvest.rviz` — orchard_sim 与
  orchard_nav 桥 / gz GUI / RViz（harvest、sim 经 moveit_sim 链起）用。
- `launch/moveit_sim.launch.py` — sim 自有 MoveIt 前端（sim.launch.py include）：
  统一机体 URDF、规划帧 `platform_base_link` 接 map/odom 主 TF 树；SRDF/IK/
  Pilz/控制器映射复用 `aubo_e5_moveit_config`（`controllers_mock.yaml`）。
- 场景参数唯一事实源在 `sim_worlds/config/orchard.yaml`；机体在 `sim_description`。

测完 Ctrl+C 停栈并 `pgrep -af 'gz sim|ros2|rviz'` 清残留（MUST）。
许可 BSD-3-Clause。
