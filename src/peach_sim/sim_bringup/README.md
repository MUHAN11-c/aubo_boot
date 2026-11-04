# sim_bringup

2026-11-02 当前默认代码为 data_orchard + vision + adaptive_shear_v1；用户要求先代码与离线核对，暂不运行 Gazebo，动态闭环尚未验收。两株树、独立果实和同源碰撞见 [项目 README](../README.md)。以下旧默认与运行记录保留为历史说明。

**2026-10-29：用户明确恢复 Gazebo 联合仿真。** 当前入口组合统一机体、Gazebo Harmonic、Nav2、MoveIt 2、RViz2 和 `sim_harvest`。历史暂停记录及旧用法保留在文末，旧命令和历史成功结论不作为本轮验收证据。

在 `peach_sim/` 根目录使用：

```bash
scripts/sim build
scripts/sim test
scripts/sim run --gui --rviz --duration 480
scripts/sim run --gui --rviz --mission --max-targets 1 --duration 480
```

默认只启动，不开采摘任务；`--mission` 显式发动作，运行器有限时长运行并清理本次进程组。它通过 `bwrap` 只读挂载项目外文件系统，构建、安装、缓存和日志全部留在本项目；目录外包仅引用，不安装或修改。GUI 需要已有桌面/GPU 环境。

## 入口与配置

- `harvest.launch.py`：完整组合，默认 `scene=demo`、`perception=ground_truth`、`autostart=false`；统一 world/map/stations/spawn/AMCL 初始位姿。运行器始终以 `autostart=false` 启动，再显式发目标。
- `sim.launch.py`：官方 `ros_gz_sim` 入口，实体生成成功后依次激活 JSB、JTC；失败关闭整栈，无固定等待冒充就绪。
- `moveit_sim.launch.py`：读取 Gazebo 同一份生成 URDF；SRDF、关节限位和规划插件只读复用已有 AUBO 配置，执行控制策略由本包 `controllers_sim.yaml` 管理。
- `perception.launch.py`：当前 Peach 完整配置 + `sim_perception` 仿真 overlay，复用现行能力源码；输出限制到本次运行目录，bond 默认 8 秒。
- `orchard_sim.launch.py`、`orchard_nav.launch.py`：历史参数的兼容 include，不再维护第二套系统。
- `sim_bridge.yaml`：clock、odom、TF、scan、RGB-D、Gazebo create/remove、持果状态与实体位姿。`/sim_harvest/entity_poses` 用于证据核对，不写入机器人 TF 树。

RViz 配置包含机器人、地图/代价地图、激光、MoveIt 规划场景、腕相机与目标 Marker。`scene=field` 是整园 mesh 预览，不能执行独立果实采收；`ground_truth` 是明确的仿真真值源，不能作为视觉精度验收。视觉链的成功须另看检测/重建结果。

架构和接口见 [doc/architecture.md](doc/architecture.md)、[doc/io.md](doc/io.md)。测试只有官方 lint；本轮已验证有限时长启动与数据流，导航/采摘/视觉的结论必须各自有会话证据，不能沿用下列历史描述。

许可 BSD-3-Clause。

<details>
<summary>2026-10-28 及以前的历史用法与配置（保留证据，当前入口见上文）</summary>

仿真子工作空间（`src/peach_sim/`）的入口包：launch + gz 桥接 + Nav2 配置 + 自主采摘编排。

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
ros2 launch sim_bringup harvest.launch.py rviz:=true        # Gazebo 服务端 + RViz；等 1-2 分钟首载
# 每站节拍：收臂导航到袋心臂展内（默认 0.55 m，航向对准袋颈中心）→ 拍照位感知。
# 有目标和 id 才重建；每个 id：重建 → 拍照位 → 抓取 → 回拍照位，再下一颗。
# 拍照位 TCP 与 platform_base_link 同向，相机光轴为平台 -Y。
# field 预设（launch 默认）须显式带 --stations-file（tour_core 坐标是程序化
# 世界的，与重建果园不对应；2026-10-15 前空串默认会误入 field 分支崩溃，已修）：
ros2 run sim_bringup orchard_harvest --max-stations 3 \
  --stations-file $(ros2 pkg prefix sim_worlds)/share/sim_worlds/maps/field_stations.yaml
# 程序化世界（field:=false）才用 --rows 选行（默认全部行、只访有果站）：
ros2 run sim_bringup orchard_harvest --rows 2 --max-stations 3
# 常用参数：--max-targets（默认 1）、--all-stations（默认只访 manifest
# reachable>0 的有果站）、--nav/--scene/--build-timeout（低 RTF 放宽）。

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
- **里程计 TF 只有一路**：`OdometryPublisher` 的 `<robot_base_frame>` 设为
  `platform_base_link`，`/tf` 经 `sim_bridge.yaml` 桥进 ROS。`map → odom`
  仍只由 AMCL 或 SLAM 发布。
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
- 自测：`colcon test --packages-select sim_bringup` 只跑官方 flake8 / pep257。
  整栈是否起来，起 `sim.launch.py` 后看 `/joint_states` 和控制器日志。

测完 Ctrl+C 停栈并 `pgrep -af 'gz sim|ros2|rviz'` 清残留（MUST）。
许可 BSD-3-Clause。

</details>
