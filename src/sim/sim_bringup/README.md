# sim_bringup

仿真子工作空间（`src/sim/`）的入口包：launch + gz 桥接 + Nav2 配置。

```bash
# 统一机体主入口（P2）：可驾驶履带 + gz_ros2_control 臂 + MoveIt（默认开、
# 无头）。spawn 在作业位（0.95 m，感知深度窗/臂展/拍照位都按此标定）。
ros2 launch sim_bringup sim.launch.py                       # 无头 + moveit
ros2 launch sim_bringup sim.launch.py gui:=true rviz_only…  # gz 自带 GUI

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

# 重建整园查看（固定工位，621M GLB 首载约 1 分钟；P2 起带地面碰撞）
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
- `config/sim_bridge.yaml` — 统一机体桥（clock/odom/tf/scan 入、cmd_vel 出、
  腕载 rgbd 四路 → `/camera/*` 感知契约；`/joint_states` 由 JSB 原生发布不经桥）。
- `config/orchard_bridge.yaml` / `config/nav2_params.yaml` / `config/gz_gui_ogre1.config`
  — orchard_sim/orchard_nav 用。
- 场景参数唯一事实源在 `sim_worlds/config/orchard.yaml`；机体在 `sim_description`。

测完 Ctrl+C 停栈并 `pgrep -af 'gz sim|ros2|rviz'` 清残留（MUST）。
许可 BSD-3-Clause。
