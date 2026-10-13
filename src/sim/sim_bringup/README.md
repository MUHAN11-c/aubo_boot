# sim_bringup

仿真子工作空间（`src/sim/`）的入口包：launch + gz 桥接 + Nav2 配置。

```bash
# 锚定建模场景（默认 peach_orchard 世界 + 完整臂链，gui 开）
ros2 launch sim_bringup orchard_sim.launch.py

# 自主导航（可驾驶履带 + Nav2；rviz 默认开、gz gui 默认关）
ros2 launch sim_bringup orchard_nav.launch.py
ros2 launch sim_bringup orchard_nav.launch.py rviz:=False gui:=false   # 无头
ros2 launch sim_bringup orchard_nav.launch.py slam:=True rviz:=False   # SLAM 校验

# 重建整园查看（固定工位，621M GLB 首载约 1 分钟）
ros2 launch sim_bringup orchard_sim.launch.py \
  world:=install/sim_worlds/share/sim_worlds/worlds/field_no_grass.sdf mobile:=false
```

- `config/orchard_bridge.yaml` — mobile 模式六路 gz↔ROS 桥（clock/joint_states/odom/tf/scan 入、cmd_vel 出）。
- `config/nav2_params.yaml` — Nav2 全参（帧 `platform_base_link`、果园限速）。
- `config/gz_gui_ogre1.config` — 本机图形栈可用的 gz GUI 附着窗配置（QT 强制 GLX + OGRE1）。
- 场景参数唯一事实源在 `sim_worlds/config/orchard.yaml`；机体在 `sim_description`。
- P2 落地 `sim.launch.py`（统一机体 + gz_ros2_control + MoveIt），P3 落地 `perception.launch.py`。

测完 Ctrl+C 停栈并 `pgrep -af 'gz sim|ros2|rviz'` 清残留（MUST）。
许可 BSD-3-Clause。
