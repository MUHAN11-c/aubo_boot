# peach_sim

室外套袋桃果园场景（Gazebo Harmonic / gz-sim 8）：参数化果园世界 + 采摘工位
（开源履带底盘改型 + AUBO E5 + 套袋刀）。**只做场景建模与摆位**——不进
`harvest_system`、不进 lifecycle、不发 `RunHarvest`，真机栈零改动。

## 目的

给感知/重建/接近链一个可复现的室外套袋桃场景：树行、套袋果、光照与工位几何
都由参数生成并带目标清单（GT），mock/回放之外多一层"带几何现场"的回归土壤。

## 公有 API

| 入口 | 说明 |
|------|------|
| `peach_sim.params` | 场景参数 schema + 校验（零 ROS；未知键/越界值/跨键约束拒绝生成） |
| `peach_sim.scene` | 布局与 SDF/清单生成纯核（零 ROS）：`render_scene()` 一次产出世界 SDF 与目标清单 |
| `ros2 run peach_sim generate_orchard` | 读 `config/orchard.yaml` 写 `worlds/peach_orchard.sdf` + `worlds/peach_orchard.manifest.yaml` |
| `ros2 run peach_sim scene_preview` | 把世界 SDF 正交投影成俯视/侧视 PNG（无 GL 环境看场景、自检摆位） |
| `ros2 launch peach_sim orchard_sim.launch.py` | 起 `gz sim` + 生成采摘工位 + `/clock` 与关节状态桥 |
| `urdf/harvester_robot.urdf.xacro` | 整机装配（履带车 + AUBO E5 + 腕载相机 + 快换 + 套袋刀），与真机帧契约同构 |

目标清单每颗给出世界系 `bag_bottom`（= `entry`，`entry_standoff_m=0`）、
`bag_neck`、`axis`（底→颈）、`fruit_center`、袋具尺寸与 `reachable` 标记，
可直接与 `runs/field_pregrasp_*` 口径对表。

## 例子

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/aubo_e5_jazzy_ws && colcon build --packages-select peach_sim && source install/setup.bash

# 改 config/orchard.yaml 后重生成世界与目标清单（同 seed 必同产物）
ros2 run peach_sim generate_orchard          # 加 --check 只校验不写文件

# 起场景（gui:=false 走无头 gz sim -s）
ros2 launch peach_sim orchard_sim.launch.py

# 无 GL 环境（gz GUI 起不来）看场景：俯视 + 作业切片侧视
ros2 run peach_sim scene_preview

# 看目标清单
python3 -c "import yaml;print(yaml.safe_load(open('src/peach_sim/worlds/peach_orchard.manifest.yaml'))['counts'])"
```

## 几何口径（来源）

- **工具**：`tool.*` 镜像 `aubo_description/config/adaptive_cylinder_v1.yaml`
  （内径 0.116 m、插入 0.2 m、径向余量 5 mm），`test_tool_mirror_matches_description` 对账；
  袋体上限 0.096 m + 双侧余量 = 0.106 m ≤ 内径，袋底→袋颈 0.09–0.12 m ≤ 插入深度。
- **现场包络**：袋底→袋颈 0.05–0.12 m、袋底在 `base_link` 上方 0.52–0.71 m、
  `|entry| ≈ 1.0 m`——取自 `src/peach_arm/test/fixtures/field_pregrasp_cases.yaml`
  与 `runs/field_pregrasp_*`，`test_targets_match_field_envelope` 对账。
- **履带底盘**：几何与履带摩擦改自 Gazebo Sim 官方示例
  [`worlds/tracked_vehicle_simple.sdf`](https://github.com/gazebosim/gz-sim) 的
  `simple_tracked`（Apache-2.0）：双履带 = 箱体 + 两端圆柱，ODE `mu 0.7 / mu2 150 / fdir1 0 1 0`。
- **确定性**：随机流按 `f'{seed}/{实体 id}'` 派生，与生成顺序无关；`test_render_is_deterministic`
  与"入库产物 == 生成器输出"双重对账。

## 怎么看（无 GL 也能看）

`gz sim` GUI / ogre2 需要 OpenGL；无 GL 环境（远程、沙箱、无 GPU）用
`ros2 run peach_sim scene_preview` 生成 `worlds/peach_orchard.preview.png`
（左：俯视 x–y 全园；右：侧视 x–z 作业切片。红=作业位可达目标，灰=不可达，红圈=可达包络，蓝框=履带车，橙点=臂座）。

## 怎么测

```bash
colcon test --packages-select peach_sim --event-handlers console_direct+
colcon test-result --all
```

纯核 pytest 覆盖：参数 schema/跨键约束、生成确定性、清单↔世界模型名一致、
袋具与工具余量、现场包络对齐、作业位可达性、树行网格对称。lint = `ament_flake8` + `ament_pep257`。

## 本轮边界（后续轮）

- **工位被锚在作业位**：`aubo_description/urdf/aubo_e5.urdf` 的 `world_joint`
  把 `base_link` 接到 URDF `world`，urdf2sdf 会把该模型固定到仿真世界——履带不可驾驶。
  移动化须把 world 锚点换成 `map → odom → base_link`（REP-105）并接
  `gz::sim::systems::TrackedVehicle` + `TrackController`（官方示例 `~L1060` 有完整配置）。
- **关节站姿**：六关节由 `gz-sim-joint-position-controller-system` 钉在真机拍照位
  （`photo_joints`，`src/peach_arm/test/fixtures/field_pregrasp_cases.yaml`）；
  `gz_ros2_control` 接管属后续轮，须授权改只读 bringup 面（`aubo_e5.ros2_control.xacro`）。
- 袋体碰撞是圆柱包络；插入/剪切接触、可分离袋果留待物理仿真轮。

## 许可

BSD-3-Clause（见 [LICENSE](LICENSE)）。履带底盘几何参数改自 gazebosim/gz-sim
官方示例（Apache-2.0），出处见 `urdf/tracked_platform.xacro` 头注释。
