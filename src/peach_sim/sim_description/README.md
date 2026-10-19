# sim_description

仿真子工作空间（`src/peach_sim/`）的机体包：采摘工位 URDF/xacro + 网格资产。

- `urdf/harvester_robot.urdf.xacro` — 旧两模式装配（锚定含臂链 `world` 根 / 导航
  `platform_base_link` 根 + VelocityControl 履带 + 2D 激光），行为保留。
- `urdf/harvester_unified.urdf.xacro` — **统一机体（P2 已落地）**：`platform_base_link`
  根 + 完整臂链（`aubo_e5_arm.xacro` 快照，绕开 world_joint）+ gz_ros2_control
  （JSB/JTC，`config/gz_controllers.yaml`）+ 腕载 rgbd sensor（P3 桥接进 `/camera/*`）
  + peach 感知契约相机帧链（wrist3→camera_link = 手眼标定快照）。法兰盘和相机外壳对 gz 相机透明（`visibility_flags=0`）。腕相机近裁剪是 2 mm：5 cm 会从贴着光心的外壳中间切开，左上角留下一块灰盘。RViz 里的机器人模型仍画这两块网格。入口
  `sim_bringup/sim.launch.py`，作业位 spawn。
- **参数面（xacro:arg）**：`tool_profile`（bite_shear_v1/shear_v1/adaptive_shear_v1）、
  `arm_mount_height`（默认 0.54，2026-10-13 起臂直接落底盘）、`arm_mount_yaw_deg`
  （-32，转台补偿）、`body_x/y/z`、`body_bottom`、`track_length/width/height/
  separation`、`wrist_camera`。几何唯一事实源 = `sim_worlds/config/orchard.yaml`
  平台段：`sim.launch.py`（robot_state_publisher）与 `sim_bringup/moveit_sim.launch.py`
  （move_group 的 robot_description）从同一份场景参数以**同一组映射**展开本
  xacro——逐参同值，防 move_group 模型与 TF 分叉。
- `urdf/tracked_platform.xacro` — 履带平台宏（BUNKER 彩色外观、履带摩擦、桅杆激光、
  chassis_visual 参数分离外观与可驾驶性）。
- `meshes/` — **gz 消费安装面**（世界经 `model://sim_description/meshes/...`
  引用，`CMakeLists` 整目录安装）：树/袋 GLB（`sim_worlds/blender/build_assets.py`
  `--out` 产物）、BUNKER `bunker_chassis.glb`（`color_bunker.py` 产物）、
  `field_no_grass.glb`（重建整园，`export_field_to_gz.py` 产物经此承载）；导出
  命令与产物侧见 `sim_worlds/blender/`。

臂链与关节限位以 `aubo_description` 为准（mesh `package://aubo_description/...` 只读引用，驱动九包不动）。
构建：`colcon build --packages-select sim_description`；测试：`colcon test --packages-select sim_description`（xacro 两模式结构断言）。
许可 BSD-3-Clause。
