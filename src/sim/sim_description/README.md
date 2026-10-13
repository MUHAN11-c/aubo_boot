# sim_description

仿真子工作空间（`src/sim/`）的机体包：采摘工位 URDF/xacro + 网格资产。

- `urdf/harvester_robot.urdf.xacro` — 旧两模式装配（锚定含臂链 `world` 根 / 导航
  `platform_base_link` 根 + DiffDrive + 2D 激光），行为保留。
- `urdf/harvester_unified.urdf.xacro` — **统一机体（P2 已落地）**：`platform_base_link`
  根 + 完整臂链（`aubo_e5_arm.xacro` 快照，绕开 world_joint）+ gz_ros2_control
  （JSB/JTC，`config/gz_controllers.yaml`）+ 腕载 rgbd sensor（P3 桥接进 `/camera/*`）
  + peach 感知契约相机帧链（wrist3→camera_link = 手眼标定快照）。入口
  `sim_bringup/sim.launch.py`，作业位 spawn。
- `urdf/tracked_platform.xacro` — 履带平台宏（BUNKER 彩色外观、履带摩擦、桅杆激光、
  chassis_visual 参数分离外观与可驾驶性）。
- `meshes/` — 树/袋/底盘 GLB（Blender 程序化与重建管线产物）与 `field_no_grass.glb` 整园查看网格。

臂链与关节限位以 `aubo_description` 为准（mesh `package://aubo_description/...` 只读引用，驱动九包不动）。
构建：`colcon build --packages-select sim_description`；测试：`colcon test --packages-select sim_description`（xacro 两模式结构断言）。
许可 BSD-3-Clause。
