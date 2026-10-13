# sim_description

仿真子工作空间（`src/sim/`）的机体包：采摘工位 URDF/xacro + 网格资产。

- `urdf/harvester_robot.urdf.xacro` — 整机装配（履带底盘 + AUBO E5 + 腕载相机座 + 三选一 peach 末端）；
  `mobile` 参数切换锚定建模（含臂链，`world` 根）与导航模式（`platform_base_link` 根，gz DiffDrive + 2D 激光）。统一机体（可驾驶带臂 + gz_ros2_control）在 P2 落地。
- `urdf/tracked_platform.xacro` — 履带平台宏（BUNKER Pro 2 彩色外观、履带摩擦、桅杆激光）。
- `meshes/` — 树/袋/底盘 GLB（Blender 程序化与重建管线产物）与 `field_no_grass.glb` 整园查看网格。

臂链与关节限位以 `aubo_description` 为准（mesh `package://aubo_description/...` 只读引用，驱动九包不动）。
构建：`colcon build --packages-select sim_description`；测试：`colcon test --packages-select sim_description`（xacro 两模式结构断言）。
许可 BSD-3-Clause。
