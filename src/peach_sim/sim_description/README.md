# sim_description

独立联合仿真的机器人描述和资产包。2026-10-29 用户明确恢复 Gazebo；当前统一入口见 [项目 README](../README.md)。目录外 `aubo_description` 的网格、工具组件和已有配置只读引用，驱动栈不改。

- `urdf/harvester_unified.urdf.xacro`：`platform_base_link` 为根，组合履带平台、AUBO E5、现有剪切工具、腕 RGB-D、相机光学帧和车载筐。臂使用 `gz_ros2_control` 的 JSB/JTC，六关节状态原生发布，不经 Gazebo joint-state 桥。
- `urdf/tracked_platform.xacro`：平台几何、激光、Gazebo 底盘运动与里程计插件。底盘接收 Nav2 命令链末端的速度。
- `urdf/aubo_e5_arm.xacro`：只去除世界固定连接的臂链快照，网格仍引用 `aubo_description`；关节顺序、限位和惯量须与只读来源对照。
- `config/gz_controllers.yaml`：仿真控制器配置，不修改外部真机/控制器配置。
- `meshes/`：Gazebo 安装资源，包括 BUNKER 外观和既有树/袋/field 资产。来源、许可和重导出说明保留在对应资产文档中。
- `urdf/harvester_robot.urdf.xacro`：旧两模式描述作为历史源码保留；当前 launch 不再切换两套独立机体。

几何来自 `sim_worlds/config/orchard.yaml` 平台段及工具档案。`sim.launch.py` 展开一份 `robot.urdf`，再注入本次目标的 Gazebo 原生 DetachableJoint 与收纳初值；RSP、Gazebo 和 MoveIt 读取同一模型，避免重复展开漂移。固定相机 TF 只由机器人描述生成，不再另起 extrinsics_publisher。

工具参数 `tool_profile` 可选 bite_shear_v1、shear_v1、adaptive_shear_v1；默认 bite_shear_v1。腕相机采用 PS800-E1 数据表的 0.2–1.0 m 工作区间、63°水平视场与 2.5 Hz 上限，输出降采样为 640×480；保留相机与腕部 CAD 可见。它仍是理想 RGB-D 代理，未模拟结构光与镜头材料。车载筐用于真实 Gazebo 果实位置与稳定性验证，不以附着状态代替入筐证据。

在项目根运行 `scripts/sim build`、`scripts/sim test`。本包测试仅官方 ament lint，没有自定义行为单测。离线运行 `scripts/sim check python3 scripts/check_models.py` 核对三工具的源模型、树结构、惯性、资源、收臂包络、传感器转换和袋几何；详见 [模型设计与核对边界](doc/architecture.md)。运行行为仍需单独取证。所有构建和日志留在本项目。

许可 BSD-3-Clause；第三方资产沿用各自来源声明。
