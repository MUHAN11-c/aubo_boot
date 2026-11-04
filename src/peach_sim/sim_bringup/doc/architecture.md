# 联合仿真架构

2026-11-02 当前默认场景代码为数据驱动小果园 `data_orchard`，默认视觉模式与 `adaptive_shear_v1`。用户要求先完成代码和离线核对，暂不运行 Gazebo，当前动态闭环未验收。历史会话单独保留；工作区外源码、配置、安装和 Git 元数据均只读。

## 职责与依赖

`sim_description`（统一机体、RGB-D/激光、车载筐）和 `sim_worlds`（世界、目标、地图、站位）向 `sim_bringup` 提供资源。`sim_bringup` 组合标准 Gazebo、ros2_control、Nav2、MoveIt 和 RViz；`sim_harvest` 消费这些标准服务/动作；`sim_perception` 仅适配目录外现行感知/重建能力。IDL 复用 `peach_interfaces` 和标准 ROS 消息，不新增平行接口包。

外部依赖包括 `/opt/ros/jazzy`、已有 `aubo_description` / `aubo_e5_moveit_config`、Peach 接口/公共/视觉能力安装和 `aubo_py3.12` 依赖；它们不由本项目构建或修改。

## 启动与坐标

1. 选择同源场景预设，生成本次地图、站位、桥配置与 URDF。
2. Include 官方 Gazebo launch，启动 RSP、桥和实体创建。
3. 实体创建成功后启动 JSB，再启动 JTC；任一步失败终止组合。
4. 控制器激活后启动 MoveIt 和可选 RViz。MoveIt、RSP、Gazebo 使用同一份 `robot.urdf`。
5. Nav2 按标准 lifecycle manager 激活；任务受理前另检查仿真时钟、关节状态、新鲜度、控制器和动作服务。导航前必须确认臂处于 SRDF 收纳位。

全图 `use_sim_time=true`，单一 `/clock`。`map → odom` 由 AMCL 或 SLAM 提供，`odom → platform_base_link` 来自 Gazebo，固定结构与六轴臂 TF 由 RSP 提供。实体位姿证据走独立话题，不再广播第二棵 TF。RGB-D/重建使用图像时刻的 TF。

底盘命令链为 Nav2 controller/behavior → velocity_smoother → sim_base_guard → collision_monitor → 桥 → Gazebo。任务节点不直写 `cmd_vel`。应用碰撞监测不代表功能安全认证。

## 作业链

`RunHarvest` 任务选择树旁观察站 → Nav2 到站 → 腕相机观察 → BeginScene/稳定视觉 ID 选果 → BuildTargetModel → 标准 `Pickup` → OMPL 接近 → Pilz LIN 套入 → 实际工具/目标残差核对 → 动态果实创建、原生 DetachableJoint 持有、树上实体移除 → PlanningScene 附着 → 直线撤退 → 搬到筐上方 → 释放 → Gazebo 果实位置及稳定性确认 → 收臂 → 再确认入筐 → 结算。

这是刚体采剪抽象：创建动态副本/移除静态目标表示剪断，原生固定连接表示持有；不建模刀具力学、纸袋柔性和摩擦。只有阶段证据全部满足才增加采收成功数。取消向在途 Nav2/MoveGroup/Pickup 依赖传播并有界等待；切断后失败或持果不确定进入恢复要求，不自动继续导航，重启仿真恢复初始场景。

## 场景与视觉边界

`data_orchard` 从已有 v11 数据驱动模型提取两株桃树和35颗独立袋果，视觉、3615段枝干圆柱、袋果包络、地图与树级站位共享清单。尺寸/外观受桃子数据约束，隐藏结构与布局仍为推断。`demo` 保留为快速接线场景，世界、地图和停车位共享布局。`orchard` 保留程序化果园。`field` 的整园 mesh 没有独立可移除果实，当前仅导航/视觉预览。

`ground_truth` 明确从仿真清单给几何，不是视觉精度证据。`vision` 调用当前 `BeginScene → target_observations → BuildTargetModel`，按稳定视觉身份选果，使用曝光时间戳 TF；实体关联只在 Pickup 仿真边界发生，不反向覆盖视觉抓取位姿。模型身份哈希包含 run、epoch、ID、revision、工具、标定/配置修订和有效期，放在标准 Grasp.id，执行侧对现有 GraspDecision 在 approach/insert/cut 前逐次复核。阶段许可未知/无效、过期或身份不一致均拒绝；未通过不得回退成真值成功。大场景和视觉能力分别验收。

## 文件与进程收口

`scripts/sim` 是统一入口：`bwrap` 将项目外文件系统只读挂载，将 Gazebo/ROS/Qt 用户目录映射到 `.runtime/`，所有构建、安装、日志和缓存留本项目。环境变量本身不是写入隔离的保证，实际边界是挂载权限。

每次运行以项目文件锁占用一个 ROS domain，使用对应 Gazebo partition。运行器持有本次子进程组，结束先取消动作，再分级 SIGINT/SIGTERM/SIGKILL 清理本组；不使用全局 `pkill`。不能将项目锁当成对任意外部 ROS 进程的权限隔离。

## 验证边界与参考

`colcon test` 只跑官方 lint。行为由有限时长系统会话输出日志核对，启动数据流、导航位移、动作执行、剪断/持有/入筐和取消分别取证。2026-10-29 的 65 秒启动检查已得到 clock/joints/RGB/odom，但不据此宣称采摘或视觉通过。

实现参考 [Nav2 Jazzy bringup](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup)、[MoveIt 2](https://github.com/moveit/moveit2_tutorials)、[gz_ros2_control demos](https://github.com/ros-controls/gz_ros2_control/tree/jazzy/gz_ros2_control_demos)、[Gazebo DetachableJoint](https://github.com/gazebosim/gz-sim/blob/gz-sim8/tutorials/detachable_joints.md)。实际 API 以已安装 Jazzy/Harmonic 版本为准；本轮执行使用 MoveGroup 与 Pilz，未宣称采用 MTC 执行器。

## 采剪与收集物理边界

剪切手模型没有可驱动的开合关节，持果用 Gazebo 原生 DetachableJoint 表达；果实与固定刀具网格的碰撞按显式位掩码排除，果实仍与筐、底盘、地面碰撞并由重力落筐。采后实测果实注册为 PlanningScene 中筐的负载，避免把 MoveIt 的 detach 瞬间位置误当成落筐位置。底盘使用 VelocityControl 运动学代理，不把平地导航结果当成履带接地动力学验收。

长工具的 OMPL 碰撞离散分辨率与 TOTG 路径容差在本组覆盖；ValidateSolution 始终保留，后处理后出现自碰的轨迹不会执行。


## 数据驱动场景作业边界

观察位按相机位置与树冠方向配置，使用当前工具的固定 TCP→光学帧 TF 推导 TCP 目标；到位后核验实际相机位置和光轴。每树场景键保持稳定，已采目标按稳定视觉 ID 排除，不用10cm空间球吞掉相邻袋果。相邻未采果以同源碰撞盒进入 MoveIt；仅当前目标与指定工具接触链放行，保留原 ACM 并在事务结束恢复。

视觉任务不支持用实体 ID 指定 PICK_SELECTED，明确拒绝该请求。每站可重复观察/采收，额度按果数计算；空站经现有 MoveTo 契约收臂后才允许前往下一站。车载筐当前至多两颗，结果区分 target_limit_reached、basket_capacity_reached、target_ratio_reached、scene_exhausted 与 no_targets_succeeded；不把容量限制报成整园已采完。第二颗入筐路径、空站换站、Gazebo 渲染及模型阶段许可是否可满足仍需运行验收，不以静态接线宣称成功。
