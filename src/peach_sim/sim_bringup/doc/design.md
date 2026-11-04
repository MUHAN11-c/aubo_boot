# 自主导航采摘联合仿真设计（2026-10-29）

## 目标与边界

用户本轮明确恢复 Gazebo，要求在 `src/peach_sim` 内构建完整的 ROS 2 Jazzy / Nav2 / MoveIt 2 / RViz2 / Gazebo Harmonic 联合仿真。目录外源码、安装、配置、Git 元数据只读，不安装或修改系统依赖。不连接真机、不调用 dashboard / SetIO。构建、缓存、运行时产物、验证日志全部写项目目录。沿用已有用户改动。

## 方案

- `sim_description`：统一平台 + AUBO E5 + 现有剪切工具 + 腕 RGB-D + 激光；真实控制器和规划配置只读引用，仿真覆盖配置在本组。
- `sim_worlds`：现有 Blender 大场景保留；新增轻量确定性 demo，同源生成世界、目标 manifest、地图、站位，适于闭环验收。世界名固定 `peach_orchard`。
- `sim_perception`：删除业务快照的运行依赖，入口作为现行 `peach_scene_perception` / `peach_target_reconstruction` 的薄适配，配置和模型走现行包。默认 demo 使用明确标识的仿真真值目标，视觉模式单独验收，不把真值当视觉精度证据。
- `sim_harvest`：新增任务/操作包，复用现有 `peach_interfaces/RunHarvest`（`/sim_harvest/run_harvest`）和标准 `moveit_msgs/Pickup`（`/sim_harvest/pickup`）；不新增 IDL。任务负责 Nav2 到站与统计，Pickup 服务端串行承担所有臂动作；CLI 仅派发 action。
- `sim_bringup`：组合入口、依赖就绪、ROS/Gazebo 隔离、失败收口、统一时钟、RViz。保留旧入口兼容，但默认入口统一为 harvest。

## 实际采摘的仿真语义

接近 → Pilz LIN 套入 → 根据 TCP 到目标残差确认接触区域 → 创建动态果实副本并删除树上静态果实（模拟剪断）→ Gazebo 原生 DetachableJoint 固定持有 → MoveIt AttachedCollisionObject 同步 → 原路撤退 → 移到车载筐上方 → 解连接 → 重力落筐 → 收臂。只有接触、Gazebo 连接确认、移动执行、释放确认均成功才计采收。模拟的固定连接不代表摩擦抓取/刀具接触力学或真机安全验收。未确认的切断/放置不得上报成功。

## 接口与配置

- demo 的 `prepare_demo(output_directory)` 返回 `world`、`manifest`、`map`、`stations`（绝对路径）。`stations.yaml` 含 spawn 和每站 nav_pose、target_id；manifest targets 含 id、bag_bottom、axis、fruit_center、body_diameter、sdf_pose。
- `add_harvest_plugins(robot_xml, manifest)` 给 robot 注入每目标原生 detachable joint，父链 `wrist3_Link`，子模型 `<id>_picked`、link `bag`，桥接每目标 attach/detach/state。
- Gazebo create/remove 服务通过 ros_gz_bridge 映射为 `/world/peach_orchard/create`、`/world/peach_orchard/remove`。
- mission/grasp 节点参数：manifest_file、stations_file、run_directory、use_sim_time=true；motion API 使用当前模型解析 SRDF，不维护关节位姿第二份快照。
- `harvest.launch.py`：scene=demo|orchard|field，gui、rviz、perception=ground_truth|vision、autostart=false、max_targets；旧 field 参数保留兼容。demo 默认。
- 自动启动仅仿真且经过隔离与就绪检查。检查 /clock、控制器 active、TF、MoveGroup/Nav2 server 与 Gazebo 世界服务，不凭固定延迟宣称就绪。
- Nav2 的 velocity_smoother → sim_base_guard → collision_monitor → cmd_vel 是底盘唯一命令链；任务不直写 cmd_vel。

## 运行约束

项目运行器以 bwrap 只读根文件系统、仅项目可写，并重定向用户缓存挂载；为每次运行分配独立 ROS_DOMAIN_ID / GZ_PARTITION，设置 ROS_HOME、ROS_LOG_DIR、GZ_LOG_PATH、GZ_FUEL_CACHE_PATH、XDG 缓存、TMPDIR 到项目内，使用 UDP 避免 FastDDS SHM 残留。仅终止本次进程组，不使用全局 pkill；正常退出、超时、异常和取消均走同一有界取消路径。

## 验收

本仓 colcon test 仅官方 ament lint，不新增自定义 pytest/gtest/launch_testing。通过项目内有限时长的系统检查/运行日志核验：包安装与 launch 展开；控制器/TF/clock；Nav2 非零实际位移；MoveIt 轨迹执行；Gazebo 果实连接/释放；action 成功/取消；RViz2 与 Gazebo 同图；收尾无本轮进程。视觉和大型 field 场景分开报告，不用 demo 成功替代。

## 参考

按 Jazzy 已安装 API 与官方源码核对，借鉴官方完整仿真项目的职责与启动方式，不复制外部项目源码：

- [Nav2 bringup](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup)：AMCL、BT Navigator、controller、velocity_smoother、collision_monitor、lifecycle_manager。
- [Nav2 Simple Commander](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_simple_commander)：可取消任务与反馈。
- [MoveIt 2 tutorials](https://github.com/moveit/moveit2_tutorials)：PlanningScene、碰撞物、抓放阶段与机器人状态。
- [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)：接近/附着/撤退/放置阶段边界；本轮实际执行用 MoveGroup + Pilz，不冒称使用 MTC。
- [gz_ros2_control demos](https://github.com/ros-controls/gz_ros2_control/tree/jazzy/gz_ros2_control_demos)：实体生成完成后启动 spawner。
- [Gazebo detachable joints](https://github.com/gazebosim/gz-sim/blob/gz-sim8/tutorials/detachable_joints.md)：原生固定连接与确认状态；源码参数名以 `child_link` 为准。
