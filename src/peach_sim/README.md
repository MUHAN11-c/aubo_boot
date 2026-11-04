# Peach 自主导航采摘联合仿真

ROS 2 Jazzy、Nav2、MoveIt 2、RViz2 与 Gazebo Harmonic 的独立项目。2026-11-02 新增数据驱动小果园，修改和产物均限定在 `peach_sim/` 内，目录外模型、接口、感知能力和 Python 环境只读引用，不连接真机、不调用 SetIO。

**当前验证边界：用户明确要求先完成代码和离线核对，暂不运行 Gazebo。** 本轮不宣称导航、视觉采收或入筐通过。以下运行用法保留供用户明确恢复后的验收；2026-10-29 的恢复记录和旧会话不能替代本轮验收。

## 数据驱动小果园

`sim_worlds/worlds/data_orchard/` 保存从现有 v11 Blender 重建提取的两株桃树、35 颗独立袋果与 3615 段实际枝干网格圆柱碰撞。袋果保留原始尺寸、倾角和遮挡，不再统一缩成工具能容纳的大小。叶片导出时减面，枝干和袋果不减面；同一清单生成 SDF、MoveIt 障碍物、占据地图和树旁观察站位。

视觉模式先按观测选果并重建，仿真执行边界才关联可删除实体，不用实体真值生成视觉候选或抓取位姿。模型身份、有效期与阶段能力在接近、套入、采剪前复核。当前车载筐仅支持至多两颗刚体代理，满筐以 `basket_capacity_reached` 结算；卸筐与大规模连续采收尚未实现。

数据来源、尺寸估计/推断布局的区别与重建方法见 [资产说明](sim_worlds/worlds/data_orchard/SOURCE.md)。包级结构见 [场景架构](sim_worlds/doc/architecture.md)。本轮构建、官方 lint 与离线核对记录位于 `.runtime/data_orchard_work/`；离线渲染检测不代表 Gazebo 相机闭环通过。

## 使用

模型检查与整理口径见 [sim_description 模型文档](sim_description/doc/architecture.md)。离线复核运行 `scripts/sim check python3 scripts/check_models.py`；2026-10-29 已核对三工具装配、收臂自碰撞与袋几何。新的模型修改仍需后续运行验收。

在本目录执行：

```bash
scripts/sim build
scripts/sim test
# 同时打开 Gazebo 与 RViz2；默认只启动系统，不自动采摘，限时结束并清本轮进程。
scripts/sim run --gui --rviz --duration 480
# 显式发起一颗目标的自主导航、采摘、落筐任务。
scripts/sim run --gui --rviz --mission --max-targets 1 --duration 480
# 无窗口的有限时长启动检查。
scripts/sim run --duration 65
# 取消链验收：任务受理后30秒请求取消。
scripts/sim run --mission --cancel-after 30 --duration 240
```

`--duration` 是整次会话的墙钟上限；任务完成、异常、Ctrl+C 或到期均结束本次进程组。`--mission` 由运行器显式派发 `/sim_harvest/run_harvest`，不是把业务目标藏进 launch。需要桌面显示时使用 `--gui --rviz`，无窗口验收省略这两个选项。

## 场景与感知

- 默认 `--scene data_orchard --perception vision --tool-profile adaptive_shear_v1`：上述数据驱动小果园。观察站位按树生成，每站按稳定视觉 ID 选果；当前视觉任务只接受 PICK_ALL，不接受实体 ID 选果。
- `--scene demo --perception ground_truth --tool-profile bite_shear_v1`：轻量确定性果园，同源生成世界、地图、停车位和可独立摘取的目标。真值模式验证导航与操作闭环，不作为视觉检测、重建或真实感精度证据。
- `--perception vision`：复用当前 Peach 的 `BeginScene → target_observations → BuildTargetModel` 链；视觉几何须通过精确时间 TF 和实体关联门，再交给 Pickup。视觉模式须单独验收，未检出或建模失败不会自动换成真值成功。
- `--scene orchard`：保留程序化果园入口，使用同源目标清单。
- `--scene field`：保留大规模重建果园的导航/视觉预览。整园 GLB 没有独立果实实体，不能执行逐果采收；该模式不要加 `--mission`。

## 包与运行边界

`sim_description` 提供统一机体、传感器和车载筐；`sim_worlds` 提供场景与离线资产；`sim_perception` 是现行感知能力的薄适配；`sim_harvest` 实现任务与 Pickup 动作；`sim_bringup` 组合系统并配置 RViz；`sim_system_tests` 保留官方 lint。

统一入口是 `scripts/sim`。它要求已有 `bwrap`、Jazzy/Gazebo/Nav2/MoveIt 及列明的工作区依赖；不安装缺失依赖。它将根文件系统只读挂载，仅本项目可写，并将 Gazebo、ROS、Qt 的用户缓存映射到项目内。`build/`、`install/`、`log/`、`.runtime/` 均在本目录，不修改工作区根安装或 Git 元数据。不要绕过运行器在工作区根执行构建/启动来复现本项目。

每次运行选择独立的 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`，以项目内文件锁防止本项目并发占用同一 domain；可用 `PEACH_SIM_DOMAIN_ID` 指定。这个锁不代替对外部 ROS 环境的隔离。FastDDS 使用 UDP，运行器只终止自己创建的进程组，不全局清理其他程序。

## 成功与证据

采摘成功要求：实际 Gazebo 机器人姿态与工具 TF 确认接触 → 原生固定连接持果 → 撤退与搬运执行 → 释放确认 → 果实实际位姿持续稳定在车载筐内 → 收臂后再次确认。创建动态果实并移除树上静态果实表示“剪断”；固定连接表示“持有”。这不仿真刀刃接触力、纸袋变形或摩擦抓取。

每次运行的 `.runtime/session_*/` 保存 `launch.log`、`summary.json`、`sim_mission.jsonl`、`sim_grasp.jsonl`，并包含生成场景、桥和 URDF。若切断后失败或持果状态不确定，停止本批任务并重启仿真恢复场景，不自动导航到下一颗。

2026-10-29 历史记录完成一次 65 秒无窗口启动检查：`/clock` 28,504 条、`/joint_states` 9,731 条、RGB 214 帧、里程计 2,919 条，运行器返回 0；见 `.runtime/session_20261029_164950/summary.json`。这是启动与数据流证据，未证明导航、采摘、落筐或视觉模式通过。后续行为结论以对应会话的动作结果和事件证据为准。

`colcon test` 仅运行官方 ament lint；没有自定义 xacro 结构断言或行为测试，不以 lint 通过替代联合仿真验收。架构、接口和阶段定义见 [sim_bringup/doc/architecture.md](sim_bringup/doc/architecture.md)、[sim_bringup/doc/io.md](sim_bringup/doc/io.md)。
