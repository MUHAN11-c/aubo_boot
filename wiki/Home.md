# 套袋桃采摘工作区 Wiki

> **生成产物（快照）**：本 Wiki 于 2026-10-13 自仓库活文档蒸馏生成，不与源码同轮维护。内容如有出入，以 [AGENTS.md](../AGENTS.md) 与各包 `doc/` 活文档为权威。源文档地图见本页末尾。

## 这是什么

AUBO E5 六轴臂 + RGB-D 相机的**套袋桃采摘机器人**工作区（ROS 2 Jazzy）。产品链一句话：**契约 → 到位（预留）→ 场景里有哪些桃 → 这一颗的局部模型 → 臂怎么动**。

- 现行产品形态：固定座 AUBO E5 + 主机单图案立体相机（`peach_stereo`，默认）；愿景含果园底盘移动
- 采摘应用九包在 `src/peach/`；臂/相机驱动九包在 `src/` 顶层（只读红线）；IVG 演示栈八包在 `src/ivg/`
- 当前验收门是 `PREGRASP_ONLY`：到预抓取停住（不套入、不打刀），方向/定位以真机目视为准
- peach2（v2 重构）已于 2026-10-12 归档 `_archive/parked_2026-10-12/peach2/`，不再构建

## 快速开始

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash

# 启动前清残留（MUST，与结束后复核成对）
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'

# mock 冒烟（默认 hardware_mode:=mock camera_enabled:=false）
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock camera_enabled:=false
```

真机另需 `hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98` + 示教器上电。默认**不上电、不派发运动、不打工具 IO、不自动开批**；监控页 `http://127.0.0.1:8090`。

> ⚠️ 未授权不得真机运动或 SetIO；硬件急停在示教器/柜，不经 ROS。详见 [01-安全红线](01-安全红线.md)。

## Wiki 导航

| 页 | 内容 |
|----|------|
| [01-安全红线](01-安全红线.md) | MUST 红线、安全三层模型、真机操作纪律 |
| [02-系统架构](02-系统架构.md) | 产品定位、五层架构、整栈启动序列、批次时序、透传链 |
| [03-接口契约](03-接口契约.md) | 跨包动作/服务/话题、相机契约、驱动层话题、QoS 与 IDL 流程 |
| [04-包索引](04-包索引.md) | 全工作区 31 包职责表与"改哪边"指南 |
| [05-构建与运行](05-构建与运行.md) | 环境、构建、launch 参数全表、冒烟清单、8090、停栈清进程 |
| [06-测试体系](06-测试体系.md) | 测试分层、launch_testing 域分配、冻结相机抓取、回放塔、真机验收门 |
| [07-末端工具与标定](07-末端工具与标定.md) | 三把剪切手档案、tool_profile 切换、手眼/内参标定 |
| [08-仿真与旁路栈](08-仿真与旁路栈.md) | 仿真组（src/peach_sim/）果园仿真、IVG 估姿/抓取、演示栈 |
| [09-开发规范](09-开发规范.md) | 编写前检索、包布局、代码风格、参数体系、缝位、反模式 |
| [10-决策与演化](10-决策与演化.md) | KEEP/UNWIND、决策表 0001–0039 摘要、归档历史 |

## 权威文档地图（本 Wiki 的源）

| 文档 | 权威范围 |
|------|----------|
| [AGENTS.md](../AGENTS.md) | 怎么演化：MUST / DEFAULT / KEEP / SNAPSHOT / UNWIND 全规则 |
| [src/peach/peach_bringup/doc/architecture.md](../src/peach/peach_bringup/doc/architecture.md) | 系统级架构（分层 / 整栈组合 / 决策表 / 驱动地图） |
| [src/peach/peach_bringup/doc/io.md](../src/peach/peach_bringup/doc/io.md) | 系统级 IO（跨包边界 / 相机契约 / 驱动层 / 外设与旁路） |
| 各能力包 `doc/architecture.md` / `doc/io.md` | 包级架构与 IO 明细 |
| [src/peach/peach_system_tests/doc/testing.md](../src/peach/peach_system_tests/doc/testing.md) | 测试流程：怎么跑、命名、验收门、真机纪律 |
| [docs/peach_v1_现行终版方案.md](../docs/peach_v1_现行终版方案.md) | v1 现行栈方案书（非活文档） |
| [docs/peach_v2_重构终版方案.md](../docs/peach_v2_重构终版方案.md) | v2 重构方案书（已归档，仅历史） |
| [README.md](../README.md) | 仓库入口导航 |
