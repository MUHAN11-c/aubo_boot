# latte_backend

咖啡拉花 ROS 2 后端（2026-09-30 自 `~/aubo_boot` latte_backend 移植，Jazzy 适配）。
不进 peach `harvest_system` / lifecycle 名单；运动薄层复用
`ivg_demo_services::RobotController`，IO 走 `aubo_io_controller`。

## 目的

5 步拉花工作流编排（取奶→打奶泡→转腕放置→心形轨迹执行）+ MSLA 参数化的
心形轨迹纯核 + 奶缸嘴标定工具 + 面板 IO 桥。

## 与 aubo_boot 原版的差异

| 项 | aubo_boot | 本仓 |
|---|---|---|
| 运动层 | `demo_driver::RobotController` | `ivg_demo_services::RobotController`（IO→aubo_msgs/SetIO） |
| IO pin 语义 | ivg SetRobotIO digital_output | `/aubo_io_controller/set_io` FUN_SET_ROBOT_BOARD_USER_DO |
| HeartParams | 节点头内 | 拆到 `latte_heart.hpp`（纯核可测） |
| latte_io_node | 面板契约引用但代码缺失 | 本包新建（DO2/DO4 SetBool→SetIO + DI 状态发布） |
| step4 前倾 45° | 已注释 | 保持禁用（roll 绝对起算） |

`lwf_*` 参数名与默认值与 aubo_boot 完全一致（预教关节角、杯口/喷嘴偏移标定值）。

## 公有 API

- 节点 `latte_workflow_node`：服务 `/latte/run_workflow`（ivg_interfaces/RunLatteWorkflow）；
  参数 `lwf_*`（见 `latte_workflow_node.cpp` 声明处注释）
- 节点 `latte_io_node`：服务 `/set_latte_do2` `/set_latte_do4`（std_srvs/SetBool）；
  话题 `/latte_di_status`（std_msgs/String，JSON 的 DI 状态）
- 库 `liblatte_trajectory`：`LatteTrajectoryGenerator`（stageApproach/Mix/Draw/Finish/Home
  + spoutToTcp），`HeartParams`
- 脚本 `spout_calibrator.py`：TF+Marker 奶缸嘴标定（param set 实时微调）

## 例子

```bash
# 须先起 move_group 与控制器栈；mock 联调用 io_simulated:=true
ros2 launch latte_backend latte_workflow.launch.py io_simulated:=true
ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow "{}"
```

## 如何 build / 测

```bash
colcon build --packages-select latte_backend
colcon test --packages-select latte_backend && colcon test-result --verbose
```

## 安全边界

- 未授权不得真机运动/SetIO（MUST）；操作员起 real 演示栈=授权
- `io_simulated=true` 仅供 mock 联调；真机 IO 需 aubo_io_controller Active
- 设计依据（MSLA 参数表）见 `latte_heart.hpp` 头注释；aubo_boot 原始长文
  `LATTE_HEART.md` 未随迁，要点已并入代码注释

## 许可

BSD-3-Clause
