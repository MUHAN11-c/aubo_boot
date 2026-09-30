# tool_changer

工具快换（2026-09-30 自 `~/aubo_boot` tool_changer 移植，Jazzy 适配）。
不进 peach `harvest_system` / lifecycle；运动薄层复用 `ivg_demo_services`，
IO 走 `/aubo_io_controller/set_io`（板载用户 DO）。

## 目的

数据驱动的末端工具快换：`tools.yaml` 定义每个工具的 dock 位、取/放轨迹
（vertical / slide 两种策略）与碰撞几何；`gripper_swap_worker` 执行
「释放当前 → 取目标 → 回 home」综合流程并维护 `/tool_changer_status`；
`scene_attach_worker` 同步 MoveIt 规划场景（AttachedCollisionObject）与
前端 URDF 显示。

## 与 aubo_boot 原版的差异

| 项 | aubo_boot | 本仓 |
|---|---|---|
| 运动层 | `demo_driver::RobotController` | `ivg_demo_services::RobotController` |
| IO | ivg SetRobotIO `/set_robot_io` | `aubo_msgs/SetIO` `/aubo_io_controller/set_io` |
| 附着帧 | `kuaihuan_Link` | 参数 `attach_frame`，默认 `quick_changer_link`（本仓 xacro 帧名） |
| URDF 换装 | popen xacro（旧 xacro 有 `gripper:` 参数） | 读本包 `urdf/` vendored 静态 URDF |
| `/debug/move_to_xyz` | 本包提供 | 删除（由 ivg_demo_services 提供，避免重复服务） |
| ToolConfig 解析 | 节点内 | 抽到 `tool_config.cpp` 纯核（gtest 直测） |

工具 mesh（gripper0/1/2、咖啡杯/牛奶杯、快换盘、场景件共 11 件，visual/collision
各一份）在**本包 `meshes/{visual,collision}/`**——IVG/peach 隔离裁定（2026-09-30）：
不落共享 `aubo_description`；tools.yaml 与 vendored URDF 均引用
`package://tool_changer/meshes/...`（URDF 中臂身 link0-6 引用 aubo_description 属
共享机器人本体，保留）。

## 公有 API

- 节点 `gripper_swap_worker`：服务 `/run_gripper_swap`（RunGripperSwap）、
  `/change_tool`（ChangeTool）、`/get_current_tool`（GetCurrentTool）；
  话题 `/tool_changer_status`（ToolChangerStatus，5s 周期）
- 节点 `scene_attach_worker`：服务 `/scene_attach` `/scene_detach`
  `/set_display_tool`（均 ChangeTool）；话题 `/attached_collision_object`
  `/planning_scene`（diff）/ `/robot_description`（transient_local，URDF 换装）
- 配置 `config/tools.yaml`（工具档案：dock 位、轨迹策略、mesh、attach_offset、touch_links）
- 脚本：`test_tool_change.py`（服务联调）、`attach_test.py`（PlanningScene 直发）、
  `compute_dock_ik.py`（经 `/aubo/get_ik` 反解 dock 关节角）、`scene_monitor.py`

## 例子

```bash
# 须先起 move_group 与控制器栈；mock 联调用 io_simulated:=true
ros2 launch tool_changer gripper_swap_worker.launch.py io_simulated:=true
ros2 run tool_changer test_tool_change.py gripper2
ros2 service call /set_display_tool ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"
```

## 如何 build / 测

```bash
colcon build --packages-select tool_changer
colcon test --packages-select tool_changer && colcon test-result --verbose
```

## 安全边界

- 未授权不得真机运动/SetIO（MUST）；操作员起 real 演示栈=授权
- `io_simulated`/`simulation_skip_io` 仅供 mock 联调
- URDF 换装会动态改变 RSP 的 TF 树（demo 机制）——勿与 peach harvest_system
  同时运行（peach 预检会拒多 child frame）

## 许可

BSD-3-Clause
