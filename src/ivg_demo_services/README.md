# ivg_demo_services

IVG 演示栈服务包（2026-09-30 自 `~/aubo_boot` demo_driver **按需补缺**移植）。
不进 peach `harvest_system` / lifecycle 名单；FK/IK/急停/上下电走 `aubo_dashboard`
（`/aubo/*`），IO 走 `aubo_io_controller`（`/aubo_io_controller/set_io`）。

## 目的

补齐旧 IVG 演示（Web 面板 / 拉花 / 换刀 / 视觉抓取循环）所需、而现栈缺失的服务，
不整包复刻 demo_driver 的 12 个 server（与现栈重叠的部分用映射表收敛）。

## 旧 demo_driver 服务映射表

| 旧服务（aubo_boot） | 现行归属 |
|---|---|
| set_robot_io | `aubo_io_controller` `/aubo_io_controller/set_io`（aubo_msgs/SetIO） |
| get_fk / get_ik / set_payload / startup / shutdown / 急停 | `aubo_dashboard` `/aubo/get_fk` `/aubo/get_ik` `/aubo/set_payload` `/aubo/startup` `/aubo/stop` `/aubo/fast_stop` |
| plan / execute_trajectory | MoveIt move_group + JTC（不移植） |
| movel_server | 无 .cpp 实现（aubo_boot 已弃），由 `/move_to_pose` use_joints=false 覆盖 |
| set_robot_pose | `/move_to_pose`（MoveToPose）覆盖 |
| get_current_state | 本包 `/get_current_state` |
| read_robot_io | 本包 `/read_robot_io`（缓存 `/aubo_io_controller/io_states`） |
| set_robot_enable | 本包 `/set_robot_enable`（转发 `/aubo/startup` / `/aubo/stop`；语义=上电/停运，**不是急停**） |
| set_speed_factor | 本包 `/set_speed_factor`（本节点后续运动的 MoveIt 缩放） |
| system_monitor_node | 本包 system_monitor_node（`/system/node_status` `/system/log`） |
| execute_grasp_pose_worker | 本包 grasp_trigger_node（视觉源改为 ivg_graspnet `/grasp_poses_base`） |
| publish_grasps_client_worker 的循环 | `/publish_grasps_worker_loop_control`（与 `/loop_grasp_control` 同处理器） |

## 公有 API

- 节点 `system_monitor_node`：订 `/aubo_io_controller/robot_status`（aubo_msgs/RobotStatus）；
  发 `/system/node_status`（ivg_interfaces/NodeStatus，transient_local）、`/system/log`（SystemLog）
- 节点 `move_service_node`：服务 `/move_to_pose` `/debug/move_to_xyz`（MoveToPose）、
  `/get_current_state`（GetCurrentState）、`/read_robot_io`（ReadRobotIO）、
  `/set_robot_enable`（SetRobotEnable）、`/set_speed_factor`（SetSpeedFactor）
- 节点 `grasp_trigger_node`：服务 `/execute_single_grasp`（ExecuteGraspPose）、
  `/loop_grasp_control`、`/publish_grasps_worker_loop_control`（std_srvs/SetBool）；
  订 `/grasp_poses_base`（PoseArray）
- 库 `libdemo_motion`：`ivg_demo_services::RobotController`（MoveIt 运动薄层 + IO），
  `motion_utils.hpp`（slerp / 笛卡尔插值 / 接近路点构造，纯函数可单测）
- 脚本 `scripts/`：`aubo_mode.py`（/aubo/mode 模式广播）、`limit_workspace.py` +
  `workspace_limits.yaml`（工作空间边界墙）、`publish_gripper_joint_states.py`、
  `publish_obstacle.py`（场景调试）。按 IVG/peach 隔离裁定（2026-09-30）自
  aubo_e5_moveit_config 迁入。
- 参数：`config/demo_services.yaml`（`egp_*` 参数名与 aubo_boot 一致）

## 例子

```bash
# 须先起 move_group 与控制器栈（如 aubo_e5_bringup + moveit）；launch 会随发
# MoveIt 参数（robot_description/SRDF/kinematics，MGI 构造必需）
ros2 launch ivg_demo_services demo_services.launch.py io_simulated:=true
ros2 service call /execute_single_grasp ivg_interfaces/srv/ExecuteGraspPose \
  "{object_id: demo, use_visual_estimation: true}"
```

## 如何 build / 测

```bash
colcon build --packages-select ivg_demo_services
colcon test --packages-select ivg_demo_services && colcon test-result --verbose
```

## 安全边界

- 未授权不得真机运动/SetIO（MUST）；操作员起 real 演示栈=授权，语义与原系统一致
- `/set_robot_enable` 不是急停；ISO 13850 急停在柜/示教器，不经 ROS
- `io_simulated=true` 仅供 mock 联调（IO 旁路直接成功）

## 许可

Apache License 2.0
