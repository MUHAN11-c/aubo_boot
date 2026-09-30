# ivg_interfaces

旁路视觉抓取（IVG）栈的唯一 IDL 包：模板估姿服务族的 `.srv` 与笛卡尔位姿
`.msg` 定义，加上 2026-09-30 自 aubo_boot 移植的**演示栈 IDL**（拉花/快换/
抓取/监控）。**机械臂 FK/IK/IO/急停走 `aubo_msgs` 不在此重复，不进 peach
接口清单。**

## 谁发谁订

| 接口 | 服务端（实现方） | 客户端 |
|------|------------------|--------|
| `srv/EstimatePose` | `ivg_pose_estimation` 节点（服务名 `estimate_pose`，根命名空间） | `ivg_pose_estimation` Web 桥（`algorithm_http_server_node`）、上位机 |
| `srv/EstimatePose2D` | 同上（`estimate_pose_2d`） | 同上 |
| `srv/ListTemplates` / `srv/StandardizeTemplate` / `srv/UpdateParams` | 同上 | Web 桥（模板管理/调参面） |
| `srv/RunLatteWorkflow` | `latte_backend`（`/latte/run_workflow`） | Web 咖啡拉花面板 / CLI |
| `srv/RunGripperSwap` / `srv/ChangeTool` / `srv/GetCurrentTool` | `tool_changer`（`/run_gripper_swap` `/change_tool` `/get_current_tool`） | Web 视觉抓取面板 / 调度脚本 |
| `srv/ExecuteGraspPose` | `ivg_demo_services`（`/execute_single_grasp`） | Web 视觉抓取面板 / 抓取循环器 |
| `msg/ToolChangerStatus` | `tool_changer` 发布（`/tool_changer_status`） | `tool_changer/scene_attach_worker`、Web 面板 |
| `msg/NodeStatus` / `msg/SystemLog` | `ivg_demo_services/system_monitor_node` 发布（`/system/node_status` `/system/log`） | Web 监控/日志面板 |

`msg/CartesianPosition`：base 系笛卡尔位姿容器（位置+四元数+RPY+关节值，
弧度制），仅被 `EstimatePose` 响应嵌用。

## 时间与单位语义（EstimatePose）

- 请求 `header` 可为空，以图像自带 stamp 为准；响应 `header.stamp` = 实际
  采用的图像采集时刻、`frame_id` = 输出参考系（`base_link`）。服务端 TF
  查询使用同一 stamp（失败回退 latest 并打 WARN）。
- 长度单位：响应全部 `CartesianPosition` 为**米**（REP-103）；像素中心在
  `position`（z 恒 0.0 占位）。深度原始值 → 米的换算（Percipio
  `depth_scale=0.00025`）只在服务端边界发生一次。

## QoS

ROS 2 服务默认 RELIABLE（本包全部为短 RPC 服务，无流式接口）。

## 改字段流程

1. 改 `.srv`/`.msg` 与字段注释（单位、frame、stamp 语义）
2. `colcon build --packages-select ivg_interfaces`（先编接口包）
3. 同轮改服务端 `ivg_pose_estimation/ros2_communication.py` 与客户端
   `web/ros_bridge/node_runtime.py`（两者都在本仓，无外部消费者）
4. 本 README 与 `docs/io.md` §8 同步

## 不做什么

- 不跑节点、不声明运行参数
- 不夹带 peach 契约（peach 走 `peach_interfaces`）
