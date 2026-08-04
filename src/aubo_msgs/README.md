# aubo_msgs — AUBO E5 驱动自定义接口包

## 简介

工作区自定义 msg/srv/action 的唯一定义点：机器人 IO 状态、关节级详细状态、
FK/IK/负载服务、IO 设置服务，以及手眼标定的 action/srv。纯接口包（rosidl，
ament_cmake），不含节点。命名沿用 ROS1 旧驱动（package.xml 描述 "Legacy
AUBO interface definitions"，版本 1.3.3），被 aubo_e5_controllers、
aubo_dashboard、aubo_hand_eye_calibration 及 tools/diagnostics 脚本依赖。

## 使用方法

构建（本包是多数包的前置依赖，改接口后需重建下游）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_msgs
source install/setup.bash
```

在其他包中依赖：

- package.xml：`<depend>aubo_msgs</depend>`
- C++：`find_package(aubo_msgs REQUIRED)` + `ament_target_dependencies(<目标>
  aubo_msgs)`，include 用 snake_case 头（如 `aubo_msgs/msg/io_state.hpp`）
- Python：`from aubo_msgs.msg import IOState`——须先 source 工作区
  install/setup.bash，venv 内亦然（参考 diagnostics/live_monitor.py:84）

查看定义：`ros2 interface show aubo_msgs/msg/IOState`（srv/action 同理）。

真机调用 SetIO / GetFK / SetPayload 等服务前，按 `docs/usage.md` 第 7 节
分阶段流程推进；涉及机械臂运动的操作，速度/加速度缩放先压 0.1（AGENTS.md 第 10 节）。

## 执行逻辑

本包自身不运行代码；各接口的生产/消费方（已全工作区 grep 核实）：

| 接口 | 生产方（服务端） | 消费方 |
|---|---|---|
| `IOState` | aubo_io_controller `~/io_states` | 标定 Web 网关、人工 echo |
| `RobotStatus` | aubo_io_controller `~/robot_status` | 标定 Web 网关、usage.md 监视命令 |
| `JointStatus` | aubo_io_controller `~/joint_status` | 标定 Web 网关、diagnostics/live_monitor.py |
| `Digital`/`Analog` | 仅作 IOState 数组元素类型 | 随 IOState |
| `SetIO` | aubo_io_controller `~/set_io` 服务 | usage.md 示例（真机设板载/工具 IO） |
| `GetFK`/`GetIK`/`SetPayload` | aubo_dashboard_node `~/get_fk`/`~/get_ik`/`~/set_payload`（仅 real） | tools/fk_ik_check.py、usage.md 示例 |
| `RunHandEyeCalibration`(action) | hand_eye_calibration_server `~/run` | 标定 Web 网关（web_gateway.py:67） |
| `ActivateHandEyeCalibration` | hand_eye_calibration_server `~/activate` | 标定 Web 网关（web_gateway.py:70） |
| `GoalPoint`/`TraPoint`/`JointPos` | 无（ROS1 旧接口遗产，为兼容保留） | 无 |

运行时语义要点：

- 三个状态话题在 aubo_io_controller 的 `update()` 每周期经 RealtimePublisher
  `try_publish` 发出（controller_manager 200 Hz；内容刷新受 SDK 推送速率
  限制）。`IOState.stamp` 是纳秒字符串，ROS1 遗留语义，不动消息定义
  （aubo_io_controller.cpp:288）。
- SetIO 走 UR asyncThread 握手：先把 async-success 清成 NaN 等硬件 IO 异步
  线程回写 1.0/-1.0。`FUN_SET_TOOL_POWER_TYPE`(5) 在 aubo_io 契约中无
  对应项，服务端有意不支持（aubo_io_controller.cpp:691）。
- sim 插件不模拟板载 IO 写回，`set_io` 返回 success=false 属预期。
  GoalPoint/TraPoint/JointPos 只被 vendor 的 ROS1 头 `aubo_driver.h` 引用，
  该头不参与编译（dashboard 仅编译 src/aubo_dashboard_node.cpp）。

## 软件框架

依赖仅 std_msgs、geometry_msgs；`member_of_group rosidl_interface_packages`；经 ament_lint_auto 接入 lint 测试。文件清单与字段要点：

- msg/
  - `Analog.msg`：pin + float32 state；`Digital.msg`：pin + flag + state
    （DO 回显尚未命令过时 flag=false）
  - `IOState.msg`：string stamp（纳秒字符串）+ 8 组数组：16 DI、16 DO
    （命令回显）、4 AI、4 AO、2 工具 DI、2 工具 AI、2 安全输入
    （pin0=急停、pin1=防护停）；安全输出恒空
  - `JointStatus.msg`：6 关节数组 current（SDK 原始单位，未换算安培）/
    temperature[°C]/tag_pos/tag_vel（rad、rad/s）/following_error/error_code
  - `RobotStatus.msg`：mode/e_stopped/drives_powered/motion_possible/
    in_motion/in_error/error_code（mode：-1 未上电、1 已上电但急停、2 自动）
  - `GoalPoint.msg`/`JointPos.msg`/`TraPoint.msg`：遗产定义，无消费者
- srv/
  - `SetIO.srv`：fun（1=板载DO 2=板载AO 3=工具DO 4=工具AO，5=工具电源·
    不支持）+ pin + float32 state → bool success
  - `GetFK.srv`：joint（6，rad）→ pos[xyz] + ori（四元数 w,x,y,z）；
    `GetIK.srv`：ref_joint（求解种子）+ pos + ori → joint
  - `SetPayload.srv`：float32 payload（kg）→ bool success；
    `ActivateHandEyeCalibration.srv`：candidate_id → success/message/active_result_file
- action/`RunHandEyeCalibration.action`：goal=plan_only/return_to_start/method
  （auto|tsai|park|horaud|andreff|daniilidis，空串=服务端 solver_method 参数）；
  result=success/candidate_id/wrist_to_camera_optical（Transform）/采样数/
  三项 RMS/result_file；feedback=stage/detail/pose_index/pose_count/
  accepted_samples/progress
