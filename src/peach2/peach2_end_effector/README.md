# peach2_end_effector

Peach v2 套袋剪刀末端：工具档案读取、带独立反馈的刀具状态机、IO 后端、三个 pluginlib 插件。
不跑节点、不订阅话题；唯一使用者是 `peach2_manipulation`（经其命令门做全部 SetIO）。

## 公有 API（安装头 `include/peach2_end_effector/`）

| 头文件 | 内容 |
|--------|------|
| `end_effector.hpp` | pluginlib 基类 `EndEffector`；`EndEffectorContext`（档案、IoBackend、`IoPins`、`ToolTiming`、`CurrentSignatureConfig`、时钟、sleep、log） |
| `plugins.hpp` | `ShearV1` / `BiteShearV1` / `AdaptiveShearV1`（`plugins.xml` 类名 `peach2_end_effector::ShearV1` 等） |
| `cutter_end_effector.hpp` | 三插件共用的 `CutterEndEffector`（prepare / cut / confirm_cut / abort_safe / release / poll） |
| `tool_state_machine.hpp` | 纯核 `ToolStateMachine`：UNKNOWN→OPEN_CONFIRMED→CLOSING→CLOSED_CONFIRMED→OPENING→OPEN_CONFIRMED；超时/意外翻转/写失败→FAULT；FAULT 只由 `reset_by_ack()` 解除（→UNKNOWN）；`suspected_loopback()` |
| `io_backend.hpp` | `IoBackend`、`GatedIoBackend`（每次写入先过许可回调）、`MockIoBackend`（延迟、电流、7 种故障注入） |
| `aubo_io_backend.hpp` | `AuboIoBackend`：写 `/aubo_io_controller/set_io`（fun 取档案），读 `/aubo_io_controller/io_states` 的 `tool_io_states[feedback_pin]`；构造时拒绝 `cmd_pin == feedback_pin` |
| `tool_profile.hpp` | `load_tool_profile(tool_id, config_dir)`：yaml-cpp 读 `aubo_description/config/<tool_id>.yaml` 的 `geometry_m` 与 `io` |
| `types.hpp` | `TargetGeometry`、`BudgetView`、`RollConstraint`、`Feasibility`、`InsertPlan`、`ToolResult`、`CutVerdict`、`ToolState`、`ToolStatus`、`roll_frame` / `tcp_rotation` / `roll_of_direction` |
| `failure_codes.hpp` | `peach2_interfaces/FailureCode` 数值镜像（零 ROS；`peach2_manipulation` 用 static_assert 核对） |

库：`peach2_end_effector_core`（纯核 + 插件类，零 ROS）、`peach2_end_effector_plugins`（pluginlib 注册）、
`peach2_end_effector_aubo_io`（rclcpp + aubo_msgs）。

## 约定

- **刃面：** TCP 在开口，+Z = 开口朝向 = 袋轴；`blade_in_tcp() = translation(0, 0, -L_blade)`；刃面对准袋颈时
  `tcp = neck + L_blade * axis`，`Feasibility.overshoot_m = L_blade` 与 `tcp_at_cut` 上报给规划做碰撞检查。
- **滚转：** `roll_frame(axis)` 的 e1 = base +X 在轴法平面的投影（轴贴近 X 时用 +Y）。
  `branch_direction` 只在 `TargetModel.branch_direction_known` 为 true 且向量可用时由 manipulation 填入。
  shear：刃侧（TCP +X）背离枝，以 `branch_direction` 的反向为中心 ±60°（无枝方向时退到 `avoid_direction` 的反向）；
  bite：TCP +X 沿 `branch_direction`（钳口沿 +Y 闭合，⟂ 枝），±20°，周期 π；缺方向或枝方向与袋轴近平行（法平面投影
  < 0.1）时两者都是全周；adaptive：全周。
- **可行性：** 径向 `(D_inner − d95)/2 − wall_clearance ≤ 0` → `TOOL_NOT_FEASIBLE bag_wider_than_opening`；
  `length > L_insert` → `bag_longer_than_L_insert`；许可不足 → `BUDGET_RADIAL_NEGATIVE` / `BUDGET_AXIAL_NEGATIVE`（`ok` 仍为 true，
  PREGRASP_ONLY 可继续）。
- **剪切确认：** 反馈沿（CLOSING→CLOSED_CONFIRMED）必需；`current.enabled` 时再要求电流峰值 ≥ `peak_min_a` 且沿后 `window_s`
  内跌到 `drop_ratio × peak` 以下，否则 `CUT_NOT_CONFIRMED`。反馈在 `min_actuation_s` 内翻转 = 自回读嫌疑 → FAULT。
- **release / abort_safe** 不看感知许可；abort_safe 在 FAULT 下也发张开，但结果仍报 `TOOL_FAULT`。
- 几何只来自工具档案；代码里没有任何工具默认尺寸。

## 话题 / 服务 / 参数

本包不声明参数、不跑节点。IO 引脚、时序、电流判据由 `peach2_manipulation` 的参数注入 `EndEffectorContext`。
`AuboIoBackend` 使用固定名 `/aubo_io_controller/set_io`、`/aubo_io_controller/io_states`（驱动栈只读）。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --base-paths src/peach2 --packages-up-to peach2_manipulation --packages-skip peach2_interfaces peach2_core \
  --build-base build/v2/peach2_manipulation --install-base build/v2/peach2_manipulation_install
colcon test  (同上参数) && colcon test-result --test-result-base build/v2/peach2_manipulation --verbose
```

gtest：`test_tool_state_machine`（全转移、超时、自回读、FAULT/ACK）、`test_tool_profile`（三档案 + 非法文件）、
`test_plugins`（可行性、blade 偏移、滚转区间含枝方向约束与退化、insert 策略）、`test_mock_flow`（prepare→cut→confirm→release 全流程与各故障分支）、
`test_pluginlib_load`（按类名加载三插件）。lint 跳过 cpplint / copyright（spec）。

## 接口需求（不改 IDL，记录给接口 owner）

1. ~~枝方向~~ 已由变更 01 提供（`TargetModel.branch_direction_known` / `branch_direction`）。相机侧障碍方向
   （`avoid_direction`）仍无来源，shear 无枝方向时全周。
2. ~~反馈三态~~ 已由变更 01 提供（`ToolState.feedback` FEEDBACK_UNKNOWN/OPEN/CLOSED、`suspected_loopback`）。
3. 刀具电流无话题 / 字段来源（`AuboIoBackend::current()` 恒为空）；`ToolState.actuator_current_a` 发布 NaN。

## 已知限制与 TODO

- TODO(M0)：`tool_io_states[feedback_pin]` 是否真是独立 DI（P0-3 驱动按地址合并的怀疑）须真机反证；
  反证方法：刀断电/机械卡住时写 DO，反馈不得跟随；`ToolState.suspected_loopback` 置位且 FAULT 原因为 `suspected_loopback`。
- TODO(M0)：刃面约定 `blade_in_tcp = (0,0,-L_blade)` 与 L_insert 的含义（本包解释为可吞入的最大袋长）台架核对。
- TODO(M0)：`min_actuation_s`（0.03 s）、`feedback_timeout_s`（档案 1.5 s）按执行器实测修正；电平型执行器的
  `max_close_energized_s` 需实测后开启；急停复位后工具 DO 的保持行为须真机核实并写进启动自检。
- TODO(M4)：电流判据的传感器通道；adaptive 的导纳套入依赖腕部力传感（现由 manipulation 回退为 LIN）。
