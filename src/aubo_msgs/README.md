# aubo_msgs

AUBO 驱动栈接口：状态、IO、FK/IK、手眼标定动作。采摘业务类型在 [`peach_interfaces`](../peach_interfaces/README.md)。

## 消息

`RobotStatus`、`JointStatus`、`IOState`、`Analog`、`Digital`、`JointPos`、`GoalPoint`、`TraPoint`。

## 服务 / 动作

- `SetIO`、`GetFK`、`GetIK`、`SetPayload`
- `ActivateHandEyeCalibration`、`RunHandEyeCalibration`

`RobotStatus` 由 `aubo_io_controller` 发，技能节点据此判断碰撞/急停。
