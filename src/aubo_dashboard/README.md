# aubo_dashboard

柜侧慢操作节点：上电、抱闸、停机、FK/IK、负载。**非运动轨迹**。

## 红线

- **禁止调用** `/aubo_dashboard/startup`
- 真机 bringup 必须 `auto_power_on=false`

上电只能人工确认后执行。节点随 `hardware_mode:=real` 由 `bringup.launch.py` 拉起。

服务包括 `startup`、`shutdown`、`release_brake`、`stop`、`fast_stop`、`collision_recover`、`GetFK`、`GetIK`、`SetPayload`。采摘栈默认不碰这些。
