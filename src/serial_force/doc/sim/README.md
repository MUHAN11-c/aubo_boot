# 五点力 CoP 仿真实验

这些工具已合并在 `serial_force` 包，与串口驱动共用 `/force/*` 契约。虚拟力源不碰串口；CoP 对中工具会直接调用 `/compute_ik` 和 JTC，只供独立、明确授权的 mock 实验，不进入 `harvest_system`。同一图中只能运行一个串口或仿真力源。

- [模型、控制律与 2026-10-21 历史结果](force_align_sim.md)
- [历史统计](force_align_sim_stats.md)、[结果图](force_align_sim.png)
- [驱动说明](../../README.md)、[点位](../geometry.md)、[接口](../io.md)

## 当前可执行入口

| 命令 | 用途 |
|---|---|
| `ros2 launch serial_force serial_force.launch.py source:=sim` | 互斥启动虚拟平面力源，节点名仍为 serial_force |
| `ros2 run serial_force force_sim_node` | 直接运行同一力源，不能与上一入口或串口源并行 |
| `ros2 run serial_force force_cop_control` | 单次 CoP→IK→JTC 对中 |
| `ros2 run serial_force force_cop_trials` | 倾角/方位/重复批量实验，写 JSONL |
| `ros2 run serial_force force_cop_analyze <jsonl>` | 离线生成 CSV、统计与图 |

默认输出目录是 `$ROS_HOME/serial_force/`，未设置 ROS_HOME 时为 `~/.ros/serial_force/`；trials 支持 `--out` / `--log-dir`，analyze 支持 `--out-dir`。运行页面由 `peach_web /force.html` 提供，不由本工具启动。

## 实验参数与边界

力源参数：`tilt_deg=3.0`、`tilt_yaw_deg=0.0`、`preload_mm=0.30`、`k_n_per_mm=0.8`。控制参数：`--center-mm 1.2`、`--max-rot-deg 5`、`--max-steps 12`；批量默认 `--tilt 1,2,3,4,5 --yaw 0,90 --repeats 1`。

首次有效 tcp TF 冻结虚拟平面，100 Hz timer 生成 `ForceSample` 和四个兼容 array。静力线性弹簧模型没有摩擦、迟滞、噪声或臂动力学；伪 AD/status/travel 不证明真实硬件精度，也不能当作主链授权。2026-10-21 的 20 组结果是历史 mock 数据，不能替代当前源码回归或真机验证。

本次源码文档整理未运行实验、硬件或 Gazebo。需要运行时按工作区 [测试指导](../../../peach/peach_system_tests/doc/testing.md) 盘点进程，使用独立 mock 环境，并清理本轮拥有的进程。
