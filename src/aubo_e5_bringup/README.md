# aubo_e5_bringup

手臂工作单元**唯一启动入口**：`bringup.launch.py`（**只读**）。采摘整栈 include 本 launch。

## 模式

| `hardware_mode` | 硬件 | 轨迹 |
|-----------------|------|------|
| `mock` | `GenericSystem` | 标准 `joint_trajectory_controller` |
| `sim` | `AuboE5SimHardware` | 透传控制器 |
| `real` | `AuboE5Hardware` | 透传控制器 |

xacro 同一份模板，按 mode 换插件。`auto_power_on` 必须为 false。**不起** `aubo_dashboard`：柜侧用示教器，规划用 MoveIt，停轨走透传取消 + 硬件 `RobotMoveStop`。

## 可选块

相机、手眼静态 TF、手眼标定流程由 launch 参数开关（`camera_enabled` 等）。独立功能不依赖 OpaqueFunction 里的 mode 分支。

## 用法

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
```

采摘请用 `peach_task_executor/harvest_system.launch.py`，它会 include 本文件。启动前 `pgrep` 避免重复容器。
