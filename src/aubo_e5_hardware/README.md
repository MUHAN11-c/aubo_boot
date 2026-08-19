# aubo_e5_hardware

ros2_control `SystemInterface` 插件：**只读**，勿改。

## 插件

| 插件 | 用途 |
|------|------|
| `AuboE5Hardware` | 真机：旧 SDK + TCP2CAN 流 |
| `AuboE5SimHardware` | 无柜仿真闭环（`hardware_mode:=sim` 首选） |

URDF 经 `aubo_e5.ros2_control.xacro` 装载插件。轨迹走透传 GPIO，不在本包做规划。

未授权不得连真机运动。
