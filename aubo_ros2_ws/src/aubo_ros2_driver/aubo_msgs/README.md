# aubo_msgs

AUBO 机械臂在 ROS 2 下使用的 **消息与服务定义**（ament_cmake），供 `aubo_driver_ros2`、`demo_driver` 等包引用。

---

## 消息（`msg/`）

| 文件 | 用途摘要 |
|------|-----------|
| `JointTrajectoryFeedback.msg` | 轨迹执行反馈（关节位置等） |
| `RobotStateRTMsg.msg` | 实时机器人状态 |
| `RobotModeDataMsg.msg` | 运行模式数据 |
| `ToolDataMsg.msg` | 工具/末端相关数据 |
| `MasterboardDataMsg.msg` | 主板数据 |
| `IOStates.msg` | IO 状态聚合 |
| `Digital.msg` / `Analog.msg` | 数字量 / 模拟量 |

---

## 服务（`srv/`）

| 文件 | 用途摘要 |
|------|-----------|
| `SetIO.srv` | 设置 IO |
| `SetSpeedSliderFraction.srv` | 速度滑条比例 |
| `SetPayload.srv` | 负载设置 |
| `GetIK.srv` / `GetFK.srv` | 逆解 / 正解 |

---

## 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_msgs
source install/setup.bash
```

---

*本包替换原占位用的上游 `ur_msgs` 说明；字段含义以各 `.msg` / `.srv` 文件内注释为准。*
