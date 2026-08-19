# aubo_e5_controllers

本地改写的 ros2_control 控制器：**只读**。

## 插件

| 控制器 | 作用 |
|--------|------|
| `AuboPassthroughTrajectoryController` | 整条轨迹透传到硬件 GPIO |
| `AuboIOController` | 板/工具 IO、`RobotStatus`、RIB |

配置由 bringup 的 `controllers.yaml` 加载（该文件亦只读）。技能与执行器不直接写关节命令，规划结果经 MoveIt / 轨迹接口到本控制器。

未授权不得 SetIO。
