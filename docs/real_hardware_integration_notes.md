# AUBO E5 真机接入核对与实时通信说明

> **历史参考（2026-07-29 起）**：本文是旧流式 JTC 架构时期的真机接入设计
> 笔记（`aubo_e5_arm_controller`、FollowModeJointMove 流式跟随、SCHED_FIFO
> 实时线程设计等），所述实现已归档 `src_legacy/`；RT 实时性要求亦已取消
> （real 模式普通内核直接运行）。现行真机流程见 `docs/usage.md` 第 7 节。

## 当前结论

工作区的 mock 链路已验证可启动、规划并完成一次轨迹执行。`controller_manager` 成功以 `SCHED_FIFO` 优先级 50 运行；当前用户的 FIFO 实时优先级范围为 1--99。

这不等价于真机链路已验证：mock 使用 `mock_components/GenericSystem`，不会加载 `aubo_e5_hardware/AuboE5Hardware` 插件，也不会连接 AUBO 控制器。

## 已验证的 mock 链路

```text
MoveIt / RViz -> move_group -> FollowJointTrajectory -> joint_trajectory_controller -> mock_components/GenericSystem
```

- `joint_state_broadcaster` 与 `aubo_e5_arm_controller` 均成功激活。
- MoveIt 已完成一次规划和执行，控制器报告 `Goal reached, success`。
- 未配置 Octomap 3D 传感器插件和 `/recognize_objects` 是未启用感知/识别功能时的非阻断提示。

## 真机插件的当前阻断项

`AuboE5Hardware` 声明了旧式 `export_state_interfaces()` 与 `export_command_interfaces()`，但没有实现。已安装共享库保留这两个未定义符号，因此真机插件在动态加载阶段不能视为可用。

必须选择并完整实现一种接口绑定方式：

1. 实现旧式导出函数，把 position、velocity 与 command 缓冲显式绑定到 `std::array`；或
2. 使用 Jazzy 框架托管接口，在 `read()` 中用 `set_state()` 更新状态、在 `write()` 中用 `get_command()` 读取控制器命令。

两种模式不可半实现或混用。修复后应执行插件加载测试，随后才可连接真实控制器。

## 推荐的真机链路

```text
                         非实时 SDK / 网络线程
             +-----------------------------------------+
             | TCP 登录、状态接收、发送、重连、健康检查 |
             | 状态回调 -> StateBuffer                 |
             | LatestCommand -> 限速/检查 -> SDK       |
             +-------------------+---------------------+
                                 ^|
                    无锁快照     || 最新命令覆盖
                                 ||
             +-------------------+---------------------+
             | controller_manager FIFO 实时线程        |
             | read -> JTC update -> write              |
             | StateBuffer -> state interface           |
             | command interface -> LatestCommand       |
             +-----------------------------------------+
```

### 生命周期

1. `on_init()`：校验 6 个关节、接口与参数，并预分配固定大小缓冲区。
2. `on_configure()`：建立 TCP/SDK 登录、注册状态回调；此阶段允许阻塞。
3. 收到连续且新鲜的关节状态后才允许激活。
4. `on_activate()`：确认控制器安全状态，按现场批准的策略上电/松刹车/设置碰撞等级；将 command 初始化为当前实际关节位置。
5. ACTIVE：实时线程只做无锁状态快照和命令发布；SDK I/O 线程执行网络通信。
6. `on_deactivate()`：停止新命令、让机械臂安全停止/制动；不要把 TCP 断开与停机混为一步。
7. `on_cleanup()`：取消状态回调、关闭状态推送、退出登录并释放网络资源。

厂商 SDK 提供 `rootServiceRobotStartup(...)`、`robotServiceRobotShutdown(...)`、碰撞等级和快速停止相关接口。实际调用顺序、工具动力学参数与安全策略必须以现场控制器版本和厂商手册为准。

## 缓冲与 TCP 策略

### 不阻塞实时循环

`read()` 和 `write()` 运行在 `controller_manager` 的实时控制路径中。它们不得等待 TCP、SDK 确认、重连、日志、动态内存分配或互斥锁。当前实现中 SDK 回调与 `read()`/`write()` 共享 `std::mutex`，真机版本应改为预分配的无锁双缓冲或 realtime buffer。

### 最新命令优先

对 `robotServiceFollowModeJointMove()` 这类流式跟随接口：

- `write()` 将整组关节目标写进一个固定大小的 `LatestCommand` 槽并立即返回；
- I/O 线程按照已验证的设备跟随频率读取该槽并发送；
- 网络慢时覆盖未发送的旧目标，不能建立无界 FIFO；
- 仅在厂商明确要求轨迹预缓冲时，维护小且固定的前瞻深度。

无限命令排队会在暂停、取消或急停后继续执行过期目标。小缓冲能降低取消延迟，但其频率、深度和控制器允许的插补间隔必须通过厂商文档和低速现场测试确定。

### 状态、超时与恢复

- 每帧状态应有接收时间戳和序号；超时即置通信故障。
- SDK 调用连续失败、状态停更、控制器进入保护/急停状态时，I/O 线程置健康标志为失败。
- 实时 `read()` 观察该标志并返回错误；非实时路径负责停机、报告和重连。
- 连接恢复后必须重新读取实际状态、重新同步 command，不能自动恢复旧轨迹。
- 急停必须依赖机械臂本体安全回路；普通 TCP 指令只能作为补充，不能作为唯一急停机制。

## 真机首次测试顺序

1. 现场确认急停、限位、碰撞等级、工装/负载、工作空间与低速模式。
2. 仅启动硬件插件和 `joint_state_broadcaster`，核对全部关节名称、单位、正方向、位置和速度。
3. 验证状态丢失、TCP 断线、SDK 错误时控制器会停用且不会继续发送旧目标。
4. 激活轨迹控制器，先做小幅、低速、单关节受监控运动。
5. 最后再启动 MoveIt/RViz 并允许轨迹执行。

不要直接使用带 MoveIt/RViz 的组合启动命令作为首次真机通电测试。

## 参考

- 本工作区 `src/aubo_e5_hardware` 中的硬件插件和随附 SDK 头文件。
- [ROS 2 Control: Getting Started](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [ROS 2 Control: Writing a Hardware Component](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- `/home/wjz/Robotics_Tutorial/05_运动控制/20_机械臂/M12_ros2_control与硬件驱动.md`
