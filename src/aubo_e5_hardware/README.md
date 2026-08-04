# aubo_e5_hardware — AUBO E5 ros2_control SystemInterface 插件（真机 + 板级模拟器）

## 简介

passthrough 架构的硬件层，提供两个 `hardware_interface::SystemInterface` 插件：
`aubo_e5_hardware/AuboE5Hardware`（真机，旧 SDK v1.3.1，TCP2CAN 位置流 + RIB
流量控制）与 `aubo_e5_hardware/AuboE5SimHardware`（控制柜模拟器，ursim 等价物，
无 SDK 无真机）。输入是 `AuboPassthroughTrajectoryController` 经
`trajectory_passthrough` GPIO 逐点下发的轨迹设定点与 `AuboIOController` 的 IO
写命令；输出是关节 position/velocity 状态、三组 GPIO 状态（含安全 IO、RIB
水位、事件/健康码）。真机插件把轨迹点五次重采样成 5ms 点流喂给控制柜接口板，
并把 SDK 推送的关节状态/事件回读到状态接口。上游契约由
`aubo_description/urdf/aubo_e5.ros2_control.xacro` 声明（接口布局 + 参数表），
启动入口在 `aubo_e5_bringup`；本包不自带 launch。

## 使用方法

构建（vendor SDK 二进制随包携带，无需额外安装）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_e5_hardware --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

启动（经 bringup 的 `hardware_mode` 选插件；sim 是无真机验证首选）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=169.254.10.98
```

sim 闭环冒烟（改本包代码后必做）：

```bash
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder 3
aubo_py3.12/bin/python tools/passthrough_traj_client.py sine_shoulder   # 压测重采样/流控
```

硬件参数全部经 URDF `<param>` 注入（`parseParams()` 全部为必填，权威默认值在
`aubo_e5.ros2_control.xacro` 宏定义处；bringup 只透传 `hardware_mode` 与
`robot_ip`，改其余默认值需改 xacro）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ip` / `server_port` | 169.254.10.98 / 8899 | 控制器地址（旧 SDK 端口） |
| `sdk_username` / `sdk_password` | aubo / 123456 | SDK 登录凭据，勿提交真实值 |
| `send_period_ms` | 4 | 发送线程周期 (ms) |
| `rib_target` / `rib_slowdown_1` / `rib_slowdown_2` | 400 / 300 / 350 | RIB 水位目标与两档降速阈值 |
| `batch_min` / `batch_max` | 2 / 8 | 每批续发点数上下限 |
| `ema_alpha` / `ema_boost_ms` | 0.1 / '10,14,20' | 发送耗时 EMA 系数与三档提速阈值 (ms) |
| `stop_retry_ms` | 20 | RobotMoveStop 重试间隔 (ms) |
| `prefill_points` / `force_start_delay_ms` | 0 / 0 | 预填充闸门（0/0 即关闭，ROS1 遗留） |
| `speed_guard_enabled` | false | 速度守卫（同点去重 + 超速拆分），默认关 |
| `max_joint_velocity` / `max_joint_acceleration` | 2.596…/3.110… / 17.31…/20.74… | 6 关节上限（前 3 大关节/后 3 小关节），供守卫用 |
| `point_spacing_s` / `same_point_eps` / `dedup_threshold` | 0.005 / 0.00015 / 0.000001 | 守卫点距/同点阈值/重采样去重 (s/rad) |
| `state_timeout_ms` | 200 | 关节状态推送新鲜度门限，超时 read() 报 ERROR |
| `auto_power_on` | false | activate 时自动上电；默认关，上电走 `/aubo_dashboard/startup` |

**真机安全约定**：任何运动测试先把速度/加速度缩放压到 0.1（RViz 滑条或
MoveIt 请求的 scaling factor），确认行为符合预期再逐步放宽；activate 后
TCP2CAN 独占运动通道、示教器运动暂停，deactivate 后交还；急停/防护停由本体
安全回路主导，软件侧只停发清队。另注意每次开机需手动关网卡 offload + 设
performance governor（见根 `AGENTS.md` 第 9 节），否则推送链路会停滞触发
read() FAULT。

## 执行逻辑

真机插件四条执行路径（`src/aubo_e5_hardware.cpp`；SDK `ServiceInterface` 实例
非线程安全，故两条连接各归一个线程独占）：

- **RT 线程**（controller_manager 200Hz）：`read()`（:910）从
  `RealtimeThreadSafeBox` 取关节状态快照（撞锁回退本地缓存帧，超
  `state_timeout_ms` 判推送断流）→ 换算（电机 RPM×kV2R→关节 rad/s，减速比
  前 3 轴 121、后 3 轴 101）填状态接口 → 健康门控。`write()`（:1015）跑
  passthrough 传递状态机（UR 契约，硬件回写 `transfer_state`/`abort`）：
  6 NEW_TRAJECTORY 清旧轨迹回写 1 → 2 TRANSFER_POINT 收一个点入
  `setpoint_queue_` 回写 1（队列满保持 2 让控制器重发）→ 3 TRANSFER_DONE
  转 4 IN_MOTION → 点收齐 + 全部重采样 + 发送队列排空后回写 5 DONE。
  两者纯内存操作，禁止 SDK 调用与阻塞。
- **发送线程 `sendLoop()`**（:1094，`send_period_ms` 4ms，独占
  `conn_control_`）：把 setpoint 段按全局 5ms 栅格五次插值重采样进
  `send_queue_`（系数与蓝本 aubo_boot 逐字一致，`quinticInterpolate()`
  :1444）；每周期查 RIB 水位（`robotServiceGetRobotDiagnosisInfo` 的
  `macTargetPosDataSize`）按三档定批量：rib<300 全速
  `clamp(ceil((400-rib)/6), 2, 8)`（EMA 超 10/14/20ms 批量下限抬 4/6/8）、
  300≤rib<350 每批 2 点、rib≥350 每批 1 点，经
  `robotServiceSetRobotPosData2Canbus` 喂接口板（5ms/点消费）。发送失败批
  次存 `pending_batch` 下周期优先重发保点序；查询或发送连败 5 次闩 FAULT。
- **IO 线程 `ioLoop()`**（:1471，20ms，独占 `conn_status_`）：drain SDK 事件
  队列（按级别打日志，ERROR 级连带清场 + 闩 FAULT）；处理 IO 写命令
  （`handleIoCommands()` :1560，DO 按名称 `U_DO_XX` 写并读回校验，结果回
  `set_io_async_success`）；~10Hz 安全 IO 轮询（DI0/8 低电平=急停、DI1/9
  低电平=防护停 → 停发清队 + health ESTOP，`pollSafetyIo()` :1643）；
  500ms 低频 IO/诊断轮询；停止原语 `RobotMoveStop` 按 `stop_retry_ms` 重试、
  1000ms 超时降级 `robotMoveFastStop`（丢弃板载 RIB 点）。
- **SDK 推送回调**（:1753/:1777）：只写快照缓存/事件队列/原子量，不调 SDK、
  不打日志（防重入死锁）。

停止分三类：正常完成=队列自然排空回写 DONE(5)；取消/抢占=控制器写 `abort=1`
→ 代际 +1 清双队列 + `RobotMoveStop`；急停/防护停=仅停发清队。健康一旦闩
FAULT/ESTOP 不在硬件层自动恢复：`read()` 报 ERROR → CM 走 `on_error()` →
`teardown()`（:839，停线程含板载停止 → 退 TCP2CAN → 注销回调/logout → 清队，
全程幂等），恢复统一走重新 configure/activate。

sim 插件（`src/aubo_e5_sim_hardware.cpp`）单线程，接口契约、状态机、五次
重采样与真机一致；虚拟接口板在 `write()` 内每周期恰好消费 1 个 5ms 点
（200Hz），`rib_level`=板载点数×6（与真板按关节分量计数的口径一致），
abort 丢弃队列即模拟 `RobotMoveStop`。不模拟板载 IO 写回（`set_io` 返回
success=false 属预期），`power_on` 恒 1、`health` 恒 0。

## 软件框架

```text
src/aubo_e5_hardware.cpp            # AuboE5Hardware：真机插件（本包主体，2051 行）
src/aubo_e5_sim_hardware.cpp        # AuboE5SimHardware：板级模拟器插件
include/aubo_e5_hardware/aubo_e5_hardware.hpp  # 真机插件声明（参数表/状态机常量/成员）
aubo_e5_hardware_plugin.xml         # pluginlib 描述（两个 SystemInterface 插件）
config/                             # auborobot.conf（DH 标定/robot_type）+
                                    #   tracelog.properties（SDK 日志）；SDK 按进程
                                    #   CWD 读 ./config/，装到 share，bringup 已把
                                    #   ros2_control_node 的 cwd 指到本包 share
vendor/                             # SDK v1.3.1 头文件 + lib64 二进制
                                    #   （libauborobotcontroller.so + protobuf 2.6.1/
                                    #   libconfig/log4cplus）+ 厂商文档；含 AMENT_IGNORE
```

对外契约（与 xacro `<gpio>` 段一一对应，`on_init` 会逐项校验布局）：

- 关节接口：6 关节（权威顺序 `shoulder_joint, upperArm_joint, foreArm_joint,
  wrist1_joint, wrist2_joint, wrist3_joint`），position 命令 + position/velocity 状态。
- `trajectory_passthrough`（命令接口，硬件回写其中 `transfer_state`/`abort`）：
  `setpoint_positions/velocities/accelerations_0..5`、`transfer_state`（0–6）、
  `time_from_start`、`abort`、`trajectory_size`。消费者：
  `AuboPassthroughTrajectoryController`。
- `speed_scaling`：`speed_scaling_factor` 状态，恒定 1.0（本驱动不做执行期缩放）。
- `aubo_io`：命令 `do_0..15`、`ao_0..3`、`tool_do_0..1`、`tool_ao_0..1`、
  `set_io_async_success`（命令槽以 NaN 为"无请求"哨兵）；状态 `di_*`、`ai_*`、
  `tool_di_*`、`tool_ai_*`、`estop`、`protective_stop`、`power_on`、`collision`、
  `in_motion`、`rib_level`、`joint_error_0..5`、`tag_pos_*`、`tag_vel_*`、
  `joint_current_*`（SDK 原始单位）、`joint_temp_*`、`send_queue_points`、
  `send_rate_pps`、`event_type`（-1=无事件）、`event_code`、`health`
  （0=OK/1=ESTOP/2=FAULT）。消费者：`AuboIOController`（IO 状态话题 +
  set_io 服务）与 diagnostics/live_monitor.py。

构建/部署要点：刻意使用 Jazzy 已 deprecated 的旧式接口导出（硬件把状态机
回写进 command 接口的关键，`-Wno-deprecated-declarations` 屏蔽告警，勿"升级"）；
链接强制 `DT_RPATH`（`--disable-new-dtags`）让传递依赖 `libprotobuf.so.9` 可被
解析，vendor 库装到 `lib/aubo_e5_hardware/vendor`（手动跑二进制需
`LD_LIBRARY_PATH` 指向它，经 launch 启动无此问题）；公共头 include 了 vendor
SDK 头而 vendor include 路径是 PRIVATE，暂不支持被下游包 include。lint 经
`ament_lint_auto` 接入 `colcon test`（cpplint/uncrustify 等全绿为合入前提）。
