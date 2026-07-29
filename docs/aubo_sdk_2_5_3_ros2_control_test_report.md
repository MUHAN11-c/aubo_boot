# AUBO SDK 2.5.3 接入 ros2_control 测试记录

> **2026-07-29 更新**：本报告（含 2026-07-24 章节）描述的 `AuboE5Hardware`
> 旧插件与其继任者 `aubo_driver::AuboRos2System`（双连接 TCP2CAN 流控 +
> RIB 水位门控 + SPSC 无锁队列）均已随旧流式 JTC 架构归档 `src_legacy/`
> （2026-07-27 passthrough 迁移），且针对的是 SDK 2.5.3（现行插件用旧 SDK
> v1.3.1）。全文仅留作基线数据参考，现行实现见
> `docs/passthrough_migration.md`。

- 测试日期：2026-07-23
- ROS 版本：ROS 2 Jazzy
- ros2_control `hardware_interface`：4.45.2
- 控制柜地址：`169.254.10.98:8899`
- 控制柜版本：`V4.5.111.456d8e2-Alpha`
- SDK：`libaubo_sdk.so.2.5.3`
- SDK SHA-256：`3e41e298d67b78f74cd1d8f92957a047575e7a7e07f34da1b5fb5443be23b5cf`
- 安全条件：全部实机测试均使用 `allow_motion_commands=false`，未发送运动命令

## 结论

SDK 2.5.3 已能以“状态读取”方式接入 ros2_control：

- ros2_control 插件可以加载并完成 initialize、configure、activate。
- `AuboE5System` 和 `joint_state_broadcaster` 均能保持 `active`。
- `/joint_states` 能发布真实六轴位置。
- SDK 阻塞被隔离在后台健康线程中，ros2_control 的 100 Hz `read()` 读取缓存。
- SDK 瞬时超时并完成多轮自动重连后，硬件组件和 broadcaster 仍保持 `active`。

当前结论不包含运动控制兼容性。`write()` 仍在 ros2_control 循环中同步调用
`robotServiceFollowModeJointMove()`；该接口的阻塞时间、持续发送频率和实机安全性尚未测试，
因此不能据此启用 `allow_motion_commands=true`。

## SDK 二进制与 ABI

- 插件只依赖 `libaubo_sdk.so.2`，不再依赖
  `libauborobotcontroller.so.1`、protobuf 9、config 或 log4cplus。
- `ldd -r` 未发现缺失库或未解析符号。
- SDK 2.5.3 头文件和动态库必须成套使用。
- SDK 2.5.3 的 `JointStatus` 为 36 字节；旧工程中另一套头文件的
  `JointStatus` 为 34 字节，不能交叉链接。

## 原始 SDK 状态读取测试

测试程序：`diagnostics/aubo_sdk_runtime_probe.cpp`

### 30 秒基准

- 目标频率：10 Hz
- 样本数：300
- 状态读取失败：0
- 周期握手失败：0
- 无效数值：0
- 调用延迟：最小 2.377 ms，平均 3.702 ms，最大 16.343 ms

### 60 秒长时测试

- 目标频率：10 Hz
- 样本数：600
- 状态读取失败：2
- 周期握手失败：0
- 无效数值：0
- 调用延迟：最小 1.598 ms，含超时平均 22.531 ms，最大 5000.242 ms

这说明 SDK 查询通常为数毫秒，但最坏可同步阻塞约 5 秒，不满足硬实时调用要求。
SDK 调用不能直接放在 ros2_control 的实时 `read()` 路径中。

## ros2_control 频率

配置：

- controller_manager 更新频率：100 Hz
- SDK 健康线程轮询间隔：每次查询完成后等待 100 ms
- SDK 新状态名义频率：约 10 Hz；考虑正常查询耗时，实际略低于 10 Hz

`ros2 topic hz /joint_states --window 500`：

- 稳态平均频率：约 100.00 Hz
- 最小消息周期：约 8 ms
- 最大消息周期：约 12 ms
- 500 点窗口周期标准差：约 0.09–0.18 ms

`/joint_states` 的 100 Hz 是缓存发布频率，不代表控制柜产生 100 Hz 新遥测。
正常情况下，同一份 SDK 状态大约会被发布十次。

## ROS 消息延迟

`ros2 topic delay /joint_states --window 500`：

- 平均时间戳延迟：工具显示为 0.000 s
- 最大时间戳延迟：约 1 ms
- 500 点窗口标准差：约 0.08–0.13 ms

该结果测量的是本机 broadcaster 生成消息到本机订阅者接收的延迟，不包含机械臂采样到
SDK 返回之间的设备端延迟。

## 阻塞、超时和恢复

长时 ros2_control 测试中多次观察到：

1. `robotServiceGetRobotJointStatus()` 超时。
2. 单次 SDK 超时可持续约 5 秒。
3. 第 1、2 次快速重新登录通常失败。
4. 后续尝试成功，典型恢复窗口约 5.9–7.5 秒。
5. 恢复期间 `/joint_states` 继续以 100 Hz 发布最后一份缓存位置，速度被置零。
6. 多轮恢复后，`AuboE5System` 与 `joint_state_broadcaster` 仍为 `active`。

如果连续 5 次重连全部失败，插件才会向 ros2_control 返回硬错误并停用组件。

## ros2_control 实机状态

重连后查询结果：

```text
joint_state_broadcaster  active
AuboE5System             active
read/write rate          100 Hz
```

实测 `/joint_states` 位置：

```text
foreArm_joint   1.2474303246
shoulder_joint -0.0055356305
upperArm_joint -0.0227579307
wrist1_joint   -0.3661535680
wrist2_joint    1.5701397657
wrist3_joint   -0.0056671035
```

## 已实现的兼容处理

- 登录后首次关节状态最多重试 5 次，间隔 250 ms。
- SDK 查询在独立健康线程中运行。
- ros2_control `read()` 只读取受互斥锁保护的缓存，不直接调用 SDK。
- 只读模式在有限重连期间保持 ros2_control 组件激活。
- 状态过期或重连时速度置零。
- 全部重连失败后才返回硬件错误。
- 关闭过程中停止重连，避免关机故障误报。

## 已知限制与后续测试

1. 运动写入尚未实机验证。
2. `robotServiceFollowModeJointMove()` 当前位于同步 `write()` 路径；启用运动前应改成
   有界队列加独立发送线程，并测量平均、P99 和最坏阻塞时间。
3. SDK 发生 5 秒阻塞时，状态数据会冻结；100 Hz 话题仍发布缓存值。
4. 本次系统未获得 FIFO 实时调度权限，controller_manager 报告
   `Operation not permitted`。这不影响功能验证，但不属于实时生产配置。
5. SDK 会输出 `Get movep joint range failed`，本次测试中该日志没有阻止登录、
   状态读取或 ros2_control 激活。

## 复现命令

构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select aubo_e5_hardware aubo_e5_bringup
```

只读启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_e5_bringup bringup.launch.py \
  use_mock_hardware:=false \
  enable_real_hardware:=true \
  allow_motion_commands:=false \
  server_host:=169.254.10.98 \
  state_timeout_seconds:=3.0 \
  camera_enabled:=false \
  hand_eye_enabled:=false \
  start_moveit:=false
```

检查：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states --window 500
ros2 topic delay /joint_states --window 500
```

---

# 2026-07-24 运动接入实测（AuboRos2System / TCP2CAN）

- 测试日期：2026-07-24
- 控制柜：`169.254.10.98:8899`，固件 `V4.5.111.456d8e2-Alpha`
- 实现：`aubo_driver::AuboRos2System`（TCP2CAN 流控），controller_manager 100 Hz
- 结果：**home ↔ camera_pose 往返运动实机验证成功**，到位误差 <0.01 rad

## SDK 基线复测（探针：diagnostics/aubo_sdk_runtime_probe.cpp）

- 10 Hz × 300 样本：0 失败，延迟 min/avg/max = 2.2/3.3/15.5 ms，与昨日基线一致。

## 推送回调频率（新探针：diagnostics/aubo_sdk_push_probe.cpp）

- `robotServiceSetRealTimeJointStatusPush(true)` + 注册回调后，JointStatus
  推送稳定 **~33 Hz**（30 ms 间隔，30s 内 33-34 次/秒）。
- 60s 长测出现过一次服务器无预警断开 socket（12.9s 推送空洞）。
- **重要**：推送与 TCP2CAN 并存时，服务器在 ~12-17s 后主动断开 socket
  （2/2 复现）；单独推送或单独 TCP2CAN 均稳定。旧栈"三推送保活"经验在
  本固件 + TCP2CAN 组合下不成立，插件因此保持推送关闭、走 10Hz 轮询。

## 轮询极限（runtime_probe 变速）

- 目标 5ms：实际 avg 15.5ms/次（≈64Hz 有效），12/6000 失败，含 5s 阻塞。
- 目标 2ms：avg 15.7ms/次，32 失败 + 80 握手失败。
- 结论：服务器对查询节流，轮询上限 ~64Hz 且延迟随频率恶化；10Hz 是
  稳定工作点。状态反馈到不了 200Hz 不影响控制（read() 重发缓存即可）。

## TCP2CAN 写入路径（新探针：diagnostics/aubo_sdk_tcp2can_probe.cpp，无运动回写当前位置）

- 批次 ≤16 可靠；**批次 32 被服务器拒绝**（1518/1639 失败）。
- 写调用延迟 avg 38-60ms，p99 ~48ms，偶发 5s 阻塞（~0.2-0.5%）。
- 产能上限 **~125 pts/s**（与批次 8/16 无关，瓶颈是写调用延迟）。
- RIB 容量 1200 槽（插件激活时读取）。
- 诊断查询每周期一次会恶化到 avg 50ms / p99 3.7s；降到 ~25ms 间隔后
  回至 ~5ms 且写入吞吐不变。

## 服务器行为（多轮 bringup 实验归纳）

- 服务器全局串行处理请求：任一连接的高频/阻塞查询会拖累另一连接。
  空闲期 2Hz 诊断"保温"查询诱发 status 连接 11-13s 级反馈失新（v5/v6），
  撤掉后恢复（v7）。空闲期不查诊断；冷启动延迟改由轨迹起始 3.5s
  保持段吸收。
- SDK 单次查询偶发 ~5s 同步阻塞，空闲期属安全态（hold），不应闩锁；
  插件已改为仅轨迹执行中失联 >5s 才闩 traj_fault_（当日下午进一步放宽到
  >12s，见文末"2026-07-24 下午"章节）。
- 残余风险：轨迹执行中若撞上一次 >5s 阻塞（单次运动概率约 10-20%），
  会闩锁 traj_fault_ 并停用组件（原地安全停止），需重启 bringup 恢复。
  自动重连/自动再激活属于后续加固项。【2026-07-24 下午修订：闩锁阈值已
  放宽至 12s；当日下午实测显示该概率高于此前估计，见文末章节】

## 运动验证（v7 配置）

- 配置要点：update_rate 100Hz（匹配 ~125 pts/s 产能）；sender 诊断 25ms
  间隔；安全 IO 兜底 5s；named_pose 轨迹带 3.5s 起始保持段，总时长 9s。
- home → camera_pose：成功，SEND_SUMMARY ok=520 fail=1，写延迟
  avg 4.2ms / max 9.9ms，RIB 峰值 30。实测到位：
  `[1.77008, -0.27411, 0.49639, -0.29787, 1.57158, -0.27501]`，
  与目标偏差 ~1e-5 rad。
- camera_pose → home：成功，到位 `[1.23600, 0.00000, -0.03340, -0.36750,
  1.57010, 0.00000]`。
- 首次尝试（200Hz + 每周期诊断 + 无保持段）均失败：冷启动 ~4s 零发送、
  流式速率被反馈失新门控压到 ~6 pts/s、JTC path/goal tolerance 中止。
  以上配置均为解决这些问题所必需。

## 探针编译命令

```bash
SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
g++ -std=c++17 -I $SDK/include diagnostics/aubo_sdk_push_probe.cpp \
  -o build/diagnostics/push_probe -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
g++ -std=c++17 -I $SDK/include diagnostics/aubo_sdk_tcp2can_probe.cpp \
  -o build/diagnostics/tcp2can_probe -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
# 用法: push_probe [host] [duration_sec]
#       tcp2can_probe [host] [duration_sec] [batch_max] [diag_interval_cycles]
```

---

# 2026-07-24 下午：MoveIt 双管线上机 + SDK 通道降级事件

背景：同日下午为验证 move_group 双管线（OMPL+Ruckig / Pilz PTP，见
`docs/moveit2_control_mechanisms.md` §7.2）进行三轮 bringup 真机测试。
结果：机制验证部分通过，但 SDK 通道处于**持续性降级**状态，运动质量不可接受
（用户现场观察到卡顿响声、运动中途停止）。

## 通过的用例

| 用例 | 机制 | 结果 |
| --- | --- | --- |
| home → camera_pose（M1 回归） | named_pose 直接 JTC | 成功；SEND_SUMMARY ok=400 fail=0，ema 4.0ms / max 27.6ms，RIB 峰值 48；到位偏差 ~0.0006 rad |
| 走廊中段 → home（M2） | move_group OMPL+TOTG+Ruckig → ExecuteTrajectory | 规划 239 点 / 2.38s（resample_dt 0.01 生效）；走廊预检全路点过门；执行成功；到位偏差 0.0045 rad |

M2 是 move_group 完整链路（规划→TEM→JTC→TCP2CAN 流式）首次上机通过，
证明双管线配置与 Ruckig 平滑在真机上工作正常。

## 失败用例与降级证据

| 用例 | 结果 | 直接原因 |
| --- | --- | --- |
| camera_pose → home（会话 1） | JTC -5 goal 容差 | 写停滞 max 429.9ms，尾部点排空后停在目标前 ~0.02 rad |
| camera_pose → home（会话 2） | JTC -4 路径容差 | 写停滞 max 564.5ms，随后 640.2ms → FAULT_STOP |
| home → camera_pose（M2，会话 3） | JTC -4 路径容差 | **写停滞 2059ms**（TIMING_SPIKE），跟踪滞后 >0.15 rad 中止，随后 FAULT_STOP |

- 三轮会话中每次运动都出现 0.4–2s 级写停滞，幅度逐轮增大；启动期即有
  持续 FEEDBACK_STALE（>10s），与 v7 基线（avg 4.2ms / max 9.9ms，全程
  无停滞）严重背离。
- 每次中止后硬件按设计闩锁 traj_fault 并停用（安全原地停），需重启 bringup。
- **疑似根因**：当日多次硬杀 ros2_control_node（SDK 未正常登出）+ 服务器
  全局串行处理 → 疑似僵尸会话/服务器状态堆积拖垮通道。本会话结束时已
  通过 launch SIGINT 走完整生命周期（stopMotion → leaveTcp2Canbus →
  shutdown logout）干净退出。
- **处置建议**：重启控制柜清除服务器侧状态后再跑完整测试矩阵；后续应
  避免直接 `kill` ros2_control_node，一律经 launch 正常关闭。
- M3（Pilz PTP）因通道降级未执行，管线本身已在 mock 链路验证（14 点 /
  1.24s 确定性轨迹）。

## 根因再分析（对照两套原版驱动后修订）

对照 `/home/mu/Videos/aubo_robot-Noetic`（ROS1）与
`/home/mu/Videos/aubo_ros2_ws`（官方 ROS2，本插件 sender 流控的原版）
后，根因从"僵尸会话"单一归因修订为三层：

1. **写停滞是 SDK 固有行为，v7 是幸存者偏差**。探针早已测得写调用
   "偶发 ~5s 阻塞（0.2-0.5%/次）"；每腿轨迹 200-600 个写批次，期望即
   0.4-3 次/腿。v7 两腿零停滞（max 9.9ms）属小概率好运；当日下午
   "每腿都撞"才符合基线概率。
2. **中止是链路架构差异，不是停滞本身**。参考驱动（aubo_ros2_ws）预插值后
   盲流式下发、无 JTC 实时跟踪监控：停滞时机械臂原地等、结束后续走，
   运动只变慢不中止。本链路 JTC trajectory 容差（原 0.15 rad）+ 反馈
   失新闩锁（原 >5s）把每次固有停滞放大成"中止 + 重启"。
3. **RIB 缓冲无法吸收停滞**：控制器消费率 ~213 pts/s（4.7ms/点）大于
   SDK 写产能 ~125 pts/s，流式期间 RIB 注定近空（v7 峰值仅 30 槽），
   加大水位带也攒不起缓冲——此方向不可行，已排除。

僵尸会话/服务器状态堆积（当日多次硬杀进程未正常登出）是叠加恶化项，
解释当日下午启动即持续失新 >10s；已通过干净登出释放，控制柜重启可彻底清除。

## 修复（2026-07-24 下午实施，构建通过，待控制柜重启后复测）

让链路与参考驱动一样"停滞可恢复"：

1. **JTC 容差放宽**（`aubo_e5_bringup/config/controllers.yaml`）：
   trajectory 0.15 → **0.75 rad**（0.3 缩放下可吸收 ~4s 停滞；v7 低速轨迹
   可吸收 ~14s）；goal_time 1.5 → **8.0s**（停滞后延迟完成不再误判失败，
   仍小于 named_pose 的 25s 结果超时）。真实脱轨防护由硬件路径门
   （走廊 ±0.02 rad）承担，JTC 容差只负责流式滞后，放宽不降安全性。
2. **反馈失新闩锁放宽**（`aubo_ros2_system.hpp`）：kFeedbackFaultNs
   5s → **12s**。写停滞会经服务器全局串行拖垮 status 查询（实测缺口 >10s），
   5s 把"停滞伴生失新"误判为失联杀死会话；真正断连仍由
   RobotEvent_socketDisconnected 事件即时检测。失新期间机械臂只在
   授权走廊内执行 RIB 存量后原地 hold，风险有界。
3. **纪律**：停用系统一律经 launch 正常关闭（SIGINT），不硬杀
   ros2_control_node，避免服务器侧堆积僵尸会话。

未做（评估后排除/留后续）：加大 RIB 水位带（产能 < 消费率，攒不起）；
自动重连/自动再激活（Phase C 加固项）；UR 式 passthrough 控制器
（长期方向，见 moveit2_control_mechanisms.md §7.4）。

复测判据：同 §8 矩阵（M1 往返 / M2 OMPL+Ruckig 往返 / M3 Pilz 往返），
预期停滞发生时运动"暂停-续走"而不再中止，全程无 traj_fault 闩锁。

---

# 2026-07-24 晚：最终修复 + 复测通过

## 修复内容（本次提交）

1. **插件**（`aubo_ros2_system.hpp/.cpp`）：`tcp2can_batch_max` 硬件参数化
   （clamp [1,32]，默认 8）；大批次重试仍失败 → 拆成 ≤8 小批按序重发并
   降回 8（服务器拒绝是状态相关的，保险回退）；SPSC 队列 1024→2048
   （@100Hz ≈ 20s 存量）；清理陈旧注释。
2. **launch**：新增 `tcp2can_batch_max` 参数（默认 8）。
3. **构建修复**：`SDK资料/COLCON_IGNORE`——colcon 此前把
   `SDK资料/aubo_robot-Noetic` 的 catkin 包纳入解析，重名
   `aubo_e5_moveit_config` 错解析到 catkin 版（依赖 aubo_gazebo）导致
   aubo_e5_bringup 构建失败。
4. **判定门（batch 16/32）**：单次调用 rc=0 可接受，但持续流下
   batch16 被拒 88%（1499/1707）、batch32 被拒 99.8%（3720/3726），
   batch8 仅 0.25% 失败 → **保持默认 8**，URDF/launch 默认值不改。

## 根因定论：严重通道降级的主因是主机 NIC 驱动/offload 状态

当日自然实验链（同一控制器、同一份代码）：

| 时点 | 主机侧变化 | 通道表现 |
| --- | --- | --- |
| 控制柜重启后、PC 重启前 | 无 | 仍降级：tcp2can_probe 1 失败 + 5000ms 停滞 + leave 时 socket 断（10004）；M1 首跑流式塌缩、goal_time 超时中止 |
| PC 更新网卡驱动（Realtek r8126 DKMS）+ 重启 + offload 关闭后 | 内核/驱动重载，gro/gso/tso=off | **全部干净**：见下 |

- PC 重启后 `runtime_probe` 60s×600：0 失败，延迟 max 17.4ms（此前有 5s 阻塞）。
- PC 重启后 `tcp2can_probe` 30s（batch 8）：599 周期 / 4792 点 / **0 失败**，
  写延迟 max **84.5ms**（此前每次必现 ~5000ms），产能 158 pts/s（此前 80–104）。
- 结论：此前归因于"SDK 服务端固有"的秒级停滞与 socket 断链，主因实际是
  **主机 Realtek r8126 网卡的驱动/offload 状态**；服务端固有毛病中仍成立的是
  jointMove 10023 拒绝与 push+TCP2CAN 同连接断链（固件行为，与 NIC 无关）。
- 注意：ethtool offload 关闭**重启后不持久**，`configure_realtime.sh`
  需每次开机后执行（realtime_preflight 已把 governor 纳入硬门槛，offload
  不在门槛内，建议运维清单化）。链路速率 1000Mb/s 全双工（带宽非瓶颈）。

## 复测矩阵（PC 重启后，同一会话）

| 用例 | 机制 | 结果 |
| --- | --- | --- |
| M1 camera_pose→home | named_pose 直接 JTC | **Goal reached**；到位误差 ~4e-4 rad |
| M1 home→camera_pose | 同上 | **Goal reached**；到位误差 ~3e-6 rad |
| M2 camera_pose→home | move_group OMPL+TOTG+Ruckig → ExecuteTrajectory | **SUCCESS**（规划 306 点 / 3.05s） |
| M2 home→camera_pose | 同上 | **SUCCESS**（311 点 / 3.09s） |
| M3 camera_pose→home | Pilz PTP → ExecuteTrajectory | **SUCCESS**（38 点 / 3.68s，确定性） |
| M3 home→camera_pose | 同上 | **SUCCESS**（38 点 / 3.63s） |

会话总计：**ok=1454 批 fail=0**，RIB 峰值 48，写 ema ~10ms（max 82.5ms 尖峰被
吸收），**0 次 FAULT_STOP、0 次 traj_fault 闩锁、0 次 JTC 中止**。干净退出
（stopMotion → leaveTcp2Canbus → logout）无僵尸会话。

复测客户端：`tools/m2_m3_retest.py`（/plan_kinematic_path + /execute_trajectory，
用法 `python3 tools/m2_m3_retest.py [ompl|pilz] [home|camera_pose]`，可复用）。

## 遗留事项

- 显卡驱动不匹配（内核模块 595.71.05 vs 用户态 595.84）→ RViz GLX 崩溃，
  需重装显卡驱动；与控制链路无关，M2/M3 已可脚本化执行不依赖 RViz。
- 自动重连（Phase C）仍留作后续加固项；真断连当前走 read() ERROR 标准
  停控制器路径（安全）。

---

# 2026-07-25 凌晨：卡顿根治——SDK 换回 v1.3.1

## 根因链（三层，全部实锤）

1. **写延迟是平滑度的决定变量**：v2.5.3 `SetRobotPosData2Canbus` 调用
   avg ~41ms → 产能天花板 ~190 pts/s，实际 125-147 pts/s < 控制器消费率，
   RIB 长期贴零（0-6 槽），生产纹波直接变成机械臂速度纹波（卡顿/响声）。
   上采样 ×2、40ms 节拍器、diag 降频等 producer 侧调优**全部无效**（实
   测体感无改善）——写延迟这堵墙在 v2.5.3 上无法逾越。
2. **v1.3.1 探针对照**（`tcp2can_probe_v131`，同一探针源码仅换库）：
   写延迟 **avg 4.8ms / p99 15ms / max 57ms，0 失败，产能 198 pts/s**
   （且是 diag 每周期查的情况下）——快 ~9 倍，产能足以灌满 RIB 让控制
   器匀速消费。这正是旧栈（ROS1/自研 ROS2）运动平滑的结构前提。
3. **换库后实测**（M1 named_pose）：写 ema 5-8ms，流式速率 179-303
   pts/s，RIB 水位 64-156（原 0-6），零失败，**用户现场确认运动平稳**。

## 变更

- `src/aubo_e5_hardware/vendor/aubo_sdk_1_3_1/`：vendor v1.3.1 全套
  （`libauborobotcontroller.so.1.3.1` + log4cplus 1.2 + libconfig 11 +
  protobuf 9.0.1 + libotgLib.a，头文件 V1.0.0 2017；来源 aubo_ros2_ws
  自研驱动）。**API 与 2.5.3 全兼容**（31 个在用方法签名、回调类型、
  字段名逐一核对），插件源码零改动一次编译通过。注意 JointStatus 为
  34B（2.5.3 为 36B），两套头/库仍严禁交叉。
- `CMakeLists.txt`：链接目标 2.5.3 → v1.3.1 五库，RPATH/安装同步；
  2.5.3 保留在 `vendor/aubo_sdk_2_5_3` 仅作对照，诊断探针仍可用它编译。
- sender 调优回退：diag 恢复每周期查（Fix14 原味，v1.3.1 下无带宽问
  题）、移除 40ms 节拍器（v2.5.3 时代补救）、保留乐观增量与 pts/s 摘要。

## 同晚已并修复（先于换库，均为必要修复）

- **推送三通道启用**（conn_status_）：并发 300s 零断链（旧"必断链"是
  网卡问题）；33Hz 位置 + 20Hz 轮询冗余。
- **jointSpeedMoto 不可作速度状态**：实测为电机侧整数量化值（静止 0/±2
  跳变），JTC 用作样条初值会开局冲刺偏出走廊（cause=2 闩锁，快照实锤
  单周期 +0.043 rad）→ 速度改推送位置 33Hz 差分，8/8 压力往返零闩锁。
- **静默闩锁仪表化**：`fault_cause_` 原因码 + 被拒命令快照（rej_cmd/
  ref/site/progress），FAULT_STOP 打印，RT 路径零日志。
- statusLoop 10→20Hz；realtime_preflight 增加 NIC offload 检查（WARN）。

## 当前状态判定

- 运动通道：v1.3.1 + TCP2CAN 流式，RIB 水位带 [60,120] 实测稳定
  64-156，M1/M2/M3 机制全部在线。
- jointMove 10023（固件拒绝）与网卡 offload/驱动注意事项不变
  （见 docs/nic_driver_incident.md）。
- 待办：RViz 显卡驱动重装；长跑稳定性观察（v1.3.1 会话老化行为）。

## 2026-07-25 凌晨补：路径门对规划器的两处误杀修复

用户在 RViz 用 OMPL（1.0 全速缩放）连续 Plan&Execute 时触发两次误闩锁，
快照定位、修复、复验如下：

1. **cause=2 走廊容差误杀**：OMPL/RRTConnect+Ruckig 路径在关节空间
   非直线，实测 wrist1 偏离 home↔camera 直线 0.0213 rad > 原容差 0.02。
   `kPathTolerance` 0.02→**0.05**（仍是有界防护，挡 ≫0.1 rad 级的假数据/
   错误轨迹）；`named_pose_controller.py` 预检容差同步放宽（否则臂停在
   略偏线位置时授权服务无法召回）。
2. **cause=3 路径反向误杀**：OMPL 路径在"线进度"坐标上非单调，返程
   中进度合法回弹 >0.002 触发反向闩锁。反向检测滞后 0.002→**0.02**
   （真正要抓的 JTC 取消/恢复追赶回跳 ≫0.02）。
3. **复验**：OMPL 1.0 缩放 3 往返 6/6 SUCCESS、0 FAULT_STOP；臂从
   离线 0.021 rad 处经 /aubo/move_home 平稳召回（Goal reached）。
4. 现象解释（用户反馈的"运动前后延时/杂音"）：每腿轨迹开始前有
   ~0.2-0.4s RIB 冷启动填充期（空闲期不查诊断+从零灌缓冲，臂才动），
   结束后 RIB 残余 56-118 点（~0.3-0.6s）继续排空才停稳——均为流式
   架构的固有瞬态，非故障；故障时的急停声来自 stopMotion（jerk 兜底）。
   v1.3.1 启动期 log4cplus 报 `could not open file ./config/tracelog.
   properties` 为老库缺日志配置文件，无害。

## 2026-07-25 凌晨补 2：中段卡顿与速度

1. **中段卡顿根因**：v1.3.1 写路径仍有偶发 ~220ms 尖峰（m13 实测
   TIMING_SPIKE ×2），按消费 ~200 pts/s 折合 44 点；旧水位带 [60,120]
   下限 60 恰被排空到 0（RIB_IDLE ×2），形成用户听到的 2 次中段卡顿。
2. **修复**：水位带 [60,120]→**[100,200]**（100 槽 ≈ 500ms 吸收窗），
   kFreezeCycles 250→350（随排空时间 ~1s 上调）。m14 复验：同类 214ms
   尖峰打在 rib=76 上被完全吸收，速度曲线无停顿、Goal reached。
3. **提速**：named_pose 保持段 6s→3s（v1.3.1 冷启动 ~0.3s 不需长保持）、
   运动段 12s→8s（~0.13 rad/s 低速安全档）；整腿墙钟时间 ~15s→~8s。
   更快可用 `tools/m2_m3_retest.py pilz <target> 0.8`。
4. **前后窗口声音的实测归因**（joint_trace_recorder 轨迹分析）：运动前
   窗口 = JTC 闭环把臂从实际位置对齐到轨迹缓存起点的微小蠕动
   （~0.002 rad）；运动后窗口 = RIB 残余轨迹尾段排空爬行
   （~0.002-0.01 rad）——均为正常定位行为，非伺服异常。
