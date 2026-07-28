# AUBO SDK 2.5.3 全面测试报告（2026-07-24，全部现场实测）

## 测试对象与环境

| 项 | 值 |
|---|---|
| 控制器 | `169.254.10.98:8899`，登录 `aubo` / `123456` |
| 控制器固件 | V4.5.111.456d8e2-Alpha（登录时 SDK 日志解析确认） |
| SDK | 2.5.3（`src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3/`，头文件与 `libaubo_sdk.so.2.5.3` 配套） |
| 机械臂 | 真机（`robotServiceGetIsRealRobotExist=1`，工作模式 real） |
| 测试程序 | `tools/aubo_sdk_full_test.cpp`（编译产物 `build/diagnostics/full_test`），辅助程序 `tools/aubo_sdk_motion_test.cpp`、`tools/aubo_sdk_jointmove_check.cpp` |

**运动约束**：全程仅使用 SRDF 中的 `home` 与 `camera_pose` 两个关节位姿（已换算为 SDK 关节序 shoulder→upperArm→foreArm→wrist1→wrist2→wrist3 并硬编码），未使用任何其他位姿；无 MoveL / 相对运动 / 拖动。

- `home`（SDK 序）：`{0, -0.0334, 1.236, -0.3675, 1.5701, 0}`
- `camera_pose`（SDK 序）：`{-0.2741117, 0.4963911, 1.7700852, -0.2978659, 1.5715849, -0.2750104}`

本报告所有数据均为 2026-07-24 当天现场实测，未引用任何历史测试结论。

## 总览

最终一轮完整运行（`full_test --rounds 2`）：**37 项用例，35 PASS / 2 FAIL**。
两项 FAIL（突发轮询、TCP2CAN 持续流）均为通道固有特性，见下文分析。

## 分阶段结果

### S0 ABI 自检 — PASS

`sizeof(JointStatus)=36`、`sizeof(wayPoint_S)=104`、`sizeof(RobotDiagnosis)=52`、`sizeof(ServiceInterface)=336`。头文件与库配套（旧版头文件 JointStatus 为 34 字节，混用即 ABI 错误）。

### S1 连接与登录 — 全部 PASS

login / connect_status / real_robot_exist(=1) / mac_communication(=1) / work_mode(=real) / logout / re-login 全部 rc=0。重复登录登出循环正常。

### S2 只读状态面 — 全部 PASS

- `robotServiceGetRobotCurrentState`：stopped
- `robotServiceGetRobotJointStatus`：6 关节位置/电流/温度/错误码正常（温度 33–38°C，各关节 error=0）
- `robotServiceGetCurrentWaypointInfo` 与 `robotServiceGetJointAngleInfo` 一致性：最大差 3.1e-6 rad
- `robotServiceGetRobotDiagnosisInfo`：power=1、brake=0、无软/远程急停、无碰撞、CAN 掩码 0
- 控制柜 IO 批量读取（用户 DI/DO/AI/AO）：46 条，rc=0；工具端数字 IO：4 条，rc=0
- 运动学回环 FK→IK：最大误差 1.8e-15 rad
- 四元数↔RPY 回环：误差 ~1e-7
- `getErrDescByCode(0)`："Success."

### S3 轮询基准 — 1 PASS / 1 FAIL

- 10Hz × 300 次 `robotServiceGetRobotJointStatus`：min/avg/p99/max = 2.19/2.77/4.42/9.28 ms，0 失败 — **PASS**
- 200 次背靠背突发：有效速率仅 **18 Hz**，2 次失败 — **FAIL**

注：同一会话早期（控制器相对"干净"时）突发实测约 **300–310 Hz、0 失败**；连续多轮测试后突发速率退化到 ~18 Hz 并出现失败，判断为服务端查询节流/资源累积。**结论：轮询应保持在 10Hz 量级，避免背靠背突发查询。**

### S4 实时推送 — PASS

换新连接后开启关节状态推送 10s：**33.4 Hz**（334 条），间隔 avg 29.45ms / max 32.66ms，关闭推送 rc=0 且确认停止。

重要伴随发现：若开启推送前刚做过突发轮询（不换新连接），推送退化到 **11–19 Hz** 并伴随 `Recv Header error` / 服务端断链，`push_off` 超时（rc=10007）。**结论：推送功能本身稳定（33Hz），但必须使用干净的连接；不要在同一连接上先压测再开推送。**

### S5 TCP2CAN 写通道（无运动，当前位姿原样回写）— 部分 FAIL

- 进入/退出 TCP2CAN 模式：rc=0
- 批量上限探测：batch **4 / 8 / 16 / 32 本次均被接受**（rc=0）
- 20 秒持续流（5ms 周期、RIB 水位门控 ≤120、batch ≤8、每周期查询诊断）：202 周期、写入 1558 点，**1 次写失败 + 1 次诊断失败**，写延迟 min/avg/p99/max = 2.0/67.9/537/5000ms — **FAIL**

**~5 秒级写停顿在多轮运行中均复现**（含完全独立的 `tcp2can_probe`：avg 65.7ms、max 5000ms、1 次失败），属于该固件 TCP2CAN 通道的固有特性，与测试顺序无关。RIB 排空正常（48 点 0.05s 排空）。

### S6 运动测试（仅 home ↔ camera_pose）

**6.1 高层 MoveJ 接口 `robotServiceJointMove` — 不可靠**

- 当天共 6 次尝试（覆盖全部三种重载：double[6] / wayPoint_S / MoveProfile_t；含/不含 `rootServiceRobotStartup`；全新连接）：**5 次被服务端拒绝，rc=10023（ErrCode_ResponseReturnError），机械臂不动**。
- 1 次（最终轮 S6.1）返回 rc=0 且 SDK 日志显示 "At track Target Pos"，但随后的位置测量表明机械臂**并未到达目标**（距 camera_pose 仍约 0.48 rad），属于假成功。
- 结论：当前固件（V4.5.111.456d8e2-Alpha）上高层运动接口不可用/不可信。

**6.2 TCP2CAN 流式插补运动 — 全部 PASS**

两端点间关节空间余弦插值（10ms 点距、平均速度 ≤0.25 rad/s、端点严格等于 home/camera_pose），经 `robotServiceSetRobotPosData2Canbus` 流式下发（与 ros2_control 硬件插件同一通道）：

| 腿 | 点数 | 耗时 | 写失败 | 到位误差 |
|---|---|---|---|---|
| home→camera_pose（R1） | 201 | 1.34s | 0 | 0.00006 rad |
| camera_pose→home（R1） | 214 | 1.50s | 0 | 0.00004 rad |
| home→camera_pose（R2） | 214 | 1.43s | 0 | 0.00005 rad |
| camera_pose→home（R2） | 214 | 1.41s | 0 | 0.00004 rad |

最终停在 home，残余误差 4.4e-5 rad（≈0.003°）。运动期间可观察到 RobotRunning 状态。

### S7 推送 + TCP2CAN 并发（破坏性）— 断链复现

推送与 TCP2CAN 回写同时开启：**22.8s 后断链**（前一轮 28.3s，推送速率随之退化）。断链后重新登录即恢复 — 断链问题在本固件上复现 2/2。**结论：推送与 TCP2CAN 不能在同一连接上并发使用。**

### S8 清理 — PASS

logout rc=0，所有阶段结束后连接均可正常关闭；阶段间掉线可自动重连恢复。

## 结论与使用建议

1. **可用的运动通道只有 TCP2CAN 流式**（`robotServiceEnterTcp2CanbusMode` + `robotServiceSetRobotPosData2Canbus`）：到位精度 ~5e-5 rad，与 ros2_control 插件现行方案一致。高层 `robotServiceJointMove` 在本固件上被拒（10023）或假成功，**不要用于生产**。
2. **状态轮询保持 ≤10Hz**；突发查询会触发服务端节流并污染该连接后续的所有操作。
3. **实时推送（33Hz）可用，但需专用干净连接**；且**严禁与 TCP2CAN 同连接并发**（约 20–30s 断链，2/2 复现）。
4. **TCP2CAN 写路径存在固有 ~5s 级停顿与偶发单次失败**（多轮 + 独立探针均复现），轨迹发送必须带 RIB 水位门控与失败重试/容忍，不能假设硬实时。
5. SDK 2.5.3 头文件与库必须配套（JointStatus=36B），ABI 自检应作为任何基于该 SDK 程序的启动检查。
6. 服务端状态随测试累积退化（突发速率 300→18Hz），长跑系统建议定期重建连接；必要时重启控制器。

## 复现方式

```bash
SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
g++ -std=c++17 -I $SDK/include tools/aubo_sdk_full_test.cpp \
  -o build/diagnostics/full_test -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
./build/diagnostics/full_test                 # 全量（含运动与并发）
./build/diagnostics/full_test --skip-motion --skip-coexist   # 只读
./build/diagnostics/motion_test               # 单独验证 TCP2CAN 流式运动
./build/diagnostics/jointmove_check           # 单独验证 jointMove 拒绝现象
```
