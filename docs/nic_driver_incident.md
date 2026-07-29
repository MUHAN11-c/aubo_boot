# AUBO 通信网卡（Realtek r8126）驱动问题记录

记录日期：2026-07-24。目标机器：`mu-MS-7E34`，Ubuntu 24.04，ROS 2 Jazzy。
涉及链路：PC `enp130s0` ↔ AUBO 控制柜 `169.254.10.98:8899`（SDK over TCP）。

## 结论（先说）

**此前归因于"AUBO SDK 服务端固有"的秒级停滞、socket 断链、会话级通道降级，
主因实际是主机 Realtek r8126 网卡的驱动/offload 状态。**
更新网卡驱动（r8125/9.011.00 → **r8126/10.017.00-NAPI**，DKMS）并重启 PC 后，
所有秒级停滞消失；`jointMove` 被拒（10023）与 push+TCP2CAN 同连接断链仍是
控制柜固件（V4.5.111.456d8e2-Alpha）行为，与网卡无关。

## 症状（2026-07-23 ~ 07-24）

- SDK 调用偶发 ~5 秒级同步阻塞（读/写/诊断调用均出现，~0.2–0.5%/次）；
- TCP2CAN 写延迟 avg 38–68ms、max 5000ms，写产能仅 ~80–125 pts/s；
- 写路径偶发 1s 整超时（ret=10007）、socket 被断开（10004）；
- 服务器对查询"节流"的假象：背靠背突发从 ~300Hz 退化到 ~18Hz；
- 运动执行中流式速率塌缩（~15 pts/s）→ JTC goal_time 超时中止
  （详见 `docs/archive/aubo_sdk_2_5_3_ros2_control_test_report.md` 下午章节）。

## 排查路径（逐项排除）

1. **控制柜重启**：burst 轮询恢复 264Hz，但 5s 写停滞与断链**依旧**
   → 不是单纯的服务器侧累积状态。
2. **SDK 全量现场实测**（`tools/aubo_sdk_full_test.cpp`，37 用例）：
   只读面/运动学/IO 全部正常 → 不是协议/ABI 问题。
3. **关节运动接口 10023**：三种重载全新连接均被拒 → 独立固件问题，与卡顿无关。
4. **自然实验（决定性）**：同一天、同一控制器、同一份代码——
   控制柜重启后、PC 重启前 M1 复测失败（流式塌缩中止）；
   **PC 更新网卡驱动 + 重启后，M1/M2/M3 六腿全部通过、1454 批零失败**。
   唯一变量是主机侧内核/网卡驱动重载。

## 修复动作

1. 用户更新网卡驱动：`realtek-r8126/10.017.00`（DKMS，已为
   6.8.0-111/134/136 generic 与 6.8.0-136-lowlatency 注册）；
   当时内核 `6.8.0-136-lowlatency`，`ethtool -i enp130s0` →
   `driver: r8126, version: 10.017.00-NAPI`。
   （2026-07-29 补记：现已不再使用 RT/lowlatency 内核，普通 generic
   内核直接运行，见文末"后续补记"。）
2. 重启 PC（加载新模块）。
3. governor→performance + **关闭 gro/gso/tso offload**（当时用
   `configure_realtime.sh`；该脚本已于 2026-07-29 随 `src_legacy` 删除，
   手动执行下方"持久化注意事项"里的两条命令即可，命令本身仍有效）。

## 修复后实测（2026-07-24 晚）

| 指标 | 修复前 | 修复后 |
| --- | --- | --- |
| `runtime_probe` 60s×600 | 有 5s 阻塞 | 0 失败，max 17.4ms |
| `tcp2can_probe` 30s | 1+ 失败，max ~5000ms | **0 失败，max 84.5ms** |
| TCP2CAN 写产能 | 80–125 pts/s | 158 pts/s |
| M1–M3 复测矩阵 | M1 中止 | **6/6 通过**，1454 批 0 失败 |
| 链路速率 | — | 1000Mb/s 全双工（带宽本就不是瓶颈） |

## 持久化注意事项（重要）

- **ethtool offload 关闭与 CPU governor 设置重启后均不持久**，
  每次开机必须重跑（原 `configure_realtime.sh` 已删除，直接执行）：
  ```bash
  sudo ethtool -K enp130s0 gro off gso off tso off
  echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
  ```
- 网卡驱动是 **DKMS 树外模块**：更换/升级内核后必须确认
  `dkms status` 中 r8126 对新内核编译成功，否则开机会回退到旧行为
  或网卡不可用。
- GRUB 默认内核的选择决定了加载哪份 DKMS 模块（参见
  `docs/archive/realtime_setup.md` 的 GRUB 章节——该文档已归档为历史参考，
  当前不再使用 RT/lowlatency 内核）。

## 运维排障指引（再遇类似症状时按序检查）

```bash
ethtool -i enp130s0            # 驱动应为 r8126 / 10.017.00-NAPI
ethtool -k enp130s0 | grep -E "gro|gso|tso"   # 应全 off
dkms status | grep r8126       # 当前内核必须有已编译模块
./build/diagnostics/runtime_probe 169.254.10.98 600 100   # 基线：0 失败，max <50ms
./build/diagnostics/tcp2can_probe 169.254.10.98 30 8      # 基线：0 失败，max <200ms
```

基线超标时优先：重跑上面的 ethtool/governor 命令 → 重启控制柜 →
重启 PC，再怀疑固件/SDK。

## 后续补记（2026-07-29）

- **不再使用 RT/lowlatency 内核**：RT 预检已取消（见 AGENTS.md 第 9 节），
  real 模式在普通 generic 内核直接运行；`configure_realtime.sh` /
  `realtime_preflight.sh` 已随旧 bringup 一并删除（offload/governor 命令
  仍有效，每次开机需重跑，见上节）。网卡 DKMS 模块需覆盖当前 generic 内核。
- 当日的 read() 推送超时 FAULT（offload 未关导致推送链路 >200ms 停滞）
  即为本文症状在非 RT 内核下的复现；此后 `on_error()` 已补充故障原因
  汇总日志（`aubo_e5_hardware.cpp`，read_error_reason/快照年龄/最后事件），
  此类故障不再需要翻代码推断分支。

## 后续补记（2026-07-25 凌晨）

- **SDK 库版本是第二个决定变量**：网卡修复后 v2.5.3 写延迟仍 avg ~41ms
  （产能 <200 pts/s），低速运动卡顿未根除；换回 v1.3.1 后写 avg ~5ms，
  卡顿消失。即"旧栈流畅"的经验 = 健康网卡 + v1.3.1 快写通道，两者缺一不可。
  插件已迁移 v1.3.1（详见 `docs/archive/aubo_sdk_2_5_3_ros2_control_test_report.md`
  末尾章节）。
- push+TCP2CAN 同连接 300s 并发零断链（网卡修复后复测），此前"固件
  必断链"的结论亦随之修正为网卡问题。

## 相关文件

- `docs/archive/aubo_sdk_2_5_3_ros2_control_test_report.md`（故障与复测全程记录）
- `docs/archive/realtime_setup.md`（实时权限/内核/GRUB 配置，历史参考）
- `configure_realtime.sh`、`realtime_preflight.sh` 已随旧 bringup 删除
  （offload/governor 命令仍有效，见上文）
- 探针：`diagnostics/aubo_sdk_runtime_probe.cpp`、`diagnostics/aubo_sdk_tcp2can_probe.cpp`
