# ros2_control 实时调度（SCHED_FIFO）与 lowlatency 内核配置记录

> **历史参考（2026-07-29 起）**：已取消 RT 要求，不再使用 RT/lowlatency
> 内核，real 模式在普通 generic 内核直接运行（见 `AGENTS.md` 第 9 节）。
> 本文的 SCHED_FIFO/limits/GRUB 内容仅供回溯；**每次开机仍需执行的只有
> 网卡 offload 关闭与 governor 设置**，命令见
> `docs/nic_driver_incident.md` 的"持久化注意事项"一节。

记录日期：2026-07-23。目标机器：`mu-MS-7E34`，Ubuntu 24.04（noble），ROS 2 Jazzy。

## 目标与依据

按 [ros2_control controller_manager 官方文档 Determinism 一节](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)
为控制回路启用实时调度：

- `ros2_control_node` 主线程默认尝试 `SCHED_FIFO` 优先级 50（`thread_priority` 参数，0–99）；
- 普通用户默认无实时优先级权限，需要 `realtime` 用户组 + PAM limits 授权；
- `lock_memory` 参数锁定节点内存，避免页错误与换出（依赖 memlock 限制）；
- 内核方面，文档建议 PREEMPT_RT 或 lowlatency 以降低抖动。

## 配置前的环境检查

| 项目 | 检查结果 |
| --- | --- |
| 内核 | `6.8.0-136-generic`（PREEMPT_DYNAMIC，非 PREEMPT_RT） |
| 用户实时权限 | `ulimit -r` 为 0，`chrt -f` 直接报"不允许的操作" |
| 显卡 | RTX 3090，NVIDIA 驱动 595，**非 DKMS**（依赖按内核预编译的 `linux-modules-nvidia-595-*` 包） |
| 网卡 | Realtek r8125/r8126，**DKMS** 驱动（机器人通信依赖，换内核需重编译）。**2026-07-24 更新**：已升级 r8125/9.011.00 → r8126/10.017.00-NAPI，该驱动/offload 状态是 SDK 通道秒级停滞的主因，详见 `docs/nic_driver_incident.md` |
| 可用 RT 内核 | `linux-image-realtime` 6.8.1（universe）、`linux-lowlatency` 6.8.0-134、Ubuntu Pro `realtime-kernel` 服务 |
| Jazzy 二进制参数 | `strings ros2_control_node` 确认支持 `thread_priority`、`lock_memory`；`cpu_affinity` 在此版本不存在 |

关键约束：`linux-modules-nvidia-595-lowlatency` 有官方候选包，而 **realtime（PREEMPT_RT）内核没有对应的 NVIDIA 595 预编译模块**——装 PREEMPT_RT 会导致 3090 驱动无法加载、CUDA 不可用、显示回落到核显。

## 方案选择

比较了三个方案：

1. **lowlatency 内核（采用）**：抖动明显改善，NVIDIA 595 有官方预编译模块，CUDA 保留，风险最低；
2. PREEMPT_RT 内核：抖动最小，但丢失 NVIDIA 驱动/CUDA；
3. 不换内核只配 FIFO 权限：零风险，但抖动改善有限。

先完成了方案 3（FIFO 权限），随后按用户决定加装方案 1 的内核。两步互相独立，权限配置对任何内核都生效。

## 工作区改动

`src/aubo_e5_bringup/config/controllers.yaml` 的 `controller_manager` 段新增：

```yaml
    thread_priority: 50
    lock_memory: true
```

说明：50 是文档默认值，显式写出以便查阅；若宿主机未配 rtprio/memlock 权限，节点只会记录警告并回退到普通调度，不会启动失败。

## 宿主机配置（已执行）

### 1. 实时权限

```bash
sudo addgroup realtime
sudo usermod -a -G realtime mu
```

向 `/etc/security/limits.conf` 追加（文档原文）：

```text
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```

新登录会话验证通过：`ulimit -r` = 99，`ulimit -l` = unlimited，`chrt -f 10 true` 成功。
注意：**配置前已打开的终端仍带旧限制**，必须注销重新登录。

### 2. lowlatency 内核

```bash
sudo apt update
sudo apt install -y linux-lowlatency linux-modules-nvidia-595-lowlatency
```

安装后确认：

- `linux-image-6.8.0-134-lowlatency` 及 `linux-modules-nvidia-595-6.8.0-134-lowlatency` 均已安装；
- `dkms status` 显示 `r8125/9.011.00, 6.8.0-134-lowlatency, x86_64: installed`（网卡驱动自动重编译成功）。

### 3. GRUB 默认项（一个坑）

lowlatency 版本号 `6.8.0-134` **小于**已有的 generic `6.8.0-136`，GRUB 按版本号排序，默认仍会从 generic 启动。已将 `/etc/default/grub` 改为（menuentry id 形式，比标题匹配更稳定）：

```text
GRUB_DEFAULT="gnulinux-advanced-05732991-0912-4703-815e-45d2d83591fc>gnulinux-6.8.0-134-lowlatency-advanced-05732991-0912-4703-815e-45d2d83591fc"
```

并执行了 `sudo update-grub`。该 UUID 是这台机器根分区的，换机器需从 `/boot/grub/grub.cfg` 重新抄。

## 重启后的验证清单

```bash
uname -a                        # 应含 6.8.0-134-lowlatency
ulimit -r                       # 99
ulimit -l                       # unlimited
nvidia-smi                      # RTX 3090 正常
ip link                         # r8125 网卡存在
```

启动 bringup 后再确认：

- `ros2_control_node` 日志中**没有** `Could not enable FIFO RT scheduling policy` 警告；
- `chrt -p $(pgrep -f ros2_control_node)` 显示主线程为 `SCHED_FIFO` 优先级 50；
- 观察 100 Hz 控制循环是否出现 overrun 警告；频繁超时再考虑 PREEMPT_RT。

## 回退方法

- 内核：开机按 Esc 进 GRUB 菜单（当前 `GRUB_TIMEOUT_STYLE=hidden`），Advanced options 里选回 `6.8.0-136-generic`；彻底回退则恢复 `GRUB_DEFAULT=0` 并 `sudo update-grub`；
- 权限：删除 `/etc/security/limits.conf` 末尾 6 行 `@realtime` 记录，重新登录；
- 参数：移除 `controllers.yaml` 中的 `thread_priority` / `lock_memory`（移除后行为同默认值）。

## 参考

- [ros2_control controller_manager userdoc — Determinism](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)
- 本工作区 `src/aubo_e5_bringup/config/controllers.yaml`、`docs/real_hardware_integration_notes.md`
