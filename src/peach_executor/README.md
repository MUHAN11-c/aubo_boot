# peach_executor

四个能力包之一：**整栈调度**。三节点同包：`peach_executor`（开批/选果/FSM/账本）、`peach_lifecycle_manager`（四节点名单）、`peach_observability`（只读监控 Web + 鉴权手动调试操作面，默认三重关）。批次唯一所有者；**launch 绝不自动开批**；导航已归档，到位一步直通 `NAV_OK`。

现行设计、文件树与「从哪读源码」（先 `harvest_fsm.react` 表，再 `_run_harvest` 的 Command 分支）：[docs/architecture.md](../../docs/architecture.md) §3。跨包调用与话题契约：[docs/io.md](../../docs/io.md) §5。怎么跑与验收门：[docs/testing.md](../../docs/testing.md)。

```bash
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false   # 真机显式 real
```

`execution_enabled` 默认 false；`execute_pregrasp_only` 默认 true（停预抓取等 ACK）。
