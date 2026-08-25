# peach_navigation

五个能力包之一：**作业位导航适配**。一节点。调度层是唯一客户端。详细作用见 [docs/architecture.md](../../docs/architecture.md) §3 `peach_navigation`。

现行 `impl: reserved_stub`：不接雷达、不发 `cmd_vel`、不调发行版 Nav2，把固定座当作已到位。SLAM / AMCL / 代价地图预留，有真底盘后再接线。

总览：[docs/architecture.md](../../docs/architecture.md)。契约：[docs/io.md](../../docs/io.md)。

## 从哪读

| 文件 | 职责 |
|------|------|
| `peach_navigation/navigation_node.py` | Lifecycle + `NavigateToWorksite` |
| `config/navigation.yaml` | `impl` / `worksite_frame` |
| `launch/navigation.launch.py` | 单独或整栈 include |

## 不负责

选下一颗、视觉、写账本、`RunHarvest`、臂规划、底盘硬件驱动。

## 启动

```bash
ros2 launch peach_navigation navigation.launch.py
```

整栈里 `autostart:=false`，由 lifecycle manager 转换。执行器默认 `navigation_enabled=false`，开批时不发送本动作。
