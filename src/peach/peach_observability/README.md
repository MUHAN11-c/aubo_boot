# peach_observability

观测为主：8090 Web（过程页只读）+ 调试 POST 桥 + 会话 bag + `peach_bag_report`。实现模块与参数 yaml（`config/observability.yaml`，W11 起随包走）都在本包，节点 `ObservabilityParams.attach`。删除本包后核心调度仍可构建，但 8090 不起。

**调试面不是纯只读**：`debug.enabled`（默认 true）放行 `POST /api/debug/<action>` 转发 18 个既有动作/服务端点（含 RunHarvest/Survey/Execute——运动类另需 `debug.motion_enabled`，默认 false 拦 423；每次操作含被拒审计落 `runs/debug_audit/`）。话题/端点/参数契约见 [doc/io.md](doc/io.md) 与 [doc/architecture.md](doc/architecture.md)。

## 公有入口

```bash
ros2 launch peach_observability observability.launch.py record_bag:=false
ros2 run peach_observability peach_bag_report runs/session_*/bag
```

默认 `record_bag:=false`。有界关闭走 bag 进程 SIGINT，不经 Web。图名仍是 `peach_observability`。会话 bag 随栈启停（`runs/session_*/bag/`，MCAP 限速进袋）并有体积预算回收（`record.max_total_bag_gb` 默认 100 GB，最旧先删、报告/账本永不删、逐条审计；分析后清理用 `peach_observability.retention.purge_analyzed_sessions` Python API）。Web 静态页资源在顶层资源包 `peach_web`（`share/peach_web/web`，本包 HTTP 层托管），另发 `/peach/observability/{tcp_path,markers}` RViz 可视化。

## 构建 / 测试 / 许可

`colcon build --packages-select peach_observability`。`colcon test --packages-select peach_observability`（`test_bag_report` 等纯核）。BSD-3-Clause。
