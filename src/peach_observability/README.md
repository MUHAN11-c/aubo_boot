# peach_observability

只读观测：8090/JSONL、会话 bag、`peach_bag_report`。实现模块在本包；参数 yaml 在 `peach_harvester/config/observability.yaml`，节点 `ObservabilityParams.attach`。删除本包后核心调度仍可构建，但 8090 不起。

## 公有入口

```bash
ros2 launch peach_observability observability.launch.py record_bag:=false
ros2 run peach_observability peach_bag_report runs/session_*/bag
```

默认 `record_bag:=false`。有界关闭走 bag 进程 SIGINT，不经 Web。图名仍是 `peach_observability`。

## 构建 / 测试 / 许可

`colcon build --packages-select peach_observability`。BSD-3-Clause。
