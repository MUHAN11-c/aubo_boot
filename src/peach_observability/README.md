# peach_observability

只读监控：HTTP 状态页 + JSONL 落盘。没有控制、没有改参、没有运动入口。

总览：[docs/flow.md](../../docs/flow.md)。参数：`config/web.yaml`。

## 从哪读

| 文件 | 职责 |
|------|------|
| `peach_observability/gateway.py` | 订阅聚合、HTTP `/api/state`、JSONL |
| `launch/observability.launch.py` | 节点启动 |
| 前端（包内 web） | 按 `HarvestState.batch_state` 渲染；事件码须与执行器词表一致 |

## 谁调谁

只订阅，不调用采摘动作。来源：感知 `/peach/perception/*`、重建 `/peach/reconstruction/*`、技能 `~/status` 与 `grasp_hypothesis`、执行器 `~/state` + `~/events`、机械臂 `RobotStatus`。

执行器事件类型是 `CanonicalEvent`。记录器按 `batch_state` 开关 `web_runs/run_*`；事件码应对齐 `target_dispatched` / `target_succeeded` / `target_skipped` / `target_failed` / `target_canceled` / `target_operator_skipped` / `round_locked`。

整栈 `record_mcap:=true` 时额外 `ros2 bag record -s mcap`（在 `harvest_system.launch.py`，不在本包内发运动）。

## 启动

```bash
ros2 launch peach_observability observability.launch.py
ros2 launch peach_observability observability.launch.py host:=0.0.0.0 port:=8090
```

浏览器：`http://127.0.0.1:8090`。默认 `host: 127.0.0.1`。

`record.root_dir: web_runs`（相对节点 CWD）。不要删 `_archive/runs/` 或现场 `web_runs/`。
