# peach_observability

只读监控：HTTP 状态页 + JSONL 落盘。没有控制、没有改参、没有运动入口。

## 流程

订阅感知 / 重建 / 技能 status / 执行器 state+events / 机械臂 `RobotStatus`，聚合成 `/api/state` 与事件时间线。可选把调试图和 TSDF 点云随关键事件写入 `web_runs/`。

执行器事件类型是 `CanonicalEvent`（`/peach_task_executor/events`）。

## 启动

```bash
ros2 launch peach_observability observability.launch.py
```

整栈默认带上。参数：`config/web.yaml`。

```bash
# 覆盖监听（空 host / port 0 表示用 yaml）
ros2 launch peach_observability observability.launch.py host:=0.0.0.0 port:=8090
```

浏览器：`http://127.0.0.1:8090`。默认 `host: 127.0.0.1`，局域网需显式 `0.0.0.0`。

## 记录

`record.root_dir: web_runs`（相对节点 CWD）。历史过程数据在 `_archive/runs/`，不要删。

## 不负责

开批、选目标、SetIO、改各节点参数。
