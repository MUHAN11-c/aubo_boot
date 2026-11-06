# Web 后端

`peach_observability` 使用 FastAPI + uvicorn 提供同源 HTTP API；`peach_web` 只提供静态前端资源。`/` 是系统总览，`/monitor.html` 是过程监控。

## API

- `/api/state`、`/api/params`、`/api/trajectory`
- `/api/analysis*`
- `/api/images/{perception,raw}.jpg`
- `/api/force`
- `/force.html`（完整 CAD 工具视图作背景，盘面/整机切换；叠加五点、tcp 三轴与 CoP；`/api/force`）
- `POST /api/params`
- `POST /api/debug/<allowlisted-action>`

## 安全边界

- 同源、CSP、`X-Frame-Options: DENY`、`X-Content-Type-Options: nosniff`；
- 调试/运动门控仍在 backend，HTTP 层只解析/转发；
- 请求体 64 KB 上限；不自动重试 POST；
- 整栈默认经 `ros2_web_link`/ROSbridge 原生 ROS 服务与动作通信；桥只监听回环并限制服务/动作 allowlist、禁止 topic publish。HTTP 调试门和 debug_audit 不覆盖直接 ROSbridge 调用；最终授权由 ROS 节点执行。独立 observability 或显式对照可使用 HTTP。

## 历史验证（2026-10-27）

- FastAPI 假 backend smoke：`/api/state`、`/api/trajectory`、静态 `/`、`POST /api/debug/test` 全部通过。
- 独立 observability 节点 8098：`/api/state` 200、`/api/params` 200、`/` 返回 HTML。
- 构建与官方 lint 通过。

长动作经 FastAPI `run_in_threadpool` 执行，动作期间 `/api/state` 等只读接口仍可用。

## 页面

- `/overview.html`：能力链路、批次、相机、账本。
- `/monitor.html`：画面/轨迹/事件/日志/参数。
- `/debug.html`：开批/使能/单目标/参数/ACK。
- `/analysis.html`：会话 bag、成功率、接触段。
- `/force.html`：完整 CAD 工具视图 + 五点/tcp/CoP。

当前传输默认值由 `GET /api/transport` 和启动事实给出；力页只读 `/api/force` 或对应 `web_force`，没有旧独立力页的录制/分段命令。完整路由和授权范围见 [observability IO](../../src/peach/peach_observability/doc/io.md)。
