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
- 不使用 rosbridge/foxglove 作为控制面。

## 验证

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
