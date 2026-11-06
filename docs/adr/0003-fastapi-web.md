# ADR 0003：同源 FastAPI 与受限 ROSbridge 分工

## 决策

2026-11-06 按当前源码更新：`peach_observability` 使用 FastAPI + uvicorn 托管同源资源、图像、历史分析及 HTTP API；`peach_web` 是静态资源包。整栈默认同时启动 `ros2_web_link` 的回环 ROSbridge，页面通过 SDK 调用原生 ROS 服务/动作并订阅三类聚合状态；HTTP 保留独立部署和对照入口。

## 理由

- 应用只需固定 ROS 契约；`harvest_system._web_bridge` 禁止 topic publish，限制服务、动作和订阅 allowlist，不依赖任意图发现。
- FastAPI/Pydantic 处理 HTTP 结构与状态；实际应用关闭 Swagger/ReDoc 页面，保留默认 OpenAPI JSON，保留同源、CSP 与 64 KB 请求体上限。
- `peach_web` 不 import ROS，前后端边界清晰。

## 后果

- 8090 为页面与 HTTP 主机，ROSbridge 默认 `127.0.0.1:9090`；旧 8099/8100 力页已合并。
- HTTP 的 `debug.enabled` / `debug.motion_enabled` 与 `debug_audit` 只覆盖 HTTP；直接 ROSbridge 调用由 ROS 节点自身门控，最终运动/工具授权仍在臂侧。不能把浏览器按钮限制或 HTTP 门写成 WS 服务端授权。
- FastAPI 假 backend 与独立节点已冒烟；浏览器手工验收仍需保留。
