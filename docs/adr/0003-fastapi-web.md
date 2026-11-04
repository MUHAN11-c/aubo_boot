# ADR 0003：Web 后端使用 FastAPI，不用 rosbridge 作控制面

## 决策

`peach_observability` 使用 FastAPI + uvicorn；`peach_web` 只放静态资源；调试/运动 API 保持 allowlist 和 backend 授权门。

## 理由

- `rosbridge_suite`/`foxglove_bridge` 暴露任意 ROS 图，不适合作为操作员控制面。
- FastAPI/Pydantic/OpenAPI 为主流 Web 栈；保留同源、CSP、64 KB body 上限和 HTTP 状态语义。
- `peach_web` 不 import ROS，前后端边界清晰。

## 后果

- 8090 为唯一 Web 主机；8099/8100 已合并。
- FastAPI 假 backend 与独立节点已冒烟；浏览器手工验收仍需保留。
