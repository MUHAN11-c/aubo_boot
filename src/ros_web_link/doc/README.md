# ros_web_link 文档索引

| 文档 | 内容 |
|---|---|
| [install.md](install.md) | 独立安装、依赖边界、最小验证、安全注意 |
| [api/README.md](api/README.md) | API 总览、版本与稳定性 |
| [api/python.md](api/python.md) | Python 网关/客户端 API |
| [api/js.md](api/js.md) | 浏览器 JS API（稳定 barrel + 门面 + action） |
| [api/http-ws.md](api/http-ws.md) | HTTP 端点、WebSocket 帧、错误与背压 |
| [api/launch.md](api/launch.md) | launch 入口、参数、config 键 |
| [protocol.md](protocol.md) | Foxglove v1 ↔ rosbridge v2.1 能力对照与迁移 |
| [architecture.md](architecture.md) | 分层、职责边界、决策记录 |
| [io.md](io.md) | 端点/话题/进程速查 |
| [testing.md](testing.md) | 测试命令、E2E 编排、验收口径 |

示例代码在 [`../examples/`](../examples/README.md)，浏览器示例在
`web/public/examples/minimal_client.html`（网关下 `/examples/minimal_client.html`）。

> 本包是**独立 ROS 2 包**：core 只依赖本包 + `ivg_foxglove_bridge` + ROS 基础包 +
> Python 网关依赖。UI 组合包（如 `aubo_ros2_web_dashboard`）与官方仿真 demo
> 都是 optional，不是运行前提。
