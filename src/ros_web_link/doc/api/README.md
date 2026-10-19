# API 总览

- 包版本：`1.1.0`（`package.xml` / `setup.py` / `ros_web_link.__version__` 同源）
- Wire 协议：Foxglove WebSocket **v1**（CDR 二进制），经 `ivg_foxglove_bridge`
- 稳定性：1.x 只做 additive；wire 路径/帧格式冻结；破坏性变更走 major

| 面 | 入口 | 文档 |
|---|---|---|
| Python | `ros_web_link.app.create_app`、`ros_web_link.foxglove_ws_client.FoxgloveWsClient` | [python.md](python.md) |
| 浏览器 JS | `/js/ros_web_link.js`（稳定 barrel） | [js.md](js.md) |
| HTTP/WS | `:8095`，`/ws/foxglove` | [http-ws.md](http-ws.md) |
| launch/config | `monitor.launch.py`、`transport.launch.xml` | [launch.md](launch.md) |
| 协议对照/迁移 | Foxglove v1 ↔ rosbridge v2.1 | [protocol.md](../protocol.md) |

## 三条推荐调用路径

1. **只监控**：`ros2 launch ros_web_link monitor.launch.py` → 浏览器
   `http://127.0.0.1:8095/`。
2. **嵌进自己的 Python 服务**：`create_app()` + `uvicorn`（见
   [`examples/python/04_embed_gateway.py`](../../examples/python/04_embed_gateway.py)）。
3. **嵌进自己的 Web UI**：`import { foxgloveRosClient } from '/js/ros_web_link.js'`
   （见 [`minimal_client.html`](../../web/public/examples/minimal_client.html)）。

## 不在 API 内

- 没有 rosbridge JSON 端点（`/ws/rosbridge` 不存在，测试有负例）；rosbridge
  客户端不能直连，需按 [protocol.md](../protocol.md) 迁移。
- 没有显式 QoS/throttle/fragment 字段；QoS 跟随图上发布者，慢消费者由网关
  有界通道丢最旧/年龄门处理。
- Python 高层 ActionClient 不在 1.x API 内；Action 高层 API 是 JS
  `createActionClient()`，Python 可用 `call_service` 组合三个 action 服务。
