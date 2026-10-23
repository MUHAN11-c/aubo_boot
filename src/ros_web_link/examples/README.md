# ros_web_link 调用示例

> **2026-10-23 迁移说明**：`src/examples` 与 `external/` 已迁到 `/home/mu/demo_ws`。下文 `src/examples/...` 指 `/home/mu/demo_ws/src/examples/...`，`external/...` 指 `/home/mu/demo_ws/external/...`；运行 `sim_demo.launch.py` 前先 `source /home/mu/demo_ws/install/setup.bash`（或确认 apt 包）。

本目录是**可直接运行**的集成示例，不是单元测试夹具。按 [API 文档](../doc/api/README.md)
的约定使用稳定入口：

- Python：`ros_web_link.app.create_app` / `ros_web_link.foxglove_ws_client.FoxgloveWsClient`
- Web：`/js/ros_web_link.js`（浏览器稳定 barrel）
- HTTP/WS：网关 `:8095`，WebSocket 路径 `/ws/foxglove`

> 所有示例只依赖本包 + `ivg_foxglove_bridge` + ROS 基础消息包。
> 完整仿真演示（TB3/Panda）是 optional demo；本地 peach 项目不在示例/测试范围内。

## 1. 自包含 fixture 栈（推荐先跑通）

```bash
# 终端 A
ros2 launch ros_web_link transport.launch.xml fixture:=true

# 终端 B（工作区根）
source install/setup.bash
python3 examples/python/01_subscribe.py --count 3
python3 examples/python/02_publish_and_service.py
python3 examples/python/03_params_and_graph.py
python3 examples/python/05_raw_protocol.py
```

`transport.launch.xml` 默认桥 8765；若被占用可显式改端口：

```bash
ros2 launch ros_web_link transport.launch.xml port:=18766 gateway_port:=8095 fixture:=true
python3 examples/python/01_subscribe.py --url ws://127.0.0.1:8095/ws/foxglove
```

## 2. 附着到运行中的仿真/真机（独立监控）

```bash
# 终端 A：正常启动完整官方仿真（不带传输栈；数据面只用 src/examples 官方项目）
ros2 launch ros_web_link sim_demo.launch.py transport:=false

# 终端 B：独立监控（桥+网关）
ros2 launch ros_web_link monitor.launch.py

# 终端 C：示例
python3 examples/python/01_subscribe.py --topic /joint_states \
  --type sensor_msgs/msg/JointState --count 2
python3 examples/python/03_params_and_graph.py --node /amcl
```

浏览器打开 <http://127.0.0.1:8095/>，或直接打开示例页
<http://127.0.0.1:8095/examples/minimal_client.html>。

## 3. 示例矩阵

| 示例 | 数据源 | 展示的 API | 预期 |
|---|---|---|---|
| [`python/01_subscribe.py`](python/01_subscribe.py) | fixture / 任意图 | `connect` → `wait_channel` → `subscribe` → `next` | 打印消息；`--json` 输出 JSON |
| [`python/02_publish_and_service.py`](python/02_publish_and_service.py) | fixture | `publish` + `callService` | echo 回包 + AddTwoInts sum=42 |
| [`python/03_params_and_graph.py`](python/03_params_and_graph.py) | fixture | `getParams`/`setParam`、`connectionGraph` | 参数读写 + 图计数 |
| [`python/04_embed_gateway.py`](python/04_embed_gateway.py) | 无 ROS 依赖 | `create_app()` + uvicorn | 启动网关/console，可 `--open` |
| [`python/05_raw_protocol.py`](python/05_raw_protocol.py) | 任意 Foxglove v1 桥/网关 | 原始 `serverInfo` 帧 | subprotocol/capabilities/channels |
| [`web/public/examples/minimal_client.html`](../web/public/examples/minimal_client.html) | fixture / 仿真 | JS barrel：subscribe/publish/service/params/action | 浏览器交互日志 |
| [`http/README.md`](http/README.md) | 网关 | `/health`、`/api/ivg/ws/stats`、mesh、video | curl 响应 |
| [`launch/README.md`](launch/README.md) | 无 | monitor/transport/demo launch | 启动矩阵 |

## 4. 参数与错误约定

- `--url` 默认 `ws://127.0.0.1:8095/ws/foxglove`。
- `--json` 只输出一行 JSON，便于 shell/CI 断言；失败写 stderr 并返回非零。
- 话题订阅前必须已有发布者（桥只 advertise 图上已有 channel）；否则 `--timeout` 后退出。
- 服务/参数依赖标准 ROS 2 服务可达；未知节点返回 `FoxgloveClientError`。
- Python 示例使用 `rclpy` 消息类；消息类型用 `pkg/msg/Type` 字符串指定。

## 5. 相关文档

- [API 索引](../doc/api/README.md)
- [Python API](../doc/api/python.md)
- [JS API](../doc/api/js.md)
- [HTTP/WS API](../doc/api/http-ws.md)
- [Launch/配置](../doc/api/launch.md)
- [Foxglove/rosbridge 对照](../doc/protocol.md)
- [安装](../doc/install.md)
