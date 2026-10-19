# Python API

## 导入路径

```python
from ros_web_link.app import create_app
from ros_web_link import config
from ros_web_link.cli import main as gateway_main
from ros_web_link.foxglove_ws_client import (
    FoxgloveWsClient, FoxgloveClientError, Subscription, SUBPROTOCOL,
)
from ros_web_link.comm_fixture import main as fixture_main
```

`ros_web_link/__init__.py` 只保留轻量 `__version__`，避免 import 包时拉起 FastAPI。

## `create_app`

```python
create_app(
    js_roots: list[str] | None = None,
    html_root: str | None = None,
    sub_routers: list | None = None,
) -> fastapi.FastAPI
```

| 参数 | 语义 |
|---|---|
| `js_roots` | 追加的 JS 静态根（按序；本包通信层 JS 永远在最前），挂到 `/js/` |
| `html_root` | 组合模式 UI 根：UI 独占 `/`，本包 console 挂 `/rwl/` |
| `sub_routers` | UI 层 BFF `APIRouter` 列表，优先于静态挂载 |

- 纯传输模式（`html_root=None`）：本包 `web/public` 挂 `/`，`/` 307 → `/console.html`。
- 组合模式：UI 根挂 `/`，本包 console 在 `/rwl/console.html`。
- `app.state.static_root` 记录当前 HTML 根（health 返回）。
- lifespan 结束时关闭视频代理共享 httpx client。

示例见 [`examples/python/04_embed_gateway.py`](../../examples/python/04_embed_gateway.py)。

## CLI / `python -m ros_web_link`

```bash
ros_web_link_gateway [port] [options]
python -m ros_web_link [port] [options]
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | 8095 | 网关端口（位置参数） |
| `--bind/-b` | 0.0.0.0 | 监听地址 |
| `--directory/-d` | None | UI HTML 根（组合模式） |
| `--js-directory/-j` | [] | UI JS 根，可重复 |
| `--foxglove-bridge-host` | 127.0.0.1 | 上游桥 |
| `--foxglove-bridge-port` | 8765 | 上游桥端口 |
| `--foxglove-bridge-ws-path` | `/` | 上游 WS 路径 |
| `--foxglove-bridge-compression` | false | 上游 deflate |
| `--foxglove-bridge-max-msg-bytes` | 128MiB | WS 帧上限 |
| `--foxglove-bridge-ping-interval/timeout` | 20/60 | 上游心跳 |
| `--foxglove-bridge-close-timeout` | 10 | 上游关闭等待 |
| `--ws-ping-interval/timeout` | 20/20 | 客户端腿心跳；`-1` 关闭心跳（不建议） |
| `--ws-permessage-deflate` | false | 客户端腿 deflate |
| `--upstream-max-queue` | 4 | 上游接收队列（条数） |
| `--data-lane-bytes` | 4MiB | 出站数据通道字节预算；0=无界 |

启动前会做端口占用预检，失败以明确错误退出。配置优先级：`config/defaults.yaml`
→ `config.init()`/CLI 覆盖；没有环境变量覆盖层。

## `config`

```python
config.init({'port': 8096, 'foxglove_bridge_port': 18766})
config.gateway_port()        # 8096
```

访问器为类型安全 getter；YAML 缺失时回退硬编码默认值。完整键见
[launch.md](launch.md#config-键) 与 `config/defaults.yaml`。

## `FoxgloveWsClient`

单连接、asyncio、foxglove v1 客户端；URL 可指向网关（`ws://host:8095/ws/foxglove`）
或直连桥。消息类型用 `rclpy` 消息类；CDR 编解码走 `rclpy.serialization`。

| 方法/属性 | 签名 | 说明 |
|---|---|---|
| `connect` | `async (url, timeout=10.0) -> dict` | 连接并等 `serverInfo`；返回 serverInfo |
| `close` | `async ()` | 关闭连接/读循环 |
| `is_open` | `bool` | 连接是否仍开 |
| `wait_channel` | `async (topic, timeout=10.0) -> dict` | 等桥 advertise 该 channel |
| `subscribe` | `async (topic, msg_type, timeout=10.0) -> Subscription` | 订阅；先 wait_channel |
| `Subscription.next` | `async (timeout=5.0) -> msg` | 取下一条并解码；超时 `asyncio.TimeoutError` |
| `Subscription.cancel` | `async ()` | 退订 |
| `advertise` | `async (topic, schema_name) -> int` | 客户端发布 channel（CDR） |
| `publish` | `async (topic, schema_name, message)` | 发布 rclpy 消息；自动 advertise |
| `wait_service` | `async (name, timeout=10.0) -> dict` | 等服务 advertise |
| `call_service` | `async (name, request, response_type, timeout=15.0) -> response` | 调用并解码 |
| `get_params` | `async (node, names=(), timeout=10.0) -> dict` | 标准 `~/get_parameters`；空 names=全部，键为 `#0…`（响应不含名字） |
| `set_param` | `async (node, name, value, timeout=10.0) -> result` | 标准 `~/set_parameters`；失败抛异常 |
| `subscribe_connection_graph` | `async ()` | 订阅图更新 |
| `graph_snapshot` | `dict` | `{topics, services, nodes}` |
| `last_status` / `status_history` | dict/list | 桥端 status 错误通告 |

### 错误与约束

- 协议/连接/参数写失败：`FoxgloveClientError`。
- 等 channel/服务/消息超时：`asyncio.TimeoutError`。
- 单连接绑定一个 asyncio event loop；不要跨 loop 复用。
- `subscribe` 要求图上已有发布者（桥只 advertise 已有 channel）；晚连接场景先
  `call_service` 触发发布者，或等基础设施就绪。
- `publish` 的 `schema_name` 必须是 `pkg/msg/Type`；QoS 跟随图上已有发布者。
- Python client 定位**测试/诊断**（对齐 roslibpy 定位）；生产浏览器端用 JS 门面。
- 高层 Action API 不在 Python 1.x 内；用 `call_service` 组合 send_goal/cancel/get_result，
  或使用 JS `createActionClient`。

### 参数值类型

`get_params` 返回 Python 原生值（bool/int/float/str/bytes/list）；`set_param` 按值推断
`ParameterValue.type`。空数组 → `PARAMETER_NOT_SET`，与 JS 门面一致。

## `comm_fixture`（演示/测试被测端）

```bash
ros2 run ros_web_link rwl_comm_fixture
ros2 run ros_web_link rwl_comm_fixture --ros-args -p rwl.bigdata:=true
```

| 名字 | 类型 | 说明 |
|---|---|---|
| `/rwl_demo/chatter` | `std_msgs/String` @5Hz | 订阅面 |
| `/rwl_demo/echo` → `/rwl_demo/echo_reply` | `String` | 发布回环 |
| `/rwl_demo/add_two_ints` | `example_interfaces/AddTwoInts` | 服务 |
| `/rwl_demo/fibonacci` | `example_interfaces/Fibonacci` | Action |
| `/comm_fixture` 参数 | `rwl.answer`(42)、`rwl.step_s`(0.15)、`rwl.bigdata`(false) | 参数面 |
| `/rwl_demo/big_image` / `big_cloud` | Image/PointCloud2 | `rwl.bigdata:=true` 时 |

## 示例

- [`01_subscribe.py`](../../examples/python/01_subscribe.py)
- [`02_publish_and_service.py`](../../examples/python/02_publish_and_service.py)
- [`03_params_and_graph.py`](../../examples/python/03_params_and_graph.py)
- [`04_embed_gateway.py`](../../examples/python/04_embed_gateway.py)
