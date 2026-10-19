# 浏览器 JS API

零构建 ES Modules。应用侧只依赖稳定 barrel：

```html
<script type="module">
  import { foxgloveRosClient, createActionClient } from '/js/ros_web_link.js';
</script>
```

高级用法可从 `/js/transport/foxglove/index.js` 或具体模块导入
`FoxglovePlayer`、`FoxgloveProtocolClient`、`CodecCache`、`buildCodec`。

## `foxgloveRosClient()` 单例

```js
const ros = foxgloveRosClient();
await ros.connect();                    // 默认 ws://<host>/ws/foxglove
// 或 ros.connect('ws://127.0.0.1:8095/ws/foxglove')
ros.disconnect();
```

| 成员 | 签名/返回 | 说明 |
|---|---|---|
| `connect` | `async (urlOverride?)` | 幂等；已连接时复用；失败进入重连状态机 |
| `disconnect` | `void` | 停止 player、关闭 socket |
| `isConnected` | `boolean` | 当前连接状态 |
| `state` | `string` | 连接状态机状态 |
| `stats` | `object` | sessionId/messages/bytes/reconnects/serviceCalls… |
| `onStateChange` | `(fn) => unsubscribe` | 连接状态变化 |
| `onStatus` | `(fn) => unsubscribe` | 桥端 status/错误通告 |
| `subscribe` | `(topic, msgType, cb, opts?) => handle` | 订阅；`handle.cancel()` 退订 |
| `publish` | `(topic, msgType, message) => boolean` | 发布；自动 advertise；类型仅校验提示 |
| `callService` | `async (name, request, opts?) => response` | `opts.timeoutMs` |
| `getParams` | `async (node, names=[]) => object` | 空 names = 全部，键为 `#0…`（响应不含名字） |
| `setParam` | `async (node, name, value) => {successful, reason}` | 标准 SetParameters |
| `getGraph` | `() => graph` | 首次调用订阅 connectionGraph，之后持续更新 |

### 订阅语义

```js
const handle = ros.subscribe('/joint_states', 'sensor_msgs/msg/JointState',
  (msg) => {
    // msg = { topic, type, data, timestampMs, bridge, raw? }
    console.log(msg.data.name, msg.data.position);
  },
  { dispatchMode: 'latest' });
// 用完
handle.cancel();
```

- `dispatchMode`：`immediate`（同步，低频控制面）/ `latest`（微任务合并，状态/图像/点云）
  / `raw`（同 latest 合并但**不解码**，回调拿 `data=Uint8Array, raw=true`）。
- 订阅前桥必须已 advertise 该 channel；`subscribe` 会提示类型不一致，但以图上
  schema 为准。
- `msgType` 是交叉校验/文档用；CDR 解码由 channel schema 驱动。
- 大消息（图像/点云）用 `latest` 或 `raw`，避免主线程解码浪费。

### 发布/服务/参数

```js
ros.publish('/cmd_vel', 'geometry_msgs/msg/Twist',
  { linear: { x: 0.2, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });

const resp = await ros.callService('/rwl_demo/add_two_ints',
  { a: 20, b: 22 }, { timeoutMs: 8000 });
// example_interfaces/AddTwoInts int64 可能为 BigInt → Number(resp.sum)

const params = await ros.getParams('/comm_fixture', ['rwl.answer']);
const r = await ros.setParam('/comm_fixture', 'rwl.answer', 43);
```

## `createActionClient(name, opts?)`

foxglove v1 无 action 原生帧；本模块在 `ros_client` 上组合
`send_goal/cancel_goal/get_result` 服务与 `status/feedback` 话题：

```js
import { createActionClient, GOAL_STATUS } from '/js/ros_web_link.js';

const ac = createActionClient('/rwl_demo/fibonacci');
const { status, statusText, result } = await ac.sendGoal(
  { order: 8 },
  { onFeedback: (fb) => console.log('feedback', fb.sequence),
    resultTimeoutMs: 30000 });
console.log(statusText, result);
ac.dispose();
```

| 方法 | 说明 |
|---|---|
| `sendGoal(goal, {onFeedback, resultTimeoutMs})` | 发 goal → 等 get_result；返回 status/result |
| `cancelGoal(goalId)` | 精确取消；`goalId` 为 16 字节 UUID |
| `cancelAll()` | 零 UUID 批量取消；**rclpy action server 上可能死锁，长期运行避免** |
| `dispose()` | 取消 feedback/status 订阅 |
| `getStatusSnapshot()` | 最近 status 数组 |

`GOAL_STATUS`：UNKNOWN 0 / ACCEPTED 1 / EXECUTING 2 / CANCELING 3 / SUCCEEDED 4 /
CANCELED 5 / ABORTED 6。

## 高级层

- `FoxglovePlayer`：连接/重连/channels/订阅合并/服务 pending/CDR codec 缓存。
- `FoxgloveProtocolClient`：L0 帧编解码 + JSON op；与 vendored 桥逐字节对齐。
- `CodecCache/buildCodec`：schema → MessageReader/Writer。
- 仅当门面不够（自定义派发、原始 CDR、批量协议）时使用；应用代码优先 barrel。

## 浏览器约束

- 需要同源或 CORS 允许；默认 URL 使用 `location.host`，TLS 页面自动 `wss:`。
- 无构建、无打包器；`/js/**` 由网关静态服务。
- `subscribe` 的 `msg.data` 是 CDR 解码后的 JS 对象（int64 可能是 BigInt）。
- 详情/错误语义见 [HTTP/WS API](http-ws.md) 与 [protocol.md](../protocol.md)。
