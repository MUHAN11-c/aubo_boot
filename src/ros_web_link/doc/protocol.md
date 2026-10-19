# Foxglove v1 ↔ rosbridge v2.1 对照与迁移

本包不是 rosbridge 的 wire 兼容实现，而是沿 Foxglove WebSocket v1 路线的独立包。
本文给能力对照、编码差异与 roslibjs 迁移方式，避免把“语义相同”误当“可直接连”。

参考：

- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) /
  [ROSBRIDGE_PROTOCOL.md](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
- [roslibjs](https://github.com/RobotWebTools/roslibjs)
- [foxglove/ws-protocol](https://github.com/foxglove/ws-protocol)

## 1. 包/层对照

| rosbridge_suite | 本包 | 说明 |
|---|---|---|
| `rosbridge_library`（协议/能力） | `web/public/js/transport/foxglove/{protocol,player,codec}.js` + Python `FoxgloveWsClient` | 帧/会话/CDR |
| `rosbridge_server`（WebSocket） | 网关 `/ws/foxglove` + `ivg_foxglove_bridge` | WS 反代 + 桥 |
| `rosapi`（图/参数） | `connectionGraph` + 标准参数服务 | 不额外起 rosapi 节点 |
| `roslibjs` | `ros_client.js` + `action_client.js` | Topic/Service/Param/Action 门面 |
| `ROSBRIDGE_PROTOCOL.md` | 本文 + [api/http-ws.md](api/http-ws.md) | 协议/端点 |
| `rosbridge_suite/examples` | [`examples/`](../examples/README.md) | 可直接运行 |

Foxglove 侧：`ivg_foxglove_bridge` ≈ `foxglove_bridge`；本包 JS 是零构建客户端层，
不引入官方 SDK（ws-protocol 已归档）。

## 2. 操作对照

| 能力 | rosbridge v2.1 | 本包（Foxglove v1） |
|---|---|---|
| 订阅 | `{"op":"subscribe","topic":...}` | `subscribe` + `channelId`；先有 channel/schema |
| 发布 | `advertise` + `publish`（JSON msg） | `advertise` + 二进制 clientPublish（CDR） |
| 服务 | `call_service`/`service_response` | `advertiseServices` + `callService` JSON 控制 + CDR 帧 |
| Action | `send_action_goal`/`cancel_action_goal`/feedback/result | 无原生 action；3 服务 + 2 话题组合（JS action_client） |
| 参数 | rosapi `get_param`/`set_param` | 标准 `<node>/get_parameters`/`<node>/set_parameters` |
| 图 | rosapi | `connectionGraph` capability |
| 编码 | JSON / base64 / CBOR / CBOR-raw | CDR 二进制 + schema |
| QoS | 显式 `qos` 对象 | 跟随图上发布者；无客户端 qos 字段 |
| 节流/队列/分片 | `throttle_rate`/`queue_length`/`fragment_size` | 无；`dispatchMode` + 网关有界通道 |
| 压缩 | PNG/CBOR | 不做消息级压缩；permessage-deflate 默认关 |
| 传输路径 | `/`（rosbridge WS） | `/ws/foxglove`（**没有** `/ws/rosbridge`） |

**结论**：rosbridge 客户端不能直连；迁移的是 API 语义与数据模型，不是 JSON 帧。

## 3. 帧示例

### rosbridge subscribe

```json
{"op":"subscribe","id":"s1","topic":"/joint_states",
 "type":"sensor_msgs/msg/JointState"}
```

### foxglove subscribe

```json
{"op":"subscribe","subscriptions":[{"id":1,"channelId":3}]}
```

`channelId` 来自首帧 `serverInfo.channels`；消息 payload 是 CDR，不是 JSON。

### rosbridge call_service

```json
{"op":"call_service","id":"c1","service":"/rwl_demo/add_two_ints",
 "args":{"a":20,"b":22}}
```

### foxglove 服务调用

1. `serverInfo.services` 给出 service id/schema；
2. JSON 控制帧 `callService` 发起；
3. 二进制帧 `[0x02][serviceId][callId][encLen][enc][CDR]`；
4. 响应二进制帧 `[0x03][serviceId][callId][encLen][enc][CDR]`。

高层 API 用 `FoxgloveWsClient.call_service()` 或 JS `callService()`，不直接拼帧。

## 4. roslibjs → 本包 JS API 迁移

| roslibjs | 本包 |
|---|---|
| `new ROSLIB.Ros({url})` | `foxgloveRosClient().connect(url)` |
| `new ROSLIB.Topic({ros,name,messageType})` + `subscribe(cb)` | `ros.subscribe(name, messageType, cb)` |
| `topic.publish(msg)` | `ros.publish(name, messageType, msg)` |
| `new ROSLIB.Service({ros,name}).callService(req,cb)` | `await ros.callService(name, req, {timeoutMs})` |
| `new ROSLIB.Param({ros,name}).get/set` | `await ros.getParams(node,[name])` / `ros.setParam(node,name,value)` |
| `new ROSLIB.ActionClient({ros,serverName,actionName})` | `createActionClient(actionName).sendGoal(goal,{onFeedback})` |
| `new ROSLIB.TFClient({ros,fixedFrame})` | 直接订阅 `/tf` + `/tf_static`，或复用 `view3d/tf_buffer.js` |

```js
import { foxgloveRosClient } from '/js/ros_web_link.js';
const ros = foxgloveRosClient();
await ros.connect();                       // ws://host/ws/foxglove

const sub = ros.subscribe('/joint_states', 'sensor_msgs/msg/JointState',
  (msg) => console.log(msg.data.name));
ros.publish('/cmd_vel', 'geometry_msgs/msg/Twist',
  { linear: { x: 0.2, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
const r = await ros.callService('/rwl_demo/add_two_ints',
  { a: 20, b: 22 }, { timeoutMs: 8000 });
sub.cancel();
```

## 5. 限制与替代

| rosbridge 能力 | 本包现状 | 替代 |
|---|---|---|
| JSON publish/subscribe | 不支持 | CDR + schema；用 JS/Python 客户端 |
| `throttle_rate` | 不支持 | `dispatchMode: 'latest'`；网关有界通道 |
| `queue_length`/fragment | 不支持 | 网关丢最旧/年龄门；桥帧上限 128MiB |
| 显式 QoS | 不支持 | 跟随图上发布者 QoS |
| 原生 action ops | 不支持 | JS action_client 组合 3 服务+2 话题 |
| rosapi 参数/图 | 不额外起节点 | 标准参数服务 + connectionGraph |
| PNG/CBOR 压缩 | 不支持 | 关闭 deflate；按需在反代压缩 |

## 6. 为什么不做 rosbridge 兼容端点

- 协议路线已冻结为 Foxglove v1（与官方 rviz_web 同路线，CDR 省带宽）；
- 维护两套 WS 传输会重复背压/心跳/重连语义，且 rosbridge JSON 对点云/图像不经济；
- 本包定位是“独立监控 + JS 客户端库”，不是 rosbridge 替代服务器的 drop-in；
- 需要 rosbridge 的现有系统可继续用 rosbridge，本包通过 `monitor.launch.py`
  与 Foxglove 兼容桥共存。

能力细节见 [api/http-ws.md](api/http-ws.md) 与 [api/js.md](api/js.md)。
