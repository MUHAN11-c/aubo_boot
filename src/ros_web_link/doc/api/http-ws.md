# HTTP / WebSocket API

网关默认 `http://127.0.0.1:8095`；上游桥默认 `127.0.0.1:8765`（`monitor.launch.py`
用 18766 规避常见占用）。

## HTTP 端点

| 端点 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 纯传输模式 307 → `/console.html`；组合模式归 UI 根 |
| `/console.html` | GET | 本包监控台（组合模式 `/rwl/console.html`） |
| `/health` | GET | 桥/视频 TCP 探测，2s 缓存；桥 down → `status=degraded`（仍 200） |
| `/api/ivg/ws/stats` | GET | `/ws/foxglove` 转发统计（连接级 + 进程累计） |
| `/api/ivg/robot-mesh/{pkg}/{rel}` | GET | ament share 内 STL/DAE/OBJ；防路径穿越；缓存 1d |
| `/api/ivg/proxy/web-video/{path}` | GET/HEAD | MJPEG/JPEG 代理到 web_video_server；上游不可达 502/504 |
| `/js/**` | GET | 通信层/console/示例 JS（纯传输模式） |
| `/examples/minimal_client.html` | GET | 浏览器最小调用示例 |

### `/health`

```json
{
  "status": "ok",
  "version": "1.1.0",
  "protocol": "foxglove.websocket.v1",
  "static_root": "/…/share/ros_web_link/web/public",
  "foxglove_bridge": "ok",
  "web_video": "down"
}
```

- `status`: `ok` | `degraded`（桥不可达时 degraded；web_video 只上报不降级）。
- `foxglove_bridge` / `web_video`: `ok` | `down`。
- 探测是 TCP connect，不发 WS/HTTP 握手；结果缓存 2s。

### `/api/ivg/ws/stats`

```json
{
  "dataLaneBytes": 4194304,
  "active": [{
    "startedAt": 1792397000.1,
    "remote": "127.0.0.1:54321",
    "upstreamFrames": 1200,
    "forwardedJson": 12,
    "forwardedData": 1180,
    "forwardedDataBytes": 8000000,
    "droppedData": 3,
    "droppedDataBytes": 24000,
    "droppedStale": 1,
    "queued": 2,
    "queuedBytes": 180000,
    "peakQueuedBytes": 900000
  }],
  "totals": {
    "connections": 5,
    "upstreamFrames": 50000,
    "forwardedJson": 400,
    "forwardedData": 49600,
    "forwardedDataBytes": 350000000,
    "droppedData": 120,
    "droppedDataBytes": 9000000,
    "droppedStale": 40
  }
}
```

`totals` 只在连接断开时归账；活跃连接看 `active[]`。`console` 监控卡的“网关丢帧/
网关积压”两者求和。

### robot-mesh

```http
GET /api/ivg/robot-mesh/turtlebot3_gazebo/models/turtlebot3_waffle_pi/meshes/base.stl
```

- `{pkg}` 必须是已安装 ament 包；`{rel}` 相对于该包 share 目录。
- 拒绝 URL `..`/绝对路径；词法 containment（兼容 colcon symlink-install）。
- 命中返回 `model/stl` / `model/vnd.collada+xml` / `model/obj`；
  不存在 404，包不存在 404。
- `Cache-Control: public, max-age=86400`。

### web-video

```http
GET /api/ivg/proxy/web-video/stream?topic=/camera/image_raw
```

- 路径原样拼到 `web_video.host:port` 上游；`..`/绝对路径 400。
- 流式转发，保留上游 `content-type`；上游连接失败 502，读超时 504。
- `web_video_server` 是 optional 组件，`/health` 中 down 不影响 status。

## `/ws/foxglove`

浏览器 ↔ 网关 ↔ `ivg_foxglove_bridge` 的反向代理。协议为
**Foxglove WebSocket v1**（`foxglove.websocket.v1`，桥也接受 `sdk.v1` 归一）。
消息体是 **CDR 二进制**，不是 rosbridge JSON。

### 握手

```http
GET /ws/foxglove HTTP/1.1
Upgrade: websocket
Sec-WebSocket-Protocol: foxglove.websocket.v1
```

### 首帧 `serverInfo`（JSON 文本）

```json
{
  "op": "serverInfo",
  "name": "foxglove_bridge",
  "capabilities": ["clientPublish", "services", "connectionGraph"],
  "channels": [
    {"id": 1, "topic": "/joint_states", "encoding": "cdr",
     "schemaName": "sensor_msgs/msg/JointState", "schema": "..."}
  ],
  "services": []
}
```

`channels[].schema`/`schemaName` 驱动客户端 CDR codec；没有 schema 就无法解码。

### 订阅/发布/服务

```json
{"op":"subscribe","subscriptions":[{"id":1,"channelId":1}]}
{"op":"advertise","channels":[{"id":1,"topic":"/cmd_vel","encoding":"cdr",
  "schemaName":"geometry_msgs/msg/Twist"}]}
{"op":"subscribeConnectionGraph"}
```

二进制帧（小端）：

| 方向 | 首字节 | 结构 |
|---|---|---|
| S→C 消息 | `0x01` | `[subId u32][timestamp u64][CDR payload]` |
| S→C 时间 | `0x02` | `[timestamp u64]` |
| S→C 服务响应 | `0x03` | `[serviceId u32][callId u32][encLen u32][enc][data]` |
| C→S 消息 | `0x01` | `[channelId u32][CDR payload]` |
| C→S 服务调用 | `0x02` | `[serviceId u32][callId u32][encLen u32][enc][data]` |

`enc` 为 `cdr`。`advertiseServices`/`callService` 为 JSON 控制帧；实际 payload 按上述
二进制帧走。

### 参数/图/Action

- 参数：标准 `<node>/get_parameters`、`<node>/set_parameters` 服务，不额外起节点。
- 图：`connectionGraph` capability；`subscribeConnectionGraph` 后增量更新。
- Action：协议无 action 概念；客户端组合 3 服务 + 2 话题（JS `action_client.js`）。

### 错误/背压/关闭

- 上游桥不可达：有限重试 + 抖动；全部失败以 close code **1011** 关闭。
- 客户端腿心跳默认 20s/20s；半开连接可检测。
- 网关出站有界数据通道：`proxy.data_lane_bytes`（默认 4MiB）超限丢最旧二进制帧，
  JSON 控制帧永不丢；`proxy.egress_max_age_s`（默认 2s）丢弃已陈旧 messageData。
- 上游 `proxy.upstream_max_queue` 默认 4，让慢客户端更早背压到桥。
- 浏览器 player 的 `latest` 合并是最后一道 latest-wins 保障。

### 安全

- 默认 `bind 0.0.0.0` 且无鉴权；生产置于反向代理/TLS 后或改 `bind:=127.0.0.1`。
- 安全响应头：`X-Content-Type-Options: nosniff`、`X-Frame-Options: SAMEORIGIN`；
  HTML/JS `Cache-Control: no-cache`。
- 不要把未鉴权的 `/ws/foxglove` 直接暴露公网。

## 与 rosbridge 的关系

不是 wire 兼容：没有 `/ws/rosbridge`，rosbridge JSON 客户端不能直连。能力映射与
roslibjs 迁移见 [protocol.md](../protocol.md)。
