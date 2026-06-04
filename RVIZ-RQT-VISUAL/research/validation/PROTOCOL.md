# Foxglove WebSocket 协议文档

> foxglove_bridge (ros-humble 3.3.0, foxglove-sdk-cpp v0.23.0) 真实数据协议分析
> 基于 `test_all.mjs` 全量数据采集 (11,220 条消息, 57 次服务调用) 和 `@foxglove/ws-protocol` v0.8.0 源码
> 测试环境: AUBO 机械臂 + MoveIt2 + 3D相机 + 自定义 IVG 接口, 46 topics / 359 services / 45 参数节点

---

## 一、协议架构

```
┌─────────────────────────────────────────────────────────────┐
│                    WebSocket (ws://host:8765)                │
│                   子协议: foxglove.sdk.v1                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─ JSON 控制帧 (Text opcode) ──────────────────────────┐   │
│  │ • op: "serverInfo"        — 握手后首发, 能力声明      │   │
│  │ • op: "advertise"         — Channel 注册 (schema文本)│   │
│  │ • op: "unadvertise"       — Channel 移除              │   │
│  │ • op: "advertiseServices" — 服务注册 (schema文本)    │   │
│  │ • op: "unadvertiseServices"— 服务移除                 │   │
│  │ • op: "serviceCallResponse"— 服务调用成功响应         │   │
│  │ • op: "serviceCallFailure" — 服务调用失败响应         │   │
│  │ • op: "parameterValues"   — 参数读取结果              │   │
│  │ • op: "connectionGraphUpdate"— 拓扑更新 (需客户端订阅) │   │
│  │ • op: "asset"             — 资产数据 (可选)           │   │
│  └───────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌─ CDR 二进制帧 (Binary opcode) ───────────────────────┐   │
│  │ • 话题数据: JointState / TF / PointCloud2 / Image ... │   │
│  │ • 服务 payload: 请求体 / 响应体 (encoding: "cdr")     │   │
│  │ • 客户端 publish: 序列化后的消息                       │   │
│  └───────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘

核心设计: 元数据走 JSON 文本, 数据走 CDR 二进制。
JSON 负责 "有什么", CDR 负责 "内容是什么"。
```

---

## 二、WebSocket 连接

### 2.1 握手

```javascript
// 客户端
const ws = new WebSocket('ws://localhost:8765', ['foxglove.sdk.v1'])

// ⚠️ 子协议验证
// foxglove.sdk.v1      → ✅ 连接成功
// foxglove.websocket.v1 → ❌ HTTP 400 Bad Request
// 不指定子协议           → ❌ 拒绝
```

### 2.2 serverInfo (连接建立后第一条消息)

```json
{
  "op": "serverInfo",
  "name": "",
  "capabilities": [
    "clientPublish",
    "connectionGraph",
    "parameters",
    "parametersSubscribe",
    "services",
    "assets"
  ],
  "supportedEncodings": ["cdr", "json"],
  "metadata": {
    "ROS_DISTRO": "humble",
    "fg-library": "foxglove-sdk-cpp/v0.23.0"
  },
  "sessionId": "1780558067576"
}
```

| capability | 含义 |
|------------|------|
| `clientPublish` | 客户端可发布消息到 ROS2 |
| `connectionGraph` | 提供 publisher/subscriber 拓扑 |
| `parameters` | 支持参数读写 (get/set) |
| `parametersSubscribe` | 支持 `/parameter_events` 订阅 |
| `services` | 支持 ROS2 服务调用 |
| `assets` | 支持资产下载 (URDF/mesh) |

| metadata 字段 | 含义 |
|---------------|------|
| `ROS_DISTRO` | ROS2 发行版标识 (实测 `"humble"`) |
| `fg-library` | foxglove-sdk-cpp 版本 (实测 `"foxglove-sdk-cpp/v0.23.0"`) |
| `sessionId` | 服务端会话 ID (字符串数字, 每次连接重新生成) |

---

## 三、Channel 管理 (话题)

### 3.1 advertise — 服务端推送可用话题

```json
{
  "op": "advertise",
  "channels": [
    {
      "id": 1,
      "topic": "/joint_states",
      "encoding": "cdr",
      "schemaName": "sensor_msgs/msg/JointState",
      "schemaEncoding": "ros2msg",
      "schema": "# JointState message\nstd_msgs/Header header\n..."
    }
  ]
}
```

字段说明:

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | number | Channel 唯一标识, 后续 subscribe/publish 使用 |
| `topic` | string | ROS2 topic 名称 |
| `encoding` | `"cdr"` \| `"json"` | 数据编码, ROS2 话题几乎都是 `"cdr"`, 系统话题可能为 `"json"` |
| `schemaName` | string | ROS2 消息类型名, 如 `sensor_msgs/msg/JointState` |
| `schemaEncoding` | `"ros2msg"` \| `"jsonschema"` | schema 文本格式, JSON 通道用 `"jsonschema"` |
| `schema` | string | **ros2msg 原始文本**或 **JSON Schema**, 不是 JSON Schema |

### 3.2 schema 解析 (关键步骤)

```javascript
import { parse as parseRosMsg } from '@foxglove/rosmsg'
import { MessageReader, MessageWriter } from '@foxglove/rosmsg2-serialization'

// ⚠️ 必须 parse 后才能传给 MessageReader/MessageWriter
const definitions = parseRosMsg(ch.schema, { ros2: true })
// → [{ name: 'header', type: 'std_msgs/Header', isComplex: true },
//    { name: 'name',   type: 'string', isArray: true },
//    { name: 'position', type: 'float64', isArray: true }, ...]

const reader = new MessageReader(definitions)
const writer = new MessageWriter(definitions)

// 不能这样做: const reader = new MessageReader(ch.schema) → 类型错误
```

### 3.3 JSON 编码通道 (特殊)

少数通道使用 `encoding: "json"` + `schemaEncoding: "jsonschema"`, 不走 CDR:

```json
{
  "id": 1,
  "topic": "/foxglove_bridge/sysinfo",
  "encoding": "json",
  "schemaName": "foxglove.SystemInfo",
  "schemaEncoding": "jsonschema",
  "schema": "{\"$schema\":\"https://json-schema.org/draft/2020-12/schema\",\"title\":\"SystemInfo\",...}"
}
```

- 数据直接是 JSON 字符串, 不需要 `MessageReader` 解析
- 包含系统监控信息: 进程内存、CPU 使用率、总系统内存等
- 这类通道没有 `codecMap` 条目, 需单独处理

### 3.4 unadvertise — 服务端通知话题移除

```json
{
  "op": "unadvertise",
  "channelIds": [1, 2, 3]
}
```

客户端应清理对应的 reader/writer/codecMap 缓存。

---

## 四、数据收发 (Pub/Sub)

### 4.1 订阅

```javascript
// 客户端 → foxglove_bridge
const subscriptionId = foxgloveClient.subscribe(channelId)
// foxglove_bridge 内部创建 ROS2 subscription, 开始接收数据
```

### 4.2 数据接收 (二进制 CDR 帧)

```javascript
foxgloveClient.on('message', ({ subscriptionId, timestamp, data }) => {
  // subscriptionId: 由 subscribe() 返回
  // timestamp: bigint, 纳秒 (ROS2 时间)
  // data: ArrayBufferView — CDR 序列化的消息体

  const tsMs = Number(timestamp / 1_000_000n)  // 纳秒 → 毫秒
  const jsObj = reader.readMessage(data)       // CDR → JS Object
})
```

### 4.3 CDR 类型映射 (重要)

| CDR 类型 | JS 类型 | 注意事项 |
|----------|---------|---------|
| `bool` | boolean | 正常 |
| `int8/uint8/int16/uint16/int32/uint32` | number | 正常 |
| `int64/uint64` | **BigInt** | `7n !== 7`, 比较前必须 `Number(value)` |
| `float32/float64` | number | 正常 |
| `string` | string | 正常 |
| `float64[]` | **Float64Array** | 不是 Array, 用 `arr[i]` 索引 |
| `int32[]` 等 | **TypedArray** | 同上 |
| `byte[]` | **Uint8Array** | 图像原始数据 |
| 嵌套消息 | Object | 递归解析 |

```javascript
// ❌ 常见错误
if (rsp.sum === 7) ...       // BigInt 7n !== 7 → 永远 false
jsObj.position.map(...)       // Float64Array 没有 .map()

// ✅ 正确写法
if (Number(rsp.sum) === 7) ...
jsObj.position[0]             // 索引访问对 Array 和 TypedArray 都有效
```

### 4.4 客户端发布

```javascript
// 1. 声明发布者
const clientChannelId = foxgloveClient.advertise({
  topic: '/chatter',
  encoding: 'cdr',
  schemaName: 'std_msgs/msg/String',
  schema: 'string data',
  schemaEncoding: 'ros2msg',
})

// 2. 序列化 + 发送
const cdrData = writer.writeMessage({ data: 'hello from foxglove' })
foxgloveClient.sendMessage(clientChannelId, cdrData)

// 3. 取消发布
foxgloveClient.unadvertise(clientChannelId)
```

---

## 五、服务调用 (Service)

### 5.1 advertiseServices — 服务端推送可用服务

```json
{
  "op": "advertiseServices",
  "services": [
    {
      "id": 1,
      "name": "/add_two_ints",
      "type": "example_interfaces/srv/AddTwoInts",
      "request": {
        "encoding": "cdr",
        "schema": "int64 a\nint64 b",
        "schemaEncoding": "ros2msg"
      },
      "response": {
        "encoding": "cdr",
        "schema": "int64 sum",
        "schemaEncoding": "ros2msg"
      }
    }
  ]
}
```

### 5.2 调用服务

```javascript
// 1. 构造请求 (CDR 序列化)
const reqWriter = new MessageWriter(parseRosMsg(svc.request.schema, { ros2: true }))
const payload = reqWriter.writeMessage({ a: 7, b: 8 })

// 2. 发送服务调用
const callId = ++counter
foxgloveClient.sendServiceCallRequest({
  serviceId: svc.id,
  callId: callId,
  encoding: svc.request.encoding,  // "cdr"
  data: payload,                    // ArrayBuffer
})

// 3. 等待响应
// 成功: { op: "serviceCallResponse", serviceId, callId, encoding: "cdr", data: ArrayBuffer }
// 失败: { op: "serviceCallFailure", callId, message: "error reason" }
```

### 5.3 服务响应解码

```javascript
foxgloveClient.on('serviceCallResponse', (rsp) => {
  // rsp.data 是 CDR 二进制 ArrayBuffer
  const resWriter = new MessageReader(parseRosMsg(svc.response.schema, { ros2: true }))
  const result = resWriter.readMessage(rsp.data)
  // { sum: 15n } — 注意 int64 是 BigInt
})

foxgloveClient.on('serviceCallFailure', (f) => {
  // f.message: 错误描述字符串
})
```

---

## 六、参数操作 (Parameter)

### 6.1 ROS2 参数模型

每个 ROS2 节点自动暴露 4 个参数服务 (rcl_interfaces):

| 服务 | 类型 | 用途 |
|------|------|------|
| `/<node>/list_parameters` | `rcl_interfaces/srv/ListParameters` | 列出所有参数名 |
| `/<node>/describe_parameters` | `rcl_interfaces/srv/DescribeParameters` | 获取参数元数据 (类型/描述/只读) |
| `/<node>/get_parameters` | `rcl_interfaces/srv/GetParameters` | 读取参数值 |
| `/<node>/set_parameters` | `rcl_interfaces/srv/SetParameters` | 写入参数值 |

### 6.2 foxglove_bridge 参数封装

foxglove_bridge 将上述 4 个 ROS2 服务封装为简化的 JSON 控制帧:

**Get (读取):**
```javascript
// → foxglove_bridge 内部调用 /<node>/get_parameters
foxgloveClient.getParameters(['use_sim_time', 'publish_frequency'], requestId)

// ← 响应
{
  "op": "parameterValues",
  "id": "pr_1234567890",
  "parameters": [
    { "name": "use_sim_time", "type": 1, "bool_value": false },
    { "name": "publish_frequency", "type": 3, "double_value": 30.0 }
  ]
}
```

**Set (写入):**
```javascript
// → foxglove_bridge 内部调用 /<node>/set_parameters
foxgloveClient.setParameters([
  { name: 'publish_frequency', value: { type: 3, double_value: 60.0 } }
], requestId)

// ← 响应 (同上 parameterValues)
```

### 6.3 ParameterValue type 枚举

| type | 名称 | 值字段 | JS 类型 |
|------|------|--------|---------|
| 0 | NOT_SET | — | — |
| 1 | BOOL | `bool_value` | boolean |
| 2 | INTEGER | `integer_value` | BigInt (需要 `Number()`) |
| 3 | DOUBLE | `double_value` | number |
| 4 | STRING | `string_value` | string |
| 5 | BYTE_ARRAY | `byte_array_value` | Uint8Array |
| 6 | BOOL_ARRAY | `bool_array_value` | Array |
| 7 | INTEGER_ARRAY | `integer_array_value` | Array (BigInt[]) |
| 8 | DOUBLE_ARRAY | `double_array_value` | Array |
| 9 | STRING_ARRAY | `string_array_value` | Array |

**⚠️ 解析时必须按 type 读取, 不能逐个试:**

```javascript
// ❌ 错误: 所有字段同时存在, bool_value=false 会覆盖 integer 的值
function pval_bad(v) {
  if (v.bool_value !== undefined) return String(v.bool_value)     // integer 也有 false!
  if (v.integer_value !== undefined) return String(v.integer_value)
  // ...
}

// ✅ 正确: 按 type 读取对应字段
const PTYPE_FIELDS = {
  1: 'bool_value', 2: 'integer_value', 3: 'double_value',
  4: 'string_value', 5: 'byte_array_value',
  6: 'bool_array_value', 7: 'integer_array_value',
  8: 'double_array_value', 9: 'string_array_value'
}

function pval(v) {
  if (!v || v.type === 0) return '(not set)'
  return String(v[PTYPE_FIELDS[v.type]] ?? '')
}
```

### 6.4 Set ↔ Get 闭环

```javascript
// 标准流程
// 1. Get 当前值
const before = await foxgloveClient.getParameters(['publish_frequency'], 'req1')
// → { type: 3, double_value: 30.0 }

// 2. Set 新值
await foxgloveClient.setParameters([
  { name: 'publish_frequency', value: { type: 3, double_value: 60.0 } }
], 'req2')

// 3. Get 验证
const after = await foxgloveClient.getParameters(['publish_frequency'], 'req3')
// → { type: 3, double_value: 60.0 } ✅
```

### 6.5 parameter_events

```javascript
// /parameter_events topic 发布 rcl_interfaces/msg/ParameterEvent:
// {
//   stamp: ...,
//   node: "/parameter_blackboard",
//   new_parameters: [{ name, value }],
//   changed_parameters: [{ name, value }],
//   deleted_parameters: [{ name, value }]
// }

// ⚠️ 必须在 Set 操作之前订阅, 否则收不到事件
```

---

## 七、CDR 带宽实测

> 基于全量 11,220 条消息的统计分析 (46 topics, 15 种消息类型)

### 7.1 全量带宽对比 (按类型)

| 消息类型 | CDR 均值 | JSON 均值 | 节省% | 样本数 | 对应 Topic |
|----------|---------|----------|------|--------|-----------|
| PointCloud2 | 5,495,633 | 71,013,472 | **92.3%** | 6 | /camera/depth_registered/points |
| ImageData (IVG) | 921,732 | 11,322,515 | **91.9%** | 14 | /image_data |
| Image (depth) | 614,472 | 7,075,445 | **91.3%** | 6 | /camera/depth/image_raw |
| DisplayTrajectory | 2,252 | 6,128 | 63.3% | 7 | /display_planned_path |
| JointTrajectoryControllerState | 768 | 2,368 | 67.6% | 320 | /joint_trajectory_controller/state |
| TFMessage | 620 | 1,654 | 62.5% | 440 | /tf |
| DynamicJointState | 492 | 721 | 31.8% | 5,281 | /dynamic_joint_states |
| CameraInfo | 440 | 665 | 33.8% | 29 | /camera/color/camera_info |
| JointState | 316 | 454 | 30.4% | 4,621 | /joint_states |
| RobotStatus (IVG) | 244 | 889 | 72.6% | 381 | /robot_status |
| Log | 187 | 254 | 26.3% | 63 | /rosout |
| CameraStatus (IVG) | 72 | 254 | 71.6% | 8 | /camera_status |
| ToolChangerStatus (IVG) | 64 | 170 | 62.4% | 2 | /tool_changer_status |
| ConnectedClients | 8 | 14 | 42.9% | 1 | /connected_clients |
| Int32 | 8 | 10 | 20.0% | 1 | /client_count |

**总体: CDR 54.3MB vs JSON 634.9MB, 节省 91.4%**

### 7.2 按频率分类

| 类别 | 消息数 | 频率估计 | 典型 Topic |
|------|--------|---------|-----------|
| 高频数值 | 5,281 | ~200Hz | /dynamic_joint_states |
| 高频关节 | 4,621 | ~200Hz | /joint_states |
| 中频变换 | 440 | ~20Hz | /tf |
| 中频状态 | 381 | ~15Hz | /robot_status |
| 低频大包 | 14 | ~0.5Hz | /image_data (921KB/条) |
| 超低频点云 | 6 | ~0.2Hz | 点云 (5.5MB/条) |

### 7.3 节省来源分析

```
float64 0.5:
  JSON:  0.5 → "0.5" → 3 字节 + 分隔符
  CDR:   0.5 → IEEE 754 binary → 8 字节 (固定)
  → 数组越大 CDR 优势越明显 (无分隔符开销)

int64 7:
  JSON:  7 → "7" → 1 字节
  CDR:   7 → 8 字节 (固定)
  → 小标量 JSON 略有优势

PointCloud2 (array of struct):
  JSON:  每个点 ≈258B (字段名重复 + 数字→字符串膨胀 + 分隔符)
  CDR:   每个点 20B (纯二进制, 无字段名, 256-bit 对齐)
  → CDR 压倒性优势
```

---

## 八、客户端实现要点 (FoxgloveAdapter)

### 8.1 事件驱动模型

```javascript
import { FoxgloveClient } from '@foxglove/ws-protocol'

const client = new FoxgloveClient({ ws: webSocket })

// 必须监听的事件
client.on('serverInfo',   (info) => { /* 能力声明 */ })
client.on('advertise',    (channels) => { /* 构建 reader/writer */ })
client.on('unadvertise',  (channelIds) => { /* 清理 reader/writer */ })
client.on('message',      ({ subscriptionId, timestamp, data }) => { /* 解码分发 */ })
client.on('advertiseServices', (services) => { /* 注册服务 */ })

// 错误处理
client.on('error', (err) => {
  console.error('FoxgloveClient error:', err)
})
```

### 8.2 完整生命周期

```
new WebSocket(url, ['foxglove.sdk.v1'])
  → serverInfo (能力)
  → advertise (话题 channels, 构建 MessageReader/MessageWriter)
  → advertiseServices (服务, 存储 request/response schema)
  → 客户端 subscribe(channelId) → 收到 CDR 数据帧
  → 客户端 sendServiceCallRequest() → 收到 serviceCallResponse
  → 客户端 publish() / getParameters() / setParameters()
  → 连接关闭: unadvertise → unadvertiseServices
```

### 8.3 错误处理清单

| 场景 | 错误 | 处理 |
|------|------|------|
| 无 request schema 的服务调用 | 4字节 CDR header 无效 | 跳过服务, 不调用 |
| int64 比较 | `7n !== 7` | 强制 `Number(value)` |
| Float64Array 操作 | `.map()` undefined | 用 `for` 循环或索引 |
| SetParameters 权限不足 | `reason: "read-only"` | 跳过只读参数 |
| 参数类型未初始化 | `type: 0` (NOT_SET) | 跳过, 不写 |
| MessageReader schema 不匹配 | 解包异常 | `try-catch` 并重新 parse |
| **BigInt 序列化** | `Do not know how to serialize a BigInt` | JSON.stringify 前用 `Number()` 转换 BigInt |
| 服务不存在 | `Service ... is not available` | 跳过该服务, 不 panic |
| 参数 Set 被拒绝 | `write rejected` | 跳过该参数 (通常是 read_only) |

---

## 九、已知限制

| 限制 | 详情 |
|------|------|
| **Action 不完整转发** | bridge 不把 `/_action/` 内部 topic 注册为 Channel, 但 `connectionGraphUpdate` 仍暴露其 pub/sub |
| **参数名当节点名** | Humble 版本中 `getParameters(['xxx'])` 会先查找节点 `xxx` |
| **无 rosapi** | `/rosapi/*` 服务属于 rosbridge_server, foxglove_bridge 没有 |
| **子协议必须** | 必须 `foxglove.sdk.v1`, 不能省略或使用 `foxglove.websocket.v1` |
| **QoS 兼容性** | parameter_events topic 的 QoS 可能不匹配导致收不到事件 (实测确认) |
| **BigInt JSON 序列化** | `get_parameters` 返回 `integer_value` 为 BigInt, `JSON.stringify` 报错 `Do not know how to serialize a BigInt` |
| **SystemInfo 不走 CDR** | `/foxglove_bridge/sysinfo` 使用 `encoding: "json"`, 不能用 MessageReader 解析 |
| **服务不可用** | 部分服务 (如 `/aubo_driver/set_io`) 在 advertise 中但调用时返回 not available |

### 9.1 connectionGraphUpdate 拓扑实测

**调用 `subscribeConnectionGraph()` 后收到 25 次更新，覆盖 56 个话题:**

```
/parameter_events                  pub=43 sub=36  ← 参数变更
/rosout                            pub=49 sub=2   ← 日志
/attached_collision_object         pub=12 sub=3   ← MoveIt
/trajectory_execution_event        pub=11 sub=2   ← 轨迹事件
/tf_static                         pub=4  sub=8   ← TF 静态变换
/execute_trajectory/_action/feedback pub=1 sub=11 ← ⚠️ Action (bridge 不转发)
/move_action/_action/feedback      pub=1  sub=11  ← ⚠️ Action (bridge 不转发)
/execute_trajectory/_action/status pub=1  sub=11  ← ⚠️ Action (bridge 不转发)
/move_action/_action/status        pub=1  sub=11  ← ⚠️ Action (bridge 不转发)
/tf                                pub=2  sub=9   ← TF 动态变换
/joint_states                      pub=1  sub=7   ← 关节状态
/robot_status                      pub=1  sub=5   ← 机器人状态
```

**关键发现:** 虽然 foxglove_bridge 不把 `/_action/` 内部 topic 作为 Channel 转发，但 `connectionGraphUpdate` 仍然暴露了它们的存在和 pub/sub 关系。这说明 Action 的 ROS2 层通信是完全正常的，只是 bridge 选择不暴露这些 channel。

### 9.2 实际环境服务命名空间

全量采集发现 48 个服务命名空间 (359 个服务):

| 命名空间 | 服务数 | 用途 |
|---------|--------|------|
| `/rosapi` | 29 | rosbridge 兼容 API (services/topics/nodes 查询) |
| `/controller_manager` | 10 | ROS2 控制器管理 |
| `/aubo_driver` | 3 | 机械臂驱动 |
| `/trajectory_recorder` | 2 | 轨迹记录 |
| `/get_current_tool` | 1 | 当前工具查询 |
| `/execute_trajectory` | 1 | 轨迹执行 |
| `/scene_detach` | 1 | 场景拆卸 |
| `/get_current_state` | 1 | 当前状态 |
| `/compute_ik` | 1 | 逆运动学求解 |
| `/standardize_template` | 1 | 模板标准化 |
| 其他 (list_parameters 等) | ~319 | 各节点参数服务 |

---

## 十、与 Rosbridge 对比

| 维度 | Foxglove (CDR+JSON) | Rosbridge (纯 JSON) |
|------|--------------------|--------------------|
| 子协议 | `foxglove.sdk.v1` (必须) | 无 |
| 话题数据 | CDR 二进制 | JSON 文本 |
| 控制消息 | JSON control frames | JSON op 分发 |
| schema 发现 | advertise 事件自动推送 | 需手动查询 `/rosapi/` |
| 点云带宽 | ~5.5MB | ~71MB (大 12.9x) |
| 图像带宽 | ~0.9MB | ~11MB (大 12.3x) |
| 参数操作 | `getParameters`/`setParameters` JSON 帧 | rosbridge JSON op |
| 服务调用 | CDR payload | JSON args |
| 客户端发布 | advertise + sendMessage (CDR) | advertise + publish (JSON) |
| rosapi 服务 | ✅ 通过 rosbridge 集成提供 29 个 `/rosapi/*` 服务 | ✅ 原生提供 |
| Action 转发 | ❌ 不完整 | ❌ 不完整 |
| 适用场景 | 3D可视化/大数据量 | 轻量级RPC/调试 |

> **实测发现**: foxglove_bridge 通过集成 rosbridge_server 也提供 `/rosapi/*` 服务族 (29个),
> 包括 `services_for_type`, `topics_and_raw_types` 等, 延迟 ~1-2ms

---

## 十一、全量分析统计 (2026-06-04 实测)

### 11.1 整体数据

| 指标 | 值 |
|------|-----|
| 话题总数 | 46 (47 channels, 含 JSON) |
| 服务总数 | 359 (去重后89个业务服务) |
| 参数节点 | 45 |
| Action | 0 (foxglove_bridge 不转发) |
| 采集消息数 | 11,220 |
| 服务调用 | 57 次 |
| CDR 总流量 | 54.3 MB |
| JSON 等效流量 | 634.9 MB |
| 整体节省 | **91.4%** |
| 目录类型 | 15 类 |

### 11.2 服务调用延迟

| 服务 | 延迟 |
|------|------|
| `/controller_manager/set_hardware_component_state` | 43ms |
| `/get_current_tool` | 1ms |
| `/execute_trajectory` | 1ms |
| `/scene_detach` | 42ms |
| `/get_current_state` | 1001ms |
| `/compute_ik` | 6ms |
| `*/list_parameters` (平均) | 1-45ms |
| `*/set_parameters` | 1-45ms |

### 11.3 新发现消息类型

| 消息类型 | Topic 示例 | CDR 均值 |
|---------|-----------|---------|
| `ivg_interfaces/msg/ImageData` | /image_data | 921,732 B |
| `ivg_interfaces/msg/RobotStatus` | /robot_status | 244 B |
| `ivg_interfaces/msg/CameraStatus` | /camera_status | 72 B |
| `ivg_interfaces/msg/ToolChangerStatus` | /tool_changer_status | 64 B |
| `control_msgs/msg/JointTrajectoryControllerState` | /joint_trajectory_controller/state | 768 B |
| `moveit_msgs/msg/DisplayTrajectory` | /display_planned_path | 2,252 B |
| `rosbridge_msgs/msg/ConnectedClients` | /connected_clients | 8 B |
| `foxglove.SystemInfo` (JSON) | /foxglove_bridge/sysinfo | JSON 直接 |

### 11.4 Topic 分类分布

```
传感器/相机     7 topics — CameraInfo, Image, trigger_event
规划/MoveIt     6 topics — PlanningScene, MotionPlanRequest, AttachedCollisionObject
其他           6 topics — SystemInfo, ConnectedClients, RecognizedObjectArray
控制/轨迹       5 topics — JointTrajectoryControllerState, TransitionEvent
可视化/Marker   4 topics — MarkerArray, InteractiveMarker
自定义/IVG      3 topics — ImageData, CameraStatus, String
应用/机器人状态   3 topics — RobotStatus, RobotIOStatus, ToolChangerStatus
机器人/TF变换    2 topics — TFMessage
机器人/URDF模型  2 topics — String (robot_description)
系统/事件参数    2 topics — ParameterEvent, WriteSplitEvent
应用/位姿抓取    2 topics — String (grasp_place_status)
传感器/点云     1 topic  — PointCloud2
机器人/关节状态  1 topic  — JointState
控制/动态状态    1 topic  — DynamicJointState
系统/日志       1 topic  — Log
系统/生命周期    1 topic  — TransitionEvent
```

---
### 10.2 connectionGraphUpdate 协议格式

**客户端订阅:**
```javascript
client.subscribeConnectionGraph()  // → { op: "subscribeConnectionGraph" }
client.unsubscribeConnectionGraph() // → { op: "unsubscribeConnectionGraph" }
```

**服务端推送** (连接后持续推送拓扑变化):
```json
{
  "op": "connectionGraphUpdate",
  "publishedTopics": [
    { "name": "/joint_states", "publisherIds": [1] }
  ],
  "subscribedTopics": [
    { "name": "/joint_states", "subscriberIds": [2, 3, 4] }
  ],
  "removedTopics": []
}
```

| 字段 | 说明 |
|------|------|
| `publishedTopics` | 有发布者的话题及 publisher ID 列表 |
| `subscribedTopics` | 有订阅者的话题及 subscriber ID 列表 |
| `removedTopics` | 已消失的话题名列表 |

**实测行为:**
- 连接后立即推送初始拓扑快照，之后增量推送变更
- 推送频率高 (实测 25 次/49s)，适合实时拓扑可视化
- 会暴露 bridge 不转发的话题 (如 `/_action/feedback`, `/_action/status`)

---
## 十二、依赖库版本

```json
{
  "@foxglove/ws-protocol": "^0.8.0",
  "@foxglove/rosmsg": "^5.0.5",
  "@foxglove/rosmsg2-serialization": "^3.1.2",
  "@foxglove/cdr": "^3.5.1"
}
```

| 库 | 职责 |
|----|------|
| `@foxglove/ws-protocol` | `FoxgloveClient` 类, WS 连接管理, 事件派发 |
| `@foxglove/rosmsg` | `parse(ros2msgText)` → `MessageDefinition[]` |
| `@foxglove/rosmsg2-serialization` | `MessageReader` (CDR→JS) / `MessageWriter` (JS→CDR) |
| `@foxglove/cdr` | 底层 CDR 二进制编解码 (rosmsg2-serialization 依赖) |

---

> 所有内容基于 foxglove_bridge 真实运行环境验证 (2026-06-04 全量分析更新)
> 全量分析原始数据: `full_analysis.json` (1.4MB, 11,220 条消息/57 次服务调用/27 次拓扑更新)
>
> ### 配套工具
>
> | 文件 | 用途 |
> |------|------|
> | `test_all.mjs` | 综合测试入口，自动发现 + 全量数据采集 |
> | `full_analysis.json` | 全量数据导出 (由 test_all.mjs 自动生成) |
> | `gen_topology_graph.mjs` | 从 full_analysis.json 生成拓扑图 |
> | `topology.html` | 交互式 ROS2 节点拓扑图 (类似 rqt_graph) |
> | `topology.dot` / `topology.svg` | Graphviz 格式拓扑图 |
