# Foxglove Bridge 客户端依赖库 — 零基础搜索全流程

> 从一台装了 foxglove_bridge 的机器出发，找到写 JS 客户端需要的所有 npm 包。
> 每一步都有具体的命令、输出和判断依据。

---

## 总览: 四层搜索法

```
第一层: 从系统已安装的包反向查 → 找到组织名和代码仓库
第二层: npm 批量搜索该组织所有包 → 得到候选列表 (19个)
第三层: 逐个读候选包的源码 → 确认依赖关系链
第四层: 实际写代码验证 → 发现遗漏的透传依赖
```

---

## 第一层: 从系统里找到"目标是谁"

### 1.1 查看已安装的 Foxglove 包

**命令:**
```bash
dpkg -l | grep foxglove
```

**输出:**
```
ii  ros-humble-foxglove-bridge  3.3.0-1jammy.20260504.100731  amd64  ROS Foxglove Bridge
```

**判断:** 系统装了 `foxglove_bridge`，版本 3.3.0。这就是要连接的 WebSocket 服务端。

---

### 1.2 查包的详细信息 (找到组织名和仓库)

**命令:**
```bash
apt-cache show ros-humble-foxglove-bridge
```

**关键输出:**
```
Maintainer: Foxglove <ros-tooling+foxglove_bridge@foxglove.dev>
Homepage: https://github.com/foxglove/foxglove-sdk
Version: 3.3.0-1jammy.20260504.100731
Description: ROS Foxglove Bridge
```

**信息提取:**

| 字段 | 提取的信息 | 推论 |
|------|-----------|------|
| Maintainer | `foxglove.dev` | 组织名是 "Foxglove" |
| Homepage | `github.com/foxglove/foxglove-sdk` | 代码在 foxglove 这个 GitHub org 下 |
| 包名 | `ros-humble-foxglove-bridge` | 它是 ROS2 Humble 的桥接节点 |

**操作:** 既然代码仓库是 `github.com/foxglove/foxglove-sdk`，那 JS 客户端大概率也在这个 org 的 npm 上。npm 的 org 名就是 `@foxglove`。

---

### 1.3 查看启动配置 (确认协议能力)

**命令:**
```bash
cat /opt/ros/humble/share/foxglove_bridge/launch/foxglove_bridge_launch.xml
```

**关键行:**
```xml
<arg name="port" default="8765" />
<arg name="address" default="0.0.0.0" />
<arg name="capabilities" default="[clientPublish,parameters,parametersSubscribe,services,connectionGraph,assets]" />
```

**信息提取:**

| 参数 | 值 | 推论 |
|------|-----|------|
| port | 8765 | WebSocket 端口 |
| address | 0.0.0.0 | 监听所有网卡 |
| capabilities | clientPublish, parameters, services, connectionGraph, assets | 支持: 客户端发布/参数读写/服务调用/拓扑图/资产下载 |

**判断:** 这是一个功能完整的 Foxglove WebSocket 协议服务端。JS 客户端需要对接这 6 种能力。

---

## 第二层: npm 批量搜索 Foxglove 组织的全部包

### 2.1 搜索所有 @foxglove/* 包

**命令:**
```bash
npm search @foxglove
```

**输出 (19 个包，按相关性分类):**

#### ✅ 高度相关 (含 ros2 / websocket / cdr / serialization 关键字)

```
@foxglove/ws-protocol
  description: "Foxglove WebSocket protocol"
  keywords: foxglove websocket robotics ros ros2
  → 这个极可能是客户端 SDK

@foxglove/rosmsg2-serialization
  description: "ROS 2 message serialization, for reading and writing bags and network messages"
  keywords: ros ros2 robot operating system cdr serialization deserialization serde rosbag
  → 这个负责 ROS2 消息的序列化/反序列化 (CDR 格式)

@foxglove/rosmsg
  description: "Parser for ROS and ROS 2 .msg definitions"
  keywords: ros ros1 ros2 message definition msg srv msgdef parser grammar
  → 这个负责解析 .msg 文件的语法

@foxglove/cdr
  description: "Common Data Representation serialization and deserialization library"
  keywords: cdr dds rtps omg serialization deserialization serde ros2
  → 这个负责底层 CDR 二进制编解码

@foxglove/rosmsg-msgs-common
  description: "Common ROS message definitions using @foxglove/rosmsg"
  keywords: ros ros2 robot operating system message definitions idl msg serialization deserialization serde rosbag
  → 这个提供常见的 ROS 消息定义 (可能不需要，因为 bridge 自带 schema)
```

#### ⚠️ 可能相关

```
@foxglove/ws-protocol-examples  — "Foxglove WebSocket protocol examples"
@foxglove/rostime               — "ROS Time and Duration primitives and helper methods"
@foxglove/message-definition    — "Common types for message definition schemas"
```

#### ❌ 无关

```
@foxglove/eslint-plugin       — ESLint 配置 (开发工具)
@foxglove/tsconfig            — TypeScript 配置 (开发工具)
@foxglove/extension           — Foxglove 扩展 API
@foxglove/schemas             — Foxglove 自有消息 schema
@foxglove/embed / embed-react — Foxglove 嵌入组件
@foxglove/rosbag              — rosbag 文件读取
@foxglove/ros1                — ROS1 协议 (我们需要 ROS2)
@foxglove/three-text          — Three.js 文字渲染
@foxglove/just-fetch          — HTTP fetch 封装
@foxglove/wasm-bz2            — bzip2 解压
```

**判断:** 前 5 个高度相关的包构成了候选集。但不能根据描述就下结论，必须读源码确认各自的职责和相互关系。

---

## 第三层: 精读候选包源码 — 确认职责和依赖关系

### 3.1 精读 `@foxglove/ws-protocol`

**阅读文件:** `FoxgloveClient.ts` (271 行)

#### L1-27: import 列表揭示了协议的全部概念

```typescript
import {
  BinaryOpcode,        // 二进制帧类型: MESSAGE_DATA=1, SERVICE_CALL_RESPONSE=3
  Channel,             // 话题: { id, topic, encoding, schemaName, schema }
  ClientChannel,       // 客户端发布的频道
  IWebSocket,          // ⚠️ WebSocket 抽象接口 (构造函数需要)
  Parameter,           // 参数: { name, value }
  Service,             // 服务: { id, name, type, request, response }
  ServiceCallPayload,  // 服务调用请求体
  SubscriptionId,      // 订阅 ID
} from "./types";
```

**推论:** 这个库完整定义了 Foxglove WebSocket 协议的所有类型。它是客户端 SDK 的核心。

#### L29-48: EventTypes — 所有事件回调

```typescript
type EventTypes = {
  serverInfo:        (event: ServerInfo) => void;          // 握手信息 (能力声明)
  advertise:         (newChannels: Channel[]) => void;      // 新话题注册
  unadvertise:       (removedChannels: ChannelId[]) => void; // 话题移除
  message:           (event: MessageData) => void;          // ⚠️ 数据消息
  advertiseServices: (newServices: Service[]) => void;      // 新服务注册
  unadvertiseServices:(removedServices: ServiceId[]) => void;
  serviceCallResponse:(event: ServiceCallResponse) => void; // 服务响应
  serviceCallFailure:(event: ServiceCallFailure) => void;   // 服务失败
  parameterValues:   (event: ParameterValues) => void;      // 参数数据
  connectionGraphUpdate: (event: ConnectionGraphUpdate) => void; // 拓扑更新
};
```

**推论:** API 设计是事件驱动的。你注册 `on('advertise', ...)` 回调，有新话题时自动触发。

#### L56-64: JSDoc — 明确告诉你需要提供 WebSocket

```typescript
/**
 * A client to interact with the Foxglove WebSocket protocol.
 * You must provide the underlying websocket client (an implementation of `IWebSocket`)
 * and that client must advertise a subprotocol which is compatible with the
 * ws-protocol spec (e.g. "foxglove.websocket.v1").
 */
export default class FoxgloveClient {
  static SUPPORTED_SUBPROTOCOL = "foxglove.websocket.v1";

  constructor({ ws }: { ws: IWebSocket }) {
    this.#ws = ws;       // ← "你传 WebSocket 进来, 我来管理协议层"
  }
}
```

**推论:**

| 源码事实 | 推论 |
|---------|------|
| 构造函数接收 `{ ws: IWebSocket }` | 不自己创建 WebSocket, 需要你传一个进来 |
| `IWebSocket` 是接口 (不是具体类) | 浏览器传原生 WebSocket, Node.js 传 `ws` npm 包的 WebSocket |
| 子协议 `foxglove.websocket.v1` | 连接时必须指定子协议 |

**→ 这就是需要 `ws` npm 包的依据:** Node.js 没有浏览器那样的全局 `WebSocket`，需要用 `ws` 这个库来创建。

#### L94-104: onmessage 处理 — 消息分类逻辑

```typescript
this.#ws.onmessage = (event) => {
  if (event.data instanceof ArrayBuffer || ArrayBuffer.isView(event.data)) {
    message = parseServerMessage(event.data);  // 二进制帧: 1字节opcode + payload
  } else {
    message = JSON.parse(event.data);           // 文本帧: JSON 控制消息
  }

  switch (message.op) {
    case "serverInfo":       // JSON: 握手
    case "advertise":        // JSON: 话题来了
    case BinaryOpcode.MESSAGE_DATA:  // 二进制: 话题数据!
      this.#emitter.emit("message", message);
    // ...
  }
};
```

**关键发现:**
- `message` 事件的 `message` 对象里, `data` 字段还是 **原始 ArrayBuffer**
- **FoxgloveClient 不解码 CDR!** 它只负责把 "这是来自 subscriptionId X 的数据帧" 告诉你
- 解码 CDR → JS 对象 需要另一个库

**→ 这就是需要 `@foxglove/rosmsg2-serialization` 的依据。**

#### L178-270: 完整的操作 API

```typescript
subscribe(channelId)                           // → subscriptionId
unsubscribe(subscriptionId)                    // 取消订阅
advertise({topic, encoding, schemaName, ...})  // → clientChannelId
unadvertise(clientChannelId)                   // 取消发布
sendMessage(clientChannelId, data: Uint8Array) // 发送消息
getParameters(names, id?)                      // 读参数
setParameters(params, id?)                     // 写参数
sendServiceCallRequest({serviceId, callId, encoding, data}) // 服务调用
```

**推论:** API 完整，覆盖了 launch.xml 里声明的所有 capabilities。

---

### 3.2 精读 `@foxglove/rosmsg2-serialization`

**阅读文件:** `README.md` (69 行) + `package.json` (64 行)

#### README.md L14-28 — MessageReader 用法

```typescript
import { MessageReader } from "@foxglove/rosmsg2-serialization";

// message definition comes from `parse()` in @foxglove/rosmsg   ← ⚠️ 关键行!
const reader = new MessageReader(messageDefinition);

// deserialize a buffer into an object
const message = reader.readMessage([0x00, 0x01, ...]);

// access message fields
message.header.stamp;
```

**信息提取:**

| 行 | 信息 | 推论 |
|-----|------|------|
| L17 | `message definition comes from parse() in @foxglove/rosmsg` | **必须同时安装 @foxglove/rosmsg** |
| L24 | `reader.readMessage([0x00, 0x01, ...])` | 输入 ArrayBuffer/Uint8Array → 输出 JS 对象 |
| L27 | `message.header.stamp` | 解码后是普通 JS 对象，可直接 . 访问 |

**→ 这就是需要 `@foxglove/rosmsg` 的依据:** README 原文写的 "comes from parse() in @foxglove/rosmsg"。

#### README.md L30-50 — MessageWriter 用法

```typescript
import { MessageWriter } from "@foxglove/rosmsg2-serialization";

// message definition comes from `parse()` in @foxglove/rosmsg   ← 又一次提到
const writer = new MessageWriter(pointStampedMessageDefinition);

// serialize the passed in object to a Uint8Array
const uint8Array = writer.writeMessage({
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "" },
  x: 1, y: 0, z: 0
});
```

**推论:** MessageWriter 负责反向操作: JS 对象 → CDR 二进制 (用于客户端 publish 和服务调用)。

#### package.json L58-61 — dependencies

```json
"dependencies": {
  "@foxglove/cdr": "^3.3.0",
  "@foxglove/message-definition": "^0.4.0",
  "@foxglove/rostime": "^1.1.2"
}
```

**信息提取:** `@foxglove/cdr` 是 `rosmsg2-serialization` 的**直接依赖**。npm install 时会自动下载，不需要手动声明在 `package.json` 里。但它会出现在 `node_modules/` 和 `package-lock.json` 中。

**→ 这就是需要 `@foxglove/cdr` 的依据:** 它是 `rosmsg2-serialization` 的内部依赖。

---

### 3.3 确认 `@foxglove/rosmsg` 的职责

npm description:
```
Parser for ROS and ROS 2 .msg definitions
```

**职责:** 把 foxglove_bridge 发来的 schema 文本 (ros2msg 格式):
```
"# JointState message\nstd_msgs/Header header\nstring[] name\nfloat64[] position\n..."
```
解析为 MessageReader 需要的结构化数组:
```javascript
[
  { name: 'header', type: 'std_msgs/Header', isComplex: true },
  { name: 'name', type: 'string', isArray: true },
  { name: 'position', type: 'float64', isArray: true },
]
```

---

### 3.4 确认 `@foxglove/cdr` 的职责

npm description:
```
Common Data Representation serialization and deserialization library
keywords: cdr dds rtps omg serialization deserialization serde ros2
```

**职责:** 最底层的 CDR 字节读写原语:
- 读取/写入: uint8, int16, int32, int64, float32, float64
- 字节对齐 (CDR 要求 4/8 字节对齐)
- little-endian 字节序处理
- string 长度前缀 + 内容

**注意:** 不需要直接 import `@foxglove/cdr`。`@foxglove/rosmsg2-serialization` 的 `MessageReader`/`MessageWriter` 内部调用它。

---

## 第四层: 实践验证 — 实际测试时发现的隐藏依赖

### 4.1 错误 1: MessageReader 构造失败

**错误代码:**
```javascript
const reader = new MessageReader(ch.schema)  // ch.schema 是字符串
```

**报错:**
```
TypeError: Expected MessageDefinition[], got string
```

**排查过程:**
1. 查看 `MessageReader` 的类型定义 → 构造函数参数声明为 `MessageDefinition[]`
2. 查看 `ch.schema` 的类型 → string (ros2msg 文本)
3. 回溯 `rosmsg2-serialization` README → "message definition comes from `parse()` in @foxglove/rosmsg"
4. 安装 `@foxglove/rosmsg` 并引入 `parse()` → 问题解决

**→ 这证实了 `@foxglove/rosmsg` 不是"可选"的，是"必须"的。**

---

### 4.2 错误 2: int64 BigInt 比较失效

**错误代码:**
```javascript
if (rsp.sum === 7) console.log('✅')
```

**现象:** `rsp.sum` 的值明明显示为 7，但判断永远为 false。

**排查过程:**
```javascript
console.log(typeof rsp.sum)  // → "bigint"
console.log(rsp.sum)         // → 7n
console.log(7n === 7)        // → false  (BigInt !== Number)
```

**修正:**
```javascript
if (Number(rsp.sum) === 7) console.log('✅')
```

**→ 这是读 `@foxglove/rosmsg2-serialization` 文档时学到的: CDR int64 → JS BigInt。**

---

### 4.3 错误 3: Float64Array 没有 .map()

**现象:** `jointState.position.map(...)` 报 `TypeError: not a function`

**排查:**
```javascript
console.log(Array.isArray(jointState.position))  // → false
console.log(jointState.position.constructor.name) // → Float64Array
```

**修正:** 用 for 循环或索引访问 `position[i]`。

**→ CDR float64[] 反序列化为 TypedArray，不是普通 Array。这也是 CDR 规范的一部分。**

---

## 最终确定的依赖清单

### 直接依赖 (4个)

| npm 包 | 安装方式 | 被发现的依据 |
|--------|---------|------------|
| `ws` | `npm install ws` | `FoxgloveClient.ts` L64: 构造函数需要 `IWebSocket`，Node.js 没有原生 WebSocket |
| `@foxglove/ws-protocol` | `npm install @foxglove/ws-protocol` | `apt-cache show` → Homepage; `npm search @foxglove` → 描述含 "Foxglove WebSocket protocol" |
| `@foxglove/rosmsg2-serialization` | `npm install @foxglove/rosmsg2-serialization` | `FoxgloveClient.ts` L94-104: message 事件 data 是原始 ArrayBuffer，需要 CDR 解码 |
| `@foxglove/rosmsg` | `npm install @foxglove/rosmsg` | `rosmsg2-serialization` README L17: "message definition comes from parse() in @foxglove/rosmsg" |

### 间接依赖 (1个，自动安装)

| npm 包 | 被谁依赖 | 安装方式 |
|--------|---------|---------|
| `@foxglove/cdr` | `@foxglove/rosmsg2-serialization` | npm install 自动 |

### 安装命令

```bash
npm install ws @foxglove/ws-protocol @foxglove/rosmsg2-serialization @foxglove/rosmsg
```

### `package.json` 最终内容

```json
{
  "name": "foxglove-bridge-validation",
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "test": "node test_all.mjs"
  },
  "dependencies": {
    "@foxglove/rosmsg": "^5.0.5",
    "@foxglove/rosmsg2-serialization": "^3.1.2",
    "@foxglove/ws-protocol": "^0.8.0",
    "ws": "^8.21.0"
  }
}
```

`@foxglove/cdr` 不出现在 `package.json` 里，但会出现在 `node_modules/` 和 `package-lock.json` 中。

---

## 搜索方法总结

| 阶段 | 工具/方法 | 用途 | 耗时 |
|------|----------|------|------|
| 发现目标 | `dpkg -l` + `apt-cache show` | 从已安装的包反向找到组织名和源码仓库 | 30秒 |
| 批量搜索 | `npm search @foxglove` | 获取该组织所有 npm 包 (19个) | 10秒 |
| 关键词筛选 | 读 npm description + keywords | 过滤出包含 "ros2/websocket/cdr" 的包 (5个候选) | 1分钟 |
| 精读源码 | Read 工具 + grep | 逐行读 FoxgloveClient.ts / README.md / package.json | 10分钟 |
| 实践验证 | 写测试代码运行 | 实际调用 API, 碰到的错误反向确认缺失的依赖 | 30分钟 |

**核心理念:** npm search 帮你找到"可能有用"的包，但**包的职责边界、谁依赖谁、怎么组合**只有读源码才能确定。README 里的 `"message definition comes from parse() in @foxglove/rosmsg"` 这种一句话就省掉了瞎猜的时间。

---

> 记录时间: 2026-06-03
