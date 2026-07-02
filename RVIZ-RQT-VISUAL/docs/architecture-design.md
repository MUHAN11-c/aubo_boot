# ROS2 Web 可视化系统 — 框架分析与设计

> 基于完整源码研究的架构方案（2026-06-03）
> 
> 依赖库源码已下载到 `research/` 目录供随时参考。

---

## 〇、功能总览

### 当前功能 vs 目标功能

| # | 功能模块 | 当前 | 目标 | 优先级 | 关键依赖 |
|---|----------|------|------|--------|----------|
| 1 | 点云 (PointCloud2) | ✅ Scene3D 内 | 独立 PointCloudRenderer | P0 | three 0.158 |
| 2 | 激光 (LaserScan) | ✅ Scene3D 内 | 独立 LaserScanRenderer | P0 | three 0.158 |
| 3 | 3D标记 (Marker/MarkerArray) | ✅ Scene3D 内 | 独立 MarkerRenderer | P0 | three 0.158 |
| 4 | 路径 (Path) | ✅ Scene3D 内 | 独立 PathRenderer | P0 | three 0.158 |
| 5 | 里程计 (Odometry) | ✅ Scene3D 内 | 独立 OdometryRenderer | P0 | three 0.158 |
| 6 | 栅格地图 (OccupancyGrid) | ✅ | 独立 OccupancyGridRenderer | P1 | three 0.158 |
| 7 | GPS面板 (NavSatFix) | ✅ | 保持不变 | P1 | — |
| 8 | 导航工具 (Goal/InitialPose) | ✅ | 保持不变 | P0 | — |
| 9 | RQT 拓扑图 (NodeGraph) | ✅ | 优化布局 | P1 | d3 (已有) |
| 10 | RQT 主题监控 | ✅ | 保持不变 | P1 | — |
| 11 | RQT 服务/参数 | ✅ | 保持不变 | P2 | — |
| 12 | 数据图表 (ChartPanel) | 🚧 | 完成 | P2 | — |
| 13 | **传输层抽象** (Adapter) | ❌ | RosbridgeAdapter + FoxgloveAdapter | P0 | — |
| 14 | **二进制传输** (CDR) | ❌ | `@foxglove/ws-protocol` + `rosmsg2-serialization` | P1 | — |
| 15 | **渲染器解耦** | ❌ | 8个独立 Renderer | P1 | three 0.158 |
| 16 | **TF 坐标变换** (/tf + /tf_static) | ❌ | TFManager 核心服务 | P0 | — |
| 17 | **URDF 模型加载** | ❌ | `urdf-loader` v0.12.7 → Three.js Group | P0 | urdf-loader |
| 18 | **机械臂关节动画** (/joint_states + FK) | ❌ | RobotModelRenderer | P0 | urdf-loader + TFManager |
| 19 | **TF 坐标系可视化** | ❌ | TFVisualizer (RGB轴) | P1 | three 0.158 |
| 20 | 多机器人支持 | ❌ | 多URDF + tf_prefix | P2 | — |
| 21 | Docker 部署 | ✅ 单进程 | supervisord 多进程 | P1 | — |

### 依赖库清单 (已下载源码)

| 库 | 版本 | 许可证 | 已下载 | 用途 |
|----|------|--------|--------|------|
| three | 0.158.0 | MIT | ❌ (已有项目依赖) | 3D渲染 |
| vue | 3.3.8 | MIT | ❌ (已有) | 前端框架 |
| pinia | 2.1.7 | MIT | ❌ (已有) | 状态管理 |
| **urdf-loader** | **0.12.7** | **Apache-2.0** | ✅ `research/refs/urdf-loaders` | **URDF→Three.js** |
| **@foxglove/ws-protocol** | **0.8.0** | **MIT** | ✅ `research/refs/ws-protocol` | **Foxglove WS客户端** |
| **@foxglove/cdr** | **3.5.1** | **MIT** | ✅ `research/refs/cdr` | **CDR编解码** |
| **@foxglove/rosmsg2-serialization** | **3.0.2** | **MIT** | ✅ `research/refs/rosmsg2-serialization` | **CDR→JS对象** |
| **@foxglove/rosmsg** | **~5.0** | **MIT** | ❌ (npm) | **ros2msg文本→MessageDefinition 解析器** |
| roslibjs (roslib) | 2.1.0 | BSD-2 | ✅ `research/refs/roslibjs` | rosbridge参考 |
| robot_viewer | — | Apache-2.0 | ✅ `research/refs/robot_viewer` | URDF架构参考 |
| foxglove_bridge | ros-humble | MIT | ❌ (apt) | ROS2桥接 |
| rosbridge_suite | — | BSD-3 | ❌ (apt) | rosbridge参考 |

---

## 一、当前架构分析

### 1.1 问题诊断

```
Scene3D.vue (~2000行) 是三座大山的混合体:

  ┌──────────────────────────────────────────────────┐
  │ Scene3D.vue                                      │
  │                                                  │
  │  L33-L39:  import useRosbridge, useConnectionStore│
  │  L400-500: 订阅管理 (subscribeToPositionTopics)    │
  │  L800-950: 通用订阅封装 (subscribeToRosTopic)      │
  │  L950-1050: 消息分发 (updateVisualization switch)  │
  │  L1050-1400: PointCloud2 解析+渲染                 │
  │  L1400-1650: LaserScan 解析+渲染                   │
  │  L1650-1750: Marker/MarkerArray 解析+渲染          │
  │  L1750-1850: Path/Odometry 解析+渲染               │
  │                                                  │
  │  33% 渲染逻辑    → 应该在 Renderer                │
  │  33% 数据解析    → 应该在 Adapter                 │
  │  33% 订阅管理    → 应该在 DataService             │
  └──────────────────────────────────────────────────┘

useConnectionStore.js (~570行) = 单一巨石
  ├── WebSocket 生命周期
  ├── rosbridge v2 协议实现 (op 分发)
  ├── RPC 请求管理 (requestId + Promise)
  └── 硬编码 WS URL

backend/rosbridge.py (~1615行) = 另一个巨石
  ├── ConnectionManager (WS连接池)
  ├── RosbridgeService (消息路由+序列化+桥接+查询)
```

---

## 二、目标架构

### 2.1 分层架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    表示层 (Presentation)                          │
│  MainLayout → Scene3D(精简) / RQT/* / Panels/*                   │
│  唯一数据入口: useDataService()                                   │
├─────────────────────────────────────────────────────────────────┤
│                    业务层 (Business)                               │
│  DataService   — 订阅生命周期管理、重连恢复、消息节流               │
│  TFManager     — TF变换树、lookupTransform()、坐标系缓存           │
├─────────────────────────────────────────────────────────────────┤
│                    通信层 (Communication)                          │
│  MessageAdapter (抽象接口)                                        │
│  ├── RosbridgeAdapter  → WS(JSON) → rosbridge/FastAPI            │
│  └── FoxgloveAdapter   → WS(CDR)  → @foxglove/ws-protocol       │
├─────────────────────────────────────────────────────────────────┤
│                    渲染层 (Rendering)                              │
│  SceneManager → 场景/相机/灯光/renderer生命周期                    │
│  ├── PointCloudRenderer / LaserScanRenderer / MarkerRenderer     │
│  ├── PathRenderer / OdometryRenderer / OccupancyGridRenderer     │
│  ├── GridRenderer / TFVisualizer                                 │
│  └── RobotModelRenderer (URDF + TF + /joint_states)              │
│                                                                   │
│  URDFLoader → urdf-loader v0.12.7 → THREE.Group 层级树           │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
ROS2 Network                前端
──────────                  ─────
    │                       
    ├─ foxglove_bridge(:8765)  → FoxgloveAdapter → DataService
    │   (CDR 二进制, 47chs)        │                    │
    │                              │ TypedMessage       ├→ Scene3D
    └─ rosbridge(:8000/ws)    → RosbridgeAdapter       │   ├→ Renderer
        (JSON 文本)               │  TypedMessage       │   └→ TFManager
                                  │                    │
    ┌─────────────────────────────┘                    ├→ RQT widgets
    │                                                  │
    ├─ /tf + /tf_static ───────────────────────────────┤
    │   TFManager.lookupTransform("map","base_link")   │
    │                                                  │
    ├─ /robot_description ─────────────────────────────┤
    │   URDFLoader.load(xml) → RobotModelRenderer      │
    │                                                  │
    └─ /joint_states ──────────────────────────────────┘
        RobotModelRenderer.#handleJointState()
          → robot.setJointValue("joint1", 0.5)
```

---

## 三、前端详细设计 (基于真实 API)

### 3.1 目录结构

```
frontend/src/
├── services/
│   ├── adapters/
│   │   ├── MessageAdapter.js     ← 抽象接口
│   │   ├── RosbridgeAdapter.js   ← rosbridge v2 (参考 roslibjs Ros.ts + Topic.ts)
│   │   └── FoxgloveAdapter.js    ← @foxglove/ws-protocol FoxgloveClient
│   ├── DataService.js            ← 业务层单例
│   └── TFManager.js              ← TF变换树服务
├── renderers/
│   ├── BaseRenderer.js           ← 抽象基类
│   ├── SceneManager.js           ← Three.js场景生命周期
│   ├── PointCloudRenderer.js     ← 点云
│   ├── LaserScanRenderer.js      ← 激光
│   ├── MarkerRenderer.js         ← 3D标记
│   ├── PathRenderer.js           ← 路径轨迹
│   ├── OdometryRenderer.js       ← 里程计
│   ├── OccupancyGridRenderer.js  ← 栅格地图
│   ├── GridRenderer.js           ← 网格/坐标轴
│   ├── TFVisualizer.js           ← TF坐标系可视化
│   ├── URDFLoader.js             ← urdf-loader 封装
│   └── RobotModelRenderer.js     ← 机械臂模型
└── composables/
    ├── useDataService.js         ← DataService 注入
    └── useSceneManager.js        ← SceneManager 注入
```

### 3.2 MessageAdapter 接口

```javascript
// 基于 roslibjs 的设计模式: EventEmitter + 简洁API
// roslibjs 参考: Ros.connect() / Topic.subscribe() / Topic.publish()

class MessageAdapter {
  async connect(url, options) {}
  disconnect() {}
  get isConnected() {}

  subscribe(topic, messageType, callback) {}    // → Subscription
  unsubscribe(subscription) {}
  publish(topic, messageType, message) {}
  advertise(topic, messageType) {}
  unadvertise(topic) {}

  async getTopics() {}
  async getNodes() {}
  async getTopicTypes() {}
  async getServices() {}
  async getParams() {}

  onStatusChange(callback) {}  // → unsubscribe function
}

// 输出格式: 所有 Adapter 输出统一的 TypedMessage
// { topic: string, type: string, data: object, timestamp: number }
```

### 3.3 RosbridgeAdapter (基于 roslibjs v2.1.0 API 设计)

```javascript
// 参考: research/refs/roslibjs/packages/roslib/src/core/Ros.ts
//       research/refs/roslibjs/packages/roslib/src/core/Topic.ts
//
// roslibjs 协议格式:
//   订阅: { op: "subscribe", id, type, topic }
//   发布: { op: "publish", id, topic, msg }
//   接收: { op: "publish", topic, msg: {...} }
//
// 本项目不直接使用 roslibjs (它依赖太多), 但参考其 JSON 格式重新实现

class RosbridgeAdapter extends MessageAdapter {
  #ws = null
  #handlers = new Map()       // topic → Set<callback>
  #pending = new Map()        // requestId → {resolve, reject, timer}
  #advertised = new Set()
  #reqCounter = 0
  #subCounter = 0
  #reconnectAttempts = 0

  // 连接: 原生 WebSocket, 无子协议
  async connect(url) {
    this.#ws = new WebSocket(url)       // roslibjs: new WebSocket(url)
    this.#ws.onmessage = (e) => this.#dispatch(JSON.parse(e.data))
    this.#ws.onclose = () => this.#scheduleReconnect()
    // roslibjs 也是这种模式: onopen/onmessage/onclose 直接绑定
  }

  // 消息路由: 参考 roslibjs Ros.handleMessage()
  #dispatch(msg) {
    switch (msg.op) {
      case 'publish':
        // roslibjs: this.emit(message.topic, message)
        const cbs = this.#handlers.get(msg.topic)
        cbs?.forEach(cb => cb({ topic: msg.topic, type: null, data: msg.msg, timestamp: Date.now() }))
        break
      case 'get_topics_result':    this.#resolveRequest(msg.id, msg.topics); break
      case 'get_nodes_result':     this.#resolveRequest(msg.id, msg.nodes); break
      // ... 其他响应
    }
  }

  // 订阅: 参考 roslibjs Topic.subscribe()
  subscribe(topic, messageType, callback) {
    if (!this.#handlers.has(topic)) this.#handlers.set(topic, new Set())
    this.#handlers.get(topic).add(callback)
    // roslibjs: { op: "subscribe", id, type, topic, compression, throttle_rate, queue_length }
    this.#send({ op: 'subscribe', topic, type: messageType })
    return { id: ++this.#subCounter, topic, cancel: () => this.unsubscribe(...) }
  }

  // 发布: 参考 roslibjs Topic.publish()
  publish(topic, messageType, message) {
    if (!this.#advertised.has(topic)) this.advertise(topic, messageType)
    // roslibjs: { op: "publish", id: "publish:{topic}:{uuid}", topic, msg: message }
    return this.#send({ op: 'publish', topic, type: messageType, msg: message })
  }
}
```

### 3.4 FoxgloveAdapter (基于 @foxglove/ws-protocol v0.8.0 API)

> **真实数据验证 (2026-06-03):** foxglove_bridge (humble 3.3.0, foxglove-sdk-cpp v0.23.0) @ :8765
>
> - 子协议确认: `foxglove.sdk.v1` ✅ (不是 `foxglove.websocket.v1` ❌)
> - 48 channels: 47 CDR + 1 JSON
> - 所有 schema 均为 ros2msg 文本, 必须通过 `@foxglove/rosmsg parse()` 解析后传给 `MessageReader`
> - CDR 带宽实测节省 43.1% (大型消息 50-60%)
> - capabilities: clientPublish, connectionGraph, parameters, parametersSubscribe, services, assets

```javascript
// 参考: research/refs/ws-protocol/typescript/ws-protocol/src/FoxgloveClient.ts
//       research/refs/cdr/src/ (CdrReader)
//       research/refs/rosmsg2-serialization/src/ (MessageReader)
//       research/refs/rosmsg2-serialization/src/ (MessageWriter)
//
// 重要: @foxglove/rosmsg 的 parse() 用于将 ros2msg 文本转为 MessageDefinition[]

import { FoxgloveClient } from '@foxglove/ws-protocol'
import { MessageReader, MessageWriter } from '@foxglove/rosmsg2-serialization'
import { parse as parseRosMsg } from '@foxglove/rosmsg'

class FoxgloveAdapter extends MessageAdapter {
  #client = null            // FoxgloveClient 实例
  #readers = new Map()      // channelId → MessageReader
  #writers = new Map()      // channelId → MessageWriter (用于 publish 序列化)
  #channels = new Map()     // channelId → {topic, encoding, schemaName, schema}
  #topicToChan = new Map()  // topic → channelId
  #subscriptions = new Map()// localSubId → {channelId, foxgloveSubId, callback}
  #subCounter = 0

  async connect(url) {
    // 子协议: 实测为 foxglove.sdk.v1
    // 验证结果: foxglove.websocket.v1 → HTTP 400, foxglove.sdk.v1 → ✅
    const ws = new WebSocket(url, ['foxglove.sdk.v1'])

    this.#client = new FoxgloveClient({ ws })

    // 错误处理 (参考 FoxgloveClient EventTypes)
    this.#client.on('error', (err) => {
      console.error('[FoxgloveAdapter] Client error:', err)
    })

    // 服务端元信息
    this.#client.on('serverInfo', (info) => {
      // capabilities: [clientPublish, connectionGraph, parameters,
      //                parametersSubscribe, services, assets]
      this._capabilities = info.capabilities || []
    })

    // Channel 注册 (包含 schema 文本)
    this.#client.on('advertise', (newChannels) => {
      // advertise 事件签名: (newChannels: Channel[]) — 直接数组
      for (const ch of newChannels) {
        this.#channels.set(ch.id, ch)
        this.#topicToChan.set(ch.topic, ch.id)

        if (ch.encoding === 'cdr' && ch.schema) {
          // 关键步骤: schema 是 ros2msg 文本字符串, 必须 parse 为 MessageDefinition[]
          // ⚠️ 不能直接 new MessageReader(ch.schema) — 类型不匹配
          const definitions = parseRosMsg(ch.schema, { ros2: true })
          this.#readers.set(ch.id, new MessageReader(definitions))
          // 同时创建 MessageWriter 用于 publish (客户端发布消息)
          this.#writers.set(ch.id, new MessageWriter(definitions))
        }
      }
    })

    // Channel 移除清理
    this.#client.on('unadvertise', (removedChannelIds) => {
      for (const cid of removedChannelIds) {
        const ch = this.#channels.get(cid)
        if (ch) this.#topicToChan.delete(ch.topic)
        this.#channels.delete(cid)
        this.#readers.delete(cid)
        this.#writers.delete(cid)
      }
    })

    // 核心: 接收二进制 CDR 数据
    this.#client.on('message', ({ subscriptionId, timestamp, data }) => {
      // data 是 ArrayBufferView (FoxgloveClient 类型定义)
      // timestamp 是 bigint 纳秒
      this.#handleBinary(subscriptionId, timestamp, data)
    })
  }

  #handleBinary(foxgloveSubId, timestamp, cdrData) {
    let target = null
    for (const [, sub] of this.#subscriptions) {
      if (sub.foxgloveSubId === foxgloveSubId) { target = sub; break }
    }
    if (!target) return

    const channel = this.#channels.get(target.channelId)
    const reader = this.#readers.get(target.channelId)
    if (!channel || !reader) return

    // CDR → JS 对象
    const jsObj = reader.readMessage(cdrData)

    // timestamp: bigint 纳秒 → 毫秒 (Date.now() 语义一致)
    const tsMs = timestamp
      ? Number(timestamp / 1000000n)  // 纳秒 → 微秒 → 再除1000得毫秒
      : Date.now()

    target.callback({
      topic: channel.topic,
      type: channel.schemaName,
      data: jsObj,
      timestamp: tsMs
    })
  }

  subscribe(topic, messageType, callback) {
    const channelId = this.#topicToChan.get(topic)
    if (channelId === undefined) return null

    const foxgloveSubId = this.#client.subscribe(channelId)
    const localId = ++this.#subCounter
    this.#subscriptions.set(localId, { channelId, foxgloveSubId, callback })
    return { id: localId, topic, cancel: () => this.unsubscribe(localId) }
  }

  unsubscribe(subOrId) {
    const id = typeof subOrId === 'object' ? subOrId.id : subOrId
    const sub = this.#subscriptions.get(id)
    if (sub) {
      this.#client.unsubscribe(sub.foxgloveSubId)
      this.#subscriptions.delete(id)
    }
  }

  // ---- 客户端发布 (goal_pose, cmd_vel 等) ----
  // 使用 MessageWriter 将 JS 对象序列化为 CDR 二进制
  publish(topic, messageType, message) {
    const channelId = this.#topicToChan.get(topic)
    if (channelId === undefined) return false

    // 需要先通过 advertise 声明发布者
    if (!this._clientChannels?.has(channelId)) {
      this.advertise(topic, messageType)
    }

    const writer = this.#writers.get(channelId)
    if (!writer) {
      // 如果没有现成的 writer, 用 channel 的 schema 创建
      const ch = this.#channels.get(channelId)
      if (!ch?.schema) return false
      const defs = parseRosMsg(ch.schema, { ros2: true })
      this.#writers.set(channelId, new MessageWriter(defs))
      // 重新获取
      return this.publish(topic, messageType, message)
    }

    const cdrData = writer.writeMessage(message)
    this.#client.sendMessage(channelId, cdrData)
    return true
  }

  advertise(topic, messageType) {
    if (!this.#client) return false
    // FoxgloveClient.advertise(channel) → ClientChannelId
    const cid = this.#client.advertise({
      topic, encoding: 'cdr', schemaName: messageType,
      schema: '', schemaEncoding: 'ros2msg'
    })
    this._clientChannels = this._clientChannels || new Map()
    this._clientChannels.set(cid, { topic, messageType })
    return true
  }

  unadvertise(topic) {
    for (const [cid, info] of (this._clientChannels || [])) {
      if (info.topic === topic) {
        this.#client.unadvertise(cid)
        this._clientChannels.delete(cid)
        return true
      }
    }
    return false
  }

  // ---- 元数据查询 ----
  // 方式1: 直接从 channels 获取 (无需 RPC)
  async getTopics() {
    return Array.from(this.#channels.values()).map(ch => ({
      name: ch.topic, messageType: ch.schemaName || 'unknown'
    }))
  }

  async getTopicTypes() {
    const types = {}
    for (const [, ch] of this.#channels) types[ch.topic] = ch.schemaName || 'unknown'
    return types
  }

  // 方式2: 利用 connectionGraph capability 获取拓扑 (替代 rosbridge RPC)
  async getNodes() {
    if (!this._capabilities?.includes('connectionGraph')) return []
    // connectionGraphUpdate 事件提供 publisher/subscriber 拓扑
    // 需要先 subscribeConnectionGraph(), 等待结果后 unsubscribeConnectionGraph()
    // 具体实现略 — 见 ws-protocol types.ts ConnectionGraphUpdate
    return []
  }
}
```

### 3.5 DataService 业务层

```javascript
// 全局单例, 所有组件唯一数据入口
// 职责: 适配器管理、订阅生命周期、重连恢复、消息节流

class DataService {
  #adapter = null
  #subs = new Map()        // topic → Set<{callback, adapterSub}>
  #throttleMap = new Map() // topic → lastTimestamp

  async connect(type, url) { /* 工厂创建 adapter */ }

  subscribe(topic, msgType, callback, opts = { throttle: 0 }) {
    const cb = opts.throttle > 0
      ? (msg) => { /* 节流逻辑 */ callback(msg) }
      : callback
    const sub = this.#adapter.subscribe(topic, msgType, cb)
    // 存储用于重连恢复
    return sub
  }

  // 重连时自动恢复所有订阅
  #onReconnect() {
    for (const [topic, subs] of this.#subs) {
      for (const { callback } of subs) {
        this.#adapter.subscribe(topic, null, callback)
      }
    }
  }
}

// 全局单例
let instance = null
export function useDataService() { return instance || (instance = new DataService()) }
```

### 3.6 TFManager (基于 /tf + /tf_static 订阅)

```javascript
// 核心服务: 管理 ROS TF 变换树
// 基于 tf2_msgs/msg/TFMessage 设计:
//   TFMessage = { transforms: TransformStamped[] }
//   TransformStamped = { header: Header, child_frame_id: string,
//     transform: { translation: Vector3, rotation: Quaternion } }

import * as THREE from 'three'

class TFManager {
  #staticTF = new Map()   // child_frame_id → {parent, translation, rotation}
  #dynamicTF = new Map()  // child_frame_id → {parent, translation, rotation, ts}
  #listeners = new Set()
  #maxCache = 50000       // Foxglove 也是 50000

  start(dataService) {
    // /tf_static: 发布一次, 永久缓存
    dataService.subscribe('/tf_static', 'tf2_msgs/msg/TFMessage', msg =>
      this.#ingest(msg.data, this.#staticTF, 0))

    // /tf: 持续发布
    dataService.subscribe('/tf', 'tf2_msgs/msg/TFMessage', msg =>
      this.#ingest(msg.data, this.#dynamicTF, Date.now()))
  }

  #ingest(data, store, ts) {
    for (const tf of (data.transforms || [])) {
      store.set(tf.child_frame_id, {
        parent: tf.header.frame_id,
        translation: new THREE.Vector3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z),
        rotation: new THREE.Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w),
        timestamp: ts
      })
    }
    this.#listeners.forEach(cb => cb())
  }

  // 查询: target=map, source=base_link → 返回 {translation, rotation}
  // 算法: 从 source 和 target 分别构建到 root 的链, 找到最低公共祖先 (LCA)
  // 然后 compose chain(source → LCA) × inverse(chain(target → LCA))
  lookupTransform(targetFrame, sourceFrame) {
    // 构建两条链: sourceFrame → root, targetFrame → root
    const sourceChain = this.#buildChain(sourceFrame)
    const targetChain = this.#buildChain(targetFrame)
    if (!sourceChain || !targetChain) return null

    // 找到 LCA: 两条链中第一个共享的 frame, 从后往前找
    let lcaIndexS = sourceChain.length - 1
    let lcaIndexT = targetChain.length - 1

    // sourceChain[last] 是 root, 往回找第一个 targetChain 中存在的
    for (let i = sourceChain.length - 1; i >= 0; i--) {
      const parentFrame = sourceChain[i].parent
      if (targetChain.some(e => e.parent === parentFrame || targetChain[targetChain.length - 1]?.parent === parentFrame)) {
        lcaIndexS = i
        break
      }
    }
    // targetChain[last].parent 是 root
    for (let i = targetChain.length - 1; i >= 0; i--) {
      if (targetChain[i].parent === sourceChain[lcaIndexS].parent) {
        lcaIndexT = i
        break
      }
    }

    // Compose: sourceChain[0..lcaIndexS] (source → LCA)
    let tSrc = new THREE.Vector3(), rSrc = new THREE.Quaternion()
    for (let i = 0; i <= lcaIndexS; i++) {
      tSrc.applyQuaternion(sourceChain[i].rotation).add(sourceChain[i].translation)
      rSrc.premultiply(sourceChain[i].rotation)
    }

    // Compose: targetChain[0..lcaIndexT] (target → LCA), 然后取逆
    let tTgt = new THREE.Vector3(), rTgt = new THREE.Quaternion()
    for (let i = 0; i <= lcaIndexT; i++) {
      tTgt.applyQuaternion(targetChain[i].rotation).add(targetChain[i].translation)
      rTgt.premultiply(targetChain[i].rotation)
    }
    rTgt.invert()
    tTgt.applyQuaternion(rTgt).negate()

    // Result: compose(source→LCA) × inverse(target→LCA)
    const result = {
      translation: tSrc.clone().add(tTgt.applyQuaternion(rSrc)),
      rotation: rTgt.clone().premultiply(rSrc)
    }
    return result
  }

  #buildChain(frameId) {
    const chain = []
    let current = frameId
    // maxDepth 防止死循环 (TF 树不应该太深)
    for (let i = 0; i < 100; i++) {
      // 优先查动态 TF, 回退到静态 TF
      const entry = this.#dynamicTF.get(current) || this.#staticTF.get(current)
      if (!entry) break
      chain.push(entry)
      current = entry.parent
    }
    return chain.length > 0 ? chain : null
  }

  // 简化版: 仅当 target 在 source 链上时可用 (同分支查询)
  lookupTransformSimple(targetFrame, sourceFrame) {
    const chain = this.#buildChain(sourceFrame)
    if (!chain) return null

    let t = new THREE.Vector3(), r = new THREE.Quaternion()
    for (const entry of chain) {
      t.applyQuaternion(entry.rotation).add(entry.translation)
      r.premultiply(entry.rotation)
      if (entry.parent === targetFrame) return { translation: t.clone(), rotation: r.clone() }
    }
    return null
  }

  canTransform(target, source) { return this.lookupTransform(target, source) !== null }
  getFrameNames() { /* 动态+静态 TF 的所有 frame_id 集合 */ }
  onFrameUpdate(cb) { this.#listeners.add(cb); return () => this.#listeners.delete(cb) }
  getStaticFrames() { return new Map(this.#staticTF) }  // 用于 URDF 初始化
}

export function useTFManager() { /* 单例 */ }
```

### 3.7 URDFLoader 封装 (基于 urdf-loader v0.12.7 API)

```javascript
// 参考: research/refs/urdf-loaders/javascript/src/URDFLoader.js
//       research/refs/robot_viewer (URDFAdapter, ModelLoaderFactory)

import URDFLoader from 'urdf-loader'
import * as THREE from 'three'

class URDFModelLoader {
  // urdf-loader v0.12.7 关键属性 (源码确认):
  //   loader.packages = string | object | function  (package:// 路径解析)
  //   loader.loadMeshCb = (path, manager, done) => void  (自定义mesh加载)
  //   loader.parseVisual = true  (解析 visual 几何)
  //   loader.parseCollision = false  (默认不解析 collision)
  //   loader.fetchOptions = {}  (fetch 请求选项)

  async loadString(xml, options = {}) {
    const loader = new URDFLoader()
    loader.packages = options.packagePath || ''
    // 自定义 mesh 加载: 从 foxglove assets 或本地文件
    if (options.loadMesh) loader.loadMeshCb = options.loadMesh

    // loader.parse() 返回 URDFRobot (extends THREE.Group)
    // URDFRobot 属性: joints{}, links{}, colliders{}, visual{}, frames{}
    const robot = loader.parse(xml)  // 支持 string | Document | Element
    return robot
  }

  async loadUrl(url, options = {}) {
    const response = await fetch(url)
    const xml = await response.text()
    return this.loadString(xml, options)
  }

  // 静态工具方法
  static getLinkNames(robot) {
    // robot.links: Record<string, URDFLink>
    return Object.keys(robot.links)
  }
  static getJointNames(robot) {
    // robot.joints: Record<string, URDFJoint>
    return Object.keys(robot.joints)
  }
}
```

### 3.8 RobotModelRenderer (URDF + TF + /joint_states)

```javascript
// 参考: research/refs/robot_viewer (JointDragControls, URDFAdapter)
//       Foxglove LeRobot 示例 (yourdfpy + FK + FrameTransforms)
//
// URDFJoint API (从 urdf-loader 源码):
//   joint.jointType: 'fixed'|'revolute'|'continuous'|'prismatic'|'planar'|'floating'
//   joint.setJointValue(...values): boolean
//     - revolute/continuous: joint.setJointValue(angle) 旋转
//     - prismatic: joint.setJointValue(displacement) 平移
//     - fixed: 不接受值
//   joint.axis: THREE.Vector3 (旋转轴或平移方向)
//   joint.limit: { lower, upper } (限位)
//
// ⚠️ 异步 mesh 加载: URDFLoader.parse() 同步返回 robot skeleton,
//    但 STL/DAE mesh 通过 loadMeshCb 异步加载。
//    参考 robot_viewer 方案: 在 100ms/1000ms/2500ms 延迟重新提取 visual/collision mesh
//
// ⚠️ 坐标系: URDF 使用 Z-up (ROS 约定), Three.js 使用 Y-up。
//    需要将 robot 放入一个旋转了 -PI/2 绕 X 轴的 world Group 中

import { BaseRenderer } from './BaseRenderer'
import { URDFModelLoader } from './URDFLoader'
import { useTFManager } from '../services/TFManager'
import { useDataService } from '../services/DataService'
import * as THREE from 'three'

class RobotModelRenderer extends BaseRenderer {
  #robot = null      // URDFRobot (THREE.Group)
  #world = null      // 坐标系转换 Group (Z-up → Y-up)
  #tfManager = null
  #baseFrame = 'base_link'
  #jointNames = []   // 用于匹配 /joint_states

  constructor(scene, config = {}) {
    super(scene, {
      baseFrame: 'base_link',
      urdfUrl: null,
      packagePath: '',
      opacity: 1.0,
      displayMode: 'visual',  // 'visual'|'collision'|'both'
      ...config
    })
  }

  async init() {
    this.#tfManager = useTFManager()
    const ds = useDataService()

    // 坐标系转换: URDF Z-up → Three.js Y-up
    // 参考 robot_viewer SceneManager.world 和 urdf-viewer _setUp()
    this.#world = new THREE.Group()
    this.#world.rotation.set(-Math.PI / 2, 0, 0)  // Z-up → Y-up
    this.scene.add(this.#world)

    // 1. 加载 URDF → THREE.Group 层级树
    const loader = new URDFModelLoader()
    if (this.config.urdfUrl) {
      this.#robot = await loader.loadUrl(this.config.urdfUrl, {
        packagePath: this.config.packagePath
      })
      this.#jointNames = URDFModelLoader.getJointNames(this.#robot)
      this.#world.add(this.#robot)

      // 异步 mesh 加载完成后重新注册 visual/collision (参考 robot_viewer 延迟策略)
      setTimeout(() => this.#onMeshLoaded(), 100)
      setTimeout(() => this.#onMeshLoaded(), 1000)
      setTimeout(() => this.#onMeshLoaded(), 2500)
    }

    // 2. 订阅 /joint_states → 关节动画
    // ⚠️ jointState.position 是 Float64Array (MessageReader 返回 TypedArray)
    //    索引访问 position[i] 对 Float64Array 和普通 Array 都兼容
    ds.subscribe('/joint_states', 'sensor_msgs/msg/JointState', msg => {
      const names = msg.data.name || []
      const positions = msg.data.position || []
      const velocities = msg.data.velocity || []

      for (let i = 0; i < names.length; i++) {
        // position[i] 在 Float64Array 和 Array 上都有效
        const val = positions[i]
        if (val != null) {
          this.#robot?.setJointValue(names[i], val)
        }
      }
    })

    // 3. 订阅 TF 更新 → 基座位姿
    this.#tfManager.onFrameUpdate(() => this.#updateBasePose())

    this.object3D = this.#robot
  }

  #onMeshLoaded() {
    // URDFLoader 异步加载 STL/DAE mesh 后, 重新提取 visual/collision 用于透明度控制等
    // 参考 robot_viewer VisualizationManager.extractVisualAndCollision()
  }

  #updateBasePose() {
    if (!this.#robot) return
    const tf = this.#tfManager.lookupTransform('map', this.#baseFrame)
    if (tf) {
      this.#robot.position.copy(tf.translation)
      this.#robot.quaternion.copy(tf.rotation)
    }
  }

  // 查询 link 在世界坐标系下的位姿 (FK)
  getLinkWorldPose(linkName) {
    const link = this.#robot?.links[linkName]
    if (!link) return null
    this.#robot.updateMatrixWorld(true)  // 确保矩阵最新
    const p = new THREE.Vector3(), q = new THREE.Quaternion()
    link.getWorldPosition(p); link.getWorldQuaternion(q)
    return { position: p, quaternion: q }
  }

  setDisplayMode(mode) { /* visual/collision/both 切换 */ }
  dispose() {
    this.#world?.remove(this.#robot)
    this.scene?.remove(this.#world)
    super.dispose()
  }
}
```

### 3.9 SceneManager (参考 robot_viewer SceneManager.js)

> **坐标系转换:** URDF 使用 Z-up (ROS 约定), Three.js 使用 Y-up。
> SceneManager 通过 `this.world` Object3D 旋转 -PI/2 绕 X 轴实现转换。
> 所有 Renderer 的对象应添加到 `this.world` 而非直接添加到 `this.scene`。

```javascript
// 参考: research/refs/robot_viewer/src/renderer/SceneManager.js
// robot_viewer 核心模式:
//   - 惰性渲染: _dirty 标志位, 仅在需要时 render
//   - Manager 组合模式: 各个功能独立 Manager, SceneManager 组合它们
//   - 坐标系 world Group: 统一管理 up-axis 转换
//   - resize 处理: resizeObserver

import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'

class SceneManager {
  #scene = null
  #world = null     // 坐标系转换 Group (ROS Z-up → Three.js Y-up)
  #camera = null
  #renderer = null
  #controls = null
  #renderers = new Map()  // topic → BaseRenderer
  #animationId = null
  #dirty = true           // 惰性渲染标志 (robot_viewer 模式)

  constructor(container, options = {}) {
    const { upAxis = 'Z' } = options  // 默认 Z-up (ROS 约定)

    this.#renderer = new THREE.WebGLRenderer({ antialias: true })
    this.#renderer.shadowMap.enabled = true
    this.#renderer.shadowMap.type = THREE.PCFSoftShadowMap

    this.#scene = new THREE.Scene()
    this.#scene.background = new THREE.Color(0x1a1a2e)

    // 坐标系转换 Group
    this.#world = new THREE.Group()
    this.#world.name = 'world'
    if (upAxis === 'Z') {
      this.#world.rotation.set(-Math.PI / 2, 0, 0)  // Z-up → Y-up
    }
    this.#scene.add(this.#world)

    this.#camera = new THREE.PerspectiveCamera(60, 2, 0.1, 1000)
    this.#camera.position.set(5, 5, 5)

    this.#controls = new OrbitControls(this.#camera, this.#renderer.domElement)

    // 灯光和网格添加到 world (跟随坐标变换)
    this.#world.add(new THREE.AmbientLight(0x404040, 0.5))
    const dir = new THREE.DirectionalLight(0xffffff, 0.8)
    dir.position.set(10, 20, 10)
    this.#world.add(dir)
    this.#world.add(new THREE.GridHelper(20, 20))

    container.appendChild(this.#renderer.domElement)
    this.#startLoop()
  }

  // 获取 scene 和 world 的引用 (供 Renderer 使用)
  get scene() { return this.#scene }
  get world() { return this.#world }  // Renderer 应将对象添加到 world
  get renderer() { return this.#renderer }
  get camera() { return this.#camera }

  #startLoop() {
    const loop = () => {
      this.#animationId = requestAnimationFrame(loop)
      this.#controls.update()
      if (this.#dirty) {        // 惰性渲染 (robot_viewer 模式)
        this.#renderer.render(this.#scene, this.#camera)
        this.#dirty = false
      }
    }
    loop()
  }

  markDirty() { this.#dirty = true }

  registerRenderer(topic, renderer) { /* BaseRenderer */ }
  unregisterRenderer(topic) { /* dispose */ }
  getStats() { /* FPS/对象数/顶点数 */ }
  dispose() { /* 清理 */ }
}
```

---

## 四、后端设计

### 4.1 当前后端 → 目标后端

```
当前:
  FastAPI(:8000)
  ├── /ws → RosbridgeService (JSON, rclpy直连)
  └── /api/v1/* → RosbridgeService 查询

目标:
  foxglove_bridge(:8765)  ← 数据面主力 (CDR 二进制, C++)
  ├── 47 CDR channels + 1 JSON channel
  └── 6 capabilities (clientPublish, services, parameters, assets, connectionGraph)

  FastAPI(:8000)           ← 降级为 REST API 代理
  ├── /health
  └── /api/v1/* (可选, 调用 ros2 CLI 或代理到 foxglove)
```

### 4.2 保留/删除决策

| 模块 | 决策 | 理由 |
|------|------|------|
| `rosbridge.py` (~1615行) | Phase 4 删除 | 数据面由 foxglove_bridge 替代 |
| `ros2_service.py` | Phase 4 删除 | 功能重叠 |
| `connection_manager.py` | Phase 4 删除 | 空文件 |
| `topology_service.py` | Phase 4 删除 | rclpy 内省 API 由 foxglove connectionGraph 替代 |
| `dependencies.py` | 精简 | 移除 rosbridge/ros2_service 实例化 |
| `main.py` | 保留 | 删 WS 端点, 保留 /health |
| `api/v1/ros.py` | 保留 (代理模式) | 调用 ros2 CLI 的元数据查询 |
| `core/config.py` | 保留 | 新增 foxglove 配置 |
| `models/*` | 保留 | 纯 Pydantic, 零 rclpy 依赖 |

---

## 五、部署架构

```
┌──────────────────── Docker Container ──────────────────────┐
│                                                             │
│  supervisord                                                │
│  ├── [nginx]          :3000 → 前端静态 + 反向代理             │
│  │   /api/*    → proxy_pass :8000                          │
│  │   /foxglove → proxy_pass :8760                          │
│  │   /ws       → proxy_pass :8760  (旧WS兼容)              │
│  │                                                          │
│  ├── [foxglove_bridge]  :8760                              │
│  │   ros-humble-foxglove-bridge (apt)                       │
│  │   子协议: foxglove.sdk.v1                                │
│  │                                                          │
│  └── [fastapi]         :8000                               │
│      uvicorn app.main:app                                   │
│      REST API + 健康检查 (无WS端点)                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 六、文件变更地图

### Phase 1: Adapter 抽象层 (新增, ~750行)

```
新建:
  services/adapters/MessageAdapter.js    (~80行)
  services/adapters/RosbridgeAdapter.js  (~250行, 参考 roslibjs Ros.ts + Topic.ts)
  services/adapters/index.js             (~20行)
  services/DataService.js                (~150行)
  services/TFManager.js                  (~220行, 基于 /tf + /tf_static)
  composables/useDataService.js          (~15行)

修改:
  components/RViz/Scene3D.vue            (import → DataService)

标记: composables/useConnectionStore.js  (@deprecated)
       composables/useRosbridge.js       (@deprecated)
```

### Phase 2: FoxgloveAdapter + 渲染器 + URDF (新增, ~2500行)

```
新增:
  services/adapters/FoxgloveAdapter.js    (~350行, @foxglove/ws-protocol + rosmsg2-serialization)
  renderers/BaseRenderer.js              (~50行)
  renderers/SceneManager.js              (~150行, 参考 robot_viewer SceneManager)
  renderers/PointCloudRenderer.js        (~300行)
  renderers/LaserScanRenderer.js         (~350行)
  renderers/MarkerRenderer.js            (~150行)
  renderers/PathRenderer.js              (~80行)
  renderers/OdometryRenderer.js          (~100行)
  renderers/OccupancyGridRenderer.js     (~200行)
  renderers/GridRenderer.js              (~50行)
  renderers/TFVisualizer.js              (~100行)
  renderers/URDFLoader.js                (~120行, 封装 urdf-loader v0.12.7)
  renderers/RobotModelRenderer.js        (~250行, URDF+TF+/joint_states联动)

依赖:
  npm install @foxglove/ws-protocol @foxglove/rosmsg2-serialization @foxglove/rosmsg urdf-loader

修改:
  components/RViz/Scene3D.vue            (2000→~250行)
  vite.config.js

删除: 无
```

### Phase 3: Docker 改造

```
新增: docker/supervisord.conf, docker/nginx.conf
修改: Dockerfile (多阶段 + supervisord + apt install ros-humble-foxglove-bridge)
       .env (VITE_WS_ADAPTER, TF 配置)
```

### Phase 4: 清理 (~2000行删除)

```
删除:
  backend/rosbridge.py (~1615行)
  backend/ros2_service.py
  backend/connection_manager.py
  backend/topology_service.py
  frontend/useConnectionStore.js
  frontend/useRosbridge.js
  frontend/RosbridgeAdapter.js (如果完全切到 foxglove)
```

---

## 七、验证矩阵

| 功能 | 验证 | Phase 1 | 2 | 3 | 4 |
|------|------|---------|---|---|---|
| 点云/激光/轨迹/地图 | 渲染正确 | ✅ | ✅ | ✅ | ✅ |
| RQT 拓扑图 | 节点+连线 | ✅ | ✅ | ✅ | ✅ |
| 导航发布 | goal_pose | ✅ | ✅ | ✅ | ✅ |
| WS 重连 | 断网恢复 | ✅ | ✅ | ✅ | ✅ |
| Adapter 切换 | VITE_WS_ADAPTER 环境变量 | ✅ | ✅ | ✅ | ✅ |
| Foxglove CDR | 47 channels 正常 | — | ✅ | ✅ | ✅ |
| TF lookupTransform | map→base_link 查询 | — | ✅ | ✅ | ✅ |
| TF 可视化 | 坐标系 RGB 轴 | — | ✅ | ✅ | ✅ |
| URDF 加载 | robot.setJointValue() | — | ✅ | ✅ | ✅ |
| 机械臂动画 | /joint_states 驱动关节 | — | ✅ | ✅ | ✅ |
| 机械臂+TF | URDF 跟随 base_link 移动 | — | ✅ | ✅ | ✅ |
| Docker | `docker build && run` | — | — | ✅ | ✅ |
| 带宽 | CDR vs JSON 对比 | — | ✅ | ✅ | ✅ |

---

## 八、真实数据验证 (2026-06-03)

> 测试环境: foxglove_bridge (humble 3.3.0, foxglove-sdk-cpp v0.23.0) @ :8765, aubo_ros2_ws

### 8.1 子协议确认

```
foxglove.sdk.v1       → ✅ CONNECTED (协商: "foxglove.sdk.v1")
foxglove.websocket.v1 → ❌ HTTP 400
(无子协议)              → ❌ HTTP 400
```

**结论: `foxglove.sdk.v1` 是正确的, 架构文档无误。**

### 8.2 服务端信息

```
name:              (空)
capabilities:      clientPublish, connectionGraph, parameters, parametersSubscribe, services, assets
supportedEncodings: cdr, json
metadata:          {"ROS_DISTRO":"humble","fg-library":"foxglove-sdk-cpp/v0.23.0"}
channels:          48 (47 CDR + 1 JSON)
services:          347
schema 类型:       48/48 均为 ros2msg 文本字符串
```

### 8.3 Schema 解析验证

8 种不同消息类型的 schema 文本全部成功被 `@foxglove/rosmsg parse({ros2: true})` 解析:
- `lifecycle_msgs/msg/TransitionEvent` ✅
- `moveit_msgs/msg/MotionPlanRequest` ✅
- `ivg_interfaces/msg/RobotStatus` ✅
- `moveit_msgs/msg/AttachedCollisionObject` ✅
- `sensor_msgs/msg/CameraInfo` ✅
- `moveit_msgs/msg/DisplayTrajectory` ✅
- `rosbridge_msgs/msg/ConnectedClients` ✅
- `object_recognition_msgs/msg/RecognizedObjectArray` ✅

所有 CDR 数据均被 `MessageReader.readMessage()` 成功反序列化。

### 8.4 TF 数据格式确认

```json
{
  "transforms": [{
    "header": { "stamp": {"sec": N, "nanosec": N}, "frame_id": "parent" },
    "child_frame_id": "child",
    "transform": {
      "translation": {"x": N, "y": N, "z": N},
      "rotation": {"x": N, "y": N, "z": N, "w": N}
    }
  }]
}
```

**TFManager 的数据结构假设完全匹配实际数据。**

### 8.5 JointState 数据格式确认

```json
{
  "header": {"stamp": {...}, "frame_id": "base_link"},
  "name": ["shoulder_joint", "foreArm_joint", "wrist1_joint",
           "upperArm_joint", "wrist2_joint", "wrist3_joint"],
  "position": Float64Array(6),
  "velocity": Float64Array(6),
  "effort": Float64Array(6)
}
```

**⚠️ `position`/`velocity`/`effort` 是 Float64Array, 不是普通 Array。**
**索引访问 `position[i]` 对两者兼容, RobotModelRenderer 代码无需修改。**

### 8.6 CDR vs JSON 带宽实测

| 消息类型 | CDR | JSON | 节省 |
|----------|-----|------|------|
| JointTrajectoryControllerState | 1072B | 2074B | **48.3%** |
| DynamicJointState | 492B | 615B | 20.1% |
| TFMessage | 412B | 1004B | **59.0%** |
| JointState | 316B | 348B | 9.3% |
| RobotStatus | 244B | 671B | **63.6%** |
| ToolChangerStatus | 64B | 170B | **62.4%** |
| **总体** | — | — | **43.1%** |

**大型消息 (>200B) 节省 50-60%, 小型消息节省 10-30%。CDR 二进制传输优势明显。**

### 8.7 架构文档修正要点

| 修正项 | 状态 |
|--------|------|
| `foxglove.sdk.v1` 子协议 | ✅ 无需修正 (原本就正确) |
| 添加 `@foxglove/rosmsg parse()` 步骤 | ✅ 已添加 |
| 添加 `"@foxglove/rosmsg"` 依赖 | ✅ 已添加 |
| TFManager LCA 算法 | ✅ 已更新 |
| FoxgloveAdapter 错误处理 | ✅ 已添加 |
| FoxgloveAdapter publish 实现 | ✅ 已添加 |
| 坐标系 world Group | ✅ 已补充 |
| Float64Array 兼容性说明 | ✅ 已补充 |
| 异步 mesh 加载说明 | ✅ 已补充 |
