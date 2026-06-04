# foxglove_bridge 完整数据验证测试套件

> ROS2 Humble + foxglove_bridge (3.3.0) 真实环境验证
> 智能自动发现，零硬编码

---

## 零、`package.json` 是怎么生成的

整个项目的起点只有一台装了 `ros-humble-foxglove-bridge` 的机器，不知道要用什么 npm 包。

### 第一步: 从系统反向找到目标组织

```bash
dpkg -l | grep foxglove                    # → ros-humble-foxglove-bridge 已安装
apt-cache show ros-humble-foxglove-bridge  # → Homepage: github.com/foxglove/foxglove-sdk
```

得知这是 **Foxglove** 组织的项目。npm 上对应的组织名是 `@foxglove`。

### 第二步: 批量搜 @foxglove 所有包

```bash
npm search @foxglove   # → 输出 19 个包
```

按 keywords (ros2 / websocket / cdr / serialization) 筛选出候选:

```
@foxglove/ws-protocol            — "Foxglove WebSocket protocol"     ← 客户端
@foxglove/rosmsg2-serialization  — "ROS 2 message serialization"      ← CDR 解码
@foxglove/rosmsg                 — "Parser for ROS .msg definitions"  ← schema 解析
@foxglove/cdr                    — "Common Data Representation ..."   ← 字节原语
```

### 第三步: 读源码确认依赖关系

读 `@foxglove/ws-protocol` 的 `FoxgloveClient.ts`:

- L64: 构造函数需要 `IWebSocket` → Node.js 没有浏览器的 `WebSocket` → 需要 `ws`
- L94-104: `message` 事件 data 是原始 `ArrayBuffer`, FoxgloveClient 不解码 CDR → 需要 `@foxglove/rosmsg2-serialization`

读 `@foxglove/rosmsg2-serialization` 的 `README.md`:

- L17: `"message definition comes from parse() in @foxglove/rosmsg"` → 需要 `@foxglove/rosmsg`

读 `@foxglove/rosmsg2-serialization` 的 `package.json`:

- dependencies 字段: `"@foxglove/cdr": "^3.3.0"` → `@foxglove/cdr` 是间接依赖

### 第四步: 执行安装

```bash
npm install ws @foxglove/ws-protocol @foxglove/rosmsg2-serialization @foxglove/rosmsg
```

这条命令同时做了三件事:
1. 下载这 4 个包 (及其依赖 `@foxglove/cdr`) → `node_modules/`
2. 记录精确版本和依赖树 → `package-lock.json`
3. 把依赖写入 → `package.json` 的 `dependencies` 字段

最后手动加上 `"type": "module"` 和 `"scripts"`，就得到现在的 `package.json`:

```json
{
  "name": "foxglove-bridge-validation",
  "version": "1.0.0",
  "description": "foxglove_bridge 真实数据验证测试套件",
  "type": "module",
  "scripts": {
    "test": "node test_all.mjs",
    "validate": "node test_all.mjs"
  },
  "keywords": ["foxglove", "ros2", "cdr", "validation"],
  "author": "",
  "license": "MIT",
  "dependencies": {
    "@foxglove/cdr": "^3.5.1",
    "@foxglove/rosmsg": "^5.0.5",
    "@foxglove/rosmsg2-serialization": "^3.1.2",
    "@foxglove/ws-protocol": "^0.8.0",
    "ws": "^8.21.0"
  }
}
```

### 每个字段的含义

#### 项目描述 (给人和工具看的)

| 字段 | 值 | 含义 |
|------|-----|------|
| `name` | `foxglove-bridge-validation` | 项目名称。npm 用来标识这个包。命名规则: 小写字母 + 连字符，不能有空格 |
| `version` | `1.0.0` | 版本号。遵循[语义化版本](https://semver.org/lang/zh-CN/): `主版本.次版本.修订号` |
| `description` | `foxglove_bridge 真实数据验证测试套件` | 项目描述。`npm search` 时会被搜索到 |
| `keywords` | `["foxglove","ros2","cdr","validation"]` | 关键字数组。`npm search` 的搜索依据，方便别人找到这个包 |
| `author` | `""` | 作者。可以是字符串 `"名字 <邮箱>"` 或空字符串 |
| `license` | `"MIT"` | 开源许可证。MIT = 最宽松，随便用、改、商用 |

#### 运行配置

| 字段 | 值 | 含义 |
|------|-----|------|
| `type` | `"module"` | **告诉 Node.js 用 ES Module 模式**。有了它才能用 `import` 语法。没有的话只能 `require()`。对应 `.mjs` 文件扩展名 |
| `scripts` | `{"test": "node test_all.mjs", "validate": "node test_all.mjs"}` | **快捷命令别名**。`npm test` 实际上执行 `node test_all.mjs`。`npm run validate` 同理 |

`scripts` 是使用方式:

```bash
npm test              # → 等价于 node test_all.mjs
npm run validate      # → 等价于 node test_all.mjs (同一个命令)
```

`type: "module"` 为什么重要:

```javascript
// ✅ 有 "type": "module" → 可以用 import
import WebSocket from 'ws';
import { FoxgloveClient } from '@foxglove/ws-protocol';

// ❌ 没有 → 报错: Cannot use import statement outside a module
//    只能用 const WebSocket = require('ws')
```

#### 依赖列表

| 字段 | 值 | 含义 |
|------|-----|------|
| `dependencies` | 5 个 npm 包 + 版本号 | **运行时必须的第三方库**。npm install 时下载到 `node_modules/` |

```json
"dependencies": {
  "@foxglove/cdr": "^3.5.1",                // CDR 二进制底层编解码
  "@foxglove/rosmsg": "^5.0.5",              // ros2msg 文本 → 结构化定义
  "@foxglove/rosmsg2-serialization": "^3.1.2", // CDR ↔ JS 对象转换器
  "@foxglove/ws-protocol": "^0.8.0",          // Foxglove WebSocket 客户端
  "ws": "^8.21.0"                             // Node.js WebSocket 实现
}
```

版本号前的 `^` 符号含义:

| 写法 | 含义 | 举例: `^3.5.1` |
|------|------|---------------|
| `^3.5.1` | **兼容 3.x.x 的最新版本** | 允许 `3.5.2`, `3.6.0`, 不允许 `4.0.0` |
| `~3.5.1` | **兼容 3.5.x 的最新版本** | 允许 `3.5.2`, 不允许 `3.6.0` |
| `3.5.1` | **锁定这个版本** | 不允许任何更新 |
| `*` | **任何版本** | 不推荐 |

#### 代码里怎么用这些依赖

```javascript
// package.json 里声明了这些包 → npm install 下载到 node_modules/
// → 代码里 import 就能用了

import WebSocket from 'ws';                          // ← "ws": "^8.21.0"
import { FoxgloveClient } from '@foxglove/ws-protocol';  // ← "@foxglove/ws-protocol"
import { MessageReader, MessageWriter } from '@foxglove/rosmsg2-serialization';
import { parse as parseRosMsg } from '@foxglove/rosmsg';
// @foxglove/cdr 是间接依赖, rosmsg2-serialization 内部使用, 不要直接 import
```

#### `package.json` / `package-lock.json` / `node_modules` 关系

```
package.json          → 你写的 "购物清单" (handwritten)
    │
    │  npm install
    │
    ├── node_modules/  → 下载的 "实物" (自动生成)
    └── package-lock.json → "精确账单" (自动生成, 锁定每个包的精确版本)
```

完整搜索过程见 [`SEARCH_PROCESS.md`](SEARCH_PROCESS.md)。

---

## 一、目录总览

```
research/validation/
├── README.md              ← 本文件 (使用指南)
├── PROTOCOL.md            ← Foxglove WebSocket 协议完整文档
├── SEARCH_PROCESS.md      ← npm 依赖库搜索查找全过程
├── test_all.mjs           ← 综合测试脚本 (唯一测试入口)
├── gen_topology_graph.mjs ← 拓扑可视化生成脚本
├── full_analysis.json     ← 全量数据采集结果 (自动生成)
├── topology.html          ← 交互式 ROS2 拓扑图 (自动生成)
├── topology.dot           ← Graphviz 拓扑图源文件 (自动生成)
├── topology.svg           ← 静态拓扑矢量图 (自动生成)
├── start_demo_nodes.sh    ← ROS2 官方 demo 节点启动脚本
├── package.json           ← npm 配置 (5 个依赖)
├── package-lock.json      ← npm 自动生成的精确版本锁
└── node_modules/          ← 第三方依赖库 (npm install 生成)
```

### 文档说明

| 文件 | 内容 | 适合谁 |
|------|------|--------|
| `README.md` | 使用方法、环境搭建、排查指南 | 使用者 |
| `PROTOCOL.md` | JSON 控制帧 + CDR 二进制帧 + topology 完整协议分析 | 开发者 |
| `SEARCH_PROCESS.md` | 从 `apt-cache show` → `npm search` → 读源码 的完整搜索链路 | 学习者 |
| `test_all.mjs` | 测试源码，可直接阅读 | 开发者 |
| `gen_topology_graph.mjs` | 从全量数据生成交互式拓扑图 | 开发者 |
| `full_analysis.json` | 全量数据采集结果 (JSON) | 数据分析师 |
| `topology.html` | 交互式 ROS2 节点拓扑图 (类似 rqt_graph) | 所有人 |

---

## 二、环境要求

| 组件 | 版本 | 检查命令 |
|------|------|---------|
| ROS2 | Humble | `echo $ROS_DISTRO` |
| foxglove_bridge | 3.3.0 | `ros2 pkg list \| grep foxglove_bridge` |
| Node.js | >= 20 | `node --version` |
| npm | >= 9 | `npm --version` |

### 外部环境 (测试时需要运行 ROS2 + foxglove_bridge)

运行实际数据时必须:

```bash
# 方式1: 连接已有的机械臂/机器人 ROS2 环境 (完整数据)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# 方式2: 仅收发测试 (官方 demo 节点)
bash start_demo_nodes.sh
```

---

## 三、快速开始

```bash
# 1. 进入目录
cd research/validation

# 2. 安装依赖 (首次)
npm install

# 3. 确保 foxglove_bridge 运行中 (另一个终端)
#    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# 4. 运行测试 (自动生成 full_analysis.json)
npm test

# 5. 生成拓扑可视化 (可选)
node gen_topology_graph.mjs
python3 -m http.server 9876
# 浏览器打开 http://localhost:9876/topology.html
```

---

## 四、npm 依赖详解

### 4.1 依赖清单

| npm 包 | 版本 | 直接/间接 | 职责 |
|--------|------|----------|------|
| `ws` | ^8.21.0 | 直接 | Node.js WebSocket 实现 (浏览器有原生的，Node 没有) |
| `@foxglove/ws-protocol` | ^0.8.0 | 直接 | Foxglove WebSocket 协议客户端 (FoxgloveClient) |
| `@foxglove/rosmsg2-serialization` | ^3.1.2 | 直接 | CDR 二进制 ↔ JS 对象 (MessageReader/MessageWriter) |
| `@foxglove/rosmsg` | ^5.0.5 | 直接 | ros2msg 文本解析器 (`.msg` → `MessageDefinition[]`) |
| `@foxglove/cdr` | ^3.5.1 | **间接** | 底层 CDR 字节编解码原语 (被 rosmsg2-serialization 内部依赖) |

### 4.2 为什么是这些库 (简版)

```
foxglove_bridge (C++, apt)
  → 用的是 Foxglove WebSocket 协议
  → Foxglove 官方 JS 客户端: @foxglove/ws-protocol
      → 不负责解码 CDR 二进制数据
      → 需要: @foxglove/rosmsg2-serialization
          → README 写明需要: @foxglove/rosmsg
          → 内部依赖: @foxglove/cdr
      → 构造函数需要 IWebSocket → Node.js 没有 → ws
```

完整搜索过程见 [`SEARCH_PROCESS.md`](SEARCH_PROCESS.md)。

### 4.3 协议说明

完整协议文档见 [`PROTOCOL.md`](PROTOCOL.md)。关键要点:

- **子协议:** `foxglove.sdk.v1` (不是 `foxglove.websocket.v1`)
- **JSON 控制帧:** serverInfo / advertise / serviceCallResponse / parameterValues 等
- **CDR 二进制帧:** 话题数据 / 服务 payload (MESSAGE_DATA opcode=1)
- **CDR 类型陷阱:** `int64` → BigInt, `float64[]` → Float64Array

---

## 五、测试结构

测试脚本 `test_all.mjs` 是一个入口，自动发现环境中所有数据，5 大模块依次执行:

```
test_all.mjs
│
├── 1. 话题 (Topic)     — 自动发现+分类+订阅+发布
│   ├── 按 15 类规则自动分类所有 channel
│   ├── 每类取 1 个代表订阅 (CDR 解码验证)
│   └── 发布→回显闭环 (需 clientPublish capability)
│
├── 2. 服务 (Service)   — 自动发现+命名空间归类+调用
│   ├── 过滤掉参数服务
│   ├── 按命名空间归类 (48 个命名空间)
│   └── 每空间取 1 个代表调用 (CDR 请求→响应验证)
│
├── 3. 动作 (Action)    — send_goal + feedback + status
│   ⚠️ bridge 不转发 _action/ 内部 channel (topology 仍可见)
│
├── 4. 参数 (Parameter) — List→Describe→Get→Set↔Get 闭环
│   ├── 自动发现所有 /list_parameters 服务 → 46 节点
│   ├── 4.1 Describe: 参数元数据
│   ├── 4.2 Get: 批量读取参数值
│   ├── 4.3 Set↔Get 闭环验证
│   └── 4.4 parameter_events 变更检测
│
├── 5. 拓扑 (Connection Graph)  ← 🆕
│   ├── subscribeConnectionGraph() 订阅
│   ├── 采集 25+ 次拓扑更新 (56 topic / 53 节点)
│   └── 数据输出到 full_analysis.json
│
├── 6. 系统概览 + CDR 带宽
│   ├── 全量 CDR vs JSON 带宽对比 (11,000+ 条消息)
│   └── 硬件信息
│
└── 导出 full_analysis.json → gen_topology_graph.mjs → topology.html
```

### 话题分类规则 (17 类)

```javascript
'传感器/相机'     → topic 含 /camera/
'传感器/点云'     → 类型含 PointCloud
'传感器/图像'     → 类型为 sensor_msgs/Image 或 topic 以 /image_raw 结尾
'机器人/关节状态' → topic === '/joint_states'
'机器人/TF变换'   → 类型含 TFMessage
'机器人/URDF模型' → topic 为 /robot_description 或 /robot_description_semantic
'控制/轨迹'       → 类型含 Trajectory 或 Controller
'控制/动态状态'   → 类型含 DynamicJoint
'规划/MoveIt'     → 类型以 moveit_msgs/ 开头 或 topic 含 planning_
'可视化/Marker'   → 类型含 Marker 或 topic 含 marker
'应用/机器人状态' → 类型含 RobotStatus 等
'应用/位姿抓取'   → 类型含 Pose 或 topic 含 goal/grasp
'系统/日志'       → topic 为 /rosout
'系统/事件参数'   → topic 为 /parameter_events
'系统/生命周期'   → 类型含 TransitionEvent
'ROS2 Demo'       → /chatter 或 /add_two_ints
'自定义/IVG'      → 类型以 ivg_interfaces/ 开头
```

自动发现: 新增 topic 自动匹配规则，不需要修改代码。

---

### 拓扑可视化 (新增)

测试完成后，从 `full_analysis.json` 生成交互式 ROS2 拓扑图 (类似 rqt_graph):

```bash
# 1. 先运行测试采集数据
npm test

# 2. 生成拓扑图
node gen_topology_graph.mjs

# 3. 在浏览器中查看
python3 -m http.server 9876
# 打开 http://localhost:9876/topology.html
```

**topology.html 功能:**
- 力导向布局，自动排布节点
- 椭圆 = Topic (标注 pub/sub 数量)
- 绿色方块 = Publisher (pub#ID)
- 黄色方块 = Subscriber (sub#ID)
- 滚轮缩放、拖拽平移、点击选中
- 会暴露 bridge 不转发的 `/_action/` 内部 topic

---
## 六、Demo 节点启动脚本

`start_demo_nodes.sh` 启动 6 个 ROS2 官方示例节点用于收发测试:

| # | 节点 | 命令 | 用途 |
|---|------|------|------|
| 1 | talker | `ros2 run demo_nodes_cpp talker` | 话题发布 (/chatter) |
| 2 | listener | `ros2 run demo_nodes_cpp listener` | 话题订阅 (/chatter) |
| 3 | add_two_ints_server | `ros2 run demo_nodes_cpp add_two_ints_server` | 服务 (/add_two_ints) |
| 4 | fibonacci_action_server | `ros2 run action_tutorials_py fibonacci_action_server` | 动作 (/fibonacci) |
| 5 | parameter_blackboard | `ros2 run demo_nodes_cpp parameter_blackboard` | 参数 (4个参数) |
| 6 | foxglove_bridge | `ros2 run foxglove_bridge foxglove_bridge` | WebSocket 桥接 |

```bash
# 启动
bash start_demo_nodes.sh

# 停止
bash start_demo_nodes.sh stop
```

---

## 七、测试输出示例

### 7.1 话题订阅

```
  传感器/相机        2 topics — sensor_msgs/msg/CameraInfo, sensor_msgs/msg/Image
  传感器/点云        1 topics — sensor_msgs/msg/PointCloud2
  机器人/关节状态    1 topics — sensor_msgs/msg/JointState

  ✅ /joint_states — CDR=316B JSON≈435B [sensor_msgs/msg/JointState]
  ✅ /tf — CDR=408B JSON≈1004B [tf2_msgs/msg/TFMessage]
  ℹ️ /camera/color/image_raw: 暂无数据
```

### 7.2 服务调用

```
  47 个服务, 12 个命名空间

  ✅ /add_two_ints → {"sum":15}   (7+8=15 → BigInt 转 Number 后正确)
  ✅ /controller_manager/list_controllers → {...}
```

### 7.3 参数闭环

```
  ✅ parameter_blackboard: 4 个参数读取成功
    use_sim_time = false [BOOL]
    publish_frequency = 30 [DOUBLE]

  ✅ parameter_blackboard/publish_frequency: 30 → 31 ✅ 闭环一致
```

### 7.4 CDR 带宽

```
  类型                                  CDR(B)  JSON(B)  节省%
  PointCloud2                          5509396 71178434  92.3%
  Image (color)                         921672 11321597  91.9%
  JointState                               316      435  27.4%

  📊 总节省: 91.5%
```

---

## 八、自定义配置

编辑 `test_all.mjs` 顶部:

```javascript
const WS_URL = 'ws://localhost:8765';        // foxglove_bridge 地址
const SUBPROTOCOL = ['foxglove.sdk.v1'];      // 子协议

// 订阅采集参数
await ctx.subCollect(topic, maxMsgs, timeoutMs)
//                      ↑ 采集条数  ↑ 超时毫秒

// 带宽采样等待时间 (第5部分，默认 8 秒)
await new Promise(r => setTimeout(r, 8000));
```

---

## 九、新增数据自动适配

**不需要修改任何代码**即可测试新增的 topic/service/parameter:

| 数据类型 | 自动发现机制 |
|---------|------------|
| 新 topic | `advertise` 事件 → 按 17 类规则自动归类 |
| 新 service | `advertiseServices` 事件 → 按命名空间自动归类 |
| 新 action | 搜索 `/_action/` 前缀的 topic/service |
| 新参数节点 | 搜索所有 `/list_parameters` 服务 |

唯一需要手动添加的情况: topic 类型不匹配现有 17 类规则时，在 `TOPIC_CATS` 对象中加一行即可:

```javascript
const TOPIC_CATS = {
  // ...
  '新分类名': (topic, type) => topic.includes('/your_prefix/'),
};
```

---

## 十、已知限制

| 限制 | 说明 |
|------|------|
| Action 不完整转发 | foxglove_bridge 不转发 `/<name>/_action/` 内部 topic/service |
| parameter_events 可能无数据 | QoS 不匹配导致 bridge 侧订阅不到 |
| rosapi 不属于 foxglove_bridge | `/rosapi/*` 服务属于 rosbridge_server |
| 子协议必须 | 必须 `foxglove.sdk.v1`，`foxglove.websocket.v1` 返回 HTTP 400 |

---

## 十一、常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `连接失败` | foxglove_bridge 未启动 | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765` |
| `HTTP 400` | 子协议错误 | 确保使用 `foxglove.sdk.v1` |
| `channel=0` | ROS2 环境未 source | `source /opt/ros/humble/setup.bash` |
| `subscribe 超时` | 该 topic 无发布者 | 正常现象，测试会显示 "暂无数据" |
| `AddTwoInts sum=7 ❌` | BigInt vs Number 比较 | CDR int64 反序列化为 BigInt, 需 `Number()` |
| `parameter Set 失败` | 参数只读 | 测试会自动跳过，显示 "write rejected" |
| `schema 解析失败` | 非标准 ros2msg | `ros2 interface show type/msg/Name` 检查 |

---

## 十二、依赖库版本锁定

```json
{
  "dependencies": {
    "@foxglove/rosmsg": "^5.0.5",
    "@foxglove/rosmsg2-serialization": "^3.1.2",
    "@foxglove/ws-protocol": "^0.8.0",
    "ws": "^8.21.0"
  }
}
```

`@foxglove/cdr` 为间接依赖，不出现在 `package.json` 中，由 `@foxglove/rosmsg2-serialization` 自动引入。

---

> 最后更新: 2026-06-03
