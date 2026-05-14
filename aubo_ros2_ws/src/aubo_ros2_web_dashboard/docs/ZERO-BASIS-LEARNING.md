# IVG Web Dashboard — 零基础快速学习核心文档

> **适用对象**: 完全零基础，不知道 ROS 2 / FastAPI / Vue 3 是什么的人
> **目标**: 读完本文档后，能理解本包每行代码的作用，并能自行修改

---

## 第 0 章：你需要先知道的 5 个概念

### 概念 1：后端 vs 前端

```
你打开浏览器访问一个网站，背后实际发生了两件事：

① 后端（服务器）                   ② 前端（浏览器）
   准备好数据                         把数据画成你看到的页面
   接收你的点击                       把你的点击转发给后端

类比：餐厅
  后端 = 厨房（做菜）          前端 = 服务员+餐桌（上菜、摆盘）
```

本项目中：
- **后端** = Python 代码（`aubo_ros2_web_dashboard/` 目录下的 `.py` 文件）
- **前端** = Vue 3 代码（`web/src/` 目录下的 `.vue` 和 `.ts` 文件）

### 概念 2：HTTP 请求和响应

浏览器和后端之间用 **HTTP 协议** 通信。就像寄信：

```
浏览器:  "请给我首页的内容"           →  HTTP 请求 (Request)
后端:    "好的，这是首页的 HTML"      →  HTTP 响应 (Response)
```

### 概念 3：WebSocket（持续连接）

HTTP 是你问一句、服务器答一句。**WebSocket** 是建立一条"电话线"，双方随时说话：

```
浏览器 ←————————————→ 后端
         持续连接
   ← 机械臂位置 (每 0.01 秒发一次)
   → 用户点击"抓取"
   ← 抓取结果
```

本项目中的 rosbridge 就是用 WebSocket 让浏览器能实时获取机械臂数据喵~

### 概念 4：ROS 2 话题和服务

ROS 2 是机器人操作系统。它有两种通信方式：

| | 话题 (Topic) | 服务 (Service) |
|--|-------------|---------------|
| 类比 | 广播电台 | 打电话 |
| 方向 | 单向，持续发送 | 双向，一问一答 |
| 例子 | 关节角度（一直发） | "执行抓取" → "成功/失败" |

### 概念 5：SPA（单页应用）

传统网站：点一个链接 → 整个页面刷新 → 从服务器重新加载

**SPA**（本项目的 Vue 3）：页面只加载一次，之后所有的页面切换、数据更新都在浏览器本地完成，不需要重新加载。优点是快、流畅，像桌面应用一样喵~

---

## 第 1 章：项目整体结构（鸟瞰图）

```
aubo_ros2_web_dashboard/          ← 这个包
│
├── 📂 aubo_ros2_web_dashboard/   ← 后端 (Python, 约1000行)
│   ├── config.py                 ← 读取 YAML 配置
│   ├── fastapi_static_gateway.py ← 程序入口
│   ├── gateway/
│   │   ├── app.py               ← FastAPI 应用工厂（组装路由）
│   │   ├── cli.py               ← 命令行参数解析
│   │   └── routes/
│   │       ├── health.py        ← /health 健康检查
│   │       ├── ivg_runtime.py   ← /api/v1/runtime 前端配置接口
│   │       ├── robot_mesh.py    ← 3D 模型文件服务
│   │       └── upstream_proxy.py ← WebSocket + 视频代理
│   │
│   ├── config/defaults.yaml     ← 所有默认配置
│   ├── launch/web_dashboard.launch.py ← ROS 2 启动文件
│   ├── setup.py / package.xml   ← 包元数据
│   │
│   └── web/src/                 ← 前端 (Vue 3 + TypeScript, 约2200行)
│       ├── main.ts              ← 应用入口
│       ├── App.vue              ← 根布局
│       ├── router/index.ts      ← 路由表 (URL→页面映射)
│       ├── constants/           ← 常量定义
│       ├── lib/                 ← 工具函数
│       ├── composables/         ← 可复用逻辑
│       ├── components/          ← UI 组件
│       └── views/               ← 6 个页面
```

---

## 第 2 章：后端代码逐文件讲解

### 2.1 `config.py` — 配置中心（142 行）

**作用**：所有配置的唯一来源。读取 `config/defaults.yaml`，提供类型安全的访问函数喵~

**关键函数**：

| 函数 | 作用 | 谁调用它 |
|------|------|---------|
| `gateway_port()` | 网关监听端口 (默认 8090) | cli.py 默认值 |
| `rosbridge_port()` | rosbridge 端口 (默认 9090) | BFF 接口、代理 |
| `runtime_config_dict()` | 构建前端 BFF 响应 | `/api/v1/runtime` 路由 |
| `save_settings_to_yaml()` | 把前端设置写入 YAML | `/api/v1/settings` 路由 |

**零基础理解**：这个文件就像一个"字典"——其他代码需要任何配置值时，都来这里查喵~

### 2.2 `gateway/app.py` — FastAPI 应用工厂（99 行）

**作用**：创建 FastAPI 应用实例，按优先级注册所有路由喵~

**路由注册顺序（越靠前优先级越高）**：

```python
app.include_router(ws_router)             # ① WebSocket 代理
app.include_router(http_proxy_router)     # ② HTTP 视频代理
app.include_router(health.router)         # ③ /health
app.include_router(ivg_runtime.router)    # ④ /api/v1/runtime + /api/v1/settings
app.include_router(robot_mesh.router)     # ⑤ /api/ivg/robot-mesh/*
app.mount("/js/robotwebtools", ...)       # ⑥ 静态 JS 库
app.mount("/", StaticFiles(..., html=True)) # ⑦ Vue 3 前端 (SPA fallback)
```

**零基础理解**：这个文件决定"浏览器发来一个 URL，由谁处理"喵~

### 2.3 `gateway/routes/upstream_proxy.py` — 代理（213 行）

**作用**：本包最核心的代码。把浏览器的请求"转发"到 ROS 2 系统喵~

**两个代理通道**：

```
① WebSocket 代理 (/ws/rosbridge)
   浏览器 ←→ FastAPI ←→ rosbridge (:9090) ←→ ROS 2
   （转发话题订阅、服务调用等所有实时通信）

② HTTP 视频代理 (/api/ivg/proxy/web-video/*)
   浏览器 ← FastAPI ← web_video_server (:8089) ← ROS 2 相机话题
   （转发 MJPEG 相机画面流）
```

**零基础理解**：像一个"翻译官"，浏览器说 HTTP，ROS 2 听不懂；FastAPI 在中间翻译转发喵~

### 2.4 `gateway/routes/ivg_runtime.py` — BFF 接口（38 行）

**作用**：BFF = Backend For Frontend，专门给前端提供配置数据喵~

**两个端点**：

| 方法 | 路径 | 作用 |
|------|------|------|
| GET | `/api/v1/runtime` | 返回端口号、话题名、服务名等前端需要的所有配置 |
| POST | `/api/v1/settings` | 前端修改设置后，保存到 YAML 配置文件 |

**零基础理解**：前端启动时需要知道"rosbridge 在哪个端口"，后端通过这个接口告诉它喵~

---

## 第 3 章：前端代码逐文件讲解

### 3.1 入口和路由

```
浏览器打开 http://IP:8090
  ↓
index.html（Vite 入口，唯一的 HTML 文件）
  ↓
main.ts（创建 Vue 应用，挂载 Pinia + Router）
  ↓
App.vue（根布局 = 导航栏 + 页面内容 + 底部状态栏）
  ↓
router/index.ts（根据 URL 决定显示哪个页面）
  ↓
views/*.vue（6 个页面之一）
```

**路由表**：

| URL | 显示哪个页面 | 作用 |
|-----|-----------|------|
| `/` | DashboardView.vue | 门户首页 |
| `/vision` | VisionGraspView.vue | 视觉抓取 |
| `/latte` | CoffeeLatteView.vue | 咖啡拉花 |
| `/monitor` | TfMonitorView.vue | 监控面板 |
| `/log` | LogView.vue | 系统日志 |
| `/settings` | SettingsView.vue | 设置 |

### 3.2 composables — 可复用逻辑层（本项目的核心）

这是本前端最重要的设计模式喵~ 每个 `useXxx.ts` 是一个 **composable**（组合函数），把某类功能封装在里面，任何页面都可以复用。

| Composable | 替代的旧代码 | 作用 |
|-----------|------------|------|
| `useRos.ts` | ivg_transport.js (298行) | ROS 连接管理：建立/断开 WebSocket、订阅话题、调用服务 |
| `useRuntime.ts` | ivg_runtime.js (172行) | 从后端获取运行时配置 |
| `useRosTopic.ts` | subscription_binder.js (172行) | 单个话题的订阅/取消（自动生命周期） |
| `useRosService.ts` | services.js (300行) | ROS 服务调用（抓取、快换、IO 控制等） |
| `useMJPEGStream.ts` | ivg_web_video.js (124行) | 相机 MJPEG 视频流 URL 构建 |

**零基础理解 composable 模式**：

```typescript
// 任何视图只需 3 行就能连接 ROS：
const { isConnected, subscribe, onRosJson } = useRos()
await connect()  // 建立连接
onRosJson('/joint_states', (msg) => { ... })  // 收到数据后做什么

// 底层怎么连 WebSocket、怎么重连、怎么清理——全部封装在 useRos 内部
// 视图只需要关心"收到数据后干什么"
```

### 3.3 数据流全景

```
[机械臂硬件]
    ↓ AUBO SDK (C++)
    ↓ ROS 2 话题 (/joint_states, /robot_status, ...)
    ↓
[rosbridge (:9090)]        ← ROS 2 ↔ WebSocket 桥
    ↓ WebSocket
    ↓
[FastAPI 网关 (:8090)]      ← /ws/rosbridge 代理转发
    ↓ WebSocket
    ↓
[浏览器 useRos composable]  ← ROSLIB.Ros 连接管理
    ↓ onRosJson / subscribe
    ↓
[Vue 组件 ref()]            ← 响应式数据
    ↓ 模板自动更新
    ↓
[用户看到的页面]             ← el-tag / 表格 / 相机画面
```

---

## 第 4 章：如何修改代码

### 场景 1：修改话题名

```typescript
// 文件: web/src/constants/ros.ts
// 改这一行:
export const JOINT_STATES_TOPIC = '/joint_states'
// 所有订阅这个常量的地方自动更新 — 不需要到处查找替换
```

### 场景 2：增加一个新页面

```typescript
// 步骤 1: 创建 web/src/views/NewPage.vue
// 步骤 2: 在 router/index.ts 加一行:
{ path: '/new', name: 'new', component: () => import('@/views/NewPage.vue') }
// 步骤 3: 在 components/ivg/SiteNav.vue 加导航链接
```

### 场景 3：增加新的 ROS 服务调用

```typescript
// 在 useRosService.ts 里加一个方法:
async function newServiceCall(param: string) {
  return call('/new_service', 'ivg_interfaces/srv/NewType', { param })
}
```

### 场景 4：修改后端网关端口

```yaml
# 文件: config/defaults.yaml
gateway:
  port: 9000  # 改这里 → 所有引用的地方自动生效
```

---

## 第 5 章：术语速查表

| 术语 | 白话解释 |
|------|---------|
| **FastAPI** | 一个 Python 库，让你快速写 HTTP 服务器 |
| **Vue 3** | 一个 JavaScript 框架，帮你管理页面数据 |
| **Vite** | 一个构建工具，把你的 Vue 代码打包成浏览器能吃的格式 |
| **TypeScript** | JavaScript + 类型标注（写的时候能发现拼写错误） |
| **composable** | Vue 3 的"可复用函数"，把一类功能封装在里面 |
| **BFF** | "Backend For Frontend" — 专门给前端提供数据的后端接口 |
| **rosbridge** | ROS 2 的 WebSocket 桥 — 让浏览器能和 ROS 2 通信 |
| **SPA** | "Single Page Application" — 页面只加载一次，之后不刷新 |
| **路由** | URL → 页面的映射关系 |
| **代理** | 中间转发 — 浏览器不能直连 ROS 2，FastAPI 在中间转发 |
| **MJPEG** | Motion JPEG — 连续发送 JPEG 图片来模拟视频流 |
| **QoS** | Quality of Service — ROS 2 中配置消息可靠性的参数 |

---

*最后更新: 2026-05-16*
