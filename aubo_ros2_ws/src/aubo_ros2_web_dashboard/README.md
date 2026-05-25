# IVG Web Dashboard

ROS 2 机械臂视觉抓取 Web 控制面板。FastAPI 网关 + 纯 HTML/JS MPA 前端，浏览器通过 WebSocket/HTTP 代理与 ROS 系统实时交互。

> **前端技术栈**：纯 HTML/JS MPA（零构建），ES modules + importmap 加载 ros3djs/roslib/three.js，FastAPI BFF 网关喵~
>
> **当前架构（2026-05-25）**：前端 5 个独立 HTML 页面（index/vision_grasp/coffee_latte/log/settings/tf_monitor），通过 `js/core/` 共享基础设施层（ros.js/ros_connector.js/log-bus.js/log-ros-bridge.js/lifecycle.js/utils.js/settings.js），子模块代码按功能域组织在 `js/vision_grasp/`、`js/latte/`、`js/view3d/`、`js/components/` 下喵~
>
> 话题/服务名通过 `localStorage` 覆盖 `config/defaults.yaml` 默认值，前端运行时从 BFF `GET /api/v1/runtime` 获取配置喵~

---

## 1. 架构总览

```
ROS 2 Launch
  ├── rosbridge (Tornado :9090)         ← ROS 消息总线 WebSocket 桥
  ├── tf2_web_republisher               ← TF 坐标变换 Web 发布
  ├── web_video_server (:8089)          ← 摄像头 MJPEG/快照 HTTP 服务
  ├── GripperSwapWorker                 ← 工具快换服务端 (/run_gripper_swap, /get_current_tool)
  │     └── /tool_changer_status        ← 定期发布当前工具状态 (每 5s)
  └── FastAPI 网关 (:8090)              ← 统一入口（代理 + 静态文件）
        │
        ├── /ws/rosbridge               → rosbridge 双向 WebSocket 代理
        ├── /api/ivg/proxy/web-video/*  → web_video_server HTTP 流代理
        ├── /api/v1/runtime             → 前端 BFF 配置接口
        ├── /api/ivg/robot-mesh/*       → 机器人 3D 模型文件服务
        ├── /health                     → 健康检查
        └── /*                          → 前端静态页面 (MPA)

浏览器 (纯 HTML/JS MPA)
  ├── [1] runtime_provider.js      → GET /api/v1/runtime → globalThis.__IVG_RUNTIME
  ├── [2] core/settings.js         → localStorage 覆盖话题/服务名
  ├── [3] core/ros.js              → ROSLIB.Ros 单例封装 (订阅/服务/重连)
  ├── [4] ivg_web_video.js         → 摄像头 MJPEG 流/快照 URL
  ├── [5] view3d/session.js        → URDF 参数 + /tf + /joint_states + 工具几何
  ├── [6] 5 个 HTML 页面           → 视觉抓取/咖啡拉花/日志/设置/TF监控
  ├── [7] core/log-bus.js          → 统一日志总线 (IndexedDB + BroadcastChannel)
  └── [8] core/log-ros-bridge.js   → ROS→日志桥接 (/rosout + service 钩子)
```

---

## 2. 配置系统

```
config/defaults.yaml (所有默认值，唯一配置来源)
  → ROS 2 launch arguments 覆盖
    → CLI 参数传递给网关进程
      → config.init() 合并覆盖 → config.xxx() 类型访问器

settings_panel.html (浏览器设置页)
  → POST /api/v1/settings → 写入 config/defaults.yaml
    → 更新内存 _cfg（即时生效）
      → localStorage 兜底（网关不可达时）
```

### 配置规则（必须遵守）

1. **所有 ROS2 相关参数必须在设置页可配置。** 包括但不限于：话题名、服务名、消息类型、连接参数。不允许在前端 JS 或 HTML 中硬编码可修改的配置值。

2. **设置页保存即覆盖 YAML。** `POST /api/v1/settings` 将前端提交的值写入 `config/defaults.yaml` 的 `vision_panel` 段，同时更新内存配置使修改即时生效（无需重启网关）。网关不可达时回退到 `localStorage`。

3. **新增话题/服务/参数时**：先在 `config/defaults.yaml` 的 `vision_panel.{common,vision,latte}` 段添加定义（含 `id`, `default`, `label`, `msg_type`/`srv_type`），设置页会自动渲染输入框。前端面板通过 `/api/v1/runtime` 获取默认值。

4. **前端面板的「话题与服务设置」按钮**：统一链接到 `settings_panel.html`（新标签页打开），不再使用各面板独立的设置弹窗。

5. **运行页读取顺序**：`useDashboardSettings.ts` 先加载 YAML/runtime 默认值，再合并 `localStorage` 覆盖。旧版 `getSetting()` 的语义由 `settings.rosName(id, fallback)`、`settings.raw(id, fallback)`、`settings.serviceType(id, fallback)` 替代喵~

6. **URDF 参数格式**：`urdf-param` 使用旧版兼容的 `/<node_full_name>:<parameter_name>` 语法，默认 `/robot_state_publisher:robot_description`。裸参数名会回退到 `/robot_state_publisher/<get_parameters>` 服务读取喵~

7. **URDF/TF 显示语义**：旧版 `ROS3D.UrdfClient` 的模型位置关系由 `ROS2TFClient` 按 `fixedFrame -> link` TF 直接驱动，URDF 只提供 link 与 mesh。Vue 3 版 `Robot3dViewer` 必须保持同一语义：不要把本地 URDF 关节链与 TF 位姿叠加，否则会出现工具、相机、末端 link 位置关系错误喵~

8. **末端工具显示语义**：后端 `scene_attach_worker` 只更新 MoveIt 碰撞（`/attached_collision_object` + 必要时 `/planning_scene` world REMOVE），不动态修改 `robot_description`。Web 端 `Robot3dViewer` 必须订阅 `/tool_changer_status`，在 `is_connected=false` 时移除工具显示模型，在 `is_connected=true` 时按 `tool_id` 加载对应 STL，并用 `kuaihuan_Link TF + tools.yaml.attach_offset` 定位喵~

9. **3D 坐标系语义**：ROS/RViz 约定 Z 轴向上，Three.js 默认 Y 轴向上。`SceneManager` 必须在 viewer 层设置相机 `up=(0,0,1)` 并将网格放到 ROS XY 地面平面，不能旋转 TF/URDF 数据本身；页面图例固定为 X=红、Y=绿、Z=蓝喵~

10. **单屏布局语义**：旧版运行页以 `body.ivg-single-screen` 为核心，目标是全宽紧凑排布并尽量避免双滚动条。Vue 3 版运行页使用 `.ivg-run-page` 限定锁高链路：桌面/平板 `>=768px` 锁在 `100svh` 内，手机 `<768px` 放行整页纵向滚动；主区与相机区按旧版 `920px` 断点语义，在 `921px` 起才进入双列喵~

11. **iPad/触摸适配语义**：入口 viewport 必须包含 `viewport-fit=cover, interactive-widget=resizes-content`，底部固定状态栏必须预留 `env(safe-area-inset-bottom)`，导航链接和全屏按钮触摸高度不少于 44px，避免 iPad 10 与触摸屏上出现遮挡或难点击喵~

12. **最终验收入口**：修改 Web Dashboard 后，最终测试必须通过工作空间根目录的 `start_aubo_new_driver.sh` 启动整套系统。该脚本会先执行 `npm run build` 生成 Vue 3 `web/dist/`，再执行 `colcon build` 让 `setup.py` 安装最新静态产物，最后访问脚本打印的 `IVG 门户`、`视觉抓取` 和 `咖啡拉花` 地址验证喵~

13. **前端构建跳过开关**：仅在确认 `web/dist/` 已是最新产物时，才可设置 `SKIP_WEB_BUILD=1` 跳过 Vue 3 构建。`SKIP_BUILD=1` 会跳过前端与 colcon 全部构建喵~

---

## 3. 文件间调用关系

### 3.1 后端 Python

```
fastapi_static_gateway.py          ← 入口: python -m aubo_ros2_web_dashboard.fastapi_static_gateway
  └── cli.py:main()                ← 解析 CLI 参数
        ├── config.py:init()       ← 合并 CLI 覆盖到配置
        └── app.py:create_app()    ← 创建 FastAPI 应用
              ├── config.py        ← 读取各种配置值
              ├── settings.py      ← 兼容重导出 (从 config.py)
              ├── health.py        ← GET /health
              ├── ivg_runtime.py   ← GET /api/v1/runtime
              │     └── config.py:runtime_config_dict()
              ├── robot_mesh.py    ← GET /api/ivg/robot-mesh/*
              └── upstream_proxy.py ← /ws/rosbridge + /api/ivg/proxy/web-video/*
                    └── config.py  ← 读取端口/超时等配置

web_dashboard.launch.py            ← ROS 2 launch 编排
  └── config.py                    ← 读取 YAML 默认值作为 launch arg 默认值
```

### 3.2 前端 JavaScript

```
vision_grasp_panel.html 加载以下 ES 模块:

┌─ 核心基础设施 ──────────────────────────────────────────────────────────┐
│                                                                          │
│  dom_cache.js                                                           │
│    └── createDomCache(doc) → getById(id)    [DOM 懒缓存, 不缓存 null]   │
│                                    ↑                                    │
│  runtime_provider.js                    │                               │
│    └── loadIvgRuntime() → __IVG_RUNTIME  │  [GET /api/v1/runtime]       │
│                                    ↑      │                              │
│  ivg_runtime.js                    │      │                              │
│    ├── loadRuntime() ──────────────┘      │  [调用 runtime_provider]    │
│    ├── parsePort(v)                       │  端口解析                   │
│    ├── fromQuery(name)                    │  URL参数读取                │
│    ├── fromRuntime(key)                   │  运行时配置读取             │
│    ├── rosbridgeWebSocketUrl()            │  构建 WS URL                │
│    ├── webVideoProxyOriginPrefix()        │  视频代理前缀               │
│    ├── createRosReconnectState()          │  创建重连状态               │
│    ├── scheduleRosReconnect(state,fn,opt) │  指数退避重连               │
│    └── wireOnlineRosReconnect(state,fn)   │  网络恢复重连               │
│                                    ↑      │                              │
│  ivg_transport.js                  │      │                              │
│    ├── IvgTransport() ────────────────────  [ROSLIB 封装]               │
│    │   ├── loadRuntime()           异步    读取 BFF 配置                │
│    │   ├── connectControl()        异步    建立 ROSLIB.Ros 连接         │
│    │   ├── close()                         关闭连接                     │
│    │   ├── isConnected() → bool           连接状态                     │
│    │   ├── subscribe({topic,msgType,maxHz}) 话题订阅                    │
│    │   ├── unsubscribe(topic)             取消订阅                      │
│    │   ├── unsubscribeAll()               全部取消                      │
│    │   ├── callService({service,type,request}) → Promise  服务调用      │
│    │   ├── onRosJson(topicFilter, fn)     注册话题消息处理器            │
│    │   ├── onControlJson(fn)              注册控制面消息处理器          │
│    │   ├── cameraStreamUrl(topic,sid,q)   构建 MJPEG 流 URL             │
│    │   └── cameraSnapshotUrl(topic,sid,q) 构建 JPEG 快照 URL            │
│    └── ivgTransport = new IvgTransport()  全局单例                     │
│                                                                          │
│  ivg_web_video.js                                                       │
│    ├── streamUrl(topic, opts)             摄像头流 URL                  │
│    ├── snapshotUrl(topic, opts)           快照 URL                      │
│    └── mjpegStreamAttachAutoReload(img,urlFn)  <img> 自动重连           │
│                                                                          │
│  ivg_site_nav.js                                                        │
│    └── 全局导航栏 + 全屏按钮（自动检测 DOM 挂载）                       │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

┌─ 视觉抓取子模块 ────────────────────────────────────────────────────────┐
│                                                                          │
│  vision_grasp/config.js                                                 │
│    ├── _bffTopics() → [...]            从 BFF 读取话题默认值            │
│    ├── _bffServices() → [...]          从 BFF 读取服务默认值            │
│    └── 导出: VISION_SETTINGS_DEFAULTS, VISION_TOPIC_IDS,                │
│              VISION_TOPIC_TYPE_MAP, VISION_SERVICE_TYPE_MAP, ...        │
│                                    ↑                                     │
│  vision_grasp/pose_card.js           │                                   │
│    ├── escapeHtml(s) → str                    HTML 转义                 │
│    ├── robotPoseHtmlIsRenderable(html) → bool 位姿 HTML 是否有效        │
│    ├── formatRobotPoseHtml(msg) → html        末端位姿 → 位姿卡 HTML   │
│    └── formatFinalGraspPoseHtml(msg) → html   抓取位姿 → 位姿卡 HTML   │
│                                    ↑                                     │
│  vision_grasp/projection_overlay.js │                                   │
│    └── createProjectionOverlayController(opts) → {                      │
│          resetState()              重置投影状态                         │
│          clearCanvas()             清空画布                              │
│          scheduleDraw()            requestAnimationFrame 绘制           │
│          setMode(mode)             切换工件/AI 模式                     │
│          setGraspMsg(msg)          接收 GraspPoseArray                  │
│          setCameraInfo(msg)        接收 CameraInfo                       │
│          ingestTfMessage(msg)      接收 TF 变换                          │
│        }                        依赖 tf_clients.js (四元数/变换数学)    │
│                                    ↑                                     │
│  vision_grasp/joint_chart.js       │                                   │
│    └── createJointChartController(opts) → {                             │
│          reset()                   重置曲线                              │
│          pushSample(names, pos)    追加关节采样                          │
│          observeResize()           监听容器尺寸                          │
│        }                        依赖 Chart.js                           │
│                                    ↑                                     │
│  vision_grasp/services.js         │                                    │
│    └── createVisionServiceActions(opts) → {                             │
│          bindControlButtons()     绑定按钮点击 → callService()          │
│          callGripperSwap(tId)     夹爪快换服务调用                       │
│          callGetCurrentTool(done) 查询后端当前工具                       │
│          setCurrentToolId(id)     设置当前工具 ID (内部状态)             │
│          getCurrentToolId()→str   获取当前工具 ID                        │
│          isSwapInFlight()→bool    快换是否进行中                          │
│          onGripperSwapDone(fn)    注册快换完成回调                       │
│        }                        调用 ivgTransport.callService()         │
│                                    ↑                                     │
│  vision_grasp/ui_settings.js      │                                    │
│    └── createVisionSettingsController(opts) → {                         │
│          applyDefaultsToDom()     将默认值写入输入框                     │
│          readFromDom()            从输入框读取当前值                     │
│          loadFromStorage()        从 localStorage 恢复                   │
│          saveToStorage()          持久化到 localStorage                  │
│          clearStorage()           清除 localStorage                     │
│          openModal() / closeModal() 话题设置弹窗                         │
│          modalOpen() → bool       弹窗是否打开                           │
│        }                                                                 │
│                                    ↑                                     │
│  vision_grasp/ui_binder.js        │                                    │
│    └── createVisionUiBinder(opts) → {                                   │
│          bindModeSwitches()       绑定工件/AI 模式切换 radio            │
│          bindTopicSettingsUi()    绑定设置按钮/弹窗/重置/保存           │
│          bindResultImageLoad()    绑定结果图加载事件                     │
│        }                                                                 │
│                                    ↑                                     │
│  vision_grasp/mode_controller.js  │                                    │
│    └── createVisionModeController(opts) → {                             │
│          syncModeUi()             同步模式 UI (显示/隐藏控制组)         │
│        }                                                                 │
│                                    ↑                                     │
│  vision_grasp/urdf_panel.js       │                                    │
│    └── createVisionUrdfPanel(opts) → {                                  │
│          start($)                 启动 3D 查看器                         │
│          stop()                   停止 3D 查看器                         │
│          layout()                 重新布局                               │
│        }                        依赖 view3d/session.js                  │
│                                    ↑                                     │
│  vision_grasp/subscription_binder.js                                   │
│    └── bindVisionSubscriptions(opts) → void                             │
│          根据当前模式按需订阅: 末端位姿/关节/相机/TF/VPE/GraspPose      │
│          每个订阅通过 transport.onRosJson + transport.subscribe 绑定    │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

┌─ 3D 查看器 ─────────────────────────────────────────────────────────────┐
│                                                                          │
│  view3d/tf_clients.js                                                   │
│    ├── normalizeFrameId(frame) → str                                    │
│    ├── ivgIdentityTransform() → {translation, rotation}                 │
│    ├── ivgCloneTransform(tf) → {translation, rotation}                  │
│    ├── ivgQuatNormalize(q) → {x,y,z,w}                                  │
│    ├── ivgQuatMultiply(a, b) → {x,y,z,w}                                │
│    ├── ivgQuatConjugate(q) → {x,y,z,w}                                  │
│    ├── ivgRotateVectorByQuat(v, q) → {x,y,z}                           │
│    ├── ivgComposeTransforms(a, b) → tf                                  │
│    ├── ivgInvertTransform(tf) → tf                                      │
│    ├── ivgBuildTfPath(frame, edges) → [{frame, transform}, ...]         │
│    ├── ivgFindRelativeTransform(src, dst, edges) → tf|null              │
│    └── IvgRos3dTfClient(ros, fixedFrame, opts)                          │
│          ├── getTransform(frame) → tf|null                              │
│          ├── hasTransformFor(frame) → bool                              │
│          ├── subscribe(frame, cb)                                       │
│          ├── unsubscribe(frame, cb)                                     │
│          ├── setFixedFrame(frame) → bool                                │
│          └── dispose()                                                  │
│                                    ↑                                     │
│  view3d/patches.js                 │                                    │
│    ├── ivgRos3dEmbeddedObject3DClass(viewer) → class      [THREE polyfill]
│    └── installIvgRos3dEmbeddedThreeSafeAddPatch(viewer)   [ros3d 补丁]  │
│                                    ↑                                     │
│  view3d/hints.js                   │                                    │
│    ├── showView3dUrdfHint(host, html)           显示加载提示             │
│    └── removeView3dUrdfHint(host)               移除加载提示             │
│                                    ↑                                     │
│  view3d/session.js                 │                                    │
│    └── IvgRos3dView3dSession(ros, $, opts)                             │
│          ├── start()        创建 ROS3D.Viewer + TF + 雷达/标记/URDF     │
│          ├── stop()         销毁所有 3D 对象和订阅                      │
│          ├── layout()       重新计算尺寸                                 │
│          └── _startUrdfStage($, host, fixedFrame)  延迟加载 URDF        │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

┌─ 面板主编排器 ──────────────────────────────────────────────────────────┐
│                                                                          │
│  vision_grasp_panel.js    [6] — 所有子模块的组装点                      │
│                                                                          │
│  初始化顺序 (DOMContentLoaded):                                          │
│    1. loadRuntime()              → runtime_provider.js                  │
│    2. loadTopicsFromStorage()    → ui_settings.js (localStorage)        │
│    3. jointChart.observeResize() → joint_chart.js                       │
│    4. bindMonitoringSectionCollapse()  内部函数 (监控底栏折叠)          │
│    5. bindMonitoringBundleMinHeightSync() 内部函数 (位姿列高度自适应)    │
│    6. uiBinder.bindResultImageLoad()    → ui_binder.js                  │
│    7. document 'visibilitychange' → suspendRealtime/resumeRealtime      │
│    8. uiBinder.bindModeSwitches()      → ui_binder.js                   │
│    9. syncModeUi()                     → mode_controller.js             │
│   10. serviceActions.bindControlButtons() → services.js (含快换按钮)    │
│   11. setupVisionLogDisplay()          → #svc-log 结构化日志框          │
│   12. serviceActions.onGripperSwapDone() → 注册快换完成回调              │
│   13. _initToolStatusBridge()          → 三层工具状态初始化              │
│   14. uiBinder.bindTopicSettingsUi()    → ui_binder.js                  │
│   15. wireOnlineRosReconnect()          → ivg_runtime.js                │
│   16. connect()   ─────────────────────────────────────┐                │
│                                                        ↓                │
│  connect() 内部流程:                                                  │
│    ivgTransport.loadRuntime()        异步 BFF                          │
│    ivgTransport.connectControl()     异步 WebSocket                    │
│    ivgTransport.onControlJson(o => { 监听 error/close 事件            │
│      o.op === 'close' → unsubscribeAll() + scheduleRosReconnect()    │
│    })                                                                 │
│    startSubscriptions() ─────────────────────────────────┐           │
│        ↓                                                  │           │
│    unsubscribeAll()    清理旧订阅                          │           │
│    bindVisionSubscriptions() → subscription_binder.js     │           │
│        │                                                  │           │
│        ├── transport.subscribe(robotTopic)  → 末端位姿    │           │
│        ├── transport.subscribe(jointTopic)  → 关节状态    │           │
│        ├── transport.subscribe(statusTopic) → VPE 状态    │           │
│        ├── transport.subscribe(graspTopic)  → AI 抓取位姿 │           │
│        ├── transport.subscribe(cameraInfoTopic) → 相机内参│           │
│        ├── transport.subscribe(tfTopic)     → TF 变换     │           │
│        └── transport.subscribe(tfStaticTopic) → TF 静态   │           │
│    re-subscribe /tool_changer_status → _onToolStatusMsg (重连时)      │
│                                                                        │
│  tf_monitor_panel.js    — TF 监控面板 (独立页面)                      │
│    使用 ivgPorts + ivgTransport + view3d/session.js                   │
│    订阅 /tf + /tf_static，在 3D 查看器中可视化                        │
│                                                                        │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 完整函数清单

### 4.1 后端 Python — 配置层

| 文件 | 函数 | 签名 | 说明 |
|------|------|------|------|
| `config.py` | `init(cli_args)` | `dict\|None → void` | 合并 CLI 覆盖到全局配置 |
| `config.py` | `gateway_bind()` | `→ str` | 网关监听地址, 默认 `0.0.0.0` |
| `config.py` | `gateway_port()` | `→ int` | 网关监听端口, 默认 `8090` |
| `config.py` | `gateway_gzip_min_size()` | `→ int` | GZip 压缩阈值, 默认 `512` |
| `config.py` | `rosbridge_host()` | `→ str` | rosbridge 上游主机 |
| `config.py` | `rosbridge_port()` | `→ int` | rosbridge 上游端口, 默认 `9090` |
| `config.py` | `rosbridge_ws_path()` | `→ str` | 代理 WS 路径, 默认 `/ws/rosbridge` |
| `config.py` | `rosbridge_max_message_bytes()` | `→ int` | 最大消息字节, 默认 `67108864` |
| `config.py` | `rosbridge_ping_interval()` | `→ int` | 心跳间隔(秒), 默认 `20` |
| `config.py` | `rosbridge_ping_timeout()` | `→ int` | 心跳超时(秒), 默认 `60` |
| `config.py` | `rosbridge_close_timeout()` | `→ int` | 关闭超时(秒), 默认 `10` |
| `config.py` | `web_video_host()` | `→ str` | web_video 上游主机 |
| `config.py` | `web_video_port()` | `→ int` | web_video 上游端口, 默认 `8089` |
| `config.py` | `web_video_listen_address()` | `→ str` | web_video 监听地址 |
| `config.py` | `web_video_server_threads()` | `→ int` | 服务线程数, 默认 `4` |
| `config.py` | `web_video_ros_threads()` | `→ int` | ROS 线程数, 默认 `2` |
| `config.py` | `web_video_proxy_prefix()` | `→ str` | 视频代理 URL 前缀 |
| `config.py` | `proxy_video_connect_timeout()` | `→ float` | httpx 连接超时 |
| `config.py` | `proxy_video_pool_timeout()` | `→ float` | httpx 连接池超时 |
| `config.py` | `proxy_video_read_timeout()` | `→ float\|None` | httpx 读取超时 |
| `config.py` | `proxy_video_chunk_bytes()` | `→ int` | 流式块大小, 默认 `65536` |
| `config.py` | `package_version_fallback()` | `→ str` | 版本回退值 |
| `config.py` | `vision_panel_config()` | `→ dict` | 视觉面板话题/服务定义 |
| `config.py` | `robotwebtools_search_dirs()` | `→ list[str]` | RWT 资产候选目录 |
| `config.py` | `runtime_config_dict(root)` | `→ dict` | BFF 响应负载 |

### 4.2 后端 Python — 网关层

| 文件 | 函数/类 | 签名 | 说明 |
|------|---------|------|------|
| `cli.py` | `main(argv)` | `list[str]\|None → void` | 解析 CLI → 初始化配置 → 启动 uvicorn |
| `app.py` | `SecurityHeadersMiddleware` | `class(ASGIApp)` | 注入 `X-Content-Type-Options: nosniff` |
| `app.py` | `create_app(web_root, *, rwt_override)` | `str, str\|None → FastAPI` | 组装路由/中间件/静态挂载 |
| `health.py` | `health(request)` | `→ {"status","static_root"}` | 健康检查 |
| `ivg_runtime.py` | `runtime_v1(request)` | `→ runtime_config_dict()` | BFF 配置接口 |
| `upstream_proxy.py` | `rosbridge_websocket_proxy(ws)` | `WebSocket → void` | 浏览器↔rosbridge 双向 WS 代理 |
| `upstream_proxy.py` | `web_video_http_proxy(path, req)` | `→ StreamingResponse` | 视频流 HTTP 代理 |

### 4.3 后端 Python — 机器人网格

| 文件 | 函数 | 签名 | 说明 |
|------|------|------|------|
| `robot_mesh.py` | `_media_type(path)` | `Path → str` | STL/DAE/OBJ → MIME 类型 |
| `robot_mesh.py` | `_is_under(candidate, parent)` | `Path,Path → bool` | 路径穿越检查 |
| `robot_mesh.py` | `_resolve_mesh(share, rel_path)` | `Path,str → Path\|None` | 大小写不敏感文件查找 |
| `robot_mesh.py` | `serve_robot_mesh(remainder)` | `str → FileResponse` | 3D 模型文件服务 |

### 4.4 前端 JS — 核心基础设施

| 文件 | 导出/函数 | 签名 | 说明 |
|------|----------|------|------|
| `core/dom_cache.js` | `createDomCache(root)` | `Document → (id→Element)` | DOM 懒缓存工厂 |
| `core/runtime_provider.js` | `loadIvgRuntime()` | `async → dict` | GET /api/v1/runtime → `__IVG_RUNTIME` |
| `ivg_runtime.js` | `loadRuntime()` | `async → dict` | 加载运行时配置 |
| `ivg_runtime.js` | `parsePort(v)` | `any → int\|null` | 端口解析 |
| `ivg_runtime.js` | `rosbridgeWebSocketUrlFromRuntime(rt)` | `dict → str` | 构建 WS URL |
| `ivg_runtime.js` | `webVideoProxyOriginPrefix()` | `→ str` | 视频代理 URL 前缀 |
| `ivg_runtime.js` | `createRosReconnectState()` | `→ {gen,attempts,timer}` | 创建重连状态对象 |
| `ivg_runtime.js` | `clearRosReconnectTimer(state)` | `state → void` | 清除重连定时器 |
| `ivg_runtime.js` | `bumpRosReconnectGen(state)` | `state → int` | 递增重连代数 |
| `ivg_runtime.js` | `scheduleRosReconnect(state,fn,opts)` | `state,fn,opts → void` | 指数退避重连调度 |
| `ivg_runtime.js` | `wireOnlineRosReconnect(state,fn)` | `state,fn → void` | 网络恢复事件监听 |

### 4.5 前端 JS — 传输层

| 文件 | 方法 | 签名 | 说明 |
|------|------|------|------|
| `ivg_transport.js` | `IvgTransport()` | 构造函数 | 创建传输实例 |
| | `loadRuntime()` | `async → dict` | 加载 BFF 运行时配置 |
| | `connectControl()` | `async → void` | 建立 ROSLIB.Ros WS 连接 |
| | `close()` | `→ void` | 关闭连接 + 清理 |
| | `isConnected()` | `→ bool` | 连接状态 |
| | `subscribe(spec)` | `→ bool` | 订阅话题: `{topic, msgType, maxHz}` |
| | `unsubscribe(topic)` | `→ void` | 取消订阅 |
| | `unsubscribeAll()` | `→ void` | 全部取消订阅 |
| | `callService(spec)` | `→ Promise` | 调用 ROS 服务: `{service, type, request, timeoutMs}` |
| | `onRosJson(topicFilter, fn)` | `→ void` | 注册 ROS 消息处理器 |
| | `clearRosHandlers()` | `→ void` | 清除所有 ROS 处理器 |
| | `onControlJson(fn)` | `→ void` | 注册控制面处理器 |
| | `clearControlJsonHandlers()` | `→ void` | 清除所有控制面处理器 |
| | `cameraStreamUrl(topic,sid,q)` | `→ str` | 构建 MJPEG 流 URL |
| | `cameraSnapshotUrl(topic,sid,q)` | `→ str` | 构建 JPEG 快照 URL |

### 4.6 前端 JS — 视频流

| 文件 | 导出/函数 | 签名 | 说明 |
|------|----------|------|------|
| `ivg_web_video.js` | `streamUrl(topic, opts)` | `→ str` | MJPEG 流 URL (优先网关代理) |
| `ivg_web_video.js` | `snapshotUrl(topic, opts)` | `→ str` | JPEG 快照 URL |
| `ivg_web_video.js` | `viewerUrl(topic, opts)` | `→ str` | 流查看器 URL |
| `ivg_web_video.js` | `mjpegStreamAttachAutoReload(img, getUrl)` | `→ void` | `<img>` 断线自动重连 |

### 4.7 前端 JS — 视觉抓取配置

| 文件 | 导出 | 类型 | 说明 |
|------|------|------|------|
| `vision_grasp/config.js` | `VISION_SETTINGS_DEFAULTS` | `dict` | `{id: default}` 所有话题/服务默认值 |
| | `VISION_ALL_SETTING_IDS` | `list[str]` | 所有设置项 ID |
| | `VISION_TOPIC_IDS` | `list[str]` | 普通话题 ID (不含 TF) |
| | `VISION_TF_TOPIC_IDS` | `list[str]` | TF 话题 ID |
| | `VISION_SERVICE_IDS` | `list[str]` | 服务 ID |
| | `VISION_TOPIC_TYPE_MAP` | `dict` | `{id: msg_type}` |
| | `VISION_SERVICE_TYPE_MAP` | `dict` | `{id: srv_type}` |
| | `VISION_FIXED_SERVICE_TYPES` | `dict` | 固定服务类型映射 |

### 4.8 前端 JS — 视觉抓取子模块

| 文件 | 导出/函数 | 签名 | 说明 |
|------|----------|------|------|
| `pose_card.js` | `escapeHtml(s)` | `str → str` | HTML 实体转义 |
| | `robotPoseHtmlIsRenderable(html)` | `str → bool` | 位姿 HTML 是否可渲染 |
| | `formatRobotPoseHtml(msg)` | `msg → html` | RobotStatus → 位姿卡 HTML |
| | `formatFinalGraspPoseHtml(msg)` | `msg → html` | PoseArray → 抓取位姿卡 HTML |
| `projection_overlay.js` | `createProjectionOverlayController(opts)` | `opts → controller` | 投影叠加控制器工厂 |
| | `controller.resetState()` | `→ void` | 重置投影状态 |
| | `controller.clearCanvas()` | `→ void` | 清空画布 |
| | `controller.scheduleDraw()` | `→ void` | 调度重绘 (rAF) |
| | `controller.setMode(mode)` | `→ void` | 切换工件/AI 模式 |
| | `controller.setGraspMsg(msg)` | `→ void` | 接收 PoseArray |
| | `controller.setCameraInfo(msg)` | `→ void` | 接收 CameraInfo |
| | `controller.ingestTfMessage(msg)` | `→ void` | 接收 TF 消息 |
| `joint_chart.js` | `createJointChartController(opts)` | `opts → controller` | 关节曲线控制器工厂 |
| | `controller.reset()` | `→ void` | 清空曲线 |
| | `controller.pushSample(names, positions)` | `→ void` | 追加采样点 |
| | `controller.observeResize()` | `→ void` | 监听容器尺寸变化 |
| `services.js` | `createVisionServiceActions(opts)` | `opts → actions` | 服务调用绑定工厂 |
| | `actions.bindControlButtons()` | `→ void` | 绑定按钮点击事件 |
| | `actions.callGripperSwap(targetId, done)` | `str, fn → void` | 夹爪快换服务调用 |
| | `actions.callGetCurrentTool(done)` | `fn → void` | 查询后端当前工具 |
| | `actions.setCurrentToolId(id)` | `str → void` | 设置内部工具 ID |
| | `actions.getCurrentToolId()` | `→ str` | 获取内部工具 ID |
| | `actions.isSwapInFlight()` | `→ bool` | 快换是否进行中 |
| | `actions.onGripperSwapDone(fn)` | `fn → void` | 注册快换完成回调 |
| `ui_settings.js` | `createVisionSettingsController(opts)` | `opts → ctrl` | 设置控制器工厂 |
| | `ctrl.applyDefaultsToDom()` | `→ void` | 写默认值到 DOM |
| | `ctrl.readFromDom()` | `→ dict` | 读当前值 |
| | `ctrl.loadFromStorage()` | `→ bool` | 从 localStorage 恢复 |
| | `ctrl.saveToStorage()` | `→ void` | 持久化到 localStorage |
| | `ctrl.clearStorage()` | `→ void` | 清除持久化 |
| | `ctrl.openModal()` / `ctrl.closeModal()` | `→ void` | 弹窗开关 |
| | `ctrl.modalOpen()` | `→ bool` | 弹窗状态 |
| `ui_binder.js` | `createVisionUiBinder(opts)` | `opts → binder` | UI 事件绑定工厂 |
| | `binder.bindModeSwitches()` | `→ void` | 模式 radio 绑定 |
| | `binder.bindTopicSettingsUi()` | `→ void` | 设置面板按钮绑定 |
| | `binder.bindResultImageLoad()` | `→ void` | 结果图加载事件绑定 |
| `mode_controller.js` | `createVisionModeController(opts)` | `opts → ctrl` | 模式控制器工厂 |
| | `ctrl.syncModeUi()` | `→ void` | 同步模式 UI 可见性 |
| `urdf_panel.js` | `createVisionUrdfPanel(opts)` | `opts → panel` | URDF 面板工厂 |
| | `panel.start($)` | `→ void` | 启动 3D 查看器 |
| | `panel.stop()` | `→ void` | 停止 |
| | `panel.layout()` | `→ void` | 重新布局 |
| `subscription_binder.js` | `bindVisionSubscriptions(opts)` | `opts → void` | 按模式订阅全部话题 |

### 4.9 前端 JS — 3D 查看器

| 文件 | 导出/函数 | 签名 | 说明 |
|------|----------|------|------|
| `view3d/tf_clients.js` | `normalizeFrameId(frame)` | `str → str` | 标准化坐标系名 |
| | `ivgIdentityTransform()` | `→ tf` | 单位变换 |
| | `ivgCloneTransform(tf)` | `any → tf` | 深拷贝变换 |
| | `ivgQuatNormalize(q)` | `q → q` | 四元数归一化 |
| | `ivgQuatMultiply(a,b)` | `q,q → q` | 四元数乘法 |
| | `ivgQuatConjugate(q)` | `q → q` | 四元数共轭 |
| | `ivgRotateVectorByQuat(v,q)` | `v,q → v` | 四元数旋转向量 |
| | `ivgComposeTransforms(a,b)` | `tf,tf → tf` | 变换合成 |
| | `ivgInvertTransform(tf)` | `tf → tf` | 变换求逆 |
| | `ivgBuildTfPath(frame,edges)` | `→ [{frame,tf}]` | 构建 TF 路径 |
| | `ivgFindRelativeTransform(src,dst,edges)` | `→ tf\|null` | 查找相对变换 |
| `view3d/tf_clients.js` | `IvgRos3dTfClient(ros,fixedFrame,opts)` | 构造函数 | ROS2 TF 客户端 |
| | `client.getTransform(frame)` | `→ tf\|null` | 获取变换 |
| | `client.hasTransformFor(frame)` | `→ bool` | 是否有该坐标系 |
| | `client.subscribe(frame, cb)` | `→ void` | 订阅坐标系 |
| | `client.unsubscribe(frame, cb)` | `→ void` | 取消订阅 |
| | `client.setFixedFrame(frame)` | `→ bool` | 设置固定坐标系 |
| | `client.dispose()` | `→ void` | 销毁 |
| `view3d/session.js` | `IvgRos3dView3dSession(ros,$,opts)` | 构造函数 | 3D 查看器会话 |
| | `session.start()` | `→ void` | 创建 Viewer + TF + 雷达/标记/URDF |
| | `session.stop()` | `→ void` | 销毁所有 3D 对象 |
| | `session.layout()` | `→ void` | 重新布局 |
| `view3d/patches.js` | `ivgRos3dEmbeddedObject3DClass(viewer)` | `→ class` | THREE.Object3D polyfill |
| | `installIvgRos3dEmbeddedThreeSafeAddPatch(v)` | `→ void` | ros3d 安全 add 补丁 |
| `view3d/hints.js` | `showView3dUrdfHint(host, html)` | `→ void` | 显示 URDF 加载提示 |
| | `removeView3dUrdfHint(host)` | `→ void` | 移除提示 |

### 4.10 前端 JS — 面板主编排器

| 文件 | 内部函数 | 签名 | 说明 |
|------|---------|------|------|
| `vision_grasp_panel.js` | `connect()` | `async → void` | 连接 rosbridge → 订阅话题 |
| | `startSubscriptions()` | `→ void` | 清理旧订阅 → 重新绑定 |
| | `unsubscribeAll()` | `→ void` | 清理全部订阅和 UI |
| | `normalizeIvgTopic(t)` | `str → str` | 标准化话题名 |
| | `buildCameraInfoTopic(colorTopic)` | `str → str` | 推导 CameraInfo 话题 |
| | `sanitizeTopicValue(id, value)` | `str,str → str` | 清洗话题/服务值 |
| | `setConnStatus(text, ok)` | `str,bool\|null → void` | 更新连接状态显示 |
| | `logSvc(msg)` | `str → void` | 服务日志输出 |
| | `rosMsgArrayField(msg, key)` | `obj,str → arr` | 提取 ROS 消息数组字段 |
| | `pushJointStateSample(msg)` | `msg → void` | 追加关节采样 |
| | `refreshAiGraspnetColorImages(reason)` | `→ void` | 刷新 AI 模式快照 |
| | `scheduleGraspColorSnapshotRefresh()` | `→ void` | 防抖调度快照刷新 |
| | `suspendRealtimeForBackground()` | `→ void` | 后台暂停实时通信 |
| | `resumeRealtimeFromForeground()` | `→ void` | 前台恢复实时通信 |
| | `pageShouldPauseRealtime()` | `→ bool` | 页面是否隐藏 |
| | `bindMonitoringSectionCollapse()` | `→ void` | 底栏折叠交互 |
| | `syncMonitoringBundleMinHeight()` | `→ void` | 位姿列高度自适应 |
| | `_initToolStatusBridge()` | `→ void` | 夹爪工具状态桥接入口 (三层初始化) |
| | `_onToolStatusMsg(msg)` | `msg → void` | /tool_changer_status 消息处理 |
| | `_updateToolStatusUI(id, connected)` | `str,bool → void` | 更新工具状态 LED/标签/参数 |
| | `_updateGripperButtons(curId, swapping)` | `str,str\|null → void` | 更新夹爪按钮 active/swapping 状态 |
| | `_callGetCurrentToolAndSubscribe()` | `→ void` | 调用 /get_current_tool + 订阅 topic |
| | `_finalizeInit()` | `→ void` | 标记工具初始化完成 |
| | `_hideInitBanner()` / `_showInitBanner()` | `→ void` | 隐藏/显示手动选择面板 |

### 4.11 ROS 2 Launch

| 文件 | 函数 | 说明 |
|------|------|------|
| `web_dashboard.launch.py` | `generate_launch_description()` | 返回 LaunchDescription: 4 个进程的启动编排 |
| | `_find_rwt_assets(pkg_share)` | 查找 robotwebtools 资产目录 |

---

## 5. 调用时序图

### 5.1 后端启动时序

```
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
  │
  ├── [1] TL;DR_LIBRARY_PATH 注入
  │
  ├── [2] rosbridge 启动 (Tornado :9090)
  │     └── ROSLIB.Ros 通过此端口与 ROS 通信
  │
  ├── [3] tf2_web_republisher 启动
  │     └── TF 坐标变换通过此节点发布到 Web
  │
  ├── [4] web_video_server 启动 (:8089)
  │     └── 摄像头图像通过此 HTTP 服务暴露
  │
  └── [5] FastAPI 网关启动 (:8090)
        └── python3 -m aubo_ros2_web_dashboard.fastapi_static_gateway
              ├── import config (YAML 自动加载)
              ├── main(argv)
              │     ├── argparse 解析 CLI 参数
              │     ├── cfg.init(cli_overrides)
              │     └── create_app(directory, rwt_override)
              │           ├── SecurityHeadersMiddleware
              │           ├── GZipMiddleware
              │           ├── ws_router (/ws/rosbridge)
              │           ├── http_proxy_router (/api/ivg/proxy/web-video/*)
              │           ├── health.router (/health)
              │           ├── ivg_runtime.router (/api/v1/runtime)
              │           ├── robot_mesh.router (/api/ivg/robot-mesh/*)
              │           ├── StaticFiles (/js/robotwebtools/*)
              │           └── StaticFiles (/* → web/public)
              └── uvicorn.run(app, host, port)
```

### 5.2 前端启动时序

```
浏览器打开 vision_grasp_panel.html
  │
  ├── [1] 加载 JS 模块 (ES import 依赖链)
  │     config.js ← ivg_runtime.js ← runtime_provider.js
  │     ivg_transport.js ← ivg_runtime.js
  │     各 vision_grasp/* 子模块
  │
  ├── [2] DOMContentLoaded 触发
  │     │
  │     ├── loadRuntime()           GET /api/v1/runtime → __IVG_RUNTIME
  │     ├── loadTopicsFromStorage() localStorage → DOM 输入框
  │     ├── jointChart.observeResize()
  │     ├── bindMonitoring*()       底栏交互
  │     ├── bindModeSwitches()      工件/AI radio
  │     ├── bindControlButtons()    服务按钮
  │     ├── bindTopicSettingsUi()   设置弹窗
  │     │
  │     └── connect()  ──────────────────────────────────┐
  │           │                                           │
  │           ├── loadRuntime()        确保配置已加载      │
  │           ├── connectControl()     WS /ws/rosbridge   │
  │           │     └── ROSLIB.Ros({url})                 │
  │           │           ├── 'connection' → resolve      │
  │           │           ├── 'error' → dispatchControl   │
  │           │           └── 'close' → unsubscribeAll    │
  │           │                         + scheduleReconnect│
  │           │                                           │
  │           └── startSubscriptions()                    │
  │                 └── bindVisionSubscriptions()          │
  │                       │                               │
  │                       ├── robotTopic    → 末端位姿卡  │
  │                       ├── jointTopic    → 关节曲线    │
  │                       ├── statusTopic   → VPE 文本    │
  │                       ├── graspTopic    → 抓取投影    │
  │                       ├── cameraInfo    → 相机内参    │
  │                       ├── tfTopic       → TF 边表     │
  │                       └── tfStaticTopic → TF 边表     │
  │                                                       │
  └── [3] 运行时: ROS 消息 → UI 更新                      │
        │                                                 │
        ├── JointState → pushSample → Chart.js 更新       │
        ├── RobotStatus → formatRobotPoseHtml → 位姿卡    │
        ├── PoseArray → formatFinalGraspPoseHtml + 投影   │
        ├── CameraInfo + TF → 投影叠加 draw               │
        └── MJPEG <img> → 摄像头实时画面                  │
```

### 5.3 断线重连时序

```
rosbridge 断开
  │
  ├── ROSLIB.Ros 'close' 事件触发
  │     └── transport.onControlJson({op:'close'})
  │           └── setConnStatus('已断开…')
  │           └── unsubscribeAll()
  │           └── scheduleRosReconnect(state, connect, {maxAttempts:12})
  │                 │
  │                 ├── state.attempts++    (第1次: attempt=1)
  │                 ├── delay = min(30s, 2s * 2^(attempts-1))
  │                 │     第1次: 2s, 第2次: 4s, 第3次: 8s, …
  │                 │
  │                 ├── setTimeout(delay) → fire()
  │                 │     ├── 检查 gen (新连接已废弃旧重连)
  │                 │     └── connect() ← 回到启动流程
  │                 │
  │                 └── 页面隐藏时推迟到 visibilitychange
  │
  └── attempts >= 12 → onExhausted()
        └── setConnStatus('已达重连上限，请刷新页面')
```

### 5.4 摄像头数据流

```
ROS 摄像头话题 (/camera/color/image_raw)
  │
  ├── [路径 A] MJPEG 流 (实时预览)
  │     web_video_server → HTTP :8089/stream?topic=…
  │       → 网关 /api/ivg/proxy/web-video/stream?topic=…
  │         → <img src="…"> 直接渲染
  │           └── 断线 → mjpegStreamAttachAutoReload 自动重连
  │
  └── [路径 B] JPEG 快照 (AI 模式抓取瞬间)
        web_video_server → HTTP :8089/snapshot?topic=…
          → 网关 /api/ivg/proxy/web-video/snapshot?topic=…
            → <img src="…"> (带防抖刷新)
```

### 5.5 工具状态初始化时序

```
页面加载 (vision_grasp_panel.html)
  │
  ├── t=0: _initToolStatusBridge()
  │     ├── 第 1 层: localStorage.getItem('ivg_last_tool_id') → UI 占位
  │     │     若有值 → setCurrentToolId(stored), UI 显示 (灰色 LED)
  │     │     若无值 → 不设置
  │     ├── 绑定 #init-tool-select change 事件 → localStorage + UI 更新
  │     ├── 绑定 .gripper-btn 点击事件 (capture) → _toolSwapping
  │     ├── 绑定 .tool-init-btn 点击事件 → localStorage + 隐藏 banner
  │     └── _callGetCurrentToolAndSubscribe()
  │
  ├── t~0.5s: _subscribeToolStatus()
  │     └── transport.subscribe('/tool_changer_status')
  │         + transport.onRosJson('/tool_changer_status', _onToolStatusMsg)
  │
  ├── t~0-3s: /get_current_tool 服务调用
  │     ├── 成功且 tool_id 非空 → 覆盖 localStorage, 绿灯, 隐藏 banner
  │     └── 失败或空 → 不影响第 1 层状态
  │
  ├── t~5s: 首个 /tool_changer_status 消息到达 (后端周期性发布)
  │     ├── hasTool=true → 信任硬件, 更新全部, 隐藏 banner
  │     └── hasTool=false → 保留用户设置, 不改变 localStorage/select
  │
  └── t~8s: _initTimer 超时
        ├── getCurrentToolId() 有值 → _finalizeInit(), 结束
        └── getCurrentToolId() 为空 → _showInitBanner()
              └── 用户手动选择工具 → 隐藏 banner + UI 更新
                    └── 后续 /tool_changer_status 空消息不覆盖 (保护机制)
```

---

## 6. 文件依赖矩阵

```
                     ┌─────┐
                     │ YAML│
                     └──┬──┘
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
   config.py      launch.py       setup.py
        │
   ┌────┼────┬─────────┐
   ↓    ↓    ↓         ↓
  cli  app  upstream  ivg_runtime
        │
   ┌────┼────┬─────────┐
   ↓    ↓    ↓         ↓
health robot_mesh  static_files  proxy_routes

═══════════════════════════════════════════

前端依赖 (JS import 链):

runtime_provider.js  ← 无依赖
       ↑
ivg_runtime.js       ← runtime_provider
       ↑
ivg_transport.js     ← runtime_provider, ivg_runtime
       ↑
config.js            ← 无依赖 (读 globalThis.__IVG_RUNTIME)
       ↑
subscription_binder  ← ivg_transport, config, pose_card, projection_overlay
       ↑
┌──────┴──────┐
│ 所有子模块   │
│             │
│ pose_card   │ ← 无依赖
│ projection  │ ← tf_clients
│ joint_chart │ ← Chart.js
│ services    │ ← ivg_transport
│ ui_settings │ ← config
│ ui_binder   │ ← config
│ mode_ctrl   │ ← 无依赖
│ urdf_panel  │ ← session, ivgPorts
│             │
│ session     │ ← tf_clients, patches, hints
│ tf_clients  │ ← ROSLIB
│ patches     │ ← ROS3D
│ hints       │ ← 无依赖
│             │
└──────┬──────┘
       ↓
vision_grasp_panel.js  ← 所有以上模块
```

---

## 7. 配置项速查

`config/defaults.yaml` 中的所有配置项：

| 路径 | 默认值 | 说明 |
|------|--------|------|
| `gateway.bind` | `0.0.0.0` | 网关监听地址 |
| `gateway.port` | `8090` | 网关监听端口 |
| `gateway.gzip_min_size` | `512` | GZip 压缩阈值(字节) |
| `rosbridge.host` | `127.0.0.1` | rosbridge 上游主机 |
| `rosbridge.port` | `9090` | rosbridge 上游端口 |
| `rosbridge.ws_path` | `/ws/rosbridge` | 代理 WS 路径 |
| `rosbridge.max_message_bytes` | `67108864` | 最大消息字节(64MiB) |
| `rosbridge.ping_interval` | `20` | 心跳间隔(秒) |
| `rosbridge.ping_timeout` | `60` | 心跳超时(秒) |
| `rosbridge.close_timeout` | `10` | 关闭超时(秒) |
| `web_video.host` | `127.0.0.1` | web_video 上游主机 |
| `web_video.port` | `8089` | web_video 上游端口 |
| `web_video.listen_address` | `0.0.0.0` | web_video 监听地址 |
| `web_video.server_threads` | `4` | 服务线程数 |
| `web_video.ros_threads` | `2` | ROS 线程数 |
| `web_video.proxy_path_prefix` | `/api/ivg/proxy/web-video` | 代理 URL 前缀 |
| `proxy.video_connect_timeout` | `15` | httpx 连接超时(秒) |
| `proxy.video_pool_timeout` | `10` | httpx 连接池超时(秒) |
| `proxy.video_chunk_bytes` | `65536` | 流式块大小(64KiB) |
| `package.version_fallback` | `0.4.0` | 版本回退 |
| `robotwebtools.search_dirs` | `[...]` | RWT 资产候选目录 |
| `vision_panel.topics` | 6 项 | 话题默认值 |
| `vision_panel.tf_topics` | 2 项 | TF 话题默认值 |
| `vision_panel.services` | 4 项 | 服务默认值 |
| `vision_panel.fixed_service_types` | 1 项 | 固定服务类型 |

---

## 8. 夹爪/工具状态管理系统

### 8.1 数据源层次

```
┌──────────────────────────────────────────────────────────────────┐
│                    工具状态数据来源                                │
│                                                                   │
│  后端 (GripperSwapWorker)                                        │
│  ├── /get_current_tool 服务  → 返回 current_tool_ (单次查询)      │
│  ├── /tool_changer_status 话题 → 定期发布 (每 5s + 状态变更时)    │
│  └── /run_gripper_swap 服务   → 执行物理快换 + 更新 current_tool_ │
│                                                                   │
│  前端 (vision_grasp_panel.js)                                    │
│  ├── localStorage key: ivg_last_tool_id  → 跨会话持久化           │
│  ├── #init-tool-select <select>          → 下拉框手动选择         │
│  └── .tool-init-btn / .gripper-btn       → 按钮手动选择/快换      │
└──────────────────────────────────────────────────────────────────┘
```

### 8.2 三层初始化策略

页面加载时按以下顺序确定当前工具：

```
┌─────────────────────────────────────────────────────────────────────┐
│ 第 1 层: localStorage → 即时 UI 占位                               │
│   _initToolStatusBridge() → localStorage.getItem('ivg_last_tool_id')│
│   立即写入 UI (灰色 LED, 标记为手动设定)                             │
│                                                                     │
│ 第 2 层: /get_current_tool → 后端查询 (3s 超时)                     │
│   _callGetCurrentToolAndSubscribe() → callGetCurrentTool()           │
│   成功且 tool_id 非空 → 覆盖 localStorage, 绿灯, 隐藏 init banner   │
│   失败或空 tool_id → 不改变第 1 层状态                               │
│                                                                     │
│ 第 3 层: /tool_changer_status 订阅 → 实时更新                       │
│   0.5s 后建立订阅, _onToolStatusMsg 处理每条消息                     │
│   8s 总超时后仍未检测到工具 → 显示手动选择面板 (#tool-init-banner)   │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.3 /tool_changer_status 消息处理规则

`_onToolStatusMsg(msg)` 的核心决策逻辑：

| 后端消息 | 行为 | 理由 |
|----------|------|------|
| `connected=true, tool_id` 非空 | 信任硬件：更新 localStorage、下拉框、currentToolId、状态 UI | 真机硬件是唯一权威数据源 |
| `connected=false` 或 `tool_id=""` | **不覆盖**用户手动设置：保留 localStorage/select/currentToolId，状态 UI 用有效工具 ID | 仿真模式后端无工具信息，不应冲掉用户选择 |

```
_onToolStatusMsg(msg) 决策树:

msg 到达
  │
  ├─ tool_id 非空 && connected=true (hasTool=true)
  │     → setCurrentToolId(toolId)           // 更新内部状态
  │     → localStorage.setItem(key, toolId)  // 持久化
  │     → sel.value = toolId                 // 更新下拉框
  │     → _updateToolStatusUI(toolId, true)  // 绿色 LED + 工具名
  │     → _updateGripperButtons(toolId)      // 高亮对应按钮
  │     → _hideInitBanner()                  // 隐藏手动选择面板
  │
  └─ tool_id 为空 或 connected=false (hasTool=false)
        → effectiveId = getCurrentToolId()   // 保留用户手动设置
        → _updateToolStatusUI(effectiveId, !!effectiveId)
        → _updateGripperButtons(effectiveId)
        → 不改变 localStorage / select / currentToolId
```

### 8.4 仿真模式 vs 真机模式

| 维度 | 真机模式 | 仿真模式 |
|------|---------|---------|
| 后端 `GripperSwapWorker` | 运行，连接真实 SDK | 运行，`simulation_skip_io_=true` |
| `initial_tool_id` 参数 | 从 launch 参数传入 (物理工具已安装) | 通常为空 (无物理工具) |
| `/tool_changer_status` 消息 | `tool_id="gripper0"`, `connected=true` | `tool_id=""`, `connected=false` |
| `/get_current_tool` 返回 | `tool_id` 非空 | `tool_id=""`, `is_connected=false` |
| 前端行为 | 自动识别工具，init banner 不显示 | 8s 后显示手动选择面板 |
| 用户选择后 | 后端确认 → 覆盖用户选择 (信任硬件) | 后端持续发空消息 → 保留用户选择 |

### 8.5 手动工具设置保护机制

**问题**：仿真模式下后端每 5s 发布空 `/tool_changer_status`，会冲掉用户手动选择的工具。

**保护规则** (2026-05-25 修复)：
1. `localStorage` 和 `<select>` 仅在 `hasTool=true` 时写入 — 空消息不覆盖
2. `setCurrentToolId()` 仅在 `hasTool=true` 时调用 — 内部状态不丢失
3. `_updateToolStatusUI()` 使用 `effectiveId`（后端有工具用后端，否则用用户设置）
4. `_updateGripperButtons()` 使用 `effectiveId` — 按钮高亮不丢失

### 8.6 夹爪快换调用链

```
用户点击 .gripper-btn (id="btn-quick-swap-0/1/2")
  │
  ├── [capture] #gripper-swap-btns 事件委托
  │     ├── 同工具? → return (跳过)
  │     ├── isSwapInFlight() → return (防重入)
  │     ├── _toolSwapping = true
  │     └── _updateGripperButtons(currentId, targetId)  // swapping 动画
  │
  └── [bubble] btnQuickSwapX.onclick
        └── callGripperSwap(targetId)
              ├── 防重入: _swapInFlight 检查
              ├── direction = _currentToolId
              │       ? (_currentToolId + '_to_' + targetId)
              │       : targetId
              ├── transport.callService('/run_gripper_swap', {direction})
              │     ├── .then(r) → _finishSwap(null)
              │     │     └── onGripperSwapDone 回调:
              │     │           _toolSwapping = false
              │     │           _updateToolStatusUI(getCurrentToolId())
              │     │           _updateGripperButtons(getCurrentToolId())
              │     └── .catch(e) → _finishSwap(e)
              └── 后端 changeToTool(targetId):
                    1. releaseTool(current_tool) → publishToolStatus(false)
                    2. pickTool(target_tool)     → publishToolStatus(true)
```

### 8.7 相关文件速查

| 文件 | 角色 |
|------|------|
| `vision_grasp_panel.js:599-817` | 工具状态桥接 (_initToolStatusBridge, _onToolStatusMsg 等) |
| `vision_grasp/services.js:74-148` | 夹爪服务封装 (callGripperSwap, getCurrentToolId 等) |
| `vision_grasp_panel.html:120-148` | 工具状态栏 + 初始化面板 + 快换按钮 HTML |
| `vision_grasp/config.js:20` | `/tool_changer_status` 话题默认配置 |
| `core/topics.js:13` | `TOOL_CHANGER_STATUS_TOPIC` 常量 |
| 后端 `gripper_swap_worker.cpp:116-117` | `/tool_changer_status` 发布者 (每 5s) |
| 后端 `gripper_swap_worker.cpp:131-134` | `/get_current_tool` 服务回调 |
| 后端 `gripper_swap_worker.cpp:121-124` | `/run_gripper_swap` 服务回调 → changeToTool() |
