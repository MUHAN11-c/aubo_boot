# IVG Web Dashboard 架构图（Mermaid）

本文档存放 **总览 + 分模块** 架构图，便于评审、打印（A4）与随代码迭代更新。

- **源码路径**：`aubo_ros2_web_dashboard/docs/architecture_diagrams.md`
- **安装后**（`colcon build`）：`share/aubo_ros2_web_dashboard/docs/architecture_diagrams.md`
- **维护**：变更 `launch/`、`gateway/`、`web/public/js/`、`test/` 或依赖库职责时，请同步检查下图是否仍准确。（pytest 目录为 ROS 2 惯例 **`test/`**。）

**预览**：在 VS Code / Cursor 装 Mermaid 插件，或用 [Mermaid Live Editor](https://mermaid.live) 粘贴代码块。

**A4 打印建议**：总览单独一页或占上半页；**第 1～4 节** 模块图可一页多格或分两页；**第 5～6 节（文件作用）**、**第 7～8 节（文件联系）** 建议各用 **1～2 页 A4** 打印；导出 PDF 时缩放约 90%～100%。

---

## 1. 总览（库与连接关系）

```mermaid
flowchart LR
  subgraph BR["浏览器"]
    B1["页面 HTML + 自研 JS<br/>ivg_runtime / ros_console / vision_grasp / ivg_web_video …"]
    BL["第三方：roslibjs<br/>ros2djs · ros3djs · three（vendor）"]
    B1 --- BL
  end

  subgraph GW["Python 网关进程"]
    G1["自研：aubo_ros2_web_dashboard.gateway<br/>FastAPI 应用工厂 + routes"]
    GL["第三方：fastapi · starlette<br/>uvicorn · httpx · websockets"]
    G1 --- GL
  end

  subgraph ROS["本机 ROS 2 进程（launch 拉起）"]
    R1["rosbridge_suite（WS）"]
    R2["web_video_server（HTTP）"]
    R3["tf2_web_republisher"]
    R1 ~~~ R2 ~~~ R3
  end

  BR -->|"HTTP 静态页<br/>GET /api/ivg/runtime-config"| GW
  BR -->|"WebSocket<br/>/ws/rosbridge（默认）"| GW
  BR -->|"HTTP 视频<br/>/api/ivg/proxy/web-video/*"| GW

  GW -->|"websockets 客户端"| R1
  GW -->|"httpx 流式 GET"| R2
  BR -.->|"roslib TF 等经 rosbridge"| R1
  R3 -.->|"TF"| BR
```

---

## 2. 模块：Launch + 本机进程（合并）

```mermaid
flowchart TB
  subgraph L["launch：web_dashboard.launch.py"]
    L1["Include：rosbridge_websocket_launch.xml"]
    L2["Node：tf2_web_republisher_node"]
    L3["Node：web_video_server（条件）"]
    L4["ExecuteProcess：python3 -m aubo_ros2_web_dashboard.fastapi_static_gateway<br/>--directory share/.../web/public；additional_env → IVG_ROSBRIDGE_* / IVG_WEB_VIDEO_*"]
    L1 --> L4
    L2 --> L4
    L3 --> L4
  end

  subgraph P["本机进程与库"]
    P1["rosbridge_suite → 依赖 rosapi 等<br/>对外：Tornado WebSocket"]
    P2["web_video_server → HTTP MJPEG"]
    P3["tf2_web_republisher → Action/TF 节流"]
    L1 -.-> P1
    L2 -.-> P3
    L3 -.-> P2
  end

  subgraph E["网关读环境"]
    E1["IVG_ROSBRIDGE_HOST:PORT → WS 代理上游（launch 注入）"]
    E2["IVG_WEB_VIDEO_HOST:PORT → HTTP 代理上游（launch 注入）"]
  end
  L4 -.-> E
```

---

## 3. 模块：网关 Python（包内文件 + 第三方库）

```mermaid
flowchart TB
  subgraph PKG["包：aubo_ros2_web_dashboard"]
    S["gateway/settings.py<br/>环境变量 · runtime-config 字典"]
    A["gateway/app.py<br/>GZip · 安全头 · include_router · StaticFiles"]
    H["routes/health.py"]
    I["routes/ivg_runtime.py"]
    U["routes/upstream_proxy.py"]
    C["gateway/cli.py + fastapi_static_gateway.py<br/>→ uvicorn.run"]
    C --> A
    A --> H
    A --> I
    A --> U
    S -.-> I
    S -.-> U
  end

  subgraph LIB["第三方库 ↔ 职责"]
    F["fastapi / starlette<br/>WebSocket · HTTP · StreamingResponse · JSONResponse"]
    HX["httpx.AsyncClient<br/>视频代理 stream=True · trust_env=False"]
    WS["websockets<br/>连接 rosbridge 上游"]
    UV["uvicorn<br/>ASGI 宿主"]
    U --> F
    U --> HX
    U --> WS
    C --> UV
  end
```

---

## 4. 模块：前端 JS（文件依赖 + 与网关协议）

```mermaid
flowchart TB
  subgraph PG["页面与加载顺序（示例）"]
    T["topics_lab.html"]
    V["vision_grasp_panel.html"]
  end

  subgraph JS["自研脚本（依赖方向）"]
    IR["ivg_runtime.js<br/>fetch runtime-config · 重连状态 API"]
    IW["ivg_web_video.js<br/>拼 stream/snapshot URL"]
    IB["ivg_rosbridge_bytes.js"]
    RC["ros_console.js"]
    VG["vision_grasp_panel.js"]
    IC["ivg_image_canvas.js（仅 vision）"]
    IR --> RC
    IR --> VG
    IW --> RC
    IW --> VG
    IB --> RC
    IB --> IC --> VG
  end

  subgraph VD["vendor（静态拷贝）"]
    RL["roslib.min.js"]
    R2["ros2d / ros3d + three"]
  end
  RC --> RL
  RC --> R2
  VG --> RL

  subgraph NET["与网关/上游关系"]
    N1["GET /api/ivg/runtime-config ← IR"]
    N2["WebSocket /ws/rosbridge ← RL（经网关）"]
    N3["GET /api/ivg/proxy/web-video/… ← img / IW"]
  end
  IR --> N1
  RL --> N2
  IW --> N3
```

---

## 5. 文件职责（A4 ①）：包根 + Python 网关

**用途**：改后端、加路由、动环境变量时对照。**注解**：`upstream_proxy` 与 `ivg_runtime` 均依赖 `settings`；`app` 先注册全部路由再挂 `StaticFiles`，避免静态资源抢 API/WS。

```mermaid
flowchart TB
  subgraph R["包根 aubo_ros2_web_dashboard/"]
    direction TB
    P["package.xml<br/>ROS 2 包元数据、exec/依赖（rosbridge 等）"]
    S["setup.py<br/>setuptools：Python 包、安装 launch/web/docs、pip 依赖"]
    C["setup.cfg<br/>与构建相关的 setuptools 配置"]
    RM["README.md<br/>使用说明、环境变量、架构摘要"]
    DOC["docs/architecture_diagrams.md<br/>本文档：Mermaid 图"]
    RS["resource/aubo_ros2_web_dashboard<br/>空标记文件，供 ament 索引包名"]
  end

  subgraph PKG["Python 包 aubo_ros2_web_dashboard/"]
    I0["__init__.py<br/>包标识"]
    FSG["fastapi_static_gateway.py<br/>薄入口：python -m / console_script<br/>导出 create_app、main"]
    subgraph G["gateway/"]
      GI["__init__.py"]
      ST["settings.py<br/>IVG_* 默认值、runtime-config 字典"]
      AP["app.py<br/>create_app：GZip、安全头、注册路由、挂载 StaticFiles"]
      CL["cli.py<br/>解析端口/bind/目录，调用 uvicorn"]
      subgraph RT["gateway/routes/"]
        RY["__init__.py<br/>路由子包"]
        HE["health.py<br/>GET /health + static_root"]
        IV["ivg_runtime.py<br/>GET /api/ivg/runtime-config"]
        UP["upstream_proxy.py<br/>WS 转 rosbridge、HTTP 转 web_video"]
      end
    end
  end
```

---

## 6. 文件职责（A4 ②）：Launch、测试、Web 静态

**用途**：改前端页面、加静态资源时对照。**说明**：测试目录名为 ROS 2 惯例 **`test/`**（非 `tests/`）；`js/vendor/` 下大型 `*.min.js` 合并为一个节点表示。

```mermaid
flowchart TB
  subgraph L["launch/"]
    WL["web_dashboard.launch.py<br/>编排 rosbridge launch、tf2_web、web_video、<br/>子进程启动 FastAPI 网关并注入 IVG_*"]
  end

  subgraph T["test/（ROS 2 惯例）"]
    TG["test_gateway.py<br/>pytest：health、runtime-config、视频路径"]
  end

  subgraph W["web/public/ 静态站点根（由网关托管）"]
    subgraph HTML["页面"]
      IX["index.html 门户"]
      TL["topics_lab.html ROS 控制台"]
      VG["vision_grasp_panel.html 视觉抓取"]
      CF["coffee_latte_panel.html 演示页"]
    end
    subgraph CSS["css/"]
      HC["home.css · topics_lab.css · vision_grasp_panel.css<br/>coffee_latte_panel.css · ivg_site_nav.css"]
    end
    subgraph JS["js/ 自研脚本"]
      IR["ivg_runtime.js 端口/代理/runtime-config/重连调度"]
      IW["ivg_web_video.js MJPEG URL 拼接"]
      IB["ivg_rosbridge_bytes.js 字节字段解析"]
      II["ivg_image_canvas.js Image→Canvas（vision）"]
      ISN["ivg_site_nav.js 全站导航"]
      RC["ros_console.js topics_lab 主逻辑"]
      VR["vision_grasp_panel.js 视觉面板主逻辑"]
      CO["coffee_latte_panel.js 演示逻辑"]
      TR["topics_lab/render.js 消息类型→视图"]
    end
    subgraph VD["js/vendor/ 第三方（roslib/ros2d/ros3d/three 等）"]
      VN["*.min.js + eventemitter2 + README + fetch_vendor.sh<br/>浏览器 ROS 与 2D/3D 可视化运行时"]
    end
    subgraph AS["assets/"]
      ASR["README、icons/images 占位"]
    end
  end
```

---

## 7. 文件联系（A4 ③）：运行时与数据流

**用途**：联调、防火墙、端口说明。**注解**：浏览器始终经同源 `/ws/rosbridge` 与 `/api/ivg/proxy/web-video`；上游地址由 `IVG_ROSBRIDGE_*` / `IVG_WEB_VIDEO_*` 决定。

```mermaid
flowchart LR
  subgraph RUN["运行时进程关系"]
    LP["launch<br/>web_dashboard.launch.py"] --> RB["rosbridge_suite"]
    LP --> TF["tf2_web_republisher"]
    LP --> WV["web_video_server"]
    LP --> UV["uvicorn 子进程<br/>python3 -m aubo_ros2_web_dashboard.fastapi_static_gateway"]
  end

  subgraph ENV["环境注入"]
    LP -.->|"additional_env"| E1["IVG_ROSBRIDGE_*<br/>IVG_WEB_VIDEO_*"]
    E1 -.-> UV
  end

  subgraph BR["浏览器"]
    PG["HTML 引 JS/CSS"] -->|"HTTP 同源"| UV
    PG -->|"fetch"| CF["/api/ivg/runtime-config"]
    PG -->|"WebSocket"| WS["/ws/rosbridge"]
    PG -->|"GET 图片流"| HV["/api/ivg/proxy/web-video/…"]
  end

  UV --> SF["StaticFiles → web/public"]
  UV --> CF
  UV --> WS
  UV --> HV

  WS -->|"websockets 库"| RB
  HV -->|"httpx 库"| WV
```

---

## 8. 文件联系（A4 ④）：源码 import 与 HTML 脚本链

**用途**：重构、拆包、查依赖。**注解**：箭头表示 **import** 或 **典型 `<script>` 加载顺序**；各 HTML 实际顺序以文件为准。

```mermaid
flowchart TB
  subgraph PYD["Python 依赖链"]
    FSG["fastapi_static_gateway"] --> AP["gateway.app.create_app"]
    AP --> HE["routes.health"]
    AP --> IV["routes.ivg_runtime"]
    AP --> UP["routes.upstream_proxy"]
    IV --> ST["gateway.settings"]
    UP --> ST
  end

  subgraph TST["测试"]
    TG["test/test_gateway.py"] --> AP
  end

  subgraph PG1["topics_lab.html 脚本链（示意）"]
    TL1["ivg_web_video.js"] --> TL2["ivg_runtime.js"]
    TL2 --> TL3["ivg_rosbridge_bytes.js"]
    TL3 --> TL4["topics_lab/render.js"]
    TL4 --> TL5["ros_console.js"]
    TL5 --> TL6["vendor：roslib + ros2d + ros3d + three + …"]
  end

  subgraph PG2["vision_grasp_panel.html 脚本链（示意）"]
    V1["ivg_web_video.js"] --> V2["ivg_runtime.js"]
    V2 --> V3["ivg_rosbridge_bytes.js"]
    V3 --> V4["ivg_image_canvas.js"]
    V4 --> V5["vision_grasp_panel.js"]
    V5 --> V6["vendor：roslib + …"]
  end

  subgraph PG0["index / coffee / 导航"]
    IX["index.html"] -.-> SN["ivg_site_nav.css/js（若引用）"]
    CF["coffee_latte_panel"] -.-> CFJ["coffee_latte_panel.js/css"]
  end
```

### 第 5～8 节打印分页建议

| 建议页 | 章节 | 内容 |
|--------|------|------|
| A4 ① | 第 5 节 | 包根 + Python 网关文件职责 |
| A4 ② | 第 6 节 | launch、test、web/public 文件职责 |
| A4 ③ | 第 7 节 | 运行时进程与浏览器—网关—上游 |
| A4 ④ | 第 8 节 | Python import 与 HTML 脚本依赖链 |

---

## 更新对照表（改代码时扫一眼）

| 变更区域 | 建议同步的图 |
|----------|----------------|
| `launch/web_dashboard.launch.py`、环境变量注入 | 第 2、1、6、7 节 |
| `gateway/*`、`upstream_proxy`、pip 依赖 | 第 3、1、5、7、8 节 |
| `web/public/js/*`、HTML 脚本顺序、vendor | 第 4、1、6、8 节 |
| 端口与环境变量约定 | 第 2 节（E）、第 4 节（NET）、第 7 节 |
| 新增/删除源码文件、移动目录、`test/` 用例 | 第 5、6、8 节 |
