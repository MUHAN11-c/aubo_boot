# IVG Web Dashboard 架构图（Mermaid）

本文档记录当前包的整体结构、前后端职责、页面脚本链，以及本次模块化后的 3D / topics_lab 分层。

- 源码路径：`docs/architecture_diagrams.md`
- 安装后路径：`share/aubo_ros2_web_dashboard/docs/architecture_diagrams.md`
- 维护原则：改 `launch/`、`gateway/`、`web/public/js/`、`tests/` 或 vendor 生成方式时，请同步更新本文。

## 1. 运行时总览

```mermaid
flowchart LR
  subgraph Browser["浏览器"]
    HTML["HTML + CSS + 自研 JS"]
    Vendor["roslib / ros2d / ros3d / three"]
    HTML --- Vendor
  end

  subgraph Gateway["FastAPI 网关"]
    App["gateway.app / routes"]
  end

  subgraph ROS["ROS 2 现场进程"]
    RB["rosbridge_suite"]
    WV["web_video_server"]
    TF["tf2_web_republisher"]
  end

  Browser -->|"GET /health / runtime-config / 静态文件"| Gateway
  Browser -->|"WS /ws/rosbridge"| Gateway
  Browser -->|"HTTP /api/ivg/proxy/web-video/*"| Gateway
  Browser -->|"HTTP /api/ivg/robot-mesh/*"| Gateway

  Gateway -->|"websockets"| RB
  Gateway -->|"httpx"| WV
  Browser -.->|"ROSLIB.ROS2TFClient"| RB
  TF -.->|"TF republisher"| Browser
```

## 2. Launch 与网关

```mermaid
flowchart TB
  subgraph Launch["launch/web_dashboard.launch.py"]
    L1["Include rosbridge_websocket_launch.xml"]
    L2["Node tf2_web_republisher"]
    L3["Node web_video_server"]
    L4["ExecuteProcess FastAPI gateway"]
  end

  subgraph Env["注入环境变量"]
    E1["IVG_ROSBRIDGE_HOST / PORT"]
    E2["IVG_WEB_VIDEO_HOST / PORT"]
  end

  subgraph Python["Python 包"]
    AP["gateway/app.py"]
    RT["gateway/routes/*"]
    CLI["gateway/cli.py"]
  end

  L4 --> CLI --> AP --> RT
  Launch -.-> Env -.-> RT
```

## 3. Python 结构

```mermaid
flowchart TB
  subgraph Root["包根"]
    Package["package.xml / setup.py / setup.cfg"]
    Docs["README.md / docs/*"]
    Tests["tests/test_gateway.py"]
  end

  subgraph Pkg["aubo_ros2_web_dashboard/"]
    Entry["fastapi_static_gateway.py"]
    subgraph GW["gateway/"]
      Settings["settings.py"]
      App["app.py"]
      Cli["cli.py"]
      Health["routes/health.py"]
      Runtime["routes/ivg_runtime.py"]
      Proxy["routes/upstream_proxy.py"]
      Mesh["routes/robot_mesh.py"]
    end
  end

  Entry --> App
  App --> Health
  App --> Runtime
  App --> Proxy
  App --> Mesh
  Settings -.-> Runtime
  Settings -.-> Proxy
```

## 4. 前端模块分层

```mermaid
flowchart TB
  subgraph Shared["共享层"]
    Runtime["ivg_runtime.js / ivg_transport.js"]
    Core["core/dom_cache.js"]
  end

  subgraph View3D["3D 模块层"]
    VPatch["view3d/patches.js"]
    VTf["view3d/tf_clients.js"]
    VHints["view3d/hints.js"]
    VPc["view3d/pointcloud.js"]
    VUrdf["view3d/urdf_loader.js"]
    VSess["view3d/session.js"]
  end

  subgraph Topics["topics_lab"]
    TPrev["topics_lab/render_preview.js"]
    TViz["topics_lab/render_visualizers.js"]
    TSess["topics_lab/view_sessions.js"]
    Console["ros_console.js"]
  end

  subgraph Vision["vision_grasp"]
    VisionEntry["vision_grasp_panel.js"]
  end

  Runtime --> Console
  Runtime --> VisionEntry
  Core --> Console
  Core --> VisionEntry

  VPatch --> VSess
  VTf --> VPc
  VTf --> VSess
  VHints --> VPc
  VHints --> VUrdf
  VUrdf --> VSess
  VPc --> VSess

  TPrev --> TViz --> Console
  TSess --> Console
  VSess --> Console
  VSess --> VisionEntry
```

## 5. `topics_lab.html` 脚本链

```mermaid
flowchart LR
  A["ivg_transport.js"] --> B["ivg_runtime.js"]
  B --> C["core/dom_cache.js"]
  C --> D["vendor/eventemitter2 + roslib-2.iife + easeljs + three + ros3d"]
  D --> E["ivg_roslib_ros2_humble_compat.js"]
  E --> F["view3d/*.js"]
  F --> G["topics_lab/render_preview.js"]
  G --> H["topics_lab/render_visualizers.js"]
  H --> I["topics_lab/view_sessions.js"]
  I --> J["ros_console.js"]
```

## 6. `vision_grasp_panel.html` 脚本链

```mermaid
flowchart LR
  A["ivg_transport.js"] --> B["ivg_runtime.js"]
  B --> C["core/dom_cache.js"]
  C --> D["vendor/eventemitter2 + roslib-2.iife + three + ros3d"]
  D --> E["ivg_roslib_ros2_humble_compat.js"]
  E --> F["view3d/*.js"]
  F --> G["vision_grasp_panel.js"]
```

## 7. vendor 与项目层补丁关系

```mermaid
flowchart TB
  Bundle["scripts/bundle_roslib2_browser.sh<br/>vendor 单一生成脚本"]
  Vendor["web/public/js/vendor/*"]
  Compat["ivg_roslib_ros2_humble_compat.js"]
  View3D["view3d/*.js"]
  Mesh["gateway/routes/robot_mesh.py"]

  Bundle --> Vendor
  Vendor --> Compat
  Compat --> View3D
  View3D --> Mesh
```

说明：

- `Bundle` 会同步 `eventemitter2 / roslib / ros2d / ros3d / three / easeljs`，并对 `ros2d.min.js`、`ros3d.min.js` 打 ROS 2 / Humble 兼容补丁。
- `Compat` 继续在 vendor 之上补 Topic 参数和 URDF 枚举兼容。
- `Mesh` 负责浏览器侧 `.dae/.stl` 大小写与 `package://` 网格同源访问。

## 8. 测试与验证

```mermaid
flowchart LR
  Tests["tests/test_gateway.py"] --> App["gateway.app.create_app"]
  App --> Health["/health"]
  App --> Runtime["/api/ivg/runtime-config"]
  App --> Proxy["/api/ivg/proxy/web-video/*"]
  App --> Mesh["/api/ivg/robot-mesh/*"]
```

当前 `tests/test_gateway.py` 覆盖：

- 健康检查
- runtime-config
- web_video 路径穿越防护
- web_video 上游不可达
- robot_mesh 路径穿越防护
- robot_mesh 大小写文件名兼容
