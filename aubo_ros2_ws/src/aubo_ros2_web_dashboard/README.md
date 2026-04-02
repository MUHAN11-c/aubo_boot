# aubo_ros2_web_dashboard

**详细结构、实现逻辑与使用说明**见：[docs/ARCHITECTURE_AND_USAGE.md](docs/ARCHITECTURE_AND_USAGE.md)。

**IVG 统一 Web 网关（默认 `8090`）**：

1. **visual_pose_estimation_python 的全部 Web 能力**：挂载与官方一致的 **`/legacy-ui`**（`web_ui`），并将 **`/api/*`、`/health`、`/status`、`POST /exit`** 等 **反向代理** 到上游 VPE FastAPI（默认 **`http://127.0.0.1:8088`**）。浏览器仍使用 `window.location.origin`，无需改 `web_ui/scripts/app.js`。
2. **RWT 仪表板**：**`/rwt/`** — 入口页选择 **演示**（`demo.html`，大图 + 预设话题 + 自动连 rosbridge）或 **开发调试**（`dev.html`，完整话题与 WebSocket 信息）；共用 **`GET /api/config`**（仅网关本地，不转发）。

网关进程内 **无 rclpy**；视觉算法、相机采集、模板与调试等仍由 **VPE 节点 + FastAPI（8088）** 完成。

## 依赖

```bash
sudo apt install ros-humble-rosbridge-suite ros-humble-rosapi ros-humble-tf2-web-republisher python3-httpx
```

工作空间需 **已编译** `visual_pose_estimation_python`（供 `share/visual_pose_estimation_python/web_ui` 与 launch 引用）。

## 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_pose_estimation_python aubo_ros2_web_dashboard
source install/setup.bash
```

## 启动方式

### A. 手动（推荐调试）

终端 1 — VPE Web（须监听 `127.0.0.1:8088`，与网关 `AUBO_VPE_UPSTREAM` 一致）：

```bash
source install/setup.bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py host:=127.0.0.1 port:=8088
```

终端 2 — 本包（rosbridge + 网关；代理指向上游）：

```bash
source install/setup.bash
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
```

浏览器打开 **`http://<主机>:8090/`** → 进入与原先 **`8088`** 相同的 **legacy UI**；所有 **`/api/...`** 由网关转发到 **8088**。

### B. 一键带起 VPE Web

```bash
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py launch_vpe_web:=true
```

将额外启动 `visual_pose_estimation_web`（`127.0.0.1:8088`）。网关仍监听 **`web_port`（默认 8090）**。

### 主要 launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `web_host` | `0.0.0.0` | 网关 FastAPI |
| `web_port` | `8090` | 统一入口端口 |
| `vpe_upstream` | `http://127.0.0.1:8088` | 反向代理目标；会写入进程环境 `AUBO_VPE_UPSTREAM` |
| `launch_vpe_web` | `false` | 是否同时启动 VPE Web 节点 |
| `rosbridge_port` | `9090` | RWT WebSocket |

## 路径一览

| 路径 | 说明 |
|------|------|
| `/` | 重定向至 `/legacy-ui/index.html`（若已安装 VPE web_ui） |
| `/legacy-ui/*` | 视觉位姿静态界面（与直接跑 VPE 时一致） |
| `/static/*` | `web_ui/static`（legacy 资源） |
| `/api/capture_image`、`/api/estimate_pose`、… | 代理到 VPE（与 [visual_pose_estimation_python/web/routers](file:///home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/visual_pose_estimation_python/web/routers) 一致） |
| `GET /api/config` | **仅网关**：RWT（rosbridge 端口、IVG 预设话题等） |
| `GET /health`、`GET /status` | 代理到 VPE（供脚本/监控使用） |
| `GET /gateway/health` | 仅网关进程是否存活（Liveness；含 `X-Request-ID`） |
| `GET /gateway/ready` | 可选 Readiness；设 `AUBO_WEB_PROBE_VPE=1` 时探测上游 VPE `/health` |
| `/rwt/` | RWT **入口**；全站含**顶栏导航**（首页 / 演示 / 开发调试 / 视觉位姿）与页脚 |
| `/rwt/demo.html` | **演示**：预设话题、自动连接、无话题列表 |
| `/rwt/dev.html` | **开发调试**：可改话题、WebSocket 行、话题列表 |

**说明**：VPE 的 **`/ws`** 当前未做代理；若前端改用 WebSocket，需在网关增加转发。

## RSP / TF

与原先一致：完整 3D/TF 需 **`robot_state_publisher` + `/joint_states`**；`demo_driver_services` 不启 RSP 时浏览器侧 TF 可能为空。

## 测试

```bash
source install/setup.bash
colcon test --packages-select aubo_ros2_web_dashboard
```

## 安全与企业级运维

`0.0.0.0` + rosbridge + 代理至 VPE 仅适用于受信网络；生产请加防火墙或反向代理与鉴权。

网关已内置：**请求 ID**（`X-Request-ID`）、**处理耗时**（`X-Process-Time`）、**基础安全响应头**、**统一 JSON 错误体**（`error.code` / `error.request_id`）。可选环境变量：

| 变量 | 作用 |
|------|------|
| `AUBO_WEB_CORS_ORIGINS` | 默认 `*`；生产可设为逗号分隔的允许源 |
| `AUBO_WEB_TRUSTED_HOSTS` | 非空时启用可信 Host 校验（逗号分隔） |
| `AUBO_WEB_PROBE_VPE` | 设为 `1` 时 `/gateway/ready` 会探测 VPE |
| `AUBO_WEB_ACCESS_LOG` | 设为 `1` 时打印 HTTP 访问摘要日志 |
| `AUBO_WEB_LOG_LEVEL` | 传给 uvicorn 的日志级别 |

详见文档 **[9.8 企业级行为说明](docs/ARCHITECTURE_AND_USAGE.md)**。
