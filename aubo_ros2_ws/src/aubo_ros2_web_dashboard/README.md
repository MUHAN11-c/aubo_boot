# aubo_ros2_web_dashboard

`aubo_ros2_web_dashboard` 是一个 ROS 2 `ament_python` 包，用 **FastAPI + Uvicorn** 在单端口下提供：

- 静态页面：`web/public/`
- 轻量 BFF：`/health`、`/api/ivg/runtime-config`
- rosbridge 同源 WebSocket 代理：`/ws/rosbridge`
- web_video_server 同源 HTTP 代理：`/api/ivg/proxy/web-video/*`
- 机械臂网格同源代理：`/api/ivg/robot-mesh/*`

浏览器不直接连接 `9090/8089`，而是统一经页面端口访问本包网关。

## 运行方式

推荐启动方式：

```bash
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
```

它会编排：

- `rosbridge_suite`
- `tf2_web_republisher`
- `web_video_server`
- `python3 -m aubo_ros2_web_dashboard.fastapi_static_gateway`

默认端口：

- 网关：`8090`
- rosbridge 上游：`9090`
- web_video_server 上游：`8089`

## 目录结构

| 路径 | 作用 |
|---|---|
| `aubo_ros2_web_dashboard/gateway/` | FastAPI 应用工厂、路由、CLI |
| `launch/` | ROS 2 launch 编排 |
| `web/public/` | HTML / CSS / JS 静态前端 |
| `web/public/js/view3d/` | 共享 3D 能力模块 |
| `web/public/js/vendor/` | 由 `scripts/bundle_roslib2_browser.sh` 生成的浏览器依赖 |
| `docs/` | 架构图、前端模块说明、vendor 审计 |
| `tests/` | pytest 网关测试 |

## 页面与实现

- `web/public/topics_lab.html`
  - 数据与监控页，入口控制器是 `web/public/js/ros_console.js`。
- `web/public/vision_grasp_panel.html`
  - 视觉抓取页，入口控制器是 `web/public/js/vision_grasp_panel.js`。
- `web/public/js/view3d/`
  - 两个页面共用的 3D 模块，负责补丁、TF、点云、URDF 与会话编排。
- `scripts/bundle_roslib2_browser.sh`
  - vendor 单一生成脚本；ROS 2 / Humble 兼容补丁也在这里维护。

细节统一放在 `docs/`：

- 架构图：[`docs/architecture_diagrams.md`](docs/architecture_diagrams.md)
- 前端模块说明：[`docs/frontend_modules.md`](docs/frontend_modules.md)
- vendor 审计与补丁原因：[`docs/vendor_audit.md`](docs/vendor_audit.md)

## Python 网关

- `gateway/app.py`
  - 创建 FastAPI、注册路由、挂载静态文件。
- `gateway/routes/upstream_proxy.py`
  - rosbridge WebSocket 代理、web_video HTTP 代理。
- `gateway/routes/robot_mesh.py`
  - 机械臂 `package://` 网格同源代理，兼容大小写扩展名。
- `gateway/cli.py`
  - Uvicorn 启动入口。

## 启动与环境变量

常用环境变量：

| 环境变量 | 默认值 | 作用 |
|---|---|---|
| `IVG_ROSBRIDGE_HOST` | `127.0.0.1` | 网关访问 rosbridge 的上游主机 |
| `IVG_ROSBRIDGE_PORT` | `9090` | 网关访问 rosbridge 的上游端口 |
| `IVG_WEB_VIDEO_HOST` | `127.0.0.1` | 网关访问 web_video 的上游主机 |
| `IVG_WEB_VIDEO_PORT` | `8089` | 网关访问 web_video 的上游端口 |
| `IVG_PROXY_WS_MAX_BYTES` | `67108864` | rosbridge 代理单帧上限 |

视频代理额外环境变量见：

- `gateway/routes/upstream_proxy.py`

## 依赖

ROS 2 侧示例：

```bash
sudo apt install ros-humble-rosbridge-suite ros-humble-tf2-web-republisher ros-humble-web-video-server
```

pip 侧依赖已写在 `setup.py` 中，如需手工安装：

```bash
pip install 'fastapi>=0.100' 'uvicorn[standard]>=0.22' 'httpx>=0.25' 'websockets>=12'
```

## 构建

```bash
colcon build --packages-select aubo_ros2_web_dashboard --symlink-install
source install/setup.bash
```

## 测试

当前测试目录统一为 `tests/`。

```bash
cd src/aubo_ros2_web_dashboard
python3 -m pytest tests/ -q
```

或在工作空间内：

```bash
colcon test --packages-select aubo_ros2_web_dashboard
```

## 相关文档

- 架构图：[`docs/architecture_diagrams.md`](docs/architecture_diagrams.md)
- 前端模块说明：[`docs/frontend_modules.md`](docs/frontend_modules.md)
- vendor 审计：[`docs/vendor_audit.md`](docs/vendor_audit.md)

*Apache-2.0*
