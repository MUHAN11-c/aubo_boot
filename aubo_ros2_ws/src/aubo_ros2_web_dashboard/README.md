# aubo_ros2_web_dashboard

ROS 2 **ament_python** 包：托管 IVG 现场 Web 门户与 **ROS 控制台**（话题 / 服务 / 参数 / 图 / 2D / 3D），可选 **FastAPI 网关**（JWT + WebSocket 代理至 rosbridge）。浏览器通过 **roslib** 与 ROS 通信；经网关时先连网关再转发到 rosbridge。

---

## 目录总览

| 路径 | 作用 |
|------|------|
| `package.xml` | ROS 包清单：依赖 `rosbridge_server`、`tf2_web_republisher` 等；说明网关需额外 `pip install -r gateway/requirements.txt`。 |
| `setup.py` / `setup.cfg` | 安装 `aubo_ros2_web_dashboard` 占位模块、`ivg_gateway`（源码在 `gateway/ivg_gateway`）、`launch/`、`web/` 到 `share/`。 |
| `resource/aubo_ros2_web_dashboard` | ament 索引用空标记文件。 |
| `aubo_ros2_web_dashboard/__init__.py` | Python 包命名空间占位。 |
| `launch/` | `ros2 launch` 描述文件，见下表。 |
| `gateway/` | FastAPI 网关（独立 pip 依赖），见下表。 |
| `web/ros2_web_bridge_demo/` | 静态前端（由 launch 中 `http.server` 提供），见下表。 |

---

## `launch/`

| 文件 | 作用与实现要点 |
|------|----------------|
| `web_dashboard.launch.py` | 启动 **rosbridge**（含 rosapi）、**tf2_web_republisher**、`python3 -m http.server` 托管 `web/ros2_web_bridge_demo`。参数：`web_host`、`web_port`（默认 8090）、`rosbridge_port`（默认 9090）。浏览器 **直连** `ws://主机:rosbridge_port`。 |
| `web_dashboard_gateway.launch.py` | 在上者基础上增加 **uvicorn** 运行 `ivg_gateway.main:app`，并 `SetEnvironmentVariable('IVG_GATEWAY_ROSBRIDGE_PORT', rosbridge_port)`，使网关与 launch 中 rosbridge 端口一致。参数额外：`gateway_host`、`gateway_port`（默认 8765）。 |

---

## `gateway/`（FastAPI 网关）

安装：`pip install -r gateway/requirements.txt`（建议在与 ROS 同一 Python 环境或已 `source install/setup.bash` 的机器上）。

| 文件 | 作用与实现要点 |
|------|----------------|
| `requirements.txt` | 网关 Python 依赖（FastAPI、uvicorn、SQLAlchemy、aiosqlite、JWT、slowapi、websockets、roslibpy 等）。 |
| `.gitignore` | 忽略本地 SQLite `ivg_gateway.db`、`.env` 等。 |
| `ivg_gateway/main.py` | **应用入口**：HTTP 路由（`/health`、`/auth/token`、`/auth/me`、`/api/v1/*`）、**WebSocket `/ws/ros`** 使用 `websockets` 库连接上游 rosbridge 并双向转发；出站消息经限流、`ws_policy`、可选空闲急停；生命周期内 `init_db`、种子管理员。 |
| `ivg_gateway/config.py` | **Pydantic Settings**：环境变量前缀 `IVG_GATEWAY_*`（密钥、端口、DB URL、限流、禁区服务子串、急停话题等）。 |
| `ivg_gateway/database.py` | **异步 SQLAlchemy** 引擎与 `SessionLocal`；`init_db()` 建表。 |
| `ivg_gateway/models.py` | ORM：`User`、`AuditLog`、`Task`、`TrajectoryRecord` 及角色/状态枚举。 |
| `ivg_gateway/schemas.py` | Pydantic 模型：Token、用户与任务/轨迹的 API 出入参、视觉占位请求/响应。 |
| `ivg_gateway/auth_core.py` | bcrypt 密码哈希；JWT 签发/解码（载荷含 `sub`、`role`）。 |
| `ivg_gateway/deps.py` | FastAPI `Depends`：`get_db`、`get_current_user`（Bearer）、`require_operator` / `require_admin`。 |
| `ivg_gateway/services/audit.py` | 审计行写入 `audit_logs` 并 `commit`。 |
| `ivg_gateway/services/ws_policy.py` | 解析 rosbridge JSON `op`；viewer 禁写；服务名禁区；**ClientMessageRateLimiter**；**PublishDebouncer**。 |
| `ivg_gateway/services/safety.py` | **同步** `emergency_stop_sync`：可选 `roslibpy` 再连 rosbridge 发布零速度 Twist（由 `asyncio.to_thread` 调用）。 |
| `ivg_gateway/services/__init__.py` | 子模块说明占位。 |
| `ivg_gateway/__init__.py` | 包版本与说明。 |

---

## `web/ros2_web_bridge_demo/`（静态前端）

由 launch 将本目录作为 `http.server` 根目录下的子路径安装到 `share/.../web/ros2_web_bridge_demo/`（以实际 `colcon` 安装为准）。**业务脚本**（`js/vision_grasp_panel.js`、`js/ros_console.js`、`js/coffee_latte_panel.js`、`js/ivg_site_nav.js` 及门户内联脚本）为 **ES2015+**（`let`/`const`、箭头函数、模板字符串等），由浏览器直接执行，**无 Babel/打包**；需现代 Chromium / Firefox / Edge。

| 文件 | 作用与实现要点 |
|------|----------------|
| `index.html` | **灵视IVG 门户首页**：功能介绍与快速入口；链到视觉面板、咖啡拉花、控制台，以及视觉位姿 / 手眼标定外链（脚本按当前页 `hostname` 与固定端口写死 URL，并保留查询串 `q`）。页面对参观者弱化技术细节，部署与端口说明见本 README。 |
| `vision_grasp_panel.html` | **视觉抓取面板**（精简 UI）。话题、服务、抓取策略与实现细节见下文 **「视觉抓取面板」**；脚本头部注释见 `js/vision_grasp_panel.js`。 |
| `topics_lab.html` | **ROS 控制台页面结构**：顶栏（直连/网关登录）、链到视觉抓取面板、IVG 快捷条、多标签主区（话题、服务、动作、参数、关系图、2D、3D）；按序加载 vendor 与 `ros_console.js`。 |
| `js/ros_console.js` | **控制台核心逻辑**：`ROSLIB.Ros` 连接（直连或 `ws://网关/ws/ros?token=`）、侧栏 rosapi 列表、话题订阅与可视化、服务调用、节点图 Canvas、ROS2D / Three.js 2D·3D；IVG 预设话题与服务。 |
| `js/vendor/*` | 第三方库：`roslib`、`eventemitter2`、`easeljs`、`ros2d`、`three`、`OrbitControls`；`fetch_vendor.sh` / `README.txt` 说明获取方式。 |
| `coffee_latte_panel.html` | **咖啡拉花面板**（占位 UI）：布局对齐视觉抓取页；当前仅 Canvas 演示动画，无 rosbridge；样式 `css/coffee_latte_panel.css`，脚本 `js/coffee_latte_panel.js`。 |
| `css/ivg_site_nav.css` | **全站主导航**样式：门户 / 视觉抓取 / 咖啡拉花 / ROS 控制台；由各 HTML 页面共用。 |
| `js/ivg_site_nav.js` | 导航链接附加当前页 `location.search`（继承网关等参数），并按 `data-ivg-page` 高亮当前页。 |
| `css/home.css` | `index.html` 门户样式（与控制台共用相近色板变量）。 |
| `css/topics_lab.css` | `topics_lab.html` 布局与组件样式（顶栏、侧栏、图、2D/3D 容器等）。 |
| `css/demo.css` / `component.css` | 上游 RobotWebTools / demo 遗留样式（其它旧页面若引用）。 |
| `README.md` | 上游 **ros2-web-bridge** 会议 demo 原始说明（硬件与 Node 桥接流程）；与本仓库 IVG 集成以本文件与 `launch` 为准。 |

---

## 视觉抓取面板（`vision_grasp_panel.html`）

面向 IVG 现场：在浏览器中查看相机图、机器人 `RobotStatus`（末端位置、四元数、RPY、关节）、VPE 字符串状态、GraspNet 发布的 `PoseArray` 摘要，以及 6 关节趋势曲线；并通过 rosbridge 调用 `demo_driver` / `graspnet_ros2` 相关服务。

### 连接与网关

- 默认 **直连** `ws://当前页主机:9090`（可用 `?rosbridge_port=` 覆盖）。
- 当前页面 **隐藏网关控件**（`body.vision-grasp-no-gateway` + `css/vision_grasp_panel.css`），脚本中 `gatewayDisabled()` 强制不走网关，避免 URL 中带 `ros_mode=gateway` 时误连。恢复网关：去掉上述 class/CSS/JS 判断，逻辑与 `topics_lab.html` / `ros_console.js` 一致（JWT 存 `sessionStorage`）。

### 话题设置

- 主界面 **「话题设置」** 打开模态框，编辑五个订阅话题。
- **保存并重连**：写入浏览器 `localStorage` 键 `ivg_vision_grasp_topics_v1` 并立即重连；下次进入页面自动恢复。
- **恢复默认**：填回内置默认话题名并清除上述存储（仍须点「保存并重连」才会按默认值重连，或直接点顶栏「重连」若输入框已是默认）。
- **Esc** 或点击遮罩、关闭按钮可关闭模态（未点保存则不上盘存储，输入框内容保留）。

### 订阅话题（默认见下表；与 `vision_grasp_panel.js` 中 `TOPIC_DEFAULTS` 一致）

| 用途 | 默认话题 | 消息类型 |
|------|-----------|----------|
| 彩色图 | `/camera/color/image_raw` | `sensor_msgs/msg/Image`（rgb8/rgba8/bgr8/mono8 等） |
| 末端与关节（文本 + 曲线） | `/aubo_driver/robot_status` | `demo_interface/msg/RobotStatus` |
| 关节曲线（与上并行） | `/joint_states` | `sensor_msgs/msg/JointState`（取 `position` 前 6 维） |
| VPE 状态 | `/system_status` | `std_msgs/msg/String` |
| 抓取位姿列表 | `/grasp_poses_base` | `geometry_msgs/msg/PoseArray` |

**关节曲线说明**：`RobotStatus` 与 `JointState` 两路若同时高频发布，会交替写入同一条历史缓冲，曲线仅表示趋势，不保证关节顺序与机械臂轴一一对应（未解析 `JointState.name`）。

### 服务调用

| 服务 | 类型 | 说明 |
|------|------|------|
| `/execute_single_grasp` | `demo_interface/srv/ExecuteGraspPose` | `object_id` + `use_visual_estimation` |
| `/loop_grasp_control` | `std_srvs/srv/SetBool` | `demo_driver` 后端循环线程（内部固定先视觉估计再抓取，**不能**通过此服务切换非视觉） |
| `/graspnet_capture_control` | `std_srvs/srv/SetBool` | GraspNet 采集开关 |
| `/publish_grasps_worker_loop_control` | `std_srvs/srv/SetBool` | GraspNet worker 循环 |
| `/run_gripper_swap` | `demo_interface/srv/RunGripperSwap` | 快换；面板按钮使用 `direction=gripper2` |

### 抓取策略（页面单选：工件 / GraspNet）

- **工件（视觉）**：单次与循环均使用 `use_visual_estimation=true`（单次直接调服务；循环为 `loop_grasp_control=true`，与后端实现一致）。
- **GraspNet（非视觉）**：单次调用 `use_visual_estimation=false`。循环时因后端 `loop_grasp_control` 线程写死视觉路径，面板会先 `loop_grasp_control=false`，再在**浏览器内**按固定间隔重复调用 `/execute_single_grasp(false)`（间隔见 `vision_grasp_panel.js` 中 `clientGraspLoop.pauseMs`）。
- **停止**：停止浏览器循环并 `loop_grasp_control=false`；WebSocket 断开时也会停止浏览器循环。
- **「单次抓取（非视觉）」**（GraspNet 区块）：始终 `use_visual_estimation=false`，不受单选影响，便于点云流程下明确触发非视觉单次。

### 相关文件

- `web/ros2_web_bridge_demo/css/vision_grasp_panel.css`
- `web/ros2_web_bridge_demo/js/vision_grasp_panel.js`（文件头为开发与运维用的完整约定表）

---

## 咖啡拉花面板（`coffee_latte_panel.html`）

- **当前**：仅前端占位；`css/coffee_latte_panel.css` 通过 `@import` 复用视觉抓取页布局与按钮样式；`js/coffee_latte_panel.js` 在画布上绘制杯口示意与奶泡心形路径动画，并写简要活动日志。
- **后续**：可参照 `vision_grasp_panel.js` 接入 rosbridge、相机话题、关节曲线与自定义服务；全站导航中 `data-ivg-page="latte"` / `data-ivg-nav="latte"` 已与 `ivg_site_nav.js` 对齐。

---

## 数据流简图

```
【直连】浏览器 --WebSocket--> rosbridge --> ROS 2
【网关】浏览器 --WebSocket+JWT--> ivg_gateway --WebSocket--> rosbridge --> ROS 2
```

---

## 相关命令备忘

```bash
# 编译安装包（在工作空间根）
colcon build --packages-select aubo_ros2_web_dashboard --symlink-install
source install/setup.bash

# 仅静态 + rosbridge
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py

# 含网关（需已 pip install -r .../gateway/requirements.txt）
ros2 launch aubo_ros2_web_dashboard web_dashboard_gateway.launch.py
```

默认管理员（仅首次空库）：用户名 `admin`，密码 `changeme`；生产环境请修改 `IVG_GATEWAY_*` 配置。

---

## 与 `.vscode/` 说明

编辑器本地配置（如 `c_cpp_properties.json`），不参与 ROS 运行时；可随个人环境调整，一般无需随包分发说明。
