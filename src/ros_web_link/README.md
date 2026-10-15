# ros_web_link — Web↔ROS 2 通信层

Web 与 ROS 2 之间的**传输层**独立包（对标 [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
分包定位，协议采用 [Foxglove WebSocket v1](https://github.com/foxglove/ws-protocol)——与官方
[rviz_web](https://github.com/ros2/rviz_web)（ROS 2 官方 Web 可视化）同一条传输路线）：

- **Python 网关**（uvicorn/FastAPI）：WebSocket 反向代理 `/ws/foxglove` → `ivg_foxglove_bridge`（CDR 二进制）、
  MJPEG 视频流代理、机器人 mesh 服务、健康检查；
- **JS 客户端库（唯一源）**（零构建 ES Modules）：`transport/foxglove/*` 协议/会话/编解码层 + `ros_client`
  门面 + `action_client`，以及 `core/ view3d/ vendor/ robotwebtools/` 共享层；
- **传输诊断台** `console.html`（**白色主题默认**，暗色可切）：RViz 风格 Displays/工具栏/RobotModel、
  rqt_plot 字段绘图、传输监控、全链路自检；
- **comm/viz fixture** 演示节点 + **E2E 测试塔**（通信矩阵/可视化消息/CDR 黄金字节/**MoveIt 2 官方
  Panda 示例端到端**）。

设计来源：分层对齐 [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
（protocol library / transport server 分离），客户端 API 语义对齐 roslibjs
（Topic/Service/Param/Action），协议与能力面沿 [foxglove ws-protocol](https://github.com/foxglove/ws-protocol)
（已归档并入 [foxglove-sdk](https://github.com/foxglove/foxglove-sdk)；官方 SDK 归档
停维护，故保持零构建 ESM 自研薄层 + JS/Python 双实现交叉验证，不新增依赖）。
详见 [doc/architecture.md](doc/architecture.md)。

## ROS 生态通信覆盖（与 rosbridge op 对照）

| ROS 生态原语 | rosbridge 方言 | 本包实现（foxglove v1） | 测试 |
|---|---|---|---|
| Topic 订阅 | `subscribe` | `subscribe` op（channel schema 下发 → CDR 解码） | `test_topic_subscribe_chatter` |
| Topic 发布 | `publish` | `advertise`+二进制 `clientPublish`（QoS 跟随图上发布者） | `test_topic_publish_roundtrip` |
| Service 调用 | `call_service` | `advertiseServices` + `callService`（CDR 双向） | `test_service_call_add_two_ints` |
| Action | `actionlib`/roslibjs 组合 | `action_client.js`：send_goal/cancel/get_result 服务 + status/feedback 话题编排 | `test_action_result_and_feedback` / `test_action_cancel` |
| 参数 | `get_param`/`set_param` | 标准参数服务 `~/get_parameters`/`~/set_parameters` 直调 | `test_params_get_set` |
| TF | `/tf` 订阅 | 同（TRANSIENT_LOCAL 语义保留） | `test_tf_stream` |
| 图谱 | `get_topics` 等 | `connectionGraph` capability 增量流 | `test_connection_graph` |

MoveIt2 / Nav2 的 Web 交互面（`FollowJointTrajectory`、`NavigateToPose`、motion plan 服务等）
全部由上表原语承载——与 [MoveIt Web](https://github.com/ros-planning/moveit_web)、
[nav2 web 工具](https://github.com/ros-navigation/navigation2) 走 roslibjs 的方式同构，
无需第二套传输。

## 快速开始

```bash
# 构建（工作区根）
colcon build --packages-select ros_web_link
source install/setup.bash

# 纯传输模式（等价 rosbridge 单独起）：网关 :8095 + 桥 :8765 + 演示被测端
ros2 launch ros_web_link transport.launch.xml fixture:=true

# 浏览器打开诊断台（自动连接），点「运行自检」
xdg-open http://127.0.0.1:8095/console.html
```

UI 组合模式（面板）见 `aubo_ros2_web_dashboard`；组合网关下本包诊断台在 `/rwl/console.html`。

> 网关 Python 依赖（fastapi/uvicorn/websockets/httpx）钉在仓库根 `requirements.txt`，
> 装进 `aubo_py3.12` venv；launch 中的 `python3` 须指向该 venv。
> WS 传输默认值（2026-10-14 起）：心跳 20s 开、permessage-deflate 关、上游连接
> 有限重试+抖动——全部可在 `config/defaults.yaml` / CLI 覆盖（见 `doc/io.md` 配置表）。

## 公有 API

Python（`from ros_web_link import ...`）：

- `app.create_app(js_roots=None, html_root=None, sub_routers=None)` — 可组合 FastAPI 应用工厂
- `cli.main()` / `ros_web_link_gateway` 入口（端口/绑定/上游桥参数）
- `config.*` — 配置访问器（`config/defaults.yaml` 单源 + CLI 覆盖）
- `comm_fixture.main()` / `rwl_comm_fixture` — 通信矩阵演示节点
- `foxglove_ws_client.FoxgloveWsClient` — Python 侧测试客户端（测试/诊断用，对齐
  rosbridge 生态的 roslibpy 定位；CDR 走 rclpy 官方序列化）

JS（`/js/...`，ES Modules 直引，无构建步骤）：

- `transport/foxglove/ros_client.js` → `foxgloveRosClient()` 单例：
  `connect/subscribe/publish/callService/getParams/setParam/getGraph`
- `transport/foxglove/action_client.js` → `createActionClient(name)`：
  `sendGoal(goal, {onFeedback})/cancelGoal/cancelAll/dispose`
- `transport/foxglove/{protocol,player,codec}.js` — L0/L1/L2 分层（可单独复用）
- `console/app.js` — 诊断台交互层（`window.__rwlSelfTestRun` 供自动化）

## 如何测试

```bash
# 单元测试（aubo_py3.12 venv 下；系统 python 会因 fastapi/starlette 版本组合 skip）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k "not comm_matrix and not visualization" -q

# E2E 通信矩阵（自动起桥+网关+fixture，随机域+随机端口，测完即清）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k comm_matrix -v

# E2E 官方仿真真实数据（turtlebot3 官方仿真 + 官方 nav2 + ros2/examples 官方示例，
# 16 用例：scan/odom/相机/点云/AMCL 粒子云/web 规划闭环；需已克隆并构建官方仓库）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k sim -v

# E2E MoveIt 2 官方 Panda 示例（latched URDF/IK/OMPL 规划/执行轨迹/PlanningScene）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k moveit -v

# JS 纯核单测（66 用例：协议帧/TF 数学/工具函数 + CDR 黄金字节交叉验证——
# rclpy 序列化真值 vs JS codec 离线解码，覆盖 16 类常用可视化消息）
cd src/ros_web_link && node_modules/.bin/vitest run

# 重新生成 CDR 黄金 fixtures（升级 ROS 后）
./aubo_py3.12/bin/python src/ros_web_link/test/generate_viz_golden.py

# 组合网关验证（dashboard UI + 逐文件合并的传输层）
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
#   面板 → http://127.0.0.1:8095/ ；传输诊断台 → /rwl/console.html
```

覆盖：`test/test_gateway.py`（路由/挂载/健康回归）、`test/test_js_units.py`（vitest 包装）、
`test/test_e2e_comm_matrix.py`（上表全矩阵 + 网关 HTTP 面 + 视频代理负例 + 停机端口复核）。

## 文档

- 架构与决策：[doc/architecture.md](doc/architecture.md)
- 输入输出与端点：[doc/io.md](doc/io.md)

## 许可

Apache-2.0
