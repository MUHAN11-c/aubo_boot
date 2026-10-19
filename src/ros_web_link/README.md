# ros_web_link — 独立 Web↔ROS 2 网关 / 客户端 / 监控包

Web 与 ROS 2 之间的**传输层**独立包（对标 [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
分包定位，协议采用 [Foxglove WebSocket v1](https://github.com/foxglove/ws-protocol)——与官方
[rviz_web](https://github.com/ros2/rviz_web)（ROS 2 官方 Web 可视化）同一条传输路线）：

- **Python 网关**（uvicorn/FastAPI）：WebSocket 反向代理 `/ws/foxglove` → `ivg_foxglove_bridge`（CDR 二进制）、
  MJPEG 视频流代理、机器人 mesh 服务、健康检查；
- **JS 客户端库（唯一源）**（零构建 ES Modules）：`transport/foxglove/*` 协议/会话/编解码层 + `ros_client`
  门面 + `action_client`，以及 `core/ view3d/ vendor/ robotwebtools/` 共享层；
- **独立监控台** `console.html`（**白色二次元商业主题默认**，暗色可切）：RViz 风格
  Displays/工具栏/RobotModel、rqt_plot 字段绘图、rqt_graph 节点图（节点+话题/仅节点、
  服务/动作/调试过滤、悬停邻域、显示项类型色标/复制）、传输监控、全链路自检；点云按 RViz2 Size(Pixels)+Z 彩虹渲染
  （圆点精灵，非方块）；首次连接按图上真实话题自动适配数据面（官方 TB3 / 任意仿真 /
  真机皆可，不绑定某一种话题命名）；
- **comm fixture** 演示节点 + **E2E 测试塔**（通信矩阵/可视化消息/CDR 黄金字节/**MoveIt 2 官方
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
# 构建（工作区根；桥是 sibling core 依赖）
colcon build --packages-select ivg_foxglove_bridge ros_web_link
source install/setup.bash

# ① 独立监控（推荐）：先把完整仿真项目正常跑起来（官方 examples 组成，见 §测试），
#    再在另一终端附着；不拥有/不干预被测仿真
ros2 launch ros_web_link sim_demo.launch.py transport:=false   # 正常起完整官方仿真
ros2 launch ros_web_link monitor.launch.py                     # 独立桥+网关
xdg-open http://127.0.0.1:8095/                                # 自动跳转监控台

# ② 纯传输模式（等价 rosbridge 单独起）：网关 :8095 + 桥 :8765 + 演示被测端
ros2 launch ros_web_link transport.launch.xml fixture:=true
xdg-open http://127.0.0.1:8095/console.html
```

`monitor.launch.py` 参数：`bridge_port`（默认 18766；8765 常被 preview_server 占）/
`gateway_port`/`bind`/`bridge_host`/`start_bridge`（复用已有桥时置 false）/
`python`（默认自动选工作区 `aubo_py3.12` venv，可用 `RWL_PYTHON` 覆盖）。

**独立包边界**：core 只依赖本包 + `ivg_foxglove_bridge` + ROS 基础包 + Python 网关依赖。
UI 组合（`aubo_ros2_web_dashboard`）与官方仿真 demo 都是 optional；`start_bridge:=false`
可复用任意 Foxglove v1 兼容桥。详见 [doc/install.md](doc/install.md)。

> 网关 Python 依赖（fastapi/uvicorn/websockets/httpx）钉在仓库根 `requirements.txt`，
> 装进 `aubo_py3.12` venv；launch 中的 `python3` 须指向该 venv。
> WS 传输默认值（2026-10-14 起）：心跳 20s 开、permessage-deflate 关、上游连接
> 有限重试+抖动。大数据稳定传输默认值（2026-10-16 起）：出站有界数据通道
> 4MiB 字节预算（超限丢最旧二进制帧，控制帧永不丢）+ 出口年龄门 2s +
> 上游 max_queue 4——慢客户端丢帧不积延迟；转发统计在 `GET /api/ivg/ws/stats`。
> 全部可在 `config/defaults.yaml` / CLI 覆盖（见 `doc/io.md` 配置表）。

## API 与示例

完整 API reference：

- [API 总览](doc/api/README.md) / [Python](doc/api/python.md) / [JS](doc/api/js.md)
- [HTTP/WS](doc/api/http-ws.md) / [Launch/配置](doc/api/launch.md)
- [Foxglove/rosbridge 对照与迁移](doc/protocol.md)
- 可直接运行示例：[`examples/README.md`](examples/README.md)

稳定入口：

- Python：`from ros_web_link.app import create_app`、`from ros_web_link.foxglove_ws_client import FoxgloveWsClient`
  （`get_params`/`set_param`、subscribe/publish/call_service/graph）；
  `python -m ros_web_link` 等价 `ros_web_link_gateway`。
- JS：`import { foxgloveRosClient, createActionClient } from '/js/ros_web_link.js'`
  （稳定 barrel；`subscribe/publish/callService/getParams/setParam/getGraph` + action）。
- 浏览器示例：`http://127.0.0.1:8095/examples/minimal_client.html`
- HTTP：`/health`（含 version/protocol）、`/api/ivg/ws/stats`、`/api/ivg/robot-mesh`、
  `/api/ivg/proxy/web-video`。

## 如何测试

```bash
# 单元测试（aubo_py3.12 venv 下；系统 python 会因 fastapi/starlette 版本组合 skip）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test \
  -k "gateway or js_units or api_surface or examples or docs_links or package_independence" -q

# 独立包/API/示例/文档链接门
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test \
  -k "api_surface or examples or docs_links or package_independence" -q

# E2E 通信矩阵（自动起桥+网关+fixture，随机域+随机端口，测完即清）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k comm_matrix -v

# E2E 官方仿真真实数据（turtlebot3 官方仿真 + 官方 nav2 + ros2/examples 官方示例，
# 19 用例：scan/odom/相机/点云/AMCL 粒子云/web 规划闭环 + 真实相机/点云流速率与
# 新鲜度（延迟门）；需已克隆并构建官方仓库）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k sim -v

# E2E 独立监控附着（完整官方仿真正常启动 + 另起桥/网关，8 用例；较慢，默认跳过。
# 只用 src/examples 官方示例；本地 peach 不参与）
RWL_FULL_SIM=1 ./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k full_sim -v
# 等价脚本（自动 source 工作区与 env）：src/ros_web_link/scripts/run_full_sim_e2e.sh

# E2E 大图/点云稳定传输（30Hz 0.92MB 图 + 10Hz 50k 点云经网关：
# 快客户端吞吐/新鲜度 + 停顿-恢复有界积压 + 网关丢帧证据）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k bigdata -v

# E2E MoveIt 2 官方 Panda 示例（latched URDF/IK/OMPL 规划/执行轨迹/PlanningScene）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k moveit -v

# JS 纯核单测（135 用例：协议帧/TF 数学/工具函数 + CDR 黄金字节交叉验证——
# rclpy 序列化真值 vs JS codec 离线解码，覆盖 16 类常用可视化消息；
# 点云共享解析器含 stride 采样；player raw 派发/数据面自动适配）
cd src/ros_web_link && npm install   # 本包自包含 devDependencies（vitest）
node_modules/.bin/vitest run

# 重新生成 CDR 黄金 fixtures（升级 ROS 后）
./aubo_py3.12/bin/python src/ros_web_link/test/generate_viz_golden.py

# 可选：组合网关（dashboard UI + 逐文件合并的传输层；非 core 依赖）
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
#   面板 → http://127.0.0.1:8095/ ；传输诊断台 → /rwl/console.html
```

覆盖：`test/test_gateway.py`（路由/挂载/健康/根路径跳转回归）、`test/test_js_units.py`
（vitest 包装）、`test/test_api_surface.py`（版本/签名/barrel）、`test/test_examples.py`
（示例 E2E）、`test/test_docs_links.py`、`test/test_package_independence.py`、
`test/test_e2e_comm_matrix.py`（上表全矩阵 + 网关 HTTP 面 + 视频代理负例 + 停机端口
复核）、`test/test_e2e_full_sim.py`（完整官方仿真独立监控附着）。

## 文档

- 文档索引：[doc/README.md](doc/README.md)
- 安装/独立运行：[doc/install.md](doc/install.md)
- API：[Python](doc/api/python.md) / [JS](doc/api/js.md) / [HTTP-WS](doc/api/http-ws.md) / [Launch](doc/api/launch.md)
- 协议对照：[doc/protocol.md](doc/protocol.md)
- 架构与决策：[doc/architecture.md](doc/architecture.md)
- 输入输出速查：[doc/io.md](doc/io.md)
- 测试流程与验收：[doc/testing.md](doc/testing.md)

## 许可

Apache-2.0
