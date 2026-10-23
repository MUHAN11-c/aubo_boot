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
# 行为：官方仿真 + ros2 launch ros_web_link monitor.launch.py，看浏览器和日志
# 可选：组合网关（dashboard UI；非 core 依赖）
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
#   面板 → http://127.0.0.1:8095/ ；传输诊断台 → /rwl/console.html
```

页面和话题是否通，看 console 自检和网关日志。本包源码目前过不了 ament flake8，所以 `test/` 里没有官方 lint 文件，避免 `colcon test` 被风格问题刷红。

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
