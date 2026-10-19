# Launch 与配置 API

## Launch 入口

| launch | 类型 | 用途 | 依赖 |
|---|---|---|---|
| `monitor.launch.py` | core | 只起桥+网关，附着运行中的图 | `ivg_foxglove_bridge` |
| `transport.launch.xml` | core | 纯传输 + 可选 comm fixture | `ivg_foxglove_bridge` |
| `official_demo.launch.py` | optional | 官方 RSP/panda + map_server/lifecycle | nav2/moveit 资源 |
| `sim_demo.launch.py` | optional | 官方 TB3 仿真 + nav2 + ros2/examples | turtlebot3 等 |
| `panda_moveit_web.launch.py` | optional | MoveIt Panda + 传输栈 | moveit_resources_panda |

### `monitor.launch.py`

```bash
ros2 launch ros_web_link monitor.launch.py
ros2 launch ros_web_link monitor.launch.py start_bridge:=false bridge_port:=8765
ros2 launch ros_web_link monitor.launch.py python:=/path/to/venv/bin/python
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `bridge_port` | `18766` | 桥端口；8765 常被 preview_server 占用 |
| `gateway_port` | `8095` | 网关 HTTP/WS 端口 |
| `bind` | `0.0.0.0` | 网关绑定地址 |
| `bridge_host` | `127.0.0.1` | 上游桥地址 |
| `start_bridge` | `true` | `false` = 复用外部 Foxglove v1 兼容桥 |
| `python` | 自动探测工作区 `aubo_py3.12/bin/python` | 网关解释器；`RWL_PYTHON` 可覆盖 |

`start_bridge:=false` 是本包独立性的关键：网关/JS/console 只要求上游是 Foxglove v1
兼容桥，不绑定 `ivg_foxglove_bridge` 实现。

### `transport.launch.xml`

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `8765` | 桥端口 |
| `gateway_port` | `8095` | 网关端口 |
| `fixture` | `false` | `true` 起 `comm_fixture`（`/rwl_demo/*`） |

`transport` 使用 PATH 中的 `python3`；请确保它指向已安装 fastapi/uvicorn/websockets
的解释器（工作区 venv 推荐）。

### optional demo

```bash
ros2 launch ros_web_link sim_demo.launch.py transport:=false   # 只起完整官方仿真
ros2 launch ros_web_link sim_demo.launch.py gui:=false         # 仿真 + 传输
ros2 launch ros_web_link official_demo.launch.py
ros2 launch ros_web_link panda_moveit_web.launch.py
```

demo 缺依赖时请先构建对应官方仓库；它们不属于 core 运行前提。

## 端口与进程

| 进程 | 默认 | 说明 |
|---|---|---|
| 网关 | `0.0.0.0:8095` | 8090 归 peach_observability，勿占 |
| 桥 | `127.0.0.1:8765`（monitor 用 18766） | Foxglove WS v1 |
| web_video_server | `127.0.0.1:8089` | optional；视频代理上游 |
| fixture | — | `rwl_comm_fixture`，`/rwl_demo/*` |

## Config 键

`config/defaults.yaml` 是单源；`config.init()`/CLI 覆盖（无环境变量层）。
启动前端口占用会预检。

| YAML | 默认 | CLI | 说明 |
|---|---|---|---|
| `gateway.port` | 8095 | `port` | 网关端口 |
| `gateway.bind` | 0.0.0.0 | `--bind` | 监听地址 |
| `gateway.gzip_min_size` | 512 | — | GZip 阈值 |
| `gateway.ws_ping_interval` | 20 | `--ws-ping-interval` | 客户端腿心跳；`null`/`-1` 关 |
| `gateway.ws_ping_timeout` | 20 | `--ws-ping-timeout` | 心跳超时 |
| `gateway.ws_per_message_deflate` | false | `--ws-permessage-deflate` | 客户端腿压缩 |
| `foxglove_bridge.host` | 127.0.0.1 | `--foxglove-bridge-host` | 上游桥 |
| `foxglove_bridge.port` | 8765 | `--foxglove-bridge-port` | 上游端口 |
| `foxglove_bridge.ws_path` | `/` | `--foxglove-bridge-ws-path` | 上游路径 |
| `foxglove_bridge.compression` | false | `--foxglove-bridge-compression` | 上游 deflate |
| `foxglove_bridge.max_message_bytes` | 134217728 | `--foxglove-bridge-max-msg-bytes` | 128MiB |
| `foxglove_bridge.ping_interval` | 20 | `--foxglove-bridge-ping-interval` | 上游心跳 |
| `foxglove_bridge.ping_timeout` | 60 | `--foxglove-bridge-ping-timeout` | 上游超时 |
| `foxglove_bridge.close_timeout` | 10 | `--foxglove-bridge-close-timeout` | 上游关闭 |
| `web_video.host` | 127.0.0.1 | — | 视频上游 |
| `web_video.port` | 8089 | — | 视频上游 |
| `proxy.upstream_connect_attempts` | 3 | — | 上游重试 |
| `proxy.upstream_connect_retry_ms` | 250 | — | 重试基础间隔 |
| `proxy.upstream_max_queue` | 4 | `--upstream-max-queue` | 上游接收队列 |
| `proxy.data_lane_bytes` | 4194304 | `--data-lane-bytes` | 出站数据预算；0=无界 |
| `proxy.egress_max_age_s` | 2.0 | — | 陈旧帧年龄门 |
| `proxy.video_*` | 10/30/10/65536 | — | 视频连接/读/池/块 |
未列出 CLI 的键只能改 YAML；以 `config/defaults.yaml` 为准。

## 安全

- 网关无鉴权；生产改 `bind:=127.0.0.1` 或反向代理/TLS。
- 交互锁不是安全急停；硬件急停不经 ROS。
- 详细端点见 [http-ws.md](http-ws.md)。
