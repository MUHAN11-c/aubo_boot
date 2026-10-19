# Launch 示例与参数

## Core：独立监控 / 纯传输

```bash
# 1) 附着到已在运行的 ROS 图（仿真/真机；不拥有被测栈）
ros2 launch ros_web_link monitor.launch.py

# 2) 纯传输 + 通信矩阵 fixture（自包含，无需仿真）
ros2 launch ros_web_link transport.launch.xml fixture:=true

# 3) 复用外部 Foxglove v1 兼容桥（不随本 launch 起桥）
ros2 launch ros_web_link monitor.launch.py start_bridge:=false \
  bridge_host:=127.0.0.1 bridge_port:=8765
```

| 参数（monitor） | 默认 | 说明 |
|---|---|---|
| `bridge_port` | `18766` | 桥端口；8765 常被 preview_server 占用 |
| `gateway_port` | `8095` | 网关 HTTP/WS 端口 |
| `bind` | `0.0.0.0` | 网关绑定地址 |
| `bridge_host` | `127.0.0.1` | 上游桥地址 |
| `start_bridge` | `true` | `false` = 复用外部桥（任意 Foxglove v1 兼容实现） |
| `python` | 工作区 venv 自动探测 | 网关解释器；可用 `RWL_PYTHON` 覆盖 |

| 参数（transport.launch.xml） | 默认 | 说明 |
|---|---|---|
| `port` | `8765` | 桥端口 |
| `gateway_port` | `8095` | 网关端口 |
| `fixture` | `false` | `true` 起 `comm_fixture`（`/rwl_demo/*`） |

## Optional demo（依赖官方 examples / 外部包）

```bash
# 官方 TB3 完整仿真（不带传输栈，供独立 monitor 附着）
ros2 launch ros_web_link sim_demo.launch.py transport:=false

# 官方 TB3 完整仿真 + 传输栈（一键演示）
ros2 launch ros_web_link sim_demo.launch.py gui:=false

# 官方 MoveIt Panda（需要 moveit_resources_panda_*）
ros2 launch ros_web_link panda_moveit_web.launch.py

# 官方 apt 组件（RSP/map_server/lifecycle，无仿真）
ros2 launch ros_web_link official_demo.launch.py
```

这些 demo 不是 core 运行依赖；缺 `turtlebot3_*`/`moveit_resources_panda_*` 时请先构建对应官方仓库，
或只用 `monitor.launch.py` 对接你自己的图。

## 端口与安全

- 网关默认 `0.0.0.0:8095`，**无鉴权**；生产请置于反向代理/TLS 后。
- 桥默认只绑定回环（`127.0.0.1`）由 `ivg_foxglove_bridge` launch 决定。
- UI 的“交互锁”只约束页面写出，不是硬件安全急停；硬件急停不经 ROS。
- 完整参数/端点见 [doc/api/launch.md](../../doc/api/launch.md)。
