# aubo_ros2_web_dashboard

IVG 演示栈 Web 控制台。FastAPI 网关（**零 ROS 依赖纯代理**，端口 **8095**）+ 8 个 MPA 面板
（门户 / 机械臂手动控制 / 视觉抓取 / 咖啡拉花 / 调试 / 监控 / 日志 / 设置）。

**2026-10-10 阶段 3 重构：rosbridge / rosapi / tf2_web_republisher 全部移除。**
唯一 ROS WebSocket 传输面 = vendored **ivg_foxglove_bridge**（CDR 二进制，
全源码本地化，见 `src/ivg/ivg_foxglove_bridge/`）。视频经 web_video_server(8089)。

**不进 peach `harvest_system` / lifecycle；8090 归 `peach_observability`，本包不改不动。**

## 架构（2026-10-10 终态）

```
浏览器 MPA（8 面板）
  ├── js/transport/foxglove/        ← 自研传输栈（协议/会话/codec/门面/ActionClient）
  │   ├── protocol.js               L0 协议客户端（二进制帧编解码）
  │   ├── player.js                 L1 会话层（状态机/订阅三表/重连/服务簿记）
  │   ├── codec.js                  L2 运行时 CDR codec 缓存
  │   ├── ros_client.js             L7 ROS 门面（订阅/发布/服务/参数/图谱）
  │   └── action_client.js          ROS 2 Action 编排（send_goal/get_result/cancel/feedback）
  ├── js/ivg_transport.js           控制面门面（API 兼容层，消费方零改动）
  ├── js/view3d/                    渲染新岛（three 0.164 + urdf-loader + foxglove TF）
  │   ├── island_session.js         URDF + Marker + 点云 + LaserScan 同场会话
  │   ├── tf_buffer.js              TF 树缓存（Lichtblick TransformTree，tf2 真值对齐）
  │   ├── urdf_island.js            URDF 查看器（/robot_description + /joint_states）
  │   └── pointcloud_island.js      点云渲染（BufferGeometry 复用管线）
  └── js/vendor/                    vendor 依赖（three 0.164 / urdf-loader / transforms）
      ↓ /ws/foxglove（WebSocket 子协议 foxglove.websocket.v1）
FastAPI 网关 :8095（子协议归一化 + CDR 透传 + mesh/视频代理 + 静态文件）
      ↓ ws://127.0.0.1:8765
ivg_foxglove_bridge（vendored, MIT）→ ROS 2 图（话题/服务/参数/Action/图谱）
```

## 与 rosbridge 时代的差异（2026-10-10 重构）

| 项 | 旧（rosbridge） | 新（ivg_foxglove_bridge） |
|---|---|---|
| 传输协议 | JSON（大消息 Base64） | CDR 二进制（CPU 友好，吞吐 3-10 倍） |
| 服务调用 | JSON op:call_service | CDR 二进制帧（含空请求服务修复） |
| TF | tf2_web_republisher action | Lichtblick TransformTree 前端自缓冲 |
| URDF | rosapi get_param | /robot_description 话题（latched） |
| 参数 | rosapi get/set_param | 标准参数服务（/node/get_parameters）直调 |
| Action | 不支持 | ActionClient（服务+话题组合，send_goal→get_result） |
| 图谱 | 无 | connectionGraph（topics/nodes/services 实时查询） |
| 依赖 | rosbridge_server + rosapi + tf2_web_republisher | 仅 ivg_foxglove_bridge（本地源码） |
| 端口 | 9090（rosbridge）+ 8765（foxglove） | 仅 8765 |

## 公有 API

- 入口 `ivg_fastapi_static_gateway`（console script）/ `python3 -m
  aubo_ros2_web_dashboard.fastapi_static_gateway`
- HTTP：`/health`、`/api/v1/runtime`、`/api/v1/settings`（POST）、
  `/api/v1/tool-geometries`、`/api/ivg/robot-mesh/{pkg}/{path}`、
  `/api/ivg/proxy/web-video/*`、静态面板 `/`
- WebSocket 代理：`/ws/foxglove` → 127.0.0.1:8765（唯一传输面）
- 配置：`config/defaults.yaml`（端口/上游/前端话题服务默认值，单一来源）

## 例子

```bash
# 一体化 launch：ivg_foxglove_bridge + FastAPI 网关（含 respawn 兜底）
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
# 浏览器打开 http://<host>:8095/
```

web_video_server 不在此 launch 内，需另行手动启动（视频面板依赖）。

## 如何 build / 测

```bash
colcon build --packages-select aubo_ros2_web_dashboard ivg_foxglove_bridge
colcon test --packages-select aubo_ros2_web_dashboard && colcon test-result --verbose
```

浏览器前端纯核单测走 vitest（协议帧/TF 数学/参数助手/工具函数零 ROS 依赖离线跑）：

```bash
cd src/ivg/aubo_ros2_web_dashboard && npm install && npm test
```

Python 依赖钉在仓库根 `requirements.txt`（fastapi==0.101.0 + starlette==0.27.0 +
uvicorn + websockets + httpx），装进 `aubo_py3.12` venv。
JS 测试依赖只装 devDependencies（vitest），`node_modules/` 不入库。

## 安全边界

- 面板可下发真机运动与 IO——**操作员起 real 栈=授权**；勿在无人监督下开 real
- 本面板不是急停；急停回路在柜/示教器
- 与 peach harvest_system 勿同机同域并行（URDF 换装会改 RSP TF 树）

## 许可

Apache-2.0
