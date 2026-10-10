# ivg_foxglove_bridge

Foxglove WebSocket v1 协议 ROS 2 服务端（vendored，全源码本地化）。作为
`aubo_ros2_web_dashboard` 的唯一 ROS 传输面，替代 rosbridge（用户裁定 2026-10-10）。

**来源与改动记录见 [VENDOR_PROVENANCE.md](VENDOR_PROVENANCE.md)。**

## 目的

- 浏览器前端经网关 `/ws/foxglove`（8095）→ 本桥（8765）与 ROS 图交互：
  话题订阅（CDR 二进制直传）、客户端发布、服务调用（含自定义 srv）
- 全部源码在仓：服务器库（ws-protocol 最终独立版，MIT）+ websocketpp 0.8.2（BSD）
  + 0.8.5 ros2 glue；无预编译二进制、无构建期 FetchContent 联网
- 不依赖 rosbridge / rosapi / tf2_web_republisher

## 与上游 foxglove_bridge 的差异

| 项 | ros-jazzy-foxglove-bridge (3.x) | 本包 |
|----|----------------------------------|------|
| 服务器核心 | foxglove-sdk 预编译 Rust 核心 zip | ws-protocol 最终独立 C++ 库（源码） |
| websocket | SDK 内置 | vendored websocketpp 0.8.2 + 系统 Boost |
| 构建联网 | FetchContent 下载 SDK zip | 零联网（系统包 nlohmann-json3-dev / libboost-dev / zlib1g-dev） |
| 版本 | 3.6.0 | 0.8.5-ivg1（glue）+ f3b4135（服务器库） |

## 如何 build / 测

```bash
colcon build --packages-select ivg_foxglove_bridge
colcon test --packages-select ivg_foxglove_bridge && colcon test-result --verbose
```

## 例子

```bash
# 默认 8765；参数文件 config/ivg_foxglove_bridge_params.yaml
ros2 launch ivg_foxglove_bridge ivg_foxglove_bridge.launch.xml
# 验证（另一终端）：
python3 src/ivg/aubo_ros2_web_dashboard/test/test_foxglove_integration.py --direct
```

## 安全边界

- 服务端无鉴权（协议 v1 无鉴权），仅限本机/局域网部署；网关 0.0.0.0:8095 是唯一对外面
- 本桥不是急停通道；运动/IO 安全语义见 aubo_ros2_web_dashboard README
