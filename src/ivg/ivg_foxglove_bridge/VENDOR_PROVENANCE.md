# VENDOR_PROVENANCE — ivg_foxglove_bridge

本包是 foxglove WebSocket bridge（ROS 2 服务端）的本地源码 vendor 组合，无任何预编译二进制。
用户裁定（2026-10-10）：替换 aubo_ros2_web_dashboard 中的 rosbridge，服务端全部本地源码可控。

## 1. vendor/foxglove-websocket/ — 服务器库

- **上游**：`foxglove/ws-protocol` 仓库 `cpp/foxglove-websocket`（Foxglove WebSocket v1 协议
  独立 C++ 服务器库，websocketpp 系）
- **版本锚点**：commit `f3b4135ad9aee77d97717af3ee849b48e2ede000`（2025-08-05，仓库归档时点，
  即该库最终状态；上游仓库归档后不再演进）
- **许可证**：MIT（见 vendor/foxglove-websocket/LICENSE）
- **本地改动**：无（源码原样拷贝；仅被本包 CMakeLists 以源码编入）
- **系统能力**：话题订阅（CDR 直传）、客户端发布、服务调用（二进制帧）、参数、fetchAsset、
  连接图；websocketpp + nlohmann-json（Ubuntu 系统包，无 FetchContent、构建期零联网）

## 2. vendor/websocketpp/ — WebSocket 传输层

- **上游**：`zaphoyd/websocketpp` tag `0.8.2`（header-only）
- **许可证**：BSD（见 vendor/websocketpp/COPYING）
- **本地改动**：无；asio 后端用系统 Boost（libboost-dev 1.83）
- **vendor 理由**：系统未装 libwebsocketpp-dev；header-only 钉版拷入免 sudo、版本可溯

## 3. src/ + include/foxglove_bridge/ — ROS 2 glue

- **上游**：ros2-gbp/foxglove_bridge-release `release/humble/foxglove_bridge` 分支
  commit `dd4de28`（upstream 0.8.5，websocketpp 时代最后发布的 ROS 2 glue），
  即上游 `ros2_foxglove_bridge/` 目录源样拷贝（include/foxglove_bridge/ 5 个头 + src/ 6 个 cpp）
- **许可证**：MIT（见 LICENSE_UPSTREAM_GLUE）
- **本地改动**（相对上游 0.8.5）：
  1. `include/foxglove_bridge/{callback_queue,common,parameter,regex_utils,server_factory}.hpp`
     为**本地新增 shim 转发头**——把 0.8.5 时代的 `<foxglove_bridge/xxx.hpp>` include 路径
     转发到最终独立库的 `<foxglove/websocket/xxx.hpp>`，使 glue 源文件保持零改动
  2. `include/foxglove_bridge/message_definition_cache.hpp`：自上游 0.8.5
     `foxglove_bridge_base/include/` 拷入（该头属 ROS 侧，最终独立库不含）
  3. `src/version.cpp`：本地新增（上游 foxglove_bridge.cpp 的替代，版本钉
     `0.8.5-ivg1` / `ws-protocol@f3b4135`）
  4. `src/ros2_foxglove_bridge_node.cpp` 上游原样保留但未编入构建（组件入口经
     rclcpp_components 注册，executable=ivg_foxglove_bridge）
- **API 兼容性核对**（2026-10-10）：0.8.5 base 与最终独立库的
  `WebSocketServerCallbacks` 虚接口 diff 为空；`ServerInterface` 仅新增
  Status/broadcastMessage（纯增量，不影响 glue）

## 4. 参考但未拷贝

- `foxglove/foxglove-sdk` ros/src/foxglove_bridge 3.6.0：SDK 重写版（FetchContent 预编译
  Rust 核心 zip）——因引入预编译二进制不 vendor，仅作 glue 语义参考

## 5. 升级策略

上游均已冻结（ws-protocol 归档 / 0.8.5 后 rosbridge 线终止）。如需服务端扩展
（自定义 op、鉴权层等），直接改本包源码并在本文件记录。

## 6. 本地改动清单（2026-10-10 实测驱动）

1. **剥离 rosx_introspection**：JSON 编码客户端发布改为拒绝（仅 cdr），见
   `ros2_foxglove_bridge.cpp` clientAdvertise/onClientMessage。消除一个 apt 依赖。
2. **clientAdvertise depth≥1 加固**：拷贝端点 QoS 时 history depth 可能为 0
   （RMW 不回填），KEEP_LAST(0) writer 会静默丢写；与 subscribe() 的防护对齐。
3. **保留上游拷贝 QoS 语义**（不强制 SystemDefaults）。
4. **GenericClient 零字段服务请求修复（关键）**：`generic_client.cpp`
   async_send_request——零字段 srv（ListControllers、GetCurrentState、RunLatteWorkflow
   等）的请求 CDR 仅 4 字节封装头，但 rosidl IDL 生成会给空结构塞占位成员
   `structure_needs_at_least_one_member`（uint8），rmw_deserialize 读占位成员越界抛
   Fast CDR 异常。修复：payload ≤ 4B 时跳过反序列化、直接发送 default-init 实例
   （零字段语义下即为正确请求；带字段请求最小 CDR > 4B 不受影响）。
5. **已知限制（Jazzy + FastDDS，2026-10-10 可靠方法实测）**：在已有
   TRANSIENT_LOCAL 发布者的话题上（如 ros2_control JSB 的 /joint_states、/rosout），
   本桥客户端发布的消息对所有订阅者均不投递（拷贝 TL QoS 与 VOLATILE 均同现象；
   同期在无 TL 发布者话题上发布完全正常：全新话题、VOLATILE 发布者话题、
   浏览器↔桥回环 20/20）。疑为 rmw_fastrtps 序列化发布 × TL durability 的交互问题，
   待上游/深入排查；面板发布测试请避开 TL 发布者话题。
6. 协议对齐注记：该服务器线的 subscribe/unsubscribe/advertise/unadvertise 四个 op
   均为**批量格式**（subscriptions[]/subscriptionIds[]/channels[]/channelIds[]），
   与归档 spec 单条形式不同；sessionId 为服务器实例级常量（非连接身份，客户端
   重连时不能据此跳过发布通道重建）。
