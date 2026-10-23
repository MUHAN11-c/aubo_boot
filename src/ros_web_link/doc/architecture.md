# ros_web_link 架构（SNAPSHOT）

> **2026-10-23 迁移说明**：`src/examples` 与 `external/` 已迁到 `/home/mu/demo_ws`。下文 `src/examples/...` 指 `/home/mu/demo_ws/src/examples/...`，`external/...` 指 `/home/mu/demo_ws/external/...`；运行 `sim_demo.launch.py` 前先 `source /home/mu/demo_ws/install/setup.bash`（或确认 apt 包）。

**现在跑什么**的包级活文档；怎么演化以工作区 AGENTS.md 为准。

## 分层

```
浏览器（面板 / console.html 诊断台）
   │  ES Modules: transport/foxglove/*（L0 protocol → L1 player → L2 codec → ros_client 门面）
   ▼
网关 ros_web_link.cli（uvicorn :8095，aubo_py3.12 venv）
   ├─ /ws/foxglove   WS 反向代理（子协议归一 → websocket.v1；CDR 二进制，128MB 上限；
   │                  上游有限重试+抖动退避；两腿 ping 开 / permessage-deflate 默认关）
   ├─ /api/ivg/proxy/web-video/*   MJPEG/快照流代理 → web_video_server :8089（共享 httpx 连接池）
   ├─ /api/ivg/robot-mesh/{pkg}/…  机器人 mesh 文件（ament share 定位，resolve 防穿越）
   ├─ /health        组件探测（桥/视频 TCP 可达性，2s 缓存；桥 down → status=degraded）
   └─ 静态：纯传输模式 web/public → /；组合模式 UI → / + 本包 → /rwl
   ▼
ivg_foxglove_bridge（vendored C++ :8765，Foxglove WebSocket v1 服务端）
   ├─ capabilities: clientPublish / services / connectionGraph（time 随 use_sim_time）
   └─ topic/service 白名单默认全开（生产由 launch 参数收敛）
   ▼
ROS 2 图（rclpy/rclcpp 任意节点；演示被测端 = 本包 comm_fixture）
```

- **单传输面**：foxglove v1 是唯一 WebSocket 传输（2026-10-10 决策，替代 rosbridge；
  与 ros2/rviz_web 官方路线同向）。action = 3 服务 + 2 话题在 `action_client.js` 编排，
  参数 = 标准参数服务直调——能力面对齐 rosbridge+roslibjs，见 README 对照表。
- **CDR 编解码**：浏览器侧自研 codec（vendored @foxglove cdr parse/write），Python 测试侧
  rclpy 官方序列化——两条实现互为交叉验证（E2E 通过 = 字节级一致）。
- **组合式网关**：`create_app()` 三扩展点（js_roots/html_root/sub_routers）；
  `aubo_ros2_web_dashboard` 用它叠 UI 静态与 BFF，不复制传输代码。
- **Python 测试客户端** `foxglove_ws_client.py`：与 JS protocol.js 同帧格式
  （与 vendored 服务器源码逐字节核对过），仅测试/诊断用，不进浏览器链路。

## 独立监控轮（2026-10-19，用户目标：独立包 + 完整官方仿真 + 白色二次元商业主题）

用户裁定：`ros_web_link` 作为**独立功能包**存在；测试时正常启动**完整仿真项目**
（数据面只用 `src/examples` 官方组件，本地 peach 不参与），再让本包独立附着做
监控/调试/可视化；Web 采用**白色二次元商业风格**。落地：

- **独立入口** `launch/monitor.launch.py`：只起桥+网关（`start_bridge:=false` 可复用
  外部桥），不拥有仿真；`ros2 launch ros_web_link monitor.launch.py` 即可附着。
  纯传输模式 `/` 307 → `/console.html`（操作员只记一个地址）。
- **数据面自动适配** `viz_core.adaptInstancesToDataPlane(instances, topics)`：无用户
  布局时按图上真实话题重定向默认 Displays（压缩图优先→原始 RGB 回落；点云按
  `/test_cloud`→腕载深度点云→感知单帧云；可选 `/peach/**` Marker）。用户保存过布局
  后不再改写；单测覆盖官方 TB3 与通用 RGB/深度两种数据面。
- **浏览器 E2E 抓出的真实缺陷**：
  ① `topics.js _updateNumbers` 把 `thead` 的 `<tr>` 当数据行 → 每 tick
  `Cannot set properties of null (setting 'textContent')`；改 `tbody tr`。
  ② HTML 两个 `#param-node` 重复 id，操作台参数读写误用参数面板输入；操作台改
  `#param-node-ops`。
  ③ Teleop `/cmd_vel` 方言硬编码 TwistStamped；增 `teleopMessage()` 纯核 + 自动识别
  （仿真桥 Twist / nav2 TwistStamped）+ 手动选择。
- **UI 主题**：白底 + 樱粉/星紫/天青渐变、圆角卡片、贴纸徽章、内联 chibi 机器人
  mascot；保留暗色切换与所有既有 id/JS 契约。品牌锚点 `LIVE STUDIO` / `自动数据面`。
- **RViz2/rqt 表现轮（2026-10-19 续）**：PointCloud2 改 **Size(Pixels)=3 + 圆点精灵 +
  Z 轴彩虹**（cfg v5；v4 米制尺寸 ×80 迁移），LaserScan 圆点/range 冷暖，场景 HUD/
  坐标图例/固定帧/适配全部/网格/坐标轴；新增 **图谱** 工作区（rqt_graph 风格节点/话题/
  服务，**节点+话题 / 仅节点**、服务/动作/隐藏调试过滤、搜索淡化、悬停邻域、统计、
  `window.__rwl_topology`）；Displays 增加类型色标/悬停复制（RViz Duplicate 语义）。
  参考窗口截图见 `gui-test-screenshots/reference/`（rviz2_tb3 / gazebo_gui / rqt_graph）。
- **E2E 编排**：`Stack(domain=...)` 附着到指定 ROS 域；`LaunchStack(..., domain=...)`
  + `wait_gateway=False` 支持"完整仿真 transport:=false"与监控分起。
  `test_e2e_full_sim.py` 8 用例：正常启动 `sim_demo.launch.py transport:=false`
  （官方 TB3+nav2+ros2/examples）→ 另起桥/网关 → robot_description/joints/TF/scan/
  相机+Info/压缩图/点云/map/topic/图谱/mesh 路由 + HTTP/入口/主题锚点全绿。
  默认 `RWL_FULL_SIM=1` 才跑；本地 peach 不参与。

## 独立包 API/示例收口轮（2026-10-19，对标 rosbridge_suite / Foxglove）

用户要求把本包收口为**可独立安装、可独立调用**的 ROS 包，并提供调用示例/API，
参考 rosbridge_suite 的 library/server/client 分层与 Foxglove 的 bridge/protocol/
client 分层。落地：

- **core vs optional**：core = 网关 + JS 客户端 + console + `monitor.launch.py` +
  `transport.launch.xml` + comm fixture；official/sim/panda demo 为 optional。
  core 只依赖本包 + `ivg_foxglove_bridge` + ROS 基础包 + Python 网关依赖；
  UI 组合包不是运行前提。`start_bridge:=false` 可复用任意 Foxglove v1 兼容桥。
- **稳定入口**：Python 保持 `create_app` / `FoxgloveWsClient`；新增
  `python -m ros_web_link` 与 `get_params/set_param`。JS 新增
  `/js/ros_web_link.js` 与 `transport/foxglove/index.js` barrel，应用不再深路径导入。
- **API 文档**：`doc/api/{README,python,js,http-ws,launch}.md`；新增 `doc/install.md`、
  `doc/README.md` 索引。`/health` additive 增加 `version`/`protocol`。
- **协议对照**：新增 `doc/protocol.md`，按 rosbridge_suite 的
  `ROSBRIDGE_PROTOCOL.md` 体例写包映射/操作映射/roslibjs 迁移；明确本包是 Foxglove
  v1 + CDR，**不是 rosbridge wire 兼容**（无 `/ws/rosbridge`）。
- **示例**：`examples/python/{01..05}.py`、`examples/http`、`examples/launch`、
  `web/public/examples/minimal_client.{html,js}`；示例 E2E 在 comm fixture 上验证
  subscribe/publish/service/params/graph，不依赖仿真/peach。
- **独立安装**：`examples/`、`doc/`、`README.md`、`CHANGELOG.rst` 随包安装到 share；
  demo 脚本去 `/home/mu` 硬编码，版本升 1.1.0（additive）。
- **门禁**：起 launch 后看日志和浏览器。本包没有官方 lint 文件（源码目前过不了 ament flake8）。

## 通信矩阵验证（本包的验收面）

行为核对：起官方仿真或 `monitor.launch.py`，浏览器打开 console，看自检、Displays 和网关日志。手工起栈要用空闲 `ROS_DOMAIN_ID`，结束清进程。桥进程需要 `ROS_DISTRO=jazzy`。

## 决策记录（本包内）

- **官方优先原则（2026-10-15 用户裁定）**：官方生态能提供的数据面一律用官方组件
  （RSP/panda、map_server、CLI、lifecycle），自研 fixture 退守合成传感器流与传输矩阵
  被测端；入口 `official_demo.launch.py`，测试面 `test_e2e_official.py`（7 用例）。
- **viz_fixture 全删、数据面上移官方仿真（2026-10-15 二次裁定）**：合成传感器流
  fixture 整体删除；验证数据面 = GitHub 热门官方仓库（克隆进项目：
  src/examples/turtlebot3_msgs、src/examples/turtlebot3_simulations、src/examples/ros2_examples；
  external/navigation2 基线克隆不参与构建）+ ros2/moveit2/nav2 官方示例；
  本地自写项目（sim/peach/ivg）不参与验证。入口 `sim_demo.launch.py`
  （官方 TB3 gz 仿真 + 官方 nav2 + ros2/examples + 官方 republish + 传输栈），
  测试面 `test_e2e_sim.py`（19 用例，`LaunchStack` 整仓拉起）。
  console Displays 默认 topic 全部指向官方数据面（/scan /odom /plan
  /particle_cloud /amcl_pose /camera/image_raw /test_cloud /map）；
  marker/range/wrench 官方无来源，默认关、topic 留空。
  web 不再有 joint_cmd 写出（viz_fixture 是其唯一消费者）——动臂走 FJT action。
- **测试超时硬上限 30s（2026-10-15 用户约束）**：harness 就绪门 30s、服务/规划/执行
  单次等待 ≤30s、xacro 展开 20s——任何单次等待不得超过。
- **多行 URDF 禁止经 `-p name:=<value>` 直传**：execve 路径下 rcl 参数 YAML 解析
  拒收（RCLInvalidROSArgsError rc=250；bash `$(...)` 路径却可过，传递层差异未深究）。
  官方标准解法 `--params-file`（YAML block scalar，harness `_rsp_params_file`）。
- **RSP 对零时间戳且值恒定的 JointState 去重**（/tf 静默）：官方 `ros2 topic pub`
  不带 header 字段即触发；web 驱动官方 RSP 必须带当前时间戳
  （`test_e2e_official.py` 与浏览器闭环均已按此实现，实测 z=0.38942 全精度）。
- **launch 文件扩展名必须与内容一致（2026-10-15 官方 demo 冒烟修复）**：
  `transport.launch.py` 内容是 XML——ros2 launch 按扩展名走 Python 前端，
  词法器对 XML 注释里的全角字符直接 SyntaxError（lxml/ElementTree 均可正常解析，
  故静态检查发现不了；E2E 走 harness 直编子进程同样未暴露）。改名
  `transport.launch.xml`，组合 launch 改用 `XMLLaunchDescriptionSource` 引用。
- **web 发布基线相位抗桥丢首帧（2026-10-15 全量连跑实证）**：advertise 刚建立时
  首条 clientPublish 可能被桥丢弃——官方闭环用例的 home 位发布未见到 panda_link1
  前周期重发（单发版在整组连跑负载下偶发 /tf 静默误报）。
- **goal 落在 JTC 激活过渡期 = 静默假成功（2026-10-15 冷启动探针实证）**：
  栈冷启动时若 follow_joint_trajectory goal 在 `panda_arm_controller` 激活窗口内
  到达，JTC 日志显示激活后 19ms 即 `Goal reached, success!`——臂零运动、mock 状态
  不变、action 仍 SUCCEEDED（ros2_control 上游行为；JSB 为连续发布 ~60Hz，
  直连 DDS 观测可判别非 web 传输问题）。执行用例先经
  `/controller_manager/list_controllers` 确认 active 再留 1s 稳定期。
- **已知环境问题：sim_demo 组合栈中 TB3 对 /cmd_vel 无物理响应（2026-10-15）**：
  vanilla turtlebot3_gazebo 同桥配置可正常驱动（CLI 实测 2.2m/4s），sim_demo
  组合（nav2 on/off、examples/transport on/off、GUI on/off、gz 分区隔离 on/off）
  均静态——桥订阅在（/cmd_vel TwistStamped RELIABLE）、/odom /scan /joint_states
  正常出流。已排除：gz 分区串扰（默认分区多仿真交叉发布，须设 GZ_PARTITION）、
  消息类型（Twist/TwistStamped 均试）、nav2 零速帧（无）。**根因已定位并修复**：
  spawn_turtlebot3 默认出生位姿 (0,0) 是 turtlebot3_world 中心柱——机器人出生即
  卡死在障碍物内，DiffDrive 指令全部无效；sim_demo 已显式传 (-2.0,-0.5) 修复。
  GUI 网格测试（web-gui-tester 轮）复核通过。
- **GUI 网格测试发现（2026-10-15 web-gui-tester 轮）**：
  ① ~~Fixed Frame 选非根帧（如 base_footprint）时 3D 视口空白~~ **已修（2026-10-16）**：
  `_onTf` 的 TF 轴改按 Fixed Frame 摆位（原按树根，与数据组两套参照系不一致），
  浏览器实测叶帧 fixed 下点云/TF/网格全渲染；
  ② 官方桥高吞吐下事件流刷 `Send buffer limit reached`（桥→WS 背压，与
  BEST_EFFORT 丢帧同源，Foxglove 语义下属预期降级）——**2026-10-16 起网关层
  有界通道+年龄门已缓解**（慢客户端丢帧发生在网关且计数可见，桥缓冲不再
  先灌满；桥侧根因未修，vendored C++ 本轮不动）；
  ③ 机器高负载（演示栈 GUI + pytest 双栈并发）会拉低 gz 实时因子，teleop 物理
  断言窗口已加长到 4.8s 并保持 2 轮重试。
- **foxglove-only，不起 rosbridge**：协议更省带宽（CDR 二进制 + 点云），rviz_web 同路线；
  生态能力（service/action/param）经组合层补齐，不再维护第二条 WS 传输。
- **`/ws/rosbridge` 不存在是刻意的**：`test_gateway.py` 有负例断言防止回归。
- **console.html 挂载点**：纯传输 `/console.html`；组合模式 `/rwl/console.html`（UI 根
  独占 `/`，诊断台仍可从面板域访问）。
- **压缩图像 fixture 用内嵌真 JPEG**（`ros_web_link/_jpeg_frames.py`，PIL 预生成 base64）：
  旧 fixture 发的是"最小合法 JPEG 段"标记字节——传输层可校验但浏览器解码必失败；
  CompressedImage 显示链路要端到端真，就不能造假数据。venv 虽能经 system-site-packages
  看到 PIL，但 fixture 零新增运行时依赖是更稳的边界。
- **odom 圆周 0.15 rad/s 的来历**：web 端轨迹采样阈 0.1m（RVIZ-RQT 参考值），半径 2m
  圆在 0.05 rad/s 时每秒弦长 0.0999m 恰好低于阈值——轨迹显示退化为 5 个点；提速到
  弦长 ~0.3m 后每拍采样。
- **fake/real 两档模式不进诊断台**（Rebot-Arm 有、我们不移植）：本台定位传输/可视化
  调试客户端，真机授权语义归工作区 AGENTS.md；「真机已解锁」红标反而暗示本台可控制真机。
- **console.html 挂载点**：纯传输 `/console.html`；组合模式 `/rwl/console.html`（UI 根
  独占 `/`，诊断台仍可从面板域访问）。
- **comm_fixture 的 action execute 长跑**：必须 `MultiThreadedExecutor`（单线程 executor
  会把 cancel_goal 服务堵到 goal 终结——E2E 曾抓出此 bug）。
- **fixture /tf 用 TRANSIENT_LOCAL**：晚连接的诊断台订阅者也能拿到最后一帧。
- **fixture echo 话题带 1Hz 存活发布者**：浏览器端 foxglove 客户端发布要求图上已有
  该话题的 channel/schema（Lichtblick 同款约束），subscriber-only 话题无 channel。

## 通信与交互优化轮（2026-10-14，对照 rosbridge/roslibjs/foxglove 官方实践）

深读 rosbridge_suite（ros2 分支 v2.1.0 协议）、roslibjs、Foxglove WebSocket spec、
tf2_web_republisher、foxglove_bridge 参数面与 urdf-loaders 后的裁定与修复：

- **WS 传输默认值对齐 foxglove_bridge 官方建议**：客户端腿心跳默认开（20s/20s，
  旧为 None——半开连接只能等 TCP 超时）；两腿 permessage-deflate 默认关
  （CDR 二进制压缩收益差 CPU 代价大；官方 `use_compression=false` 同理）。
  全部留 yaml/CLI 旋钮。
- **上游连接有限重试 + 抖动退避**（`proxy.upstream_connect_attempts=3` ×
  250ms±50%）：桥重启窗口内不再立刻断客户端；全部失败以 close code 1011
  告知（浏览器 protocol.js 把 code 透传进重连诊断），替代旧的无码突然断开。
  与已知问题「重连风暴击垮桥 clientPublish」配套：客户端 player 重连退避
  加 ±20% 抖动错峰。
- **/health 组件探测**：桥/视频 TCP 可达性（1s 超时，2s 缓存）；桥 down →
  `status=degraded`（仍 200 不破坏现有监控），`web_video` 只上报不降级
  （可选辅助组件）。
- **console TF 全精度单源**：viz.js 弃 `console/tftree.js`（只叠平移 + 局部
  四元数 + /tf_static 回调传参 bug 致静态变换从未进树），迁移到
  `view3d/tf_buffer.js`（Lichtblick TransformTree，/tf+/tf_static 全精度链路
  累积）——`lookupLatest(frame, root)` 取世界位姿。tftree.js 已删除。
- **帧感知放置（RViz 语义）**：cloud/scan3d/markers/path/poses 读
  `header.frame_id` 经 TF 解析后摆位；数据组（cloud/scan/markers/path/poses/map）
  共享 `_rosFrame` Z-up 容器，与 TF/URDF 同一 ROS 世界；goal_pose/clicked_point
  发布的坐标映射修正（ROS y = −scene z）。
- **PointCloud2 解析单源**（`view3d/pointcloud_parse.js`）：按 `fields[].offset`
  + DataView 读 float32/float64（旧 viz 实现逐字节读 data → 渲染出的是字节值）；
  rgb packed/intensity（含整型归一）；viz 与渲染岛共用。
- **消息派发 O(1)**：player 维护 subscriptionId 反向 Map（旧为每消息线性扫描
  订阅表）；视频代理共享 httpx 连接池（旧每请求新建）；OccupancyGrid 换图
  释放旧纹理；robot_mesh containment 改 resolve()（symlink 防穿越）+ 修
  大小写回退分支的检查对象错位。
- **Python 测试客户端可靠性**：`_send_json` 改 await（JSON 控制帧与二进制帧
  保序，发送异常不再被吞）；status op 记录+日志；订阅队列满丢最旧（旧丢新帧）；
  `websockets.exceptions` 改显式子模块导入（venv 曾出现混合损坏安装，
  懒属性访问在 import 期即炸；已按 requirements.txt 钉版重装修复）。

## 单源化重构（2026-10-13，参考 foxglove-sdk / rosbridge_suite / roslibjs）

深度阅读 [foxglove-sdk](https://github.com/foxglove/foxglove-sdk)（ws 协议库已并入且归档于
[ws-protocol](https://github.com/foxglove/ws-protocol)）、[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
（protocol library / transport server / api 三层分离 + spec-first）与 roslibjs 客户端 API 后的裁定：

- **不引入官方 SDK 依赖**：ws-protocol 已归档停维护，且 vendored 桥用批量 op 形态；
  零构建 ESM + 自研薄层（protocol/player/codec）总量小于官方 SDK 依赖链，保留双实现
  （JS + Python/rclpy 序列化）交叉验证——这是"精简"的最优解，而非加包。
- **JS 传输层单源**：ros_web_link 是 `transport/ core/ view3d/ vendor/ robotwebtools/
  ivg_{transport,runtime}.js` 的**唯一源**（此前与 aubo_ros2_web_dashboard 存在整套
  物理拷贝，且 dashboard 拷贝滞后——含 ros_client.publish bug）。dashboard 仅保留
  UI 层（面板/组件/latte/vision_grasp/状态栏等），组合网关启动时由 launch_gateway
  **逐文件 symlink 合并**（269 个文件；目录级合并不适用于 core/ 混居）。
- **共享纯核**：tf-math/utils 在本包。2026-10-23 起不再保留 vitest 文件。
- **SymlinkStaticFiles**：colcon `--symlink-install` 下 starlette 0.27 的 realpath
  containment 检查会把指向 build 树的符号链接判越界 → install 树静态全 404。本包
  挂载改用 normpath `..` 段检查的 lookup_path（目录路径也回 stat 以驱动 index.html
  回退）。MultiStaticFiles 同型。

### JS 树归属（重构后）

| 位置 | 内容 |
|---|---|
| ros_web_link `web/public/js/` | transport/（foxglove 协议栈+门面+action）、core/（log/tf/utils/runtime 共享纯核）、view3d/、vendor/（three/urdf/transforms）、robotwebtools/（importmap+builds+libs+roslib-esm）、console/、ivg_{transport,runtime}.js |
| dashboard `web/public/js/` | 面板 html/js、components/、latte/、vision_grasp/、core/{dom_cache,ivg_status_bar,lifecycle,topics}、ivg_site_nav.js |

已删除（两树确证死代码）：core/entities/{connection,ros_data,service}_store、
entities/ui_state_store、components/topic_browser、core/{ros_connector,settings}、
view3d/hints；本包 config 移除未用的 package_version_fallback。

## 大数据稳定传输轮（2026-10-16，深读 ws-protocol / Lichtblick / rosbridge / rviz2）

深读 foxglove ws-protocol spec（messageData 帧格式 + 背压语义属实现自定）、
Lichtblick FoxgloveWebSocketPlayer（parsedMessages 字节记账 + 超限丢最旧至 80%）、
rosbridge websocket_handler（write_queue 满丢出站 + 入站 deque(maxlen) 丢最旧）、
rviz2（ImageDisplay latest-wins 纹理、PointCloud 状态行）与 vendored 桥
（send buffer 超限丢新帧，10MB）后的裁定与实施：

- **网关出站有界数据通道**（`_BoundedForwardLane`）：字节预算 4MiB + 丢最旧
  （滞回 80%），JSON 控制帧永不丢、全局保序。此前慢客户端会在
  网关→浏览器腿无界积压（桥 send buffer 10MB + 内核套接字缓冲逐帧灌满），
  延迟随积压线性增长。三处官方实现同构（rosbridge write_queue /
  Lichtblick parsedMessageCache / 桥 send-buffer），语义对齐。
- **出口年龄门**（`egress_max_age_s` 默认 2.0s）：推送前已陈旧的 messageData
  帧直接丢。慢客户端恢复排积压时不送旧帧；内核套接字缓冲里「已提交」的
  积压服务端不可回收（TCP 物理现实，`ss -tm` 实证 w4.2MB 打满），彻底的
  latest-wins 显示由浏览器 player 的 latest 合并模式兜底（Lichtblick 同款）。
- **上游 `max_queue` 16→4**：慢客户端背压更早传给桥，减少网关内大帧堆积。
- **WS 转发统计**：连接级 forwarded/dropped/droppedStale/queued + 进程累计，
  `GET /api/ivg/ws/stats`（totals 仅连接断开时归账，活跃数在 active[]——
  console 监控卡两者求和）。console 监控区新增「网关丢帧 / 网关积压」卡。
- **player `raw` 派发模式**：同 latest 的微任务合并但零解码（回调拿原始
  CDR 载荷），codec 也不构建。话题卡 Hz/流量统计-only 订阅改用 raw——
  大消息（图像/点云）全量 CDR 解码曾是主线程纯浪费。
- **点云 stride 采样**：超 maxPoints 用 stride 空间均匀采样（旧实现截断尾部，
  organized 云 height>1 时直接丢下半幅画面）。
- **Fixed Frame 统一**（GUI 网格测试遗留问题 ①修复）：`_onTf` 的 TF 轴过去按
  树根摆位而数据组按 Fixed Frame 摆位——Fixed Frame 选非根帧时两套参照系
  不一致致视口"空白"。统一为 fixed（RViz 语义：视口世界即 fixed frame），
  浏览器实测 fixed=rwl_demo_marker（叶帧）点云/TF/网格全渲染。
- **点云/激光显示 state 惰性确保**（浏览器实测抓出的渲染级 bug）：
  `_subscribeDisplay` 订阅回调先以无 init 的 `_stateFor` 创建空 state，
  `_onCloud`/`_onScan3d` 的 init 闭包从此不生效 → `BufferAttribute(undefined)`
  → 每帧回调抛错零渲染。改 handler 内惰性确保字段。
- **无 URDF 栈蒙层兜底**：6s 内图上无 /robot_description 通道即收起
  「加载机器人模型…」蒙层（纯传感器栈 3D 视口曾被永久遮挡）。
- **图像解码零分配**：ImageData 按 (w,h) 复用（原每帧 createImageData）；
  增补 rgba8/bgra8 编码。Displays 点云状态带 stamp 时延（rviz Delay 语义）。
- **comm_fixture 大数据源**（`-p rwl.bigdata:=true`）：`/rwl_demo/big_image`
  640×480 rgb8 @30Hz（~0.92MB/帧）+ `/rwl_demo/big_cloud` 50k 点 @10Hz；
  E2E `test_e2e_bigdata.py`：快客户端吞吐≥65% 名称率、年龄毫秒级、丢帧率<5%；
  停顿 4s 恢复后停顿窗内帧送达数有硬上限（<60，无界旧行为≈120）+
  追上直播的首帧新鲜（<1.5s）+ 网关丢帧计数可见。
- **console 自动化入口**：`window.__rwl_viz`（addOrEnable/内部状态，诊断用，
  同 `__rwl_topics` 模式）。

证据面（2026-10-16 浏览器实测，浏览器全速消费）：/rwl_demo/big_image +
big_cloud 双流并行 25MB/s、RTT 2ms、重连 0、解码失败 0、网关丢帧 0、
积压 0 帧、单连接累计转发 5.37GB。当时的自动用例文件已于 2026-10-23 删除。

## 官方示例 E2E 轮（2026-10-16 二弧）：gz 相机火流发现与压缩流默认

以官方示例项目（TB3+nav2+examples+Panda MoveIt）为数据面的全量 E2E + 浏览器实测轮
（流程与验收口径固化在 [testing.md](testing.md)；新增真实流延迟用例
`test_tb3_{camera,pointcloud}_stream_rate_and_freshness`）：

- **发现：gz Harmonic 无头相机不按 SDF `update_rate` 节流**。waffle_pi 相机 SDF 声明
  30Hz，实测 DDS 侧 ~4.7kHz / 5.2GB/s（物理配速正常：/scan 8.8Hz≈10Hz sim、/clock
  781Hz——仅渲染传感器失控，gz-sim 8.15 上游行为）。该火流下网关传输仍健康：
  快客户端收到 755fps / 695MB/s、帧龄 5ms、积压有界（按设计丢帧）；受控 30Hz 场景
  由 bigdata 合成基准覆盖（丢帧 <5%）。
- **发现：火流订阅会饿死服务响应（浏览器实测根因定位）**。console 默认订阅原始
  相机流时，桥→网关发送缓冲持续打满（`Send buffer limit reached` 刷屏），桥"丢新帧"
  降级不区分帧类型——**服务响应帧被一并丢弃**，浏览器自检服务/动作项 10~25s 超时
  （DDS 直调同服务 42 即回，排除 fixture；关掉相机订阅后自检 9/9、服务 22ms——
  因果实验闭环）。桥侧丢帧语义（vendored C++）本轮不动，记录为已知问题。
- **修复：console 默认相机显示项改走官方 republish 压缩流**（`imagec1` →
  `/camera/image_raw/compressed`，~15fps×20KB；原始流 `image1` 默认关、可手动开启）。
  实测浏览器连接 **0 丢帧 / 0 积压 / 0 陈旧丢弃**（此前 70,613 帧丢弃）、自检 9/9、
  压缩图 640×480 真实画面渲染、激光/点云/代价地图全渲染。单源真身在
  `viz_core.js defaultInstances()`（viz.js 里的 dict 副本是漂移死代码，已删）。
  （更正：本段初版曾记"机器人 mesh 全渲染"——当时 mesh 实际全部 404、视觉
  分析把激光点/TF 轴簇误读为机器人本体；真相见下节 URDF 检查轮。）
- **真实流基准数值**（sim_demo 栈，8s 裸 WS 客户端）：/camera/image_raw 754.8fps、
  帧龄 5.1ms；/test_cloud 29.9fps、3.8ms；/camera/image_raw/compressed 15.0fps、3.6ms。
- 浏览器自动化经验：`playwright.evaluate` 的字符串按**表达式**求值（`() => {}` 只会
  返回函数对象，须写成 IIFE）；IAB 模块缓存换血须关旧标签+新标签+清
  `rwl-viz-cfg-v4`（Displays cfg 会被旧模块页重新持久化，先清再导航）。

## URDF 显示检查轮（2026-10-16 三弧）：robot-mesh symlink 误杀修复

对 RobotModel 显示链（/robot_description → URDF 解析 → robot-mesh 服务 → 渲染 →
TF 对齐 → 驱动跟动）做专项检查，抓到并修复一个真实缺陷：

- **robot-mesh 全 404（resolve-based containment 误杀 colcon 布局）**：TB3 URDF 的
  4 个 mesh（waffle_pi_base/left_tire/right_tire/lds .stl）请求全 404——`_is_under`
  两侧 resolve() 比对，而 `--symlink-install` 布局下 install share 树内资源目录
  合法 symlink 回 src 树，候选路径 resolve 后落在 install 树外被判越界。与
  SymlinkStaticFiles（starlette realpath containment）同款坑第二次出现。**修复**：
  改词法 containment（normpath 折叠 `..` 后树下比较，不 resolve）；穿越防线不变
  （路由拒 URL `..` + normpath 检查），盘上 symlink 目标位置不再作为拒绝依据。
  单测 `test_robot_mesh_symlink_layout_and_traversal` 同步翻转语义并加 colcon
  布局回归。修复后 curl 实测 200 / model/stl / 7.9MB。
- **验证面**（浏览器实测）：mesh 4×200 加载、TF 树 12 帧 = URDF 全部 11 链节 +
  odom（100% 对齐）、TF 轴与 Fixed Frame 下拉 14 项正常、机器人实体网格视觉确认
  （底盘+双轮胎+上层结构）、经 `/cmd_vel` 驱动 0.786m 模型随 TF 移动且 odom 轨迹
  线正确延伸。修复后全量 66/66 绿。
- **TF 轴位置数值核验（2026-10-16 TF 轮）**：11 帧轴世界坐标逐项对上 waffle_pi
  URDF 真值（base_link 高 10mm、轮 ±144mm/高 33mm、雷达 (−64,0,132)mm、相机
  (73,−11,94)mm、IMU 高 78mm），方向映射无反转；标签位置语义修复一处——
  TF 帧标签原挂帧 Group（继承帧姿态）沿**局部 +Z** 上浮，光学帧
  （camera_rgb_optical_frame，Z 朝前）标签会水平飘到轴前方 0.14m；改挂 Z-up
  世界容器沿**世界 +Z** 上浮（帧姿态不参与），实测三帧（含光学帧）偏移统一
  (0, 0.14, 0)。探针换算：three 世界系 three.y=ROS z、three.z=−ROS y
  （_rosFrame 旋转）；读矩阵免 THREE 全局用 `obj.matrixWorld.elements[12..14]`。
- **机器人姿态与 Fixed 根双修复（2026-10-16 姿态轮，用户实测反馈"机器人侧倒/
  TF 不准"）**：① `_onTf` 的 `baseTf.decompose` 曾直接写 `_robotFrame` 的
  quaternion，把创建时的 -90°x Z-up 补偿整个覆写——第一条 /tf 后 URDF 即侧倒、
  与 TF 轴差 90°（数值实锤 quaternion=[0,0,0,1]）。改为内层 `_robotBase` 承载
  ROS 基座位姿、`_robotFrame` 恒保补偿（ghost 同挂 _robotBase）。② `_tfRoot()`
  按树插入序取首个 isRoot 并**锁定**——树生长早期（仅 RSP 关节变换时 base_link
  是根）会把链中帧钉成 fixed，世界参照系错位。改为按 RViz 惯例优先 map → odom
  且仅锁定首选根，退解根不缓存待树长全重评。终验（同刻同读）：odom 真值
  (5.75, 0.42, −156.8°) = 模型 = base_footprint 轴，位置姿态全等；俯视截图
  确认立正（轮胎左右圆形完整、雷达顶部居中偏后）。另注：按需渲染
  （_dirty 门控）下 `matrixWorld` 非实时，探针读数前须手动
  `updateWorldMatrix(true,false)`。
- **教训（自动化方法）**：① 页面内探针读 `_tree.frames()`/`_tfObjects` 这类
  **Map** 结构时 `Object.keys()` 恒空，须用 `.size`/`[...m.values()]`（本轮曾因此
  误报"TF 帧丢失"）；Fixed Frame 下拉真 id 是 `gopt-fixed`。② 视觉分析工具会把
  激光点/TF 轴簇"脑补"成机器人本体——mesh 类验证必须以 HTTP 状态码/字节数为
  硬证据，视觉只作旁证。③ JS `client.publish` 是三参 `(topic, msgType, message)`
  （对齐 teleop.js），两参调用静默无效。

## nav2 官方化轮（2026-10-16 四弧，用户裁定"参考原始官方启动调试测试方法"）

用户实测反馈"机器人超出地图的障碍物"。根因：sim_demo 的 nav2 段此前用
nav2_bringup 分段 localization/navigation launch + **默认参数（按 tb3_sandbox
场景调）**——AMCL 参数与 TB3 雷达/世界不匹配，定位漂移即"穿出障碍物"。
按官方原始方法整段官方化：

- **官方 `turtlebot3_navigation2` 包进工作区**（ROBOTIS-GIT/turtlebot3 jazzy
  分支，external/ 克隆 + src/examples/ symlink 构建；纯安装 launch/param/rviz/map）。
- **sim_demo nav2 段**：`rviz:=true` → Include 官方 `navigation2.launch.py`
  全官方形态（bringup_launch + **param/waffle_pi.yaml 官方调参** + 官方 RViz
  tb3_navigation2.rviz）；默认无头 → 同一 bringup_launch + 同一官方参数与
  官方配套 map（PythonExpression 组合 nav2∧¬rviz 条件——Jazzy 无 AndCondition）。
  此前自拷的 maps/ 目录删除（官方包单源）。
- **官方调试流程验证**（ros2 CLI = RViz 2D Pose Estimate / Nav2 Goal 话题等价）：
  /initialpose 一次即精确收敛（−2.00,−0.50）；目标 (1,1) 落在障碍柱占据区时
  Nav2 投影到最近可达点停稳（**不穿障碍**）；空闲目标 (−1.2,0.8) 精确到达
  (−1.19,0.81)，全程位姿连续无跳变。web console 经网关同流程同样秒收敛。
- 回归：sim 19/19、全量 66/66 全绿。e2e_harness 白名单加 turtlebot3_navigation2；
  ament_cmake 包构建须补 PYTHONPATH（ament_package）。
- 教训：launch 复合条件 Jazzy 用 `IfCondition(PythonExpression(...))`；
  TaskStop/kill 主进程后 nav2 组件容器会孤儿残留，须按进程名补刀清理。

## 已知问题（演示/长跑注意）



- **gz 无头相机火流**（2026-10-16 实证，见上节）：gz-sim 8.15 无头模式相机渲染不按
  SDF update_rate 节流（~4.7kHz/5GB/s DDS），抬高整机负载且订阅原始流会饿死服务
  响应。缓解已落地（console 默认压缩流）；桥侧丢帧不区分帧类型为上游缺口
  （vendored C++，暂不动）。


- **FastDDS SHM 段随反复起栈累积**（/dev/shm/fastrtps_* 今日达 114 个）：长命桥遇
  「fixture 重启后的新参与者」会丢数据面（订阅成功但 0 消息，CLI 同域可收）；
  长命 fixture 也可能出生即哑。演示前整组重启（桥+fixture 同批），必要时清
  /dev/shm/fastrtps_*（先停全部 ROS 进程）。E2E 测试不受影响（每轮全新随机域整组起停）。
- **zero-uuid 批量 cancel（cancelAll）可令 rclpy action server 端 executor 死锁**
  （2026-10-13 浏览器自检实证：cancelAll 后 fixture 全部回调停摆、进程存活；按
  goalId 精确取消不受影响）。console 自检已改用精确取消；演示长跑建议避免 cancelAll。
- **桥实例经历大量异常重连风暴后 clientPublish 通路可能失效**（表现为 advertise 成功
  但消息不出 DDS）。上游代理不可达期的浏览器重连风暴可触发；重启桥即恢复。生产
  部署应保证桥先于网关可用（transport.launch 顺序已保证）。**2026-10-14 起网关层
  已缓解**（上游有限重试 + 客户端/player 双侧抖动退避错峰），桥侧根因未修
  （vendored C++，本轮不动）。
- **CDR 解码的整型标量是 BigInt**（vendored @foxglove MessageReader 语义）：浏览器侧
  比较数值须先 `Number()`（console 自检 add_two_ints 曾因此误判失败）。
- ~~ros_client.publish 把 msgType 透传给 player.publish 当消息体编码成空载荷~~
  已修（2026-10-13 浏览器自检抓出；player.publish 只收 (topic, message) 两参）。
  发布验证以 E2E（rclpy CDR）与浏览器自检（JS CDR）双路交叉为准。
