# ros_web_link 架构（SNAPSHOT）

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

## 通信矩阵验证（本包的验收面）

| 层 | 工具 | 内容 |
|---|---|---|
| JS 纯核 | vitest（复用 dashboard node_modules） | 协议帧/参数助手/TF 数学/**CDR 黄金字节交叉验证**（rclpy 序列化真值 vs JS codec，17 类常用可视化消息，含 PoseWithCovarianceStamped）/**PointCloud2 共享解析器**/**viz_core（缓动/运动动画器/协方差椭圆/odom 轨迹/回放段/预设存取）** 共 108 用例 |
| Python 单元 | pytest（venv） | 路由挂载 / 组合模式 / health+console 回归 / 配置默认值 / robot_mesh 解析与 symlink 防穿越 / 静态文件类 / 上游重试 helper 共 16 用例 || E2E 通信矩阵 | pytest（自起桥+网关+fixture，随机域/端口） | 订阅/发布 roundtrip/服务/action(goal+feedback+cancel+result)/参数 get-set-get/TF/图谱/健康(含桥组件探测)/视频代理负例/停机端口复核 11 用例（栈编排复用 e2e_harness） |
| E2E 官方示例 | pytest（`Stack(official=True)`：官方 RSP/panda + 官方 CLI joint_states 驱动 + 官方 map_server/lifecycle_manager） | 官方 panda /robot_description、官方 /tf_static、**web JointState（带时间戳）→官方 RSP→/tf 关节角闭环**、官方 /map 384²、官方生命周期服务 GetState/LoadMap、官方 /diagnostics+/rosout 共 7 用例（panda 资源缺失整组 skip） |
| E2E 官方仿真数据 | pytest（`LaunchStack` 整仓拉起 `sim_demo.launch.py`） | turtlebot3 官方仿真 + 官方 nav2 + ros2/examples 官方示例经网关：robot_description/joint_states/scan/odom/imu/tf/相机+Info/压缩图/官方点云/talker/map/amcl_pose/粒子云/诊断共 16 用例，含 **web→AMCL 定位闭环** 与 **web→NavFn 规划闭环**（`test_e2e_sim.py`，替代已删的合成流可视化套件） |
| 人工/自动化 | console.html「运行自检」+ 话题回显 | 浏览器内同矩阵（`window.__rwl_selftest`，9 项）+ 任意话题直接回显 |

黄金字节生成器 `test/generate_viz_golden.py`：schema 文本与桥 message_definition_cache
同构（根 .msg + DFS 首见序依赖 + `====…MSG: pkg/Type` 分隔；同包裸名按同包解析，
Header→std_msgs/Header）。升级 ROS 或改 fixture 值后重新生成。

E2E 编排注意（与工作区 MUST 对齐）：`asyncio.create_subprocess_exec` +
`start_new_session=True`，SIGINT 整组收尾，收尾断言"端口已释放"；子环境补
`ROS_DISTRO`（桥 banner 对 null getenv 会崩）与桥包 lib 目录（平时由 setup.bash 补）。
**随机域避开本机占用域**（2026-10-14 修）：`free_domain()` 读 /proc/*/environ
枚举存活进程的 ROS_DOMAIN_ID 并排除——此前随机 2-230 会撞上并发会话/demo 栈的
常驻域（实证 2/80/86），表现为 /tf 混入他人帧（AUBO link 名出现在 viz 断言里）
与 JTC goal 被拒；MoveIt 执行用例的 send_goal 另加了退避重试（冷启动偶发拒绝，
机器高负载时放大）。

## 决策记录（本包内）

- **官方优先原则（2026-10-15 用户裁定）**：官方生态能提供的数据面一律用官方组件
  （RSP/panda、map_server、CLI、lifecycle），自研 fixture 退守合成传感器流与传输矩阵
  被测端；入口 `official_demo.launch.py`，测试面 `test_e2e_official.py`（7 用例）。
- **viz_fixture 全删、数据面上移官方仿真（2026-10-15 二次裁定）**：合成传感器流
  fixture 整体删除；验证数据面 = GitHub 热门官方仓库（克隆进项目：
  src/turtlebot3_msgs、src/turtlebot3_simulations、src/ros2_examples；
  external/navigation2 基线克隆不参与构建）+ ros2/moveit2/nav2 官方示例；
  本地自写项目（sim/peach/ivg）不参与验证。入口 `sim_demo.launch.py`
  （官方 TB3 gz 仿真 + 官方 nav2 + ros2/examples + 官方 republish + 传输栈），
  测试面 `test_e2e_sim.py`（16 用例，`LaunchStack` 整仓拉起）。
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
  ① Fixed Frame 选非根帧（如 base_footprint）时 3D 视口空白，切回 map 恢复——
  `_frameToFixed` 对"根→叶"反向查找疑似返回空，待修（显示链问题，数据流正常）；
  ② 官方桥高吞吐下事件流刷 `Send buffer limit reached`（桥→WS 背压，与
  BEST_EFFORT 丢帧同源，Foxglove 语义下属预期降级）；
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
  rgb packed/intensity（含整型归一）；viz 与渲染岛共用，vitest 覆盖。
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
- **共享纯核测试随迁**：tf-math/utils 单测从 dashboard 迁入本包（vitest 51 用例）；
  dashboard vitest `--passWithNoTests`。
- **SymlinkStaticFiles**：colcon `--symlink-install` 下 starlette 0.27 的 realpath
  containment 检查会把指向 build 树的符号链接判越界 → install 树静态全 404。本包
  挂载改用 normpath `..` 段检查的 lookup_path（目录路径也回 stat 以驱动 index.html
  回退）。MultiStaticFiles 同型。

### JS 树归属（重构后）

| 位置 | 内容 |
|---|---|
| ros_web_link `web/public/js/` | transport/（foxglove 协议栈+门面+action）、core/（log/tf/utils/runtime 共享纯核）、view3d/、vendor/（three/urdf/transforms）、robotwebtools/（importmap+builds+libs+roslib-esm）、console/、ivg_{transport,runtime}.js、全部 `__tests__` |
| dashboard `web/public/js/` | 面板 html/js、components/、latte/、vision_grasp/、core/{dom_cache,ivg_status_bar,lifecycle,topics}、ivg_site_nav.js |

已删除（两树确证死代码）：core/entities/{connection,ros_data,service}_store、
entities/ui_state_store、components/topic_browser、core/{ros_connector,settings}、
view3d/hints；本包 config 移除未用的 package_version_fallback。

## 已知问题（演示/长跑注意）

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
