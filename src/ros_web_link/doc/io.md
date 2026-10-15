# ros_web_link 输入输出（SNAPSHOT）

本包对外的全部端点、话题、服务与进程边界。

## 官方示例互操作验证（2026-10-15，用户指定验收面）

与 ROS 2 生态**官方组件**的双向互操作（非 fixture，全部经网关 /ws/foxglove）。
**官方优先原则（2026-10-15 用户裁定）**：官方生态能提供的数据面一律用官方组件，
自研 fixture 退守合成传感器流（无硬件/仿真时）与传输矩阵被测端。
一键官方栈：`ros2 launch ros_web_link official_demo.launch.py`（官方 RSP/panda +
官方 CLI joint_states 驱动 + 官方 map_server/lifecycle_manager + 官方地图 +
本包桥/网关 + fixture 补充，`viz_publish_urdf:=false` 让位官方 /robot_description）。

| 官方面 | 组件 | 验证内容 |
|---|---|---|
| MoveIt 2 | `moveit_resources_panda`（官方 ros2 分支） | E2E 6 用例：latched /robot_description → /joint_states → /compute_ik → OMPL /plan_kinematic_path → /panda_arm_controller/follow_joint_trajectory 执行 SUCCEEDED → /monitored_planning_scene（`test_e2e_moveit.py`） |
| Nav2 | `nav2_map_server` + `nav2_lifecycle_manager` + 官方地图 `tb3_sandbox`（384²@0.05m） | /map OccupancyGrid 浏览器渲染；官方生命周期服务经网关调用（GetState→active、LoadMap 重载成功）；`ros2 lifecycle set` 官方 CLI 兜底拉起（`Stack.ensure_map_active`） |
| 官方 RSP | `robot_state_publisher`（panda URDF） | 官方 /robot_description + /tf_static；**web 发布 JointState（带时间戳）→ 官方 RSP → /tf 反映关节角**（浏览器实测 z=0.38942 vs 期望 sin(0.4)=0.38942 全精度；`test_e2e_official.py` 7 用例） |
| ROS 2 官方工具 | `ros2 topic pub/echo`、`ros2 run tf2_ros static_transform_publisher`、`ros2 lifecycle set` | 官方 pub /chatter → 浏览器回显 ✓；浏览器发布 /chatter → 官方 echo 捕获 ✓；**2D Goal Pose 位姿比特级往返**（float64 全精度）；官方 static TF（map→odom）进 TF 显示并把工具 fixed-frame 切到 `map`（RViz 语义） |
| 官方诊断/日志 | lifecycle transition /diagnostics、/rosout | 诊断面板「Nav2 Health」；rqt_console 面板收官方节点日志 |
| 不可用项 | `demo_nodes_cpp` / `turtlesim` 未装且无免密 sudo | 以 `ros2 topic pub/echo`（同为 rclpy 官方实现）替代覆盖同等互操作面 |

环境注意（三个实坑，均已记入 architecture）：
1. 官方组件用 `/opt/ros/jazzy/bin/ros2` + env 前缀；RSP 的 URDF 插件需 vendor 库链
   （`/opt/ros/jazzy/opt/*/lib`，sdformat14→gz-math7）。
2. **多行 URDF 禁止经 `-p name:=<value>` 直传**（execve 路径下 rcl 参数 YAML 解析
   拒收 rc=250）——用官方标准 `--params-file`（YAML block scalar）。
3. **RSP 对零时间戳且值恒定的 JointState 去重**（/tf 静默）——官方 CLI pub 无
   header 字段即触发；web 驱动 RSP 必须带当前时间戳。

## MoveIt 2 官方 Panda 示例端到端（2026-10-14）

官方 `moveit_resources_panda`（GitHub ros2 分支本地构建）+ 本包传输栈组合：

```bash
bash src/ros_web_link/scripts/run_panda_demo.sh   # 桥 18766 + 网关 8095
# 浏览器 http://127.0.0.1:8095/console.html（白色主题；RobotModel 加载 panda）
```

E2E（`test/test_e2e_moveit.py`，6 用例，全经网关 /ws/foxglove）：
latched `/robot_description`（panda URDF+mesh）→ `/joint_states`（9 关节）→
`/compute_ik`（GetPositionIK）→ `/plan_kinematic_path`（OMPL RRTConnect 规划，
goal 约束→轨迹）→ `/panda_arm_controller/follow_joint_trajectory`（执行→
joint_states 反映终点）→ `/monitored_planning_scene`（触发规划后收到快照）。

环境要点：MoveIt 节点需 vendor 库链（sdformat14→gz-math7、ompl18——
`/opt/ros/jazzy/opt/*/lib`），桥默认 18766（8765 常被外来 preview_server 占用）；
`/monitored_planning_scene` 仅随变化发布（测试先订阅再触发规划）；OMPL 冷启动
偶发 99999（current-state monitor 未就绪，测试退避重试）。

## 进程与端口

| 进程 | 入口 | 默认端口 | 说明 |
|---|---|---|---|
| 网关 | `ros_web_link_gateway` / `python3 -m ros_web_link.cli` | 8095（bind 0.0.0.0） | 8090 归 peach_observability，不可占 |
| 上游桥 | `ivg_foxglove_bridge`（本包 launch Include） | 8765 | 唯一 ROS WebSocket 传输面 |
| 演示节点 | `rwl_comm_fixture`（launch `fixture:=true`） | — | `/rwl_demo/*`，验证/演示用 |

环境要求：网关须在 `aubo_py3.12` venv 下运行（fastapi==0.101+starlette==0.27+
uvicorn+websockets==12，仓库根 requirements.txt）；launch 中 `python3` 指向该 venv。

## HTTP / WebSocket 端点（网关）

| 端点 | 方法 | 方向 | 语义 |
|---|---|---|---|
| `/health` | GET | 诊断 | `{status, static_root, foxglove_bridge, web_video}`；桥/视频 TCP 探测（1s 超时，2s 缓存）；桥 down → `status=degraded`（仍 200） |
| `/ws/foxglove` | WS | 浏览器→桥 | 子协议 `foxglove.websocket.v1`（归一 sdk.v1）；CDR 二进制；上游有限重试+抖动，全失败 close 1011 |
| `/api/ivg/proxy/web-video/{path}` | GET/HEAD | 浏览器→web_video_server | MJPEG 流/JPEG 快照；上游不可达 502/504（共享 httpx 连接池） |
| `/api/ivg/robot-mesh/{pkg}/{rel}` | GET | 浏览器→ament share | STL/DAE/OBJ；防路径穿越（resolve-based，含 symlink）；缓存 1d |
| `/console.html`（纯传输）或 `/rwl/console.html`（组合） | GET | 诊断台 | 生态通信矩阵人工验证 |
| `/js/**`（纯传输模式） | GET | 静态 | 通信层 JS 资产 |

## comm_fixture 图名（`/rwl_demo` 命名空间，全部故意避开生产命名）

| 名字 | 类型 | 说明 |
|---|---|---|
| `/rwl_demo/chatter` | std_msgs/String @5Hz | 订阅面被测 |
| `/rwl_demo/echo` → `/rwl_demo/echo_reply` | std_msgs/String | 客户端发布 roundtrip（回发 `echo:<原串>`） |
| `/rwl_demo/add_two_ints` | example_interfaces/srv/AddTwoInts | 服务面 |
| `/rwl_demo/fibonacci` | example_interfaces/action/Fibonacci | 0.15s/步 feedback，支持 cancel |
| 参数 `comm_fixture.rwl.answer`（int，默认 42）、`rwl.step_s`（double） | — | 参数服务读写 |
| `/tf`：`rwl_demo_base` → `rwl_demo_marker` @1Hz TRANSIENT_LOCAL | tf2_msgs/TFMessage | TF 面 |

## viz_fixture 图名（`/rwl_viz` 命名空间，@1Hz，TRANSIENT_LOCAL）

常用可视化消息独立被测端（`ros2 run ros_web_link rwl_viz_fixture`；console 话题回显可直接订阅查看）：

| 话题 | 类型 | 覆盖的 CDR 形态 |
|---|---|---|
| `/rwl_viz/image` | sensor_msgs/Image（mono8 棋盘 64×48） | uint8 blob + 帧序号嵌入首像素 |
| `/rwl_viz/image_compressed` | sensor_msgs/CompressedImage | 真实可解码棋盘 JPEG（两相位交替；`_jpeg_frames.py` 内嵌，web 端实解渲染） |
| `/rwl_viz/camera_info` | sensor_msgs/CameraInfo | float64[9] 定长矩阵 + 变长 d |
| `/rwl_viz/points` | sensor_msgs/PointCloud2（XYZ×128） | 嵌套 PointField[] + data blob |
| `/rwl_viz/scan` | sensor_msgs/LaserScan（181 点） | float32[] 含 inf |
| `/rwl_viz/range` | sensor_msgs/Range | 标量 + 枚举 |
| `/rwl_viz/odom` | nav_msgs/Odometry | 嵌套 Pose/Twist + float64[36]×2；圆周轨迹（0.15 rad/s 弦长>0.1m 采样阈） |
| `/rwl_viz/path` `/pose_array` `/pose` | nav_msgs/Path, geometry_msgs/PoseArray/PoseStamped | 嵌套位姿数组 |
| `/rwl_viz/pose_cov` | geometry_msgs/PoseWithCovarianceStamped | 位姿 + 摆动协方差（95% 椭圆显示；float64[36] CDR 解码为 Float64Array） |
| `/rwl_viz/wrench` | geometry_msgs/WrenchStamped | 嵌套向量 |
| `/rwl_viz/marker` `/markers` | visualization_msgs/Marker(SPHERE_LIST)/MarkerArray | points+colors 嵌套数组、UTF-8 文本 |
| `/rwl_viz/diagnostics` | diagnostic_msgs/DiagnosticArray | 嵌套 status/KeyValue、int8 标量（rclpy 侧为 bytes） |
| `/rwl_viz/joint_states` | sensor_msgs/JointState | 变长并行数组（6 关节） |
| `/tf`（rwl_viz_base→rwl_viz_marker） | tf2_msgs/TFMessage | 动态 TF |
| `/rwl_viz/clicked_point` | geometry_msgs/PointStamped | 存活发布（Publish Point 工具的 schema 前置），真实点击由 web 端发布 |
| `/initialpose`（PoseWithCovarianceStamped）、`/goal_pose`（PoseStamped） | geometry_msgs | Nav2/RViz 交互目标话题存活发布；web「2D 初始位姿 / 2D Goal Pose」工具同型 schema |
| `/rosout`（rcl_interfaces/Log） | — | 各节点 rosout；fixture 每 5s 一条 INFO（rqt_console 面板数据源） |

验证面：Python E2E（`test/test_e2e_visualization.py`，14 用例，rclpy 交叉解码）、
CDR 黄金字节（`viz_golden.json` + vitest 15 用例，rclpy 序列化真值 vs JS codec）、
浏览器实测（console 订阅抽验）。

## 实时可视化（console.html「实时可视化」区，RViz2/rqt 风格）

连接后按 **Displays 面板**配置订阅并渲染（`js/console/viz.js` + `plot.js` + `topo.js`，
three.js 经 robotwebtools importmap 裸名引入）：

| 组件 | 内容 |
|---|---|
| 工具栏 | 测距、发布点、2D Goal Pose、**2D 初始位姿（/initialpose + RViz 标准协方差）**、等轴/俯视/侧视/保存恢复、**TCP 拖拽**/录制、ROS Time |
| Displays 面板 | 每显示项：启用 / 话题（datalist）/ 属性（点大小、TF 名签、幽灵、odom 轨迹、wrench 缩放）/ 状态；配置持久化 |
| 3D 视口 | Grid、RobotModel（mesh 经 robot-mesh 路由 + **幽灵预览** + **TCP 示教拖拽** + DLS IK 伺服 + **工作空间包络线框**）、OccupancyGrid、TF（**tf_buffer 全精度链路累积**）、PointCloud2（**fields offset 解析**，与渲染岛共用 pointcloud_parse）、LaserScan 3D、Marker、Path、PoseArray/PoseStamped（箭头）、**Odometry（箭头+0.1m 阈值轨迹环）**、**Range（±fov 扇形）**、**Wrench（force 红/torque 蓝箭头）**、**PoseWithCovarianceStamped（95% 协方差椭圆）**——数据组按 `header.frame_id` 经 TF 摆位（RViz 帧语义） |
| 2D 视口 | Image（mono8/rgb8/bgr8）+ **CompressedImage（createImageBitmap 实解 JPEG/PNG，与 raw 共卡）**、LaserScan 扇形、JointState 关节条（**镜像防环 v2** + **跟踪误差读数** + TCP HUD 毫米读数） |
| 字段绘图（rqt_plot） | 任意字段路径 + **计算字段**（`_computed_min/max/avg`）、暂停/时间窗、EMA Hz |
| 诊断面板（rqt_runtime_monitor） | DiagnosticArray 按级别三色列表（话题可配，默认 /rwl_viz/diagnostics，真机改 /diagnostics）；`__rwl_diag()` |
| ROS 日志面板（rqt_console） | /rosout 订阅、级别过滤（DEBUG+/INFO+/WARN+/ERROR+）、暂停/清空、上限 500；`__rwl_roslog()` |
| 拓扑图 | SVG 节点/话题/服务四层 + 流动粒子 + Hz 徽标 + 滚轮缩放/拖拽/fit |
| 示教/轨迹面板 | 航点录制（90ms+4mm）/导出 JSON/**本地回放（段时长 80-900ms 钳制 + easeInOutQuad）**/**姿态预设（内置零位+用户存档，easeInOutCubic 850ms，armed 时 45ms 节流写 joint_cmd）**/**FollowJointTrajectory 真执行**（失败→低级逐点 joint_cmd 回退）+ 交互锁 guardedCall |
| 分割条 | 左右列间拖拽（30-80%） |

`/rwl_viz/clicked_point`、`/goal_pose`、`/initialpose`（PoseWithCovarianceStamped）、
`/robot_description`（7 links / 6 revolute joints 演示箱链，joint_0..5 与 joint_states 同名）、
`/rwl_viz/joint_cmd`（关节命令闭环）均由 viz_fixture 提供存活/latched 发布（浏览器发布的
schema 前置；RViz 亦为持续 advertise 语义）。**关节闭环**：web 滑块 → `/joint_cmd` →
fixture 3s 内跟随 → `joint_states` → URDF 模型（Rebot-Arm 镜像模式最小版）。
快照：`window.__rwl_viz_stats()`（含 fps/objectCount/jointCmdSent/odom/range/wrench/posecov/imagec）、
`window.__rwl_plot()`、`window.__rwl_topics()`、`window.__rwl_diag()`、`window.__rwl_roslog()`。

### 前端设计参考来源（2026-10-14 深读移植轮）

深读两项目源码后的逐函数移植（vanilla-JS 零构建，不引框架）：

**Rebot-Arm 交互链**（`rebot-sim.js`/`rebot-ros-ui.js` 行号映射）：
| 移植件 | 本包文件 | 参考行号 |
|---|---|---|
| 命令总线（source 标记/浅拷贝分发/插值静默） | `console/bus.js` | :1708-1734 |
| DLS IK 伺服（gain12/λ.035/雅可比 eps.004/3×3 Cramer/16ms 子步） | `console/ikcore.js` | :1455-1676 |
| 工作空间包络（960 确定性采样+半球夹紧） | `console/ikcore.js` | :512-542, :1173-1200 |
| 航点录制（90ms+4mm 双闸）/ 导出 waypoints_v1 | `console/ikcore.js` + `teach.js` | :1273-1390 |
| 轨迹构造（首点当前@50ms/比例重排/≥0.2s 地板） | `console/ikcore.js` | :427-438 |
| TCP 拖拽标记+相机朝向平面+目标球+误差线 | `console/teach.js` | :963-1221 |
| 幽灵机器人（clone+独立半透明材质+getObjectByName 回退） | `viz.js _createGhost` | :590-597 |
| 镜像防环 v2（一次性 hold/到位 0.025 rad 即删/1800ms 超时） | `viz.js _onJoints` | :190-220 |
| 跟踪误差读数（最大/RMS 角度，绿/琥珀/红） | `viz.js _onJoints` | :676-701 |
| ROS Z-up 容器（rotation.x=-π/2） | `viz.js _onRobotDescription` | :251 |
| mesh 解析链（packages 回调→robot-mesh 路由+file://→package:// 重写） | `viz.js _onRobotDescription` | dashboard urdf_island 同构 |
| 低级轨迹回退（墙钟逐点 joint_cmd） | `console/panel.js replayLowLevel` | :466-487 |

**RVIZ-RQT-VISUAL 可视化**（`frontend/src/` 行号映射）：
| 移植件 | 本包文件 | 参考行号 |
|---|---|---|
| SVG 拓扑图（四层+箭头 marker+类型色板+Hz 徽标+流动粒子） | `console/topo.js` | NodeTopicGraph.vue:155-302, 792-895 |
| 拓扑交互（滚轮绕鼠标缩放/平移/fit） | `console/topo.js _bindEvents` | GraphInteraction.js:246-270, 410-441 |
| 绘图计算字段（`_computed_min/max/avg` 后缀+数组解析） | `console/plot.js` | ChartPanel.vue:1538-1630 |
| 字段活体解析（depth=1；数组→[0]+计算字段） | `console/plot.js parseFields` | ChartPanel.vue:719-778 |
| OccupancyGrid 话题（data[-1..100]→色板→CanvasTexture/PlaneGeometry） | `viz.js _onOccupancyGrid` | Scene3D.vue:2976-3173 |
| 列间分割条（30-80% 钳制） | `console/panel.js initSplitter` | MainLayout.vue:826-867 |

**通信对照表**（rebot-ros-client.js / useConnectionStore.js 逐项核对）：
| 能力 | 参考实现 | 本包现状 |
|---|---|---|
| 重连+会话锚 | :42-85 connect-sequence guard | ✅ player.js sessionId 重置 |
| 服务 Promise/超时 | :118-134 pending map | ✅ player.js callService + timeout |
| 订阅去重 | refcount 一线一订 | ✅ player.js _topicSubs callbacks Set |
| advertise 竞态防护 | auto-advertise before publish | ✅ player.js publish 惰性 advertise+300ms hold |
| per-topic liveness | :214-217 lastMessageAt | ✅ app.js mon.topicLastAt（监控差分） |
| 旧 socket 迟到 close | connect-sequence guard | ✅ protocol.js `this._ws !== ws` 判定 |
| action 确认语义 | formatServiceResult | ✅ panel.js guardedCall + 确认弹窗 |

**2026-10-15 深读补植轮**（第二轮未移植件，全部 vanilla-JS 零构建）：
| 移植件 | 本包文件 | 参考行号 |
|---|---|---|
| moveToAngles 缓动（easeInOutCubic + RAF 状态机） | `console/viz_core.js` createJointMotion + viz.js `_stepMotion/_animateTo` | rebot-sim.js:876-903 |
| 姿态预设（通用化：内置零位 + 用户存档 localStorage） | `viz_core.js` createPresetStore + panel.js | rebot-sim.js:50-57, 745-759 |
| 示教本地回放（段时长 80-900ms 钳制 + easeInOutQuad） | `viz_core.js` replaySegmentDuration/sampleSegment + panel.js | rebot-sim.js:1298-1352 |
| 工作空间包络线框（5 环 + 12 经线弧，主/辅透明度） | `console/teach.js _buildEnvelope` | rebot-sim.js:352-393, 551-559 |
| TCP 可达 HUD（毫米读数 + 夹紧变红） | `teach.js _updateMarkerScreen` | rebot-sim.js:1767-1779 |
| XY 协方差椭圆（95% 置信，特征值分解） | `viz_core.js` covarianceEllipse + viz.js `_onPoseCov` | Scene3D.vue:1956-1993 |
| 里程计轨迹（0.1m 位移阈 + 环形上限） | `viz_core.js` createOdomTrail + viz.js `_onOdom` | Scene3D.vue:1807-1817 |
| 诊断指示（DiagnosticArray → 级别三色列表） | `console/monpanels.js` DiagnosticsPanel | StatusPanel.vue:224-242 |
| ROS 日志（/rosout 级别过滤/暂停/上限淘汰） | `console/monpanels.js` RosLogPanel | LogViewer.vue:102-160（数据层换真 /rosout） |
| 2D Pose Estimate（协方差常量 0.25/0.0685…） | `viz.js` 工具 'poseest' | Scene3D.vue:2352-2424 |
| RViz 类型显示补全：Range 扇形 / Wrench 双箭头 / Odometry 箭头+轨迹 / CompressedImage 解码 | `viz.js` 各 handler | RViz2 对应 display 语义 |

**明确不移植**（裁定）：fake/real 两档模式（Rebot-Arm rebot-ros-ui.js:153-171, 548-567）——
诊断台定位是传输/可视化调试客户端，不是真机控制面；真机授权语义归工作区 AGENTS.md
（交互锁仅面向 /rwl_viz 演示闭环），引入「真机已解锁」红标反而暗示本台可控制真机。

## 传输与交互观测（console.html「传输监控」区）

看 web↔ROS 传输/交互情况的主入口（纯传输 `/console.html`，组合 `/rwl/console.html`）：

| 观测项 | 数据源 | 说明 |
|---|---|---|
| 状态 / 心跳 RTT | player.state + heartbeat（参数服务往返） | RTT 即传输往返延迟基线 |
| msg/s、KB/s 曲线（60s） | player.stats 差分 | 聚合吞吐 sparkline |
| 累计消息/流量/服务调用/重连/解码失败 | player.stats | 含服务成功/失败分计 |
| 话题级 Hz / KB/s / 最近一帧时延 | player._topicSubs.stats 差分（诊断台特权读取） | 任意订阅自动入表；「＋监控话题」按输入框订阅 |
| 交互日志 | callService/publish 包装（含 action 的 send_goal/get_result/cancel_goal） | 时间戳 + 成败 + 耗时 ms |
| 事件流 | onStateChange/onStatus/重连/CDR 解码失败差分 | 连接与告警时间线 |
| 自动化快照 | `window.__rwl_monitor()` | 上表全部数据的 JSON |

回显渲染 400ms 节流（大消息逐帧渲染曾是 CPU 热点）。dashboard 面板侧的同类观测在
`transport/bridge_logger.js` + 状态栏（`core/ivg_status_bar.js`）。

## foxglove 协议面（经 /ws/foxglove）

- capabilities：`clientPublish`、`services`、`connectionGraph`（time 随 use_sim_time）
- JSON op（客户端→服务器，vendored 批量形式）：`subscribe`/`unsubscribe`/`advertise`/
  `unadvertise`/`subscribeConnectionGraph`
- 二进制（客户端→服务器）：`[0x01][channelId][CDR]` 发布、`[0x02][serviceId][callId]
  [encLen]['cdr'][CDR]` 服务请求
- 二进制（服务器→客户端）：`[0x01][subId][ts][CDR]` 消息、`[0x02][ts]` 时间、
  `[0x03][serviceId][callId][encLen][enc][CDR]` 服务响应
- action 面 = `_action/{send_goal,cancel_goal,get_result}` 服务 +
  `_action/{status,feedback}` 话题（`action_client.js` 编排）

## 消费方（谁用本包）

- `aubo_ros2_web_dashboard`：`create_app(js_roots, html_root, sub_routers)` 组合；
  面板 JS 直接 import 本包 `transport/*` 与 `core/*`。
- `test/test_e2e_comm_matrix.py`：经 `/ws/foxglove` 全矩阵验收（与浏览器同路）。
- console.html 自检：`window.__rwl_selftest`（自动化可读）+ `window.__rwlSelfTestRun()`。

## 配置单源

`config/defaults.yaml`（网关端口/绑定、WS 传输、桥上游、代理超时与重试）→
`ros_web_link/config.py` 访问器 → CLI 覆盖。改默认值同轮改此文件与 README 端口表。

2026-10-14 传输默认值（对齐 foxglove_bridge 官方建议，均可 yaml/CLI 覆盖）：

| 键 | 默认 | 说明 |
|---|---|---|
| `gateway.ws_ping_interval` / `ws_ping_timeout` | 20 / 20 | 客户端腿心跳（检测半开连接；`--ws-ping-interval -1` 关闭） |
| `gateway.ws_per_message_deflate` | false | 客户端腿 permessage-deflate（CDR 二进制压缩收益差） |
| `foxglove_bridge.compression` | false | 上游腿同上 |
| `foxglove_bridge.ws_path` | `/` | 上游 WS 路径（vendored 桥服务根路径；旧默认 `/ws/foxglove` 是悬空旋钮已修） |
| `proxy.upstream_connect_attempts` | 3 | 上游连接总尝试次数（含首次） |
| `proxy.upstream_connect_retry_ms` | 250 | 重试基础间隔（实际 ±50% 抖动错峰） |

启动预检：CLI bind 失败即打印明确错误退出（launch respawn=true 下端口被占
不再静默死循环）。
