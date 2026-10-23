# ros_web_link 输入输出（SNAPSHOT）

> **2026-10-23 迁移说明**：`src/examples` 与 `external/` 已迁到 `/home/mu/demo_ws`。下文 `src/examples/...` 指 `/home/mu/demo_ws/src/examples/...`，`external/...` 指 `/home/mu/demo_ws/external/...`；运行 `sim_demo.launch.py` 前先 `source /home/mu/demo_ws/install/setup.bash`（或确认 apt 包）。

本包对外的全部端点、话题、服务与进程边界。

> 完整签名/请求响应/错误语义见 [api/http-ws.md](api/http-ws.md)、
> [api/python.md](api/python.md)、[api/launch.md](api/launch.md)；本文只做速查。

## 独立监控模式（2026-10-19）

`ros2 launch ros_web_link monitor.launch.py` = 只起 Foxglove 桥 + 网关，附着到
**已在运行**的 ROS 2 图（完整仿真/真机）。不启动任何演示/仿真节点，不发布运动
指令；浏览器 `http://127.0.0.1:8095/` 307 跳转到 `/console.html`。首次连接且无
用户布局时，`viz_core.adaptInstancesToDataPlane` 按图上真实话题重定向 Displays
（压缩图优先、`/test_cloud`→腕载点云、`/peach/**` 标记可选识别等）；用户保存过
布局后不再改写。测试数据面只用 `src/examples` 官方项目，本地 peach 不参与。

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
| Nav2 | `nav2_bringup/bringup_launch.py` + **官方 `turtlebot3_navigation2` param/waffle_pi.yaml + 官方配套 map**（2026-10-16 官方化；此前 nav2_bringup 分段 launch + 默认参数，AMCL 与 TB3 雷达不匹配会定位漂移） | /map OccupancyGrid 浏览器渲染；官方生命周期服务经网关调用；`rviz:=true` 走官方 navigation2.launch.py（含官方 RViz） |
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

手工核对（浏览器连网关 `/ws/foxglove`，看话题和日志）：
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
| 独立监控 | `monitor.launch.py`（`start_bridge:=false` 可复用外部桥） | 桥 18766 / 网关 8095 | 不拥有仿真；附着到运行中的图（8765 常被 preview_server 占） |

环境要求：网关须在 `aubo_py3.12` venv 下运行（fastapi==0.101+starlette==0.27+
uvicorn+websockets==12，仓库根 requirements.txt）；launch 中 `python3` 指向该 venv。

## HTTP / WebSocket 端点（网关）

| 端点 | 方法 | 方向 | 语义 |
|---|---|---|---|
| `/health` | GET | 诊断 | `{status, version, protocol, static_root, foxglove_bridge, web_video}`；桥/视频 TCP 探测（1s 超时，2s 缓存）；桥 down → `status=degraded`（仍 200） |
| `/ws/foxglove` | WS | 浏览器→桥 | 子协议 `foxglove.websocket.v1`（归一 sdk.v1）；CDR 二进制；上游有限重试+抖动，全失败 close 1011；出站经有界数据通道（字节预算+丢最旧+出口年龄门，见配置单源） |
| `/api/ivg/ws/stats` | GET | 诊断 | WS 转发统计：`{dataLaneBytes, active[按连接], totals 累计}`——forwarded/dropped/droppedStale/queued；console 监控卡与压测证据面 |
| `/api/ivg/proxy/web-video/{path}` | GET/HEAD | 浏览器→web_video_server | MJPEG 流/JPEG 快照；上游不可达 502/504（共享 httpx 连接池） |
| `/api/ivg/robot-mesh/{pkg}/{rel}` | GET | 浏览器→ament share | STL/DAE/OBJ；防路径穿越（resolve-based，含 symlink）；缓存 1d |
| `/`（纯传输模式） | GET | 入口 | 307 → `/console.html`（组合模式 `/` 归 UI 层） |
| `/console.html`（纯传输）或 `/rwl/console.html`（组合） | GET | 诊断台 | 白色二次元商业主题；数据面自动适配；生态通信矩阵人工验证 |
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
| `/rwl_demo/big_image` | sensor_msgs/Image 640×480 rgb8 @30Hz | 大数据源（`-p rwl.bigdata:=true` 启用；~0.92MB/帧，稳定传输压测被测端） |
| `/rwl_demo/big_cloud` | sensor_msgs/PointCloud2 50k 点×16B @10Hz | 大数据源（同上；xyz f32 + packed rgb，正弦环图案） |

## 官方仿真真实数据面（sim_demo.launch.py，2026-10-15 viz_fixture 删除）

viz_fixture 合成流已删除（用户裁定）：可视化验证数据面全部来自 GitHub 官方仓库
（克隆进本项目：src/examples/turtlebot3_msgs、src/examples/turtlebot3_simulations、src/examples/ros2_examples；
external/navigation2 为 Nav2 基线克隆，未参与构建）与 ros2/moveit2/nav2 官方示例，
本地自写项目不参与验证。`ros2 launch ros_web_link sim_demo.launch.py` 一键起：

| 话题 | 类型 | 来源（全部官方） |
|---|---|---|
| `/robot_description` `/tf` `/joint_states` | String/TFMessage/JointState | turtlebot3_gazebo（官方 RSP + gz 关节桥，waffle_pi 轮关节） |
| `/scan` | sensor_msgs/LaserScan（360 点） | gz 激光（ros_gz_bridge，官方 TB3 仿真） |
| `/odom` | nav_msgs/Odometry | gz DiffDrive |
| `/imu` | sensor_msgs/Imu | gz IMU |
| `/camera/image_raw` `/camera/camera_info` | Image/CameraInfo（640×480 rgb8） | gz 相机（官方 image_bridge） |
| `/camera/image_raw/compressed` | CompressedImage（JPEG） | 官方 image_transport republish |
| `/test_cloud` | sensor_msgs/PointCloud2 | ros2/examples 官方点云示例 |
| `/topic` | std_msgs/String | ros2/examples 官方 talker 示例 |
| `/map` | OccupancyGrid（turtlebot3_world 官方配套图 384²@0.05m） | 官方 nav2 map_server（地图与参数均来自官方 turtlebot3_navigation2 包） |
| `/amcl_pose` `/particle_cloud` | PoseWithCovarianceStamped/PoseArray | 官方 AMCL（web 发 /initialpose 激活） |
| `/plan` | nav_msgs/Path | 官方 NavFn 规划器（web 发 /compute_path_to_pose action 触发） |
| `/diagnostics` | DiagnosticArray | 官方 lifecycle_manager（Nav2 Health） |
| `/initialpose` `/goal_pose` `/clicked_point` | geometry_msgs | web 工具发布（Nav2/RViz 标准交互话题） |
| `/rosout` | rcl_interfaces/Log | 各官方节点自带 |

验证：起官方仿真和 `monitor.launch.py`，在浏览器里看 Displays，对照网关日志。

纯 apt 官方示例栈（panda RSP + nav2 map_server/lifecycle，无仿真）见
`official_demo.launch.py`。MoveIt2 官方 panda 全栈见 `panda_moveit_web.launch.py`
（moveit_resources 克隆于 src/examples/）。对不对看浏览器和节点日志。

## 实时可视化（console.html「实时可视化」区，RViz2/rqt/Foxglove 风格）

连接后按 **Displays 面板**配置订阅并渲染（`js/console/viz.js` + `plot.js` + `topo.js`
+ `teleop.js`，three.js 经 robotwebtools importmap 裸名引入）。
2026-10-15 升级轮：**Displays 多实例化**（cfg v5 实例数组，RViz Add/Remove Display
语义；Grid/RobotModel/TF 单例，其余可增删）+ **Global Options**（Fixed Frame 下拉/
背景色/跟随帧）+ **命名布局快照**（存/删/导出/导入 JSON）+ **Teleop 面板**
（WASD 键盘遥控发布 /cmd_vel TwistStamped，仅限仿真）+ **参数面板**
（rqt_reconfigure 简化版：节点全量参数读取+类型内联编辑+写回）+ 快捷键
（F=适配/G=网格/1·2·3=视角/M=测距/Esc=退出）。

| 组件 | 内容 |
|---|---|
| 工具栏 | 测距、发布点、2D Goal Pose、**2D 初始位姿（/initialpose + RViz 标准协方差）**、等轴/俯视/侧视/保存恢复、**TCP 拖拽**/录制、ROS Time |
| Global Options | Fixed Frame（TF 树根+map/odom）、背景色、跟随帧（相机随 TF 帧位移，Foxglove 语义） |
| Displays 面板 | **实例驱动**：＋添加（类型下拉）/删除/悬停复制（RViz Duplicate 语义）/启用/话题（datalist）/类型色标/属性（点大小 px、点云着色 Z/RGB/强度、透明度、TF 名签、幽灵、odom 轨迹长度、wrench 缩放）/状态；配置持久化 `rwl-viz-cfg-v5`（v4 自动迁移） |
| 3D 视口 | Grid、RobotModel（mesh 经 robot-mesh 路由 + **幽灵预览** + **TCP 示教拖拽** + DLS IK 伺服 + **工作空间包络线框**）、OccupancyGrid/**Costmap（RViz 蓝→红代价色带，/map+/global_costmap+/local_costmap 三层叠层）**、TF（**tf_buffer 全精度链路累积**）、PointCloud2（**多实例**；**圆点精灵 + Size(Pixels) + Z 轴彩虹/RGB/强度着色**，RViz2 AxisColor 语义）、LaserScan 3D（range 冷暖色）、Marker、Path、PoseArray/PoseStamped（箭头）、**Odometry（箭头+轨迹环）**、**Range（±fov 扇形）**、**Wrench（force 红/torque 蓝箭头）**、**PoseWithCovarianceStamped（95% 协方差椭圆）**——数据组按 `header.frame_id` 经 TF 摆位（RViz 帧语义）；HUD：场景状态/坐标图例/适配全部/网格/坐标轴 |
| 2D 视口 | Image（mono8/rgb8/bgr8，**滚轮缩放/拖拽平移/适配/1:1 + overlay（分辨率/fps/stamp 时延）**）+ **CompressedImage（createImageBitmap 实解 JPEG/PNG，与 raw 共卡）**、LaserScan 扇形、JointState 关节条（**镜像防环 v2** + **跟踪误差读数** + TCP HUD 毫米读数） |
| Teleop | WASD/方向键合键 + 线速/角速滑条 + 松键急停 + 停止钮 → `/cmd_vel`（TwistStamped，TB3 gz 桥约定）；**仅限仿真** |
| 字段绘图（rqt_plot） | 任意字段路径 + **计算字段**（`_computed_min/max/avg`）、暂停/时间窗、EMA Hz |
| 诊断面板（rqt_runtime_monitor） | DiagnosticArray 按级别三色列表（话题可配，默认 /diagnostics）；`__rwl_diag()` |
| ROS 日志面板（rqt_console） | /rosout 订阅、级别过滤（DEBUG+/INFO+/WARN+/ERROR+）、暂停/清空、上限 500；`__rwl_roslog()` |
| 参数面板 | 节点全量参数读取（get_parameters）+ 类型内联编辑（bool/num/str）+ set_parameters 写回；节点 datalist 来自连接图 |
| 节点图（图谱工作区） | rqt_graph 风格 SVG：**节点+话题 / 仅节点** 两种模式（仅节点时 publisher→subscriber 直连）+ 流动粒子 + Hz 徽标 + **服务/动作/调试话题过滤** + 搜索淡化 + 悬停邻域高亮 + 滚轮缩放/拖拽/fit + 统计；`window.__rwl_topology` 自动化入口 |
| 示教/轨迹面板 | 航点录制（90ms+4mm）/导出 JSON/**本地回放（段时长 80-900ms 钳制 + easeInOutQuad）**/**姿态预设（内置零位+用户存档，easeInOutCubic 850ms）**/**FollowJointTrajectory 真执行**（需官方控制器在环：TB3 `/joint_trajectory_controller`、panda `/panda_arm_controller`）+ 交互锁 guardedCall |
| 分割条 | 左右列间拖拽（30-80%），位置随布局快照持久化 |

web 写出（浏览器 → ROS，全官方标准话题）：`/clicked_point`（Publish Point 工具，RViz 语义）、
`/goal_pose`、`/initialpose`（2D 工具，官方 AMCL/Nav2 交互话题）、`/cmd_vel`（Teleop，
TwistStamped）。viz_fixture 删除后 web 无 joint_cmd 写出——动臂走
FollowJointTrajectory action（官方控制器在环）；armed 交互锁仅驱动本地模型动画
（镜像保持防 /joint_states 回读抖动）。
快照：`window.__rwl_viz_stats()`（含 fps/objectCount/vertexCount/odom/range/wrench/posecov/imagec）、
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
- 浏览器经 `/ws/foxglove` 看订阅、发布、服务和 action，对照节点日志。
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

2026-10-16 大数据稳定传输默认值（有界出站通道，见 architecture.md 同日轮）：

| 键 | 默认 | 说明 |
|---|---|---|
| `proxy.upstream_max_queue` | 4 | 上游腿 websockets 接收队列（消息条数）；小值让慢客户端背压更早传给桥 |
| `proxy.data_lane_bytes` | 4 MiB | 出站数据通道字节预算：二进制 messageData 帧超预算丢最旧（丢至 80% 滞回），JSON 控制帧永不丢；0=禁用（无界旧行为） |
| `proxy.egress_max_age_s` | 2.0 | 出口年龄门：推送前已早于该年龄的 messageData 帧直接丢；0=关闭 |

启动预检：CLI bind 失败即打印明确错误退出（launch respawn=true 下端口被占
不再静默死循环）。
