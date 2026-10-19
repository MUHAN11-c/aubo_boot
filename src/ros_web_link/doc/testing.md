# ros_web_link 测试流程（活文档）

**怎么测**：环境前置 → 分层命令 → 延迟验收数值 → 浏览器页面验证 → 清理纪律。
架构与测试矩阵见 [architecture.md](architecture.md)，端点与配置见 [io.md](io.md)；
本文只写可复制的流程与验收口径，不复制架构内容。

## 0. 被测对象与官方示例数据面

被测 = 本包传输链（JS transport → 网关 `/ws/foxglove` → ivg_foxglove_bridge → ROS 2 图）。
两种编排：
- **独立监控（2026-10-19 用户裁定，推荐）**：先按项目正常方式启动完整仿真，再
  `ros2 launch ros_web_link monitor.launch.py` 另起桥+网关；本包不拥有/不干预仿真；
- 自含演示栈（`sim_demo.launch.py` 等）仅用于一键复现，不改变"被测数据面来自官方项目"。

**数据面全部用 `src/examples` 官方标准项目**（2026-10-15 / 2026-10-19 用户裁定：
本地 peach 项目不参与验证）：

| 官方项目 | 来源 | 提供的数据流（经网关） |
|---|---|---|
| turtlebot3_gazebo（waffle_pi） | ROBOTIS-GIT/turtlebot3_simulations jazzy 分支，克隆于 `src/examples/turtlebot3_simulations` | `/camera/image_raw` 640×480 rgb8 @30Hz ≈0.92MB/帧、`/camera/image_raw/compressed`(JPEG)、`/scan` 360 点 @10Hz、`/odom` `/imu` `/tf` `/joint_states` `/robot_description` |
| turtlebot3_navigation2 | ROBOTIS-GIT/turtlebot3 jazzy 分支，官方包（src/examples/ symlink 构建） | 官方配套 `/map`(384²) + 官方 nav2 全参数 param/waffle_pi.yaml（AMCL 按 TB3 调参）；`rviz:=true` 时官方 RViz 随栈起 |
| nav2_bringup | 官方 apt | bringup_launch.py 载体；`/amcl_pose`、`/particle_cloud`、`/plan`、`/diagnostics`；action `/compute_path_to_pose` |
| examples_rclpy_pointcloud_publisher / minimal_publisher | ros2/examples，克隆于 `src/examples/ros2_examples` | `/test_cloud` 100×100 点云 @30Hz、`/topic`(String) |
| image_transport republish | 官方 apt | raw → compressed 转发链 |
| moveit_resources_panda_moveit_config | moveit/moveit_resources ros2 分支，克隆于 `src/examples/panda_moveit_config` | latched `/robot_description`、`/joint_states`(9 关节 @~60Hz)、`/monitored_planning_scene`、`/display_planned_path`；服务 `/compute_ik` `/plan_kinematic_path`；action `/panda_arm_controller/follow_joint_trajectory` |

组合入口：`ros2 launch ros_web_link sim_demo.launch.py`（TB3+nav2+examples+传输栈，一键；
`transport:=false` 时只起完整仿真，供独立 monitor 附着）、
`ros2 launch ros_web_link panda_moveit_web.launch.py`（MoveIt 栈 + 传输栈）。
合成大数据基准用本包 `comm_fixture -p rwl.bigdata:=true`（`/rwl_demo/big_image` 30Hz +
`/rwl_demo/big_cloud` 50k 点 10Hz）——合成源只用于受控延迟基准，真实流验收走官方仿真。

## 1. 环境前置（每轮开始前）

```bash
cd /path/to/aubo_e5_jazzy_ws     # 或设置 WS=<workspace> 后 cd "$WS"
# ① venv 完好（网关/测试必须 venv；系统 python 缺 websockets 会 skip）
./aubo_py3.12/bin/python -c 'import websockets, fastapi'
# ② 无残留进程（MUST：前清后清）
pgrep -af 'ros2 launch|gz sim|component_container|ivg_foxglove_bridge|ros_web_link'
# ③ 桥默认端口空闲（真机 8765 常被 preview_server 占；测试栈用随机端口不受影响）
ss -tlnp | grep 8765 || echo free
```

容器/沙箱环境还需 `/dev/shm`（FastDDS 共享内存，缺则 official 的 lifecycle
map_server 90s 不激活）与 `/dev/dri`（gz 渲染，缺则相机/点云无数据）。两者被拒时
请在宿主/完整权限 shell 下跑对应 E2E，不要改产品代码规避。

E2E 套件自起栈（`e2e_harness`）：随机 ROS 域（`free_domain()` 读 /proc 排除占用域）+
随机端口（`free_port()` 避开临时端口段）+ 唯一 `GZ_PARTITION`，测完 SIGINT 整组收尾并断言端口释放。
单次等待硬上限 30s（2026-10-15 用户约束）。

## 2. 分层命令与用例数

全部命令在工作区根执行；`-k` 过滤选套件（勿直接点 .py 路径）。③-⑦ 各套件
需 `PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ROS_DISTRO=jazzy` env 前缀（下文以 `$ROS_ENV` 代指；人手 shell 已 source install/setup.bash
则不需要）。**勿用 `not xxx` 反选当"单元跑法"**——会把全部 E2E 套件卷进来。

```bash
# ① 单元：路由/挂载/配置/有界通道语义（21）+ vitest 包装（1）
#    免 source 跑法须带 ROS env 前缀（ament_index_python/rclpy 在 /opt/ros/jazzy 下）
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
LD_LIBRARY_PATH=/opt/ros/jazzy/lib ROS_DISTRO=jazzy \
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k "gateway or js_units" -q

# ①b 独立包/API/示例/文档链接门（不依赖仿真；examples 用 comm fixture）
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
LD_LIBRARY_PATH=/opt/ros/jazzy/lib ROS_DISTRO=jazzy \
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test \
  -k "api_surface or examples or docs_links or package_independence" -q

# ② JS 纯核：协议帧/TF 数学/CDR 黄金字节/点云/数据面自动适配/barrel/色彩模式（135）
cd src/ros_web_link && node_modules/.bin/vitest run; cd ../..

# ③ E2E 通信矩阵：订阅/发布/服务/action/参数/TF/图谱/健康（11）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k comm_matrix -v

# ④ E2E 大数据延迟基准（合成 30Hz 大图+10Hz 点云，2）——§3 数值表
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k bigdata -v

# ⑤ E2E 官方仿真真实数据（TB3+nav2+examples，19）——含 §3 真实流延迟
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k sim -v

# ⑥ E2E MoveIt 官方 Panda（6）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k moveit -v

# ⑦ E2E 官方组件（官方 RSP/panda + map_server/lifecycle，7）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k official -v

# ⑧ E2E 独立监控附着（完整官方仿真正常启动 + 另起桥/网关，8；默认跳过，较慢）
RWL_FULL_SIM=1 ./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k full_sim -v
```

**完整仿真正常启动 + 独立监控（手工 E2E，等价 ⑧ 的编排）**：

```bash
# 终端 A：完整官方仿真（不带传输栈）
ros2 launch ros_web_link sim_demo.launch.py transport:=false
# 终端 B：独立监控（桥/网关，不干预仿真）
ros2 launch ros_web_link monitor.launch.py
# 浏览器：http://127.0.0.1:8095/（自动跳转 /console.html）
```

无 X/GLX 的 CI 机器给 `gz sim` 注入 `--headless-rendering`（`RWL_GZ_WRAPPER_DIR`
指向包装器目录；仅测试编排，不改变产品 launch）。

CDR 黄金字节 fixtures 升级 ROS 后重生成：
`./aubo_py3.12/bin/python src/ros_web_link/test/generate_viz_golden.py`。

## 3. 大数据延迟验收数值表（单一权威表）

「稳定传输没有延迟」的语义：**快消费者拿到满速率且帧新鲜；慢/停顿消费者丢帧而不积压**
（丢帧是 BEST_EFFORT 视觉流的正确降级，积压不是）。帧年龄 = 本地墙钟 − logTime（桥接收时刻）。

| 流 | 名义率 | 到达率门 | 帧年龄门 | 依据用例 |
|---|---|---|---|---|
| 合成 `/rwl_demo/big_image` 0.92MB | 30Hz | ≥19.5fps(65%) | mean<200ms / max<600ms | `test_bigdata_fast_client_throughput_and_freshness` |
| 合成 `/rwl_demo/big_cloud` 50k 点 | 10Hz | ≥6.5fps(65%) | 同上 | 同上 |
| 快客户端网关丢帧率 | — | <5% | — | 同上 |
| 停顿 4s 恢复（合成图流） | — | 停顿窗送达 <60 帧（无界≈120） | 恢复首帧 <1.5s；droppedData 递增 | `test_bigdata_stall_recovery_bounded_backlog_and_fresh_resume` |
| 真实 `/camera/image_raw`（gz 相机） | 30Hz 声明（实测无节流 ~4.7kHz/5GB/s，2026-10-16） | ≥15fps(50%，容忍 gz 实时因子) | mean<300ms / max<1.0s（实测 5.1ms） | `test_tb3_camera_stream_rate_and_freshness` |
| 真实 `/test_cloud`（官方点云） | 30Hz | ≥18fps(60%)（实测 29.9fps） | mean<300ms / max<1.0s（实测 3.8ms） | `test_tb3_pointcloud_stream_rate_and_freshness` |

配套观测面：`GET /api/ivg/ws/stats`（连接级/进程累计 forwarded/dropped/droppedStale/queued）；
console 监控区「网关丢帧 / 网关积压」卡。
2026-10-16 实测基线（sim_demo 栈，8s 裸 WS）：相机 754.8fps/5.1ms、点云 29.9fps/3.8ms、
压缩图 15.0fps/3.6ms；浏览器压缩流订阅 0 丢帧 0 积压。

## 4. 浏览器页面验证（人工/自动化）

```bash
# 推荐：完整仿真 + 独立监控（两终端）
ros2 launch ros_web_link sim_demo.launch.py transport:=false
ros2 launch ros_web_link monitor.launch.py
# 浏览器 http://127.0.0.1:8095/ → /console.html（自动连接；点「运行自检」）
# 浏览器示例 http://127.0.0.1:8095/examples/minimal_client.html
# 主题：白色二次元商业风格（LIVE STUDIO / 樱粉-星紫渐变 / 圆角卡片）；
# 首次连接无用户布局时按图上话题自动适配 Displays（官方 TB3 的压缩图/test_cloud
# 与任意仿真的 RGB/腕载点云都识别）。
```

核对项（自动化钩子同名）：

1. **自检 9 项**全绿：`await window.__rwlSelfTestRun()`（`window.__rwl_selftest`）——
   覆盖连接/订阅/发布/服务/action(精确 cancel)/参数/图谱；8 项依赖 comm_fixture
   （同域 `python3 -m ros_web_link.comm_fixture` 配套，无 fixture 时按设计 SKIP）。
2. **Displays 真实渲染**（默认 topic 已指向官方数据面）：压缩图(/camera/image_raw/compressed
   ，2026-10-16 起默认——原始流默认关，gz 无头相机不节流会灌满桥缓冲饿死服务响应，
   见 architecture.md)、PointCloud2(/test_cloud)、OccupancyGrid(/map)、LaserScan3D(/scan)、
   RobotModel(mesh 经 robot-mesh 服务，**以 mesh 请求 200 为硬证据**——视觉分析会把
   点云簇误读为机器人本体)、TF；点云应为**小圆点 + Z 轴彩虹**（RViz2 Size(Pixels)
   语义，不应出现大蓝方块），`window.__rwl_viz_stats().cloud` 应含
   `colorBy:'axis'`/`sizePx:3`/`points≈10k`；自动化经 `window.__rwl_viz`。
3. **渲染与传输健康**：`window.__rwl_viz_stats`（FPS/点数）、监控卡「网关丢帧/网关积压」
   应为 0（快消费者）；图像/点云 stamp 时延（rviz Delay 语义）秒级以内。
4. **浏览器 console 无错误**；改过 JS 后必须**新开标签**（IAB 模块缓存不重验证，带 `?v=`）。
5. **图谱工作区**：点顶部「图谱」，核对 `#topo-stats`（节点/话题/服务或仅节点连线）、
   方向粒子、服务/动作/隐藏调试过滤、搜索淡化、悬停邻域高亮、适配按钮；
   切换「节点+话题 / 仅节点」，仅节点模式不应残留话题节点或旧粒子；
   `window.__rwl_topology` 为自动化入口。
6. **RViz 参考细节**：Displays 行有类型色标；悬停行出现 `⧉` 复制；复制点云后
   实例数 +1 且两路独立订阅。
7. 交互闭环抽测：发布 `/initialpose` → `/particle_cloud` 出流；teleop W 键 → `/odom` 位移。

截图存证放 `gui-test-screenshots/`；参考窗口（RViz2 / Gazebo / rqt_graph）在
`gui-test-screenshots/reference/`。注意 `rqt_graph` 启动后若画布空白，点工具栏左侧
「Refresh ROS graph」；`rqt_plot` 的 `QLayout::removeWidget` 是上游 Qt 警告，不影响绘图。

## 5. 验收口径

- **隔离跑全绿**（②-⑦ 逐套件单独跑）是必要条件；
- **全量连跑至少一次全绿**（机器高负载会轮转抖动，见 architecture.md 决策记录——
  高负载下 moveit 执行偶发 accepted=False 需退避重试，单套件恒绿）；
- 延迟数值以 §3 表为准；回归时先看 `/api/ivg/ws/stats` 再定位（网关 lane / 桥 send buffer / 内核套接字）。

## 6. 清理纪律（MUST，测完即停）

```bash
# 任何演示栈/浏览器验证结束后：
# launch 终端 Ctrl+C；一次性命令带 timeout；挂死按 PID：kill -TERM <pid>，2s 后 kill -9
pgrep -af 'ros2 launch|gz sim|component_container|ivg_foxglove_bridge|ros_web_link'
```

复核清零才收工。上一轮栈残留（桥+网关+fixture 常驻发流）会占 CPU、污染延迟测量，
并占 FastDDS SHM（`/dev/shm/fastrtps_*`；反复起栈累积时先停全部 ROS 进程再清）。

## 7. 环境坑速查（详细根因见 architecture.md 决策记录）

- 桥直 exec 须补 `ROS_DISTRO=jazzy` 与桥包 lib 目录进 `LD_LIBRARY_PATH`。
- MoveIt 节点 env 须含 `/opt/ros/jazzy/opt/*/lib` 全链 + `/opt/ros/jazzy/lib/x86_64-linux-gnu`。
- 多 gz 仿真共存必须互异 `GZ_PARTITION`；TB3 出生位姿必须显式 (-2.0,-0.5)（中心柱卡死）。
- 本机 curl 需 `--noproxy '*'`；agent Bash 环境免 source 用 env 前缀。
- ament_python 包不用 `--symlink-install` 重装（egg-link 丢 dist-info）。
