# peach_perception_web

## 简介

本包是 `peach_pose_ros2`、`peach_reconstruction_ros2`、
`peach_approach_grasp` 与 `peach_harvest_orchestrator` 的**只读数据监控台**
（2026-08-13 起取消全部 Web 写入口：控制/调试/策略下发/参数档案均已移除，
测试与运行一律走自动全流程，问题定位依靠本页过程监测）。
图像、Marker 和点云继续在 RViz2 查看；Web 页面不订阅、不转换、不传输
`sensor_msgs/Image` 或 `PointCloud2`，只展示算法和任务状态数据。

重点显示（流程反馈驾驶舱，自上而下四个区域）：

- **流程反馈区**：批次宏观过程线（就绪→拍照位姿→感知锁定→观察规划→
  质量验证→靠近抓取→工具动作→撤离收尾→完成/复扫，当前环节高亮脉冲、
  已通过绿色勾、故障红色，复扫显示「第 N/M 轮」）、当前目标周期条
  （target_id / cycle_id / 阶段中文名 / 阶段耗时 / 批次进度条）、
  编排器事件时间线（severity 色点 + code + message + target_id，新事件在顶）；
- **调度策略区**：锁定目标总数 / 已抓 / 待抓计数与已抓 ID 集合、当前选中、
  目标表（ID/优先级/采摘状态/跟踪状态/置信度/距离/诊断标志）、
  策略开关徽标（auto_start/execution/grasp/tool）与未就绪 blockers 红条；
- **节点状态区**：感知 / 重建 / 靠近抓取 / 编排器 / 机械臂五张卡片，
  各带话题新鲜度（>5s 黄、>15s 红、从未收到灰）；
- **性能参数区**：主机 CPU/内存/load、GPU 利用率与显存（无 GPU 时显示
  「无 GPU 数据」）、关键 ROS 进程 CPU/RSS（psutil 按 cmdline 匹配）、
  链路关键耗时（TF 延迟、TSDF 积分耗时等，来自既有诊断 JSON）、
  可折叠的当前参数只读镜像与原始 JSON 快照。

除页面展示外，节点还把监控数据分类落盘（只读：订阅 + 写文件，无任何 ROS
写入口；`record.enabled: false` 可整体关闭）。页面顶部状态栏显示当前记录
目录，不在页面显示图像/点云内容。

目录归属按**编排器批次执行期**划分：`batch_state` 进入 DISCOVERY/RUNNING
（或节点启动后见到的首个活动批次）开 `run_<时间戳>/`，批次终局
（COMPLETED/INTERRUPTED/RECOVERY_REQUIRED）关闭并生成 summary；
同一批次的所有复扫轮次在同一文件夹内（轮次由事件/状态自带，不拆目录）；
无批次期间的记录进 `idle_<时间戳>/`（节点启动时创建，首个批次开始后关闭，
批次结束后再开新的）。目录结构：

```text
web_runs/                        # record.root_dir（相对节点 CWD）
└── run_<yyyyMMdd_HHmmss>/       # 一次批次一个文件夹
    ├── events.jsonl             # HarvestEvent 全量，一行一条（带 recorded_at 墙钟）
    ├── state.jsonl              # HarvestState 按 revision 去重
    ├── perception.jsonl         # 目标快照逐帧全量（剔除 mask，保留 candidate/
    │                            # fitting 摘要）+ harvest_state 变化并入（kind 区分）
    ├── reconstruction.jsonl     # 重建 status/diagnostics/grasp_decision 每次更新
    ├── approach.jsonl           # 靠近抓取 status 每次更新
    ├── metrics.jsonl            # 性能采样每条一行（metrics_period_s 默认 1.0s）
    ├── images/                  # 逐帧调试图 JPEG q85（img_<序号>_<时间戳>.jpg）+
    │                            # image_index.jsonl（事件码+target_id+最近图像名）
    ├── clouds/                  # target 终局时刻的 TSDF 点云
    │                            # （手写 binary_little_endian PLY，xyz+rgb）
    ├── summary.csv              # 逐目标 outcome 表（派发/终局时刻、耗时、原因）
    └── summary.md               # 批次概览 + 每阶段耗时 + 事件/感知/重建/性能统计
```

`summary.md` 统计项：批次概览（run_id/终局/复扫轮数/起止墙钟/总时长/各
outcome 计数）、逐目标 outcome 表、每阶段耗时统计（从 state.jsonl 的
target_phase 跃迁推导，按目标周期分列 OBSERVING~COMPLETING 各阶段秒数）、
事件统计（按 code/severity 计数）、感知统计（帧间隔中位数/FPS 估算、目标数
变化范围）、重建关键指标终值（captured_views/baseline/RMSE 门等，取
reconstruction.jsonl 最后一条 diagnostics）、运行性能统计（CPU/内存/GPU 均值
峰值）。summary 在批次终局生成，节点关闭时若批次未结也补一份；所有盘写走
独立守护线程 + 队列，jsonl 按路径缓冲批量/空闲间隔 flush，单批写盘失败只
降级告警不阻断采集，不阻塞 ROS 回调与 HTTP 线程。

页面为纯只读监控：不提供任何控制/调试/参数写入 HTTP 入口；自动全流程由
编排器闭环，现场只通过本页定位「当前跑到哪一步、该步数据是什么」。
`allowed=true` 也只表示视觉质量门通过；上电与安全恢复始终由现场人员完成。

完整视觉链路见
[桃子首帧感知与连续局部重建联动说明](../../docs/peach_pose_reconstruction_integration.md)。

## 使用方法

构建并启动：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_pose_msgs peach_perception_web
source install/setup.bash
ros2 launch peach_perception_web peach_perception_web.launch.py
```

浏览器访问 `http://127.0.0.1:8090`。

现场平板或局域网电脑访问：

```bash
ros2 launch peach_perception_web peach_perception_web.launch.py \
  host:=0.0.0.0 port:=8090
```

然后访问 `http://<机器人电脑IP>:8090`。服务没有登录鉴权，只应暴露在可信采摘
局域网，不要映射到公网。

launch 默认加载 `config/web.yaml`：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `host` | `127.0.0.1` | HTTP 监听地址 |
| `port` | `8090` | HTTP 监听端口 |
| `param_poll_period_s` | `3.0` | 参数镜像轮询周期（秒） |
| `event_buffer_size` | `100` | 事件时间线环形缓冲上限（条） |
| `metrics_period_s` | `1.0` | 系统/GPU/进程性能采样周期（秒） |
| `metrics_process_patterns` | 见 YAML | 进程监控的 cmdline 子串匹配关键字 |
| `record.enabled` | `true` | 监控数据分类落盘总开关 |
| `record.root_dir` | `web_runs` | 记录根目录（相对节点 CWD；一批次一目录） |
| `record.save_images` | `true` | 逐帧保存调试图 JPEG q85 |
| `record.save_clouds` | `true` | target 终局时刻保存 TSDF 点云 PLY |
| `*_topic` | 见 YAML | 结构化数据输入话题，可按部署重映射 |

图像和三维效果仍按以下方式查看：

- RViz2 Image：感知调试图，或用 `rqt_image_view` 查看
  `/peach/perception/debug_image`；
- RViz2 PointCloud2：`/peach/reconstruction/local_cloud`；
- RViz2 PointCloud2：`/peach/reconstruction/tsdf_cloud`；
- RViz2 MarkerArray：两包的 `/peach/perception/markers` 与
  `/peach/reconstruction/markers`。

测试：

```bash
cd src/peach_perception_web
../../aubo_py3.12/bin/python -m pytest test/ -q
```

## 执行逻辑

```text
peach_pose_ros2
  ├─ target_observations ──> 目标表、调度计划计数、选中目标
  └─ harvest_state ────────> run ID、锁定计划、已抓集合

peach_reconstruction_ros2
  ├─ status / diagnostics ─> 重建卡片状态、视角数、关键耗时
  └─ grasp_decision ───────> （原始快照保留）

peach_approach_grasp
  └─ status ──────────────> 靠近抓取卡片（周期状态/说明/执行 arm）

peach_harvest_orchestrator
  ├─ state ───────────────> 批次过程线、周期条、策略徽标、blockers
  └─ events ──────────────> 事件时间线（环形缓冲最近 100 条）

aubo_io_controller
  └─ robot_status ────────> 机械臂卡片（上电/可运动/急停/错误）

MetricsSampler（独立线程 2.5s）
  └─ psutil + nvidia-smi ─> 主机/GPU/进程性能区

DashboardState（线程安全最近值 + 事件环形缓冲）
  └─ GET /api/state ───────> 浏览器 1 s 轮询渲染驾驶舱
```

数据新鲜度按网关收到各话题的墙钟时间计算：小于 5 秒为绿色，5～15 秒为黄色，
超过 15 秒或从未收到为红色/灰色。重建和 refined 话题是 transient-local 低频输出，
静态 READY 结果变红仅表示长时间未更新，应结合状态解释。

## 软件框架

| 文件 | 职责 |
|---|---|
| `gateway.py` | ROS 节点：只读订阅、参数镜像轮询与 HttpBackend 窄接口组装 |
| `state.py` | DashboardState：线程安全最新值缓存 + 事件环形缓冲（含参数镜像） |
| `metrics.py` | MetricsSampler：独立线程采 CPU/内存/load/GPU/进程，失败降级 |
| `recorder.py` | Recorder：批次期单目录分类落盘（jsonl/JPEG/PLY/summary 统计），队列+守护写线程 |
| `http_server.py` | 零 ROS 的只读 HTTP 层：GET Handler 与 start_http（可 fake 后端单测） |
| `codec.py` | 目标/事件/机械臂状态消息与 JSON String 的结构化转换 |
| `config/web.yaml` | 端口、轮询/采样周期、事件缓冲与输入话题的权威默认值 |
| `launch/peach_perception_web.launch.py` | 默认加载 YAML；显式 host/port 才覆盖 |
| `web/index.html` | 流程反馈/调度策略/节点状态/性能参数四区域语义结构 |
| `web/app.css` | 深色工业风、状态色（绿/黄/红/灰）与响应式布局 |
| `web/app.js` | 1 s 状态轮询与四区域渲染 |

订阅接口：

| 话题 | 类型 |
|---|---|
| `/peach/perception/target_observations` | `PeachTargetObservationArray` |
| `/peach/perception/harvest_state` | `std_msgs/String` JSON |
| `/peach/reconstruction/status` | `std_msgs/String` |
| `/peach/reconstruction/diagnostics` | `std_msgs/String` JSON |
| `/peach/reconstruction/grasp_decision` | `std_msgs/String` JSON |
| `/peach/reconstruction/refined_pose` | `BagGraspCandidateArray` |
| `/peach/reconstruction/refined_axis` | `geometry_msgs/Vector3Stamped` |
| `/peach/reconstruction/refined_diagnostics` | `BagFittingArray` |
| `/peach_approach_grasp_node/status` | `std_msgs/String` JSON |
| `/peach_harvest_orchestrator/state` | `peach_harvest_msgs/HarvestState` |
| `/peach_harvest_orchestrator/events` | `peach_harvest_msgs/HarvestEvent` |
| `/aubo_io_controller/robot_status` | `aubo_msgs/RobotStatus` |
| `/peach/perception/debug_image` | `sensor_msgs/Image`（bgr8，仅记录器） |
| `/peach/reconstruction/tsdf_cloud` | `sensor_msgs/PointCloud2`（latched，仅记录器） |

HTTP API：

| 路径 | 方法 | 返回/作用 |
|---|---|---|
| `/api/state` | GET | 流程/调度/节点/性能/参数镜像与数据新鲜度单 JSON 快照 |
| `/` | GET | 随包安装的驾驶舱 HTML/CSS/JavaScript |

一切 POST 统一 405（只读监控台，无写入口）。

本包使用系统 Python 的标准 console_scripts 入口，不依赖 torch、open3d、OpenCV、
numpy、WebGL、npm、CDN 或互联网连接。
