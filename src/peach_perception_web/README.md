# peach_perception_web

## 简介

本包是 `peach_pose_ros2`、`peach_reconstruction_ros2` 与
`peach_approach_grasp` 的只读数据监控台。
图像、Marker 和点云继续在 RViz2 查看；Web 页面不订阅、不转换、不传输
`sensor_msgs/Image` 或 `PointCloud2`，只展示算法和任务状态数据。

重点显示：

- 感知 selected ID、重建 target ID、refined target ID 的一致性；
- 固定目标数量、优先级、稳定 ID、跟踪状态和采摘状态；
- 初始抓取入口 XYZ、四元数、方向、袋底、袋颈、直径和建议行程；
- 有效深度、轴线置信度、内点比例、误差和诊断标志；
- 策略、模型、内外参和刀具版本；
- 重建状态、视角数、TF 延迟、目标中心和掩膜缓存；
- FK/ICP 模式、fitness、RMSE、平移及旋转修正量；
- TSDF 模型点数、积分帧、体素、耗时和 ROI 数值摘要；
- refined 入口/轴线、最终拟合质量和 `grasp_decision.allowed`；
- 主动靠近阶段、执行/arm 状态、视角覆盖和最终抓取编排状态；
- 各 ROS 数据话题的新鲜度及完整原始 JSON 快照。

页面没有服务客户端、Action 客户端或运动发布器，不能启动机械臂、切换目标、
finalize 或触发抓取。`allowed=true` 也只表示视觉质量门通过。

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
| `*_topic` | 见 YAML | 结构化数据输入话题，可按部署重映射 |

图像和三维效果仍按以下方式查看：

- RViz2 Image：感知调试图，或用 `rqt_image_view` 查看
  `/peach_pose_node/debug_image`；
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
  ├─ target_observations ──> 目标表、初始位姿、质量、版本
  └─ harvest_state ────────> run ID、锁定计划、完成集合

peach_reconstruction_ros2
  ├─ status / diagnostics ─> 状态、TF、FK/ICP、TSDF 数值摘要
  ├─ grasp_decision ───────> 视觉抓取许可和拒绝原因
  └─ refined 三件套 ──────> 最终入口、轴线和拟合质量

peach_approach_grasp
  └─ status ──────────────> 主动视点、质量门和抓取状态机

DashboardState（线程安全最近值）
  └─ GET /api/state ───────> 浏览器 500 ms 刷新数据仪表盘
```

浏览器将三个目标 ID 显式串联：

```text
perception.selected_target_id
  == reconstruction.diagnostics.target_id
  == refined_pose.candidates[0].target_id
```

前两级不一致时页面立即标红并提示停止融合；refined 尚未发布时显示“等待
finalize”，不会误判为不一致。该检查是现场可见性保护，不会替代上层任务管理器的
硬门控。

数据新鲜度按网关收到各话题的墙钟时间计算：小于 3 秒为绿色，3～15 秒为橙色，
超过 15 秒或从未收到为红色。重建和 refined 话题是 transient-local 低频输出，
静态 READY 结果变红仅表示长时间未更新，应结合状态解释。

## 软件框架

| 文件 | 职责 |
|---|---|
| `gateway.py` | ROS 订阅、线程安全状态缓存、只读 HTTP 路由与生命周期 |
| `codec.py` | 目标消息、初始/refined 结果和 JSON String 的结构化转换 |
| `config/web.yaml` | 端口及结构化输入话题的权威默认值 |
| `launch/peach_perception_web.launch.py` | 默认加载 YAML；显式 host/port 才覆盖 |
| `web/index.html` | 数据审计台语义结构 |
| `web/app.css` | 高对比、响应式现场数据布局 |
| `web/app.js` | 500 ms 状态轮询、ID 不变量和数据字段渲染 |

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

HTTP API 只有两个只读入口：

| 路径 | 返回 |
|---|---|
| `/api/state` | 目标、重建、refined 和数据新鲜度 JSON |
| `/` | 随包安装的数据仪表盘 HTML/CSS/JavaScript |

本包使用系统 Python 的标准 console_scripts 入口，不依赖 torch、open3d、OpenCV、
numpy、WebGL、npm、CDN 或互联网连接。
