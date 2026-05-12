# IVG2.0 软件架构设计

> 面向未来的机械臂操作系统：视觉抓取 + Web 控制面板 + ML/VLA/AI（后续方向）
>
> **本文档定位**：前瞻性架构蓝图。当前项目功能正常稳定，本文档为后续 ML/AI 方向扩展提前规划技术路线，确保技术选型不会落后、无需二次重学。分阶段实施计划将根据需要另行制定。

## 现状 vs 目标

| 维度 | 当前（2026.05 可用） | 目标架构（未来） |
|------|---------------------|-----------------|
| Web 后端 | FastAPI + Flask 并存 | FastAPI 统一 |
| 前端 | 原生 JS + Web Components | Vue 3 + TypeScript |
| 重复代码 | 7 处四元数转换、3 个 YOLO 副本 | 统一到 ivg_utils |
| 单体文件 | hand_eye_calibration_node.py 5958 行 | 拆分为 6 个模块 |
| MoveIt 2 | 本地源码复刻 | apt 标准包 |
| 包命名 | demo_*, interface 不一致 | ivg_* 统一前缀 |
| 部署 | 手动 apt/pip 安装 | Docker Compose（后续参考）|
| 容器化 | 无 | Docker 多阶段构建（后续参考）|
| 测试/CI | 几乎为零 | pytest + GitHub Actions（后续参考）|
| 硬编码路径 | /home/mu/IVG2.0 | 相对路径 + ament_index |
| 前端包管理 | importmap + 自构建 | Vite + npm |
| ML 实验追踪 | 无 | MLflow + DVC（后续参考）|
| VLA 基座 | 无 | OpenVLA + LeRobot（后续参考）|
| LLM 推理 | 无 | vLLM + llama.cpp（后续参考）|

---

## 一、分层架构

```
┌─────────────────────────────────────────────────────────────┐
│                     Layer 6: Web & UI                       │
│  ┌──────────────┐  ┌───────────────────────────────────┐    │
│  │ FastAPI 网关  │  │  Vue 3 + TypeScript + Pinia       │    │
│  │ (反向代理+BFF)│  │  (Vite 构建, ROSLIB.js 通信)      │    │
│  └──────┬───────┘  └──────────────┬────────────────────┘    │
│         │        HTTP/WS          │                         │
├─────────┼─────────────────────────┼─────────────────────────┤
│         │                         │                         │
│  ┌──────┴─────────────────────────┴──────────────────┐      │
│  │             Layer 5: Application                   │      │
│  │  ┌──────────────┐  ┌────────────┐  ┌───────────┐  │      │
│  │  │ 抓取执行引擎  │  │ 工具快换   │  │ 应用演示   │  │      │
│  │  │ (grasp worker)│  │ (swap/sync)│  │ (latte等)  │  │      │
│  │  └──────┬───────┘  └─────┬──────┘  └─────┬─────┘  │      │
│  └─────────┼────────────────┼──────────────┼────────┘      │
│            │   ROS 2 srv    │              │                │
├────────────┼────────────────┼──────────────┼────────────────┤
│            │                │              │                │
│  ┌─────────┴────────────────┴──────────────┴──────────┐    │
│  │               Layer 4: Perception                   │    │
│  │  ┌────────────┐ ┌───────────┐ ┌──────────────────┐  │    │
│  │  │ 位姿估计    │ │ 抓取预测   │ │ 目标检测+跟踪    │  │    │
│  │  │ (VPE)       │ │ (GraspNet) │ │ (YOLO OBB)       │  │    │
│  │  │ FastAPI+ROS │ │ PyTorch    │ │ Ultralytics      │  │    │
│  │  └────────────┘ └───────────┘ └──────────────────┘  │    │
│  │  ┌──────────────────────────────────────────────┐    │    │
│  │  │         手眼标定 (FastAPI + ROS)              │    │    │
│  │  └──────────────────────────────────────────────┘    │    │
│  └──────────────────────┬───────────────────────────────┘    │
│                         │ ROS 2 srv/topic                   │
├─────────────────────────┼───────────────────────────────────┤
│                         │                                    │
│  ┌──────────────────────┴──────────────────────────────┐    │
│  │            Layer 3: Core Services                   │    │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────┐  │    │
│  │  │ 轨迹规划  │ │ 轨迹执行  │ │ 状态查询  │ │ IO控制 │  │    │
│  │  │ (MoveIt)  │ │ (action)  │ │ (service) │ │(srv)  │  │    │
│  │  └──────────┘ └──────────┘ └──────────┘ └───────┘  │    │
│  │  ┌──────────────────────────────────────────────┐    │    │
│  │  │    工具管理 (快换 + PlanningScene 同步)       │    │    │
│  │  └──────────────────────────────────────────────┘    │    │
│  └──────────────────────┬───────────────────────────────┘    │
│                         │ ROS 2 action/srv                  │
├─────────────────────────┼───────────────────────────────────┤
│                         │                                    │
│  ┌──────────────────────┴──────────────────────────────┐    │
│  │           Layer 2: Hardware Interface               │    │
│  │  ┌────────────────┐  ┌────────────┐  ┌──────────┐   │    │
│  │  │ 轨迹控制器      │  │ 状态广播器  │  │ Dashboard │   │    │
│  │  │ (TCP2CAN流式)   │  │ (200Hz回调) │  │ (18+服务)  │   │    │
│  │  └───────┬────────┘  └─────┬──────┘  └─────┬────┘   │    │
│  │          │                 │               │         │    │
│  │     ┌────┴─────────────────┴───────────────┴───┐     │    │
│  │     │         AUBO SDK (双 TCP 连接)            │     │    │
│  │     │  conn_control_     │    conn_status_       │     │    │
│  │     │  (TCP2CAN + RIB)   │    (状态查询 + IO)     │     │    │
│  │     └────────────────────┬──────────────────────┘     │    │
│  │                          │                            │    │
│  │  ┌───────────────────────┴──────────────────────┐     │    │
│  │  │         相机驱动 (Percipio FM830)              │     │    │
│  │  └──────────────────────────────────────────────┘     │    │
│  └───────────────────────────────────────────────────────┘    │
│                         │                                     │
│                    ┌────┴────┐                                │
│                    │ 机械臂   │                                │
│                    │ 相机     │                                │
│                    └─────────┘                                │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────────────────────────────────────────┐    │
│  │              Layer 0: 基础设施                        │    │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐  │    │
│  │  │ ivg_utils│ │ Docker   │ │ rosbag2  │ │ CI/CD  │  │    │
│  │  │ (共享库)  │ │(后续参考)│ │ (数据录制)│ │(后续参考)│  │    │
│  │  └──────────┘ └──────────┘ └──────────┘ └────────┘  │    │
│  │  ┌────────────────────────────────────────────────┐  │    │
│  │  │        ivg_bringup (启动编排 + 全局配置)         │  │    │
│  │  └────────────────────────────────────────────────┘  │    │
│  └──────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

---

## 二、各层职责与通信契约

### Layer 0：基础设施

| 组件 | 职责 | 被谁依赖 |
|------|------|---------|
| **ivg_utils** | 跨包共享：数学变换、IO 常量、TF 缓存、单位转换、位姿工具 | 全部 Python 包 |
| **ivg_bringup** | 统一启动/停止脚本、全局默认配置（YAML）、健康检查 | 运维 |
| **rosbag2** | 全话题录制，会话级数据回溯 | 调试 + 回归测试 |
| **Docker Compose**（后续参考）| 一键部署：开发/仿真/真机三套 profile | 开发 + 部署 |
| **CI/CD**（后续参考）| GitHub Actions：lint + test + build + rosbag 回归 | 开发 |

### Layer 2：硬件接口

| 节点 | 技术栈 | 对外接口 | 职责 |
|------|--------|---------|------|
| `joint_trajectory_controller` | C++ / AUBO SDK | Action: `/follow_joint_trajectory` | 五次多项式插值，TCP2CAN 流式发送 |
| `aubo_state_broadcaster` | C++ / AUBO SDK | Topic: `/joint_states` (200Hz), `/io_states`, `/robot_status` | SDK 回调驱动状态发布 |
| `aubo_dashboard` | C++ / LifecycleNode | 18+ Service: IK/FK/IO/Move | 机械臂基础能力封装 |
| `percipio_camera` | C++ | Topic: `/camera/*/image_raw`, `/camera/*/points` | 知微 FM830 深度相机驱动 |

**设计决策**：
- SDK 双连接架构不变 — `conn_control_` (TCP2CAN+RIB) 和 `conn_status_` (状态+IO) 已验证稳定
- SDK 调用隔离在独立线程，ROS 2 回调仅做 atomic 写入 — 当前模式正确，不改动
- Dashboard 保持 LifecycleNode — `Unconfigured→Inactive→Active` 状态机是 ROS 2 最佳实践
- 相机驱动 Percipio SDK C++，输出深度+彩色+点云三路话题
- **技术栈量化**：PyTorch 学术论文 55%+ 占比 / 158k Stars；AUBO SDK 厂商专用，同步阻塞模式通过独立线程隔离

### Layer 3：核心服务

| 节点 | 对外接口 | 职责 |
|------|---------|------|
| `move_group` (MoveIt 2 apt) | Action: `FollowJointTrajectory`; Service: `compute_ik`, `plan_kinematic_path`, `get_planning_scene` | 运动规划核心 |
| `plan_trajectory_server` | Service: `/plan_trajectory` | MoveIt 规划包装 |
| `execute_trajectory_server` | Service: `/execute_trajectory` | 轨迹执行 |
| `get_current_state_server` | Service: `/get_current_state` | 当前机器人状态 |
| `set_speed_factor_server` | Service: `/set_speed_factor` | 速度比例调整 |
| `set_robot_pose_server` | Service: `/move_to_pose` | 点到点运动 |
| `gripper_swap_worker` | Service: `/change_tool`, `/run_gripper_swap`, `/get_current_tool` | 物理快换 + IO 控制 |
| `scene_attach_worker` | Topic: `/planning_scene` (diff); Param: `/robot_description` | PlanningScene 同步 + URDF 更新 |

**接口统一化**：当前 `demo_interface` 和 `tool_changer_interface` 有职责重叠（都定义 IO/运动消息），重构后合并为一个统一的 `ivg_service_interfaces` 包，包含所有应用层接口定义。

### Layer 4：感知

| 节点 | 技术栈 | 对外接口 | 职责 |
|------|--------|---------|------|
| `ivg_vpe_node` | Python / OpenCV / rembg / FastAPI | Service: `/estimate_pose`, `/list_templates` | 模板匹配姿态估计 |
| `ivg_graspnet_node` | Python / PyTorch / pointnet2 | Topic: `/grasp_poses_base` | 6-DoF 抓取位姿预测 |
| `ivg_yolo_obb_node` | Python / Ultralytics | Topic: 检测结果 + MarkerArray | 旋转框目标检测 |
| `ivg_yolo_track_node` | Python / Ultralytics | Topic: 跟踪结果 | 特定目标跟踪 |
| `ivg_hand_eye_node` | Python / OpenCV / FastAPI | Service: `/compute_hand_eye`, `/collect_calibration_data` | 手眼标定 |

**关键重构**：
- YOLO 三个节点提取共享基类 `YoloBaseNode`（模型加载、CV Bridge、结果发布）
- VPE 的 `ros2_communication.py` 拆分为采集/预处理/特征提取/模板匹配/标定加载 5 个独立模块
- 手眼标定的 Flask → FastAPI，分离 ROS 节点与 Web 服务
- 所有 `_quaternion_to_rotation_matrix` 私有副本 → 统一调用 `ivg_utils.math`

### Layer 5：应用

| 节点 | 对外接口 | 职责 |
|------|---------|------|
| `execute_grasp_pose_worker` | Service: `/execute_single_grasp` | 单次抓取原语：接近→握持→抬升→放置 |
| `publish_grasps_client_worker` | Topic: `/grasp_poses_base` (sub); Service: `/loop_grasp_control` | GraspNet 循环抓取编排 |
| `coffee_latte_service` | Service: `/set_latte_do2`, `/set_latte_do4` | 咖啡拉花 IO 控制 |
| `latte_trajectory_player` | Action: `/follow_joint_trajectory` | 示教轨迹回放 |

**设计决策**：
- `execute_grasp_pose_worker` 的 Reentrant callback group 模式保留 — 已验证避免死锁
- `publish_grasps_client_worker` 和 `publish_grasps_AB.cpp` 合并 — 消除 80% 代码重复
- IO 引脚语义不一致（`true=打开` vs `true=闭合`）统一为 `ivg_utils.robot_constants` 中的命名常量

### Layer 6：Web & UI

| 组件 | 技术栈 | 职责 |
|------|--------|------|
| **ivg_web_gateway** | FastAPI + uvicorn + httpx + websockets | 统一入口：静态文件服务 + rosbridge WS 代理 + MJPEG 视频代理 + BFF API + 安全头 |
| **ivg_web_frontend** | Vue 3 + TypeScript + Pinia + Vite + ROSLIB.js | 6 个面板：门户/视觉抓取/咖啡拉花/TF监控/设置/日志 |

**网关路由设计**：
```
网关 (0.0.0.0:8090)
├── /index.html                        → 静态文件 (Vue 3 SPA)
├── /assets/*                          → 打包资源 (Vite 产物)
├── /ws/rosbridge                      → WebSocket 代理 → 127.0.0.1:9090 (rosbridge)
├── /api/ivg/proxy/web-video/*        → HTTP 代理 → 127.0.0.1:8089 (web_video_server)
├── /api/ivg/proxy/calibration/*      → HTTP 代理 → 127.0.0.1:8091 (手眼标定 FastAPI)
├── /api/v1/runtime                    → BFF: 运行时配置
├── /api/v1/settings                   → BFF: 设置读写
└── /health                            → 健康检查
```

**前端架构**：
```
src/
├── main.ts                     # 入口，注册 Router + Pinia
├── App.vue                     # 根布局 + StatusBar
├── router/index.ts             # 6 条路由
├── stores/
│   ├── robot.ts                # 机器人状态（joint_states, io, tool）
│   ├── camera.ts               # 相机状态（流 URL, 质量）
│   └── grasp.ts                # 抓取状态（位姿列表, 循环控制）
├── composables/
│   ├── useRos.ts               # ROSLIB.Ros 单例 + 自动重连
│   ├── useRosTopic.ts          # 话题订阅/取消 (类型安全泛型)
│   └── useRosService.ts        # 服务调用 (async/await 包装)
├── components/
│   ├── RobotViewer.vue         # 3D 查看器（集成 ROS3D.js）
│   ├── CameraStream.vue        # MJPEG 视频流 <img>
│   ├── RobotStatusBar.vue      # 全局状态栏
│   ├── GraspPanel.vue          # 抓取控制面板
│   └── LattePanel.vue          # 咖啡拉花面板
└── views/
    ├── DashboardView.vue       # 门户首页
    ├── VisionGraspView.vue     # 视觉抓取页
    ├── CoffeeLatteView.vue     # 咖啡拉花页
    ├── TfMonitorView.vue       # TF 监控页
    ├── SettingsView.vue        # 设置页
    └── LogView.vue             # 日志页
```

---

## 三、包依赖图（最终状态）

```
                    ┌─────────────┐
                    │  ivg_utils  │  (零依赖，被所有人依赖)
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
┌───────────────┐  ┌──────────────┐  ┌──────────────┐
│ ivg_driver_   │  │ ivg_         │  │ ivg_appli-   │
│ layer         │  │ perception   │  │ cations      │
│               │  │              │  │              │
│ aubo_driver   │  │ vpe ────────►│  │ latte_       │
│ aubo_moveit   │  │ graspnet ───►│  │ imitation    │
│ aubo_desc     │  │ yolo         │  │ coffee_      │
│ aubo_msgs     │  │ hand_eye     │  │ latte_demo   │
│ ivg_services  │  │              │  │              │
│ ivg_service_  │  └──────┬───────┘  └──────┬───────┘
│ interfaces    │         │                  │
│ tool_changer  │         │                  │
└───────┬───────┘         │                  │
        │                 │                  │
        └─────────────────┼──────────────────┘
                          │
                  ┌───────┴───────┐
                  │   ivg_web     │
                  │               │
                  │ web_gateway   │
                  │ web_frontend  │
                  └───────────────┘
```

**依赖原则**：
- `ivg_utils` 零 ROS 依赖，纯 Python 工具库
- 感知层包之间无直接依赖，通过 ROS 2 topic/service 通信
- 应用层依赖感知层和核心服务层的 ROS 2 接口（非 Python import）
- Web 层通过 rosbridge WebSocket 协议桥接，不 import ROS 2 包

---

## 四、关键数据流

### 4.1 视觉抓取主链路

```
相机 → /camera/*/image_raw, /camera/*/points
                │
    ┌───────────┴───────────┐
    ▼                       ▼
GraspNet (GPU)          YOLO OBB
/grasp_poses_base       检测结果 (MarkerArray + JSON)
    │                       │
    └───────┬───────────────┘
            ▼
publish_grasps_client_worker
(窗口收集 → 最优选择 → Z翻转修正)
            │
            ▼
execute_grasp_pose_worker
(接近 → 握持 → 抬升 → 放置)
            │
    ┌───────┴───────┐
    ▼               ▼
move_group      aubo_dashboard
(运动规划)       (IO 控制夹爪)
    │               │
    ▼               ▼
joint_trajectory_controller
(TCP2CAN → SDK → 机械臂)
```

### 4.2 工具快换链路

```
/change_tool srv
    │
    ▼
gripper_swap_worker
├── 1. publishToolStatus(false)  → scene_attach_worker 脱离当前工具
├── 2. MoveIt 笛卡尔运动到快换位
├── 3. set_io(QUICK_SWAP, true)  → aubo_dashboard
├── 4. 运动到新工具对接位
├── 5. loadNewToolYAML()         → 更新 URDF 参数
├── 6. pub/sub tool_changer_status → scene_attach_worker 附着新工具
└── 7. 返回到安全位
```

### 4.3 Web → 机器人双向通信

```
浏览器 (Vue 3)
    │
    ├── WebSocket /ws/rosbridge ──→ FastAPI 网关 ──→ rosbridge (9090)
    │                                                   │
    │   订阅 /joint_states                               │ ROS 2
    │   调用 /change_tool                                │ Network
    │   调用 /execute_single_grasp                       │
    │                                                   │
    ├── HTTP /api/ivg/proxy/web-video/* ──→ FastAPI ──→ web_video_server (8089)
    │                                                   │
    │   <img src> MJPEG 流                              │ ROS 2 camera
    │                                                   │
    └── HTTP /api/ivg/robot-mesh/* ──→ FastAPI ──→ aubo_description 网格文件
```

---

## 五、技术栈总表

| 层级 | 技术 | 量化依据 |
|------|------|----------|
| **机器人框架** | ROS 2 Humble → "L" Turtle (2026 Q3) | Humble 48.53% 市场占比，2027.05 EOL；"L" Turtle 预计 2026.05 发布，支持至 2031 年 |
| **运动规划** | MoveIt 2 (apt) | 移除本地源码复刻，回退到 apt 标准包 |
| **机械臂 SDK** | AUBO SDK C++ | 双连接 (TCP2CAN+RIB / 状态+IO)，线程隔离 |
| **深度学习** | PyTorch | 学术论文 55%+ 占比，HuggingFace 85%+ native，158k Stars |
| **相机驱动** | Percipio SDK C++ | FM830 深度相机 |
| **Python** | 3.10 → 3.12（随 ROS 2 迁移）| 3.11 快 10-60%，3.12 额外 5%+ |
| **后端网关** | FastAPI + uvicorn（Litestar 观察）| FastAPI 92k Stars / 450万日下载 / 38% 采用率；Litestar 5.9k Stars 性能更优（2×吞吐/10×省内存/团队维护/内置JWT+CSRF+限流）但生态太小，列为长期观察 |
| **后端 HTTP 客户端** | httpx | 异步反向代理 |
| **后端 WebSocket** | websockets | rosbridge 代理 |
| **前端框架** | Vue 3 + TypeScript | 52.8k Stars，npm 843万周下载，开发者保留率 87%，中国市场 ~50% 渗透率 |
| **前端状态管理** | Pinia | 80% 采用率，Vue 3 官方推荐 |
| **前端路由** | Vue Router | 6 条路由 |
| **前端 ROS 通信** | ROSLIB.js (importmap) | 浏览器 ↔ ROS 2 标准方案 |
| **前端 3D 渲染** | ROS3D.js | 机器人模型可视化 |
| **WebSocket 桥** | rosbridge_suite | 浏览器 ↔ ROS 2 |
| **视频流** | web_video_server | MJPEG |
| **仿真（当前）** | ros2_control + mock_components | 无硬件时自动切换 |
| **仿真（后续参考）** | Isaac Sim（AI/视觉 RL）+ MuJoCo（运动 RL）+ Gazebo Harmonic（ROS 2 集成）| Isaac Sim GPU 加速万倍并行/2025 开源 Apache 2.0；MuJoCo 接触动力学最佳/MJX 70×加速/DeepMind 维护；Gazebo LTS 至 2028 |
| **数据录制** | rosbag2 | 全话题录制 |
| **容器化（后续参考）** | Docker + Compose | 96% 组织采用，ROS 官方镜像 |
| **编排（后续参考）** | K3s（多机器人时）| 82% 生产采用 K8s，K3s ~500MB/5 分钟安装 |
| **CI/CD（后续参考）** | GitHub Actions | lint + test + build |
| **共享工具** | ivg_utils | 零 ROS 依赖，纯 Python |
| **构建** | colcon | ament_cmake + ament_python |
| **实验追踪（后续参考）** | MLflow + DVC | MLflow 57% 采用率/16M+ 月下载；DVC Git 原生数据版本控制 |
| **VLA 框架（后续参考）** | OpenVLA (1.1k+ Stars) + LeRobot (24k Stars) + StarVLA (2.2k+ Stars) | OpenVLA MIT 完全开源/多工作基座；LeRobot ICLR 2026/HF 背书/200万+轨迹；StarVLA backbone×header 自由组合/8+ Benchmark |
| **LLM 推理（后续参考）** | vLLM (74k Stars) + llama.cpp (100k Stars) | vLLM 50并发 920 tok/s/OpenAI 兼容 API；llama.cpp 纯 C++/CUDA/Metal/Vulkan 全覆盖 |

---

## 六、关键设计决策（附理由）

| # | 决策 | 理由 |
|---|------|------|
| 1 | **Web 框架统一为 FastAPI** | FastAPI 92k Stars / 450万日下载 / 38% 开发者采用 / OpenAI+Anthropic+MS+Netflix 生产使用。Litestar (5.9k Stars) 技术更优但生态太小，列为长期观察。Flask 同步老旧，手眼标定统一迁移 |
| 2 | **前端选 Vue 3 + TypeScript** | Vue 3 52.8k Stars / 87% 保留率（React 75%）/ 中国市场 ~50% 渗透率 / 学习曲线平缓 / 渐进式迁移友好 |
| 3 | **ivg_utils 零 ROS 依赖** | 纯 Python 数学/常量工具库可被非 ROS 环境（训练脚本、MLflow、VLA）复用 |
| 4 | **感知层包间仅通过 ROS 2 接口通信** | Python import 耦合会导致单体化；ROS topic/service 接口是天然解耦边界 |
| 5 | **Web 层通过 rosbridge 桥接** | 不依赖 ROS 2 Python API，可独立部署/扩缩/测试 |
| 6 | **保留 SDK 双连接架构** | `conn_control_`（TCP2CAN+RIB）和 `conn_status_`（状态+IO）职责分离已验证稳定 |
| 7 | **移除 MoveIt 2 本地源码复刻**（审计后）| apt 标准包是官方维护版本，本地复刻导致升级困难 |
| 8 | **IO 引脚语义统一为命名常量** | `ivg_utils.robot_constants` 定义 `GRIPPER_OPEN`/`GRIPPER_CLOSE`/`QUICK_SWAP` 消除语义歧义 |
| 9 | **GraspNet sys.path 改为标准依赖** | 运行时 `sys.path.insert` 脆弱 → `pip install -e` 或 ament_python 包依赖 |
| 10 | **Python 3.10 → 3.12 随 ROS 2 迁移** | 3.11 快 10-60%，3.12 额外 5%+；跟随 "L" Turtle 迁移时统一升级 |
| 11 | **ROS 2 Humble → "L" Turtle** | Humble 48.53% 市场份额但 2027 EOL；"L" Turtle 预计 2031 EOL |
| 12 | **仿真栈：Isaac Sim + MuJoCo**（后续参考）| Isaac Sim (GPU 万倍并行/视觉 RL) + MuJoCo (接触动力学最佳/RL 学术标准) + Gazebo Harmonic (ROS 2 集成最深) |
| 13 | **Docker Compose 三套 profile**（后续参考）| `dev`（源码热重载）、`sim`（仿真）、`prod`（真机）一套配置管理 |
| 14 | **VLA 选型：OpenVLA + LeRobot + StarVLA**（后续参考）| OpenVLA (MIT 完全开源/多工作基座) + LeRobot (24k Stars/ICLR 2026/200万+轨迹) + StarVLA (统一实验平台/8+ Benchmark) |
