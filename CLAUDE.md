# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ROS 2 通用知识 → 技能引用

ROS 2 通用概念（话题/服务/参数/动作/插件/launch/生命周期/执行器/QoS/TF2/MoveIt2/ros2_control/nav2）已提取到 **ros2-development** 技能。遇到通用 ROS 2 问题时优先加载该技能，本文件仅保留项目特定知识（AUBO SDK、工具切换、前端架构等）喵~

## Agent skills

### Issue tracker

问题以本地 markdown 文件形式存于 `.scratch/<feature-slug>/` 下。详见 `docs/agents/issue-tracker.md`。

### Triage labels

使用默认标准标签：needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix。详见 `docs/agents/triage-labels.md`。

### Domain docs

多上下文布局 — 根目录 `CONTEXT-MAP.md` 指向各上下文文档（USAGE / DEPLOYMENT / architecture / PROCESS-FLOW 等）。详见 `docs/agents/domain.md`。

## ROS 2 参数须知（项目特定）

`robot_state_publisher` 不订阅 `/robot_description` 话题，只通过 `/parameter_events` 监听自身参数变更。**本项目必须用 `AsyncParametersClient::set_parameters()` 而非 `publish()` 来触发 `setupURDF()` 重建 TF 树** 喵~

> ROS 2 参数通用机制见 ros2-development 技能喵~

## 调试原则

出现问题首先查**源码和官方文档**，不要猜测。具体报错查看 ROS 2 日志（`~/.ros/log/`），运行时数据回溯 `aubo_ros2_ws/rosbags/ivg_session/` 下由 `start_aubo_new_driver.sh` 录制的 rosbag 文件喵~

## 构建规则

**必须在 `aubo_ros2_ws/` 目录下编译。**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

选择性构建（修改后只需编译相关包，无需全量构建）：

```bash
colcon build --packages-select aubo_driver_ros2 tool_changer latte_imitation latte_cartesian_planner
```

`start_aubo_new_driver.sh` 启动时会自动执行 `colcon build`，日常开发中修改代码后手动运行上述命令即可喵~

**首次部署依赖安装**：

```bash
# ROS 2 包依赖（从包声明的 package.xml 自动安装）
cd /home/mu/aubo_boot/aubo_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Python 运行依赖（非 colcon 管理）
pip3 install httpx websockets numpy==1.23.5 opencv-python==4.9.0
```

**运行测试**：

```bash
# 运行所有包的测试
colcon test

# 运行指定包的测试
colcon test --packages-select latte_imitation graspnet_ros2

# 查看测试结果
colcon test-result --all

# 查看详细输出
colcon test --event-handlers console_direct+
```

测试目录在各包的 `test/` 下，使用 `ament_cmake_pytest` (Python) 或 `ament_cmake_gtest` (C++) 喵~

### 启动脚本环境变量

```bash
# 跳过编译（代码未修改时加快启动）
SKIP_BUILD=1 ./start_aubo_new_driver.sh

# 跳过 rosbag 录制
SKIP_ROSBAG=1 ./start_aubo_new_driver.sh

# 跳过 RViz2
SKIP_RVIZ=1 ./start_aubo_new_driver.sh

# 自定义机械臂 IP
AUBO_IP=192.168.1.100 ./start_aubo_new_driver.sh

# 自定义 Web 端口
WEB_DASH_PORT=9000 ./start_aubo_new_driver.sh
```

脚本自动检测机械臂是否在线（TCP 端口探测），不可达时回退到仿真模式 (`mock_components/GenericSystem`)。退出时自动清理所有子进程（`cleanup()` trap）喵~

其他辅助脚本：
- `start_latte_test.sh` — 单独启动 latte_imitation 轨迹重放节点（无需完整机械臂驱动）
- `wait_for_service.sh <service_name>` — 轮询等待 ROS 2 service 就绪，用于脚本中的同步点

### 仿真模式服务可用性

`start_aubo_new_driver.sh` 在机械臂不可达时自动回退到仿真模式 (`mock_components/GenericSystem`)。此时 `aubo_dashboard_node` (LifecycleNode) 不启动，其 `/aubo/*` 服务全部不可用。`demo_driver` 节点不受影响喵~：

| 服务 | 仿真 | 真机 | 原因 |
|------|------|------|------|
| `/set_speed_factor` | ✅ 正常 | ✅ 正常 | demo_driver + MoveIt move_group_，不依赖 SDK |
| `/set_robot_enable` | ⚠️ success=false | ✅ 正常 | demo_driver 转发到 `/aubo/startup`，dashboard 不存在 |
| `/get_current_state` | ⚠️ success=false | ✅ 正常 | 内部调用 `/aubo_driver/get_fk` client，无 server |
| `/read_robot_io` | ⚠️ 无数据 | ✅ 正常 | 依赖 `/robot_io_status` 话题 |
| `/move_to_pose` | ❌ 阻塞 | ✅ 正常 | MoveIt `plan()+execute()`，CurrentStateMonitor 超时 (单线程 Executor) |
| `/aubo_driver/get_fk`、`/aubo_driver/get_ik`、`/aubo_driver/set_io` | ❌ 无 server | ✅ 正常 | 仅 `aubo_dashboard_node` 提供 (仿真不启动) |
| `/aubo/stop`、`/aubo/move_joint` 等 15+ | ❌ 无 server | ✅ 正常 | 仅 `aubo_dashboard_node` (LifecycleNode) 提供喵~ |

> 前端调试面板 `callService()` 带 timeoutMs 超时保护，仿真下不会永久阻塞喵~

### `demo_driver` 与 `aubo_dashboard_node` 服务命名空间

| 服务 | 提供者 | 仿真可用 |
|------|--------|---------|
| `/set_speed_factor` | demo_driver (SetSpeedFactorServer) | ✅ |
| `/set_robot_enable` | demo_driver (SetRobotEnableServer) → 调用 `/aubo/startup` | ⚠️ success=false |
| `/move_to_pose` | demo_driver (MoveToPoseServer) → SDK moveJ/moveL | ❌ |
| `/get_current_state` | demo_driver (GetCurrentStateServer) → 内部 client `/aubo_driver/get_fk` | ⚠️ |
| `/read_robot_io` | demo_driver (ReadRobotIOServer) | ⚠️ |
| `/set_robot_pose` | demo_driver (SetRobotPoseServer) → 内部 client `/aubo_driver/get_ik` | ❌ |
| `/plan_trajectory` | demo_driver (PlanTrajectoryServer) | ❌ |
| `/execute_trajectory` | demo_driver (ExecuteTrajectoryServer) | ❌ |
| `/aubo/get_fk`、`/aubo/get_ik`、`/aubo/set_io` 等 20 个 | aubo_dashboard_node (LifecycleNode) | ❌ 仿真不启动 |

> demo_driver 内 `get_current_state_server`、`plan_trajectory_server`、`set_robot_pose_server` 均创建到 `/aubo_driver/*` 的 **client**（非 server）。真机下 aubo_dashboard_node 提供 `/aubo/*` 服务端，两者之间服务名不一致依赖 remap 或实际部署配置对齐喵~

## 重要注意事项

### 0. 行为准则

1. **所有回复必须使用中文**
2. **修改前必须阅读源码和官方文档**，不允许凭经验和注释猜测。分析问题必须追溯到完整调用链：入口 → 中间层 → 底层库/API 的实际行为。对于 ROS 2 / MoveIt / AUBO SDK 等关键依赖，优先查阅官方源码仓库而非二手博客喵~
3. **每次对话后将理论依据、参考文献、链接、实现细节记录到 CLAUDE.md 或相关文档**，让后来者无需重复调研喵~
4. **每次代码修改后必须同步更新关联文档**（包级 README），不更新的文档比没有文档更危险喵~
5.搜索网络，查询github和所用库的官方文档，找到最佳的解决方案所有内容都需要有依据，禁止猜测，不确定可以查看源码啊、查看官方文档、或者启动脚本验证

### 1. AUBO SDK 双连接架构

SDK 是同步阻塞的（TCP 通信，单次调用 2-225ms），不能放在 ROS 2 实时控制循环中直接调用。

- `conn_control_` — TCP2CAN 模式，轨迹流式发送 + RIB 诊断查询。**RIB 必须在同一条 TCP 连接上读写**喵~
- `conn_status_` — 普通模式，状态查询 + IO 读写 + 事件回调。TCP2CAN 连接的状态查询缓存可能过时喵~
- SDK 回调运行在 SDK 内部线程上，回调内不能调用 SDK API，只能做 atomic 写入 + 日志输出喵~
- Dashboard 服务使用 `IsBolck=false` 避免长时间占用 TCP 连接；`sdk_mutex_` 串行化所有 SDK 调用喵~
- `aubo_state_broadcaster` 实际发布频率约 50Hz（受 SDK RIB 轮询 `readDiagnosisInfo()` 阻塞限制），非标称 200Hz 喵~
- `JointTrajectoryController::sendLoop` 默认 4ms 间隔 (250Hz) 发送点位，RIB 积压时缩至 1ms，超时放弃当前点 (TCP2CAN 流控)喵~
- `aubo_dashboard_node` (LifecycleNode) 在 `on_configure` 创建 20 个 ROS 2 service + 双 TCP 连接，`Inactive` 状态下 service 调用直接失败喵~
- `aubo_callback_monitor`：RIB 周期性查询 + 事件回调分发，回调运行在 SDK 内部线程喵~

> 详见：`aubo_ros2_driver/README.md` 喵~

### 2. JointTrajectoryController 非 ros2_control 标准

本项目的 `JointTrajectoryController` 继承自 `rclcpp::Node`（非 `controller_interface::ControllerInterface`），实现为独立的 Action Server 节点。真机模式下不使用 `ros2_control`，仿真模式下才用。因此 `ros2 control list_controllers` 在真机模式下为空是正常的，`moveit_manage_controllers=False` 是配套配置喵~

### 3. IO 引脚语义不一致

`ExecuteGraspPoseWorker`（`true=打开`）与 `PublishGraspsClientWorker`/`ABWorker`（`true=闭合`）的夹爪 IO 语义相反，源于不同工位电气接线差异。引脚映射见 `aubo_ros2_driver/README.md` 第 3 节喵~

### 4. GraspNet Z 轴 180° 翻转

所有 GraspNet 消费者通过 `applyGraspZFlip180()` 自动修正预测位姿的 Z 轴喵~

### 5. ROS 2 / MoveIt 2 通用知识 → 技能

以下通用概念已提取到 ros2-development 技能，此处仅保留项目特定配置喵~：

- **CurrentStateMonitor 超时** — 本项目 MoveGroup 节点用 `MultiThreadedExecutor` 防阻塞
- **`shared_from_this()` / `AsyncParametersClient`** — 不能在构造函数中使用，延后到外部 `init()` 或 wall timer
- **Callback Group 死锁** — 本项目 `gripper_swap_worker`（MutuallyExclusive + MultiThreadedExecutor(2)）、`execute_grasp_pose_worker`（Reentrant 组）
- **QoS 配置** — `/joint_states` (depth 3000, RELIABLE, 100Hz), `moveItController_cmd` (BEST_EFFORT, depth 20000), `/robot_description` (TRANSIENT_LOCAL, depth 1)
- **MoveGroupInterface** — `kCartesianEefStep = 0.015`；`jump_threshold = 0.0` 禁用跳变检测
- **Launch OpaqueFunction** — `aubo_new_driver.launch.py` 的 TCP 探测在 launch 解析阶段执行，决策结果（真机/仿真）启动后不再重新评估
- **`declare_parameter()` 冲突** — launch 用了 `automatically_declare_parameters_from_overrides(true)` 时需 `has_parameter()` 前置检查
- **LifecycleNode** — `aubo_dashboard_node` 在 `on_configure` 创建 20 个 ROS 2 service，`Inactive` 状态下 service 调用直接失败

> 通用 API 用法、代码示例、设计模式见 ros2-development 技能喵~

### 6. 工具切换碰撞与 ACO 架构

核心要点：
- `scene_attach_worker` 通过 `/attached_collision_object` 发布 ADD/REMOVE（非 `/planning_scene` ACO diff）；向 `/planning_scene` 仅发 world REMOVE 清除 detach 残留
- 工具附着位姿唯一数据源：`tools.yaml.attach_offset`，直接写入 `AttachedCollisionObject.object.pose`
- `gripper_swap_worker` 在物理取放完成瞬间更新 `/tool_changer_status`，不要在切换一开始就清除状态
- SRDF ACM 需豁免 5 种工具 × 4 个末端 link（kuaihuan/camera/wrist3/tool_tcp）+ 3 条固定链内部豁免，否则 MoveIt 误判碰撞喵~
- `aubo_e5.srdf:72-91` 中 ACM 条目当前被全部注释，运行时通过 `scene_attach_worker` 的 ACO `touch_links` 动态豁免碰撞喵~

> 详见：`docs/PROCESS-FLOW.md`、`aubo_ros2_ws/src/aubo_e5_moveit_config/config/aubo_e5.srdf:69-91`、`src/tool_changer/config/tools.yaml` 喵~

### 7. pydantic 1.x 不兼容 `X | Y` 语法

FastAPI 路由参数用 `dict | None` 等 PEP 604 语法时，pydantic 1.8.x 会抛 `TypeError`。必须用 `typing.Optional[dict]` / `typing.Union[X, Y]`。本项目 `visual_pose_estimation_python/web/routers/` 下已修复喵~

> 参考：[FastAPI PR #7350](https://github.com/tiangolo/fastapi/issues/7350)

### 8. Web Dashboard 网关依赖

`httpx` 和 `websockets` 不由 colcon 管理，新环境需手动 `pip3 install httpx websockets`喵~

### 9. 本地路径引用原则

文档中引用编译文件（源码/头文件/库）必须用本地真实路径。真实文件是唯一数据源，软链接只是别名。外部链接可能失效喵~

### 10. `ivg_interfaces` 统一接口包

所有自定义 ROS 2 接口统一在 `src/ivg_interfaces/`（17 msg + 35 srv = 52 个）。旧包（`aubo_msgs`/`demo_interface`/`tool_changer_interface` 等）已 `COLCON_IGNORE` 废弃喵~

- C++：`#include <ivg_interfaces/msg/robot_status.hpp>` + `find_package(ivg_interfaces REQUIRED)`
- Python：`from ivg_interfaces.msg import RobotStatus`
- JS：`msgType: 'ivg_interfaces/msg/RobotStatus'`

### 11. 工具切换数据驱动架构

轨迹参数全部从 `config/tools.yaml` 加载，不再硬编码 per-gripper 函数。新增工具只需编辑 YAML（`dock_approach_joints` + `trajectory` 字段），无需修改 C++ 代码。两种轨迹策略：`"vertical"`（纯 Z 轴升降）和 `"slide"`（Y 轴侧滑）。`changeToTool()` 通用流水线走同一逻辑喵~

- `gripper0` ✅ vertical 策略，已验证关节角喵~
- `gripper1` ⚠️ `dock_approach_joints` 复用 gripper0 参考值，待真机 IK 验证喵~
- `gripper2` ✅ slide 策略喵~
- `gripper1coffeecup` / `gripper1milkcup` ⏭️ 仅仿真：无 `dock_approach_joints` + `trajectory`，仅用于 3D 展示和碰撞调试喵~

> 详见：`src/tool_changer/config/tools.yaml`、`src/tool_changer/src/gripper_swap_worker.cpp` 喵~

### 12. latte_workflow_node (C++ 工作流编排)

5 步咖啡拉花工作流编排节点，位于 `demo_driver` 包。服务 `/latte/run_workflow` (ivg_interfaces/srv/RunLatteWorkflow)。

> step1-4 激活，step0（取放咖啡杯）和 step5（拉花执行）保留未启用。**倾倒方向: X 轴旋转 = 牛奶杯倾倒方向**，默认拉花图案 `pattern_type="heart"` 喵~

| 步骤 | 函数 | 运动 | 说明 | 状态 |
|------|------|------|------|------|
| 0 | `step0_pickCoffee` | `moveToJoints` | 取咖啡杯 | ⏸️ |
| 0 | `step0_placeCoffee` | `moveToJoints` | 放咖啡杯 | ⏸️ |
| 1 | `step1_pickMilk` | `moveToJoints` | 取牛奶杯 → 抓取 (不退避) | ✅ |
| 2 | `step2_approachNozzle` | 笛卡尔来回×2 | 去喷嘴 → 2s打奶泡 → 回 → Z退避 | ✅ |
| 3a | `step3_reorient` | 笛卡尔 slerp | 转腕朝上: FK rotate_up_joints | ✅ |
| 3b | `step3_reorient` | 笛卡尔 slerp | 放置咖啡杯: FK place_coffee_joints | ✅ |
| 4 | `step4_pour` | 笛卡尔 slerp | **嘴口倾倒: X轴旋转45° 倾倒牛奶** | ✅ |
| 5 | `step5_executeLatte` | service call | 拉花执行: `/latte/replay_trajectory` | ⏸️ |

**启动**: `ros2 launch aubo_moveit_config latte_workflow.launch.py`
**测试**: `ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow "{}"`

**参数**: 所有参数使用 `lwf_` 前缀，每次 service 回调前 `readParameters()` 实时刷新。笛卡尔失败自动重试 (底层5次×步级3次=最多15次)。默认 `lwf_pattern_type="heart"` 喵~

> 详见：`aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/README.md` 喵~

### 13. RobotController — 笛卡尔 slerp 运动

`demo_driver/robot_controller.h` 中的组合模式运动封装。

**核心接口 `moveCartesianStraight(target, vel, acc)`**:
```
起点 = getCurrentPose()
waypoints = interpolateCartesian(起点, 目标, steps=20)  // 20步 slerp
for retry in 0..5:
    fraction = computeCartesianPath(waypoints, eef_step=0.002, avoid_collisions=true)
    if fraction < 0.99: continue
    execute(traj)
    if success: break
```

**slerp 数学**: 四元数 S³ 球面最短路径 (测地线)，关键细节:
- `dot < 0` → 翻转 q1 到同号半球 (保证最短弧)
- `dot > 0.9995` → 线性插值 + 归一化 (避免除零)
- 默认 20 步 waypoints 密化，喂给 `computeCartesianPath` 做 IK

**其他工具**:
- `slerp(q0, q1, t)` — 静态四元数 slerp，可供外部调用
- `interpolateCartesian(from, to, steps)` — 位置线性 + 朝向 slerp 生成 waypoints
- `jointsToPose(joints)` — FK 将关节角转为 TCP 位姿 (RobotState::setJointGroupPositions + getGlobalLinkTransform)
- `moveToJoints(joints, vel, acc)` — 关节空间 plan()+execute() (注意: 大角度旋转可能绕远路)
- `setGripper(pin, open)` — open=true 打开夹爪

### 14. latte_imitation (Python, 已废弃)

Python 端原轨迹管线，由 `latte_workflow_node` (C++ 工作流) + `latte_cartesian_planner` (C++ 规划执行) 替代喵~

### 15. latte_cartesian_planner (C++, 已废弃)

C++ MoveIt2 规划+执行节点，由 `latte_workflow_node` 内嵌的 `moveCartesianStraight` 替代喵~

### 16. 共享工具包 `ivg_utils`

`aubo_ros2_ws/src/ivg_utils/` — `ivg_utils.math`（四元数/旋转矩阵/欧拉角）、`ivg_utils.io`（`IO_GRIPPER=6`, `IO_QUICK_SWAP=7`）。新增数学工具优先加到此包喵~

## 前端开发速查

**当前架构**: 纯 HTML/JS MPA（零构建），ES modules + importmap 加载 ros3djs/roslib/three.js，FastAPI BFF 网关。

```
web/public/                    # 静态文件根目录
├── index.html                 # MPA 首页
├── vision_grasp_panel.html    # 视觉抓取
├── coffee_latte_panel.html    # 咖啡拉花
├── log_panel.html             # 日志面板
├── settings_panel.html        # 话题/服务设置
├── debug_panel.html            # 调试面板 (状态/运动/IO/IK/FK/话题/服务)
├── tf_monitor_panel.html      # TF 监控
├── css/                       # 样式
└── js/
    ├── core/                  # 基础设施层
    │   ├── log-bus.js         # LogEventBus 单例 + IndexedDB + BroadcastChannel
    │   ├── log-ros-bridge.js  # ROS→日志桥接 (/rosout + service 钩子 + topic 摘要)
    │   ├── ros.js             # RosManager 单例 (连接/订阅/服务调用/重连)
    │   ├── ros_connector.js   # 共享 rosbridge 连接生命周期 (vision_grasp + tf_monitor 共用)
    │   ├── utils.js           # $(id), escapeHtml, canonicalRosTopic, rosMsgArrayField
    │   ├── lifecycle.js       # 页面生命周期 (init/pause/resume/cleanup)
    │   ├── settings.js        # 话题/服务设置管理
    │   ├── dom_cache.js       # document.getElementById 缓存
    │   ├── tf-math.js         # 四元数/RPY 转换
    │   ├── topics.js          # 共享 ROS topic 名/类型常量 (单一数据源)
    │   ├── runtime_provider.js # 从 BFF GET /api/v1/runtime 获取运行时配置
    │   └── ivg_status_bar.js  # 状态栏 Web Component
    ├── components/            # 可复用 UI 组件
    │   ├── pose-card.js       # 位姿 HTML 格式化 (合并自两份旧文件)
    │   ├── joint-chart.js     # Canvas 2D 关节角曲线 (合并自两份旧文件)
    │   └── monitoring-collapse.js  # 监控区折叠/展开 (消除 vision_grasp 和 latte 重复代码)
    ├── vision_grasp/          # 视觉抓取子模块
    │   ├── config.js, services.js, ui_binder.js, mode_controller.js
    │   ├── subscription_binder.js, projection_overlay.js
    │   ├── urdf_panel.js, ui_settings.js
    │   ├── pose_card.js       # → re-export components/pose-card.js
    │   └── joint_chart.js     # → re-export components/joint-chart.js
    ├── latte/                 # 咖啡拉花子模块
    │   ├── main.js            # 入口: 连接 + 3D + 关节图 + 订阅
    │   ├── latte_controls.js  # 参数控制 + 预览/执行 + 全链路日志
    │   ├── execute.js         # 轨迹执行 ROS 服务调用 (带进度心跳)
    │   └── preview.js         # BFF 预览 API
    ├── view3d/                # 3D 渲染 (ros3djs + Three.js)
    │   ├── session.js         # SceneManager — URDF 模型加载 + 渲染
    │   ├── urdf-viewer.js     # createUrdfViewer() 工厂函数
    │   ├── tf_clients.js      # TF2Client 封装 (ros3djs)
    │   ├── patches.js         # ros3djs Three.js 兼容补丁
    │   └── hints.js           # 坐标轴/网格 helper
    ├── ivg_transport.js       # WebSocket 传输层 + rosbridge 协议
    ├── ivg_runtime.js         # 运行时配置 (rosbridge 端口/web_video 端口等)
    ├── ivg_site_nav.js        # 顶部导航栏 Web Component
    ├── ivg_web_video.js       # 相机视频流 (web_video_server MJPEG)
    ├── coffee_latte_io.js     # 拉花 DI 反馈灯 + DO 开关 (绑定 ROS 服务)
    ├── vision_grasp_panel.js  # 视觉抓取页面主逻辑
    ├── tf_monitor_panel.js    # TF 监控页面主逻辑
    ├── log_panel.js           # 日志面板页面主逻辑
    ├── settings_panel.js      # 设置面板页面主逻辑
    └── debug_panel.js          # 调试面板主逻辑 (状态/运动/IO/IK/FK)
```

### 日志系统架构

```
                    ┌──────────────────────────────────┐
                    │     LogEventBus (每页单例)         │
                    │  addLog(level,source,msg,meta)     │
                    │  ← IndexedDB 持久化 (5K 环形缓冲)    │
                    │  ← BroadcastChannel 跨页面实时同步    │
                    └──────────┬───────────────────────┘
                               │
        ┌──────────────────────┼──────────────────────────┐
        │                      │                          │
   [浏览器事件]           [ROS 消息]                [用户操作]
   console拦截            /rosout 订阅              按钮点击 →
   window.onerror         所有 service 响应         参数变更 →
   ros.js 生命周期         topic 摘要               执行阶段 →
   Promise rejection      (mode/tool/status)        DI/DO 变化 →
```

**日志分类 (source categories)**：

| Category | 来源 | 说明 |
|----------|------|------|
| `console` | console.log/warn/error/info/debug | 应用代码直接打印 |
| `error` | window.onerror / unhandledrejection | 未捕获错误 |
| `rosbridge` | WebSocket 连接/关闭/错误 | 传输层事件 |
| `topic` | ROS topic 数据到达 | 模式/状态变更 + 高频摘要 |
| `service` | ROS service 调用 | 开始→进行中(心跳)→✓完成/✗失败 |
| `rosout` | `/rosout` 转发 | ROS 2 节点日志 (带 node/function/line) |
| `lifecycle` | 页面可见性/关闭 | 页面生命周期 |
| `ros_manager` | ros.js 内部事件 | 连接/重连/暂停 |
| `system` | 系统级事件 | 桥接就绪/面板启动/恢复历史 |

**执行状态机**：所有长耗时 ROS 服务调用走 `IDLE → STARTING → IN_PROGRESS(每 5s 心跳) → COMPLETED/FAILED`。

**关键踩坑速记**（保留可能复现的架构性陷阱）：

| 陷阱 | 修复 | 详见 |
|------|------|------|
| 新增 HTML 页面忘记 import `log-ros-bridge.js` → `/rosout` 不订阅 | 每个 HTML 页面加 `<script type="module" src="js/core/log-ros-bridge.js">` | `log-ros-bridge.js` |
| ES module 单例 per-page → 日志面板看不到其他页面日志 | `log-bus.js` 使用 `BroadcastChannel('ivg_log_bus')` 跨页面同步 | `log-bus.js` |
| Three.js r184 `dispose()` 不释放 GL 上下文 | `forceContextLoss()` + `renderer.domElement.remove()` | `view3d/session.js` |
| WebSocket 被系统代理拦截 | 启动脚本 `unset http_proxy`；浏览器对 localhost 绕过代理 | `start_aubo_new_driver.sh` |
| `/aubo_driver/get_fk` 等 SDK 服务在仿真下挂死 | 前端 `callService` 带 timeoutMs 超时保护 | `debug_panel.js` |

**传输层两种访问方式**：

| 页面类型 | 使用方式 | 全局引用 |
|---------|---------|---------|
| vision_grasp / debug | 直接 `import { ivgTransport }` | `globalThis.ivgTransport` |
| latte / 其他 | `import { ros } from './core/ros.js'` (RosManager 单例) | `globalThis.__rosManager` |

`ros.js` 内部持有 `this._transport = ivgTransport`，所有操作最终都走同一传输层。`log-ros-bridge.js` 通过 `_getTransport()` 兼容两种方式喵~

**接口速记**：

- 工具快换服务：`/run_gripper_swap` (ivg_interfaces/srv/RunGripperSwap)，方向格式 `"current_id_to_target_id"`
- 工具几何数据：前端从 BFF `/api/v1/tool-geometries` 动态获取，数据源统一为 `tools.yaml`
- 拉花执行：`/latte_imitation/replay_trajectory` (ivg_interfaces/srv/ReplayLatteTrajectory)，mode=`"preview"|"debug"|"action"`
- URDF 显示切换：`/set_display_tool` (ivg_interfaces/srv/ChangeTool) — 仅仿真显示，不触发物理快换
- DO 控制：`/set_latte_do2`、`/set_latte_do4` (std_srvs/srv/SetBool)
- 话题/服务名在前端通过 `localStorage` 覆盖，默认值见 `config/defaults.yaml` 喵~
- `ivg_transport.js` 提供 `publish({topic, type, msg})` 方法，用于调试面板话题发布喵~

## 常见报错速查

| 报错关键词 | 可能原因 | 优先检查 |
|-----------|---------|---------|
| `Failed to fetch current robot state` | CurrentStateMonitor 回调被单线程 Executor 阻塞 | `ros2 node info /move_group` 查看 callback group |
| `bad_weak_ptr` | 构造函数中调用了 `shared_from_this()` | 延后到回调中初始化 |
| `Could not find parameter robot_description` | 参数未设置到该节点 | `ros2 param get <node> robot_description` |
| `Connection refused` / `TCP` | 机械臂不可达或 SDK 连接失败 | `ping 169.254.10.98`、检查网线 |
| `Controller XXX failed to activate` | ros2_control 类型名不匹配或 command_interface 错误 | `ros2 control list_controllers` |
| `RIB` 相关 | TCP 连接混用或队列溢出 | 检查 `conn_control_` 连接一致性 |
| `planning failed` / `No valid plan` | 碰撞检测失败或目标不可达 | `ros2 service call /get_planning_scene` |
| `transform not found` | TF 树断链 | `ros2 run tf2_tools view_frames` |
| `deadlock` / 服务调用永不返回 | 回调组配置错误 | 检查回调与 client 是否共享 MutuallyExclusive 组 |
| `TypeError: types.UnionType` | FastAPI 用了 `dict \| None` pydantic 1.x 不支持 | 改为 `Optional[dict]` |
| `ModuleNotFoundError: No module named 'httpx'` | 网关依赖未 pip 安装 | `pip3 install httpx websockets` |
| FastAPI `/health` 超时 | `web/dist/` 不存在或代理干扰 | `ls install/.../web/dist/`; `curl --noproxy '*'` |
| `网络连接失败` / 下载失败 | 系统代理未设置或不可达 | `export all_proxy=http://127.0.0.1:7890` |
| Dashboard 生命周期未激活 | `aubo_dashboard` 节点未就绪或 Lifecycle 状态机转换失败 | 检查 terminator 标签页日志；`ros2 lifecycle list /aubo_dashboard` |
| Web Dashboard gateway 崩溃不重启 | ✅ 已修复 (2026-05-18): `web_dashboard.launch.py:155-156` 添加 `respawn=True, respawn_delay=5.0` | — |
| VPE 姿态估计失败 `NameError: pose_estimate_start` | `ros2_communication.py:1578` 使用未定义变量 | 添加 `pose_estimate_start = time.time()` 喵~ |
| VPE pydantic TypeError | `native_api.py`/`resources.py` 使用了 `dict \| None` PEP 604 语法 | 改为 `Optional[dict]` / `Union[dict, None]` |
| GraspNet 节点无法启动 | CUDA 扩展 (pointnet2/knn) 未安装 | `pip install` 对应的 CUDA 扩展，详见错误消息中的 setup.py 路径 |
| tools.yaml 工具快换失败 (gripper1/coffeecup/milkcup) | `tools.yaml` 中这三个工具缺少 `dock_approach_joints` + `trajectory` 字段 | 补充 YAML 配置或确认这些工具不需要快换喵~ |
| GripperSwapWorker 死锁/service 超时 | callback group 是 MutuallyExclusive（非 Reentrant），与文档描述不一致 | `MultiThreadedExecutor(2)` 提供足够并发，当前无死锁风险喵~ |
| `/latte/run_workflow` 不可达 | `latte_workflow_node` 未启动 | 检查 `ros2 service list \| grep latte`; `ros2 launch aubo_moveit_config latte_workflow.launch.py` |
| `/latte/plan_and_execute` 不可达 | (已废弃) `latte_cartesian_planner` 未启动，现由 latte_workflow_node 内嵌笛卡尔替代 | 如有旧脚本引用此服务需更新为 `/latte/run_workflow` |
| 笛卡尔直线 fraction < 0.99 | `moveCartesianStraight` 自动重试 (底层5次+步级3次)，连续失败说明目标不可达或碰撞 | 检查目标位姿是否在工作空间内；检查 avoid_collisions 状态 |
| 步骤5a 笛卡尔直线失败 | `jointsToPose` FK 结果与当前位置差距过大 | 添加 `lwf_transition_joints` 过渡位姿或在 step4 后调低 retract_height |
| `error while loading shared libraries: libauborobotcontroller.so.1` | AUBO SDK .so 未安装 | `colcon build --packages-select aubo_driver_ros2` |
| `Subscription to deprecated ~/state topic` | rosbag 或节点订阅了弃用 topic | `ros2 topic info -v /joint_trajectory_controller/state` |
| 前端日志面板无 ROS 2 日志 | `log-ros-bridge.js` 未被 import 或传输层检测失败 | 检查 HTML 中是否有 `<script type="module" src="js/core/log-ros-bridge.js">`；检查 `globalThis.__rosManager` |
| 前端日志面板看不到其他页面日志 | 跨页面隔离 — 旧版无 BroadcastChannel | 确认 `log-bus.js` 已更新含 `BroadcastChannel('ivg_log_bus')` |
| `SyntaxError: does not provide an export named 'escapeHtml'` | 合并 pose_card.js 时丢失导出 | 确认 `components/pose-card.js` 有 `export { _escapeHtml as escapeHtml }` |

## 外部文档

ROS 2 / MoveIt 2 / ros2_control 官方文档链接见 ros2-development 技能（SKILL.md 末尾 External Resources 章节）喵~

本地 REP 摘要：`docs/ros2-reps/INDEX.md`（REP-0110 Parameters / REP-0125 Lifecycle / REP-0127 URDF / REP-2000~2007 等）喵~

## 遗留/废弃目录

| 目录 | 状态 | 说明 |
|------|------|------|
| `aubo_ros2_ws/src/moveit_ros_planning/` | `COLCON_IGNORE` | 无本地补丁 (2026-05-15) |
| `aubo_ros2_ws/src/moveit_ros_planning_interface/` | `COLCON_IGNORE` | 无本地补丁 (2026-05-15) |
| `aubo_ros2_ws/src/moveit_ros_visualization/` | 本地复刻 | **有本地补丁**：URDF 切换 bug 修复 |
| `aubo_ros2_ws/src/camport_ros2/src/image_data_bridge/` | 已删除 (2026-05-14) | 由 `hand_eye_calibration/image_data_converter_node` 替代 |

## Python 依赖兼容性

稳定组合：`numpy==1.23.5` + `opencv-python==4.9.0` + 系统 `matplotlib`。NumPy 2.x 不兼容 opencv-python，`transforms3d` 依赖已删除的 `np.float`喵~
