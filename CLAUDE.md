# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Agent skills

### Issue tracker

问题以本地 markdown 文件形式存于 `.scratch/<feature-slug>/` 下。详见 `docs/agents/issue-tracker.md`。

### Triage labels

使用默认标准标签：needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix。详见 `docs/agents/triage-labels.md`。

### Domain docs

多上下文布局 — 根目录 `CONTEXT-MAP.md` 指向各上下文文档（USAGE / DEPLOYMENT / architecture / PROCESS-FLOW 等）。详见 `docs/agents/domain.md`。

## ROS 2 参数隔离（重要！）

**ROS 2 没有全局参数服务器**（与 ROS 1 `rosparam` 完全不同）。每个节点各自维护独立的参数副本，通过 services 对外暴露：

- `get_parameters(names)` — 读取参数值
- `set_parameters(values)` — 逐参数验证并设置，任一个失败不影响其他的已生效
- `set_parameters_atomically(values)` — 原子设置，任一失败则全部回滚
- `list_parameters(prefixes)` — 列出参数名
- `describe_parameters(names)` — 查询参数类型

**正确做法**：用 `AsyncParametersClient::set_parameters()` 设置其他节点参数。YAML 参数文件需用 `/<node_full_name>/ros__parameters` 架构对参数归属节点进行限定喵~

`robot_state_publisher` 不订阅 `/robot_description` 话题，只通过 `/parameter_events` 监听自身参数变更。必须用 `set_parameters()` 而非 `publish()` 来触发 `setupURDF()` 重建 TF 树。`/robot_description` 话题使用了 `TRANSIENT_LOCAL` QoS（latched），订阅者必须配置匹配的 QoS 才能收到消息喵~

> 参考：[ROS 2 Parameters Design](https://design.ros2.org/articles/ros_parameters.html) | [REP-0110](https://www.ros.org/reps/rep-0110.html) | [robot_state_publisher 源码](https://github.com/ros/robot_state_publisher)

## 调试原则

出现问题首先查**源码和官方文档**，不要猜测。具体报错查看 ROS 2 日志（`~/.ros/log/`），运行时数据回溯 `aubo_ros2_ws/rosbags/ivg_session/` 下由 `start_aubo_new_driver.sh` 录制的 rosbag 文件喵~

## 构建规则

**必须在 `aubo_ros2_ws/` 目录下编译。**

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

选择性构建（修改后只需编译相关包，无需全量构建）：

```bash
colcon build --packages-select aubo_driver_ros2 tool_changer latte_imitation
```

`start_aubo_new_driver.sh` 启动时会自动执行 `colcon build`，日常开发中修改代码后手动运行上述命令即可喵~

## 重要注意事项

### 0. 行为准则

1. **所有回复必须使用中文，每句末尾加"喵~"**，代码注释也不例外喵~
2. **修改前必须阅读源码和官方文档**，不允许凭经验和注释猜测。分析问题必须追溯到完整调用链：入口 → 中间层 → 底层库/API 的实际行为。对于 ROS 2 / MoveIt / AUBO SDK 等关键依赖，优先查阅官方源码仓库而非二手博客喵~
3. **每次对话后将理论依据、参考文献、链接、实现细节记录到 CLAUDE.md 或相关文档**，让后来者无需重复调研喵~
4. **每次代码修改后必须同步更新关联文档**（CHANGELOG.md、包级 README、CLAUDE.md），不更新的文档比没有文档更危险喵~
5.搜索网络，查询github和所用库的官方文档，找到最佳的解决方案所有内容都需要有依据，禁止猜测，不确定可以查看源码啊、查看官方文档、或者启动脚本验证

### 1. AUBO SDK 双连接架构

SDK 是同步阻塞的（TCP 通信，单次调用 2-225ms），不能放在 ROS 2 实时控制循环中直接调用。

- `conn_control_` — TCP2CAN 模式，轨迹流式发送 + RIB 诊断查询。**RIB 必须在同一条 TCP 连接上读写**喵~
- `conn_status_` — 普通模式，状态查询 + IO 读写 + 事件回调。TCP2CAN 连接的状态查询缓存可能过时喵~
- SDK 回调运行在 SDK 内部线程上，回调内不能调用 SDK API，只能做 atomic 写入 + 日志输出喵~
- Dashboard 服务使用 `IsBolck=false` 避免长时间占用 TCP 连接；`sdk_mutex_` 串行化所有 SDK 调用喵~

> 详见：`aubo_ros2_driver/README.md` 喵~

### 2. JointTrajectoryController 非 ros2_control 标准

本项目的 `JointTrajectoryController` 继承自 `rclcpp::Node`（非 `controller_interface::ControllerInterface`），实现为独立的 Action Server 节点。真机模式下不使用 `ros2_control`，仿真模式下才用。因此 `ros2 control list_controllers` 在真机模式下为空是正常的，`moveit_manage_controllers=False` 是配套配置喵~

### 3. IO 引脚语义不一致

`ExecuteGraspPoseWorker`（`true=打开`）与 `PublishGraspsClientWorker`/`ABWorker`（`true=闭合`）的夹爪 IO 语义相反，源于不同工位电气接线差异。引脚映射见 `aubo_ros2_driver/README.md` 第 3 节喵~

### 4. GraspNet Z 轴 180° 翻转

所有 GraspNet 消费者通过 `applyGraspZFlip180()` 自动修正预测位姿的 Z 轴喵~

### 5. MoveIt CurrentStateMonitor 回调竞争

`CurrentStateMonitor::waitForCurrentRobotState()` 超过 1 秒未收到 `/joint_states` 则报 `Failed to fetch current robot state`。**根因是单线程 Executor 中长耗时操作阻塞了 `jointStateCallback`**。解决方案：使用 `MultiThreadedExecutor`，或将长耗时服务放入独立 callback group 喵~

> 参考：[moveit2 Issue #2645](https://github.com/moveit/moveit2/issues/2645) | [About Executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)

### 6. `shared_from_this()` 不能在构造函数中调用

C++ 标准规定 `enable_shared_from_this` 的 `weak_ptr` 在 `shared_ptr` 构造完成后才初始化。解决方案：(1) wall timer 延后初始化，(2) 外部 `init()` 方法，(3) 组合模式（不继承 Node）喵~

> 参考：[Stack Overflow](https://stackoverflow.com/questions/73188194/using-shared-from-this-goes-to-bad-weak-ptrros2)

### 7. `AsyncParametersClient` 不可在构造函数中创建

同理需要 `shared_from_this()`，延后到外部 `init()`、timer 中按需创建喵~

### 8. 工具切换碰撞与 ACO 架构

核心要点：
- `scene_attach_worker` 通过 `/attached_collision_object` 发布 ADD/REMOVE（非 `/planning_scene` ACO diff）；向 `/planning_scene` 仅发 world REMOVE 清除 detach 残留
- 工具附着位姿唯一数据源：`tools.yaml.attach_offset`，直接写入 `AttachedCollisionObject.object.pose`
- `gripper_swap_worker` 在物理取放完成瞬间更新 `/tool_changer_status`，不要在切换一开始就清除状态
- SRDF ACM 需豁免 5 种工具 × 4 个末端 link（kuaihuan/camera/wrist3/tool_tcp）+ 3 条固定链内部豁免，否则 MoveIt 误判碰撞

> 详见：`docs/PROCESS-FLOW.md`、`aubo_ros2_ws/src/aubo_e5_moveit_config/config/aubo_e5.srdf:69-91`、`src/tool_changer/config/tools.yaml` 喵~

### 9. MoveGroupInterface 关键行为

- `computeCartesianPath()` 返回值是 **fraction**（0.0~1.0），**必须检查 fraction 再执行**
- `move()` 是阻塞的，需 async spinner 才能正常完成
- `jump_threshold`：0.0 禁用跳变检测。本项目 `kCartesianEefStep = 0.015`（15mm）
- `allowReplanning(true)` 可能导致运动中轨迹突变，工具切换流程中用 `publishToolStatus(false)` 防止
- `setMaxVelocityScalingFactor()` / `setMaxAccelerationScalingFactor()`：值域 (0, 1]

> 参考：[MoveGroupInterface API](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html) | [CartesianInterpolator](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html)

### 10. ROS 2 Callback Group 死锁规则

- 每个节点默认有一个 `MutuallyExclusive` callback group
- **死锁场景**：在 MutuallyExclusive 组回调中同步调用 `async_send_request` + `future.wait_for()` → done-callback 继承同一组锁 → 永久阻塞
- **正确配置**：服务回调和 client 放入不同 MutuallyExclusive 组，或共享 Reentrant 组。项目中 `gripper_swap_worker`（MutuallyExclusive + MultiThreadedExecutor(2)）和 `execute_grasp_pose_worker`（Reentrant 组）遵循此规则
- `CallbackGroup::SharedPtr` 必须保持存活，否则 executor 不会调度该组回调喵~

> 参考：[Using Callback Groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)

### 11. ROS 2 QoS 兼容性规则

- `RELIABLE` 发布者不能兼容 `BEST_EFFORT` 订阅者；`TRANSIENT_LOCAL` 发布者不能兼容 `VOLATILE` 订阅者
- 本项目关键 QoS：`/joint_states` (depth 3000, RELIABLE, 100Hz)，`moveItController_cmd` (BEST_EFFORT, depth 20000)，`/robot_description` (TRANSIENT_LOCAL, depth 1, latched)
- `/robot_description` 订阅者必须用 `rclcpp::QoS(1).transient_local().reliable()` 才能收到消息

> 参考：[About QoS Settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | [REP-2003](https://www.ros.org/reps/rep-2003.html) | `docs/ros2-reps/rep-2003-qos-sensor-map.md`

### 12. LifecycleNode 状态机

4 个主状态：`Unconfigured` → `Inactive` → `Active` → `Finalized`。持久化资源在 `on_configure` 创建，`on_activate` 轻量级，`onError` 必须全面清理。`Inactive` 状态下 service 调用直接失败喵~

> 参考：[Lifecycle Node Design](https://design.ros2.org/articles/node_lifecycle.html) | [REP-0125](https://www.ros.org/reps/rep-0125.html)

### 13. Launch OpaqueFunction 在解析时执行

`aubo_new_driver.launch.py` 的 TCP 探测发生在 launch 解析阶段，决策结果（真机/仿真）启动后不再重新评估。机械臂启动后断连不会自动回退仿真喵~

### 14. `declare_parameter()` 冲突

当 launch 使用 `automatically_declare_parameters_from_overrides(true)` 时，构造函数内需用 `has_parameter()` 前置检查：
```cpp
if (!this->has_parameter("egp_xxx")) {
    this->declare_parameter("egp_xxx", default_value);
}
```

### 15. `computeCartesianPath` 跳变检测

相对模式：`|Δjoint_i| < jump_threshold × mean(|Δjoint|)`。`jump_threshold = 0.0` 禁用。MoveIt 2 底层支持绝对阈值（弧度/米），但 `MoveGroupInterface` 仅暴露相对因子参数喵~

> 参考：[CartesianInterpolator](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html)

### 16. pydantic 1.x 不兼容 `X | Y` 语法

FastAPI 路由参数用 `dict | None` 等 PEP 604 语法时，pydantic 1.8.x 会抛 `TypeError`。必须用 `typing.Optional[dict]` / `typing.Union[X, Y]`。本项目 `visual_pose_estimation_python/web/routers/` 下已修复喵~

> 参考：[FastAPI PR #7350](https://github.com/tiangolo/fastapi/issues/7350)

### 17. Web Dashboard 网关依赖

`httpx` 和 `websockets` 不由 colcon 管理，新环境需手动 `pip3 install httpx websockets`喵~

### 18. 本地路径引用原则

文档中引用编译文件（源码/头文件/库）必须用本地真实路径。真实文件是唯一数据源，软链接只是别名。外部链接可能失效喵~

### 19. `ivg_interfaces` 统一接口包

所有自定义 ROS 2 接口统一在 `src/ivg_interfaces/`（17 msg + 35 srv = 52 个）。旧包（`aubo_msgs`/`demo_interface`/`tool_changer_interface` 等）已 `COLCON_IGNORE` 废弃喵~

- C++：`#include <ivg_interfaces/msg/robot_status.hpp>` + `find_package(ivg_interfaces REQUIRED)`
- Python：`from ivg_interfaces.msg import RobotStatus`
- JS：`msgType: 'ivg_interfaces/msg/RobotStatus'`

### 20. 工具切换数据驱动架构

轨迹参数全部从 `config/tools.yaml` 加载，不再硬编码 per-gripper 函数。新增工具只需编辑 YAML（`dock_approach_joints` + `trajectory` 字段），无需修改 C++ 代码。两种轨迹策略：`"vertical"`（纯 Z 轴升降）和 `"slide"`（Y 轴侧滑）。`changeToTool()` 通用流水线走同一逻辑喵~

> 详见：`src/tool_changer/config/tools.yaml`、`src/tool_changer/src/gripper_swap_worker.cpp` 喵~

### 21. latte_imitation MoveIt2 管线

废弃自定义 PyKDL DLS IK，全部走 MoveIt2 标准 API：
- `/compute_cartesian_path` — KDL IK 全 6-DOF，`avoid_collisions=True`
- `/execute_trajectory` action — MoveIt2 标准 action
- Fraction 处理：≥0.95 直接执行，0.50~0.95 retry with `avoid_collisions=False`，<0.50 失败
- 参数: `max_step=0.01`, `jump_threshold=0.0`, `ReentrantCallbackGroup` + `MultiThreadedExecutor(4)` 防死锁

> 详见：`aubo_ros2_ws/src/latte_imitation/DESIGN.md` 喵~

### 22. 共享工具包 `ivg_utils`

`aubo_ros2_ws/src/ivg_utils/` — `ivg_utils.math`（四元数/旋转矩阵/欧拉角）、`ivg_utils.io`（`IO_GRIPPER=6`, `IO_QUICK_SWAP=7`）。新增数学工具优先加到此包喵~

## 前端开发速查

前端从原生 JS 迁移到 Vue 3 + TypeScript + Vite 6。核心架构：rosbridge WebSocket 协议、roslib npm 包、Three.js 原生 3D、FastAPI 网关。

> **完整前端迁移文档**：`docs/frontend-migration-plan.md` 喵~

**关键踩坑速记**：

| 陷阱 | 修复 | 详见 |
|------|------|------|
| Vue 3 `shallowRef` 中访问 JS 私有字段 `#e` → TypeError | `Ros` 对象移出响应式系统，连接状态用独立 ref | `docs/frontend-migration-plan.md#34` |
| Vite 增量构建缓存旧模块 | `rm -rf dist/ .vite/` 强制全量重建 | `docs/frontend-migration-plan.md#33` |
| Three.js Canvas 容器 `h-full` 在无显式高度父级中 = 0 | 用固定 `h-[400px]` + 防除零 aspect | `docs/frontend-migration-plan.md#41` |
| `connectPromise` 失败路径未清空 → 重连失效 | `finally { connectPromise = null }` | `docs/frontend-migration-plan.md#35` |
| WebSocket 被系统代理拦截 | 启动脚本 `unset http_proxy` + 前端走网关代理 | `docs/frontend-migration-plan.md#37` |
| `let sceneMgr = null` 非响应式 → 模板不更新 | 必须用 `shallowRef()` 包装 Three.js 对象 | `docs/frontend-migration-plan.md#32` |
| VisionGraspView/CoffeeLatteView 路由切换不清理订阅 | ✅ 已修复 (2026-05-18): `Robot3dViewer.stop()` 调用 `unsubscribe()` 清理 rosbridge 订阅 + `initGen` 代数计数器防止 async 竞态 | `Robot3dViewer.vue:462-475` 喵~ |
| WebGL 上下文泄露 SPA 路由切换 3D 空白 | ✅ 已修复 (2026-05-18): `SceneManager.stop()` 新增 `forceContextLoss()` + `renderer.domElement.remove()` | Three.js `r184` `dispose()` 不释放 GL 上下文，浏览器限制 ~16 个喵~ |
| Pinia store 已注册但目录为空 | `main.ts` 中 `createPinia()` 已注册，但 `src/stores/` 无任何 store 文件 | 若不需要可移除注册喵~ |
| `/execute_single_grasp` 等少数服务名硬编码 | 建议改为 `settings.rosName()` 动态获取，与现有架构一致 | 审查发现 2026-05-18 喵~ |

**接口速记**：

- 工具快换服务：`/run_gripper_swap` (ivg_interfaces/srv/RunGripperSwap)，方向格式 `"current_id_to_target_id"`
- 工具几何数据：前端从 BFF `/api/v1/tool-geometries` 动态获取，数据源统一为 `tools.yaml`
- 前端话题/服务名大部分通过 `useDashboardSettings.ts` 读取，少数（如 `'/execute_single_grasp'`、`'/debug/move_to_xyz'`）仍硬编码，待统一喵~

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
| tools.yaml 工具快换失败 (gripper1/coffeecup/milkcup) | `tools.yaml` 中这三个工具缺少 `dock_approach_joints` + `trajectory` 字段 | 补充 YAML 配置或确认这些工具不需要快换喵~ |
| GripperSwapWorker 死锁/service 超时 | callback group 是 MutuallyExclusive（非 Reentrant），与文档描述不一致 | `MultiThreadedExecutor(2)` 提供足够并发，当前无死锁风险喵~ |
| `error while loading shared libraries: libauborobotcontroller.so.1` | AUBO SDK .so 未安装 | `colcon build --packages-select aubo_driver_ros2` |
| `Subscription to deprecated ~/state topic` | rosbag 或节点订阅了弃用 topic | `ros2 topic info -v /joint_trajectory_controller/state` |

## 官方文档与源码仓库

### ROS 2

| 资源 | 链接 |
|------|------|
| ROS 2 Humble 官方文档 | https://docs.ros.org/en/humble/ |
| ROS 2 参数机制设计 | https://design.ros2.org/articles/ros_parameters.html |
| LifecycleNode 状态机设计 | https://design.ros2.org/articles/node_lifecycle.html |
| Callback Groups 官方指南 | https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html |
| Executor 机制 | https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html |
| QoS 策略与兼容性 | https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html |
| robot_state_publisher 源码 | https://github.com/ros/robot_state_publisher |
| rclcpp 源码 (humble) | https://github.com/ros2/rclcpp/tree/humble |
| ROS 2 设计文档总览 | https://design.ros2.org/ |

### ROS 2 REP 标准（本地副本见 `docs/ros2-reps/`）

| REP | 标题 | 状态 | 本地摘要 |
|-----|------|------|---------|
| [REP-2000](https://www.ros.org/reps/rep-2000.html) | Releases and Target Platforms | Active | [Humble 平台 + Tier](docs/ros2-reps/rep-2000-releases-and-target-platforms.md) |
| [REP-2001](https://www.ros.org/reps/rep-2001.html) | ROS 2 Variants | Active | [ros_base/desktop 构成](docs/ros2-reps/rep-2001-variants.md) |
| [REP-2003](https://www.ros.org/reps/rep-2003.html) | Sensor Data and Map QoS | Draft | [传感器/地图 QoS 标准](docs/ros2-reps/rep-2003-qos-sensor-map.md) |
| [REP-2004](https://www.ros.org/reps/rep-2004.html) | Package Quality Categories | Active | [质量等级 L1-L5](docs/ros2-reps/rep-2004-package-quality-categories.md) |
| [REP-2005](https://www.ros.org/reps/rep-2005.html) | ROS 2 Common Packages | Active | [核心包列表](docs/ros2-reps/rep-2005-common-packages.md) |
| [REP-2007](https://www.ros.org/reps/rep-2007.html) | Type Adaptation Feature | Final | [rclcpp 类型适配](docs/ros2-reps/rep-2007-type-adaptation.md) |

核心设计 REP：`docs/ros2-reps/INDEX.md` （REP-0110 Parameters / REP-0125 Lifecycle / REP-0127 URDF 等）喵~

### MoveIt 2

| 资源 | 链接 |
|------|------|
| MoveIt 2 Humble 官方文档 | https://moveit.picknik.ai/humble/index.html |
| MoveIt 2 源码 (humble) | https://github.com/moveit/moveit2/tree/humble |
| MoveGroupInterface C++ API | https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html |
| CartesianInterpolator API | https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html |
| GetCartesianPath 服务定义 | http://docs.ros.org/en/api/moveit_msgs/html/srv/GetCartesianPath.html |
| Planning Scene ROS API | https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html |

### ros2_control

| 资源 | 链接 |
|------|------|
| ros2_control 官方文档 | https://control.ros.org/humble/index.html |
| ros2_controllers 文档 | https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html |
| joint_trajectory_controller | https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html |
| controller_manager | https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html |

## 遗留/废弃目录

| 目录 | 状态 | 说明 |
|------|------|------|
| `aubo_ros2_ws/src/moveit_ros_planning/` | `COLCON_IGNORE` | 无本地补丁 (2026-05-15) |
| `aubo_ros2_ws/src/moveit_ros_planning_interface/` | `COLCON_IGNORE` | 无本地补丁 (2026-05-15) |
| `aubo_ros2_ws/src/moveit_ros_visualization/` | 本地复刻 | **有本地补丁**：URDF 切换 bug 修复 |
| `aubo_ros2_ws/src/camport_ros2/src/image_data_bridge/` | 已删除 (2026-05-14) | 由 `hand_eye_calibration/image_data_converter_node` 替代 |

## 审查新增发现（2026-05-18 架构审查）

### 23. AUBO SDK 驱动层关键实现细节

- `aubo_state_broadcaster` 实际发布频率约 50Hz（受 `client.getJointStatus()` 阻塞时间限制），非文档标注的 200Hz 喵~
- `JointTrajectoryController` 的 `sendLoop` 实现精细的 TCP2CAN 流量控制：固定 5ms 间隔发送，超时后放弃当前点喵~
- `aubo_dashboard_node` (LifecycleNode) 在 `on_configure` 创建 20 个 ROS 2 service + 建立双 TCP 连接；`Inactive` 状态下 service 调用直接失败（ROS 2 Lifecycle 标准行为）喵~
- `sigHandler` 中通过全局指针 `g_node` 访问 Dashboard，析构时可能存在悬空指针风险喵~
- `aubo_callback_monitor` 用于 RIB 周期性查询 + 事件回调分发，回调运行在 SDK 内部线程上喵~

### 24. VPE 视觉姿态估计运行时 Bug

- **严重 Bug**: `ros2_communication.py:1578` 中 `_estimate_pose_for_feature()` 使用未定义变量 `pose_estimate_start`，每次调用必抛 `NameError`，导致姿态估计失败喵~
- `native_api.py:44,141` 和 `resources.py:139` 中存在 `dict[str, Any] | None` PEP 604 语法，pydantic 1.x 可能不兼容喵~
- `graspnet_demo_points_node.py:45-46` CUDA 算子导入无 try/except，若 CUDA 扩展未安装则节点无法启动喵~
- `hand_eye_calibration` 仍使用 Flask（非 FastAPI），与架构目标不一致；主节点是约 5881 行的单体类喵~

### 25. tools.yaml 配置状态（2026-05-18 审查）

- `gripper0` ✅ 完整：vertical 策略 + 已验证关节角喵~
- `gripper1` ⚠️ 待验证：`dock_approach_joints` 复用了 gripper0 参考值（待启动机械臂后运行 `scripts/compute_dock_ik.py` 通过 GetIK 服务求解）喵~
- `gripper2` ✅ 完整：slide 策略喵~
- `gripper1coffeecup` / `gripper1milkcup` ⏭️ 仅仿真：无 `dock_approach_joints` + `trajectory`（`loadToolConfig()` 会跳过），仅用于末端工具几何展示和碰撞调试，不参与真机物理快换喵~

### 26. SRDF ACM 条目全部被注释

- `aubo_e5.srdf:72-91` 中 5 种工具 × 4 个末端 link 的碰撞豁免条目被全部注释喵~
- 运行时通过 `scene_attach_worker` 的 ACO `touch_links` 动态豁免碰撞，功能不受影响；但如果采用 SRDF 静态豁免方案，需取消这些注释喵~

### 27. 前端 3D 模型导航切换修复（2026-05-18）

**4 个根因 + 修复**：

| 根因 | 修复位置 | 方案 |
|------|---------|------|
| WebGL 上下文泄露 — Three.js `r184` `dispose()` 不释放 GL 上下文，SPA 路由切换反复创建 → 浏览器 ~16 限制 → 空白 | `SceneManager.ts:133-138` | `forceContextLoss()` + `renderer.domElement.remove()` |
| Robot3dViewer 卸载不取消 rosbridge 订阅 — 残留 4 个话题占用带宽 | `Robot3dViewer.vue:469-473` | `stop()` 中 `unsubscribe()` 清理 `/tf` `/tf_static` `/joint_states` `/tool_changer_status` |
| async `init()` 竞态 — `await` 期间组件若卸载，后续代码操作已 dispose 场景或注册永不清理 handler | `Robot3dViewer.vue:69,428-454` | `initGen` 代数计数器，`stop()` 递增使在途 async 失效 |
| CoffeeLatteView 订阅 `/tf` `/tf_static` 但无 handler — 隐藏耦合（已删除） | `CoffeeLatteView.vue:164-168` | Robot3dViewer 自管订阅，View 层无需兜底 |

**设计原则**：
- 各组件独立订阅所需话题（符合 ROS 2 节点隔离模型），`useRos.topicSpecs` 去重防止 rosbridge 重复订阅喵~
- Robot3dViewer 自给自足 — 不依赖父 View 为其订阅 TF 话题喵~
- `onRosJson()` 优先使用目标话题名而非 `null` 通配，减少无谓的消息过滤喵~

## latte_imitation 拉花轨迹重定目标包

27. **latte_imitation SE(3) 重定目标理论**：基于 SPOT (Object-centric) + Isaac Teleop (Se3RelRetargeter) + SVRC + SO(3) Action Repr. 等 10 篇文献喵~
 - **核心数据结构**: `CartesianTrajectory` (positions/ orientations/ timestamps)，纯 numpy，无 ROS 依赖喵~
 - **核心变换**: `retarget_trajectory()` — Option B (默认, Se3RelRetargeter 语义): `R_rel = R(rpy_user) @ R(q_cup)`, `p_new = R_rel @ (p - p0) + p_cup`, `q_new = q_rel * q_orig` (Hamilton) 喵~
 - **四元数约定**: Hamilton (xyzw)，`euler_deg_to_quat()` 内旋 ZYX = 外旋 XYZ，与 tf2::Quaternion::setRPY() 完全一致喵~
 - **MoveIt2 参数**: `max_step=0.01`, `jump_threshold=0.0`, `revolute_jump_threshold=0.0`, `prismatic_jump_threshold=0.0` — 审计自 `cartesian_interpolator.cpp` L53/L227/L230 喵~
 - **工作空间安全**: `check_workspace_bounds()` — `base_link` = `world` (URDF identity fixed joint)，边界基于 AUBO E5 官方工作半径 886.5mm + URDF DH 链审计，详见 `workspace_safety.yaml` 喵~
 - **Preview 模式**: `mode="preview"` 发布 5 个 RViz2 markers: Path (TCP 轨迹)、PoseArray (方向箭头)、Marker LINE_STRIP (spout 轨迹)、Marker CUBE (杯子位置)、Marker LINE_LIST (工作空间框) 喵~
 - **并发模型**: `MultiThreadedExecutor(4)` + `ReentrantCallbackGroup`，`self._executing` 防重入锁喵~
 - 详见 `aubo_ros2_ws/src/latte_imitation/DESIGN.md` 喵~

## Python 依赖兼容性

稳定组合：`numpy==1.23.5` + `opencv-python==4.9.0` + 系统 `matplotlib`。NumPy 2.x 不兼容 opencv-python，`transforms3d` 依赖已删除的 `np.float`喵~
