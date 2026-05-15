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

> 参考：[ROS 2 Parameters Design](https://design.ros2.org/articles/ros_parameters.html) | [Migrating Parameters](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html) | [robot_state_publisher 源码](https://github.com/ros/robot_state_publisher)

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
colcon build --packages-select aubo_driver_ros2 demo_interface aubo_moveit_config
colcon build --packages-select tool_changer
```

`start_aubo_new_driver.sh` 启动时会自动执行 `colcon build`，日常开发中修改代码后手动运行上述命令即可喵~

## 重要注意事项

1. **所有回复必须使用中文，且每句末尾都要加上"喵~"**，无论回答、解释、代码注释还是错误信息，一律遵守此规则喵~

2. **回复要有修改的理论依据最好先阅读源码和所用库的官方源码以及文档，详细方案经过确认后再执行修改代码，每次修改完代码后都需要更新相关的文档喵~**

3. **严格禁止根据经验和注释猜测判断，必须查看所在流程的源码和所用库的源码及官方文档分析，要有十足的理论依据再判断和修改喵~**
   - 不允许凭"我觉得"、"通常来说"、"应该是"等经验推断下结论喵~
   - 不允许仅凭代码注释就断定行为，注释可能过时或误导，必须以实际源码实现为准喵~
   - 分析任何问题时，必须追溯到相关流程的实际源码（本地源码 + 依赖库源码），不可停留在表面日志或报错信息上喵~
   - 修改前必须确认理解了完整调用链：入口 → 中间层 → 底层库/API 的实际行为喵~
   - 对于 ROS 2 / MoveIt / AUBO SDK 等关键依赖，优先查阅官方源码仓库而非二手博客喵~

4. **AUBO SDK 双连接架构**：SDK 是同步阻塞的（TCP 通信，单次调用 2-225ms），不能放在 ROS 2 实时控制循环中直接调用。新框架将 SDK 调用隔离在独立线程。需要维护两条 TCP 连接：
   - `conn_control_` — TCP2CAN 模式，用于轨迹流式发送 + RIB 诊断查询。**RIB 必须在同一条 TCP 连接上读写**，不同连接读取的 RIB 值不会更新喵~
   - `conn_status_` — 普通模式，用于状态查询（robotServiceRobotInfo 等）+ IO 读写 + 事件回调（onJointStatus/onRoadPoint/onEndSpeed）。TCP2CAN 连接的状态查询缓存可能过时喵~
   - 两条连接的 SDK 回调运行在 SDK 内部线程上，回调内不能调用 SDK API，只能做 atomic 写入 + 日志输出喵~
   - Dashboard 的 18+ 服务都使用 `IsBolck=false` 以避免长时间占用 TCP 连接；`sdk_mutex_` 串行化所有 SDK 调用喵~

5. **本项目的 `JointTrajectoryController` 不是 `ros2_control` 标准控制器**：它继承自 `rclcpp::Node`（非 `controller_interface::ControllerInterface`），实现为一个独立的 Action Server 节点。不与 `controller_manager` 交互，不被 `spawner` 管理。真机模式下不使用 `ros2_control`，仿真模式下才使用 `ros2_control` + `mock_components/GenericSystem`。因此 `ros2 control list_controllers` 在真机模式下为空是正常的。`moveit_manage_controllers=False` 是配套的正确配置喵~

6. **IO 引脚语义不一致**：`ExecuteGraspPoseWorker`（`true=打开`）与 `PublishGraspsClientWorker`/`ABWorker`（`true=闭合`）的夹爪 IO 语义相反，源于不同工位的电气接线差异。IO 引脚映射见 `aubo_ros2_driver/README.md` 第 3 节喵~

7. **GraspNet Z 轴 180° 翻转**：所有 GraspNet 消费者通过 `applyGraspZFlip180()` 自动修正预测位姿的 Z 轴喵~

8. **MoveIt CurrentStateMonitor 回调竞争**：`CurrentStateMonitor` 通过 `jointStateCallback` 订阅 `/joint_states` 更新内部时间戳。`waitForCurrentRobotState()` 检查此时间戳，超过 1 秒未更新则报 `Failed to fetch current robot state`。**根因是单线程 Executor 中长耗时操作阻塞了 `jointStateCallback`**——Executor 在处理回调时无法调度其他回调（wait-set 只报告有消息，不按 FIFO 顺序分发），导致时间戳永不更新。解决方案：使用 `MultiThreadedExecutor`，或将长耗时服务放入独立的 callback group 喵~
   > 参考：[moveit2 Issue #2645](https://github.com/moveit/moveit2/issues/2645) | [moveit2_tutorials Issue #958](https://github.com/moveit/moveit2_tutorials/issues/958) | [ROS 2 About Executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)

9. **`shared_from_this()` 不能在 Node 构造函数中调用**：C++ 标准规定 `shared_from_this()` 必须在对象已被 `std::shared_ptr` 管理后才能调用（`enable_shared_from_this` 的 `weak_ptr` 在 `shared_ptr` 构造完成后才初始化）。ROS 2 Node 构造函数内部尚未就绪，会抛出 `std::bad_weak_ptr`。解决方案：(1) 用 wall timer 延后初始化，(2) 外部 `init()` 方法，(3) 组合模式（不继承 Node，将 Node 作成员变量）喵~
   > 参考：[Stack Overflow](https://stackoverflow.com/questions/73188194/using-shared-from-this-goes-to-bad-weak-ptrros2) | [Robotics Stack Exchange](https://robotics.stackexchange.com/questions/107443/)

10. **`AsyncParametersClient` 不可在构造函数中创建**：同理，需要 `shared_from_this()`。改为在首次 `onToolStatus()` 回调中按需创建喵~

11. **工具切换通过 AttachedCollisionObject + 动态 URDF 双层协作**：`gripper_swap_worker.changeToTool()` 入口处调用 `publishToolStatus(false)` → `scene_attach_worker` 发送 PlanningScene diff（REMOVE 旧工具 AttachedCollisionObject）+ 更新空工具 URDF；运动完成后 `publishToolStatus(true)` → 发送 ADD AttachedCollisionObject + 更新新工具 URDF + 设置 `robot_state_publisher` 参数。

 **碰撞由两层协作提供**：
 - `AttachedCollisionObject`（`/planning_scene` diff，网格附着到 `kuaihuan_Link`）→ `move_group` 原生订阅此 topic，用于规划避障
 - URDF `<collision>`（`updateRobotDescription()`）→ RViz2 视觉渲染 + `robot_state_publisher` TF 树重建
 - **`move_group` 不重载 URDF**：`PlanningSceneMonitor` 的 `RobotModelLoader` 只在构造函数读取一次 `robot_description` 参数，`RDFLoader` 的 `SynchronizedStringParameter` 回调链未连接到上层。因此仅靠动态 URDF 无法让 `move_group` 感知工具碰撞，必须配合 `AttachedCollisionObject` diff 喵~

 **碰撞对象机制**（2026-05-15）：仅管理已附着工具的碰撞。`scene_attach_worker` 发送 `AttachedCollisionObject` diff（ADD/REMOVE）到 `/planning_scene` topic，网格附着到 `kuaihuan_Link`。未附着工具不保留碰撞对象喵~ 不使用 world dock 碰撞对象喵~

 关键文件: `src/tool_changer/src/scene_attach_worker.cpp` (~220行, 已从 410行精简), `config/tools.yaml`, `config/aubo_e5.urdf.xacro`喵~

    **`moveit_ros_visualization` 本地补丁**（MoveIt2 Humble 的运行时 URDF 切换 bug）——
    (a) `planning_scene_display.cpp:createPlanningSceneMonitor`：`Options` 双参构造不初始化 `robot_description_` → "parameter name must not be empty"
    (b) `planning_scene_display.cpp:clearRobotModel` + `motion_planning_display.cpp:clearRobotModel`：不重置 `planning_scene_robot_` / `query_robot_start_` / `query_robot_goal_` → 渲染循环 `rviz::Robot` 持旧 link → "Link not found"

    **双层碰撞机制**（2026-05-15 更新）：
    - `AttachedCollisionObject` — 网格数据通过 `/planning_scene` diff 附着到 `kuaihuan_Link`，`move_group` 的 `PlanningSceneMonitor` 原生监听此 topic，用于规划避障
    - URDF `<collision>` — 视觉渲染 + `robot_state_publisher` TF 更新（`updateRobotDescription()` 保持不变）
    - 不再使用 `world.collision_objects`（不管理 dock 上的未附着工具）喵~
   > 参考：[Planning Scene ROS API](https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)

12. **`tools.yaml` 的 `attach_offset` 与 `aubo_e5.urdf.xacro` 的 `gripper_link` origin 必须严格对齐**：位姿不一致会导致 Web/RViz2 中模型位置与物理安装位置出现偏差喵~

12b. **SRDF ACM (Allowed Collision Matrix) — 末端夹爪工具碰撞豁免**（`aubo_e5.srdf:69-91`）：
 - 5 种可更换末端工具（gripper0/1/2/1coffeecup/1milkcup）× 4 个末端固定链 link（`kuaihuan_Link`, `camera_Link`, `wrist3_Link`, `tool_tcp`）= 20 条 `disable_collisions`
 - 额外 3 条固定链内部豁免（2026-05-15）：`tool_tcp` ↔ `wrist3_Link`（Never）、`tool_tcp` ↔ `camera_Link`（Never）、`tool_tcp` ↔ `kuaihuan_Link`（Never）——这 3 个 link 都从 `wrist3_Link` 通过不同固定关节分叉，物理几何在 Z 方向重叠（tool_tcp 在 Z+0.0235，camera_Link 在 Z+0.020，kuaihuan_Link 在 Z+0.0415），必须豁免碰撞避免红色误报喵~
 - **kuaihuan_Link ↔ 工具 Link**：**Adjacent**（直接父子固定关节）
 - **camera_Link ↔ 工具 Link**：**Adjacent**（隔一层固定关节）
 - **wrist3_Link ↔ 工具 Link**：**Never**（隔两层固定关节）
 - **tool_tcp ↔ 工具 Link**：**Never**（同父 `wrist3_Link` 分叉的两支，固定偏移）
 - 缺乏这些豁免会导致 MoveIt 将工具 mesh 与末端 link 的几何重叠误判为碰撞，规划失败 + RViz2 显示红色伪碰撞喵~

13. **MoveGroupInterface 关键行为**：
    - `move()` 是阻塞的，内部 `plan()` + `execute()` 串行执行，需 async spinner 才能正常完成（它等待 action feedback/result）
    - `computeCartesianPath()` 返回值是 **fraction**（0.0~1.0），表示成功规划的路径比例；< 1.0 表示路径被截断（IK 失败、碰撞、或 jump_threshold 触发）。**必须检查 fraction 再执行**，否则可能只运动了部分路径喵~
    - `jump_threshold`：相对跳变阈值因子（0.0 = 禁用跳变检测）。禁用在真实硬件上可能有风险（奇异点附近 IK 解跳变导致不可预知的大幅关节运动）。MoveIt 2 底层 `CartesianInterpolator` 支持 `JumpThreshold{revolute, prismatic}` 绝对阈值（弧度/米），但 `MoveGroupInterface` 层面仅暴露原始的 `jump_threshold` 因子参数喵~
    - `eef_step`：笛卡尔插值步长（米），值越小路径点越密、IK 求解次数越多。本项目 `kCartesianEefStep = 0.015`（15mm）喵~
 - `allowReplanning(true)`：如果 PlanningScene 在执行过程中发生变化，MoveIt 可能重新规划轨迹，导致运动中轨迹突变。工具切换流程中 `publishToolStatus(false)` 正是为了防止此问题喵~
    - `setMaxVelocityScalingFactor()` / `setMaxAccelerationScalingFactor()`：值域 (0, 1]，传 0 回退到 `joint_limits.yaml` 默认值，>1 被截断为 1.0喵~
    > 参考：[MoveGroupInterface API](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html) | [CartesianInterpolator](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html) | [GetCartesianPath.srv](http://docs.ros.org/en/api/moveit_msgs/html/srv/GetCartesianPath.html)

14. **ROS 2 Callback Group 死锁规则**：
    - 每个节点默认有一个 `MutuallyExclusive` callback group，任何未显式指定 `callback_group` 的实体（subscription/timer/client）都归入此默认组
    - 默认组 + MultiThreadedExecutor 的行为等价于 SingleThreadedExecutor（所有回调互斥）
    - **死锁场景**：在 MutuallyExclusive 组的回调中同步调用 `async_send_request` + `future.wait_for()` → client 内部 spawn 的 `Future done-callback` 继承同一个 mutually exclusive 组 → timer 回调本身占据着组锁永不释放 → done-callback 永远不会执行 → 永久阻塞
    - **正确配置**：服务回调和 client 放入不同 MutuallyExclusive 组，或共享一个 Reentrant 组。项目中 `gripper_swap_worker`（2 线程：服务回调组 + 默认 spin 组）和 `execute_grasp_pose_worker`（Reentrant 组避免死锁）即遵循此规则喵~
    - **注意保留 callback group 引用**：`rclcpp::CallbackGroup::SharedPtr` 必须保持存活，否则 executor 不会调度该组的回调喵~
   > 参考：[Using Callback Groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) | [rclcpp PR #1218](https://github.com/ros2/rclcpp/pull/1218)

15. **ROS 2 QoS 兼容性规则**：
    - `RELIABLE` 发布者不能兼容 `BEST_EFFORT` 订阅者（反之可以）——BEST_EFFORT 订阅者可以接收 RELIABLE 发布
    - `TRANSIENT_LOCAL` 发布者不能兼容 `VOLATILE` 订阅者（反之可以）——VOLATILE 订阅者连 TRANSIENT_LOCAL 发布者时不会收到历史消息
    - 本项目关键 QoS 配置：`/joint_states` (depth 3000, RELIABLE, 100Hz)，`moveItController_cmd` (depth 20000, BEST_EFFORT, 高频防阻塞)，`/camera/*/image_raw` (BEST_EFFORT, depth 5, 图像允许丢帧)，`/robot_description` (TRANSIENT_LOCAL, depth 1, latched 语义)
    - `/robot_description` 的订阅者必须显式使用 `rclcpp::QoS(1).transient_local().reliable()` 才能收到消息喵~
   > 参考：[About QoS Settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

16. **ROS 2 LifecycleNode 状态机规则**：
    - 4 个主状态：`Unconfigured` → `Inactive` → `Active` → `Finalized`
    - 持久化资源（topic pub/sub、service server、buffer 分配）应在 `on_configure` 回调中创建，不应在构造函数中创建。`Inactive` 状态下 service 调用直接失败；`Active` 状态下 service 才正常响应喵~
    - `on_activate` 应轻量级（如获取硬件访问权），不应做耗时初始化喵~
    - `onError` 回调必须全面清理——如果从 `Active` 进入 ErrorProcessing，需要同时完成 `onDeactivate` + `onCleanup` 的工作才能成功回到 `Unconfigured`喵~
    - 节点的 `lifecycle_state` 是一个 latched topic，每次状态转换都会发布（含成功/失败信息）喵~
    > 参考：[Lifecycle Node Design](https://design.ros2.org/articles/node_lifecycle.html)

17. **Launch 文件 OpaqueFunction 在解析时执行**：`aubo_new_driver.launch.py` 的 TCP 探测（`_check_robot_reachable`）发生在 launch 文件解析阶段，决策结果（真机/仿真）在启动后不再重新评估。机械臂在启动后断连不会自动回退到仿真模式喵~

18. **`declare_parameter()` 与 `automatically_declare_parameters_from_overrides` 的冲突**：当 launch 文件使用 `automatically_declare_parameters_from_overrides(true)` 时，参数在节点构造前已被声明。节点构造函数内再次 `declare_parameter()` 同名参数会抛异常。本项目 `execute_grasp_pose_worker` 使用 `has_parameter()` 前置检查 + 条件声明来规避此问题：
    ```cpp
    if (!this->has_parameter("egp_xxx")) {
        this->declare_parameter("egp_xxx", default_value);
    }
    ```
    这是必须的，不能用标准模式（无条件 `declare_parameter` + override）替代喵~

19. **`computeCartesianPath` 跳变检测详细机制**：
    - 相对模式：`|Δjoint_i| < jump_threshold × mean(|Δjoint|)`，任一个关节超出则截断路径。`jump_threshold = 0.0` 完全禁用跳变检测喵~
    - 绝对模式（MoveIt 2 底层 `JumpThreshold` 结构体）：`revolute_jump_threshold`（弧度）和 `prismatic_jump_threshold`（米），非零时启用。混动机器人建议用绝对模式避免弧度/米混合导致误判喵~
    - `MoveGroupInterface::computeCartesianPath()` 仅暴露原始的 `jump_threshold` 因子，底层绝对阈值在服务调用中不可用喵~
    > 参考：[CartesianInterpolator](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html) | [GetCartesianPath.srv](http://docs.ros.org/en/api/moveit_msgs/html/srv/GetCartesianPath.html)

20. **对话后必须将理论依据、参考文献、链接、实现细节和过程写清楚**：每次对话结束后，需要在 CLAUDE.md 或相关文档中记录本次讨论涉及的关键知识点，包括：
    - **理论依据**：为什么这样做，背后的原理是什么喵~
    - **参考文献**：查阅了哪些官方文档、源码文件、API 定义喵~
    - **链接**：所有相关的外部资源 URL（官方文档、GitHub issue、Stack Overflow 等）喵~
    - **实现细节**：具体的修改内容、代码位置、配置变更、关键决策点喵~
    - **完整过程**：从问题发现 → 根因分析 → 方案设计 → 验证方法的完整链路，确保后续读者能理解整个来龙去脉喵~
    > 目的：让任何后来者（包括未来的自己）无需重复调研，直接通过文档就能了解整个过程的完整上下文喵~

21. **每次问题和代码修改后都要更新相关联的文档和代码注释**：任何代码变更（bug 修复、新功能、重构、配置修改）完成后，必须同步更新与之关联的所有文档，包括但不限于：
    - **`aubo_ros2_ws/CHANGELOG.md`**：工作空间级版本变更日志 — 所有代码修改必须在 `[Unreleased]` 节按 Added/Changed/Deprecated/Removed/Fixed 分类记录喵~
    - 包级 `README.md`：接口说明、参数列表、使用方法、引脚映射等喵~
    - 项目级 `README.md`：架构图、启动流程、依赖关系等喵~
    - `CLAUDE.md`：新发现的关键规则、踩坑记录、API 行为细节、常见报错速查表等喵~
    - 技术笔记（如 `IVG2_TECHNICAL_NOTES.md`）：设计决策、方案对比、性能数据等喵~
    - 不更新的文档比没有文档更危险——过时信息会误导后续开发者，浪费大量排查时间喵~
    > 判断标准：如果另一个开发者只看文档不看代码，能否正确理解和使用修改后的功能？不能则文档需要更新喵~

22. **pydantic 1.x 不兼容 Python 3.10 `X | Y` union 类型语法**：FastAPI 路由参数中使用 `dict | None` 等 PEP 604 新式 union 语法时，pydantic 1.8.x 会抛 `TypeError: Fields of type "<class 'types.UnionType'>" are not supported.`。必须使用传统 `typing.Optional[dict]` / `typing.Union[X, Y]` 替代。受影响的 FastAPI 版本 < 0.100（pydantic v1 依赖链）。本项目 `visual_pose_estimation_python/web/routers/` 下 `camera.py`、`templates.py`、`debug.py` 已修复喵~
    > 参考: [FastAPI PR #7350](https://github.com/tiangolo/fastapi/issues/7350) | [pydantic v1 UnionType](https://docs.pydantic.dev/1.10/usage/types/#unions)

23. **Web Dashboard 网关的 Python 依赖不由 colcon 管理**：`aubo_ros2_web_dashboard/setup.py` 声明了 `httpx>=0.25.0` 和 `websockets>=12.0`，但 colcon build 不会自动执行 `pip install -e .`。新环境部署需手动安装：`pip3 install httpx websockets`。缺少 `httpx` 会导致 `ModuleNotFoundError: No module named 'httpx'`，网关启动失败喵~

24. **编译相关文件的引用必须指向真实本地路径**：文档中引用编译所需文件（源码、头文件、库文件、CMakeLists.txt、package.xml、配置文件等）时，必须使用本地文件路径，确保版本与当前编译环境一致喵~
    - **源码引用**：指向本地源码路径，如 `/opt/ros/humble/include/rclcpp/`、`~/ws_moveit/src/moveit2/moveit_core/` 等喵~
    - **头文件引用**：指向具体头文件路径，如 `/opt/ros/humble/include/moveit/robot_state/robot_state.h` 喵~
    - **库文件引用**：指向真实文件（real name，含完整版本号），如 `libprotobuf.so.9.0.1`。soname（`libprotobuf.so.9`）和 linker name（`libprotobuf.so`）用软链接指向真实文件即可，不需要多份副本浪费磁盘喵~
    - 如果编译依赖的源码/库不在本地，应先用包管理器或 `git clone` 获取到本地再引用喵~
    > 理由：外部链接可能失效、版本可能与当前编译环境不一致；本地文件永远与当前构建产物同步。真实文件是唯一数据源，软链接只是别名，引用真实文件确保路径稳定喵~

25. **所有自定义 ROS 2 接口统一在 `ivg_interfaces` 包**：2026-05-13 将原 5 个散落的接口包（`aubo_msgs`、`demo_interface`、`percipio_camera_interface`、`tool_changer_interface`、`interface`）合并为单一 `src/ivg_interfaces/`。共 17 msg + 35 srv = 52 个接口类型。旧包已加 `COLCON_IGNORE` 废弃。
    - **C++ 用法**：`#include <ivg_interfaces/msg/robot_status.hpp>`，类型为 `ivg_interfaces::msg::RobotStatus`，`find_package(ivg_interfaces REQUIRED)` + `<depend>ivg_interfaces</depend>`
    - **Python 用法**：`from ivg_interfaces.msg import RobotStatus`，`from ivg_interfaces.srv import MoveToPose`
    - **JS rosbridge 用法**：`msgType: 'ivg_interfaces/msg/RobotStatus'`，`serviceType: 'ivg_interfaces/srv/ExecuteGraspPose'`
    - 修改接口定义时只需编辑 `src/ivg_interfaces/msg/` 或 `srv/` 下的文件，re-build 后所有依赖包自动更新喵~

26. **`gripper_swap_worker` 数据驱动架构（2026-05-14 重构）**：轨迹参数全部从 `config/tools.yaml` 加载，不再硬编码 per-gripper 函数。
    - **新增工具只需编辑 YAML**：在 `tools.yaml` 中为目标工具添加 `dock_approach_joints`（6 关节角）+ `trajectory`（strategy+参数）两个字段，无需修改 C++ 代码喵~
    - **两种轨迹策略**：`"vertical"`（纯 Z 轴升降，gripper0）和 `"slide"`（Y 轴侧滑+分段 Z 轴，gripper2）。如需新策略，在 `TrajectoryStrategy` enum + `pickTool()`/`releaseTool()` 中添加即可喵~
    - **`changeToTool()` 无硬编码分支**：通用流水线 = 查 YAML 配置 → publishToolStatus(false) 清碰撞 → releaseTool(current) → pickTool(target) → moveToHome() → publishToolStatus(true)。任意工具组合走同一逻辑喵~
    - **`onGripperSwapRequest()` 通用方向解析**：`"X_to_Y"` → 取 `_to_` 后的 `"Y"` 作为 target_id，不再维护硬编码映射表喵~
    - 关键文件：`src/tool_changer/src/gripper_swap_worker.cpp` (loadToolConfig/pickTool/releaseTool/changeToTool)、`config/tools.yaml` (dock_approach_joints + trajectory 字段) 喵~
    - YAML 加载使用 `yaml-cpp` + `ament_index_cpp::get_package_share_directory()`，与 `scene_attach_worker` 一致的加载模式喵~

## 常见报错速查

| 报错关键词 | 可能原因 | 优先检查 |
|-----------|---------|---------|
| `Failed to fetch current robot state` | CurrentStateMonitor 回调被单线程 Executor 阻塞 | `ros2 node info /move_group` 查看 callback group |
| `bad_weak_ptr` | 构造函数中调用了 `shared_from_this()` | 检查节点构造函数代码，延后到回调中初始化 |
| `Could not find parameter robot_description` | 参数未设置到该节点 | `ros2 param get <node> robot_description` |
| `Connection refused` / `TCP` | 机械臂不可达或 SDK 连接失败 | `ping 169.254.10.98`、检查网线 |
| `Controller XXX failed to activate` | ros2_control 控制器类型名不匹配或 command_interface 声明错误 | `ros2 control list_controllers`，检查 YAML 配置 |
| `RIB` 相关 | TCP 连接混用或队列溢出 | 检查 `conn_control_` 连接一致性 |
| `planning failed` / `No valid plan` | 碰撞检测失败或目标不可达 | `ros2 service call /get_planning_scene` 检查碰撞场景 |
| `transform not found` | TF 树断链 | `ros2 run tf2_tools view_frames` |
| `deadlock` / 服务调用永不返回 | 回调组配置错误 | 服务回调和 client 是否共享 MutuallyExclusive 组 |
| `TypeError: types.UnionType` | FastAPI 路由用了 `dict \| None` 语法，pydantic 1.x 不支持 | 改为 `Optional[dict]`（`from typing import Optional`） |
| `ModuleNotFoundError: No module named 'httpx'` | Web Dashboard 网关依赖未 pip 安装 | `pip3 install httpx websockets` |
| FastAPI `/health` 超时 | (1) `setup.py` 未安装 `web/dist/` → 目录不存在 → `ValueError` 崩溃 (2) 代理干扰 | (1) `ls install/aubo_ros2_web_dashboard/share/aubo_ros2_web_dashboard/web/dist/` 确认目录存在; (2) `curl --noproxy '*'` |
| Dashboard 生命周期未激活 | 启动脚本 `local tick=0` 在子 shell `(...)` 中非法 (`local` 仅函数内可用)，`set -e` 下静默退出 | 检查 terminator 标签页是否有 `[dashboard] ✓ 生命周期激活完成` 日志 |
| Web Dashboard gateway 崩溃不重启 | `ExecuteProcess` 无 `respawn` → 进程退出后永久不可用 | `web_dashboard.launch.py:176` 检查 `respawn=True` 是否已配置 |
| `error while loading shared libraries: libauborobotcontroller.so.1` / exit code 127 | AUBO SDK .so 未安装到 `install/` 或未在 `LD_LIBRARY_PATH` | `colcon build --packages-select aubo_driver_ros2`（CMakeLists.txt 已含 install 规则） |
| `Subscription to deprecated ~/state topic` | `ros2 bag record -a` 订阅了弃用 topic；或其他节点订阅了 `~/state`（非 `~/controller_state`） | `ros2 topic info -v /joint_trajectory_controller/state` 确认订阅者；rosbag 场景已在 `start_aubo_new_driver.sh` 中加 `-x '/state$'` 排除 |

## 官方文档与源码仓库

### ROS 2

| 资源 | 链接 |
|------|------|
| ROS 2 Humble 官方文档 | https://docs.ros.org/en/humble/ |
| ROS 2 参数机制设计 | https://design.ros2.org/articles/ros_parameters.html |
| LifecycleNode 状态机设计 | https://design.ros2.org/articles/node_lifecycle.html |
| ROS 1→2 参数迁移指南 | https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html |
| Callback Groups 官方指南 | https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html |
| Executor 机制 (wait-set/调度) | https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html |
| QoS 策略与兼容性 | https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html |
| robot_state_publisher 源码 | https://github.com/ros/robot_state_publisher |
| rclcpp 源码 (humble) | https://github.com/ros2/rclcpp/tree/humble |
| ROS 2 设计文档总览 | https://design.ros2.org/ |

### MoveIt 2

| 资源 | 链接 |
|------|------|
| MoveIt 2 Humble 官方文档 | https://moveit.picknik.ai/humble/index.html |
| MoveIt 2 源码 (humble) | https://github.com/moveit/moveit2/tree/humble |
| MoveIt 2 教程 (humble) | https://github.com/moveit/moveit2_tutorials/tree/humble |
| MoveGroupInterface C++ API | https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html |
| CartesianInterpolator API | https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html |
| GetCartesianPath 服务定义 | http://docs.ros.org/en/api/moveit_msgs/html/srv/GetCartesianPath.html |
| Planning Scene ROS API | https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html |
| CurrentStateMonitor 源码 | `moveit_core/planning_scene_monitor/src/current_state_monitor.cpp` |

### ros2_control

| 资源 | 链接 |
|------|------|
| ros2_control 官方文档 | https://control.ros.org/humble/index.html |
| ros2_controllers 官方文档 | https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html |
| joint_trajectory_controller | https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html |
| controller_manager | https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html |

## 遗留/废弃目录

| 目录 | 状态 | 说明 |
|------|------|------|
| `aubo_ros2_ws/src/moveit_ros_planning/` | `COLCON_IGNORE` — 不参与编译 | 审计确认无本地补丁 (2026-05-15) |
| `aubo_ros2_ws/src/moveit_ros_planning_interface/` | `COLCON_IGNORE` — 不参与编译 | 审计确认无本地补丁 (2026-05-15) |
| `aubo_ros2_ws/src/moveit_ros_visualization/` | 本地复刻 | **有本地补丁**：`planning_scene_display.cpp` + `motion_planning_display.cpp` 修复 MoveIt2 Humble 运行时 URDF 切换 bug喵~ |
| `aubo_ros2_ws/src/camport_ros2/src/image_data_bridge/` | 已删除 (2026-05-14) | 功能由 `hand_eye_calibration/image_data_converter_node` 替代（`start_aubo_new_driver.sh` 中设 `enable_image_data_converter:=true`）喵~ |



27. **latte_imitation MoveIt2 标准管线（2026-05-14 重构）**：废弃自定义 PyKDL DLS IK，全部走 MoveIt2 标准 API 喵~
    - **笛卡尔规划**: `/compute_cartesian_path` — MoveIt2 KDL IK 全 6-DOF，`avoid_collisions=True` 内置碰撞检测喵~
    - **轨迹执行**: `/execute_trajectory` action — MoveIt2 标准 action（非直接 FollowJointTrajectory）喵~
    - **起点自动检测**: 通过 TF `lookup_transform(base_link, tool_tcp)` 获取当前末端位姿，`apply_start_pose()` 自动对齐喵~
    - **Fraction 处理**: ≥0.95 直接执行；0.50~0.95 retry with `avoid_collisions=False`；<0.50 失败喵~
    - **旧字段兼容**: `pos_only` 和 `collision_check` 在服务接口中保留但不再生效（no-op），`ik_success_count` 语义改为 `int(fraction * num_frames)`喵~
    - `computeCartesianPath` 参数: `max_step=0.01`, `jump_threshold=0.0`, `start_state=RobotState()`(空=当前状态)喵~
    - `ReentrantCallbackGroup` + `MultiThreadedExecutor(4)` 防死锁（服务回调中同步等待 action result）喵~
    > 参考：[MoveGroupInterface API](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html) | [GetCartesianPath.srv](http://docs.ros.org/en/api/moveit_msgs/html/srv/GetCartesianPath.html)

28. **已删除模块**: `robot_model.py`(PyKDL FK+IK)、`collision_checker.py`(/check_state_validity)、`action_executor.py`(FollowJointTrajectory)、`trajectory_publisher.py`(旧8阶段管线)。
    新模块 `trajectory_pipeline.py` 实现 5 阶段管线：Load → Transform+AutoPose → Debug → computeCartesianPath → execute 喵~
    - `trajectory_transform.py` 修复了 orientation 旋转 bug：`apply_start_pose()` 现在用 `rot_to_quat()` + `quat_multiply()` 正确旋转 orientation，不再只复制不旋转喵~

## 共享工具包

`ivg_utils`（`aubo_ros2_ws/src/ivg_utils/`）提供跨包共享的 Python 工具喵~：

- `ivg_utils.math` — 四元数↔旋转矩阵↔欧拉角、角度归一化、连通域筛选
- `ivg_utils.io` — IO 引脚定义（`IO_GRIPPER=6`, `IO_QUICK_SWAP=7`）

新增数学工具应优先添加到 `ivg_utils.math` 而非在各包中重复实现喵~

28. **ros3djs/ros2djs 已废弃（2026-05-15）**：原 robotwebtools 中的 ros3djs/ros2djs 源码目录已删除。前端 3D 改用 Three.js 原生实现：
   - `SceneManager` (Three.js 场景/渲染器/OrbitControls)
   - `UrdfParser` (URDF XML → 结构化数据)
   - `UrdfModel` (URDF → Three.js Object3D 树 + STL 加载)
   - `TfUpdater` (TF + joint_states → 关节更新)
   - 所有模块位于 `web/src/lib/three_urdf/`，由 `Robot3dViewer.vue` 组件封装
   - roslibjs 现在通过 npm 包 `roslib@^2.1.0` 引入（不再本地 vendor），rosbridge 协议不变喵~

30. **useRos 自动重连机制（2026-05-15）**：模块级管理 `visibilitychange`/`pagehide`/`online` 事件，12 次指数退避自动重连（1s→30s 上限），与旧框架 `ivgPorts.scheduleRosReconnect` 逻辑一致。页面后台自动断开 rosbridge 连接，前台恢复自动重连。各页面不需要单独处理喵~

31. **工具快换服务接口**：`/run_gripper_swap` (ivg_interfaces/srv/RunGripperSwap)，请求参数 `{ direction: "gripper0_to_gripper2" }`。ToolSwapBar 组件自动根据当前工具 ID 和目标工具 ID 构建 direction 字符串。不是 `/change_tool` 喵~

32. **Vue 3 响应式陷阱 — 模板中的普通变量不触发更新（2026-05-15）**：`let sceneMgr = null` 在 `<script setup>` 中定义后，模板 `v-if="!sceneMgr"` 只在初始渲染时读取值。后续 `sceneMgr = new SceneManager()` 赋值不会触发模板更新（非响应式）。**必须使用 `ref()` / `shallowRef()` 包装**。Three.js 对象（Scene/Renderer/Camera）用 `shallowRef` 避免深度代理导致性能问题。本项目中 `Robot3dViewer.vue` 中的 `sceneMgr` 已修复为 `shallowRef` 喵~

33. **Vite 构建缓存导致旧代码残留（2026-05-15）**：修改 `useRos.ts` 后 `npm run build`，产物中可能不包含最新修改（Vite 增量构建缓存旧模块）。**怀疑构建不生效时先 `rm -rf dist/ .vite/` 强制全量重建**。本会话中 `unsubscribeAll()` 的 `connected.value` 检查因此未生效数轮喵~

34. **roslib npm 包 `Ros` 类的 `isConnected` 是 JS 私有字段 getter（2026-05-15）**：`roslib@2.1.0` 的 `Ros.isConnected` → `get isConnected() { return this.#e; }`（读私有字段 `#e`）。Vue 的 `shallowRef` 虽不深度代理值，但模板/`watch` 中通过 `ros.value?.isConnected` 访问时，Vue 响应式追踪仍通过 `Reflect.get` 包装，遇到私有字段抛 `TypeError: Cannot read from private field`。**修复**: `Ros` 对象彻底移出 Vue 响应式系统，改为 `let rosInstance: Ros | null = null`（普通变量），连接状态通过独立 `ref(false)` 暴露。所有 `ros.value.isConnected` 改用 `isConnected()` 函数（读 `connected.value`）喵~

35. **`connectPromise` 生命周期（2026-05-15）**：`connect()` 的 `connectPromise` 必须在成功和失败路径都清空（`try/finally`），否则重连时 `connectPromise` 仍指向旧 Promise → `connect()` 直接返回旧 Promise → 重连失效。本会话中修复为 `finally { connectPromise = null }` 喵~

36. **`bad_weak_ptr` 根因与修复（2026-05-15）**：`RobotController` 构造函数中 `owner->shared_from_this()` 导致所有 3 个 Worker 节点启动崩溃。根本原因是 `enable_shared_from_this` 的 `weak_ptr` 在 `shared_ptr` 构造完成后才初始化，`std::make_shared` 先调构造函数再设 `weak_ptr`。**修复**：`RobotController` 两阶段初始化 — 构造函数只存 `node_` 指针，新增 `init()` 方法创建 `MoveGroupInterface`（参考 MoveIt2 官方 `MoveGroupInterface(shared_ptr<Node>)` 签名）。3 个 Worker 在 `main()/create()` 中 `make_shared` → `initRobot()` → `robot_->init()` 三步完成初始化喵~

37. **网页 WebSocket 连接浏览器代理拦截（2026-05-15）**：系统 `http_proxy=http://127.0.0.1:7890` 时，浏览器将 `ws://127.0.0.1:8090/ws/rosbridge` 走代理，代理不支持 WebSocket → 连接立即关闭。**修复**：启动脚本 `launch()` 函数对所有终端 `unset http_proxy https_proxy ...`；前端走网关代理 `/ws/rosbridge` 而非直连 9090（代理通常不拦截非标准端口，但 8090 被拦截）。浏览器仍需 `--no-proxy-server` 或系统代理绕过本地地址喵~

38. **`robotwebtools` 已全部删除（2026-05-15）**：`roslibjs` 通过 npm 引入，`ros3djs/ros2djs` 已废弃。启动脚本移除 `ROBOTWEBTOOLS_*` 变量和构建步骤，launch 文件移除 `_find_rwt_assets()` 和相关参数喵~

39. **Python 依赖兼容性（2026-05-15）**：NumPy 2.x 与 `opencv-python`（编译时用 NumPy 1.x）不兼容；`transforms3d` 使用 `np.float`（NumPy 1.24+ 已删除）。**当前稳定组合**: `numpy==1.23.5` + `opencv-python==4.9.0` + 系统 `matplotlib`（卸载 pip 版避免版本冲突）喵~

29. **前端功能模块映射（旧 → 新）**：
   | 旧模块 (JS) | 新模块 (TS/Vue 3) | 说明 |
   |------------|-------------------|------|
   | `ivg_transport.js` + `ros_connector.js` | `useRos.ts` composable | ROS WebSocket 连接管理 |
   | `ROS3D.Viewer` (ros3djs) | `SceneManager.ts` | Three.js 3D 场景 |
   | `ROS3D.UrdfClient` (ros3djs) | `UrdfParser.ts` + `UrdfModel.ts` | URDF 加载与渲染 |
   | `tf_clients.js` | `TfUpdater.ts` | TF + joint_states 更新 |
   | `urdf_panel.js` + `session.js` | `Robot3dViewer.vue` | 3D 查看器 Vue 组件 |
   | `joint_chart.js` | `useJointChart.ts` | Canvas 2D 关节曲线 |
   | `projection_overlay.js` | `useProjectionOverlay.ts` | Canvas 2D 抓取投影 |
   | `services.js` | `useRosService.ts` | ROS 服务调用 |
   | `subscription_binder.js` | `useRosTopic.ts` + inline | 话题订阅管理 |
   | `pose_card.js` | inline (VisionGraspView) | 位姿格式化 |
   | `ivg_runtime.js` | `useRuntime.ts` | BFF 运行时配置 |
   | `log_panel.js` | `LogView.vue` | 系统日志面板 |
   | `robotwebtools/ros3djs/` | npm `three` | 3D 引擎 |
   | `robotwebtools/ros2djs/` | 无（已废弃） | 2D 地图 |
   | `robotwebtools/roslibjs/` (local vendor) | npm `roslib` | ROS WebSocket 客户端 |
   | `vision_grasp_panel.html` + `.js` (649行) | `VisionGraspView.vue` | 视觉抓取主页面喵~

前端从原生 JS + Web Components 渐进式迁移到 Vue 3，详见 `docs/frontend-migration-plan.md` 喵~

| 工具 | 量化依据 |
|------|---------|
| **Vite 6** | 尤雨溪创建，npm 周下载 2000万+，Vue 3 标准构建工具 |
| **Vue 3.5** | Composition API + `<script setup>`，开发者保留率 87%，中国市场 ~50% |
| **TypeScript 5** | ivg_interfaces 52 个接口需类型覆盖，写代码时发现拼写错误 |
| **Tailwind CSS v4** | 87k Stars，2025 Rust 重写核心，构建 268ms 反超 UnoCSS，AI 代码生成 100% 兼容 |
| **Element Plus** | 28k Stars，119万/月 npm 下载，Vue 3 最广泛 UI 组件库，中文母语 |
| **VueUse** | 20k+ Stars，Anthony Fu（中国）创建，200+ composables，`useWebSocket`/`useLocalStorage` 等一行替代手写文件 |
| **Pinia** | Vue 官方状态管理，80% 采用率，仅用于跨页面全局状态（ROS 连接/runtime/工具ID） |
| **vue-echarts** | 64k Stars（百度→Apache），中文文档最完善，Vue 3 官方封装 |
| **unplugin-auto-import** | Anthony Fu 创建，自动导入 Vue API / Element Plus 组件，零样板代码 |

**关键架构不变**：rosbridge WebSocket 协议、roslib/ros3d API、FastAPI 网关路由、MJPEG 视频代理。仅前端封装从构造函数 → composables 模式喵~

**现有代码保留**：`tf_clients.js`（四元数数学）、`patches.js`（Three.js monkey-patch）、`projection_overlay.js`（Canvas 2D 投影）转为 `src/lib/` 下纯 TypeScript 模块，框架无关可直接搬喵~

40. **Web Dashboard 新旧框架行为对齐（2026-05-15）**：基于 Git 历史 `24f4bdfee` 提取旧框架全部 43 个源文件，逐一对比新 Vue 3 框架后发现以下细节差异喵~：
 - **`defaults.yaml` topic-robot 默认值错误**：`/robot_status` → `/aubo_driver/robot_status`。旧版 `config.js:9` 和新 `constants/ros.ts:13` 都正确，唯独 YAML 配置中有错喵~
 - **`defaults.yaml` 接口类名陈旧**：`tool_changer_interface/srv/RunGripperSwap` → `ivg_interfaces/srv/RunGripperSwap`，`demo_interface/srv/ExecuteGraspPose` → `ivg_interfaces/srv/ExecuteGraspPose`（接口统一到 `ivg_interfaces` 包后 YAML 未同步更新）喵~
 - **CoffeeLatteView 严重缩水**：旧版 latte 页面复用 `vision_grasp_panel.js` 的全部订阅编排（3D模型/关节曲线/末端位姿），新版只保留了 IO 开关和流程示意。修复后添加 Robot3dViewer + useJointChart + 末端位姿面板 + 底部连接状态栏 + 监控区折叠喵~
 - **VisionGraspView 缺少"停止"按钮**：旧版 `services.js:100-106` 有 `btn-wp-single-stop` 调用 `SetBool(false)` 停止循环喵~
 - **末端位姿缓存机制**：旧版 `vision_grasp_panel.js:239` 有 `robotPoseCache`，数据空时不闪回默认值。新版缺失导致无数据时显示 `'—'`。修复后缓存有效位姿 HTML 并在模板中优先显示喵~
 - **`ivg_display` 回退字段**：旧版 `pose_card.js:128-130` 在无位姿数据时检查 `msg.ivg_display` 字符串字段。新版未处理此字段喵~
 - **AI 位姿格式化差异**：新版用简陋 `X:...Y:...|QX:...` 单行文本；旧版 `pose_card.js:77-96` 用 `formatPoseBlockHtml()` 生成双 section（位姿 + 四元数）富 HTML。修复后用 `quatToRpyDeg` + `formatPoseBlockHtml` 生成对齐旧版的 HTML 结构，并在 `base.css` 中添加 `pose-card__*` CSS 类喵~
 - **AI 模式左栏快照**：旧版 `vision_grasp_panel.js:380-391` 同时刷新 `cam-mjpeg`（左栏底图）和结果图；新版只刷新结果图。修复后同时更新两幅图喵~
 - **LogView 页面生命周期**：旧版 `log_panel.js:205-216` 监听 `visibilitychange`/`beforeunload`/`pagehide` 事件。新版缺失。修复后添加喵~
 - **CoffeeLatteView DO 开关**：旧版注释说"纯前端切换不调后端服务"（`coffee_latte_io.js:4`），新版的 `try/catch` 也有类似行为，确认一致喵~
 - **pose_card 内联 CSS**: `base.css` 新增 `.pose-card__body`/`.pose-card__pill` 等样式类，供 `v-html` 渲染使用喵~
 - **Robot3dViewer 重复 onUnmounted**: 存在两个连续的 `onUnmounted(() => stop())`，删除重复块喵~

41. **Three.js Canvas 容器高度必须是显式值（2026-05-15）**：`Robot3dViewer` 的 Three.js 渲染器需要一个明确像素高度的容器喵~
 - `class="h-full"` 在父级无明确高度时回退为 `height: 100% of auto = 0` → Canvas 不可见喵~
 - 正确做法：外层 div 用 `h-[400px]`，内层 hostRef div 也用 `h-[400px]`，给 Canvas 显式高度喵~
 - `SceneManager` 构造函数中 `PerspectiveCamera` 的 aspect 计算 `w / h` 需防除零：`w / Math.max(1, h)`喵~
 - `VisionGraspView` / `CoffeeLatteView` 根 div 不能用 `overflow-y-scroll`，否则浏览器双滚动条 + 子容器高度计算异常喵~
 - RViz2 的 `Link 'gripperX_Link' is not known to URDF` 碰撞豁免警告是预期行为：SRDF ACM 预先列出全部 5 种末端工具的豁免，但当前 URDF 只含一种，MoveIt 规划不受影响喵~
