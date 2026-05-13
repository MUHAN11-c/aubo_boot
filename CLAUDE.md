# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Agent skills

### Issue tracker

问题以本地 markdown 文件形式存于 `.scratch/<feature-slug>/` 下。详见 `docs/agents/issue-tracker.md`。

### Triage labels

使用默认标准标签：needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix。详见 `docs/agents/triage-labels.md`。

### Domain docs

多上下文布局 — 根目录 `CONTEXT-MAP.md` 指向各上下文 `CONTEXT.md`。详见 `docs/agents/domain.md`。

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
source ~/ws_moveit/install/setup.bash   # MoveIt 2 独立工作空间（如有）
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

11. **工具切换前必须清除碰撞模型**：`gripper_swap_worker.changeToTool()` 入口处调用 `publishToolStatus(false)`，让 `scene_attach_worker` 立即脱离 PlanningScene 附着体 + 更新 URDF 为无工具版本。

    **PlanningScene diff 机制**：所有的 attach/detach 通过发布 `moveit_msgs::msg::PlanningScene` diff（`is_diff=true`）到 `/planning_scene` 话题实现。attach 一次 diff 中需两条操作——
    (a) `world.collision_objects` 添加一条 `REMOVE`（同 id），(b) `robot_state.attached_collision_objects` 添加一条 `ADD`（指定 `link_name` + `touch_links` + `object.operation=ADD`）。
    detach 是逆向操作。`robot_state.is_diff=true` 必须设置。`touch_links` 用于声明该附着对象允许与哪些 link 碰撞（如夹爪手指）喵~
   > 参考：[Planning Scene ROS API](https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)

12. **`tools.yaml` 的 `attach_offset` 与 `aubo_e5.urdf.xacro` 的 `gripper_link` origin 必须严格对齐**：位姿不一致会导致 Web/RViz2 中模型位置与物理安装位置出现偏差喵~

13. **MoveGroupInterface 关键行为**：
    - `move()` 是阻塞的，内部 `plan()` + `execute()` 串行执行，需 async spinner 才能正常完成（它等待 action feedback/result）
    - `computeCartesianPath()` 返回值是 **fraction**（0.0~1.0），表示成功规划的路径比例；< 1.0 表示路径被截断（IK 失败、碰撞、或 jump_threshold 触发）。**必须检查 fraction 再执行**，否则可能只运动了部分路径喵~
    - `jump_threshold`：相对跳变阈值因子（0.0 = 禁用跳变检测）。禁用在真实硬件上可能有风险（奇异点附近 IK 解跳变导致不可预知的大幅关节运动）。MoveIt 2 底层 `CartesianInterpolator` 支持 `JumpThreshold{revolute, prismatic}` 绝对阈值（弧度/米），但 `MoveGroupInterface` 层面仅暴露原始的 `jump_threshold` 因子参数喵~
    - `eef_step`：笛卡尔插值步长（米），值越小路径点越密、IK 求解次数越多。本项目 `kCartesianEefStep = 0.015`（15mm）喵~
    - `allowReplanning(true)`：如果 PlanningScene 在执行过程中发生变化（如 scene_attach_worker 更新了碰撞对象），MoveIt 可能重新规划轨迹，导致运动中轨迹突变。工具切换流程中 publishToolStatus(false) 清除碰撞模型正是为了防止此问题喵~
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

25. **所有自定义 ROS 2 接口统一在 `ivg_interfaces` 包**：2026-05-13 将原 5 个散落的接口包（`aubo_msgs`、`demo_interface`、`percipio_camera_interface`、`tool_changer_interface`、`interface`）合并为单一 `src/ivg_interfaces/`。共 17 msg + 34 srv = 51 个接口类型。旧包已加 `COLCON_IGNORE` 废弃。
    - **C++ 用法**：`#include <ivg_interfaces/msg/robot_status.hpp>`，类型为 `ivg_interfaces::msg::RobotStatus`，`find_package(ivg_interfaces REQUIRED)` + `<depend>ivg_interfaces</depend>`
    - **Python 用法**：`from ivg_interfaces.msg import RobotStatus`，`from ivg_interfaces.srv import MoveToPose`
    - **JS rosbridge 用法**：`msgType: 'ivg_interfaces/msg/RobotStatus'`，`serviceType: 'ivg_interfaces/srv/ExecuteGraspPose'`
    - 修改接口定义时只需编辑 `src/ivg_interfaces/msg/` 或 `srv/` 下的文件，re-build 后所有依赖包自动更新喵~

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
| FastAPI `/health` 超时 | uvicorn 进程启动阶段崩溃 / 代理干扰 | 查看 terminator 标签页日志; `curl --noproxy '*'` |
| `error while loading shared libraries: libauborobotcontroller.so.1` / exit code 127 | AUBO SDK .so 未安装到 `install/` 或未在 `LD_LIBRARY_PATH` | `colcon build --packages-select aubo_driver_ros2`（CMakeLists.txt 已含 install 规则） |

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
| `aubo_ros2_ws/src/ros_arm_tutorials/` | `COLCON_IGNORE` — 不参与编译 | xArm 教学示例（非 Aubo），仅作参考喵~ |
| `aubo_ros2_ws/legacy/` | `COLCON_IGNORE` — 不参与编译 | 旧版 URDF、视觉姿态估计（C++）等历史代码喵~ |
| `aubo_ros2_ws/src/moveit_ros_planning/` | 本地复刻 | MoveIt2 Humble 源码副本，可能有本地补丁，需调查后决定是否移除喵~ |
| `aubo_ros2_ws/src/moveit_ros_planning_interface/` | 本地复刻 | MoveIt2 Humble 源码副本喵~ |
| `aubo_ros2_ws/src/moveit_ros_visualization/` | 本地复刻 | MoveIt2 Humble 源码副本喵~ |

## 共享工具包

`ivg_utils`（`aubo_ros2_ws/src/ivg_utils/`）提供跨包共享的 Python 工具喵~：

- `ivg_utils.math` — 四元数↔旋转矩阵↔欧拉角、角度归一化、连通域筛选
- `ivg_utils.io` — IO 引脚定义（`IO_GRIPPER=6`, `IO_QUICK_SWAP=7`）

新增数学工具应优先添加到 `ivg_utils.math` 而非在各包中重复实现喵~
