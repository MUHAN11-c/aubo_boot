# IVG2.0 技术参考笔记

本文档记录 IVG2.0 项目中 ROS2 Humble、MoveIt2 及本地源码的特性、模式、注意事项，供后续开发和调试参考。

## 目录

1. [ROS2 特性与模式](#1-ros2-特性与模式) — MultiThreadedExecutor, Callback Groups, QoS, Static TF, 参数, Launch
2. [MoveIt2 特性与模式](#2-moveit2-特性与模式) — MoveGroupInterface, Cartesian, MTC, Python API, KDL, 容忍度, IO
3. [URDF/XACRO 模型架构](#3-urdfxacro-模型架构) — 动态工具链, TCP, Transmission, URDF 切换, ros2_control
4. [本地源码特性](#4-本地源码特性) — 架构层次, 固定常量, Dock 几何, CHECK 宏, GraspNet 坐标
5. [工作空间包结构速查](#5-工作空间包结构速查)
6. [手眼标定特性](#6-手眼标定特性) — Custom/OpenCV 算法, TF 发布, 深度校验
7. [常见问题与注意事项](#7-常见问题与注意事项) — 时序, 超时, 仿真, 速度因子
8. [数据流管道速查](#8-数据流管道速查) — Motion/Vision/Calibration Pipeline
9. [MoveIt2 历史问题与修复](#9-moveit2-历史问题与修复) — 超时, 卡顿, 抖动, Pilz 规划器
10. [VPE Web API 架构](#10-vpe-web-api-架构) — API 清单, 并发安全, Event 模式
11. [视觉位姿估计算法](#11-视觉位姿估计算法) — Preprocessor, FeatureExtractor, 模板匹配
12. [AUBO 驱动内部机制](#12-aubo-驱动内部机制) — 控制模式, 数据流, 诊断指标
13. [MoveIt2 OMPL 规划配置](#13-moveit2-ompl-规划配置) — 完整规划器清单, Adapter, 投影
14. [VPE Web 架构](#14-vpe-web-架构-algorithm_http_server_node) — algorithm_http_server_node
15. [构建系统与包配置](#15-构建系统与包配置) — colcon, package.xml, setup.py
16. [GraspNet 推理流水线](#16-graspnet-推理流水线) — 节点架构, 坐标变换
17. [开发工作流建议](#17-开发工作流建议) — 启动顺序, 调试技巧, 常见场景
18. [PlanningScene 动态障碍物](#18-planningscene-动态障碍物) — CollisionObject ADD/REMOVE
19. [ExecuteGraspPoseWorker 详解](#19-executegraspposeworker-详解) — 双模式抓取, 参数表
20. [TF2 使用模式](#20-tf2-使用模式) — StaticTransform, TransformListener, 坐标系约定
21. [MoveIt2 RViz 操作指南](#21-moveit2-rviz-操作指南) — 面板, 交互, 常见问题
22. [执行抓取坐标系统](#22-执行抓取坐标系统) — 变换链, 视觉重试, Z安全下限
23. [OpenCV 手眼标定实现细节](#23-opencv-手眼标定实现细节) — 会话管理, JSON 序列化
24. [工作空间边界限制](#24-工作空间边界限制-limit_workspacepy) — 6面碰撞墙体
25. [工具快换 RViz 可视化](#25-工具快换-rviz-可视化-tool_changer_visualizerpy) — CollisionObject Mesh
26. [夹爪关节状态发布](#26-夹爪关节状态发布) — 消除 MoveIt 警告
27. [MoveIt2 C++ 项目最小模板](#27-moveit2-c-项目最小模板) — 创建/编译/运行
28. [参数管理深度分析](#28-参数管理深度分析) — 三种声明模式, 优先级链
29. [demo_driver 服务节点统一模式](#29-demo_driver-服务节点统一模式) — wait_for_robot_description, main() 模式, service_mutex
30. [ROS2 Executor 内部机制](#30-ros2-executor-内部机制) — Wait Set, 回调组并发规则, spin 对比

---

## 1. ROS2 特性与模式

### 1.1 多线程执行器 (MultiThreadedExecutor)

**使用模式**: 系统中大量使用 `MultiThreadedExecutor`，通常 2 个线程。

| 节点 | 线程数 | 用途 |
|------|--------|------|
| `gripper_swap_worker` | 2 | 线程A: 服务回调组(阻塞执行快换流程)，线程B: spin MoveIt/IO 响应 |
| `visual_pose_estimation_python` | 4 | 处理图像订阅+服务回调+Web 线程 |
| `execute_grasp_pose_worker` | 2 | 服务回调+后台 loop 线程 |

**关键模式**: 服务回调中调用 `async_send_request` + `future.wait_for()` 阻塞等待响应，依赖另一个线程接收并 set future ready。

```cpp
// 典型模式 (gripper_swap_worker.cpp / publish_grasps_client_worker.cpp)
auto future = set_io_client_->async_send_request(req);
if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
    RCLCPP_ERROR(...); return false;
}
auto res = future.get();
```

**注意事项**:
- 回调组类型必须匹配: 服务回调用 `MutuallyExclusive`，client 响应处理用 `Reentrant`
- 不要在 `spin()` 线程中调用 `spin_until_future_complete()` — 会造成隐藏的单线程行为
- 使用 `spin_until_future_complete()` 时需注意它会临时接管 executor

### 1.2 回调组 (Callback Groups)

**本地实践**: 系统中使用三种回调组类型:

| 类型 | 用途 | 示例 |
|------|------|------|
| `MutuallyExclusive` | 服务回调(避免并发执行) | `gripper_swap_worker` 的 swap service |
| `Reentrant` | 视觉服务 client (避免死锁) | `execute_grasp_pose_worker` 的 estimate_pose client |
| 默认组 | spin MoveIt/TF/IO 响应 | executor 第二个线程 |

**注意事项**:
- 历史问题: 长耗时服务使用默认互斥回调组曾导致 `getCurrentPose` 异常，修复方式是使用独立回调组
- `execute_grasp_pose_worker` 中 `estimate_pose_client_cb_group_` 使用 `Reentrant`，防止在服务回调中 `wait_for` 时与默认互斥组死锁

### 1.3 QoS 配置

**ROS2 QoS 核心参数**:

| 参数 | 选项 | 说明 |
|------|------|------|
| **Reliability** | `RELIABLE` (默认) / `BEST_EFFORT` | RELIABLE 保证送达但可能阻塞；BEST_EFFORT 尽力而为不重传 |
| **Durability** | `VOLATILE` (默认) / `TRANSIENT_LOCAL` | VOLATILE 只发给当前订阅者；TRANSIENT_LOCAL 新订阅者也能收到最后一条 |
| **Depth** | 整数值 | KEEP_LAST 模式下的队列深度，超过则丢弃最旧的 |
| **History** | `KEEP_LAST` (默认) / `KEEP_ALL` | KEEP_LAST 保留最近 N 条；KEEP_ALL 保留全部 |
| **Lifespan** | 持续时间 | 消息过期时间，过期后不发送 |

**本地关键 QoS 配置**:

| 话题 | QoS | 原因 |
|------|-----|------|
| `moveItController_cmd` | depth 20000 | 轨迹点高频发送，需大缓冲防丢点导致卡顿 |
| `moveItController_cmd` (simulator pub) | depth 2000 | 插值器输出 |
| `joint_states` | depth 3000 | 100Hz 高频发布 |
| `joint_path_command` | depth 100 | 完整轨迹一次发布 |
| 相机图像订阅 | BEST_EFFORT + KEEP_LAST(depth=1) | 视觉位姿估计使用，BEST_EFFORT 避免延迟堆积；depth=1 只处理最新帧 |
| `device_event` | transient_local | 设备事件只需最新值 |
| `robot_description` (URDF 切换) | transient_local(depth=1) | 新订阅者需收到最后发布的 URDF |

**QoS 兼容性规则**:
- Publisher 和 Subscriber 的 QoS 必须兼容才能通信
- RELIABLE publisher 可以对接 RELIABLE 或 BEST_EFFORT subscriber
- BEST_EFFORT publisher 只能对接 BEST_EFFORT subscriber
- TRANSIENT_LOCAL publisher 即使没有活跃 subscriber 也会缓存消息

### 1.4 静态 TF 发布

**模式**: `StaticTransformBroadcaster` 发布一次即可，不需要持续 spin。

```python
# hand_eye_calibration_tf_publisher.py 模式
tf_broadcaster = StaticTransformBroadcaster(self)
tf_broadcaster.sendTransform(transform_stamped)
# 发布后 spin_once(1.0s) 然后退出 — 静态 TF 持久存在
```

### 1.5 参数声明与覆盖

**模式**: 使用 `automatically_declare_parameters_from_overrides(true)` 从 launch 文件注入参数。

```cpp
// demo_driver 各节点模式
auto node = std::make_shared<Node>(
    "node_name",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
```

**注意事项**:
- 这种模式下，launch 文件中的参数不强制声明也可传入
- 在构造函数中使用 `has_parameter()` 检查后再 `declare_parameter()` 避免重复声明

### 1.7 标准 MoveIt2 Demo Launch 结构 (demo.launch.py)

**节点启动顺序** (按要求依次启动):

```
1. RViz2 (可视化)                    — 最先启动，准备接收可视化数据
2. Static TF: world → base_link     — 虚拟关节的物理表示
3. robot_state_publisher             — 发布 /tf 和 /robot_description
4. move_group (MoveIt2 核心)         — 规划、执行、场景管理
5. ros2_control_node                — ros2_control 硬件抽象层
6. joint_state_broadcaster_spawner  — 发布 /joint_states
7. joint_trajectory_controller_spawner — 接收并执行轨迹命令
8. MongoDB warehouse (可选)          — 存储规划场景，`db:=true` 启用
```

**MoveItConfigsBuilder 模式**:
```python
moveit_config = (
    MoveItConfigsBuilder("aubo_e5", package_name="aubo_moveit_config")
    .robot_description(
        file_path="config/aubo_e5.urdf.xacro",
        mappings={"ros2_control_hardware_type": LaunchConfiguration("...")},
    )
    .robot_description_semantic(file_path="config/aubo_e5.srdf")
    .to_moveit_configs()
)
```

**关键细节**:
- `ros2_control_node` 需要 `remappings=[("/controller_manager/robot_description", "/robot_description")]`
- controller spawner 通过 `-c /controller_manager` 参数指定 controller manager 命名空间
- `robot_state_publisher` 从 `moveit_config.robot_description` 参数获取 URDF
- rviz2 需要 4 个参数: `robot_description`, `robot_description_semantic`, `planning_pipelines`, `robot_description_kinematics`

### 1.8 Python ROS2 常见模式

**客户端缓存模式** (避免重复创建 Service/Action Client):
```python
def _get_or_create_client(node, attr, create_fn):
    client = getattr(node, attr, None)
    if client is None:
        client = create_fn()
        setattr(node, attr, client)
    return client
```

**可中断睡眠** (C++ 对应 `sleepInterruptible`):
```python
# Python 端使用 time.sleep() 或 threading.Event.wait(timeout)
# C++ 端使用 sleepInterruptible 每 100ms 检查 rclcpp::ok() 和 shutdown flag
```

**spin_until_future_complete 陷阱**:
- 在单线程 executor 中调用 `spin_until_future_complete()` 会临时接管 executor
- 如果有其他回调也需要 spin，必须使用 `MultiThreadedExecutor`
- Python 端 pattern:
```python
future = client.call_async(request)
rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
```

**节点销毁与清理**:
- `destroy_node()` 释放 ROS2 资源
- `rclpy.shutdown()` 关闭全局上下文
- 实际中 Python GC 也会自动销毁，但显式调用更安全
- `executor.shutdown()` 停止所有 spin 线程

---

## 2. MoveIt2 特性与模式

### 2.1 规划组命名约定

| 配置 | 机械臂组 | 夹爪组 | 末端 Link | 默认基座帧 |
|------|---------|--------|-----------|------------|
| **aubo_moveit_config** (真机) | `manipulator` | `endeffector` | `tool_tcp` | `base_link` |
| **xarm_moveit_config** (教程) | `xarm` | `gripper` | `gripper_centor_link` | `base_link` |

**命名状态 (Named Targets)**:

| 配置 | 可用状态 |
|------|---------|
| aubo_moveit_config | `up`, `home`, `camera_pose` |
| xarm_moveit_config | `Home`, `Down`, `Close_gripper`, `Open_gripper`, `Handeye_Calibration` |

### 2.2 MoveGroupInterface 使用模式

**规划+执行 (最常用)**:
```cpp
auto move_group = MoveGroupInterface(node, "manipulator");
move_group->setStartStateToCurrentState();  // 关键: 从当前状态开始规划
move_group->setMaxVelocityScalingFactor(0.5);
move_group->setJointValueTarget(joints);
Plan plan;
if (move_group->plan(plan) == SUCCESS)
    move_group->execute(plan);
```

**注意事项**:
- `setStartStateToCurrentState()` 必须在每次规划前调用，否则规划器可能使用旧状态
- `move()` = plan() + execute() 一步完成，适合简单场景
- `move_group->allowReplanning(true)` 允许首次规划失败后自动重试

### 2.3 笛卡尔路径 (computeCartesianPath)

**本地固定参数** (所有 Worker 统一):

| 参数 | 值 | 说明 |
|------|-----|------|
| `eef_step` | 0.015 | 末端插值步长 (m) |
| `jump_threshold` | 0.0 | 不限制关节跳变 |
| 重试次数 | 3 | 失败后重试 |
| 重试间隔 | 0.5s | |
| 初始延迟 | 0.05-0.2s | 给 TF/状态稳定时间 |

**注意事项**:
- `computeCartesianPath` 返回的 `fraction` 表示可达路径比例，1.0=完全可达
- 即使 fraction=1.0，执行仍可能失败（关节极限/碰撞）
- 笛卡尔路径点数不能超过 ~60 点，否则执行变慢/异响

### 2.4 MTC (MoveIt Task Constructor)

**本项目中 MTC 仅用于教学演示** (`xarm_moveit_demo/moveit_pick_place_demo.cpp`)，生产环境未使用。

**关键概念**:
- `Task` 本身是 `SerialContainer`，内部按顺序执行 Stage
- `SerialContainer` 可以有多个 `Solution` (通过 `plan(max_solutions)` 生成)
- `GenerateGraspPose` + `ComputeIK` 是抓取姿态生成的标准组合
- `ModifyPlanningScene` 用于动态修改碰撞检测（附着/分离物体、允许/禁止碰撞）

### 2.5 MoveIt2 通过 Action/Service 使用 (Python)

**grasp_motion_controller.py 模式**: 不使用 C++ `MoveGroupInterface`，而是通过 ROS2 Action 和 Service 直接调用:

```python
# 关节空间规划: MoveGroup Action
move_group_action = ActionClient(self, MoveGroup, '/move_action')
# 笛卡尔路径: GetCartesianPath Service  
cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
# 轨迹执行: ExecuteTrajectory Action
execute_action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
```

**位置约束**: 球体容差 2mm，姿态容差 0.01 rad

### 2.6 KDL 运动学求解器配置

**本机配置** (`kinematics.yaml`):
```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

**说明**:
- KDL (Kinematics and Dynamics Library) 是 MoveIt2 默认数值 IK 求解器
- `search_resolution: 0.005` — IK 搜索精度 0.005 rad，越小精度越高但越慢
- `timeout: 0.005` — 单次 IK 求解超时 5ms，超时返回失败
- KDL 对 6 轴机械臂通常够用，关节极限附近可能求解失败
- 如需更高成功率，可换用 `trac_ik_kinematics_plugin`（支持并行多种子求解）

### 2.7 MoveIt2 规划容忍度 vs 执行容忍度

**关键区别** (来自 MoveIt2 官方文档):
- `setGoalTolerance()` — 设置的是**规划**的容忍度，不是执行的容忍度
- 执行容忍度在 `controller.yaml` 中配置，或手动修改轨迹消息
- 位置约束（`PositionConstraint`）的球体半径是规划约束，不影响控制器
- 本项目 Python 端使用位置球体 2mm + 姿态容差 0.01 rad 作为规划约束

### 2.8 wait_for_robot_description 模式

**demo_driver 所有 service 节点共用**: 循环尝试创建 `MoveGroupInterface` 直到成功。

```cpp
while (rclcpp::ok()) {
    auto test_mg = std::make_shared<MoveGroupInterface>(node, "manipulator");
    if (!test_mg->getPlanningFrame().empty()) return true;
    // 失败则继续等待
}
```

**原因**: demo_driver 服务节点需要 `robot_description` 参数就绪才能创建 MoveGroupInterface，但该参数由 `robot_state_publisher` 并行启动时设置。启动脚本中使用 15 秒延迟确保就绪。

### 2.9 IO 控制模式对比

**C++ async_send_request + future.wait_for** (MultiThreadedExecutor 依赖):
```cpp
auto future = set_io_client_->async_send_request(req);
if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) { /* 超时 */ }
auto res = future.get();
// 线程A阻塞在 wait_for，线程B接收响应后 set future ready → 线程A继续
```

**Python call_async + spin_until_future_complete** (单线程):
```python
future = client.call_async(req)
rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
resp = future.result()
```

**Python Event 等待** (FastAPI/Web 线程，不占用 ROS2 spin):
```python
future = client.call_async(request)
event = threading.Event()
future.add_done_callback(lambda _: event.set())
event.wait(timeout=30.0)  # 后台 spin_once() 处理回调触发 event.set()
return future.result()
```

**关键区别**:
- C++ async_send_request **必须**配合 MultiThreadedExecutor 使用（阻塞当前线程，依赖另一个线程接收响应）
- Python call_async 可直接用 spin_until_future_complete 在单线程中完成
- Python Event 等待适合 FastAPI 线程，与 ROS2 后台 spin 线程解耦

### 2.10 MoveIt2 launch 文件覆盖 YAML 参数

**优先级**: launch 文件 > YAML 配置文件

**示例**: `aubo_moveit_pure_ros2.launch.py` 中的 `moveit_controllers_dict` 更新会覆盖 `moveit_controllers.yaml` 中的值。调试时应检查 launch 文件传递的实际参数字典。

---

## 3. URDF/XACRO 模型架构

### 3.1 动态末端工具链

**aubo_e5.urdf.xacro** 使用 `xacro:arg` + `xacro:if` 实现多工具动态切换:

```xml
<xacro:arg name="gripper" default=""/>
<xacro:if value="${'$(arg gripper)' == 'gripper0'}">
  <xacro:gripper_link name="gripper0" mesh_file="gripper0_link.stl"/>
</xacro:if>
```

**支持的工具**: gripper0 (气动 φ40), gripper1 (电动 A), gripper2 (电动 φ60), gripper1coffeecup, gripper1milkcup

**工具挂载结构** (xacro macro):
- 每个工具定义为一个 link + 一个 fixed joint
- joint 连接: `kuaihuan_Link`(快换盘) → `{name}_Link`
- 偏移: z=0.02m (快换盘到工具中心)
- 每个工具有独立的 visual mesh 和 collision mesh

### 3.2 TCP 定义

```xml
<link name="tool_tcp"/>
<joint name="wrist3_to_tcp" type="fixed">
  <parent link="wrist3_Link"/>
  <child link="tool_tcp"/>
  <origin xyz="0.0 0.0 0.0235" rpy="0.0 0.0 0.0"/>
</joint>
```

- TCP (Tool Center Point) 在 wrist3_Link 前方 0.0235m
- TCP 是规划用的末端 link，不在工具链上（工具是独立的 URDF）

### 3.3 Transmission 与 ros2_control

**所有 6 个关节** 使用 `transmission_interface/SimpleTransmission`:
- `hardwareInterface`: `hardware_interface/PositionJointInterface` (位置命令接口)
- `mechanicalReduction`: 1 (无减速比)

**ros2_control 硬件接口** (`aubo_e5.ros2_control.xacro`):
```xml
<ros2_control name="auboHardwareInterface" type="system">
  <hardware>
    <plugin>fake_components/GenericSystem</plugin>
  </hardware>
  <!-- 每个关节: command_interface=position, state_interfaces=[position, velocity] -->
</ros2_control>
```

**关键点**:
- 仿真模式: `use_fake_hardware:=true` → 使用 `fake_components/GenericSystem`
- 真机模式: `use_fake_hardware:=false` → 需要真实硬件驱动插件
- 每个关节仅 `position` 作为 command interface，velocity/effort 不使用
- 关节位置范围: [-3.05, 3.05] rad (所有关节统一)

### 3.4 运行时 URDF 切换机制

**gripper_swap_worker** 实现了一种动态切换 URDF 的机制，无需重启节点:

```cpp
// 方式1：通过 /robot_description topic 发布（transient_local QoS）
auto msg = std::make_unique<std_msgs::msg::String>();
msg->data = urdf_string;
robot_description_pub_->publish(std::move(msg));

// 方式2：直接设置 robot_state_publisher 节点的参数
auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
    shared_from_this(), "robot_state_publisher");
param_client->set_parameters({rclcpp::Parameter("robot_description", urdf_string)}, callback);
```

**工作原理**:
1. 预加载所有工具的 URDF 到 `gripper_urdfs_` map
2. 切换工具时同时使用两种方式发布:
   - Topic 方式: 确保 transient_local QoS 的新订阅者能收到最新值
   - Parameter 方式: 确保 robot_state_publisher 节点参数更新
3. 两种方式互补，提高可靠性

**注意事项**:
- `robot_description` topic 的 QoS 设为 `transient_local` (depth=1)，确保新订阅者收到最后一条消息
- 参数方式使用异步回调，不阻塞主线程
- 此机制仅更新可视化/TF，不影响 MoveGroupInterface 的内部状态

### 3.5 C++ CHECK 宏模式

**项目内多个 Worker 统一使用的控制流宏**:
```cpp
#define CHECK(expr) do { if (!(expr)) return false; } while (0)
```

**用途**: 在长流程函数中，每一步失败时立即返回 false，避免深层嵌套的 if-else。

**典型用法**:
```cpp
bool runOneCycle() {
    CHECK(selectBestGrasp());        // 步骤1失败 → return false
    CHECK(computeApproachPath());    // 步骤2失败 → return false
    CHECK(executeGrasp());           // 步骤3失败 → return false
    return true;
}
```

**注意事项**:
- 宏展开为 `do { ... } while(0)`，确保在任何上下文中安全（if/else/无花括号）
- 不记录失败日志，调用者需要自行在 CHECK 失败前打日志
- 仅适用于返回 bool 的函数，不适用于需要错误码或异常的场景

### 3.6 sleepInterruptible 模式

**C++ 可中断睡眠** (所有 C++ Worker 共用):
```cpp
static void sleepInterruptible(Worker* worker, double seconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && !worker->isShutdownRequested() && 
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 每100ms检查一次
    }
}
```

**用途**: 在服务回调中需要 sleep 时，支持 SIGINT/SIGTERM 中断退出，避免阻塞 shutdown。

**对应 Python 端**: 使用 `time.sleep()` 或 `threading.Event.wait(timeout)`（Python 端无统一的可中断睡眠封装）。

### 3.7 ros2_control 控制器配置

**`ros2_controllers.yaml`** 定义了:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 200  # Hz

joint_trajectory_controller:
  ros__parameters:
    command_interfaces:
      - position          # 仅位置命令
    state_interfaces:
      - position
      - velocity
    joints: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint]
```

**注意**: `command_interfaces` 只有 `position`，没有 `velocity`。这意味着控制器按纯位置模式运行，速度由轨迹时间戳隐式控制（MoveIt 的 AddTimeOptimalParameterization 适配器负责时间参数化）。

---

## 4. 本地源码特性

### 4.0 代码架构层次

**C++ 继承体系**:
```
Node (rclcpp)
  └── MoveitGripperIoBase                    — 基类：MoveIt2 运动能力 + IO 控制
        ├── MoveToPoseServer                 — 服务：位姿运动
        ├── PlanTrajectoryServer             — 服务：轨迹规划
        ├── SetSpeedFactorServer             — 服务：速度设置
        ├── SetRobotPoseServer               — 服务：位姿设置
        ├── ExecuteGraspPoseWorker           — 服务：执行抓取
        └── PublishGraspsClientWorker        — 服务驱动：抓取放置循环
  ├── GripperSwapWorker (独立)              — 快换 Worker，不继承 MoveitGripperIoBase
  └── ExecuteTrajectoryServer (独立)        — Action server，直接使用 MoveIt

**Python 继承体系**:
```
Node (rclpy)
  └── GraspMotionController                  — 基类：MoveIt2 Action/Service 调用
        └── PublishGraspsClient             — 订阅 GraspNet → 选优 → 执行
  ├── VisualPoseEstimationNode               — 视觉位姿估计主节点 (main.py)
  ├── AlgorithmHTTPServerNode                — FastAPI Web 服务节点
  └── LatteNode                              — 咖啡拉花 IO 控制
```

### 4.1 固定常量 (Hardcoded)

| 常量 | 值 | 位置 | 用途 |
|------|-----|------|------|
| `kHomeJointsRad1` | `{1.21, 0.13, 1.93, 0.23, 1.57, 1.21}` | moveit_gripper_io_base.cpp | 回安全位关节角 (camera_pose) |
| `kJoints_DockStation` | `{0.937, 0.017, 1.419, -0.168, 1.572, 0.936}` | gripper_swap_worker.cpp | gripper2 dock 对接工位 |
| `kJoints_ReleaseGripper0` | `{1.138, 0.223, 1.598, -0.195, 1.572, 1.137}` | gripper_swap_worker.cpp | gripper0 释放工位 |
| `kDockDepth` | 0.206m | gripper_swap_worker.cpp | 快换 Z 行程 |
| `kDockSeat` | 0.012m | gripper_swap_worker.cpp | 快换锁紧微行程 |
| `kDockLift` | 0.210m | gripper_swap_worker.cpp | 快换抬升行程 |
| `kDockSlideY` | 0.100m | gripper_swap_worker.cpp | gripper2 Y 滑入/滑出 |
| `kCartesianEefStep` | 0.015m (C++ worker), 0.01m (gripper_swap / Python) | 各 Worker | 笛卡尔插值步长，值越小轨迹点越密 |
| `kCartMaxRetries` | 5 (gripper_swap), 3 (其他 Worker) | 各 Worker | 笛卡尔路径最大重试次数 |
| `kIoCallTimeoutSeconds` | 60s | C++ Workers | IO 异步调用超时 |
| `kReleaseWaitSec` | 0.3s | gripper_swap_worker | 开IO释放夹爪后气动响应+机械稳定时间 |
| `kLockWaitSec` | 0.5s | gripper_swap_worker | 关IO锁定夹爪后气动响应+机械稳定时间 |
| `depth_scale` | 0.00025 | percipio/hand_eye | 深度值缩放因子 (raw × 0.00025 = m) |
| 关节最大速度 | shoulder/upperArm/foreArm: 2.5964 rad/s, wrist1/2/3: 3.1105 rad/s | joint_limits.yaml | 各关节独立 |
| 关节最大加速度 | shoulder/upperArm/foreArm: 2.5 rad/s², wrist1/2/3: 3.0 rad/s² | joint_limits.yaml | Pilz 规划器 + adapters 依赖 has_acceleration_limits: true |
| 默认速度/加速度因子 | 0.1 (10%, joint_limits.yaml) → 实际运行中通过 MoveGroupInterface 覆盖 | joint_limits.yaml + launch | joint_limits.yaml 的默认值是全局保底，运行时 setMaxVelocityScalingFactor() 覆盖 |

### 4.2 Dock 几何 (快换工位)

```
gripper0 dock: x=0.27017, y=0.29517, z=0.270 (dock base)
gripper2 dock: x=0.3741,  y=0.30394, z=0.270 (dock base)
安全高度 Z:     0.4755 = 桌面0.235 + 安全余量0.2405
```

### 4.3 关节空间 ↔ 笛卡尔空间切换延时

**设计原因**: 关节空间和笛卡尔空间规划使用不同的约束模型，连续执行时轨迹衔接处可能出现瞬时速度跳变。延时给控制器一个稳定窗口。

**配置**: `joint_cartesian_switch_delay_sec`，不同 Worker 有不同默认值:
- tool_changer: 0.05s
- execute_grasp_pose_worker: 0.2s

### 4.4 IO 引脚分配

**规则**: `io_type=digital_output`, `io_index + 32 → 硬件引脚`

| 逻辑引脚 | 硬件引脚 | 使用者 | 用途 |
|---------|---------|--------|------|
| 2 | 34 | LatteNode | 打花开关 |
| 4 | 36 | LatteNode | 咖啡开关 |
| 6 | 38 | ExecuteGraspPose / PublishGraspsClient | 夹爪开/关 |
| 7 | 39 | GripperSwapWorker | 快换盘锁紧/释放 |

**注意事项**:
- ExecuteGraspPoseWorker 的 IO 语义与 PublishGraspsClientWorker **相反** (`true=打开` vs `true=闭合`)，源于气动/电动夹爪不同工位的电气接线差异
- 仿真模式通过 `simulation_skip_io=true` 或 `kSkipTemporaryGripperIo=true` 跳过 IO 调用

### 4.5 轨迹插值器设计 (五次多项式)

**aubo_robot_simulator_ros2**:
- 算法: 五次多项式 (quintic) 插值，位置/速度/加速度连续
- 频率: 200Hz (与 MoveIt sample_duration 1/200 一致)
- 边界平滑: 新轨迹到达时用 50ms-150ms 五次多项式 blend，避免位置突变
- 流速调节: 根据 `rib_buffer_size` 和 `minimum_buffer_size` 节流，防止溢出控制器缓冲区

### 4.6 GraspNet 坐标系转换

**GraspNet 输出**: 旋转矩阵列含义:
```
col0 = approach (手指伸出方向)
col1 = width (左右方向)  
col2 = height (上下方向)
```

**ROS 末端标准**: 
```
Z = approach (手指伸出)
X = width (左右)
Y = height (上下)
```

**转换公式**: `R_ros = [col1, col2, col0]` — 列重排

**GraspNet 180° 修正** (`kQuatZ180`):
- 网络输出的抓取系与真实夹爪在绕接近轴（局部 Z）上常差 π (180°)
- **修复**: 对 GraspNet 位姿在规划前右乘四元数 `(0, 0, 1, 0)`（绕 Z 轴 180° 旋转）
- **应用范围**: C++ 端 `publish_grasps_client_worker`；Python 端 `grasp_motion_controller._apply_grasp_z_flip_180()`
- **注意**: 关节空间回退时需要取消 180° 修正，避免多转半圈。Python 端 `_pose_unflip_if_needed()` 处理此情况。

**normalizeYawToPlusMinusHalfPi** (C++ 端特有):
- 将绕 Z 轴的偏航角约束到 (-π/2, π/2]，等价于选择 ±90° 短路径
- 避免绕 Z 轴旋转超过半圈导致无效运动

**gripper_tip → end_effector 补偿**:
- GraspNet 输出的是夹爪指尖 (gripper_tip) 位姿
- 末端执行器 (end_effector / tool_tcp) 在夹爪指尖后方
- 补偿量: Python 端 0.15m (沿抓取 Z 轴负向)，C++ Worker 端 0.01m
- 变换: `构建局部 4×4 齐次变换矩阵 T_local[2,3] = -grasp_z_offset` → `T_base_target = T_base_grasp @ T_local`

### 4.7 抓取窗口策略

**Python 版** (publish_grasps_client.py):
- 窗口大小: 5 组
- 最少入库: 3 组
- 选优策略: 垂直度得分 (approach 与 -Z 对齐度)，取最高

**C++ 版** (publish_grasps_client_worker.cpp):
- 相同窗口策略
- 额外: `gripper_tip → end_effector` z 轴补偿 (0.15m Python, 0.01m C++ Worker)

---

## 5. 工作空间包结构速查

| 包名 | 语言 | 功能 |
|------|------|------|
| `aubo_driver_ros2` | C++ | AUBO 真机驱动 (TCP→CANbus 通信) |
| `aubo_robot_simulator_ros2` | Python | 五次多项式轨迹插值器 (200Hz) |
| `aubo_moveit_config` | YAML/Launch/Python | MoveIt2 配置包 (URDF/SRDF/kinematics/planners) |
| `aubo_description` | URDF/XACRO | AUBO E5 机械臂模型与 mesh |
| `demo_driver` | C++ | demo 服务节点: 运动/规划/抓取/IO |
| `graspnet_ros2` | Python | GraspNet 推理 + 抓取运动控制 |
| `visual_pose_estimation` | Python | 视觉位姿估计 (VPE) + Web 服务 |
| `hand_eye_calibration` | Python | 手眼标定 (custom + OpenCV 算法) |
| `tool_changer` | C++ | 夹爪快换 Worker + 场景测试 |
| `coffee_latte_demo` | Python | 咖啡拉花 IO 演示 |
| `vision_perception` | Python | YOLO 目标检测/跟踪 |
| `aubo_ros2_web_dashboard` | Python | Web 仪表盘 (FastAPI 静态网关) |
| `ros_arm_tutorials` | C++/Python | ROS2 入门教程 (base/advance/xarm) |
| `robotwebtools` | JS | Web 前端 (roslibjs/ros3djs) |

---

## 6. 手眼标定特性

### 6.1 数据流

```
percipio_camera → /camera/color/image_raw (sensor_msgs/Image)
    → image_data_bridge → /image_data (percipio_camera_interface/ImageData)
        → hand_eye_calibration_node
    → /camera/depth/image_raw → hand_eye_calibration_node (深度校验)
/aubo_driver/robot_status → hand_eye_calibration_node (机器人位姿)
```

### 6.2 两种标定算法

| 算法 | 实现 | 特点 |
|------|------|------|
| **custom** | SVD 初始估计 + scipy.optimize.least_squares (Levenberg-Marquardt) | XY+Z 约束，参数化: 轴角(3)+平移(3)；残差 = T_base_shot @ T_cam2gripper @ P_cam - P_base_actual |
| **opencv** | cv2.calibrateHandEye() | 标准算法 (TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS)，AX=XB 约束验证 |

### 6.2.1 Custom 算法求解流程

```
输入: shot_pose, pick_poses[], camera_points_shot[] (mm)
                      ↓
1. 角点质量检查: 重投影误差<2px, 深度300-2000mm, 深度误差<10%
2. Z值: fixed_z = mean(pick_poses[].z) → 棋盘格桌面高度
3. 初值: SVD 分解 (H = cameras_centered.T @ grippers_centered → R_init = Vt.T @ U.T)
4. 优化: scipy.least_squares(method='lm', max_nfeval=10000)
   参数: [ax,ay,az, tx,ty,tz] (轴角+平移, 共6参数)
   残差: T_base_shot @ T_cam2gripper @ P_cam - P_base_actual (XY+Z约束, 3N维)
                      ↓
输出: T_camera2gripper (4×4, 单位mm)
```

### 6.2.2 OpenCV 算法求解流程

```
输入: 位姿数据 (robot_pose + board_pose 配对)
                      ↓
1. prepare_data():
   单位转换: m(机器人) → 去重(0.1mm容差) → rvec/tvec提取
   优先用 solvePnP 原始输出, 否则 cv2.Rodrigues 转换
2. solve_hand_eye():
   cv2.calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs)
   5算法: TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS, 默认 TSAI
   校验: det(R)≈1, Z值(100-800mm预期)
   输出: t_cam2gripper (m→mm)
3. calculate_errors():
   AX=XB 验证: 每对 A@X - X@B
   平移误差(mm) + 旋转误差(rad/deg) RMS/max/min/std
                      ↓
输出: T_camera2gripper (4×4, mm) + 误差统计
```

### 6.3 标定结果发布 (`hand_eye_calibration_tf_publisher.py`)

**流程**:
1. 从 YAML 文件加载 `T_camera_to_ee` (4×4 变换矩阵, 单位 mm)
2. 验证旋转矩阵行列式 ≈ 1.0
3. 取逆: `T_ee_to_camera = inv(T_camera_to_ee)` — 因为相机固定在末端执行器上
4. 单位转换: `mm → m` (÷1000)
5. 旋转矩阵 → 四元数 (支持 scipy `from_matrix`/`from_dcm`，fallback 到 Shepperd's method 手动转换)
6. `StaticTransformBroadcaster.sendTransform()` 发布一次
7. `spin_once(timeout_sec=1.0)` 然后退出 — 静态 TF 持久存在

**默认 TF 帧**: `ee_link → camera_link`

**文件搜索顺序**: 
1. 参数 `calibration_file` 指定的路径
2. 源码目录 `hand_eye_calibration/config/calibration_results/`
3. install 目录 `share/hand_eye_calibration/config/calibration_results/`

**注意事项**:
- `T_camera_to_ee` 是标定直接输出（相机→末端），但 TF 发布的是 `ee_link→camera_link`（需要取逆）
- YAML 中单位为 mm，TF 发布使用 m（ROS 标准）
- 手动实现的 `_rotation_matrix_to_quaternion` 使用 Shepperd's method，兼容所有 scipy 版本

### 6.4 深度误差校验

- 深度缩放因子: 0.00025 (raw × 0.00025 = m)
- 使用 solvePnP 估计棋盘格 Z 值，与实际深度值对比
- 有效范围: 300-2000mm
- 误差阈值: < 10%

---

## 7. 常见问题与注意事项

### 7.1 服务就绪时序

**问题**: demo_driver 服务启动时 `robot_description` 参数可能尚未就绪。

**解决**: `demo_driver_services.launch.py` 使用 `TimerAction(period=15.0)` 延迟启动所有服务节点。

### 7.2 力控制/抖动问题

**历史记录** (aubo_driver_ros2/doc/):
- `PORTING_MOTION_FIX.md`: ROS1→ROS2 移植中的运动卡顿修复
- `MOTION_JITTER_FIX_SUMMARY_2026-03-18.md`: 轨迹抖动修复总结
- 关键修复: 轨迹间停顿修复、插值节点速度/加速度连续性

### 7.3 超时问题

**历史记录** (aubo_moveit_config/doc/TIMEOUT_ROOT_CAUSE.md):
- MoveIt 规划超时的根因分析

### 7.4 速度因子影响

- 速度因子过小 (如 0.1) 会导致 `minimum_buffer_size=600` 时仍出现卡顿
- 仿真测试可使用 1.0 因子避免此问题

### 7.5 仿真模式

**关键参数**:
- `simulation_skip_io=true`: 跳过实际 IO 调用
- `use_fake_hardware="true"`: 使用 mock hardware
- `moveit_manage_controllers=False`: MoveIt 不管理 ros2_control 控制器生命周期
- `kSkipTemporaryGripperIo=true`: 跳过夹爪 IO (C++ 编译时常量)

### 7.6 路径规划失败处理

**笛卡尔路径**: 3 次重试，每次间隔 0.5s
**关节空间**: 启用 `allowReplanning(true)` 自动重试
**MTC**: 通过 `plan(max_solutions=5)` 生成多个候选解

---

## 8. 数据流管道速查

```
[Motion Pipeline]
MoveIt2 (move_group) 
  → joint_trajectory_controller/follow_joint_trajectory (Action, control_msgs)
  → aubo_ros2_trajectory_action (Action Server)
    → joint_path_command (trajectory_msgs/JointTrajectory)
    → aubo_robot_simulator (五次多项式插值, 200Hz)
      → moveItController_cmd (trajectory_msgs/JointTrajectoryPoint)
      → aubo_driver_ros2 (真机驱动, TCP→CANbus)
        → AUBO 控制器

[Vision Pipeline]
percipio_camera
  → /camera/depth_registered/points (PointCloud2)
  → graspnet_demo_points_node (GraspNet推理, 碰撞检测, NMS)
    → grasp_poses_base (PoseArray, base_link)
    → publish_grasps_client / publish_grasps_client_worker
      → MoveIt2 Cartesian approach → /aubo_driver/set_io (gripper)

[Calibration Pipeline]
percipio_camera → /camera/color/image_raw
  → image_data_bridge → /image_data (ImageData)
  → hand_eye_calibration_node
    → Flask Web :8070 → YAML calibration result
    → hand_eye_calibration_tf_publisher → Static TF (ee_link→camera_link)
```

---

## 9. MoveIt2 历史问题与修复

### 9.1 轨迹执行超时 (TIMEOUT_ROOT_CAUSE)

**现象**: MoveIt 报 `waitForExecution timed out` + `PATH_TOLERANCE_VIOLATED: aborted`，但机械臂实际在运动。

**根因**:
```
allowed_time = trajectory_duration × scaling + margin
```
- 规划轨迹时长约 6s，默认 scaling=1.2 时允许 7.2s
- 实际机械臂受速度/加速度限制，到位时间约 9s
- 7.2s 时 MoveIt 已到允许上界并取消 goal，此时误差仍有 0.5 rad

**修复** (launch 中设置):
- `allowed_execution_duration_scaling`: 1.5 (实验后推荐 5.0)
- `allowed_goal_duration_margin`: 1.0 (实验后推荐 10.0)
- `allowed_start_tolerance`: 0.15

**当前配置** (`aubo_moveit_config/config/moveit_controllers.yaml`):
- `allowed_execution_duration_scaling`: 5.0
- `allowed_goal_duration_margin`: 3.0
- `allowed_start_tolerance`: 0.15
- `trajectory_duration_monitoring`: true

**注意**: `moveit_controllers.yaml` 中的 margin 为 3.0s，而 launch 文件 (`aubo_moveit_pure_ros2.launch.py`) 中可能覆盖为 10.0s。两者不一致，以 launch 文件覆盖值为准（launch 参数优先级高于 YAML）。

### 9.2 运动卡顿与位置突变 (MOTION_JITTER_FIX)

**根因A**: `simulator.trajectory_callback` 收到新轨迹时直接将内部关节状态强制写为新轨迹首点，绕过时间推进 → 位置瞬变

**修复A**: 移除回调中的直接状态覆写，保持由 `motion_worker` 按轨迹时间推进

**根因B**: 驱动发送链路「短时大量突发 + SDK 调用阻塞」→ 发送节拍不均匀 → 卡顿

**修复B**:
- overspeed 分段上限
- 发送侧自适应节流（按队列积压调整批量）
- 诊断查询降频（减少阻塞）
- 批量入队优化

### 9.3 Gazebo+RViz 联合仿真 (参考)

**延迟启动模式**: robot_state_publisher → Gazebo → spawn_entity(TimerAction 5s 延迟) → controller_spawner → RViz(OnProcessExit 触发)

**关键参数**: `use_sim_time:=true` (robot_state_publisher/move_group/rviz), `subscribe_to_robot_description_topic:=true`

### 9.4 Pilz Industrial Motion Planner 特性

**配置**: `pilz_industrial_motion_planner_planning.yaml`

**与 OMPL 的关键区别**:
- 使用 `CommandPlanner` 而非 `OMPLPlanner`
- **不使用 AddTimeOptimalParameterization** 适配器（自己控制时间参数化）
- 共 4 个 adapter: FixWorkspaceBounds → FixStartStateBounds → FixStartStateCollision → FixStartStatePathConstraints
- 支持 3 种运动类型: PTP(点对点/关节空间)、LIN(直线/笛卡尔)、CIRC(圆弧)
- 需要 `has_acceleration_limits: true`（joint_limits.yaml 已配置）
- 笛卡尔限速: 平移 1.0 m/s, 平移加速度 2.25 m/s², 旋转 1.57 rad/s

**capabilities**:
```yaml
pilz_industrial_motion_planner/MoveGroupSequenceAction
pilz_industrial_motion_planner/MoveGroupSequenceService
```

**注意事项**: Pilz 规划器在本项目中作为备选方案，生产环境默认使用 OMPL+RRTConnect。Pilz 的 LIN 运动适合精确笛卡尔直线场景，但其规划时间通常比 OMPL 长。

---

## 10. VPE Web API 架构

### 10.1 调度核心: threading.Event + 后台 spin 线程

```python
# 典型模式 (visual_pose_estimation_python/web/ros_bridge/node_runtime.py)
future = client.call_async(request)
event = threading.Event()
future.add_done_callback(lambda _: event.set())
# FastAPI 线程阻塞在此，不干扰 ROS2
event.wait(timeout=30.0)
# 后台 spin 线程处理响应 → 触发 done callback → event.set()
return future.result()
```

**关键**: FastAPI 线程被 `Event.wait()` 阻塞，ROS2 回调在后台 `spin_once(0.1s)` 中处理。两者互不干扰。

### 10.2 完整 API 清单

| 类别 | 端点 | 超时 | 调用的 ROS2 |
|------|------|------|------------|
| 图像 | `/api/capture_image` | 20s | `/software_trigger` + 临时订阅 depth/color topics |
| 姿态 | `/api/estimate_pose` | 30s | `/estimate_pose` (3D, 含模板匹配+手眼标定) |
| 姿态 | `/api/estimate_pose_2d` | 30s | `/estimate_pose_2d` + 读缓存 depth |
| 机器人 | `/api/set_robot_pose` | 180s | `/move_to_pose` |
| 机器人 | `/api/set_robot_io` | 10s | `/aubo_driver/set_io` |
| 机器人 | `/api/get_robot_status` | 即时 | 读缓存 `/aubo_driver/robot_status` |
| 抓取 | `/api/execute_single_grasp` | 300s | `/execute_single_grasp` |
| 抓取 | `/api/loop_grasp_control` | 10s | `/loop_grasp_control` (SetBool) |
| 抓取 | `/api/publish_grasps_loop_control` | 10s | `/publish_grasps_worker_loop_control` (SetBool) |
| 快换 | `/api/run_gripper_swap` | 可变 | `/run_gripper_swap` |
| 模板 | `/api/list_templates` | 5s | `/list_templates` |
| 模板 | `/api/standardize_template` | 60s | `/standardize_template` |
| 调试 | `/api/debug/get_images` | 5s | 本地 Preprocessor/FeatureExtractor (不走 ROS2!) |

### 10.3 并发安全

| 组件 | 保护机制 |
|------|---------|
| 图像缓存 | `threading.Lock` (image_lock) |
| 机器人状态缓存 | `threading.Lock` (robot_status_lock) |
| ROS2 Future 等待 | 每个 Future 独立 `threading.Event` |
| 前端移动请求 | `_setRobotPoseInFlight` 防重入 |
| 前端循环抓取 | `_loopAutoGraspRunning` 防重复启动 |
| WebSocket 广播 | `asyncio.Lock` |

**已知限制**:
- 长耗时操作中 FastAPI 线程被阻塞，但不干扰 ROS2 调度
- `POST /exit` 在长操作进行中可能延迟响应
- `/api/debug/get_images` 在 HTTP 线程中直接调 OpenCV，避免与 `/api/estimate_pose` 并行

---

## 11. 视觉位姿估计算法

### 11.1 预处理 (Preprocessor)

```
深度图 → 0值插值(补齐法) → 二值化([min,max]) → 8连通域提取 → 
面积/尺寸/长宽比/数量筛选 → 彩色工件抠图
```

**筛选条件** (可调):
- 面积: [10, 100000] px²
- 最小宽/高: 60 px
- 长宽比: [0.3, 4.0]
- 最大保留数量: 3

### 11.2 特征检测 (FeatureExtractor)

```
连通域掩码 → findContours → 工件外接圆(minEnclosingCircle) → 
腐蚀→最大连通域→膨胀→阀体外接圆 → 
atan2(阀体-工件中心) → 标准化角度
```

多线程处理 (默认 4 线程池)

### 11.3 模板匹配

两种模式:
- **特征距离匹配**: 基于工件圆/阀体圆/面积等特征向量的距离
- **暴力匹配** (brute_force_matching): 遍历模板库所有旋转角度 (0°~360°)，找最佳匹配

### 11.4 3D 姿态计算

```
标准化角度 → 相机内参 + 深度图(中心点深度) → 工件3D位置(相机系)
→ T_E_C (手眼标定) → T_B_E (机器人位姿) → T_B_C = T_B_E @ T_E_C
→ 输出 T_B_E_grasp/prep/preplace/place
```

## 12. AUBO 驱动内部机制

### 12.0 线程架构总览

**当前稳定版本的线程布局** (PORTING_MOTION_FIX.md):

| 线程 | 频率 | 职责 |
|------|------|------|
| **主线程** | — | `spin_some` + `updateControlStatus` (计数/`start_move_` 置位) — 仅由 wall timer 触发 |
| **feedToRosMotionLoop** | 200Hz (5ms) | 每周期取 1~3 点 (rib<200 时 3 次)；超时管理 `start_move_` |
| **publishWaypointToRobot** | ~250Hz | 按需刷新 RIB → 按缺额送点 (max_cnt=8) → RIB<200 时 1ms 否则 4ms |
| **timerCallback** | 50Hz | 状态查询、`robot_status` / `rib_status` 发布 |
| **publishJointStateAndFeedbackLoop** | 50Hz | 发布 `joint_states` / `feedback_states` |

**线程安全保护**:

| 资源 | 保护方式 |
|------|----------|
| `buf_queue_` | `buf_queue_mutex_` (所有 push/pop/size/clear 操作均加锁) |
| `rib_buffer_size_` | `std::atomic<int>` |
| `ros_motion_queue_` | `moodycamel::ReaderWriterQueue` (无锁队列) |
| `robot_diagnosis_info_` | `publishWaypointToRobot` 使用局部 `pub_diag` 而非共享的 `rs.robot_diagnosis_info_` — 避免数据竞争 |
| `current_joints_` / `target_point_` | `joints_mutex_` |

### 12.1 两种控制模式

| 模式 | topic 切换 | 通信方式 | 适用场景 |
|------|-----------|---------|---------|
| **RosMoveIt** (默认) | `/aubo_driver/controller_switch` (Int32) | TCP→CANbus 透传 (RIB 缓冲) | 正常 MoveIt 轨迹执行 |
| **AuboAPI** (直接API) | 同上 | 直接调 AUBO SDK API | 单点运动/调试 (`aubo_direct_api_node`) |

**切换**: 通过 `/aubo_driver/controller_switch` topic (Int32)
**RIB 缓冲**: 400 个点，按 6 个为一组消费 (即 ~67 组)

### 12.2 驱动内部数据流

```
moveItController_cmd (trajectory_msgs/JointTrajectoryPoint, depth 20000)
    → moveItPosCallback: 入队 buf_queue_ (mutex 保护)
    → feedToRosMotionLoop (5ms/200Hz): buf_queue_ → ros_motion_queue_
    → publishWaypointToRobot 线程:
        实时查询 RIB(current_macsz)
        → robotServiceSetRobotPosData2Canbus (TCP→CANbus)
        → 机器人控制器 (RIB 缓冲 400)
```

### 12.3 关键内部量

| 名称 | 类型 | 含义 |
|------|------|------|
| `buf_queue_` | 队列 | 来自插值节点的轨迹点（moveItPosCallback 写入） |
| `rib_buffer_size_` | `std::atomic<int>` | 控制器 RIB 缓冲量（publishWaypointToRobot 实时查询） |
| `ros_motion_queue_` | 无锁队列 | 待送机关的节点（setRobotJointsByMoveIt 写入） |
| `start_move_` | bool | 运动中为 true，轨迹完成变 false |
| `buffer_size_` | 400 | 预缓冲阈值 |

### 12.4 运动异常诊断指标 (快速参考)

| 指标 | 正常范围 | 异常 |
|------|---------|------|
| 取点间隔 (feeder) | ~2ms (500Hz) | >10ms 尖峰 → 线程竞争 |
| 插值发布间隔 | ~5ms (200Hz) | >20ms 或 4s 尖峰 |
| RIB 缓冲量 | 几十~400 | 运动中降至 0 → 送点不足 |
| TCP 送点耗时 | 平均<20ms | >50ms 尖峰 → 阻塞导致漏送 |
| `start_move_` | 单轨迹内始终 true | 变为 false → 误关 feeder |
| `ros_motion_queue_` | 有数据流动 | 长时间为 0 但 buf_queue_ 有数据 → 卡在 feeder |

> 详细诊断方法参见 PORTING_MOTION_FIX 日志约定；完整修复历史参见 §12.6 的 13 轮迭代表。

### 12.5 Direct API 节点

`aubo_direct_api_node` 完全不走 TCP2CAN 跟随，直接调 AUBO SDK 原生 API：
- 服务: `/aubo_driver/direct/get_current_state`, `/aubo_driver/direct/set_robot_enable`, `/aubo_driver/direct/set_speed_factor`, `/aubo_driver/direct/set_robot_pose`
- 话题: `/aubo_driver/direct_joint_cmd` (Float32MultiArray), `/aubo_driver/direct/robot_status`
- 参数: `server_host`(默认 169.254.10.98), `server_port`(8899), `username`/`password`, `auto_startup`

### 12.6 publishWaypointToRobot 核心送点算法

这是驱动最关键的循环，经过 13 轮迭代才达到稳定:

```
按需 RIB 刷新 (RIB≤0 或 diag_age 超时)
  → robotServiceGetRobotDiagnosisInfo (TCP往返, ~2-12ms)
  → 判断 current_macsz < expect_macsz (400)
    → 计算缺额: cnt = min(8, ceil((400 - current_macsz) / 6))
    → tryPopWaypoint(cnt) → robotServiceSetRobotPosData2Canbus (TCP)
  → RIB<200 且队列非空: sleep 1ms (加速灌点)
  → 否则: sleep 4ms (正常 ~250Hz)
```

**13 轮迭代修复要点**:

| 迭代 | 关键修改 | 解决的问题 |
|------|----------|-----------|
| Fix1 | 分离取点线程 feedToRosMotionLoop (500Hz) | 连续卡顿、突然加速 |
| Fix2 | Python 插值: 整数步数 + wall-clock 补偿 | 发布间隔不稳定 |
| Fix3 | start_move_ 双队列空才关 | 误关运动标志 |
| Fix4 | buffer_size_=5 (错误尝试) | 引入严重抖动 |
| Fix5-11 | 各种 RIB 缓存/流控策略迭代 | 吞吐不足、RIB 过冲 |
| **Fix12** | **恢复 ROS1 架构: 实时 RIB + buffer_size_=400 + 局部 pub_diag** | 连续/偶发卡顿解决 |
| **Fix13** | **RIB=0 时 sleep 1ms (原 10ms); feeder 按 rib 调节取点量** | 单条轨迹内停顿消除 |

### 12.7 ROS1→ROS2 移植核心教训

**最大教训: 不要"优化"已验证的硬件通信协议设计。**

ROS1 在 publisher 热循环中实时查询 RIB 看似低效 (每次 TCP 往返)，但这确保了流控的准确性。ROS2 移植时的缓存方案破坏了流控精度:

| 项目 | ROS1 原始 | ROS2 移植初版(错误) | 后果 |
|------|-----------|-------------------|------|
| RIB 查询 | 热循环中实时调用 | 50Hz timer 缓存 | 缓存滞后 → RIB 耗尽 |
| buffer_size_ | 400 | 曾改为 5~60 | 预缓冲不足 |
| publisher 间隔 | 4ms (250Hz) | 曾改为 2ms | 速率不匹配 |
| RIB=0 时 sleep | 10ms | 无/10ms | 叠加成数秒停顿 |
| robot_diagnosis_info_ | 单线程访问 | 多线程并发读写 | 数据竞争 |

### 12.8 start_move_ 双路径启动

| 路径 | 条件 | 理论触发时间 |
|------|------|-------------|
| **A: moveItPosCallback** | `buf_queue_.size() > buffer_size_` (400) | 401点 @ 200Hz ≈ 2s |
| **B: updateControlStatus** | `data_count_ == 50` 且 `bq > 0` | 50 × 2ms = 100ms |

**ROS1**: 单线程，路径 B 先触发 (约 100ms)。
**ROS2**: 多线程，`moveItPosCallback` 在独立 listener 线程，路径 B 可能不触发 → 实际 2s 延迟。

**修复**: `data_count_` → `std::atomic<int>`；首点入队时 `store(0)`；`updateControlStatus` 移到 500Hz wall timer。

### 12.9 运动异常诊断速查 (含 ros_motion_queue_)

| 指标 | 正常 | 异常 |
|------|------|------|
| 取点间隔 `interval_ms` | ~2ms (500Hz) | >10ms 尖峰 → 线程竞争 |
| 插值发布间隔 | ~5ms (200Hz) | >20ms 或 4s 尖峰 → Python 线程阻塞 |
| RIB 缓冲 `current_macsz` | 几十~400 | 运动中降至 0 → 送点不足 |
| TCP 送点耗时 | 平均 <20ms | >50ms 尖峰 (如 225ms) → 阻塞漏送 |
| `ros_motion_queue_` | 有数据流动 | 长时间为 0 但 buf_queue_ 有数据 → 卡在 feeder |
| `start_move_` | 单轨迹内始终 true | 轨迹未结束变 false → 误关 |

## 13. MoveIt2 OMPL 规划配置

### 11.1 OMPL 规划器完整清单 (来自 ompl_planning.yaml)

| 规划器 | OMPL 类型 | max_iterations | max_seconds | 额外参数 |
|--------|-----------|----------------|-------------|---------|
| **RRTstar** | geometric::RRTstar | 5000 | 2.0 | goal_bias=0.1, range=0.0, delay_collision_checking=1, optimize=true, cost_threshold=1e-6 |
| **RRTConnect** (默认) | geometric::RRTConnect | 3000 | 1.0 | range=0.0 |
| RRT | geometric::RRT | 2000 | 1.0 | goal_bias=0.05, range=0.0 |
| AnytimePathShortening | geometric::AnytimePathShortening | — | — | shortcut=true, hybridize=true, max_hybrid_paths=12, num_planners=2, planners="RRTConnect+RRTstar" |
| SBL | geometric::SBL | 2000 | 1.0 | range=0.0 |
| EST | geometric::EST | 2000 | 1.0 | range=0.0, goal_bias=0.05 |
| LBKPIECE | geometric::LBKPIECE | 2000 | 1.0 | range=0.0, border_fraction=0.8, min_valid_path_fraction=0.5 |
| BKPIECE | geometric::BKPIECE | 2000 | 1.0 | range=0.0, 同上+failed_expansion_score_factor=0.5 |
| KPIECE | geometric::KPIECE | 2000 | 1.0 | goal_bias=0.05, 同上 |
| TRRT | geometric::TRRT | 3000 | 1.5 | goal_bias=0.05, temp_change_factor=2.0, init_temperature=1e-5 |
| PRM | geometric::PRM | 3000 | 1.5 | max_nearest_neighbors=8 |
| PRMstar | geometric::PRMstar | 4000 | 2.0 | max_nearest_neighbors=8, optimize=true |
| FMT | geometric::FMT | 3000 | 2.0 | num_samples=800, radius_multiplier=1.0 |
| BFMT | geometric::BFMT | 3000 | 2.0 | num_samples=800, balanced=0, optimality=1 |
| PDST | geometric::PDST | 2000 | 1.0 | — |
| STRIDE | geometric::STRIDE | 2000 | 1.0 | range=0.0, goal_bias=0.05, degree=16 |
| BiTRRT | geometric::BiTRRT | 2000 | 1.0 | init_temperature=100, temp_change_factor=0.1 |
| LBTRRT | geometric::LBTRRT | 3000 | 1.5 | range=0.0, goal_bias=0.05, epsilon=0.3 |
| BiEST | geometric::BiEST | 2000 | 1.0 | range=0.0 |
| ProjEST | geometric::ProjEST | 2000 | 1.0 | range=0.0, goal_bias=0.05 |
| LazyPRM | geometric::LazyPRM | 2000 | 1.0 | range=0.0 |
| LazyPRMstar | geometric::LazyPRMstar | 3000 | 1.5 | optimize=true |
| SPARS | geometric::SPARS | 3000 | 2.0 | stretch_factor=2.0, sparse_delta_fraction=0.25 |
| SPARStwo | (非 geometric 命名空间) | 4000 | 2.0 | 同上, max_failures=4000 |

**manipulator 组配置**:
- 默认规划器: `RRTConnect`
- 候选: `[RRTstar, AnytimePathShortening, RRTConnect, PRMstar]`
- `projection_evaluator: joints(shoulder_joint, upperArm_joint)` — 使用前两个关节做状态空间投影
- `longest_valid_segment_fraction: 0.005` — 碰撞检测粒度

**endeffector 组配置**:
- 默认规划器: `RRTstar`
- 候选: `[RRTstar, AnytimePathShortening, RRTConnect]`
- `longest_valid_segment_fraction: 0.005`

### 11.2 全局规划参数 (launch 覆盖)

```python
ompl_planning["move_group"].update({
    "sample_duration": 1/motion_command_hz,  # 0.005s @ 200Hz
    "planning_time": 15.0,                    # 最大规划时间
    "max_planning_attempts": 10,              # 最大重试次数
    "resample_dt": 0.05,                      # 路径重采样间隔
})
```

### 11.3 请求适配器 (按顺序执行)

```
1. AddTimeOptimalParameterization  → 时间最优参数化 (速度/加速度约束)
2. ResolveConstraintFrames        → 约束坐标系解析
3. FixWorkspaceBounds             → 工作空间边界修正 (default: 10m×10m×10m)
4. FixStartStateBounds            → 起始状态边界修正 (max_bounds_error: 0.1)
5. FixStartStateCollision         → 起始状态碰撞检查
6. FixStartStatePathConstraints   → 起始路径约束修正
```

**适配器顺序至关重要**: 每个适配器的输出是下一个的输入，顺序错误会导致规划失败。

### 11.4 move_group 能力

```python
move_group_capabilities = {
    "capabilities": "move_group/ExecuteTaskSolutionCapability"
}
```

**含义**: 启用 MTC (MoveIt Task Constructor) 任务执行能力。生产环境未使用 MTC，仅教学演示使用。

### 11.5 状态空间投影 (Projection Evaluator)

`projection_evaluator: joints(shoulder_joint, upperArm_joint)`

**用途**: OMPL 规划器使用投影来度量状态空间距离。选择 shoulder_joint + upperArm_joint 作为投影维度，因为这两个关节对末端位置的影响最大。

**注意事项**:
- 投影维度越少，规划越快但路径质量可能下降
- 6 轴机械臂通常选择 2-3 个主要关节做投影
- 此配置影响所有使用 `setGoalTolerance()` 的规划请求

---

## 14. VPE Web 架构: algorithm_http_server_node

### 14.1 节点架构

```
FastAPI(/api/*) → NativeWebService → ROS2Node("algorithm_http_server_node")
                                         ├─ Clients (按需):
                                         │  /estimate_pose, /estimate_pose_2d
                                         │  /move_to_pose, /aubo_driver/set_io
                                         │  /execute_single_grasp, /loop_grasp_control
                                         │  /publish_grasps_worker_loop_control
                                         │  /run_gripper_swap, /software_trigger
                                         │  /list_templates, /standardize_template
                                         │  /update_params
                                         ├─ Subscriptions (持久):
                                         │  /aubo_driver/robot_status → latest_robot_status
                                         │  /image_data → latest_image_data
                                         │  /camera/depth/*, /camera/color/* (临时, 拍照时)
                                         └─ 本地算法:
                                            Preprocessor, FeatureExtractor,
                                            ConfigReader, ParamsManager
```

### 14.2 核心模式: call_async + _spin_future

```python
def _spin_future(self, future, timeout_sec):
    """自旋等待异步 ROS2 调用完成"""
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    deadline = time.time() + timeout_sec
    while not event.is_set() and time.time() < deadline:
        rclpy.spin_once(self, timeout_sec=0.01)
    return future.done()
```

**关键区别**: `main.py` 的 `visual_pose_estimation_python` 节点使用 `MultiThreadedExecutor`，而 Web 端的 `algorithm_http_server_node` 使用手动 `spin_once` 循环。两者互不干扰。

### 14.3 图像采集流程

```
POST /api/capture_image
  → NativeWebService.capture_image() → ROS2Node.capture_image(camera_id)
    → 临时订阅 /camera/depth/image_raw, /camera/color/image_raw
    → /software_trigger(call_async)
    → 轮询等待 depth+color 均收到 (timeout=10s)
    → 清理临时订阅 → cv2.imencode → base64 返回
```

### 14.4 服务超时汇总

| API 端点 | ROS2 服务 | 超时 |
|----------|----------|------|
| capture_image | /software_trigger | 20s |
| estimate_pose | /estimate_pose | 30s |
| estimate_pose_2d | /estimate_pose_2d | 30s |
| set_robot_pose | /move_to_pose | 180s |
| set_robot_io | /aubo_driver/set_io | 10s |
| execute_single_grasp | /execute_single_grasp | 300s |
| loop_grasp_control | /loop_grasp_control | 10s |
| run_gripper_swap | /run_gripper_swap | 可配置 |
| standardize_template | /standardize_template | 60s |

### 14.5 ROS2Node 可选依赖模式

Web 服务的 `ROS2Node` 使用 try/except 优雅处理可选依赖:

```python
try:
    from percipio_camera_interface.msg import ImageData
    from percipio_camera_interface.srv import SoftwareTrigger
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    ImageData = None; SoftwareTrigger = None

# 然后按需创建客户端
self.trigger_client = self.create_client(SoftwareTrigger, "/software_trigger") if CAMERA_AVAILABLE else None
self.robot_status_sub = self.create_subscription(RobotStatus, ...) if ROBOT_AVAILABLE else None
```

**设计优点**: 在缺少相机 SDK、机器人驱动接口等可选依赖时，节点仍能启动，只是部分功能不可用。Web API 返回错误码而非崩溃。

### 14.6 图像采集临时订阅模式

```python
# capture_image 流程
1. 创建临时 depth/color 订阅 (QoS: BEST_EFFORT + KEEP_LAST(1))
2. 调用 /software_trigger 触发相机拍照
3. 轮询等待 depth_image_received AND color_image_received (timeout=10s)
4. cv2.imencode → base64 返回给 Web 前端
5. 清理临时订阅 (destroy_subscription)
```

**关键**: 临时订阅避免持久占用带宽；拍照完成后立即清理。

---

## 15. 构建系统与包配置

### 15.1 colcon 构建要点

**常用构建命令**:
```bash
# 完整构建
colcon build --symlink-install

# 选择包构建
colcon build --packages-select demo_driver --symlink-install

# 跳过失败包继续构建
colcon build --continue-on-error

# 并行构建 (默认使用所有核心)
colcon build --parallel-workers 4
```

**`--symlink-install`**: Python 包使用符号链接而非复制，修改 Python 源码无需重新构建。

### 15.2 package.xml 关键字段

```xml
<package format="3">
  <name>package_name</name>
  <version>0.1.0</version>
  
  <!-- 构建依赖 -->
  <buildtool_depend>ament_cmake</buildtool_depend>       <!-- C++ -->
  <buildtool_depend>ament_python</buildtool_depend>      <!-- Python -->
  
  <!-- 依赖类型 -->
  <depend>rclcpp</depend>              <!-- 构建+导出+执行都需要 -->
  <build_depend>rosidl_default_generators</build_depend>  <!-- 仅构建时需要 -->
  <exec_depend>rosidl_default_runtime</exec_depend>       <!-- 仅执行时需要 -->
  <test_depend>ament_lint_auto</test_depend>              <!-- 仅测试时需要 -->
</package>
```

### 15.3 Python setup.py 关键字段

```python
setup(
    name='package_name',
    packages=['package_name'],           # 需要安装的 Python 包
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'node_exe = package_name.module:main',   # ros2 run 使用的入口点
        ],
    },
)
```

**注意事项**:
- `data_files` 中的 launch 和 config 文件必须显式声明才会安装到 `install/` 目录
- console_scripts 入口点格式: `命令名 = 包名.模块名:函数名`
- 缺少 data_files 声明是最常见的 "文件找不到" 错误原因

---

## 16. GraspNet 推理流水线

### 16.1 节点架构 (`graspnet_demo_points_node.py`)

```
percipio_camera → /camera/depth_registered/points (PointCloud2)
    → graspnet_demo_points_node
        → PointCloud2 → numpy (sensor_msgs_py.point_cloud2)
        → GraspNet 推理 (end_points['point_clouds'])
        → pred_decode → GraspGroup
        → ModelFreeCollisionDetector (碰撞检测)
        → NMS + sort_by_score
        → 发布:
            - grasp_poses_base (PoseArray, base_link 下)
            - MarkerArray (RViz 可视化)
            - TF (抓取坐标系广播)
            - Open3D 独立进程可视化 (可选)
```

### 16.2 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 模型 | `minkuresunet` (默认) | GraspNet backbone |
| 输入点云 | N×3 float32 (m) | 来自 PointCloud2 话题 |
| 碰撞检测 | `voxel_size=0.005`, `collision_thresh=0.01` | ModelFreeCollisionDetector |
| NMS | `nms(translate=0.03, rotation=30°)` | 非极大值抑制 |
| 排序 | `sort_by_score()` | 按抓取得分降序 |

### 16.3 坐标变换链

```
camera_depth_optical_frame → base_link (TF)
    graspnet_demo_points_node 订阅此 TF
    → 将抓取位姿从 camera 系转到 base_link 系
    → 发布 grasp_poses_base (base_link 坐标系)
```

---

## 17. 开发工作流建议

### 17.1 启动顺序检查清单

1. **基础环境**: `source /opt/ros/humble/setup.bash`
2. **工作空间**: `source install/setup.bash`
3. **真机驱动**: `ros2 launch aubo_driver_ros2 aubo_driver.launch.py`
4. **MoveIt2**: `ros2 launch aubo_moveit_config demo.launch.py`
5. **仿真插值器**: `ros2 run aubo_robot_simulator_ros2 aubo_robot_simulator_node`
6. **视觉感知**: `ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py`
7. **Web 服务**: `ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py`
8. **工具快换**: `ros2 launch tool_changer gripper_swap_worker.launch.py`
9. **Demo 服务**: `ros2 launch demo_driver demo_driver_services.launch.py`

### 17.2 调试技巧

| 工具 | 用途 |
|------|------|
| `ros2 topic list` | 列出所有活跃话题 |
| `ros2 topic echo /topic` | 查看话题实时数据 |
| `ros2 topic hz /topic` | 检查话题发布频率 |
| `ros2 service list` | 列出所有可用服务 |
| `ros2 service call /srv_name type '{...}'` | 手动调用服务测试 |
| `ros2 node info /node_name` | 查看节点的话题/服务/参数 |
| `ros2 param list /node_name` | 查看节点所有参数 |
| `ros2 run rqt_reconfigure rqt_reconfigure` | 动态参数调节 (GUI) |
| `ros2 run rqt_tf_tree rqt_tf_tree` | 可视化 TF 树 |
| `rviz2` | 可视化机器人状态/规划/点云/Marker |

### 17.3 常见调试场景

**场景 1: MoveIt 规划一直失败**
- 检查 `ros2 topic echo /joint_states` 确认关节状态在发布
- 检查 TF 树: `ros2 run rqt_tf_tree rqt_tf_tree`
- 检查目标位姿是否在可达工作空间内
- 检查 `longest_valid_segment_fraction` 是否过大

**场景 2: 轨迹执行超时**
- 增大 `allowed_execution_duration_scaling` 和 `allowed_goal_duration_margin`
- 检查 `joint_limits.yaml` 中的速度/加速度限制是否过小
- 确认 `default_velocity_scaling_factor` 不等于 0.1 (10% 太慢)

**场景 3: 机械臂卡顿/抖动**
- 检查 RIB 缓冲量: 运动中应保持非零值
- 检查 `moveItController_cmd` topic 频率是否为 200Hz
- 检查是否有线程竞争导致取点间隔尖峰 >10ms

**场景 4: 视觉位姿估计无结果**
- 检查 `/camera/color/image_raw` 和 `/camera/depth/image_raw` 话题
- 检查光照条件 (图像过暗或过曝)
- 检查 `depth_scale` 参数是否正确 (默认为 0.00025)
- 检查手眼标定 TF 是否已发布

---

## 18. PlanningScene 动态障碍物

### 18.1 添加障碍物 (`publish_obstacle.py` 模式)

**两种发布渠道**:
```python
# 方法1: 直接发布 CollisionObject 到 Monitoring 话题
collision_object_publisher = create_publisher(CollisionObject,
    '/move_group/planning_scene_monitor', 10)

# 方法2: 发布完整 PlanningScene (is_diff=True)
planning_scene_publisher = create_publisher(PlanningScene,
    '/move_group/publish_planning_scene', 10)
```

**关键步骤**:
1. 创建 `CollisionObject`，设置 `id`、`header.frame_id`、`primitives`、`primitive_poses`
2. `operation = CollisionObject.ADD`
3. **发布3次** (间隔 200ms) 确保 move_group 处理
4. `is_diff = true` 表示增量更新而非替换整个场景

**注意事项**:
- `frame_id` 必须与规划参考坐标系一致（本机用 `world`）
- 障碍物 ID 必须唯一，重复 ID 会覆盖之前的对象
- 移除障碍物用 `operation = CollisionObject.REMOVE`
- `PlanningSceneMonitor` 会自动更新碰撞检测

## 19. ExecuteGraspPoseWorker 详解

### 19.1 双模式抓取

| 模式 | 触发条件 | 数据来源 |
|------|---------|---------|
| **视觉估计** (默认) | `use_vision=true` | `/estimate_pose` 服务 (VPE) |
| **参数常量** | `use_vision=false` | launch 参数: `egp_grasp_position/egp_grasp_orientation` |

### 19.2 视觉估计流程

```
1. 调用 /estimate_pose 服务 (最多重试3次, 间隔1s)
2. 从响应中提取 grab_position, prep_position, preplace_position, place_position
3. 应用 gripper_tip→EEF 补偿 (egp_grasp_z_offset, 默认0.01m)
4. 抓取接近: XY → 姿态 → Z下降 (笛卡尔)
5. 关IO夹取
6. Z抬起 lift_offset (0.2m)
7. 移动到安全位 camera_pose
8. 移动到放置点
9. 开IO释放
10. 返回 camera_pose
```

### 19.3 关键参数表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `egp_grasp_z_offset` | 0.01m | gripper_tip→EEF Z轴补偿 |
| `egp_height_above` | 0.1m | 抓取点上方安全高度 |
| `egp_lift_offset` | 0.2m | 抓取后Z轴抬起 |
| `egp_place_offset_y` | -0.2m | 放置点Y偏移 |
| `egp_place_offset_z` | -0.15m | 放置点Z偏移 |
| `egp_joint_velocity_scaling` | 0.7 | 速度缩放 |
| `egp_joint_acceleration_scaling` | 0.3 | 加速度缩放 |
| `egp_cartesian_max_points` | 40 | 笛卡尔轨迹点数上限 |
| `egp_joint_cartesian_switch_delay_sec` | 0.2 | 关节↔笛卡尔切换延时 |
| `kExecuteGraspZMinLimit` | 0.19m | 抓取Z最低限 (防止碰撞桌面) |

### 19.4 调试开关

```cpp
static constexpr bool kUseTemporaryFixedGraspPose = false;  // true=忽略视觉，用硬编码位姿
static constexpr bool kSkipTemporaryGripperIo = false;      // true=跳过IO调用
```

切回真实流程时两项都改为 `false`。

## 20. TF2 使用模式 (本地总结)

### 20.1 StaticTransformBroadcaster (一次性发布)

**适用场景**: 固定不变的变换（手眼标定结果、虚拟关节）

```python
# 发布一次即可，静态TF持久存在
tf_broadcaster = StaticTransformBroadcaster(node)
tf_broadcaster.sendTransform(transform_stamped)
rclpy.spin_once(node, timeout_sec=1.0)  # 给时间发送
# 不需要持续spin，静态TF被系统缓存
```

**本地使用**: `hand_eye_calibration_tf_publisher.py` — 发布 `ee_link → camera_link`；`demo.launch.py` — 发布 `world → base_link`

### 20.2 TransformListener + Buffer (动态查询)

**适用场景**: 查询正在变化的TF（获取当前末端位姿）

```python
# 创建TF监听器并等待数据积累
buffer = Buffer()
listener = TransformListener(buffer, node)
time.sleep(0.15)  # 冷启动延迟，等待TF数据

# 查找变换 (最多重试20次)
for attempt in range(20):
    t = buffer.lookup_transform(target_frame, source_frame, 
                                 rclpy.time.Time(), 
                                 rclpy.duration.Duration(seconds=0.12))
    if t: break
    time.sleep(0.03)
```

**本地使用**: `grasm_motion_controller._get_current_ee_pose()` — 查询 `base_link → tool_tcp`

### 20.3 坐标系约定

```
world (固定世界)
  └── base_link (机器人基座, StaticTransform identity)
        └── shoulder_joint → upperArm_joint → foreArm_joint
              → wrist1_joint → wrist2_joint → wrist3_Link
                    └── tool_tcp (TCP, fixed joint: z=+0.0235m)
                    └── kuaihuan_Link (快换盘)
                          └── {gripper}X_Link (工具, fixed joint: z=+0.02m)

camera_depth_optical_frame (相机坐标系)
  └── ee_link → camera_link (手眼标定 Static TF)
```

### 20.4 TF 时间戳与查找策略

- `lookup_transform(..., rclpy.time.Time(), ...)`: Time() 使用最新可用变换
- 超时 `Duration(seconds=0.12)` 表示等待最多 120ms
- 若 TF 尚未发布，lookup_transform 会阻塞等待直到超时

---

## 21. MoveIt2 RViz 操作指南

### 21.1 关键面板

| 面板 | 用途 |
|------|------|
| **MotionPlanning** | 设置规划组、起始/目标状态、规划并执行 |
| **Trajectory Slider** | 步进查看轨迹各点、播放 |
| **RvizVisualToolsGui** | 手动逐步控制 demo |
| **Displays → Scene Robot** | 显示当前规划场景中的机器人 |
| **Displays → Planned Path** | 显示规划出的轨迹 (Show Trail 显示轨迹线) |

### 21.2 交互与操作

- **Query Start State** (绿色标记): 拖拽设置起始关节配置
- **Query Goal State** (橙色标记): 拖拽设置目标位姿
- 使用 **Interact** 工具拖拽交互标记
- **Use Collision-Aware IK**: 勾选后 IK 求解器避免自碰撞（碰撞处仍显示红色但能求解）
- **Use Cartesian Path**: 勾选后强制直线笛卡尔运动
- 超出工作空间时拖拽无反应（IK 无解）
- 开始/目标状态间有自碰撞时关节变红

### 21.3 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 拖动无反应 | 目标超出可达空间 | 缩小目标/移动更近 |
| 规划失败 | 自碰撞或 IK 无解 | 调整姿态/关 Collision-Aware IK 试试 |
| **Play 回放旧轨迹** | 改变目标后未重新 Plan | 必须先 Plan 再 Execute |
| 轨迹不显示 | Planned Path 未勾选 | 检查 Displays 树 |
| Fixed Frame 错误 | 未设置为 base_link/world | 在 Global Options 中设置 |

---

## 22. 执行抓取坐标系统

### 22.1 完整变换链

```
T_base_grasp = T_base_EE * T_EE_camera * T_camera_object * T_object_grasp
```

其中:
- `T_base_EE`: 机器人末端位姿（从 /joint_states + FK 或 TF 获取）
- `T_EE_camera`: 手眼标定结果 (ee→camera, 需对标定结果取逆)
- `T_camera_object`: 视觉位姿估计结果 (VPE  `/estimate_pose`)
- `T_object_grasp`: 抓取点局部偏移 (gripper_tip→EEF 沿Z轴补偿)

### 22.2 视觉估计重试

```cpp
static constexpr int kVisionEstimateMaxRetries = 3;
static constexpr double kVisionEstimateRetryDelaySec = 1.0;
```

视觉位姿估计可能因光照/遮挡/特征不足而失败，3次重试提高成功率。

### 22.3 Z 安全下限

`kExecuteGraspZMinLimit = 0.19m` — 抓取目标 Z 必须 ≥ 0.19m (桌面高度 + 安全余量)，防止末端与桌面碰撞。

---

## 23. OpenCV 手眼标定实现细节

### 23.1 标定会话管理

```python
self.session_id = int(time.time() * 1000)  # 毫秒时间戳
self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
```

每次标定生成唯一 session ID，用于区分多次标定结果。

### 23.2 NumPy JSON 序列化

标定结果通过自定义 `_convert_to_json_serializable()` 递归处理:
- `np.integer` → `int`
- `np.floating` → `float` (NaN/Inf → None/null)
- `np.ndarray` → `list`
- `np.bool_` → `bool`

**原因**: Python `json.dumps()` 不支持 NumPy 类型。

### 23.3 配置路径解析

```python
try:
    package_share = get_package_share_directory('hand_eye_calibration')
    config_dir = os.path.join(package_share, 'config')
except:
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
```

优先使用 install 目录 (ament_index)，fallback 到源码目录。

---

## 24. 工作空间边界限制 (`limit_workspace.py`)

### 24.1 设计原理

通过发布 6 面碰撞墙体到 PlanningScene 来限制机械臂工作空间，防止规划超出安全区域。

**墙体布局**:
```
       +Z顶墙 (top)
       ┌─────────┐
      /│         /│
左墙 / │ 右墙   / │
(left)│        │(right)
     │ 前墙   │
     │ (front)│
     └─────────┘
后墙(back)   底墙(bottom, 默认禁用)
```

### 24.2 配置优先级

```
命令行参数 > YAML 配置文件 > 默认值
```

### 24.3 关键实现

```python
# 每面墙是一个 CollisionObject
wall = CollisionObject()
wall.id = f"workspace_wall_{name}"
wall.header.frame_id = "world"
wall.primitives = [SolidPrimitive(type=BOX, dimensions=[l, w, h])]
wall.operation = CollisionObject.ADD

# 发布3次确保被move_group接收
for i in range(3):
    collision_object_publisher.publish(wall)
    time.sleep(0.1)

# 移除墙面
wall.operation = CollisionObject.REMOVE
collision_object_publisher.publish(wall)
```

### 24.4 默认边界

| 轴 | 最小值 | 最大值 |
|-----|--------|--------|
| X | -0.8m | 0.8m |
| Y | -0.8m | 0.8m |
| Z | 0.0m | 1.2m |
| 墙厚度 | 0.05m | — |

### 24.5 用途

- 模拟实际工作台/安全围栏的物理限制
- 防止机械臂碰撞实验环境中的敏感设备
- 底墙默认禁用（地板由物理环境提供）

---

## 25. 工具快换 (tool_changer) — AttachedCollisionObject + world 清理

### 25.1 架构总览

两个独立 C++ 节点协作（接口均在 **`ivg_interfaces`**）喵~

```
gripper_swap_worker_node              scene_attach_worker_node
(物理运动 + IO，RobotController)        (MoveIt 规划场景：附着 / 清残留)

changeToTool
  ├─ /scene_detach (ChangeTool)  ─────→ detachToolFromScene
  │                                        ├─ pub /attached_collision_object REMOVE
  │                                        └─ pub /planning_scene world REMOVE
  ├─ releaseTool → publishToolStatus(false)
  ├─ pickTool    → publishToolStatus(true) ─→ onToolStatus()
  │                                        ├─ detach 旧 + attach 新（/attached_collision_object）
  │                                        └─ world REMOVE attached_tool_<id>（/planning_scene）
  └─ moveToHome
```

- **gripper_swap_worker_node**：`aubo_ros2_ws/src/tool_changer/src/gripper_swap_worker.cpp` — 数据驱动 `changeToTool()`，快换 IO 经 `RobotController::setGripper` → `/set_robot_io`喵~  
- **scene_attach_worker_node**：`aubo_ros2_ws/src/tool_changer/src/scene_attach_worker.cpp` — 订阅 `/tool_changer_status`；**不**发布 `/robot_description`、**不**写 `robot_state_publisher` 参数喵~  

### 25.2 启动

```bash
# launch 方式（两个节点一起启动）
ros2 launch tool_changer gripper_swap_worker.launch.py

# 单独运行
ros2 run tool_changer gripper_swap_worker_node &
ros2 run tool_changer scene_attach_worker_node &
```

### 25.3 工具定义 (`config/tools.yaml`)

5 种工具，每工具定义碰撞 mesh、dock 工位、附着偏移、touch_links：

| 工具 ID | 描述 | Dock 位置 |
|---------|------|-----------|
| gripper0 | 气动夹爪 φ40 | (0.270, 0.295, 0.270) |
| gripper1 | 电动夹爪 A | (0.270, 0.180, 0.270) |
| gripper2 | 电动夹爪 φ60 | (0.374, 0.304, 0.270) |
| gripper1coffeecup | 咖啡杯工具 | (0.270, 0.100, 0.270) |
| gripper1milkcup | 牛奶杯工具 | (0.270, 0.020, 0.270) |

### 25.4 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/run_gripper_swap` | `ivg_interfaces/srv/RunGripperSwap` | 前端调用，direction: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| `/change_tool` | `ivg_interfaces/srv/ChangeTool` | 按 tool_id 切换（物理运动 + 状态话题驱动 scene_attach） |
| `/get_current_tool` | `ivg_interfaces/srv/GetCurrentTool` | 查询当前工具 |
| `/scene_attach` | `ivg_interfaces/srv/ChangeTool` | 手动附着工具碰撞到 PlanningScene（不运动） |
| `/scene_detach` | `ivg_interfaces/srv/ChangeTool` | 手动脱离（不运动） |

### 25.5 快换动作时序（数据驱动 `changeToTool`，示意）

```
1. moveToDockApproach(current)
2. /scene_detach(current)              — 提前卸碰撞，避免 release 笛卡尔路径被挡
3. releaseTool(current)               — 数据驱动 vertical/slide
4. publishToolStatus(false)           — 仅 release 成功后
5. moveToDockApproach(target)
6. pickTool(target)
7. publishToolStatus(true)            — pick 成功后立即（scene_attach_worker 附着新工具）
8. moveToHome()
```

### 25.6 手动测试命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash   # 在 aubo_ros2_ws 目录下

# 查询当前工具
ros2 service call /get_current_tool ivg_interfaces/srv/GetCurrentTool "{}"

# 仅测试场景附着（不需要机械臂运动）
ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /scene_detach ivg_interfaces/srv/ChangeTool "{tool_id: gripper2}"

# 模拟工具状态变更（测试 scene_attach_worker 自动响应）
ros2 topic pub -1 /tool_changer_status ivg_interfaces/msg/ToolChangerStatus \
  "{tool_id: 'gripper0', tool_name: '气动夹爪', tool_type: 'gripper', is_connected: true, tool_parameters: '{}'}"

ros2 topic pub -1 /tool_changer_status ivg_interfaces/msg/ToolChangerStatus \
  "{tool_id: 'gripper2', tool_name: '电动夹爪', tool_type: 'gripper', is_connected: true, tool_parameters: '{}'}"

# 完整快换（需要 move_group 在运行）
ros2 service call /change_tool ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"

ros2 service call /run_gripper_swap ivg_interfaces/srv/RunGripperSwap \
  "{direction: gripper0_to_gripper2}"
```

---

## 26. 夹爪关节状态发布

### 26.1 问题背景

MoveIt2 规划时需要所有活动关节的状态。如果 SRDF 中定义了 `grap0_joint` 和 `grap1_joint`（夹爪关节）但没有对应的 joint state 发布者，MoveIt 会在日志中持续输出警告。

### 26.2 解决方案 (`publish_gripper_joint_states.py`)

```python
# 简单的 10Hz 定时发布
self.timer = self.create_timer(0.1, self.publish_joint_states)

msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = ['grap0_joint', 'grap1_joint']
msg.position = [0.0, 0.0]   # 固定位置 0
msg.velocity = [0.0, 0.0]
msg.effort = [0.0, 0.0]
self.publisher_.publish(msg)
```

**关键点**: 
- 夹爪在本项目中视为固定末端工具，不参与主动运动控制
- 发布频率 10Hz 足够满足 MoveIt 监控需求
- 直接发布到 `/joint_states` 话题

---

## 27. MoveIt2 C++ 项目最小模板

### 27.1 创建包

```bash
ros2 pkg create --build-type ament_cmake \
  --dependencies moveit_ros_planning_interface rclcpp \
  --node-name hello_moveit hello_moveit
```

### 27.2 最小化 MoveIt 节点

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 等待 robot_description 参数就绪
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "manipulator");

  // 设置目标位姿
  geometry_msgs::msg::Pose target;
  target.position.x = 0.3; target.position.y = 0.0; target.position.z = 0.5;
  target.orientation.w = 1.0;
  move_group->setPoseTarget(target);

  // 规划并执行
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group->execute(plan);
  }

  rclcpp::shutdown();
  return 0;
}
```

### 27.3 编译

```bash
colcon build --mixin debug --packages-select hello_moveit
```

**重要**: 需要先启动 `move_group` (`ros2 launch ... demo.launch.py`)，否则 `robot_description` 参数不可用，`MoveGroupInterface` 构造会失败。本项目使用 `wait_for_robot_description` 模式解决此问题（参见 §2.8）。

---

## 28. 参数管理深度分析

### 28.1 参数声明与覆盖三种模式对比

| 模式 | 位置 | 适用场景 | 示例 |
|------|------|---------|------|
| **automatically_declare_from_overrides** | C++ NodeOptions | 参数完全由 launch 文件控制 | `demo_driver` 所有节点 |
| **手动 declare + get** | C++ 构造函数 | 需要默认值、类型安全 | `gripper_swap_worker.cpp` |
| **declare + get + has_parameter 检查** | C++ 构造函数 | 避免与 launch overrides 冲突 | `execute_grasp_pose_worker.cpp` |

**推荐模式** (最健壮):
```cpp
// 先检查是否已被 automatically_declare 注入
if (!has_parameter("param_name"))
    declare_parameter("param_name", default_value);
auto value = get_parameter("param_name").as_double();
```

### 28.2 Python 参数模式

```python
# Python 端参数声明
self.declare_parameter('param_name', default_value)
value = self.get_parameter('param_name').value

# 在 launch 文件中覆盖
Node(
    package='pkg',
    executable='node',
    parameters=[{'param_name': override_value}],
)
```

### 28.3 参数优先级链

```
1. launch 文件参数覆盖 (最高优先级)
2. YAML 配置文件 ({'param_name': value})
3. Node.declare_parameter(default_value) (最低/默认)
```

---

## 29. demo_driver 服务节点统一模式

### 29.1 wait_for_robot_description 模式

所有 6 个 demo_driver 服务节点共享完全一致的 `wait_for_robot_description()` 实现:

```cpp
bool XxxServer::wait_for_robot_description(int timeout_seconds) {
    auto start_time = std::chrono::steady_clock::now();
    int check_count = 0;
    while (rclcpp::ok()) {
        try {
            auto test_move_group = std::make_shared<MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            std::string test_frame = test_move_group->getPlanningFrame();
            if (!test_frame.empty()) return true;  // robot_description 就绪
        } catch (const std::exception& e) {
            check_count++;
            if (check_count % 10 == 0) {
                // 每 10 次检查打印一次日志 (约每 5 秒)
                RCLCPP_INFO(get_logger(), "Still waiting... (elapsed %ld s, error: %s)", ...);
            }
        }
        if (elapsed >= timeout_seconds) {
            RCLCPP_ERROR(get_logger(), "Timeout waiting (%d seconds)", timeout_seconds);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 500ms 重试间隔
    }
    return false;
}
```

**关键点**:
- 每 500ms 尝试创建 `MoveGroupInterface`，检查 `getPlanningFrame()` 是否非空
- 每 10 次检查 (约 5 秒) 打印一次进度日志
- 所有 6 个节点的代码完全一致（`ExecuteTrajectoryServer`, `GetCurrentStateServer`, `PlanTrajectoryServer`, `SetRobotPoseServer`, `SetSpeedFactorServer`, `MoveToPoseServer`）— 可提取为基类方法

### 29.2 服务节点的 main() 模式

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);  // 允许 launch 注入参数
    
    auto node = XxxServer::create(options);  // 工厂方法创建 (调用 initMoveGroup)
    
    // 多线程 executor
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    std::thread spinner([&exec]() { exec.spin(); });
    
    // 等待 robot_description 就绪
    if (!node->wait_for_robot_description(30)) { /* 退出 */ }
    
    // 等待依赖服务
    if (!node->waitForServices(10s)) { /* 退出 */ }
    
    // 主线程等待退出
    while (rclcpp::ok()) { sleep(100ms); }
    // ...
}
```

### 29.3 MoveToPoseServer 的 service_mutex 保护

`MoveToPoseServer` 使用 `std::lock_guard<std::mutex> lock(service_mutex_)` 保护服务回调，防止并发运动请求导致机械臂冲突。这是唯一一个使用互斥锁保护的服务节点 — 因为运动命令不能并发执行。

---

## 30. ROS2 Executor 内部机制

### 30.1 Wait Set 与调度

ROS2 Executor 通过 **Wait Set** 与 DDS 中间件通信。Wait Set 只报告 "某话题是否有消息"，不报告具体数量。

**调度语义**:
- 当回调处理速度 > 事件到达速度: 近似 FIFO
- 当回调处理速度 < 事件到达速度: 切换到 **Round-Robin** (非 FIFO)
- Timer 优先级 (来自 Eloquent) 已被移除

### 30.2 回调组并发规则

| 场景 | 能否并发 |
|------|---------|
| 同一 MutuallyExclusive 组内的两个回调 | **否** |
| 同一 Reentrant 组内的两个回调 | **是** |
| 不同组的回调 (任意类型) | **是** |
| 未分配组的回调 → 默认组 (MutuallyExclusive) | 与同默认组回调互斥 |

**重要推论**:
- 将长耗时服务放在独立 MutuallyExclusive 组 → 同组内串行，不同组并行
- 所有未分配组的服务/订阅共享默认 MutuallyExclusive 组 → 可能意外互斥
- MultiThreadedExecutor 的线程数决定最大并发回调数

### 30.3 StaticSingleThreadedExecutor

一种优化变体: 在节点添加时只扫描一次结构 (subscriptions/timers/services)，之后不再扫描。适用于节点结构固定的场景，可以降低 CPU 开销。**本项目未使用。**

### 30.4 spin 机制对比

| 方法 | 行为 | 使用场景 |
|------|------|---------|
| `spin()` | 阻塞直到 shutdown | 独立节点/进程 |
| `spin_once(timeout)` | 执行一次回调后返回 | 手动控制循环 |
| `spin_some()` | 执行所有已就绪的回调 | 非阻塞检查 |
| `spin_until_future_complete()` | 阻塞直到 future 完成 | 等待异步调用结果 |

**本项目中**:
- `spin()`: C++ Worker 的 main() 后台线程
- `spin_once()`: Python Web 服务的 `_spin_future()` 循环、TF 发布节点一次性发布
- `spin_until_future_complete()`: Python 服务调用等待

---

*最后更新: 2026-04-30*
*本文档基于 ROS2 Humble + MoveIt2 Humble + 本地源码迭代深入分析*

```
1. launch 文件参数覆盖 (最高优先级)
2. YAML 配置文件 ({'param_name': value})
3. Node.declare_parameter(default_value) (最低/默认)
```

---

*最后更新: 2026-04-30*
*本文档基于 ROS2 Humble + MoveIt2 Humble + 本地源码迭代 100 轮深度分析*
