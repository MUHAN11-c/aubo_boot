# Changelog

本文件记录 IVG 2.0 工作空间（`aubo_ros2_ws`）中所有包的版本变更喵~
格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 约定，
版本号遵循 [Semantic Versioning](https://semver.org/lang/zh-CN/) 喵~

<!--
  每次修改后按以下分类更新 [Unreleased] 节:
  - Added      新增功能
  - Changed    现有功能的变更
  - Deprecated 即将废弃的功能
  - Removed    已移除的功能
  - Fixed      缺陷修复
  - Security   安全漏洞修复

  发版时将 [Unreleased] 内容移至新版本号节下，并添加日期喵~
-->

## [Unreleased]

### Added

#### 补齐 3 个缺失的运行时接口 (2026-05-14)
- **demo_driver**: 新建 `set_robot_enable_server` — `/set_robot_enable` 服务，代理到仪表盘 `/aubo/startup`/`/aubo/shutdown`，客户端在独立 MutuallyExclusive 回调组避免 `future.wait_for` 死锁喵~
- **demo_driver**: 新建 `read_robot_io_server` — `/read_robot_io` 服务，订阅 `/robot_io_status` 缓存后按索引查询，`io_state_received_` 使用 `std::atomic<bool>` 消除数据竞争，支持 8 种 io_type 喵~
- **aubo_driver_ros2**: `AuboStateBroadcaster` 新增 `/robot_io_status` 发布者 — 10Hz 独立定时器调用 `readFullIOStatus()`，与 50Hz `pollTick` 串行化避免 `conn_status_` 并发喵~

#### RobotController 组合模式 (2026-05-14)
- **demo_driver**: 新建 `robot_controller.h/.cpp` (230行) — 组合模式封装 MoveIt 运动 (`moveToHome`/`moveToJoints`/`moveCartesianPath`/`moveCartesianZ`) + IO 控制 (`setGripper`/`setQuickSwap`)，统一 IO 语义 `open=true` 打开夹爪，可配置 `eef_step`/`z_min_limit`/重试参数喵~
- **tool_changer**: `gripper_swap_worker` 接入 RobotController — 23 处调用迁移，`move_group_` 共享 `robot_->moveGroup()` 实例喵~

#### 机械臂驱动
- **aubo_driver_ros2**: 新建 `JointTrajectoryController` — 独立 Action Server 节点，预计算完整轨迹 + 独立发送线程（ROS1 `publishWaypointToRobot` 架构移植）喵~
- **aubo_driver_ros2**: 新建 `AuboDashboardNode` — Dashboard 状态管理节点（18+ 服务，`IsBolck=false` 非阻塞模式）喵~
- **aubo_driver_ros2**: 新建 `AuboHardwareInterface` — SDK 双连接架构（`conn_control_` TCP2CAN + `conn_status_` 普通模式），SDK 调用隔离在独立线程喵~
- **aubo_driver_ros2**: 新建 `AuboStateBroadcaster` — 100Hz 关节状态广播节点喵~
- **aubo_driver_ros2**: 新建 `aubo_new_driver.launch.py` — 统一真机/仿真启动文件，TCP 探测自动决策喵~
- **aubo_driver_ros2/doc/ARCHITECTURE.md**: 架构文档 — 动态 URDF 重载章节、双连接架构、QoS 配置喵~
- **aubo_driver_ros2/doc/BUILD_NOTES.md**: 编译注意事项速查喵~

#### 轨迹执行
- **aubo_driver_ros2**: 轨迹执行从订阅 `moveItController_cmd` topic 逐点缓冲的"边算边发"模式改为 Action Server 预计算完整轨迹 + 独立发送线程的"一次计算后下发执行"模式 — 五次多项式插值 (quintic interpolator) + RIB 自适应批量发送 (EMA 补偿, 2-8 点/批) + 目标约束检查 (50ms/次) 喵~
- **aubo_driver_ros2**: `sendLoop()` 独立线程 — RIB 诊断读取（活动 120ms / 空闲 250ms 降频）、`RIB≥300` 防溢出等待、EMA 自适应批量策略 (`ceil((400-rib)/6)`)、`update()` 50Hz 安全急停检查喵~

#### RViz2 & 可视化
- **moveit_ros_visualization**: `PlanningSceneDisplay` 订阅 `/robot_description` 话题动态重载 RobotModel（运行时 URDF 切换支持）喵~
- **moveit_ros_visualization**: `planning_scene_display.cpp` 本地补丁 — `createPlanningSceneMonitor` Options 双参构造初始化 `robot_description_`、`clearRobotModel` 重置 `planning_scene_robot_` 修复 MoveIt2 Humble 运行时 URDF 切换 bug 喵~
- **moveit_ros_visualization**: `motion_planning_display.cpp` 本地补丁 — `clearRobotModel` 重置 `query_robot_start_` / `query_robot_goal_` 防止渲染循环持旧 link 导致 "Link not found" 喵~
- **demo_driver**: 新增 `system_logger.h` — 系统日志工具喵~

#### 接口统一
- **ivg_interfaces**: 新建统一自定义 ROS 2 接口包 — 合并原 5 个散落接口包（`aubo_msgs`、`demo_interface`、`percipio_camera_interface`、`tool_changer_interface`、`interface`），共 17 msg + 35 srv = 52 个接口类型喵~
- **ivg_interfaces**: 所有 52 个接口文件补全中文注释（用途/调用方/服务方/字段说明）喵~
- **camera_control_node**: 从接口包迁移至 `percipio_camera` 包内喵~

#### 工具快换
- **tool_changer**: `tools.yaml` 数据驱动轨迹配置 — `dock_approach_joints`（6 关节角）+ `trajectory`（strategy + 参数）字段喵~
- **tool_changer**: `gripper_swap_worker` 数据驱动重构 — `loadToolConfig()` YAML 加载、`pickTool()`/`releaseTool()` 按 strategy 分发、`changeToTool()` 通用流水线无硬编码 per-gripper 分支、`onGripperSwapRequest()` 通用方向解析（`X_to_Y` → 取 `_to_` 后 `Y` 作 target_id）喵~
- **tool_changer**: `TrajectoryStrategy` enum — `"vertical"`（纯 Z 轴升降）和 `"slide"`（Y 轴侧滑 + 分段 Z）两种策略，新增策略在 enum + pickTool/releaseTool 添加即可喵~
- **tool_changer/README.md**: 新增 `tools.yaml` 轨迹字段文档（字段表 + 示例配置）喵~

#### 抓取管线
- **latte_imitation**: 新建 `trajectory_pipeline.py` — 5 阶段 MoveIt2 标准管线 (Load → Transform+AutoPose → Debug → computeCartesianPath → execute) 喵~
- **latte_imitation**: 通过 TF `lookup_transform(base_link, tool_tcp)` 自动获取当前末端位姿作为轨迹起点喵~
- **latte_imitation**: `trajectory_transform.py` 新增 `rot_to_quat()` 和 `quat_multiply()` 函数 — 修复 orientation 旋转 bug 喵~
- **start_latte_test.sh**: 新增 `/execute_trajectory` action 就绪等待喵~
- **ivg_utils/ivg_utils/io.py**: IO 引脚定义常量 (`IO_GRIPPER=6`, `IO_QUICK_SWAP=7`) 喵~

#### 编译 & 部署
- **aubo_driver_ros2/CMakeLists.txt**: 安装 AUBO SDK `.so` 到 `install/lib/`（修复 `exit code 127` 缺库报错）喵~
- **SDK .so**: `libauborobotcontroller.so`、`liblog4cplus.so`、`libconfig++.so`、`libconfig.so`、`libprotobuf*.so` 等编译必需的 SDK 库文件纳入 Git 版本管理喵~
- **.gitignore**: 统一规则 — 移除全局 `*.so` / `*.so.*` 排除（保留必需的 SDK .so），排除 `ros_arm_tutorials` / `yolov26_src` 嵌入式 git 仓库、`*.la` / `*.bak` / `*.a` 文件喵~
- **robotwebtools/roslibjs**: 添加 `COLCON_IGNORE` 避免 colcon 误识别为 CMake 包喵~
- **robotwebtools/docs/SOURCE_CHANGES.md**: 本地修改记录文档喵~

#### 项目文档
- **README.md**: 新增 §0 硬件配置（IP/IO/相机/快换盘完整信息）喵~
- **DEPLOYMENT.md**: 新增部署文档 — SDK 运行时库路径、环境变量说明喵~
- **ROS2_SETUP_NOTES.md**: 新增 ROS 2 环境设置说明喵~
- **docs/large-files-inventory.md**: 大文件清单 — 记录所有被 `.gitignore` 排除的大文件位置与用途喵~
- **docs/tech-stack-recommendations.md**: 技术栈选型推荐文档喵~
- **.agents/skills/**: 8 个 Claude Code skill（caveman / diagnose / grill-me / grill-with-docs / handoff / improve-codebase-architecture / prototype / setup-matt-pocock-skills）喵~

#### Web 前端
- **aubo_ros2_web_dashboard**: `ros_connector.js` — ROS 连接管理模块喵~
- **aubo_ros2_web_dashboard**: `log_panel.js` — 日志面板组件喵~
- **aubo_ros2_web_dashboard**: `ivg_status_bar.js` — 状态栏扩展（100+ 行变更）喵~

---

### Changed

#### 5 个服务接口重写精简 (2026-05-14)
- **demo_driver**: `move_to_pose_server` 重写 (128→106行) — 移除 `MoveitGripperIoBase` 继承，直接创建 `MoveGroupInterface`，删除 `run()`/`create()`/手动线程喵~
- **demo_driver**: `plan_trajectory_server` 重写 (457→158行) — 删除 `wait_for_robot_description()`/`initialize()` 样板代码，IK 客户端移至 Reentrant 回调组，`spin_until_future_complete` → `future.wait_for()`，删除 6 个 `static bool` 日志门控喵~
- **demo_driver**: `execute_trajectory_server` 重写 (321→96行) — 删除所有 `wait_for`/`initialize`/重复轨迹检查/未使用变量喵~
- **demo_driver**: `get_current_state_server` 重写 (567→146行) — 合并 FK 两条路径为 `computeFK()`，O(n*m) 关节名查找 → 直接索引，FK 客户端 Reentrant 组喵~
- **demo_driver**: `set_speed_factor_server` 重写 (302→87行) — 删除 `wait_for_robot_description()`/`initialize()`/`sleep(100ms)`/重复 `set_parameter`，`SyncParametersClient` → 成员 `AsyncParametersClient` + Reentrant 组喵~

#### RobotController 迁移 (2026-05-14)
- **demo_driver**: `execute_grasp_pose_worker` 完全迁移 — `MoveitGripperIoBase` 继承 → `rclcpp::Node` + `RobotController` 成员，15 处调用改为 `robot_->`，删除 `create()`/`initMoveGroup()`/`waitForServices()`/`run()`喵~
- **demo_driver**: `publish_grasps_client_worker` 混合迁移 — 构造函数创建 `robot_` 并共享 `move_group_ = robot_->moveGroup()`，`runOneCycle` 中 10 处核心调用改为 `robot_->`，删除 `create()`/`initMoveGroup()`喵~
- **demo_driver**: `move_to_pose_server` 独立迁移 — 移除 `MoveitGripperIoBase` 依赖，直接创建 `MoveGroupInterface`喵~

#### 接口统一修复
- **demo_driver**: 话题名 `/aubo_driver/robot_status` → `/robot_status`，服务名 `/aubo/set_io` + `/aubo_driver/set_io` → `/set_robot_io` (13文件) 喵~
- **aubo_driver_ros2**: `AuboDashboardNode` `onTeachStop`/`onSetToolVoltage`/`onSetIO` 补 `sdk_mutex_` (3处) 喵~
- **aubo_driver_ros2**: `aubo_state_broadcaster.cpp` `pollTick` 中 `isConnected()` 重复调用 → 缓存 `online` 变量喵~

#### 机械臂驱动
- **aubo_driver_ros2**: 轨迹执行架构从"边算边发"改为"预计算 + 独立发送线程" — `JointTrajectoryController` Action Server 替代旧 `moveItController_cmd` topic 订阅模式，MoveIt 无需逐点下发，减少 ROS 消息开销喵~
- **aubo_driver_ros2**: `aubo_state_broadcaster.cpp` `pollTick` 补回 `cartesian_rpy` 计算（与旧驱动 `fillCartesianPoseAndRpy` 公式一致）喵~
- **aubo_driver_ros2**: `start_aubo_new_driver.sh` 外部引用从 `aubo_moveit_pure_ros2.launch.py` 迁移至 `aubo_new_driver.launch.py`，LD_LIBRARY_PATH 追加 SDK 路径，rosbag 录制排除 `~/state` 弃用 topic 喵~
- **aubo_driver_ros2/CMakeLists.txt**: 新增 SDK `.so` install 规则，添加 `yaml_cpp_vendor` + `ament_index_cpp` 依赖喵~
- **aubo_moveit_config**: `aubo_new_driver.launch.py` 参数重组织（真机/仿真自动决策），`aubo_e5.srdf` 新增 20 条 SRDF ACM 碰撞豁免（5 工具 × 4 末端 link），`aubo_e5.urdf.xacro` 工具 link origin 对齐 `tools.yaml` 喵~
- **aubo_moveit_config**: 新增 `aubo_mode.py` 脚本 — 机械臂模式切换工具喵~

#### 接口统一
- **demo_driver**: 9 个服务/worker 头文件同步更新接口引用 — `aubo_msgs`/`demo_interface` → `ivg_interfaces`（`execute_grasp_pose_worker.cpp`、`execute_trajectory_server.cpp`、`move_to_pose_server.cpp` 等）喵~
- **tool_changer**: 接口引用 `demo_interface` → `ivg_interfaces`，IO 服务名 `/aubo_driver/set_io` → `/set_robot_io` 喵~
- **camport_ros2**: 接口引用 `percipio_camera_interface` → `ivg_interfaces`喵~
- **hand_eye_calibration**: 接口引用 `interface`/`percipio_camera_interface` → `ivg_interfaces`喵~
- **visual_pose_estimation_python**: 接口引用 `interface` → `ivg_interfaces`、rosbridge 消息类型 `aubo_msgs` → `ivg_interfaces`喵~
- **latte_imitation**: `package.xml` 依赖 `control_msgs` → `moveit_msgs` + `tf2_ros`，`setup.py` 入口点 `trajectory_publisher:main` → `trajectory_pipeline:main`，launch 文件移除 `pos_only` 参数喵~
- **latte_imitation**: 管线从自定义 PyKDL DLS IK (pos_only) 全面迁移到 MoveIt2 标准管线 (`/compute_cartesian_path` + `/execute_trajectory`) — IK 全 6-DOF、碰撞检测 `avoid_collisions=True`、轨迹执行走 MoveIt2 action 喵~
- **latte_imitation/README.md**: 全面重写 — 架构图、模块表、示例命令、参数表、设计决策均更新喵~

#### 工具快换 & PlanningScene
- **tool_changer**: `scene_attach_worker` 直接发布 URDF 到 `/robot_description` 话题（替代 `topic_tools relay`），通过 PlanningScene diff 管理碰撞模型（`is_diff=true`）喵~
- **tool_changer**: `gripper_swap_worker` 数据驱动重构 — 轨迹参数全部从 `config/tools.yaml` 加载，不再硬编码 per-gripper 函数，新增工具只需编辑 YAML 喵~
- **tool_changer/CMakeLists.txt**: 新增 `yaml_cpp_vendor` + `ament_index_cpp` 依赖喵~

#### 编译 & 构建
- **moveit_ros_planning_interface/CMakeLists.txt**: 追加 `BUILD_TESTING OFF`（修复 OMPL cmake `PACKAGE_PREFIX_DIR` 被 Eigen3 覆盖导致 configure 失败）喵~
- **build_robotwebtools.sh**: `node_modules` 不完整时自动检测并重装缺失依赖喵~

#### Web 前端
- **pose_card.js**: 修正 `poseToRpyDeg` 四元数转欧拉角公式 sign 错误（`cartesian_rpy` 后端弧度值前端未转度）喵~
- **aubo_ros2_web_dashboard**: `web_dashboard.launch.py` FastAPI 网关 `ExecuteProcess` 增加 `respawn=True, respawn_delay=5.0`喵~
- **aubo_ros2_web_dashboard**: `setup.py` 静态文件安装路径 `web/public` → `web/dist`（Vue 3 构建产物）喵~
- **aubo_ros2_web_dashboard**: `README.md` 更新前端路径说明 `web/public/` → `web/dist/`喵~

#### 启动脚本
- **start_aubo_new_driver.sh**: 删除 Step [5] `image_data_bridge` 启动，Step [6] 手眼标定增加 `enable_image_data_converter:=true`（由 hand_eye 内置 converter 替代外部 bridge），cleanup 新增 `latte_imitation` 进程清理，步骤 [11] 标注 MoveIt2 新管线喵~
- **start_hand_eye_calibration.sh**: 更新接口引用和参数喵~
- **start_latte_test.sh**: 所有 YAML 命令移除 `pos_only` 和 `collision_check`，参数速查更新喵~

#### 测试 & 调试
- **test_replay_service.py**: 期望值更新 (`collision_count=0`, `ik_success_count` 语义变化) 喵~
- **visualize_latte_trajectory.py**: `RobotModel` FK 替换为内联 PyKDL FK (已删除模块) 喵~
- **compare_demo_robot_status_with_moveit.py**: 接口引用更新喵~

#### 文档
- **CLAUDE.md**: 新增 #5(`JointTrajectoryController` 非 `ros2_control`)、#7(`publishWaypointToRobot` 架构)、#11(动态 URDF 切换机制 + `moveit_ros_visualization` 本地补丁)、#12b(SRDF ACM 碰撞豁免)、#25(接口统一化)、#26(gripper_swap 数据驱动架构)、#27(latte_imitation MoveIt2 标准管线)、#28(已删除模块) 规则，速查表新增 Dashboard/web dashboard/SDK 库缺漏/弃用 topic 条目，补充前端技术栈 (Vue 3/Vite/Tailwind CSS v4/Element Plus) 等喵~
- **docs/architecture.md**: `latte_trajectory_player` Action 更新为 `/execute_trajectory`，新增动态 URDF 重载章节喵~
- **docs/architecture-audit-2026-05-13.md**: 架构审计文档更新喵~
- **IVG2_TECHNICAL_NOTES.md**: 技术笔记更新喵~
- **aubo_ros2_ws/README.md**: 工作空间说明更新喵~

---

### Deprecated

- **ReplayLatteTrajectory.srv**: `pos_only` 字段 — 保留兼容但不再生效 (MoveIt2 始终全 6-DOF IK) 喵~
- **ReplayLatteTrajectory.srv**: `collision_check` 字段 — 保留兼容但不再生效 (MoveIt2 内置碰撞检测) 喵~
- **ReplayLatteTrajectory.srv**: `collision_count` 响应字段 — 始终为 0 喵~
- **ReplayLatteTrajectory.srv**: `collision_details` 响应字段 — 始终为空 喵~
- **image_data_bridge**: 包目录标记 `COLCON_IGNORE`（功能由 `hand_eye_calibration/image_data_converter_node` 替代）喵~

---

### Removed

#### 冗余基类和重复代码 (2026-05-14)
- **demo_driver**: 删除 `moveit_gripper_io_base.h/.cpp` (604行) — 被 `RobotController` 替代，`execute_grasp_pose_worker`/`publish_grasps_client_worker`/`move_to_pose_server` 均已迁移喵~
- **demo_driver**: 删除 `CartesianSegment` 重复定义 (3处) — 统一至 `robot_controller.h`喵~
- **demo_driver**: 删除 `wait_for_robot_description()` ×4 (240行) + `initialize()` ×4 (180行) — 构造函数内直接创建 `MoveGroupInterface` 喵~
- **demo_driver**: 删除 `create()` 工厂模式 ×3 + `run()` 空存根 ×3 + `spin()` ×5 — `main()` 统一为 `make_shared` + `executor.spin()`喵~
- **demo_driver**: 删除 `static bool` 日志门控变量 ×20+ + 通用 `catch(...)` ×4 — 精简代码喵~

#### 旧驱动框架 (2026-05-08)
- **aubo_driver_ros2**: 删除 `aubo_driver_ros2.cpp` (1204行) + `aubo_driver.h` (274行) + `driver_node_ros2.cpp` (34行) — 旧 `moveItController_cmd` topic 订阅模式驱动，被 `JointTrajectoryController` Action Server 替代喵~
- **aubo_ros2_trajectory_action**: 删除整个包 (`aubo_ros2_trajectory_action.cpp` 314行 + header + main) — 轨迹 Action 功能集成入 `JointTrajectoryController` 喵~
- **aubo_robot_simulator_ros2**: 删除整个包 (`aubo_robot_simulator_node.py` 339行 + `trajectory_speed.py` + 配置) — 仿真功能由 `mock_components/GenericSystem` 替代喵~
- **aubo_moveit_pure_ros2.launch.py**: 删除旧 launch 文件 (209行) — 被 `aubo_new_driver.launch.py` 替代喵~

#### 旧接口包 (2026-05-13)
- **aubo_msgs**: 删除（6 msg + 5 srv）— 合并至 `ivg_interfaces`喵~
- **demo_interface**: 删除（6 msg + 22 srv）— 合并至 `ivg_interfaces`喵~
- **percipio_camera_interface**: 删除（2 srv + 3 msg）— 合并至 `ivg_interfaces`喵~
- **tool_changer_interface**: 删除（4 srv）— 合并至 `ivg_interfaces`喵~
- **interface**: 删除（3 srv）— 合并至 `ivg_interfaces`喵~

#### 旧管线模块
- **latte_imitation**: 删除 `robot_model.py` — PyKDL FK + DLS IK 求解器 (被 MoveIt2 替代) 喵~
- **latte_imitation**: 删除 `collision_checker.py` — 手动 `/check_state_validity` 碰撞检测 (被 MoveIt2 内置替代) 喵~
- **latte_imitation**: 删除 `action_executor.py` — 直接 `FollowJointTrajectory` action (被 `/execute_trajectory` 替代) 喵~
- **latte_imitation**: 删除 `trajectory_publisher.py` — 旧 8 阶段管线 (被 `trajectory_pipeline.py` 替代) 喵~

#### 死代码 & 冗余
- **demo_driver**: 删除 `publish_grasps_AB.h` (165行) — 旧 AB 抓取 worker 喵~
- **scene_attach_worker**: 去除 `AttachedCollisionObject` — 改用 URDF 碰撞几何 + Assimp 平滑渲染喵~
- **image_data_bridge**: 删除 `image_data_bridge_node.cpp` (178行) — 功能由 `hand_eye_calibration/image_data_converter_node` 替代喵~
- **visual_pose_estimation_python**: 删除 `node_runtime.py` 中 `/image_data` 订阅、`ImageData` import、`image_data_callback` 方法及相关状态变量 — `latest_image_data` 只写不读，死代码喵~
- **start_aubo_new_driver.sh**: 删除 `IVG_INCLUDE_POINTCLOUD_WEB_BRIDGE` / `IVG_POINTCLOUD_WEB_MAX_POINTS` 环境变量（无对应实现）、删除 `SKIP_RVIZ` 环境变量（声明但从未检查）喵~
- **test_coordinate_transforms.sh** + **yolov8n-obb.pt** + **docs/generate_interface_doc.py** + **接口明细.xlsx**: 过期文件清理喵~
- **legacy/**: 旧 `aubo_demo` 脚本/源码 (11 文件)、旧 `hand_eye_calibration_tool` 代码 喵~
- **ros_arm_tutorials**: 删除子模块引用喵~
- **AUBO SDK .bak/.a/.la**: 非编译必需文件排除喵~
- **.claude.yaml**: 删除（配置迁移至 `.claude/settings.json`）喵~

---

### Fixed

#### 14 处死锁/数据竞争/缺锁 (2026-05-14)
- **demo_driver**: `set_robot_enable_server` — 客户端与服务同默认 MutuallyExclusive 组导致 `future.wait_for` 永远超时，客户端移至独立 `client_cb_group_` 修复喵~
- **demo_driver**: `get_current_state_server` — FK 客户端 + `spin_until_future_complete` 死锁，FK 客户端移至 Reentrant 回调组喵~
- **demo_driver**: `plan_trajectory_server` — IK 客户端 + `spin_until_future_complete` 死锁 (2处)，IK 客户端移至 Reentrant 组 + `SyncParametersClient` → 本地 `get_parameter`喵~
- **demo_driver**: `execute_trajectory_server` + `set_speed_factor_server` — `SyncParametersClient` 死锁 (3处)，分别替换为本地参数读取和 `AsyncParametersClient` + Reentrant 组喵~
- **camport_ros2**: `camera_control_node` — `SyncParametersClient` 死锁，替换为 `AsyncParametersClient` + Reentrant 回调组喵~
- **demo_driver**: `read_robot_io_server` — `io_state_received_` 普通 `bool` 无锁读取数据竞争 → `std::atomic<bool>` + `memory_order` 配对喵~
- **aubo_driver_ros2**: `AuboStateBroadcaster` — `pollTick` 与 `pollIOTick` 不同回调组并发访问 `conn_status_` → 回默认组串行化喵~
- **aubo_driver_ros2**: `AuboDashboardNode` — `onTeachStop`/`onSetToolVoltage`/`onSetIO` 缺 `sdk_mutex_` (3处)，补 `std::lock_guard`喵~

#### SetCameraParameters.srv trigger_mode 语义修复
- **ivg_interfaces**: `SetCameraParameters.srv` trigger_mode 值域 `-1=连续` → `0=连续` (与代码 `>=0` 过滤一致)，对齐 `CameraStatus.msg` 喵~

#### 前端
- **pose_card.js**: 修正 `poseToRpyDeg` 四元数转欧拉角公式 sign 错误 — `cartesian_rpy` 后端弧度值前端未转度，导致 Web 面板欧拉角显示为零喵~
- **aubo_ros2_web_dashboard**: 修复 `setup.py` 安装 `web/public`（空目录）但 launch 文件引用 `web/dist`（Vue 3 构建产物）导致 FastAPI 网关 `ValueError` 启动崩溃、`/health` 超时的问题喵~
- **aubo_ros2_web_dashboard**: 修复 `ExecuteProcess` 无 `respawn` 导致网关进程崩溃后不自动重启的问题喵~

#### 启动脚本
- **start_aubo_new_driver.sh**: 修复 `local tick=0` 在子 shell `(...)` 中非法导致 Dashboard 生命周期未激活的 bug（`local` 仅函数内可用，`set -e` 下子 shell 静默退出，20 个 Dashboard 服务从未创建）喵~

#### 机械臂驱动
- **aubo_state_broadcaster.cpp**: `pollTick` 中补回 `cartesian_rpy` 计算 — 旧驱动 `fillCartesianPoseAndRpy` 公式在新框架中被遗漏，导致 `/aubo_driver/robot_status` 话题中欧拉角始终为零喵~

#### RViz2 / MoveIt 本地补丁
- **planning_scene_display.cpp**: 修复 `createPlanningSceneMonitor` 中 `Options` 双参构造不初始化 `robot_description_` 导致 `"parameter name must not be empty"` 异常喵~
- **planning_scene_display.cpp** + **motion_planning_display.cpp**: 修复 `clearRobotModel` 不重置 `planning_scene_robot_` / `query_robot_start_` / `query_robot_goal_` 导致渲染循环 `rviz::Robot` 持旧 link 抛出 "Link not found" 喵~
- **planning_scene_display.cpp**: 修复 RViz2 工具快换时 `robot_description_property_` 空值导致 `loadRobotModel` 异常 — `param_name` 空值时 fallback 为 `"robot_description"`喵~

#### 编译
- **moveit_ros_planning_interface/CMakeLists.txt**: 追加 `BUILD_TESTING OFF` 修复 OMPL cmake `PACKAGE_PREFIX_DIR` 被 Eigen3 覆盖导致 configure 失败喵~
- **aubo_driver_ros2/CMakeLists.txt**: 安装 AUBO SDK `.so` 到 `install/lib/` 修复新环境 `colcon build` 后 `exit code 127` (`error while loading shared libraries: libauborobotcontroller.so.1`) 喵~
- **.gitignore**: 移除全局 `*.so` / `*.so.*` 排除规则，SDK `.so` 库纳入版本管理，新电脑 clone 后可直接编译喵~

#### 管线
- **latte_imitation**: 修复 `apply_start_pose()` 中 orientation 不旋转的 bug — 现在使用 `rot_to_quat()` + `quat_multiply()` 正确应用 `R_rel` 旋转喵~
- **gripper_swap_worker**: 修复 IO 服务名 `/aubo_driver/set_io` → `/set_robot_io`（接口统一后的正确服务名）喵~
- **camera_control_node**: 修复接口引用（从已删除的 `percipio_camera_interface` 迁移至 `ivg_interfaces`）喵~

#### 其他
- **roslibjs**: 添加 `COLCON_IGNORE` 避免 colcon 误识别为 CMake 包导致构建警告喵~
- **robotwebtools**: `build_robotwebtools.sh` `node_modules` 不完整时自动检测并重装缺失依赖喵~
- **.gitignore**: 排除 `ros_arm_tutorials` 嵌入式 git 仓库（避免主 repo git 警告 `hint: You've added another git repository inside your current repository`）喵~

---

## 格式说明

每次代码修改后，在 `[Unreleased]` 的对应分类下新增条目。发版 (tag) 时按以下模板归档:

```markdown
## [X.Y.Z] - YYYY-MM-DD

### Added
- 包名: 描述

### Changed
- 包名: 描述

### Fixed
- 包名: 描述
```

**包名** 使用 `aubo_ros2_ws/src/` 下的目录名，涉及多个包时分行列出喵~
