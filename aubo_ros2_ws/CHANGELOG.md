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

### Changed
- **latte_imitation**: 完整重写 v3.0 — 从零重新设计全部模块（基于 SPOT/Isaac Teleop/SVRC/SO(3) Action Repr/FluidLab 等 10 篇文献 + 8 项源码审计）。新增 SE(3) 重定目标管线、3 轴 RPY 可调、RViz2 Preview 可视化模式、交互式 shell 控制面板。详见 `src/latte_imitation/DESIGN.md` 喵~
- **start_latte_test.sh**: 重写 v3.0 — 适配 6 阶段管线 + RViz2 Preview 模式。5 个 terminator 标签页: [Sim] + [Latte Node] (默认 preview) + [Preview Panel] (test_latte_pour.py) + [RViz2] (latte_preview.rviz) + [Cmd Ref] (v3.0 新字段命令速查)。默认 mode=preview 可在 RViz2 中即时预览轨迹喵~
- **tool_changer**: `gripper_swap_worker` 添加周期性 `/tool_changer_status` 发布定时器（5 秒间隔），解决 VOLATILE QoS 下后连接的前端永远收不到初始状态的问题；首次发布在构造函数中立即执行，之后每 5 秒重发喵~
- **aubo_ros2_web_dashboard**: 消除工具几何数据 DRY 违规 — 新增 BFF 端点 `/api/v1/tool-geometries`，从 `tool_changer` 包的 `tools.yaml` 读取工具定义并返回前端；`Robot3dViewer.vue` 从动态 API 获取工具数据替代硬编码 `TOOL_MODELS`，BFF 不可用时自动回退到硬编码数据喵~
  - `gateway/routes/tool_geometries.py`: 新增路由，读取 `tool_changer` 包的 `tools.yaml`，将 `package://` URI 转换为 `/api/ivg/robot-mesh/` 代理 URL，返回前端期望的 JSON 格式；附带与 `tools.yaml` 同步的硬编码回退喵~
  - `gateway/app.py`: 注册 `tool_geometries.router`，在 `ivg_runtime` 之后、`robot_mesh` 之前喵~
  - `Robot3dViewer.vue`: 移除硬编码 `TOOL_MODELS` 常量；新增 `ToolGeometry` 接口、`toolGeometries` ref、`fetchToolGeometries()` 异步获取函数；`loadToolMesh()` 改为从 `toolGeometries` ref 读取；`init()` 中在工具状态订阅前调用 `fetchToolGeometries()` 避免竞态喵~
- **latte_imitation**: 全面代码优化 — 消除重复代码、将硬编码常量改为 ROS 2 参数、修复 YAML 注入风险、清理死代码喵~
  - 新建 `tf_utils.py` 共享 TF 查询模块（消除 `trajectory_pipeline.py`、`visualize_latte_trajectory.py`、`latte_debug_panel.py` 三处重复 TF 查询逻辑）喵~
  - `trajectory_pipeline.py`: 12 个硬编码常量 → `declare_parameter()` ROS 2 参数，支持运行时/YAML/launch 文件配置喵~
  - `trajectory_pipeline.py`: 删除 `_publish_planned_path()` stub 死代码及两个未使用的发布者 (`planned_ee_pose`/`planned_ee_path`) 喵~
  - `test_replay_service.py` / `latte_debug_panel.py`: YAML 字符串拼接 → `yaml.dump()` 消除注入风险喵~
  - `visualize_latte_trajectory.py`: 硬编码 URDF/SRDF 跨包路径 → `ament_index_python.get_package_share_directory()` 喵~
  - `visualize_latte_trajectory.py`: 内联 PyKDL 四元数提取 → `rot_to_quat()` 标准函数喵~
  - `latte_debug_panel.py`: 内联手动旋转矩阵 → `apply_start_pose()` 标准库函数喵~
  - `latte_debug_panel.py`: 内联 TF 节点管理 → `TfQueryNode` 持久查询类喵~
  - `latte_debug_panel.py`: 删除 `_do_execute()` 底部 `_poll` 未定义变量导致的冗余 QTimer 创建喵~
  - `infer_yolo26.py`: PEP 604 `str | None` → `Optional[str]` 兼容 Python 3.8/3.9 喵~
  - `__init__.py`: 填充公共 API `__all__`，支持 `from latte_imitation import CartesianTrajectory` 简洁导入喵~
  - `package.xml`: 新增 `python3-yaml` 运行时依赖喵~
  - `launch/replay_trajectory.launch.py`: 暴露全部 16 个 ROS 2 参数喵~
  - `README.md`: 更新模块表、参数表、话题表、Python API 示例喵~
- **Docs（工具快换 / MoveIt）**：与当前源码对齐 — `tool_changer/README.md` 数据流与 QoS、`CLAUDE.md` 规则 11、`docs/PROCESS-FLOW.md` §六、`docs/architecture.md` 节点表、仓库根 `README.md` 数据流、`IVG2_TECHNICAL_NOTES.md` §25、删改「仅 `/planning_scene` 发 ACO」「scene_attach 更新 URDF / robot_description」等过时表述；`scene_attach_worker`/`gripper_swap_worker`/`robot_controller` 头文件与块注释、`tool_changer_test.launch.py` 增补 `scene_attach_worker` 与接口名喵~

### Removed

### Fixed
- **aubo_ros2_web_dashboard**: `ToolSwapBar.vue` 末端夹爪无硬件反馈场景的完整解决方案喵~
  - 根因：`/tool_changer_status` 使用 VOLATILE QoS + 构造函数一次性发布 → 后连接的前端永远收不到状态 → `isToolConnected=false` → 所有按钮 disabled → 死锁喵~
  - 新增三重状态获取机制：(1) 连接时调用 `/get_current_tool` 服务立即获取，(2) 订阅 `/tool_changer_status` 话题，（后端新增 5 秒周期性发布确保后连接者能收到），(3) 若以上均返回 `is_connected=false`，显示**手动工具选择器**让用户指定当前物理安装的工具喵~
  - 手动选择器：用户选择 gripper0/gripper2/无工具 → 持久化到 localStorage (`ivg_tool_manual_id`) → 后端真实状态到达时自动覆盖喵~
  - `doChange()` 智能方向构建：有当前工具时 `direction = "current_to_target"`（含 release），无工具时 `direction = targetId`（仅 pick，后端 `changeToTool` 跳过 release 步骤）喵~
  - 按钮 disabled 条件改为 `isWaitingForStatus`（等待中禁用），非等待状态且非当前工具的按钮始终可用喵~
- **demo_driver**: `plan_trajectory_server_node`、`get_current_state_server_node`、`execute_trajectory_server_node`、`set_speed_factor_server_node`、`move_to_pose_server_node` 在构造函数内通过 `shared_from_this()` 构造 `MoveGroupInterface` / `AsyncParametersClient` 会抛出 `std::bad_weak_ptr`；改为 `std::make_shared` 返回后调用 `init()` 完成 MoveIt 与参数客户端初始化喵~
- **tool_changer**: `scene_attach_worker` 通过 `/attached_collision_object` 管理已连接工具的 `AttachedCollisionObject`，并用 `/planning_scene` world diff 清理 detach 残留；彻底移除动态 URDF / `robot_description` 更新链路喵~
  - 删除 `robot_description` topic 发布、`AsyncParametersClient` 参数设置、xacro 渲染和 URDF 缓存，避免该节点再影响 `robot_state_publisher` 或 Web/RViz RobotModel 喵~
  - 恢复 Git 历史 `748c7bb3d` 的末端工具位姿语义：`object.header.frame_id = kuaihuan_Link`、`object.pose = tools.yaml.attach_offset`、`mesh_poses[0]` 为单位位姿，修复当前实现丢失 `attach_offset.position` 的问题喵~
  - `gripper_swap_worker` 改为在 `releaseTool()` 成功后才发布 `is_connected=false`，在 `pickTool()` 成功后立即发布 `is_connected=true`，使附着碰撞状态与物理取放时刻同步喵~
  - `gripper_swap_worker` 在释放当前工具的 dock 放置笛卡尔路径前调用 `/scene_detach` 只提前移除 PlanningScene 碰撞体，避免附着碰撞模型阻挡放置路径；若 IO 尚未释放且路径失败，会尝试 `/scene_attach` 恢复规划场景喵~
  - `scene_attach_worker` 的 `/scene_detach` 在发布 `AttachedCollisionObject REMOVE` 后同步发布同名 world collision object 的 `REMOVE` diff，避免 MoveIt 将 detach 后对象重新放回 world 导致 `attached_tool_gripper2` 残留并阻挡释放路径喵~
  - 移除快换调试用的 NDJSON 文件探针、`/get_planning_scene` 场景快照与 `RobotController` 内 `/check_state_validity` 采样逻辑喵~
  - `Robot3dViewer` 根据 `/tool_changer_status` 在 Web 前端动态增删末端工具显示模型：后端只负责碰撞附着，前端用 `tool_id + attach_offset + kuaihuan_Link TF` 加载并定位对应 STL 喵~
  - 所有工具的 `touch_links` 增加 `tool_tcp`，允许附着工具与末端 TCP 虚拟 link 的预期重叠，降低取放 dock 附近笛卡尔路径被自碰撞误拦截的概率喵~
  - 文档同步为 `ivg_interfaces` 接口与「ACO 走 `/attached_collision_object`，detach 残留走 `/planning_scene` world REMOVE，不发布 URDF / 不添加 dock world 网格」的数据流喵~
- **hand_eye_calibration**: 默认 Web 端口改为 **8070**，绑定前检测占用且 `app.run` 捕获 `EADDRINUSE`；`start_aubo_new_driver.sh` 增加 `web_port:=${HAND_EYE_PORT}`（默认 8070），减少与其它服务抢占 **8080** 导致 `OSError: Address already in use` 喵~
  - **aubo_ros2_web_dashboard** `constants/ros.ts`（`HAND_EYE_WEB_PORT` / `VPE_WEB_PORT`）与 `DashboardView.vue` 外链端口对齐喵~
  - `aubo_ros2_ws/README.md`、手眼包 README 与 IVG 遗留启动脚本注释中的端口说明同步更新喵~
- **hand_eye_calibration**: `package.xml` 补充 `python3-flask-cors` 运行时依赖，避免节点启动时报 `ModuleNotFoundError: No module named 'flask_cors'`（`setup.py` 已声明 `flask-cors`，但 `colcon build` 不会自动安装 pip 依赖，需通过 apt/rosdep 或 `pip3 install flask-cors` 满足）喵~
  - `README.md` 系统包安装示例同步增加 `python3-flask-cors` 喵~
- **graspnet_ros2**: 从 Git 历史恢复运行必需的 `logs/log_kn/checkpoint-rs.tar`，修复 `graspnet_demo_points_node` 加载模型时报 `FileNotFoundError` 的问题喵~
  - 根目录 `.gitignore` 与 `graspnet-baseline/.gitignore` 添加反向规则，确保 `checkpoint-rs.tar` 后续 `git add`、commit、push 不会被排除喵~
  - `docs/large-files-inventory.md` 更新 GraspNet checkpoint 为已纳入 Git 的运行必需文件喵~
- **graspnet_ros2**: 修复 `pointnet2` CUDA 扩展安装失败导致 `graspnet_demo_points_node` 启动时报 `ModuleNotFoundError: No module named 'pointnet2._ext'` 的问题喵~
  - `graspnet-baseline/pointnet2/setup.py` 移除普通 C++ 编译参数中的 NVCC 专用 `-Xcompiler`，避免 GCC 报 `unrecognized command-line option '-Xcompiler'` 喵~
  - `README.md` 补充 `pointnet2._ext` 构建失败的根因与安装验证说明喵~
- **aubo_ros2_web_dashboard**: 修复 Vue 3 迁移后设置页话题/服务配置未贯通到运行页的问题喵~
  - 新增 `useDashboardSettings.ts`，统一合并 `/api/v1/runtime` 的 YAML 默认值与 `ivg_vision_grasp_topics_v3` 浏览器覆盖喵~
  - `VisionGraspView`、`CoffeeLatteView`、`ToolSwapBar`、`RobotStatusBar`、`TfMonitorView` 改为按设置订阅 ROS 话题和调用服务喵~
  - `Robot3dViewer` 支持旧版 `/<node>:<parameter>` URDF 参数格式，并修复工具热重载同步 ready 时可能闪空或异常的问题喵~
  - `Robot3dViewer` 的模型位姿改回旧版 `ROS3D.UrdfClient` 语义：URDF 只提供 link/mesh，所有 link 的位置和姿态由 `fixedFrame -> link` TF 直接驱动，避免本地 URDF 关节链与 TF 重复叠加导致模型位置关系错误喵~
  - `SceneManager` 改为 ROS/RViz 一致的 Z-up 视图：相机 `up=(0,0,1)`，地面网格从 Three.js 默认 XZ 平面旋到 ROS XY 平面，并在 `Robot3dViewer` 左上角显示 X=红、Y=绿、Z=蓝的坐标轴图例喵~
  - `TfUpdater.getTransform()` 复用旧版 `tf_clients.js` 的相对变换求解方向，支持通过共同祖先计算任意 `fixedFrame -> targetFrame`，不再只处理 target 逐级回溯到 base 的单一路径喵~
  - `useRos` 的 `onRosJson`、`onControlJson`、`onLog` 自动绑定 Vue scope dispose，避免 SPA 路由切换后回调叠加喵~
  - `config.py` 修复 `_HAS_YAML` 未定义导致 `/api/v1/settings` 写回 YAML 失败的问题喵~
  - 严格按旧版页面行为补回导航全屏按钮、设置入口新标签打开、咖啡 DO 纯前端切换、DI 显示顺序，以及工件模式结果图空配置时不回退显示彩色相机流喵~
  - `VisionGraspView`/`CoffeeLatteView` 布局改回旧版单屏紧凑策略：全宽页面、视觉页 URDF 与结果视图同排、右侧控制列约 22vw、监控区低间距折叠布局、Canvas 高度使用 `clamp(260px,36vh,400px)` 以减少 iPad 10 与窄屏双滚动条喵~
  - 运行页布局再次对齐旧版：主区和相机区改为 921px 起才进入双列，咖啡页左栏恢复 URDF + 流程示意同排结构，`base.css` 用 `:has(.ivg-run-page)` 限定桌面/平板锁高链路，手机宽度放行整页纵向滚动喵~
  - `web/index.html` viewport 补回旧版 `interactive-widget=resizes-content`，`RobotStatusBar` 补 `safe-area-inset-bottom` 高度和横向滚动，`SiteNav` 补触摸设备 44px 点击高度喵~
  - `start_aubo_new_driver.sh` 构建阶段新增 Vue 3 前端 `npm run build`，确保 `colcon build` 安装到 `share/aubo_ros2_web_dashboard/web/dist/` 的是最新页面产物喵~
  - 最终验收入口明确为 `aubo_ros2_ws/start_aubo_new_driver.sh` 启动整套系统后的 Web/ROS 端到端验证喵~
- **aubo_ros2_web_dashboard**: 修复导航栏切换导致 3D URDF 模型加载异常的问题（4 项根因）喵~
  - `SceneManager.stop()` 新增 `renderer.forceContextLoss()` + `renderer.domElement.remove()` — Three.js `r184` `dispose()` 不释放 WebGL 上下文，SPA 路由切换反复创建/销毁 WebGL 上下文 → 浏览器限制 ~16 个 → 3D 视图空白喵~
  - `Robot3dViewer.stop()` 新增 `unsubscribe()` 取消 4 个 rosbridge 话题订阅 (`/tf` `/tf_static` `/joint_states` `/tool_changer_status`) — 组件卸载后残留订阅占用带宽 + 阻止 rosbridge GC 喵~
  - `Robot3dViewer.init()` 新增 `initGen` 代数计数器 — async `init()` 在 `await` 后检查，若组件期间已卸载/重挂载则丢弃结果，防止操作已 dispose 场景或注册永不清理 handler 喵~
  - `CoffeeLatteView` 删除无 handler 的 `/tf` `/tf_static` 订阅及未使用的 import/computed — Robot3dViewer 已自管订阅，无需父 View 兜底喵~
  - `Robot3dViewer` 中 `onRosJson()` 从 catch-all (`topic: null`) 改为目标话题名精确匹配，减少每条消息触发不必要的字符串过滤喵~

### Added
- **graspnet_ros2**: 新增 `graspnet-baseline/install_graspnet_deps.sh`，用于新电脑克隆项目后按相对路径检测环境、编译安装 `graspnet-baseline`、`pointnet2`、`knn` 与 `graspnetAPI` 喵~
  - 脚本检查 `python3`、`pip`、`c++`、`ninja`、`nvcc`、`torch` 与 CUDA 可用性，并在安装后验证 `pointnet2._ext` 和 `knn_pytorch.knn_pytorch` 导入链喵~
  - `README.md` 更新为推荐使用安装脚本，并保留手工安装步骤作为排查参考喵~

#### scene_attach_worker：AttachedCollisionObject + world 清理（对齐当前源码，2026-05-16）

**背景**：仅凭动态 URDF / `/robot_description` 不能让 `move_group` 内的 `RobotModel` 随工具切换更新碰撞几何：`PlanningSceneMonitor` 不把 URDF topic 当作 robot model 增量来源，`RobotModelLoader` 通常只在启动时读一次 `robot_description` 参数喵~

**当前实现**（源码：`aubo_ros2_ws/src/tool_changer/src/scene_attach_worker.cpp`）：
1. **`/attached_collision_object`** — 发布 `AttachedCollisionObject` ADD/REMOVE，`PlanningSceneMonitor` 原生订阅（与启动日志 *Listening to '/attached_collision_object'* 一致）喵~
2. **`/planning_scene`（`is_diff=true`）** — 仅用于 `world.collision_objects` 中对 `attached_tool_<tool_id>` 的 **REMOVE**：detach 后 MoveIt 可能把同名对象放回 world，若不清理会导致与 `kuaihuan_Link` 等链路误判碰撞（快换释放笛卡尔路径 `fraction=0`）喵~
3. **不维护静态 dock world 物体** — 不向场景添加 dock 的 `ADD` 网格；与旧版「world dock 障碍物」方案不同喵~
4. **附着位姿语义（与 Git `748c7bb3d` 一致）**：`object.pose = tools.yaml.attach_offset`，`mesh_poses[0]` 为单位姿态，`link_name` 与 `header.frame_id` 均为 `kuaihuan_Link`喵~
5. **不负责 Web/RViz 工具网格显示** — 末端工具可视化由 Web `Robot3dViewer` + `/tool_changer_status` + TF + `attach_offset` 完成喵~

> 参考：MoveIt 2 Humble `PlanningSceneMonitor`（订阅 `/planning_scene` 与 `/attached_collision_object`）；本地日志 `move_group_*`: *Listening to '/attached_collision_object'* / *Listening to '/planning_scene'*喵~

#### SRDF ACM 添加 tool_tcp ↔ 末端固定链碰撞豁免 + 修复 tool_tcp/wrist3 关系 (2026-05-15)

**根因**：`tool_tcp` 和 `camera_Link` / `kuaihuan_Link` / `wrist3_Link` 都从 `wrist3_Link` 通过不同固定关节分叉，物理几何在 Z 方向重叠喵~

```
wrist3_Link (revolute)
  ├── tool_tcp       (fixed, xyz=(0,0,0.0235))  ← 分叉 A
  └── camera_Link    (fixed, xyz=(0,0,0.020))   ← 分叉 B
       └── kuaihuan_Link (fixed, xyz=(0,0,0.0215))
            └── gripperX_Link (fixed, xyz=(0,0,0.033))
```

`tool_tcp` mesh 和 `kuaihuan_Link` mesh / `camera_Link` mesh 的包围盒在 MoveIt 碰撞检测引擎中重叠 → RViz2 渲染红色碰撞（伪阳性）喵~

**修复**（`aubo_e5.srdf`）：
- `tool_tcp` ↔ `wrist3_Link`：`Adjacent` → **`Never`**（同父分叉固定子链，Z 方向重叠不可避）
- `tool_tcp` ↔ `camera_Link`：**新增 `Never`**（之前缺失，camera_Link 只有 visual+collision mesh，与 tool_tcp 重叠）
- `tool_tcp` ↔ `kuaihuan_Link`：**新增 `Never`**（之前缺失，kuaihuan_Link 同样有 physical collision mesh）
- 同时 `publish_robot_description_semantic: True` 确保 move_group 将更新后的 SRDF 发布给 RViz2 喵~

**RViz2 更换末端时的卡顿**是预期行为：`moveit_ros_visualization` 本地补丁需销毁旧的 `planning_scene_robot_` + `planning_scene_monitor_` 并重建 RobotModel，这个 `clearRobotModel()` → `loadRobotModel()` 过程约 50-100ms。不可消除但 ACM 修复可确保重载后不再显示红色碰撞喵~

#### Web Dashboard 3D 模型不显示 + 页面布局修复 (2026-05-15)

**根因**：`Robot3dViewer` 容器使用 `h-full`（100% of auto = 0），Three.js canvas 被渲染到高度为 0 的 div 中不可见。`VisionGraspView` 根元素 `overflow-y-scroll` 产生双滚动条并破坏子容器高度计算喵~

**修复**：
- `Robot3dViewer.vue` 模板：外层 div `min-h-[360px] h-full` → `h-[400px]`，内层 hostRef div `h-full` → `h-[400px]`，给 Three.js canvas 显式高度喵~
- `VisionGraspView.vue`：移除根 div 的 `overflow-y-scroll`，消除双滚动条并恢复正常布局喵~
- `SceneManager.ts`：`PerspectiveCamera` 中 `w / h` → `w / Math.max(1, h)` 防止除零喵~

**RViz2 碰撞警告说明**：`Link 'gripperX_Link' is not known to URDF. Cannot disable/enable collisions.` 是预期行为——SRDF ACM 预先列出全部 5 种末端工具的碰撞豁免，但当前 URDF 只含一种。MoveIt 规划不受影响喵~ URDF 热切换时的短暂卡顿因 `moveit_ros_visualization` 本地补丁的 `clearRobotModel` + 重建 RobotModel 是必要的喵~

#### Web Dashboard 新框架 UI 对齐旧框架功能 (2026-05-15)

基于旧框架（Git 历史 commit `24f4bdfee`）逐页面对比修复新 Vue 3 框架中所有缺失的 UI 元素和行为偏差喵~

**P0 修复 — 功能缺失**:
- `defaults.yaml`: 修复 `topic-robot` 默认值 `/robot_status` → `/aubo_driver/robot_status`（与旧版 `config.js` 和新 `constants/ros.ts` 对齐）喵~
- `CoffeeLatteView.vue`: 添加机械臂 URDF 3D 模型、关节曲线图、末端位姿面板、底部连接状态栏、话题设置入口、监控区折叠按钮（旧版 `coffee_latte_panel.html` 有这些元素）喵~
- `VisionGraspView.vue`: 添加工件模式"停止"按钮（`btn-wp-single-stop`，旧版 `services.js:100-106` 调用 `SetBool(false)` 停止循环）喵~

**P1 修复 — 行为偏差**:
- `VisionGraspView.vue`: 末端位姿添加 `robotPoseCache` 缓存（数据空时不闪回默认值，旧版 `vision_grasp_panel.js:239`），添加 `ivg_display` 字段回退显示（旧版 `pose_card.js:128`）喵~
- `VisionGraspView.vue`: AI 抓取位姿改为富文本 HTML 格式化（`formatPoseBlockHtml` 生成双 section：位姿 + 四元数，对齐旧版 `pose_card.js:77-96`）喵~
- `VisionGraspView.vue`: AI 模式同时刷新左栏相机快照和结果图（旧版 `vision_grasp_panel.js:380-391` 两幅图都更新）喵~
- `VisionGraspView.vue`: 连接状态栏显示重连倒计时文本（`c.message` 而非固定字符串）喵~
- `CoffeeLatteView.vue`: DO 开关改为纯前端切换（不强制调用后端服务，与旧版 `coffee_latte_io.js:4` 注释一致）喵~

**P2 修复 — 子功能**:
- `LogView.vue`: 添加页面生命周期事件（`visibilitychange`/`beforeunload`/`pagehide`，旧版 `log_panel.js:205-216`）喵~
- `base.css`: 新增 `pose_card` 内联样式类（`pose-card__body`/`pose-card__pill` 等）供 `v-html` 渲染使用喵~
- `Robot3dViewer.vue`: 删除重复的 `onUnmounted` 块喵~

**对比方法论**: 从 Git 提取旧框架全部 43 个源文件（commit `24f4bdfee`），逐个与新 Vue 3 文件进行结构级比对，确保不漏掉任何功能喵~

#### 3D 查看器响应式修复 + robotwebtools 清理 + C++/Python 依赖修复 (2026-05-15)

**前端**:
- **3D 模型卡在"等待连接"**: `Robot3dViewer.vue` 中 `sceneMgr` 是普通 `let`，非响应式 → 模板永不更新。改为 `shallowRef<SceneManager | null>(null)` 喵~
- **roslib `Ros` 对象私有字段冲突**: npm `roslib` 的 `isConnected` getter 读私有字段 `#e`，Vue Proxy 拦截报 `Cannot read from private field`。`rosInstance` 改为普通变量脱离响应式系统喵~
- **`unsubscribeAll` CLOSING 警告**: 断连时跳过网络消息，只清空本地 Map。注意 Vite 增量构建缓存可能导致修改不生效，需 `rm -rf dist/ .vite/` 强制重建喵~
- **`connectPromise` 泄漏**: 成功后未清空 → 重连直接返回旧 Promise → 重连失效。`finally` 块保证清理喵~
- **`robotwebtools/` 整体删除**: npm `roslib` 已替代本地构建。清理启动脚本和 launch 文件中的 `ROBOTWEBTOOLS_*` 变量和 `_find_rwt_assets()` 喵~

**C++ 运动控制与 IO**:
- **`bad_weak_ptr` 崩溃**: `RobotController` 构造函数中 `owner->shared_from_this()` → 3 个 Worker 节点启动崩溃。改为两阶段初始化（参考 MoveIt2 官方 `MoveGroupInterface(shared_ptr<Node>)` 签名）：构造函数只存指针，`init()` 创建 `MoveGroupInterface` 喵~
- **`publish_grasps_client_worker` 同类崩溃**: 延后 `RobotController` 初始化到 `make_shared` 之后喵~
- **`moveCartesianPath` API 迁移**: 6 处旧式调用 `('z', val, v, a)` → 新 API `({{'z', val}}, v, a)` 喵~
- **`setGripper` API 迁移**: 8 处 `setGripper(bool)` → `setGripperIoSafe(bool)` 喵~
- **`getCurrentPose` 返回类型**: `PoseStamped` → `.pose` 提取 `Pose` 喵~
- **`moveToHome` 一致性**: 补充 `setStartStateToCurrentState()` 喵~
- **`UrdfParser` axis 解析**: `||` 吞掉合法 0 值 → 改用 `isFinite()` 检查喵~
- **`TfUpdater` 关节旋转**: `rotateOnWorldAxis` → `rotateOnAxis`（局部轴），关节轴从 URDF 传递到更新函数喵~
- **析构函数声明缺失**: `ExecuteGraspPoseWorker` 头文件缺少 `~ExecuteGraspPoseWorker() override` 声明喵~
- **`auto` lambda 模板推导失败**: 5 个 server 文件中 `create_service` 的 `[](auto req, auto res)` → 显式类型；`set_parameters` 回调同理喵~
- **`computeCartesianPath` 参数缺失**: 补上 `kCartesianEefStep, kCartesianJumpThreshold` 喵~

**启动脚本**:
- **`set -e` 杀死脚本**: 11 处 `active_wait` 超时返回 1 → `set -e` 立即退出 → 后续终端标签未创建。全部加 `|| true` 喵~
- **所有终端清除代理**: `launch()` 函数统一 `unset http_proxy https_proxy ...`，不再仅 Web 网关喵~
- **URL 更新**: `vision_grasp_panel.html` → `/vision`，`coffee_latte_panel.html` → `/latte`（Vue 3 SPA 路由）喵~

**Python 依赖**:
- **NumPy 2.x 不兼容**: `cv2`/`cv_bridge` 编译时用 NumPy 1.x；`transforms3d` 用 `np.float` 已删除 → `numpy==1.23.5` 喵~
- **matplotlib 版本冲突**: 系统 `mpl_toolkits` vs 用户 pip `matplotlib` → 卸载 pip 版使用系统版喵~
- **useRos 自动重连**: 新增指数退避自动重连（12 次，1s→30s），`visibilitychange` 后台暂停/恢复，`online` 事件即时重连，`pagehide` 清理连接。与旧框架 `ivgPorts.scheduleRosReconnect` 逻辑一致喵~
- **ToolSwapBar 服务调用修复**: 使用 `/run_gripper_swap` 服务 + `direction` 参数（`current_to_target`），替代错误的 `/change_tool` + `tool_id`喵~
- **camera_info 动态推导**: `buildCameraInfoTopic()` 从 color 话题名自动推导 camera_info 话题名，不再硬编码喵~
- **AI 模式快照回退**: snapshot URL 加载失败时自动回退到 MJPEG 流，与旧框架 `setAiColorSnapshotImg` fallback 逻辑一致喵~
- **App.vue 全局 ROS 连接**: 应用根组件挂载时自动建立 rosbridge 连接，确保直接访问 `/latte`、`/monitor` 等子页面也能正常连接，不再依赖先访问 `/vision` 喵~
- **移除死代码**: `useRosService` 删除未使用的 `getCurrentTool` 函数；`VisionGraspView` 修复重复 `useRos()` 调用喵~

### Changed

#### Vue 3 前端功能补全 — 替代 ros3djs/ros2djs 为 Three.js 原生实现 (2026-05-15)
- **robotwebtools**: 删除 ros3djs/ros2djs 目录（过于陈旧），保留 roslibjs 本地构建。构建脚本 `build_robotwebtools.sh` / `copy_runtime_js_assets.sh` / `runtime_js_assets/importmap.js` 同步更新为仅 roslibjs 喵~
- **前端依赖**: npm 安装 `three` + `@types/three`（Three.js 3D 引擎）、`roslib`（roslibjs npm 包替代本地 vendor）、`@types/node`（构建时类型支持）喵~
- **3D 机械臂查看器**: 新建 `three_urdf/SceneManager.ts`（场景/相机/OrbitControls）、`UrdfParser.ts`（URDF XML 解析）、`UrdfModel.ts`（URDF→Three.js Object3D 树 + STL 加载）、`TfUpdater.ts`（TF+joint_states→关节更新）。集成在 `Robot3dViewer.vue` 组件中，支持拖拽旋转/缩放、工具快换时无闪烁 reload、自动 camera focus 喵~
- **关节角曲线图**: 新建 `useJointChart.ts` composable — Canvas 2D 折线图，6 关节 × 280 采样点，Y 轴自适应、图例、ResizeObserver 响应。与旧框架逻辑完全一致喵~
- **抓取位姿投影叠加层**: 新建 `useProjectionOverlay.ts` composable — Canvas 2D 在相机图像上叠加绘制夹爪形状（手指/腕部/标签），复用已有 `tf_math.ts` TF 数学库。与旧框架逻辑完全一致喵~
- **VisionGraspView 完整重写**: 左栏 3D 模型 + 相机画面（含投影叠加层），右栏抓取控制 + 工具快换 + 调试 XYZ 移动 + 服务日志，底部可折叠监控区（关节曲线 + 末端位姿 + VPE/AI 状态）。AI 模式自动使用 snapshot 替代 MJPEG 流，收到抓取位姿后防抖 400ms 刷新喵~
- **useRos 增强**: 新增 `onLog` 事件钩子（subscribe/unsubscribe/service_call/service_result/service_error），LogView 改用此钩子替代 `globalThis.ivgTransport` 轮询喵~
- **useMJPEGStream 增强**: 新增 `cameraSnapshotUrl()` 方法，支持 JPEG 快照 URL 构建（AI 模式用）喵~
- **vite.config.ts / tsconfig.json**: roslib 别名指向 npm 包，移除 ros3d 别名。tsconfig.node.json 添加 `skipLibCheck` 喵~

### Removed
- **robotwebtools/ros3djs/** — 删除整个 ros3djs 源码和构建产物，前端改由 Three.js 原生实现 3D 渲染喵~
- **robotwebtools/ros2djs/** — 删除整个 ros2djs 源码和构建产物，未使用的 2D 地图功能喵~
- **web/src/lib/ros3d_patches.ts** — 删除，patch 仅用于已移除的 ros3djs 喵~

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
- **tool_changer**: （历史记录，已由后续重构废止）曾一度描述为：`scene_attach_worker` 发布 URDF 到 `/robot_description` 并用 PlanningScene diff 管理碰撞喵~ **当前源码**不再发布 URDF；工具碰撞附着走 **`/attached_collision_object`**，detach 后 world 残留清理走 **`/planning_scene`（world REMOVE）** — 见 `[Unreleased]` 节与本仓库 `tool_changer/README.md`喵~
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
