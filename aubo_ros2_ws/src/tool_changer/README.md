# tool_changer — 快换与手爪管理

gripper0 ↔ gripper2 双向自动快换，工具状态发布与查询。

**架构（2026-05-14 重构）**：轨迹参数全部数据驱动，存于 `config/tools.yaml`。`gripper_swap_worker` 通过 `loadToolConfig()` 加载，`changeToTool()` 实现通用的"释放当前→取目标→回 home"流水线，不再有硬编码 per-gripper 函数。新增可快换工具只需编辑 `tools.yaml`，无需修改 C++ 代码喵~

### tools.yaml 轨迹字段

每个可快换工具需以下额外字段：

```yaml
tools:
  gripper0:
    # ... 原有字段 (mesh_visual, dock_pose, attach_offset, touch_links) ...
    dock_approach_joints: [1.137820, 0.222690, 1.598043, -0.194970, 1.571688, 1.136957]
    trajectory:
      strategy: "vertical"     # "vertical" 或 "slide"
      depth: 0.210             # 取/放下降深度 (m)
      lift: 0.210              # 取/放抬起高度 (m)
      settle_sec: 0.5          # IO 动作后稳定延时 (s)
      # slide 策略额外参数:
      # slide_y: 0.100         # Y 轴侧滑距离 (m)
      # seat: 0.012            # 锁止机构坐入/脱扣高度 (m)
      # release_sec: 0.3       # 释放后稳定延时 (s)
      # lock_sec: 0.5          # 锁紧后稳定延时 (s)
```

| 字段 | 必需 | 说明 |
|------|------|------|
| `dock_approach_joints` | 是 | 6 关节角 (rad)，工具 dock 接近位 |
| `trajectory.strategy` | 否 (默认 `"vertical"`) | 轨迹策略：`"vertical"`（纯 Z 轴）或 `"slide"`（Y 轴侧滑+分段 Z） |
| `trajectory.depth` | 否 (默认 0.210) | 取/放下降深度 |
| `trajectory.lift` | 否 (默认 0.210) | 取/放抬起高度 |
| `trajectory.settle_sec` | 否 (默认 0.5) | vertical 策略的 IO 稳定延时 |
| `trajectory.slide_y` | slide 必需 | Y 轴侧滑距离 |
| `trajectory.seat` | slide 必需 | 锁止机构坐入/脱扣高度 |
| `trajectory.release_sec` | slide 必需 | slide 放策略的释放后稳定延时 |
| `trajectory.lock_sec` | slide 必需 | slide 放策略的锁紧后稳定延时 |

## AUBO E5 机械臂规格

| 参数 | 数值 | 来源 |
|------|------|------|
| 自由度 | 6 旋转关节 | — |
| 最大负载 | 5 kg | 型号 |
| 工作半径 | 886.5 mm | 资料 |
| 重复定位精度 | ±0.02 mm | 资料 |
| 本体重量 | ~24 kg | 资料 |
| 防护等级 | IP54 | 资料 |
| 安装方式 | 任意角度 | 资料 |
| 功耗 | 200W (典型) / 500W (最大) | 资料 |

### 关节限制 (来源: `aubo_e5_10.urdf`)

| 关节 | 运动范围 | 最大速度 | 最大力矩 |
|------|----------|----------|----------|
| shoulder_joint (J1) | ±175° (±3.054 rad) | 2.5964 rad/s | 133 Nm |
| upperArm_joint (J2) | ±175° (±3.054 rad) | 2.5964 rad/s | 133 Nm |
| foreArm_joint (J3) | ±175° (±3.054 rad) | 2.5964 rad/s | 133 Nm |
| wrist1_joint (J4) | ±175° (±3.054 rad) | 3.1105 rad/s | 13.5 Nm |
| wrist2_joint (J5) | ±175° (±3.054 rad) | 3.1105 rad/s | 13.5 Nm |
| wrist3_joint (J6) | ±175° (±3.054 rad) | 3.1105 rad/s | 13.5 Nm |

> 大关节 (J1-J3) 速度 ~149°/s，小关节 (J4-J6) 速度 ~178°/s。力矩以大关节高一个数量级。

### 运动学链 (来源: `aubo_e5_10.urdf` + `aubo_e5.urdf.xacro`)

```
world → pedestal_Link → base_link
  → shoulder_joint → upperArm_joint → foreArm_joint
  → wrist1_joint → wrist2_joint → wrist3_joint → wrist3_Link
      ├─ camera_joint (z=+0.020) → camera_Link
      │    └─ kuaihuan_joint (z=+0.0215, ry=π) → kuaihuan_Link
      │          └─ kuaihuan_to_${name} (z=+0.033) → 工具 Link
      └─ wrist3_to_tcp (z=+0.0235) → tool_tcp (MoveIt EEF)
```

> TCP 位于 wrist3_Link 前方 23.5mm，kuaihuan 快换法兰位于 wrist3_Link 前方 41.5mm。

## 节点

### gripper_swap_worker_node

| 属性 | 值 |
|------|-----|
| 功能 | 夹爪快换 Worker：gripper0 ↔ gripper2 双向自动快换 |
| 依赖 | tool_changer_interface, demo_interface (SetRobotIO), moveit |
| 多线程 | MultiThreadedExecutor(2)：回调线程执行快换，spin 线程处理 MoveIt/IO 响应 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/run_gripper_swap` | tool_changer_interface/srv/RunGripperSwap | 执行快换，direction: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| `/change_tool` | tool_changer_interface/srv/ChangeTool | 按 tool_id 切换工具 |
| `/get_current_tool` | tool_changer_interface/srv/GetCurrentTool | 查询当前工具状态 |

### 话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/tool_changer_status` | tool_changer_interface/msg/ToolChangerStatus | 快换盘状态（工具 ID / 名称 / 类型 / 连接状态） |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_velocity_scaling` | 0.7 | 关节速度缩放 |
| `joint_acceleration_scaling` | 0.3 | 关节加速度缩放 |
| `home_velocity_scaling` | 0.7 | 回安全位速度缩放 |
| `home_acceleration_scaling` | 0.3 | 回安全位加速度缩放 |
| `gripper_io_index` | 7 | 快换 IO pin 号 |
| `joint_cartesian_switch_delay_sec` | 0.05 | 关节↔笛卡尔切换延时 |

### scene_attach_worker_node

| 属性 | 值 |
|------|-----|
| 功能 | 1) PlanningScene 工具碰撞模型附着显示 2) robot_description 实时更新 (Web 前端模型同步) |
| 依赖 | tool_changer_interface, moveit_msgs, yaml-cpp, aubo_moveit_config (xacro) |

#### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/scene_attach` | tool_changer_interface/srv/ChangeTool | 手动附着：工具碰撞网格 → kuaihuan_Link |
| `/scene_detach` | tool_changer_interface/srv/ChangeTool | 手动脱离：工具碰撞网格 → world dock |

#### 话题 (发布)

| 话题名 | 类型 | QoS | 说明 |
|--------|------|-----|------|
| `/planning_scene` | moveit_msgs/msg/PlanningScene | transient_local(10) | 增量更新：AttachedCollisionObject 附着/脱离 |
| `/robot_description` | std_msgs/msg/String | transient_local(10) | 含当前末端工具的完整 URDF (xacro 实时渲染) |

#### 话题 (订阅)

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/tool_changer_status` | tool_changer_interface/msg/ToolChangerStatus | 物理快换状态变化 → 自动触发场景更新 + URDF 更新 |

## 话题/服务完整索引

### 话题

| 话题 | 类型 | 发布者 | QoS | 说明 |
|------|------|--------|-----|------|
| `/tool_changer_status` | `ToolChangerStatus` | gripper_swap_worker | default | 工具状态: `tool_id` (当前工具ID/空), `tool_name` (中文名), `tool_type` (gripper/other), `is_connected` (是否已连接) |
| `/planning_scene` | `PlanningScene` | scene_attach_worker | transient_local(10) | 增量场景更新: `world.collision_objects` (dock 位姿), `robot_state.attached_collision_objects` (kuaihuan_Link 附着) |
| `/robot_description` | `String` | scene_attach_worker | transient_local(10) | 含当前末端工具的完整 URDF 字符串 (xacro 实时渲染)，供 robot_state_publisher + Web 前端加载 |

### 服务

| 服务 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/change_tool` | `ChangeTool` | gripper_swap_worker | 按 tool_id 自动快换 (运动 + IO), 返回 success/error_code/message |
| `/run_gripper_swap` | `RunGripperSwap` | gripper_swap_worker | 按方向快换: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| `/get_current_tool` | `GetCurrentTool` | gripper_swap_worker | 查询当前工具: 返回 tool_id/tool_name/tool_type/is_connected |
| `/scene_attach` | `ChangeTool` | scene_attach_worker | 手动附着碰撞网格到 kuaihuan_Link (5 种末端均支持) |
| `/scene_detach` | `ChangeTool` | scene_attach_worker | 手动脱离碰撞网格回 world dock |

### IO 控制

| IO | 逻辑引脚 | 硬件引脚 | 用途 |
|----|----------|----------|------|
| 快换盘锁紧/释放 | 7 | 39 | gripper_swap_worker 通过 `/aubo_driver/set_io` 控制 |

### 消息定义

`ToolChangerStatus` (tool_changer_interface/msg/ToolChangerStatus):
```
string tool_id       # 工具 ID (gripper0/gripper1/gripper2/gripper1coffeecup/gripper1milkcup)
string tool_name     # 中文名称
string tool_type     # 类型 (gripper/other)
bool is_connected    # 是否已连接到快换盘
```

`ChangeTool` (tool_changer_interface/srv/ChangeTool):
```
# Request
string tool_id

# Response
bool success
int32 error_code
string message
```

`RunGripperSwap` (tool_changer_interface/srv/RunGripperSwap):
```
# Request
string direction   # gripper0_to_gripper2 / gripper2_to_gripper0 / gripper2

# Response
bool success
int32 error_code
string message
```

```bash
# 一键启动两个节点
ros2 launch tool_changer gripper_swap_worker.launch.py

# 或分别启动
ros2 run tool_changer gripper_swap_worker_node
ros2 run tool_changer scene_attach_worker_node
```

## 测试命令

### 物理快换（仅 gripper0 / gripper2）

```bash
# 查看当前工具
ros2 service call /get_current_tool tool_changer_interface/srv/GetCurrentTool

# 切换工具
ros2 service call /change_tool tool_changer_interface/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /change_tool tool_changer_interface/srv/ChangeTool "{tool_id: gripper0}"

# 按方向快换
ros2 service call /run_gripper_swap tool_changer_interface/srv/RunGripperSwap "{direction: gripper2}"
ros2 service call /run_gripper_swap tool_changer_interface/srv/RunGripperSwap "{direction: gripper0_to_gripper2}"
ros2 service call /run_gripper_swap tool_changer_interface/srv/RunGripperSwap "{direction: gripper2_to_gripper0}"
```

### 场景附着（5 种末端全部支持）

```bash
# 附着到机械臂
ros2 service call /scene_attach tool_changer_interface/srv/ChangeTool "{tool_id: gripper0}"
ros2 service call /scene_attach tool_changer_interface/srv/ChangeTool "{tool_id: gripper1}"
ros2 service call /scene_attach tool_changer_interface/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /scene_attach tool_changer_interface/srv/ChangeTool "{tool_id: gripper1coffeecup}"
ros2 service call /scene_attach tool_changer_interface/srv/ChangeTool "{tool_id: gripper1milkcup}"

# 脱离，回到 dock
ros2 service call /scene_detach tool_changer_interface/srv/ChangeTool "{tool_id: gripper0}"
ros2 service call /scene_detach tool_changer_interface/srv/ChangeTool "{tool_id: gripper1}"
ros2 service call /scene_detach tool_changer_interface/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /scene_detach tool_changer_interface/srv/ChangeTool "{tool_id: gripper1coffeecup}"
ros2 service call /scene_detach tool_changer_interface/srv/ChangeTool "{tool_id: gripper1milkcup}"
```

### 监控

```bash
# 工具状态
ros2 topic echo /tool_changer_status

# 场景更新
ros2 topic echo /planning_scene

# URDF 更新 (含当前工具)
ros2 topic echo /robot_description
```

## 数据流：工具切换 → 纯 URDF 多端同步

```
gripper_swap_worker.changeToTool()
  │
  ├── 1. publishToolStatus(false)          ← 切换前清除状态（避免附着碰撞干扰运动）
  │
  ├── 2. 执行物理快换（运动 + IO）
  │
  └── 3. publishToolStatus("gripper0")     ← 切换完成后发布新状态
          │
          └→ scene_attach_worker.onToolStatus()
                ├── addToWorldDock(旧工具)    ← PlanningScene: dock 恢复障碍物
                ├── removeFromWorldDock(新工具) ← PlanningScene: dock 移除障碍物
                │
                └── updateRobotDescription()
                      ├── xacro 渲染含工具的完整 URDF
                      ├── publish /robot_description topic
                      └── set_parameters /robot_state_publisher
                            │
                            ├→ robot_state_publisher.onParameterEvent()
                            │     └→ setupURDF(新 URDF)
                            │           ├→ 发布 gripper0_Link TF 帧
                            │           └→ publish /robot_description topic
                            │
                            ├→ MoveIt PlanningSceneMonitor
                            │     └→ 订阅 /robot_description topic
                            │     └→ reloadRobotModel()  ← 碰撞模型从 URDF <collision> 原生获取
                            │
                            ├→ RViz2 RobotModel
                            │     └→ 参数变更 / 话题更新 → 重建显示模型
                            │
                            └→ Web Frontend (rosbridge)
                                  └→ get_param robot_description → ROS3D.UrdfClient
```

**核心设计决策 — 纯 URDF 方案**：

不再使用 `PlanningScene AttachedCollisionObject`，改为仅通过 `robot_description` 参数更新 URDF：

1. **碰撞由 URDF `<collision>` 原生提供**：MoveIt 从 URDF 解析碰撞几何，用 SRDF 的 ACM (Allowed Collision Matrix) 做自碰撞判断，无需 `touch_links`
   - **ACM 配置位置**：`aubo_moveit_config/config/aubo_e5.srdf:69-89`，20 条 `<disable_collisions>` 覆盖 5 种末端工具 × 4 个末端固定链 link（`kuaihuan_Link`, `camera_Link`, `wrist3_Link`, `tool_tcp`），详见 CLAUDE.md 规则 12b 喵~
2. **一条路径同时覆盖 RViz2 + Web**：`robot_state_publisher` 接收参数变更 → `setupURDF()` → 发布新 TF + `/robot_description` 话题 → MoveIt PlanningSceneMonitor / RViz2 RobotModel / Web ROS3D 全部自动同步
3. **切换前清除，避免运动干涉**：`gripper_swap_worker.changeToTool()` 入口处 `publishToolStatus(false)` 触发 `updateRobotDescription()`（无工具 URDF），运动期间机械臂上无夹爪碰撞模型，规划不受干扰
4. **无 z-fighting**：只有 URDF 唯一表示，不存在 PlanningScene 附着体与 RobotModel 重叠问题

- **位姿一致**：`aubo_e5.urdf.xacro` 中 `kuaihuan_to_${name}` 的 origin 与 `tools.yaml` 中各工具的 `attach_offset` 严格对齐（gripper0/1/2: z=0.033, rpy=0; coffeecup: ry=π; milkcup: ry=-π/2）
- **transient_local QoS**：保证重启后的 Web Dashboard / MoveIt 订阅者能收到最新的场景状态和 URDF

## 话题/服务完整索引

### 末端工具 attach_offset 计算

### 背景

工具碰撞网格通过 `scene_attach_worker` 附着到 `kuaihuan_Link`（快换法兰）上。
`attach_offset` 决定工具 mesh 原点在 `kuaihuan_Link` 坐标系中的位置。
目标：工具顶面与法兰安装面恰好接触，**无重叠、无间隙**。

### 方法

用 Python `struct` 逐三角形解析 STL 二进制文件，取所有顶点坐标的极值得到包围盒。
脚本位于 `scripts/measure_mesh.py`。

### 坐标系约定

URDF 中 Z+ 沿机械臂末端**向外**（指向工具方向），`kuaihuan_joint` 的 rpy (0, 0, π) 绕 Z 轴 180°，不翻转 Z 方向，因此 `kuaihuan_Link` 的 Z+ 也指向外。

```
wrist3_Link
  ├─ camera_joint   origin(0, 0, 0.020) ─→ camera_Link
  │    └─ kuaihuan_joint  origin(0, 0, 0.0215), rpy(0, 0, π) ─→ kuaihuan_Link
  │          └─ kuaihuan_to_${name}  origin(0, 0, 0.033) ─→ 工具 Link
  └─ wrist3_to_tcp  origin(0, 0, 0.0235) ─→ tool_tcp
```

### 步骤 1：实测 STL 网格包围盒

```
网格                     Z_min       Z_max       高度
───────────────────────────────────────────────────────
kuaihuan (碰撞/视觉)     -0.0268     +0.0330     59.8mm
gripper0                 -0.0038     +0.1405    144.3mm
gripper1                 -0.0038     +0.1784    182.2mm
gripper2                 -0.0038     +0.1768    180.6mm
gripper1coffeecup        -0.0038     +0.1920    195.8mm
gripper1milkcup          -0.0038     +0.2006    204.4mm
```

关键发现：
- kuaihuan mesh **Z_max = +0.0330** 位于 XY≈(0, 0) 中心，是法兰的工具对接面（安装面）
- 所有工具 mesh 原点以下 **Z_min = -0.0038**：工具原点在安装面，mesh 向原点之上（Z-）延伸 3.8mm，这是**快换锁止机构的机械咬合部分**

### 步骤 2：确定安装面对齐点

工具的安装面是**工具原点**（Z=0 在工具 Link 坐标系），不是 Z_min 极值点。
法兰的安装面是 Z_max = **+0.0330**（法兰最远端）。

```
对齐条件: 工具原点(即安装面) = 法兰安装面

    offset = Z_max_kuaihuan = 0.0330
```

### 步骤 3：验证

```
offset = 0.033:

工具 mesh 在 kuaihuan 坐标系中的 Z 范围:
  = [0.033 + (-0.0038),  0.033 + 0.1405]
  = [0.0292,             0.1735]

kuaihuan mesh Z 范围:
  = [-0.0268,            0.0330]

安装面: 工具 Z=0.033 = 法兰 Z_max=0.033          → 对齐 ✅
咬合量: 0.033 - 0.0292 = 0.0038m = 3.8mm          → 模拟锁止机构进入法兰 ✅
```

### 步骤 4：为什么旧值不对

| offset | 效果 | 原因 |
|--------|------|------|
| 0.020（旧） | 重叠 16.8mm | 工具被塞进法兰过深 |
| 0.0368（上一版） | 间隙 3.8mm | 误把 mesh Z_min 尖端点当安装面，安装面悬空 |

### 配置位置

| 文件 | 字段 | 值 |
|------|------|----|
| `config/tools.yaml` | 每个工具的 `attach_offset.position.z` | `0.033` |
| `aubo_moveit_config/config/aubo_e5.urdf.xacro` | `kuaihuan_to_${name}` origin z | `0.033` |

---

## 场景更新双路径架构 & 实战排错记录

### 背景：模型不更新的排错

**现象**：工具快换完成后，RViz2 和 Web 前端均不显示新工具模型。

**排查过程**：第一阶段 `scene_attach_worker_node` 直接 SIGSEGV (exit code -11)，第二阶段 `bad_weak_ptr` 崩溃，第三阶段节点正常运行但模型仍不更新。最终发现是架构设计问题。

### Bug #1: SIGSEGV — 空指针解引用

**根因**：构造函数中 `updateRobotDescription()` 先于 `AsyncParametersClient` 创建被调用，内部访问未初始化的成员指针。

```cpp
// 错误顺序
updateRobotDescription();                          // 内部调用 robot_state_params_client_->wait_for_service()
robot_state_params_client_ = make_shared<...>();   // 太晚了！已崩溃
```

GDB 栈帧确认：
```
#0  rclcpp::AsyncParametersClient::wait_for_service_nanoseconds()
#2  tool_changer::SceneAttachWorker::updateRobotDescription()
#3  tool_changer::SceneAttachWorker::SceneAttachWorker()  ← 构造函数
```

**修复**：不要在构造函数中调用 `updateRobotDescription()`——初始 URDF 已由 `robot_state_publisher` 在系统启动时发布，无需 scene_attach_worker 再发一次。

### Bug #2: bad_weak_ptr — 构造函数中调用 shared_from_this()

**根因**：`rclcpp::AsyncParametersClient` 构造函数需要 `Node::SharedPtr`，但 ROS 2 的 `enable_shared_from_this` 机制在构造函数**内**尚未就绪。虽然 `make_shared` 已分配控制块，但 `rclcpp::Node` 的 `shared_from_this()` 走 `NodeBaseInterface` 路径，在构造阶段仍会抛出 `std::bad_weak_ptr`。

**修复**：延迟初始化——在 `onToolStatus()` 回调中（此时节点已由 `rclcpp::spin` 管理）首次调用时按需创建 `AsyncParametersClient`。

### Bug #3: robot_state_publisher 不订阅 /robot_description 话题

**调查过程**：查阅 [ROS 2 Humble `robot_state_publisher` 源码](https://github.com/ros/robot_state_publisher/blob/humble/src/robot_state_publisher.cpp)：

```cpp
// 关键发现：robot_state_publisher 没有 /robot_description topic subscriber！
// 它只在 startup 时通过 declare_parameter 读取，运行时通过 /parameter_events 监听变更
void RobotStatePublisher::onParameterEvent(...)
{
    if (event->node != this->get_fully_qualified_name()) return;  // 只处理本节点参数变更
    // ...
    setupURDF(new_urdf);  // 重新解析 URDF → 发布新 TF 帧
}
```

`setupURDF()` 校验逻辑 (`parameterUpdate`):
1. URDF 非空 → 否则拒绝 `"Empty URDF is not allowed"`
2. `urdf::Model::initString()` + `kdl_parser::treeFromUrdfModel()` 解析成功 → 否则拒绝

**结论**：**发布 `/robot_description` 话题无法让 `robot_state_publisher` 更新 TF！** 必须通过 `set_parameters` 设置 `robot_description` 参数 → 触发 `onParameterEvent` → `setupURDF()`。

### Bug #4: ROS 2 参数隔离 — 每个节点独立副本

**背景知识**：ROS 2 **没有全局参数服务器**（与 ROS 1 的 `rosparam` 不同）。`launch` 文件中 `parameters=[robot_description]` 会把参数**分别写入每个节点的私有存储**：

```python
rsp_node       = Node(... parameters=[robot_description])  # robot_state_publisher 自己的副本
move_group_node = Node(... parameters=[robot_description]) # move_group 自己的副本
rviz_node      = Node(... parameters=[robot_description])  # rviz2 自己的副本
```

修改 `/robot_state_publisher` 的参数**不会影响** `/move_group` 和 `/rviz2`。

| 节点 | 用途 | 不更新后果 |
|------|------|-----------|
| `/robot_state_publisher` | 建 KDL 树 → 发布 TF 帧 | 不发布 `gripper0_Link` TF |
| `/move_group` | 建 RobotModel → 碰撞检测 + 规划 | MoveIt 不把夹爪纳入碰撞模型 |
| `/rviz2` | 建视觉模型 → 渲染 RobotModel | 即使有 TF 也不知 gripper mesh 长什么样 |

### 最终架构：纯 URDF 方案

经实测验证，运行时更新 `robot_description` 参数 → `robot_state_publisher.onParameterEvent()` → `setupURDF()` → 发布 `/robot_description` 话题 → MoveIt `PlanningSceneMonitor` 自动 `reloadRobotModel()` 整个链路正常工作。因此不需要双路径，只用 URDF 一条路同时搞定碰撞、RViz2 显示、Web 显示。

**碰撞安全**：`gripper_swap_worker.changeToTool()` **入口处**发布空工具状态 → `scene_attach_worker` 立即更新 URDF 为无工具版本 → 后续切换运动中机械臂上无夹爪碰撞体 → 不会发生自碰撞干涉。

### RViz2 显示

只用 `RobotModel` 显示即可（URDF 路径自动更新），不再有 PlanningScene 附着体 → **无 z-fighting**。

### 关键实现细节

1. **`setupURDF()` 会重新发布 `/robot_description` 话题**：`robot_state_publisher` 收到参数变更后会调用 `setupURDF()`，后者内部 `description_pub_->publish()` 把新 URDF 重新发到话题（供 MoveIt 的 PlanningSceneMonitor 等订阅者读取）。

2. **`AsyncParametersClient` 延迟初始化**：`updateRobotDescription()` 在 `onToolStatus` 回调中首次被调用时创建，此时节点已由 `rclcpp::spin` 管理，`shared_from_this()` 正常工作。构造函数中不调用。

3. **xacro 渲染的一致性**：`scene_attach_worker` 调用 xacro 的路径必须与主 launch 中 `MoveItConfigsBuilder.robot_description()` 生成的是同一个 `.xacro` 文件，确保基础 URDF 结构一致（只是多了 `gripper:=` 参数）。

4. **仅更新 `robot_state_publisher` 参数**：`setupURDF()` 内部会重新发布 `/robot_description` 话题，MoveIt 订阅此话题自动更新模型，无需逐节点设置参数。
