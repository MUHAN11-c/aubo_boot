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
| 依赖 | ivg_interfaces, moveit |
| 多线程 | MultiThreadedExecutor(2)：回调线程执行快换，spin 线程处理 MoveIt/IO 响应 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/run_gripper_swap` | ivg_interfaces/srv/RunGripperSwap | 执行快换，direction: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| `/change_tool` | ivg_interfaces/srv/ChangeTool | 按 tool_id 切换工具 |
| `/get_current_tool` | ivg_interfaces/srv/GetCurrentTool | 查询当前工具状态 |

### 话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/tool_changer_status` | ivg_interfaces/msg/ToolChangerStatus | 快换盘状态（工具 ID / 名称 / 类型 / 连接状态） |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_velocity_scaling` | 0.7 | 关节速度缩放 |
| `joint_acceleration_scaling` | 0.3 | 关节加速度缩放 |
| `home_velocity_scaling` | 0.7 | 回安全位速度缩放 |
| `home_acceleration_scaling` | 0.3 | 回安全位加速度缩放 |
| `gripper_io_index` | 7 | 快换 IO pin 号 |
| `joint_cartesian_switch_delay_sec` | 0.05 | 关节↔笛卡尔切换延时 |
| `initial_tool_id` | `""` (空) | 启动时预设的末端工具 ID（如 `gripper0`/`gripper2`）。设为空字符串表示以无工具状态启动喵~ |

### scene_attach_worker_node

| 属性 | 值 |
|------|-----|
| 功能 | PlanningScene 已附着工具碰撞模型 ADD/REMOVE |
| 依赖 | ivg_interfaces, moveit_msgs, yaml-cpp, resource_retriever |

#### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/scene_attach` | ivg_interfaces/srv/ChangeTool | 手动附着：工具碰撞网格 → kuaihuan_Link |
| `/scene_detach` | ivg_interfaces/srv/ChangeTool | 手动脱离：移除已附着工具碰撞网格 |

#### 话题 (发布)

| 话题名 | 类型 | QoS | 说明 |
|--------|------|-----|------|
| `/attached_collision_object` | moveit_msgs/msg/AttachedCollisionObject | transient_local(10) | ADD/REMOVE：工具网格附着于 `kuaihuan_Link`（与源码 `create_publisher` QoS 一致）喵~ |
| `/planning_scene` | moveit_msgs/msg/PlanningScene | transient_local(10) | `is_diff=true`：**仅** `world.collision_objects` REMOVE `attached_tool_<id>`（清除 detach 残留）喵~ |

#### 话题 (订阅)

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/tool_changer_status` | ivg_interfaces/msg/ToolChangerStatus | 物理快换状态变化 → 自动触发 AttachedCollisionObject ADD/REMOVE |

## 话题/服务完整索引

### 话题

| 话题 | 类型 | 发布者 | QoS | 说明 |
|------|------|--------|-----|------|
| `/tool_changer_status` | `ToolChangerStatus` | gripper_swap_worker | default | 工具状态: `tool_id` (当前工具ID/空), `tool_name` (中文名), `tool_type` (gripper/other), `is_connected` (是否已连接) |
| `/attached_collision_object` | `AttachedCollisionObject` | scene_attach_worker | transient_local(10) | `attached_tool_<id>` 附着/移除 `kuaihuan_Link`喵~ |
| `/planning_scene` | `PlanningScene` | scene_attach_worker | transient_local(10) | world REMOVE：`attached_tool_<id>`喵~ |

### 服务

| 服务 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/change_tool` | `ChangeTool` | gripper_swap_worker | 按 tool_id 自动快换 (运动 + IO), 返回 success/error_code/message |
| `/run_gripper_swap` | `RunGripperSwap` | gripper_swap_worker | 按方向快换: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| `/get_current_tool` | `GetCurrentTool` | gripper_swap_worker | 查询当前工具: 返回 tool_id/tool_name/tool_type/is_connected |
| `/scene_attach` | `ChangeTool` | scene_attach_worker | 手动附着碰撞网格到 kuaihuan_Link (5 种末端均支持) |
| `/scene_detach` | `ChangeTool` | scene_attach_worker | 手动移除已附着碰撞网格 |

### IO 控制

| IO | 逻辑引脚 | 硬件引脚 | 用途 |
|----|----------|----------|------|
| 快换盘锁紧/释放 | 7 | 39 | gripper_swap_worker 通过 `/aubo_driver/set_io` 控制 |

### 消息定义

`ToolChangerStatus` (ivg_interfaces/msg/ToolChangerStatus):
```
string tool_id       # 工具 ID (gripper0/gripper1/gripper2/gripper1coffeecup/gripper1milkcup)
string tool_name     # 中文名称
string tool_type     # 类型 (gripper/other)
bool is_connected    # 是否已连接到快换盘
```

`ChangeTool` (ivg_interfaces/srv/ChangeTool):
```
# Request
string tool_id

# Response
bool success
int32 error_code
string message
```

`RunGripperSwap` (ivg_interfaces/srv/RunGripperSwap):
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
ros2 service call /get_current_tool ivg_interfaces/srv/GetCurrentTool

# 切换工具
ros2 service call /change_tool ivg_interfaces/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /change_tool ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"

# 按方向快换
ros2 service call /run_gripper_swap ivg_interfaces/srv/RunGripperSwap "{direction: gripper2}"
ros2 service call /run_gripper_swap ivg_interfaces/srv/RunGripperSwap "{direction: gripper0_to_gripper2}"
ros2 service call /run_gripper_swap ivg_interfaces/srv/RunGripperSwap "{direction: gripper2_to_gripper0}"
```

### 场景附着（5 种末端全部支持）

```bash
# 附着到机械臂
ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"
ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper1}"
ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper1coffeecup}"
ros2 service call /scene_attach ivg_interfaces/srv/ChangeTool "{tool_id: gripper1milkcup}"

# 脱离，移除已附着碰撞网格
ros2 service call /scene_detach ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"
ros2 service call /scene_detach ivg_interfaces/srv/ChangeTool "{tool_id: gripper1}"
ros2 service call /scene_detach ivg_interfaces/srv/ChangeTool "{tool_id: gripper2}"
ros2 service call /scene_detach ivg_interfaces/srv/ChangeTool "{tool_id: gripper1coffeecup}"
ros2 service call /scene_detach ivg_interfaces/srv/ChangeTool "{tool_id: gripper1milkcup}"
```

### 监控

```bash
# 工具状态
ros2 topic echo /tool_changer_status

# 场景 / 附着对象（调试）
ros2 topic echo /attached_collision_object
ros2 topic echo /planning_scene
```

## 数据流：工具切换 → MoveIt 附着碰撞

源码：`gripper_swap_worker.cpp::changeToTool()`、`scene_attach_worker.cpp::onToolStatus()`喵~

```
gripper_swap_worker.changeToTool(target)
  │
  ├── （若有当前工具）moveToDockApproach(current) → /scene_detach(current)
  │       └── scene_attach_worker：ACO REMOVE + world REMOVE attached_tool_<current>
  ├── releaseTool(current) → publishToolStatus(false)
  │       └── scene_attach_worker：无附着（is_connected=false）
  ├── moveToDockApproach(target) → pickTool(target)
  ├── publishToolStatus(true)   ← pick 成功后立即发布（不等 moveToHome）
  │       └── scene_attach_worker.onToolStatus()
  │             ├── detachToolFromScene(旧 id)  — /attached_collision_object REMOVE + world REMOVE
  │             └── attachToolToScene(新 id) — /attached_collision_object ADD（附着前后各做一次 world REMOVE 以防陈旧副本）
  └── moveToHome()
```

**核心设计决策（与源码一致）**：

1. **ACO 走 `/attached_collision_object`**：`move_group` 的 `PlanningSceneMonitor` 订阅该话题应用附着几何（启动日志含 *Listening to '/attached_collision_object'*）喵~  
2. **detach 残留走 `/planning_scene`**：仅发布 `world.collision_objects` 的 REMOVE（`attached_tool_<id>`），避免对象落在 world 中与 `kuaihuan_Link` 误判碰撞喵~  
3. **不管理 dock world 静态网格**：不向场景 ADD dock 碰撞体；未连接工具无 PlanningScene 碰撞喵~  
4. **位姿只看 `attach_offset`**：`object.pose = tools.yaml.attach_offset`，`mesh_poses[0]` 为单位姿态，`link_name`/`header.frame_id`=`kuaihuan_Link`喵~  
5. **与 URDF/Web 显示解耦**：本节点不改 `robot_description`；Web `Robot3dViewer` 用 `/tool_changer_status` + TF + `attach_offset` 显示工具 STL喵~  
6. **QoS**：两处 publisher 均为 `QoS(10).transient_local()`，与 `PlanningSceneMonitor` 默认 transient_local 订阅相容喵~  

- **历史依据**：Git `748c7bb3d` 的 `aco.object.pose = attach_offset` + identity `mesh_pose`喵~  
- **末端自碰撞豁免**：`touch_links` 须含 `kuaihuan_Link`、`camera_Link`、`wrist3_Link`、`tool_tcp`（详见 SRDF ACM，`CLAUDE.md` 规则 12b）喵~  

## 末端工具 attach_offset 计算

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

## 历史实现对照

Git 历史 `748c7bb3d` 中的 `scene_attach_worker` 已经证明了末端工具附着的正确位姿语义：`AttachedCollisionObject.object.pose` 表示工具 mesh 原点在 `kuaihuan_Link` 坐标系下的位姿，`mesh_poses[0]` 只保留单位位姿喵~

当前实现保留 `748c7bb3d` 的附着语义，同时：**不再**维护动态 URDF、**不再**向 PlanningScene 添加 dock 静态 world 网格；并通过 `/planning_scene` world REMOVE 清理 detach 后残留的 `attached_tool_<id>`喵~

需要避免的旧路径：

- 不要在 `scene_attach_worker` 中调用 xacro 或缓存 URDF 喵~
- 不要发布 `/robot_description` 喵~
- 不要通过 `AsyncParametersClient` 设置 `robot_state_publisher.robot_description` 喵~
- 不要把 `attach_offset.position` 丢掉，也不要只把 orientation 写到 `mesh_poses[0]` 喵~
