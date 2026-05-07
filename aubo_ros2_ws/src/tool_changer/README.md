# tool_changer — 快换与手爪管理

gripper0 ↔ gripper2 双向自动快换，工具状态发布与查询。

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
| 功能 | PlanningScene 工具附着显示：订阅 `/tool_changer_status` 自动更新场景 |
| 依赖 | tool_changer_interface, moveit_msgs, yaml-cpp |

#### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/scene_attach` | tool_changer_interface/srv/ChangeTool | 手动附着：工具碰撞网格 → kuaihuan_Link |
| `/scene_detach` | tool_changer_interface/srv/ChangeTool | 手动脱离：工具碰撞网格 → world dock |

## 运行

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
ros2 topic echo /tool_changer_status
```

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
