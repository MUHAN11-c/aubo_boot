# latte_imitation — 拉花轨迹模仿学习包 (MoveIt2 标准管线)

从 [ridxm/latte-pour-demos](https://huggingface.co/datasets/ridxm/latte-pour-demos) 数据集提取双臂拉花末端笛卡尔轨迹，
通过 **MoveIt2 标准管线** (`computeCartesianPath` → `execute`) 规划并执行拉花动作。

## 架构

```
数据集 (latte-pour-demos)
  │
  ├─ resource/original/*.parquet   RM65 14D 关节角 (原始，保留)
  │       │ [一次性: RM65 FK]
  │       ▼
  └─ resource/cartesian/{left,right}/*.npz   笛卡尔末端轨迹 (直接加载)
                         │
                         ▼
                CartesianTrajectory.load()
                         │
              ┌──────────┴──────────┐
              ▼                     ▼
     visualize (matplotlib)    ROS2 Service
     - 40条叠加动画              │
     - 逐帧播放控制         ReplayLatteTrajectory
                               │
                          ┌────┴────┐
                     debug mode   action mode
                     PoseStamped   │
                     + Path        ▼
                              5-Phase Pipeline (MoveIt2):
                              ① 加载 npz
                              ② start_pose 6-DOF 刚性变换 (自动获取当前 EE 位姿)
                              ③ 发布 debug 位姿/路径
                              ④ MoveIt2 computeCartesianPath (全6-DOF IK + 内置碰撞)
                              ⑤ MoveIt2 /execute_trajectory action
```

## 模块

| 文件 | 职责 |
|------|------|
| `trajectory.py` | `CartesianTrajectory` 数据类：加载/保存 npz，统计，导出 ROS2 消息喵~ |
| `trajectory_transform.py` | 6-DOF 刚性变换：RM65 base frame → AUBO 当前 EE 位姿 (含 orientation 旋转修复) 喵~ |
| `trajectory_pipeline.py` | `LatteImitationNode`：服务回调 + 5 阶段 MoveIt2 管线编排喵~ |
| `tf_utils.py` | 共享 TF 查询工具：`get_current_ee_pose()` (ROS 节点模式) / `get_ee_pose_from_tf()` (脚本模式) / `TfQueryNode` (GUI 持久模式) 喵~ |
| `scripts/visualize_latte_trajectory.py` | 离线播放器：40 条轨迹叠加 + 动画回放喵~ |
| `scripts/test_replay_service.py` | 交互式测试脚本：菜单选择, 无需手敲 ros2 service call 喵~ |
| `scripts/latte_debug_panel.py` | PySide6 可视化调试面板：3D 预览 + 欧拉角控制 + 一键执行喵~ |

> 已删除: `robot_model.py`, `collision_checker.py`, `action_executor.py`, `trajectory_publisher.py` — 均被 MoveIt2 标准管线替代喵~

## 数据文件

```
resource/
├── original/               ← 40 个原始 14D parquet（RM65 关节角，保留）
├── cartesian/
│   ├── left/               ← 40 个左臂末端笛卡尔 npz (400×3 positions + 400×4 orientations)
│   └── right/              ← 40 个右臂末端笛卡尔 npz
└── trajectory_overview.png ← 40 条右臂轨迹叠加图
```

每个 npz 包含：
- `positions` (400, 3) — 末端 XYZ 位置 (m)
- `orientations` (400, 4) — 四元数 xyzw
- `timestamps` (400,) — 从 0 开始，dt=0.05s
- `dt` — 时间步长
- `episode_idx` — episode 编号

## 使用方法

### 构建

```bash
# 必须在 workspace 根目录运行 colcon
cd aubo_ros2_ws
colcon build --packages-select ivg_interfaces latte_imitation
source install/setup.bash
```

### 2. 一键启动 (推荐)

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
./start_latte_test.sh                  # 自动: 构建→仿真→服务→交互菜单
./start_latte_test.sh --skip-build     # 跳过 colcon build
./start_latte_test.sh --real           # 真机模式 (需 AUBO IP 可达)
```

脚本流程: `colcon build` → 启动仿真 (轮询等待 controller) → 启动 latte_imitation (轮询等待服务) → 交互式测试菜单喵~
退出时自动清理所有进程 (Ctrl+C 或菜单选 0) 喵~

### 3. 手动分终端启动

终端1 — 仿真:
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
```

终端2 — latte_imitation:
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run latte_imitation latte_imitation_node --ros-args -p mode:=debug
```

### 4. 典型使用场景

#### 场景 A: 快速预览轨迹 (Debug 模式)

不执行机械臂运动，只发布 PoseStamped/Path，可在 RViz2 中查看喵~

```bash
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 3, arm: 'right', speed_scale: 1.0, mode: 'debug', \
    start_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

#### 场景 B: Action 模式 (MoveIt2 CartesianPath→Execute)

自动从 TF 获取当前末端位姿作为轨迹起点，MoveIt2 全 6-DOF IK + 内置碰撞检测喵~

```bash
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: 'right', speed_scale: 2.0, mode: 'action', \
    start_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

响应示例：
```
success: True
message: "轨迹执行成功完成"
num_frames: 400
path_length: 1.53
ik_success_count: 400         # fraction * num_frames (规划成功等效帧数)
collision_count: 0            # MoveIt2 内置处理，始终为 0
```

#### 场景 C: start_pose 覆盖 (手动指定杯子位姿)

相机检测到杯子位姿 → 轨迹做 6-DOF 刚性变换对齐到杯子。不传 start_pose 时自动从 TF 获取当前 EE 位姿喵~

```bash
# 杯子在 (0.45, 0, 0.50), 杯口 Z轴朝上 (identity)
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: 'right', speed_scale: 1.0, mode: 'action', \
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 杯子绕 Z轴 90° 倾斜 (orientation: z=0.707, w=0.707)
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: 'right', speed_scale: 1.0, mode: 'action', \
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}"
```

#### 场景 D: 批量测试全部 40 条 episode

```bash
for ep in $(seq 0 39); do
  echo "=== Episode $ep ==="
  ros2 service call /latte_imitation/replay_trajectory \
    ivg_interfaces/srv/ReplayLatteTrajectory \
    "{episode_idx: $ep, arm: 'right', speed_scale: 1.0, mode: 'action', \
      start_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
  sleep 22
done
```

#### 场景 E: Launch 文件一键启动

```bash
# Debug 模式
ros2 launch latte_imitation replay_trajectory.launch.py episode_idx:=5 arm:=left

# Action 模式
ros2 launch latte_imitation replay_trajectory.launch.py \
    mode:=action episode_idx:=0 arm:=right speed_scale:=0.5
```

#### 场景 F: 离线可视化 (不依赖 ROS2)

```bash
cd src/latte_imitation
python3 scripts/visualize_latte_trajectory.py                 # 右臂 (默认)
python3 scripts/visualize_latte_trajectory.py --arm left      # 左臂
python3 scripts/visualize_latte_trajectory.py --speed 2.0     # 2 倍速

# 手动指定起点 (刚性变换到目标位姿)
python3 scripts/visualize_latte_trajectory.py \
    --start-x 0.35 --start-y -0.10 --start-z 0.52

# 自动从 ROS 2 TF 获取当前末端位姿作为起点 (需机械臂运行中)
python3 scripts/visualize_latte_trajectory.py --from-robot

# 从 SRDF camera_pose 状态计算 FK 末端位姿作为起点
python3 scripts/visualize_latte_trajectory.py --from-camera-pose
```
键盘：`[ ]` 切 episode | 空格 播放/暂停 | `← →` 逐帧 | `↑ ↓` 变速 | `a` 叠加 | `r` 重置

#### 场景 G: 交互式测试脚本 (推荐)

```bash
python3 src/latte_imitation/scripts/test_replay_service.py
```

数字菜单选择测试, 无需手敲长命令喵~

```
==============================================================
  latte_imitation 测试菜单
==============================================================
  [1] Debug — 预览轨迹
  [2] Action + 碰撞检测
  [3] start_pose 纯平移
  [4] start_pose 平移+旋转
  [5] 错误处理 — 不存在 episode
  [6] 左臂持杯轨迹
  [7] 自定义 — 手动输入参数
  [0] 退出
--------------------------------------------------------------
```

### 5. 完整测试命令 (手动)

> 也可用场景 G 的交互式脚本自动生成喵~

```bash
# 前提: 终端1启动仿真, 终端2启动 latte_imitation, 终端3 source 环境后执行

# ── Test 1: Debug 模式 — 预览轨迹, 立即返回 ──
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: right, speed_scale: 1.0, mode: debug, \
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
# 预期: success=true, num_frames=400, path_length≈1.53

# ── Test 2: Action — MoveIt2 CartesianPath→Execute (~10秒@2倍速) ──
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: right, speed_scale: 2.0, mode: action, \
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
# 预期: success=true, ik_success_count~400 (fraction*400), collision_count=0

# ── Test 3: start_pose 覆盖 — 杯子在 (0.45,0,0.50), 杯口Z轴朝上 ──
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: right, speed_scale: 1.0, mode: action, \
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
# 预期: 轨迹起点=(0.45,0,0.50), 路径长度不变(刚性保距)

# ── Test 4: start_pose 覆盖 — 杯子绕Z轴90°倾斜 ──
# orientation: z=sin(45°)=0.707, w=cos(45°)=0.707
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: right, speed_scale: 1.0, mode: action, \
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}"
# 预期: 路径长度不变(刚性保距), orientation 被正确旋转

# ── Test 5: 错误处理 — 不存在的 episode ──
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 999, arm: right, speed_scale: 1.0, mode: debug, \
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
# 预期: success=false, message="episode_000999.npz ... 未找到"

# ── Test 6: 左臂轨迹 — 持杯臂, 路径~0.31m ──
ros2 service call /latte_imitation/replay_trajectory \
  ivg_interfaces/srv/ReplayLatteTrajectory \
  "{episode_idx: 0, arm: left, speed_scale: 1.0, mode: debug, \
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
# 预期: success=true, num_frames=400, path_length≈0.31 (右臂1/5)
```

### 6. 服务参数参考

#### 请求字段

| 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `episode_idx` | int32 | 0 | Episode 编号 (0-39) 喵~ |
| `arm` | string | right | "left"=左臂持杯 / "right"=右臂拉花 喵~ |
| `speed_scale` | float32 | 1.0 | 播放速度倍率 [0.01, 10.0] 喵~ |
| `mode` | string | debug | "debug"=仅发布位姿 / "action"=MoveIt2 CartesianPath→Execute 喵~ |
| `start_pose` | Pose | 零位姿 | 零位姿=自动从 TF 获取当前 EE 位姿 / 非零=手动指定起点 喵~ |
| `pos_only` | bool | — | **[废弃]** 保留兼容，不再生效 (MoveIt2 始终全 6-DOF IK) 喵~ |
| `collision_check` | bool | — | **[废弃]** 保留兼容，不再生效 (MoveIt2 内置 avoid_collisions) 喵~ |

#### 响应字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 调用是否成功 喵~ |
| `message` | string | 状态消息或错误描述 喵~ |
| `num_frames` | int32 | 轨迹总帧数 喵~ |
| `path_length` | float32 | 笛卡尔路径总长度 (m) 喵~ |
| `ik_success_count` | int32 | `int(fraction * num_frames)` — 规划成功等效帧数 (debug 模式为 0) 喵~ |
| `collision_count` | int32 | 始终为 0 (MoveIt2 内部处理碰撞) 喵~ |
| `collision_details` | string[] | 始终为空 (MoveIt2 内部处理碰撞) 喵~ |

### 7. 碰撞检测

MoveIt2 `computeCartesianPath` 使用 `avoid_collisions=True` 内置碰撞检测，无需手动调用 `/check_state_validity` 喵~

**SRDF ACM** (Allowed Collision Matrix) 在 MoveIt2 PlanningScene 中自动生效，豁免相邻 link 对 (如 gripper→wrist3) 喵~

### Python API

```python
from latte_imitation import CartesianTrajectory, apply_start_pose
from latte_imitation import get_current_ee_pose, get_ee_pose_from_tf, TfQueryNode
from latte_imitation import quat_to_rot, rot_to_quat, quat_multiply, euler_deg_to_quat

# 加载全部 40 条
carts = CartesianTrajectory.load_all("resource", "right")
for ep, cart in carts.items():
    print(f"Ep{ep}: {cart.path_length():.2f}m, {cart.num_frames} frames")

# 加载单条
cart = CartesianTrajectory.load("resource/cartesian/right/episode_000000.npz")
print(cart.start)   # [-0.34, 0.028, 0.521]
print(cart.end)     # [-0.34, 0.010, 0.585]

# 导出 ROS2 消息
path_msg = cart.to_ros2_path()
pose_msg = cart.to_pose_stamped(10)

# TF 查询 — 三种模式
pose_tf = get_current_ee_pose(node)          # ROS 节点内使用
pose_script = get_ee_pose_from_tf()          # 独立脚本使用
tf_gui = TfQueryNode()                       # GUI 持久查询
pose = tf_gui.get_pose(timeout=1.5)
tf_gui.shutdown()
```

## 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `episode_idx` | 0 | episode 编号 (0-39) |
| `arm` | right | left / right |
| `speed_scale` | 1.0 | 播放速度倍率 |
| `mode` | debug | debug=PoseStamped+Path, action=MoveIt2 CartesianPath→Execute |
| `planning_group` | manipulator | MoveIt2 规划组名 |
| `base_frame` | base_link | TF 基准坐标系 |
| `ee_link` | tool_tcp | 末端执行器 link |
| `cartesian_max_step` | 0.01 | 笛卡尔路径插值步长 (m) |
| `cartesian_jump_threshold` | 0.0 | 跳变检测阈值 (0=禁用) |
| `fraction_acceptable` | 0.95 | 直接通过的 fraction 阈值 |
| `fraction_min_executable` | 0.50 | 最小可执行的 fraction 阈值 |
| `waypoint_sample_step` | 4 | waypoint 采样间隔 (帧) |
| `service_timeout` | 15.0 | 服务发现超时 (秒) |
| `cartesian_timeout` | 60.0 | 笛卡尔规划超时 (秒) |
| `execution_timeout` | 120.0 | 轨迹执行超时 (秒) |
| `tf_retry_count` | 20 | TF 查询重试次数 |
| `tf_retry_interval` | 0.03 | TF 重试间隔 (秒) |

## CartesianTrajectory API

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `load(path)` | `CartesianTrajectory` | 从 npz 加载单条 |
| `load_all(dir, arm)` | `OrderedDict[int, CartesianTrajectory]` | 加载目录下全部 npz |
| `save(path)` | — | 保存为 npz |
| `path_length()` | float | 累计路径长 (m) |
| `velocity_profile()` | ndarray (T-1,) | 帧间瞬时速度 (m/s) |
| `segment(i, j)` | `CartesianTrajectory` | 切片子轨迹 |
| `to_pose(idx)` | `Pose` | 单帧 ROS2 Pose |
| `to_pose_stamped(idx)` | `PoseStamped` | 单帧 PoseStamped |
| `to_ros2_path(step=5)` | `Path` | 完整 ROS2 Path |

属性: `positions` (T,3), `orientations` (T,4), `timestamps` (T,), `dt`, `start`, `end`, `num_frames`

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/latte_imitation/ee_pose` | PoseStamped | 轨迹 waypoints（每 5 帧采样，debug+action 模式均发布）|
| `/latte_imitation/ee_path` | Path | 完整轨迹路径（debug+action 模式均发布）|

## 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/latte_imitation/replay_trajectory` | `ivg_interfaces/srv/ReplayLatteTrajectory` | 按需触发拉花轨迹回放喵~ |

## 数据集信息

- 来源：[ridxm/latte-pour-demos](https://huggingface.co/datasets/ridxm/latte-pour-demos)
- 40 个 episode，每个 400 帧 @ 20fps（20 秒）
- 14D 关节角：左臂 dims 0-6 + 右臂 dims 7-13 + 夹爪 dim 13
- 右臂（拉花臂）末端路径长约 1.53m，分四阶段：进杯 → 调整 → 核心拉花 → 退杯

### 14 维关节分解

| 维度 | 所属 | 运动特征 |
|------|------|---------|
| dim 0-5 | 左臂 6 关节 | 几乎静止（持杯） |
| dim 6 | 左臂夹爪 | 恒定 0.948 |
| dim 7-12 | **右臂 6 关节** | **拉花运动** |
| dim 13 | 右臂夹爪 | 恒定 0.991 |

### 笛卡尔末端轨迹（右臂，RM65 FK）

| 轴 | 范围 (m) | 跨度 (mm) |
|---|---|---|
| X | [-0.365, -0.153] | 212 |
| Y | [-0.094, 0.028] | 121 |
| Z | [0.173, 0.585] | 413 |

- 直线位移 67mm，总路径 **1.53m**
- 40 条轨迹帧间 std 均值仅 0.044m，高度一致

## 数据流详解

### 数据生成（一次性）

原始 parquet 是 RM65 关节角，不可直接用于 Aubo E5。通过 RM65 FK 预计算生成笛卡尔 npz：

```
resource/original/ (40 个 parquet, 14D 关节角)
    │
    │  RM65 FK (robot_model.py)
    ▼
resource/cartesian/left/  (40 个 npz, 3D 位置 + 4D 姿态)
resource/cartesian/right/ (40 个 npz)
```

之后所有模块只加载笛卡尔 npz，不再依赖 parquet 和 RobotModel（RM65）。

### 节点执行流程

> 权威全流程文档以 `latte_imitation/trajectory_pipeline.py` 文件头 docstring 为准，以下为概要喵~

**入口** (3 种触发方式):
| 入口 | 触发方 | 说明 |
|------|--------|------|
| ROS 2 Service | 外部节点 / Web Dashboard | `~/replay_trajectory` (ivg_interfaces/srv/ReplayLatteTrajectory) |
| Launch 参数 | 节点启动时自动 | `_delayed_start()` 2 秒 timer 执行默认 episode (向后兼容) |
| 交互菜单 | 开发者 | `test_replay_service.py` 封装 ros2 service call |

**编排层 — `_execute_pipeline()`** (trajectory_pipeline.py:214):
```
并发锁 self._executing=True → _pipeline() → finally: self._executing=False
```
同一时刻仅允许一条管线运行，重复请求直接返回 "已有轨迹正在执行" 喵~

**5 阶段管线 — `_pipeline()`** (trajectory_pipeline.py:228):
```
Phase ①: _load_cartesian()           加载 resource/cartesian/{arm}/episode_{idx}.npz
    │                                  失败 → (success=false, "未找到")
    ▼
Phase ②: apply_start_pose()          6-DOF 刚性变换 (RM65→AUBO 当前 EE 位姿)
    │                                   - is_default_pose() → TF 自动获取当前 EE
    │                                     (20 次重试 × 30ms, 失败 → "无法获取末端位姿")
    │                                   - 否则: 手动指定 start_pose (相机检测杯子位姿)
    │                                   - R_rel = R_tgt @ R_orig^T
    │                                   - p_new = R_rel @ (p - p0) + p_target
    │                                   - q_new = q_rel * q_orig (orientation 也旋转)
    ▼
Phase ③: _publish_poses()            发布轨迹 waypoints + Path (debug 可视化)
    │
    ├─ mode="debug" → return (success=true, 跳过规划/执行)
    │
    ▼
Phase ④: _compute_cartesian_path()   MoveIt2 /compute_cartesian_path
    │                                   - start_state=RobotState() (空=当前状态)
    │                                   - max_step=0.01, jump_threshold=0.0
    │                                   - avoid_collisions=True (内置碰撞检测)
    │                                   - fraction≥0.95 → 直接进入 Phase ⑤
    │                                   - 0.50≤fraction<0.95 → retry with avoid_collisions=False
    │                                     (取两者中 fraction 较大者)
    │                                   - fraction<0.50 → fail
    │                                   - 成功 → 按 speed_scale 缩放 timestamps
    ▼
Phase ⑤: _execute_trajectory()       MoveIt2 /execute_trajectory action
                                        - send_goal_async → wait_for_accept → wait_for_result
                                        - 超时: send_goal 5s, 执行 120s
                                        - 成功判定: error_code.val == 1
```

### 设计决策

| 决策 | 原因 |
|------|------|
| MoveIt2 标准管线 | 统一使用 `/compute_cartesian_path` + `/execute_trajectory`，与其他节点一致喵~ |
| 不再使用自定义 IK | MoveIt2 KDL IK 全 6-DOF 匹配，废弃 pos_only 需求喵~ |
| TF 自动获取起点 | `lookup_transform(base_link, tool_tcp)` — 更 ROS-idiomatic 喵~ |
| orientation 旋转修复 | `apply_start_pose()` 现在正确用 `rot_to_quat()` + `quat_multiply()` 旋转 orientation 喵~ |
| 碰撞由 MoveIt2 内置 | `avoid_collisions=True` — 不再需要手动 `/check_state_validity`喵~ |
| 只存笛卡尔 npz | 关节角跨机械臂不可复用，笛卡尔位姿是通用的喵~ |
| 保留原始 parquet | 可追溯数据来源，需要时可重新 FK 计算喵~ |
| `load_all` 返回 OrderedDict | 按 episode 编号排序，遍历顺序可预测喵~ |
| ReentrantCallbackGroup + MultiThreadedExecutor | 服务回调中同步等待 Action，需避免 MutuallyExclusive 组死锁喵~ |
| start_pose 6-DOF 刚性变换 | 相机检测杯子的完整位姿 (位置+朝向)，纯平移不足以对齐杯子朝向喵~ |

## YOLO26 训练与推理

### 环境

| 项目 | 详情 |
|------|------|
| Ultralytics 版本 | **8.4.49**（pip 安装，`pip3 show ultralytics`） |
| 源码参考 | `yolov26_src/` — 只作参考，实际训练推理走 pip 安装的包 |
| 模型权重 | `yolo26n.pt` (5.3MB), `yolo26m.pt` (44.3MB), `yolo26x.pt` (113MB) |
| 数据集 | `datasets/coco128/` (128 张, 7.5MB, 仅功能验证) |
| GPU | RTX 3090 24GB, CUDA 13.2, PyTorch 2.12.0 |

> `yolov26_src/` 是 ultralytics 官方源码副本，**未 `pip install -e .` 到环境中**。`yolov6/` 同。训练推理脚本走系统安装的 `ultralytics==8.4.49`。
>
> 确认当前版本：`python3 -c "import ultralytics; print(ultralytics.__version__)"` → 8.4.49

### YOLO26 模型规格

| 模型 | 参数量 | GFLOPs | 权重大小 | RTX 3090 推理 |
|------|--------|--------|---------|-------------|
| yolo26n | 2.4M | 5.4 | 5.3MB | 1.0ms |
| yolo26m | 20.4M | 68.2 | 44.3MB | 2.9ms |
| yolo26x | 59.0M | 209.5 | 113.2MB | 不可训练 (OOM) |

核心特性：NMS-Free 端到端推理、MuSGD 优化器、移除 DFL、CPU 推理速度提升 43%。

### 训练

```bash
# 基础训练（脚本自动使用包目录下的 .pt 权重）
python3 scripts/train_yolo26.py

# 完整参数示例
python3 scripts/train_yolo26.py \
    --model m \
    --data datasets/coco128.yaml \
    --epochs 300 \
    --batch 64 \
    --device 0 \
    --half \
    --project runs/yolo26 \
    --name my_experiment
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | n | 模型规模 n/s/m/l/x |
| `--data` | datasets/coco128.yaml | 数据集 yaml |
| `--epochs` | 300 | 训练轮数 |
| `--batch` | -1 (自动) | batch size |
| `--imgsz` | 640 | 输入尺寸 |
| `--device` | 0 | GPU 编号 / cpu |
| `--half` | False | FP16 半精度 |
| `--cache` | False | 数据集缓存到 RAM |
| `--resume` | False | 从 checkpoint 恢复 |

### 推理

```bash
# 单图推理
python3 scripts/infer_yolo26.py --source image.jpg

# 目录批量
python3 scripts/infer_yolo26.py --source images/ --save-txt

# 摄像头实时
python3 scripts/infer_yolo26.py --source 0 --show

# 指定模型和阈值
python3 scripts/infer_yolo26.py --model m --source img.jpg --conf 0.5 --save-crop
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--source` | (必填) | 图片/目录/摄像头编号 |
| `--model` | n | 模型规模 n/s/m/l/x |
| `--weight` | — | 自定义权重路径（覆盖 `--model`） |
| `--conf` | 0.25 | 置信度阈值 |
| `--iou` | 0.7 | NMS IOU 阈值 |
| `--show` | False | 实时显示结果 |
| `--save-txt` | False | 保存 YOLO 格式 txt |
| `--save-crop` | False | 裁剪保存检测目标 |

### GPU 训练基准（300 epochs × COCO128）

#### yolo26n（轻量基线）

```
yolo train model=yolo26n.pt data=coco128.yaml epochs=300 device=0
```

| 指标 | 数值 |
|------|------|
| Batch size | 16 (默认) |
| **GPU 显存** | **4.26 GB (18%)** |
| GPU 利用率 | ~40% |
| 训练速度 | 12-13 it/s |
| 总训练时间 | **0.094 小时 (5.6 分钟)** |
| 最佳 mAP50 | 0.901 |

#### yolo26m（压榨 GPU）

```
yolo train model=yolo26m.pt data=coco128.yaml epochs=300 batch=64 device=0 workers=4
```

| 指标 | 数值 |
|------|------|
| Batch size | **64** |
| **GPU 显存** | **17.9 GB (73%)** |
| GPU 利用率 | **100%** |
| GPU 功耗 | **337W / 350W** |
| 总训练时间 | **0.249 小时 (15 分钟)** |
| 最佳 mAP50 | 0.977 |

### 性能瓶颈分析

#### 1. 数据集过小

COCO128 仅 128 张图，几分钟跑完，mAP 虚高无参考价值。真实训练需用完整 COCO（118K 张）或自定义工业数据集。

#### 2. yolo26x 显存溢出

yolo26x (59M 参数, 209 GFLOPs) 在当前环境中无法训练：

| 尝试配置 | 结果 | 原因 |
|----------|------|------|
| batch=48, imgsz=640 | CUDA OOM | 模型 + AMP 超出 24GB |
| batch=24 (auto-reduce) | 仍然 OOM | 同上 |

RTX 3090 24GB 训练 yolo26x 的方案：`--half` (FP16)、`--cache` (缓存到 RAM)、减小 `--imgsz 480`、或多卡/云 GPU。

#### 3. 僵尸进程显存泄漏

OOM 崩溃后 PyTorch DataLoader worker 进程可能残留 CUDA context 占用显存：

```bash
fuser -v /dev/nvidia* | grep pt_data_worker | awk '{print $2}' | xargs kill -9
```

#### 4. 模型 vs 显存（实测）

| 模型 | batch=16 显存 | 最大可用 batch | 推荐 batch |
|------|-------------|---------------|-----------|
| yolo26n | ~4.3 GB | ~128 | 16-64 |
| yolo26m | ~13 GB | ~64 | 32-64 |
| yolo26x | — | <16 (不可用) | GPU 不足 |

### 依赖

- numpy, tf2_ros
- ROS2 Humble: rclpy, trajectory_msgs, geometry_msgs, nav_msgs, moveit_msgs, ament_index_python
- 可视化: matplotlib
- YOLO26: ultralytics>=8.4.0, torch>=1.8.0
