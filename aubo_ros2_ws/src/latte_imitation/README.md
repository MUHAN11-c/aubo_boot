# latte_imitation — 拉花轨迹模仿学习仿真包

从 [ridxm/latte-pour-demos](https://github.com/ridxm/latte-art-robot) 数据集提取双臂拉花轨迹，
将 RM65 右臂（拉花臂）关节轨迹转换为 Aubo E5 可执行的 JointTrajectory，在现有仿真器中回放验证。

## 架构

```
Parquet(HF) → 右臂关节角(6D) → RM65 FK(PyKDL) → 笛卡尔位姿
    → 重定向(scale+offset) → Aubo E5 IK(DLS) → Aubo关节角
    → JointTrajectory msg → /joint_path_command → 仿真器(quintic插值200Hz)
```

## 数据集分析

| 项目 | 内容 |
|---|---|
| 机器人 | 双臂 Realman RM65 (每臂 6 DOF + 1 夹爪 = 7D) |
| 总维度 | 14D：左臂 dims 0-6，右臂 dims 7-13 |
| 拉花臂 | **右臂 (dims 7-12, 6 个关节角)** — 运动方差 ≈30x 左臂 |
| 持杯臂 | 左臂 (dims 0-6) — 基本静止，仅微动 |
| 格式 | LeRobot v3.0 Parquet |
| 数据量 | 40 episodes, 每集 400 帧, 20Hz, ≈20秒 |
| 视觉 | 3 相机 (top, left_wrist, right_wrist) 640×480 |

### 关键发现

1. **左臂基本静止**（total variance 0.0058），仅静态持杯，拉花任务本质上是单臂操作
2. **右臂是拉花臂**（total variance 0.1707），6 个关节大幅协调运动
3. **夹爪恒定**（右臂 dim 13 = 0.991），奶缸被固定握持
4. **轨迹数据可直接提取** `observation.state[:, 7:13]` 得到 (T, 6) 关节角

## 姿态不可达问题与解决方案

### 问题

RM65 的末端姿态（roll=1.29, pitch=-0.75, yaw=-1.33 rad）在通过 FK→重定向→IK 后，
Aubo E5 无法在重定向位置下复现相同的 6D 位姿（position error ~0.41m, IK 不收敛）。

### 原因

- RM65 和 Aubo E5 手腕结构不同（DH 参数不同）
- 相同的笛卡尔位姿需要不同的 joint configuration，在特定位置可能超出 Aubo 关节限位
- 这是异构机器人运动重定向的经典问题

### 解决方案：位置优先（position-only）IK

拉花任务的**核心是末端位置轨迹**（奶缸嘴在液面划出的图案），姿态不需要精确复制：

- 从 RM65 提取末端**位置**轨迹（丢弃姿态信息）
- 用 DLS IK 仅追踪位置（3D error），允许 IK 自由选择可达姿态
- 结果：400/400 帧全部收敛，位置误差 < 0.1mm

**这并非妥协，而是合理的工程简化** —— 拉花的图案由 XY 平面轨迹决定，
倒奶角度可以独立设置，无需从 RM65 复制。

## 模块说明

### robot_model.py — PyKDL 运动学封装

- 从 URDF 手动构建 `PyKDL.Chain`（遍历 revolute joint 路径）
- FK: `ChainFkSolverPos_recursive`
- IK: 自研 DLS（阻尼最小二乘）+ 有限差分雅可比，支持 `pos_only` 模式
- 自适应阻尼：收敛时缩小，发散时增大
- Temporal warm-start：每帧用上一帧的 IK 解作为初始值
- 第一帧随机重启 20 次确保收敛

### dataset_loader.py — 数据集加载

- HF Hub 下载 + 本地缓存（`~/.cache/huggingface/hub/`）
- 也支持 `load_from_local(parquet_path)` 直读本地文件（绕过 HF 下载和网络代理问题）
- 自动提取右臂 6 关节角（dims 7:12）

### retarget.py — 笛卡尔重定向

- 策略：以 RM65 轨迹质心为参考 → 缩放（0.85×）→ 平移到 Aubo 工作空间中心
- `auto_center=True` 从数据自动计算质心
- 视觉姿态保持不变（Identity rotation offset）

### trajectory_publisher.py — ROS2 主节点

- **发布**: `/joint_path_command` (JointTrajectory) → 现有仿真器
- **调试**: `~/rm65_pose`, `~/aubo_pose` (PoseStamped)
- **参数**:
  - `episode_idx`: 回放的 episode 编号（默认 0）
  - `speed_scale`: 播放速度倍率（默认 1.0 = 20Hz）
  - `local_parquet`: 本地 parquet 路径（跳过 HF 下载）
  - `pos_only`: 仅位置 IK（默认 true，RM65 姿态在 Aubo 上不可达）
  - `position_scale`: 重定向缩放因子（默认 [0.85, 0.85, 0.85]）
  - `aubo_center`: Aubo 工作空间中心（默认 [0.3, 0.0, 0.6]）

## 使用方法

### 1. 准备数据

```bash
# 从 HF 下载（需要网络）
# 自动缓存到 ~/.cache/huggingface/hub/

# 或手动下载单集用于测试
wget https://huggingface.co/datasets/ridxm/latte-pour-demos/resolve/main/data/chunk-000/episode_000000.parquet
```

### 2. 构建

```bash
cd aubo_ros2_ws
colcon build --packages-select latte_imitation
source install/setup.bash
```

### 3. 启动仿真回放

```bash
# 从本地文件（推荐，无需网络）
ros2 launch latte_imitation replay_trajectory.launch.py \
    local_parquet:=./episode_000000.parquet

# 从 HF 下载
ros2 launch latte_imitation replay_trajectory.launch.py \
    episode_idx:=0

# 调参
ros2 launch latte_imitation replay_trajectory.launch.py \
    local_parquet:=./episode_000000.parquet \
    speed_scale:=0.5
```

### 4. 监控输出

```bash
# 查看仿真器插值输出
ros2 topic echo /moveItController_cmd

# 查看 RM65 / Aubo 调试位姿
ros2 topic echo /latte_imitation/rm65_pose
ros2 topic echo /latte_imitation/aubo_pose
```

## 验证结果

```
Episode 0: 400 frames, dt=0.050s
RM65 FK: (-0.340, 0.028, 0.521)
Retargeting: rm65_center=[-0.242, -0.042, 0.253] -> aubo_center=[0.3, 0.0, 0.6]
IK: 400/400 frames converged, max pos error < 0.1mm
Simulator: "Received trajectory with 400 points"
```

## 依赖

- PyKDL (python3-pykdl)
- urdf_parser_py
- huggingface_hub, pyarrow, pandas, numpy
- ROS2 Humble: rclpy, trajectory_msgs, geometry_msgs, ament_index_python

## 局限与改进方向

1. **姿态丢抛**: 当前 position-only IK 丢弃了 RM65 的姿态信息。
   改进方案：添加姿态重定向（axis-angle 分解 + wrist alignment）
2. **手动重定向参数**: `position_scale` 和 `aubo_center` 需要手动调整。
   改进方案：自动标定（通过 TCP 标定 → 工作空间映射）
3. **仿真器仅验证运动学**: 当前仿真只验证轨迹的 joint 空间可行性。
   改进方案：接入 MoveIt + Rviz 可视化末端轨迹
4. **无视觉输入**: 模仿学习最终需要视觉策略。
   改进方案：配合 vision_perception 包做视觉伺服
