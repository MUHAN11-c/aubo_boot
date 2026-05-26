# 单臂 AUBO E5 咖啡拉花轨迹执行 — 完整技术方案

> 版本: v2.0 | 日期: 2026-05-26
> 参考: latte-art-robot (GitHub/ridxm), latte_art_robot_research, latte_imitation (当前实现)
> 真机位姿参考: ivg_monitor_2026-05-26.json, URDF aubo_e5_base.urdf

---

## 0. 物理布局与工作流程

### 0.1 参考位置 (URDF 固定 Link)

所有位置在 base_link 坐标系中，pedestal_Link 在 base_link 的偏移为 (-0.105, -0.105, -0.028):

```
                 Y ↑
                   │
          coffee_Link (咖啡机出咖啡)
          (-0.645, 0.098)             ← ① 取咖啡杯起点
                   │
                   │    ╔═══ 机械臂 ═══╗
                   │    ║   (0,0,0)   ║
                   │    ╚═════════════╝
                   │
     cup0_Link ────┼──── lizhu_Link
     (-0.528,-0.198)    (-0.630,-0.368)
     ② 取牛奶杯位置      ③ 放咖啡杯位置
                   │
                   └────────────── X →

| Link | joint 偏移 (in pedestal) | base_link 位置 | 用途 |
|------|--------------------------|---------------|------|
| coffee_Link | (-0.540, +0.203, +0.046) | (-0.645, +0.098, +0.018) | 咖啡机出杯, 取咖啡杯 |
| lizhu_Link | (-0.525, -0.263, +0.006) | (-0.630, -0.368, -0.022) | 放置咖啡杯, 拉花目标 |
| cup0_Link | (-0.423, -0.093, -0.012) | (-0.528, -0.198, -0.040) | 牛奶杯/拉花壶位置 |
| JSON 参考 | — | (-0.419, -0.400, +0.246) | 杯口朝上姿态, RPY(-23.5°,88.1°,76.0°) |

### 0.2 完整工作流程

```
① 取咖啡杯 (coffee_Link)
   机械臂移动到咖啡机出杯位置
   末端: coffee_Link XYZ + Z微调
   夹取含浓缩咖啡的咖啡杯

② 放咖啡杯 (lizhu_Link)
   机械臂移动到立柱上方
   末端: lizhu_Link XYZ + Z微调
   放置咖啡杯, 松开夹爪

③ 取牛奶杯 (cup0_Link)
   机械臂移动到牛奶杯位置
   末端: cup0_Link XYZ + Z微调
   夹取含打发奶泡的拉花壶

④ 姿态调整 (参考 JSON 位姿)
   移动到安全中转位姿
   末端: (-0.419, -0.400, 0.246)
   RPY: (-23.5°, 88.1°, 76.0°) → 杯口Z轴朝上, 奶泡不洒

⑤ 移动到咖啡杯上方 → 拉花执行
   latte_imitation 管线接管
   从参考位姿移动到 lizhu_Link 上方
   pitch 88°→45° (竖直→前倾倒奶)
   执行拉花轨迹
```

### 0.3 参数分层架构

```
┌─ ROS2 参数层 (latte_imitation 节点参数) ─────────────────────┐
│ declare_parameter, YAML 加载, 一次性设置                       │
│                                                              │
│  coffee_link_x/y/z/roll/pitch/yaw    ← 取咖啡杯位置           │
│  lizhu_link_x/y/z/roll/pitch/yaw     ← 放咖啡杯位置           │
│  cup0_link_x/y/z/roll/pitch/yaw      ← 取牛奶杯位置           │
│  reference_pose_x/y/z/roll/pitch/yaw ← 杯口朝上参考位姿        │
│                                                              │
│  前端入口: 设置面板 (话题设置弹窗 或 独立区域)                   │
└──────────────────────────────────────────────────────────────┘

┌─ 前端操作参数层 (latte_controls.js) ─────────────────────────┐
│ localStorage 持久化, 每次可调                                 │
│                                                              │
│  pattern_type, episode_idx   ← 图案选择                       │
│  cup_surface_z, cup_radius   ← 杯子参数 (XY从 lizhu_Link 继承) │
│  mix/draw/finish_h           ← 倾倒工艺高度                    │
│  wiggle_amp/freq             ← 摆动参数                       │
│  max_vel/acc/jerk, anti_slosh ← 速度约束                      │
│  roll/pitch/yaw              ← 进杯角度微调                    │
│  dx/dy/dz                    ← 平移微调                       │
│  speed_scale, waypoint_step  ← 执行参数                       │
│                                                              │
│  前端入口: 咖啡拉花面板 → 拉花参数控制                          │
└──────────────────────────────────────────────────────────────┘
```

---

## 1. 需求概述

### 1.1 目标

单臂 AUBO E5 机械臂完成咖啡拉花，杯子固定不动，单臂控制奶缸完成位置移动 + 倾倒倾斜 + 图案绘制。

### 1.2 核心约束

```
单臂 vs 双臂的本质差异:

  latte-art-robot (RM65 双臂):     本方案 (AUBO E5 单臂):
  ┌────────┐  ┌────────┐          ┌──────────────────────┐
  │ 臂1: 奶缸│  │ 臂2: 杯子│          │ 单臂: 奶缸             │
  │ 位置+倾倒│  │ 倾斜30-45°│         │ 位置+倾倒+图案        │
  │ 相对稳定 │  │ 持续旋转  │         │ 杯子固定不动           │
  └────────┘  └────────┘          └──────────────────────┘

单臂约束:
  ✅ 位置移动: XYZ 三轴自由
  ✅ 倾倒角度: pitch 绕 Y 轴 (工具前后倾斜)
  ❌ 侧倾: roll 始终为 0 (液体不允许侧漏)
  ⚠️ 进杯角度: yaw 绕 Z 轴 (改变进杯方向, 但倾倒始终向下)
```

### 1.3 功能清单

| # | 功能 | 说明 | 优先级 |
|---|------|------|--------|
| F1 | 图案选择 | heart / rosetta / tulip / swan / 录制回放 | P0 |
| F2 | 杯子配置 | 杯口中心 XYZ + 半径 (在 base_link 中手动设置) | P0 |
| F3 | 倾倒工艺参数 | 三阶段高度偏移 + 摆动振幅/频率 + 郁金香层数 | P0 |
| F4 | 速度约束 | 最大速度/加速度/jerk + 抗晃荡开关 (S曲线剖面) | P0 |
| F5 | 进杯角度 | RPY 滑块 (yaw=进杯方向, roll/pitch=图案位置微调) | P0 |
| F6 | 平移微调 | ΔX/ΔY/ΔZ 手动修正轨迹起点 | P1 |
| F7 | 3D 实时预览 | 参数变更后自动刷新前端 3D 轨迹渲染 | P0 |
| F8 | 轨迹执行 | mode="action" → MoveIt2 笛卡尔规划+执行 | P0 |
| F9 | 停止/回原点 | 紧急停止 + 机械臂回安全位姿 | P1 |
| F10 | 执行状态 | 进度心跳 + 成功/失败消息 + 全链路日志 | P0 |
| F11 | DO 控制 | 咖啡开关(DO4) + 打花开关(DO2) | P1 |
| F12 | DI 反馈 | 咖啡反馈(DI2) + 打花反馈(DI3) + 警告(DI4) | P1 |
| F13 | 关节监控 | 6 关节角实时曲线 + 末端位姿实时显示 | P1 |
| F14 | URDF 3D | 机械臂模型 + 工具切换显示 | P1 |

---

## 2. 完整执行管线

### 2.1 管线总览

```
┌─────────────────────────────────────────────────────────────────┐
│                    前端 (coffee_latte_panel.html)                 │
│                                                                  │
│  图案选择 → 杯子配置 → 倾倒参数 → 速度约束 → 进杯角度 → 平移微调    │
│  [实时预览] ←── waypoints ──→ [3D渲染]                           │
│  [预览] [执行] [停止] [回原点]                                    │
└───────────────────────────┬─────────────────────────────────────┘
                            │ ReplayLatteTrajectory Service
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              LatteImitationNode (trajectory_pipeline.py)         │
│                                                                  │
│  ① Generate  ──→ 参数化函数 → XYZ 轨迹                           │
│  ② Smooth    ──→ 五次样条 + S曲线速度剖面 (flow-matching 启发)    │
│  ③ Orient    ──→ 动态 pitch 剖面, 每帧独立四元数                  │
│  ④ Retarget  ──→ SE(3) 分离式: 位置完整R, 朝向仅yaw               │
│  ⑤ Preview   ──→ RViz2 markers + 前端 waypoints                  │
│  ⑥ Safety    ──→ 工作空间边界检查 (AUBO E5 886.5mm 工作半径)      │
│  ⑦ Plan      ──→ /compute_cartesian_path (MoveIt2)              │
│  ⑧ Execute   ──→ /execute_trajectory (MoveIt2 action)           │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 各阶段详解

#### Phase ①: Generate — 参数化轨迹生成

```
输入:  pattern_type, CupConfig, PourConfig
输出:  (T, 3) XYZ 位置数组

LatteArtTrajectory (latte_art/generator.py):
  .heart(num_points=200)    → 心形五步法
  .rosetta(num_points=350)  → 树叶四阶段 (推云→回拉递减→收拢→划穿)
  .tulip(layers=3)          → 郁金香多层堆叠 (层间抬高停顿)
  .swan(num_points=400)     → 天鹅组合 (身体+颈部+头部)
  .from_image(path)         → 自定义图案 (Canny边缘→样条拟合)

compose_full_trajectory(xyz, cup, pour):
  → 前加 mix 阶段 (50帧, 高位注入 + 微小扰动)
  → 后加 finish 阶段 (30帧, 上提 + 划穿)
  → 输出 (T_pattern + 80, 3) 完整轨迹
```

#### Phase ②: Smooth — 轨迹平滑 (flow-matching 启发)

```
Pi0 flow matching 的核心洞察:
  - "流场" 保证动作在位置/速度/加速度三个层次全连续
  - jerk 隐含在流场中自然最小化

我们的参数化对应:

  Step 1: 弧长参数化
    arc[i] = Σ||Δp_j||  →  等弧长重采样, 消除速度不均

  Step 2: 五次样条插值 (C² 连续)
    p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
    → 位置、速度(p')、加速度(p'') 全部连续

  Step 3: S 曲线速度剖面 (7 段式 jerk-bounded)
    ┌───┬─────┬───────┬─────┬───────┬─────┬───┐
    │+J │+a   │-J     │vmax │-J     │-a   │+J │  jerk
    │   │     │       │     │       │     │   │
    │   │  ╭──╯       ╰───╮ │       ╰──╮  │   │  acc
    │  ╭╯                 ╰╮         ╰╮ │   │
    │ ╱                    ╲          ╲╱  │   │  vel
    └─────────────────────────────────────┘
    
    对比当前梯形剖面: jerk 在拐点处为 ∞ → 液体晃动
    S曲线剖面: jerk 处处有限且连续 → 液体稳定

  Step 4: 按速度重参数化时间轴
    dt[i] = Δarc[i] / v[i]
    → 输出时间戳保证恒速段均匀, 加减速段平滑
```

#### Phase ③: Orient — 动态朝向剖面

```
问题: 当前 bridge.py 所有帧固定 pitch=45°
影响: 融合阶段奶缸过早倾斜 → 奶泡不沉底
      收尾阶段倾斜不够 → draw-through 线太粗

方案: 每帧独立四元数, pitch 按三阶段物理规律变化

  pitch
  60° ┤                              ╱
      │                             ╱
  45° ┤╲                           ╱
      │ ╲                         ╱
  30° ┤  ╲___________╱‾‾‾‾‾‾‾‾‾‾
      │
      ├─────┬─────────┬─────────┤
      │ mix │  draw   │ finish  │
      │ 25% │   60%   │   15%   │

  融合 [0~25%]:   pitch 45°→30° 线性下降
                 奶缸从高位大倾斜逐渐贴近液面

  成形 [25~85%]:  pitch 30°±3° 微变
                 贴近液面, 奶泡浮于表面形成图案
                 Rosetta 图案: 摆动自然产生 ±5° 微变化

  收尾 [85~100%]: pitch 30°→60° 线性上升
                 奶缸上提, 细流 draw-through

  roll: 全程 0 (无侧倾)
  yaw:  全程 0 (canonical frame, Phase ④ 统一变换)

实现: latte_art/orientation_profile.py (NEW)
  compute_pitch_profile(total_frames, phases)
  assemble_cartesian_with_orientation(xyz, pitch_profile)
```

#### Phase ④: Retarget — SE(3) 分离式重定目标

```
问题: 当前 retarget_trajectory() 做 q_new = q_rel ⊗ q_orig
      q_rel 包含全部旋转 → 奶缸倾倒方向被旋转 → 液体侧漏

数学分解:
  R_rel = R_z(yaw) @ R_y(pitch) @ R_x(roll)
           ↑              ↑              ↑
       绕世界Z旋转     绕世界Y旋转     绕世界X旋转
       (杯子朝向)     (倾倒方向!)     (=0, 不侧倾)

变换策略:
  ┌─ 位置 (完整 R_rel):
  │  p_new[i] = R_rel @ (p[i] - p[0]) + p_cup + translation_offset
  │  → 图案在 3D 空间随杯子整体旋转 + 平移到杯子上方
  │  → 旋转中心 = Frame 0 (轨迹起始点, 杯子上方)
  │
  └─ 朝向 (仅 yaw 分量):
     yaw_rel = extract_yaw_from_rotation(R_rel)
              = np.degrees(np.arctan2(R[1,0], R[0,0]))
     q_yaw = euler_deg_to_quat(0, 0, yaw_rel)
     q_new[i] = q_yaw ⊗ q_profile[i]
     → pitch 剖面完全不受影响
     → 奶缸始终向下倾倒, 不会侧漏

yaw 提取的数学基础:
  R[1,0] = sin(yaw)·cos(pitch),  R[0,0] = cos(yaw)·cos(pitch)
  → atan2(R[1,0], R[0,0]) = yaw   (当 cos(pitch) ≠ 0)
  → 拉花 pitch ∈ [25°, 60°] → cos(pitch) ≠ 0 → 无 gimbal lock

实现: trajectory_transform.py (MODIFIED)
  extract_yaw_from_rotation(R)
  retarget_with_orientation_constraint(cart, start_pose, rpy_user, translation_offset)
```

#### Phase ⑤: Preview — 双向预览

```
ROS 端:
  发布 5 种 RViz2 markers:
    ~/preview/tcp_path         Path       绿色 TCP 轨迹
    ~/preview/tcp_waypoints    PoseArray  方向箭头 (每5帧)
    ~/preview/spout_path       Marker     蓝色壶嘴轨迹
    ~/preview/cup_pose         Marker     黄色杯子方块
    ~/preview/workspace_bounds Marker     红色安全框

  采样 waypoints 返回到前端:
    sampled_poses = [cart.to_pose(i) for i in range(0, N, step)]

前端:
  latte/main.js (ros3djs + Three.js):
    接收 waypoints → Path 线段 + PoseArray 箭头 → 3D 渲染
    实时预览: 参数变更 200ms 防抖后自动刷新
```

#### Phase ⑥: Safety — 工作空间检查

```
AUBO E5 工作半径 886.5mm (URDF DH 链审计 + AUBO 官方规格书):

  修正后的安全边界:
    X: [-0.87, 0.87]   (886.5mm × cos(45°), 留~2%裕度)
    Y: [-0.87, 0.87]   (同上)
    Z: [-0.85, 1.10]   (URDF DH 链: shoulder@0.122, 臂完全伸展)

  安全策略:
    mode="preview"/"debug": 仅报告警告, 不阻止
    mode="action": warn_and_block → 超出边界阻止执行
```

#### Phase ⑦: Plan — MoveIt2 笛卡尔规划

```
waypoints = [cart.to_pose(i) for i in range(0, N, step)]

每个 waypoint 是 geometry_msgs/Pose:
  position:    (x, y, z)     — retarget 后的位置
  orientation: (qx,qy,qz,qw) — 约束后的朝向 (仅 yaw 被旋转)

/compute_cartesian_path 参数:
  max_step = 0.01           覆盖帧间位移 ~3.8mm
  jump_threshold = 0.0      禁用相对跳变
  revolute_jump_threshold = 0.0  禁用绝对跳变
  avoid_collisions = True   启用碰撞检测

分数处理:
  fraction ≥ 0.95  → 直接执行
  0.50 ≤ fraction < 0.95 → 重试 avoid_collisions=False
  fraction < 0.50  → 失败返回
```

#### Phase ⑧: Execute — 轨迹执行

```
按 speed_scale 缩放时间戳:
  for i, pt in enumerate(trajectory.joint_trajectory.points):
      t = i * dt / speed_scale
      pt.time_from_start.sec = int(t)
      pt.time_from_start.nanosec = int((t - int(t)) * 1e9)

发送到 /execute_trajectory action:
  进度心跳每 5s 一次 → 前端显示 "执行中 (已等待 XXs...)"
  完成 → 成功/失败消息
```

---

## 3. RPY 滑块语义

```
用户在 Web 端调 RPY 三个值, 作用于 R_rel = R_z(yaw) @ R_y(pitch) @ R_x(roll):

  Roll (绕X): → R_rel 的 roll  分量
    作用于: 位置 (图案在 YZ 平面旋转)
    不作用于: 朝向 (奶缸 roll 始终为 0)
    → "图案在杯子里转了转"

  Pitch (绕Y): → R_rel 的 pitch 分量
    作用于: 位置 (图案在 XZ 平面旋转)
    不作用于: 朝向 (轨迹自身的 pitch 剖面控制倾倒角度)
    → "图案前后倾斜了一点"

  Yaw (绕Z): → R_rel 的 yaw   分量
    作用于: 位置 AND 朝向
    → "进杯方向改变了"
    关键: yaw 绕世界 Z (重力方向) → 不改变倾倒对齐

  平移微调 ΔX/ΔY/ΔZ: 直接叠加到 p_cup 上, 手动修正轨迹起点
```

---

## 4. start_pose 语义 — 两种场景

```
场景 A: 参数化生成 (pattern_type 非空)
  → retarget 目标 = 手动设置的杯子位姿
  → start_pose 使用杯子配置的 (cup_x, cup_y, cup_z)
  → q_cup = euler(0, 0, yaw_user)  (仅 yaw, 杯子放置在桌面)
  → 当前 TF 位姿仅用于后续可能的"接近段"规划

场景 B: 录制回放 (pattern_type 为空)
  → retarget 目标 = 当前 EE 位姿 (TF) 或手动设置
  → 轨迹平移到机械臂当前所在位置
  → 保持现有逻辑不变

实现: trajectory_pipeline.py Phase ②
  if pattern_type:
      target = 杯子位姿 (来自 cup_config)
  else:
      target = TF 当前位姿 (现有逻辑)
```

---

## 5. 手动参数完整清单

### 5.1 ROS2 参数 (一次性设置, 前端设置面板调整)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| coffee_link.x | -0.645 | 咖啡机出杯 X (base_link) |
| coffee_link.y | 0.098 | 咖啡机出杯 Y |
| coffee_link.z | 0.05 | 咖啡机出杯 Z (需真机微调) |
| lizhu_link.x | -0.630 | 立柱 X |
| lizhu_link.y | -0.368 | 立柱 Y |
| lizhu_link.z | 0.04 | 液面 Z = 立柱顶 + 杯高 (需真机微调) |
| cup0_link.x | -0.528 | 牛奶杯 X |
| cup0_link.y | -0.198 | 牛奶杯 Y |
| cup0_link.z | 0.05 | 牛奶杯 Z (需真机微调) |
| reference_pose.x/y/z | JSON 参考值 | 杯口朝上安全位姿 |
| reference_pose.roll/pitch/yaw | (-23.5, 88.1, 76.0) | 杯口竖直朝上 |

配置文件: `latte_imitation/config/latte_positions.yaml` (NEW)

### 5.2 前端操作参数 (每次可调, 拉花面板)

| 参数 | 前端 ID | 典型值 | 说明 |
|------|---------|--------|------|
| 杯子中心 X | latte-cupX | 0.35 m | 杯口在 base_link 中的 X |
| 杯子中心 Y | latte-cupY | -0.05 m | 杯口在 base_link 中的 Y |
| 液面高度 Z | latte-cupZ | 0.22 m | 液面在 base_link 中的 Z |
| 杯口半径 | latte-cupR | 40 mm | 影响图案缩放 |

### 5.3 有默认值可调 (工艺参数)

| 参数 | 前端 ID | 默认值 | 来源 |
|------|---------|--------|------|
| 融合高度 | latte-mixH | 76 mm | Sunergos 视频: "3 英寸" |
| 成形高度 | latte-drawH | 6 mm | Sunergos 视频: "1/4 英寸" |
| 收尾高度 | latte-finishH | 76 mm | Sunergos 视频: "3 英寸" |
| 摆动振幅 | latte-wiggleAmp | 6 mm | latteartguide.com + 视频 |
| 摆动频率 | latte-wiggleFreq | 5 Hz | latteartguide.com (3-6Hz) |
| 最大速度 | latte-maxVel | 0.05 m/s | Di Leva 2023 抗晃荡论文 |
| 最大加速度 | latte-maxAcc | 0.1 m/s² | Di Leva 2023 |
| 最大 jerk | latte-maxJerk | 0.5 m/s³ | Di Leva 2023 |
| 抗晃荡 | latte-antiSlosh | ✓ | S 曲线速度剖面 |

### 5.4 微调参数

| 参数 | 前端 ID | 默认值 | 作用 |
|------|---------|--------|------|
| Roll | latte-roll | 0° | 图案 YZ 面旋转 (不影响倾倒) |
| Pitch | latte-pitch | 0° | 图案 XZ 面旋转 (不影响倾倒) |
| Yaw | latte-yaw | 0° | 进杯方向 (影响位置+倾倒方向) |
| ΔX | latte-dx | 0 mm | 轨迹起点平移微调 |
| ΔY | latte-dy | 0 mm | 同上 |
| ΔZ | latte-dz | 0 mm | 同上 |
| 速度倍率 | latte-speedScale | 1.0 | 整体调速 (0.01~10.0) |
| 采样步长 | latte-waypointStep | 5 | MoveIt2 waypoint 间隔 |

### 5.5 系统配置 (一次性)

| 参数 | 位置 | 值 |
|------|------|-----|
| 工具偏移 (TCP→壶嘴) | config/tool_offset.yaml | (0, 0, -0.15) m |
| 工作空间边界 | config/workspace_safety.yaml | X/Y ±0.87, Z [-0.85, 1.10] |
| HOME 位姿 | 前端 localStorage (待定义) | 安全回退位姿 |

---

## 6. 前端界面改造

### 6.1 参数分布

```
设置面板 (话题设置弹窗扩展 或 独立 Settings 区):
  ┌─ 参考位置微调 ──────────────────────────────────────────┐
  │ coffee_Link (取咖啡杯):                                  │
  │   X[-0.645] Y[0.098] Z[0.05] Roll[0] Pitch[0] Yaw[0]   │
  │ lizhu_Link (放咖啡杯):                                   │
  │   X[-0.630] Y[-0.368] Z[0.04] Roll[0] Pitch[0] Yaw[0]  │
  │ cup0_Link (取牛奶杯):                                    │
  │   X[-0.528] Y[-0.198] Z[0.05] Roll[0] Pitch[0] Yaw[0]  │
  │ 参考位姿 (杯口朝上):                                      │
  │   X[-0.419] Y[-0.400] Z[0.246] R[-23.5] P[88.1] Y[76.0]│
  │                                                        │
  │ [恢复默认] [保存]                                        │
  └────────────────────────────────────────────────────────┘

拉花面板 (咖啡拉花参数控制):
  ┌─ 轨迹图案 ────────────────────────────────────────────┐
  │ [心形] [树叶] [郁金香] [天鹅] [录制回放]                │
  └──────────────────────────────────────────────────────┘
  ┌─ 杯子配置 ───────────────────────────────────────────┐
  │ 液面Z [0.04]m  杯半径 [40]mm                          │
  │ (XY 自动使用 lizhu_Link 位置, 在设置面板中调整)        │
  └──────────────────────────────────────────────────────┘
  ┌─ 倾倒工艺 ── [NEW] ─────────────────────────────────┐
  │ 融合高[76] 成形高[6] 收尾高[76] 摆幅[6] 频率[5]Hz    │
  └──────────────────────────────────────────────────────┘
  ┌─ 速度约束 ── [NEW] ─────────────────────────────────┐
  │ Vmax[0.05] Amax[0.1] Jmax[0.5] [✓]抗晃荡            │
  └──────────────────────────────────────────────────────┘
  ┌─ 进杯角度 ──────────────────────────────────────────┐
  │ Roll[0]° Pitch[0]° Yaw[0]° 速度倍率[1.0]            │
  └──────────────────────────────────────────────────────┘
  ┌─ 平移微调 ──────────────────────────────────────────┐
  │ ΔX[0]mm ΔY[0]mm ΔZ[0]mm 采样步长[5]                  │
  └──────────────────────────────────────────────────────┘
  [预览] [执行] [停止] [回原点]
```

### 6.2 条件显示规则

```
patternType === ''  (录制回放):
  显示: Episode 选择, RPY, 平移微调, 预览, 按钮
  隐藏: 杯子配置, 倾倒参数, 速度约束

patternType !== ''  (参数化生成):
  显示: 杯子配置, 倾倒参数, 速度约束, RPY, 平移微调, 预览, 按钮
  隐藏: Episode 选择

patternType === 'tulip':
  显示: 郁金香层数输入框 (在倾倒参数组内)
  隐藏: (无额外)

patternType !== 'tulip':
  隐藏: 郁金香层数输入框
```

---

## 7. 数据流全链路 (心形拉花示例)

```
┌─ 前端输入 ────────────────────────────────────────────────────┐
│ pattern_type = "heart"                                        │
│ cup_x=0.35, cup_y=-0.05, cup_z=0.22, cup_r=0.04              │
│ mix_h=0.076, draw_h=0.006, finish_h=0.076                     │
│ wiggle_a=0.006, wiggle_f=5.0                                  │
│ v_max=0.05, a_max=0.1, j_max=0.5, anti_slosh=true             │
│ roll=0, pitch=0, yaw=15, speed=1.0                            │
│ dx=0, dy=0, dz=0, step=5                                      │
│ mode = "action"                                                │
└──────────────────────────┬────────────────────────────────────┘
                           ▼
┌─ Phase ①: Generate ──────────────────────────────────────────┐
│ cup = CupConfig(0.35, -0.05, 0.22, 0.04)                      │
│ pour = PourConfig(0.076, 0.006, 0.076, 0.006, 5.0, ...)      │
│ gen = LatteArtTrajectory(cup, pour)                            │
│ xyz = gen.heart()                          → (200, 3)        │
│ xyz = compose_full_trajectory(xyz, cup, pour) → (280, 3)     │
│ xyz = apply_anti_sloshing(xyz, pour)       → (280, 3) smooth │
└──────────────────────────┬────────────────────────────────────┘
                           ▼
┌─ Phase ②: Smooth (flow-matching 启发) ───────────────────────┐
│ arc = cumulative(||Δxyz||)                                     │
│ xyz = quintic_spline_resample(xyz, arc) → C² 连续             │
│ vel_profile = s_curve_profile(arc, v_max, a_max, j_max)        │
│ xyz = reparameterize_by_velocity(xyz, arc, vel_profile)        │
└──────────────────────────┬────────────────────────────────────┘
                           ▼
┌─ Phase ③: Orient ────────────────────────────────────────────┐
│ mix_end=67, draw_end=238, finish_end=280                       │
│ pitch[0:67]    = linspace(45°, 30°, 67)                        │
│ pitch[67:238]  = 30° + 3°·sin(2π·0.5·t)                       │
│ pitch[238:280] = linspace(30°, 60°, 42)                        │
│                                                                │
│ for i in 0..279:                                               │
│   q[i] = euler_deg_to_quat(0, pitch[i], 0)                     │
│                                                                │
│ cart = CartesianTrajectory(positions=xyz, orientations=q)      │
│                                                                │
│ Frame   0: pos=(0.35, -0.038, 0.296), q≈euler(0,45,0)        │
│ Frame 140: pos=(0.35, -0.05,  0.226), q≈euler(0,30,0)        │
│ Frame 279: pos=(0.35, -0.058, 0.296), q≈euler(0,60,0)        │
└──────────────────────────┬────────────────────────────────────┘
                           ▼
┌─ Phase ④: Retarget (分离式) ─────────────────────────────────┐
│ R_rel = R(0,0,15) @ R(identity) = R_z(15°)                    │
│ yaw_rel = extract_yaw(R_rel) = 15°                            │
│                                                                │
│ 位置 (完整 R):                                                 │
│   p_new[i] = R_z(15°) @ (p[i] - p[0]) + (0.35, -0.05, 0.22)  │
│   → 整个心形绕 Z 轴旋转 15°                                    │
│                                                                │
│ 朝向 (仅 yaw):                                                 │
│   q_yaw = euler(0, 0, 15°)                                    │
│   q_new[i] = q_yaw ⊗ euler(0, pitch[i], 0)                    │
│   → pitch 剖面不受影响, 奶缸倾倒方向正确                        │
└──────────────────────────┬────────────────────────────────────┘
                           ▼
┌─ Phase ⑤~⑧: Preview → Safety → Plan → Execute ─────────────┐
│ Preview:  发布 RViz2 markers + 返回 56 waypoints 到前端        │
│ Safety:   全部帧在 X/Y±0.87, Z[-0.85,1.10] 内 ✓               │
│ Plan:     /compute_cartesian_path → fraction = 98.5% ✓        │
│ Execute:  /execute_trajectory → 机械臂执行拉花                  │
└────────────────────────────────────────────────────────────────┘
```

---

## 8. latte-art-robot 的借鉴与差异

### 8.1 借鉴的部分

| 概念 | latte-art-robot | 我们的应用 |
|------|----------------|-----------|
| 心形五步法动作 | Pi0 隐式学习 40 个示教 | 参数化函数显式编码 |
| 三阶段高度 | 从示教数据统计得出 | 多源交叉验证的固定值 |
| 杯子倾斜 | 左臂持续旋转 30-70° | 无法实现 (单臂, 杯子固定) |
| Flow matching 平滑 | 神经网络流场保证 C² 连续 | 五次样条 + S 曲线剖面 |
| Temporal ensembling | 相邻 chunk 加权融合 | 样条插值天然 C² 连续 |
| Action chunk (20步) | 1 秒预测视界 | waypoint 采样 (每5帧) |

### 8.2 差异

| 维度 | latte-art-robot | 本方案 |
|------|----------------|--------|
| 轨迹生成 | 神经网络 (Pi0, 需 GPU) | 参数化函数 (CPU 毫秒级) |
| 朝向控制 | 隐式学习 (黑盒) | 显式动态 pitch 剖面 |
| 泛化方式 | 需新示教数据 fine-tune | 修改函数参数即可 |
| 实时调整 | 重新推理 (秒级) | 重新生成 (毫秒级) |
| 可解释性 | 黑盒 | 每个参数有物理含义 |
| 双臂→单臂 | 直接不可用 | 专门为单臂设计 |

---

## 9. 文件改动清单

### 9.1 后端新增

| 文件 | 说明 |
|------|------|
| `latte_art/orientation_profile.py` | `compute_pitch_profile()` + `assemble_cartesian_with_orientation()` |
| `config/latte_positions.yaml` | 4 个参考位姿的 ROS2 参数默认值 (coffee/lizhu/cup0/reference) |

### 9.2 后端修改

| 文件 | 改动 |
|------|------|
| `latte_art/config.py` | `PourConfig` 加朝向剖面字段 (7 个) |
| `latte_art/bridge.py` | `parametric_to_cartesian()` → 调用 `assemble_cartesian_with_orientation()` |
| `latte_art/__init__.py` | 导出新模块 |
| `trajectory_transform.py` | 新增 `extract_yaw_from_rotation()` + `retarget_with_orientation_constraint()` |
| `trajectory_pipeline.py` | Phase ②: 从 ROS2 参数获取 lizhu_Link 位置; Phase ④: 调新 retarget 函数 |
| `config_loader.py` | 新增 `load_latte_positions()` 加载参考位姿 YAML |

### 9.3 后端不改

| 文件 | 原因 |
|------|------|
| `trajectory.py` | `CartesianTrajectory` 数据结构不变 |
| `generator.py` | XYZ 生成逻辑不变 |
| `anti_sloshing.py` | 升级实现在 `orientation_profile.py` 中, 旧文件保留 |

### 9.4 前端修改

| 文件 | 改动 |
|------|------|
| `coffee_latte_panel.html` | 新增杯子配置/倾倒参数/速度约束 3 组 DOM; 杯子 XY 从 lizhu_Link 继承 |
| `coffee_latte_panel.css` | 新增 range slider 样式 + 条件显示样式 |
| `latte_controls.js` | 完善条件显示逻辑; 杯子 XY 从 ROS2 参数/BFF 获取; 更新 DEFAULTS |
| `settings_panel.html` 或独立 Settings 区 | **新增** 参考位姿微调区域 (4组 × 6DOF) |

### 9.5 前端不改

| 文件 | 原因 |
|------|------|
| `latte/main.js` | 3D 渲染接口不变 (waypoints 格式不变) |
| `coffee_latte_io.js` | DO/DI 控制独立功能 |
| `core/ros.js` | 传输层不变 |
| `core/log-bus.js` | 日志系统不变 |

### 9.6 接口不变

```
Service: /latte_imitation/replay_trajectory
  Request:  ReplayLatteTrajectory (格式不变)
  Response: ReplayLatteTrajectory_Response (格式不变)

内部替换:
  retarget_trajectory() → retarget_with_orientation_constraint()
  parametric_to_cartesian() → assemble_cartesian_with_orientation()

对外完全兼容, 前端无需改动 service 调用逻辑
```

---

## 10. 实施顺序

| 阶段 | 内容 | 预计改动量 |
|------|------|-----------|
| Step 1 | `orientation_profile.py` + `config.py` + `bridge.py` | ~150 行新增 |
| Step 2 | `trajectory_transform.py` (extract_yaw + retarget_constrained) | ~80 行新增 |
| Step 3 | `config/latte_positions.yaml` + `config_loader.py` | ~40 行新增 |
| Step 4 | `trajectory_pipeline.py` (ROS2参数获取 + 新函数调用) | ~40 行修改 |
| Step 5 | 前端 HTML + CSS (拉花面板: 杯子/倾倒/速度 3 组 DOM) | ~80 行新增 |
| Step 6 | 前端 设置面板 (参考位姿微调 4 组) | ~60 行新增 |
| Step 7 | 前端 latte_controls.js (条件显示 + 默认值更新) | ~30 行修改 |
| Step 8 | 编译 + 测试 + 前端联调 | — |
