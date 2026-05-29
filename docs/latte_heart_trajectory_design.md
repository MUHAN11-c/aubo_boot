# 心形拉花轨迹设计文档

## 坐标系映射

从 `latte_workflow_node.cpp` step4 验证：倾倒旋转为**绕世界 X 轴左乘 45°**。

```
q_new = q_rot_x * q_current  (左乘 = 全局 X 轴旋转)
```

- 倾倒平面 = 世界 YZ 平面（牛奶沿 Y 方向流出，Z 方向下落）
- 世界 X 轴垂直于倾倒平面 = 左右摆动轴
- 杯坐标系与世界坐标系 **1:1 对齐**（杯在桌面上，Z 朝上）

| 杯轴 | 运动 | 世界轴映射 |
|------|------|-----------|
| 杯 X | 左右摆动 (wiggle) | 世界 X |
| 杯 Y | 前后划穿 (draw-through) | 世界 Y |
| 杯 Z | 高度控制 | 世界 Z |

## 心形轨迹 4 阶段

以 step4 结束时的 TCP 位姿 `P0 = (x0, y0, z0, q0)` 为起点。
200 个 waypoints，总时长约 4 秒（50Hz 路点密度）。

| 阶段 | 占比 | 路点范围 | X | Y | Z | 朝向 |
|------|------|---------|---|---|---|------|
| 1. 融合 | 0-25% | 0-50 | `x0` | `y0` | `z0` | `q0` |
| 2. 成形摆动 | 25-70% | 50-140 | `x0 + A·sin(3·2π·t)` | `y0` | `z0 - 76mm + 6mm` | `q0` |
| 3. 抬升 | 70-85% | 140-170 | `x0` | `y0` | `z0 - 76mm + 6mm → z0` | `q0` |
| 4. 划穿收尾 | 85-100% | 170-200 | `x0` | `y0 → y0 - 8mm` | `z0` | `q0` |

### 硬编码默认参数

| 参数 | 值 | 来源 |
|------|-----|------|
| 融合高度 | 液面 + 76mm (3 英寸) | Sunergos 视频 / latteartguide.com |
| 成形高度 | 液面 + 6mm (1/4 英寸) | Sunergos 视频 / latteartguide.com |
| 收尾高度 | 液面 + 76mm (3 英寸) | Sunergos 视频 |
| 摆动振幅 | 1.8mm (6mm × 0.3 心形专用) | latte_art_trajectory.py |
| 摆动周期 | 3 个完整正弦波（成形阶段内） | latte_art_trajectory.py |
| 杯口半径 R | 40mm（标准拿铁杯 8cm 直径） | CupConfig |
| 划穿距离 | 0.35R ≈ 14mm（杯前 0.15R → 杯后 0.2R） | latte_art_trajectory.py heart() |
| 朝向 | 保持 step4 的 q0 不变（X 轴 45° 倾倒） | latte_workflow_node.cpp step4 |
| 最大速度 | 0.05 m/s | 抗晃荡约束 (Di Leva 2023) |
| eef_step | 0.002m | robot_controller.cpp |

### 阶段说明

1. **融合段** — 位置不动，奶泡以较高动能冲入咖啡底层（76mm 高度差）
2. **成形段** — X 轴 1.8mm 振幅正弦摆动，奶缸嘴紧贴液面（6mm），白色奶泡浮于表面形成圆形
3. **抬升段** — Z 轴从 6mm 线性升至 76mm，"吸力"使圆形顶部形成心形弧线
4. **划穿段** — Y 轴从杯前部穿过圆心移至杯后部，Z 保持 76mm 高度，在图案中穿过一条细线

## 服务接口

新建 `ivg_interfaces/srv/PourHeart.srv`：

```
# Request
float64 cup_surface_z   # 咖啡液面高度 (m)

# Response
bool   success
string message
```

服务名：`/latte/pour_heart`

## 执行方式

使用 MoveIt2 `MoveGroupInterface::computeCartesianPath()` 多路点版本：

```cpp
move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, traj,
                                   avoid_collisions, &error_code);
move_group_->execute(plan);
```

复用 `demo_driver::RobotController` 的 MoveGroupInterface 实例化模式。

## 架构

```
latte_workflow_node.cpp (step1-4 不变)
    │
    │ step5: 调用 /latte/pour_heart
    ▼
latte_pour_node (新 C++ 节点, demo_driver 包)
    │
    ├── 从当前 TCP 位姿获取起点 P0
    ├── 生成 200 个世界坐标系 waypoints
    ├── MoveIt2 computeCartesianPath (多路点)
    └── execute()
```

完全舍弃旧设计：
- `latte_imitation` (Python 管线) → 废弃
- `latte_cartesian_planner` (C++ Plan+Execute 节点) → 废弃
- `ivg_interfaces/srv/ReplayLatteTrajectory.srv` → 废弃
- `ivg_interfaces/srv/LatteCartesianPlan.srv` → 废弃
