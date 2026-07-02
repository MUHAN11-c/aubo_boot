# latte_backend — 咖啡拉花 ROS 2 后端

独立功能包，包含拉花工作流编排 + 心形轨迹生成。依赖 `demo_driver` (RobotController 运动基础层)、`ivg_interfaces`、MoveIt2。

## LatteWorkflowNode — 咖啡拉花 5 步工作流

**Service**: `/latte/run_workflow` (ivg_interfaces/srv/RunLatteWorkflow)

### 坐标系约定

代码中所有 "roll" 指 **绕世界坐标系 X 轴的倾角**（非 TCP body-fixed roll）：

```
世界 X 轴 (前) → 倾倒倾角轴: 绕此轴旋转使奶缸前倾(+)/后仰(-)
世界 Y 轴 (左) → 划穿方向轴: 奶缸嘴沿此方向倒奶, cut-through 沿此方向  
世界 Z 轴 (上) → 高度轴: 液面上方距离
```

step4 提供 45° 基准倾角（绕 X 轴），step5 各阶段在此基准上叠加增量：`rel_roll = roll_deg - 45°`

### 工作流步骤

| 步骤 | 函数 | 运动 | 说明 | 状态 |
|------|------|------|------|------|
| 0 | `step0_pickCoffee` | `moveToJoints` | 取咖啡杯 (保留未启用, 多杯方案时可激活) | ⏸️ |
| 0 | `step0_placeCoffee` | `moveToJoints` | 放咖啡杯 (保留未启用, 多杯方案时可激活) | ⏸️ |
| 1 | `step1_pickMilk` | `moveToJoints` | 取牛奶杯 → 抓取 (不退避) | ✅ |
| 2 | `step2_approachNozzle` | 笛卡尔来回×2 | 去喷嘴 → 2s打奶泡 → 回 → Z退避 | ✅ |
| 3a | `step3_reorient` | 笛卡尔 slerp | 转腕朝上: FK rotate_up_joints | ✅ |
| 3b | `step3_reorient` | 笛卡尔 slerp | 放置咖啡杯: FK place_coffee_joints | ✅ |
| 4 | `step4_pour` | 笛卡尔 slerp | 绕世界X轴前倾45° (倾倒基准) | ✅ |
| 5 | `step5_executeLatte` | 笛卡尔分段 | 心形拉花: 融合画圈→成形注入→划穿收尾 | ✅ |

### step5 心形轨迹分段 (基于 Barista Hustle MSLA 权威资料)

| 子阶段 | 高度 (Z) | 倾角 (Roll) | 运动 | 点数占比 |
|--------|----------|-------------|------|----------|
| [5a] 融合画圈 | 80mm (高位) | 45° | r=10mm 画圈 ×2圈, 细流穿透 crema | 25% |
| [5b] 成形注入 | 5mm (贴液面) | 60°→45° 动态渐变 | 壶嘴杯中心定点, 高流量泡沫浮面扩散白圆 | 55% |
| [5c] 划穿收尾 | 80mm (抬高) | 50° | 沿世界Y轴推进 15mm, 细流穿过圆心产生尖部 | 20% |

### 启动

```bash
# 完整拉花工作流 (需先启动机械臂驱动)
ros2 launch latte_backend latte_workflow.launch.py

# 单独测试拉花轨迹 (无需完整机械臂驱动)
./start_latte_test.sh
```

### 测试命令

```bash
# 执行完整 5 步工作流
ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow "{}"
```

### 参数 (lwf_ 前缀)

所有参数通过 ROS 2 Parameter Server 管理，每次 service 调用前实时刷新。

#### 控制参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lwf_approach_height` | double | 0.15 | 笛卡尔接近高度 (m) |
| `lwf_retract_height` | double | 0.10 | 笛卡尔退避高度 (m) |
| `lwf_velocity` | double | 0.5 | 通用速度因子 [0,1] |
| `lwf_acceleration` | double | 0.5 | 通用加速度因子 [0,1] |
| `lwf_gripper_pin` | int | 6 | 夹爪 IO pin |
| `lwf_pattern_type` | string | "heart" | 拉花图案类型 |
| `lwf_execute_latte` | bool | true | false 时跳过 step5 |

#### 心形轨迹参数 (lwf_heart_ 前缀)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lwf_heart_mix_height` | double | 0.08 | 融合高度 (m) |
| `lwf_heart_draw_height` | double | 0.005 | 成形高度 (m) |
| `lwf_heart_mix_circle_r` | double | 0.010 | 融合画圈半径 (m) |
| `lwf_heart_mix_circles` | double | 2.0 | 融合画圈圈数 |
| `lwf_heart_push_y` | double | 0.015 | 划穿收尾推进距离 (m) |
| `lwf_heart_total_points` | int | 200 | 总路点数 (分配: 25%+55%+20%) |
| `lwf_heart_sway_offset_y` | double | 0.01 | 世界Y轴杯前偏移 (m) |
| `lwf_heart_velocity` | double | 0.5 | 拉花专用速度缩放 |
| `lwf_heart_verbose` | bool | true | 打印阶段详情 |
| `lwf_heart_roll_mix` | double | 45.0 | 融合绝对倾角 (°) |
| `lwf_heart_roll_draw` | double | 60.0 | 成形绝对倾角 (°) |
| `lwf_heart_roll_finish` | double | 50.0 | 收尾绝对倾角 (°) |
| `lwf_heart_roll_draw_start` | double | 60.0 | 成形起始倾角 (°) |
| `lwf_heart_roll_draw_end` | double | 45.0 | 成形结束倾角 (°) |
| `lwf_heart_roll_draw_dynamic` | bool | true | true=渐变, false=固定 roll_draw |

### 仿真容错

- IO 操作 (夹爪) 失败时仅打印 WARN 日志，继续执行
- 笛卡尔直线失败自动重试: 底层 5 次 × 步级 3 次 = 最多 15 次
- `roll_draw_dynamic=false` 时回退到固定倾角，降低 IK 求解失败风险
- 仿真模式下 `/move_to_pose` 可能阻塞（CurrentStateMonitor 超时），建议仅在真机测试 step5
