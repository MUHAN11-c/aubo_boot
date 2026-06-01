# 咖啡拉花心形图案 — 理论与实现

本文件整合 Barista Hustle MSLA (Milk Science & Latte Art) 课程理论、SCA WLAC 竞赛标准、流体力学原理，以及本项目的机器人实现细节。

> **权威参考**: [Barista Hustle MSLA 课程](https://www.baristahustle.com/education-products/single-course-sales/course-milk-science-and-latte-art/)、SCA WLAC (World Latte Art Championship) 评分标准。

---

## 1. 牛奶科学基础

### 1.1 蒸汽打奶 (MSLA Ch.3)

| 参数 | 目标值 | 原理 |
|------|--------|------|
| 终点温度 | 55–65°C | 乳糖甜度峰值在 60°C；超过 70°C 蛋白质变性、甜度骤降 |
| 蒸汽时间 | <20 秒 | 蒸汽提供汽化潜热 (latent heat of condensation)，短时间高效率注入 |
| 空气注入量 | 20–30% 体积增加 | 形成微泡沫 (microfoam)，气泡直径 <0.1mm |
| 漩涡 (vortex) | 持续到温度达标 | 将粗泡打碎并均匀分布，避免分层 |

**微泡沫质量标准** (MSLA 2.04)：
- 表面光泽、无可见气泡 (wet paint / glossy finish)
- 流动性好，可倾倒 (非凝固状态)
- 泡沫与液体牛奶均匀混合，无分层

### 1.2 倾倒流体力学 (MSLA Ch.4)

#### 流速-线宽关系 (MSLA 4.04)

```
线宽 ∝ √流量

流量 +100% → 线宽 +42%
```

| 阶段 | 流量 | 线宽 (估计) | 用途 |
|------|------|------------|------|
| 填充 (filling) | ~10 ml/s | ~3mm | 构建 canvas 基底 |
| 绘画 (painting/drawing) | ~20 ml/s | ~5mm | 形成白色图案 |
| 加速 (turbo/heart-in-heart) | 30–40 ml/s | ~8mm | 嵌套心形设计 |

#### 倾倒高度效应 (MSLA 4.02 — Pin-Drop Technique)

| 高度 | 效果 | 问题 |
|------|------|------|
| ~10 cm (推荐) | 流线连续、涡流正确 | — |
| >10 cm (过高) | **piddles** — 流线断裂、飞溅、气泡 | 终端速度导致表面张力失效 |
| <5 cm (过低) | **snail trails** — 褪色条纹 | 泡沫留在表面而非穿透 crema |

> **Pin-Drop 原则**: 奶流应如针脚入水，垂直进入液面，保持流线连续。

#### 倾倒倾角-流量关系

```
倾角越大 → 重力沿流出方向分量越大 → 流速越高
```

奶缸嘴倾角 (°) 与有效流出面积 (重力辅助) 的关系：

| 倾角 | 相对流速 | 对应阶段 |
|------|---------|---------|
| 45° | ~10 ml/s | 融合 (mix) — 细流穿透 crema 沉底 |
| 50° | ~15 ml/s | 收尾 (finish) — 中速细流 |
| 60° | ~20 ml/s | 成形 (draw) — 高流量泡沫浮面扩散白圆 |

---

## 2. 心形图案解剖 (MSLA Ch.5)

### 2.1 图案结构

心形 (Heart) 是僧帽 (Monk's Head) 的进阶版，属于**双元素设计**：

```
    融合画圈 (僧帽白圆)          划穿收尾 (心形尖部)
    ╭────────────────╮         ╭──────────────╮
    │   ⊚ → ⊚         │  +      │   ↓ 细流穿透  │  =  ♥
    │  r=1cm, 2圈     │         │   沿Y轴 1.5cm │
    ╰────────────────╯         ╰──────────────╯
```

### 2.2 三大阶段

#### [5a] 融合画圈 (Mix Circle) — 占总时长 25%

**目的**: 建立白色圆形基底 (僧帽白圆)，为后续成形提供画布。

| 参数 | 值 | MSLA 依据 |
|------|-----|-----------|
| 奶缸高度 (液面上方) | 80mm (7–10cm) | MSLA 5.02: 推荐 10cm |
| 奶缸倾角 (绕X轴) | 45° | 细流 ~10ml/s, 高位穿透 crema |
| 画圈半径 | 10mm (~1cm) | 硬币大小, MSLA 推荐 |
| 圈数 | 2 圈 | 足够建立清晰圆形 |
| 轨迹总长 | ~126mm | 2πr × 2 = 12.6cm |

**运动模式**: 在 X-Y 平面画圆 (世界坐标系), 奶缸嘴绕杯中心做圆周运动。Z 固定在高位 (80mm)，奶流从高处落下穿透咖啡油脂 (crema) 沉入杯底，白色微泡沫在表面形成圆形。

**关键细节**:
- 圆必须画在杯中心偏前 (Y 轴 sway_offset_y=10mm 偏移)，留空间给后面的划穿
- 融合阶段不需要摆动 (wiggle)，摆动仅用于 Rosetta 叶形图案
- 画圈结束后，圆形应清晰可见，边缘光滑

#### [5b] 成形注入 (Draw / Pour) — 占总时长 55%

**目的**: 定点注入高流量奶泡，白色圆形自然扩散成形。

| 参数 | 值 | MSLA 依据 |
|------|-----|-----------|
| 奶缸高度 (液面上方) | 5mm (<1cm) | MSLA 4.02: 降低高度减少飞溅 |
| 奶缸倾角 (起始→结束) | 60° → 45° (动态渐变) | 模拟人类回正动作 |
| 流速 | ~20 ml/s | MSLA 4.04: 绘画速度 |
| 奶缸嘴位置 | X=原点, Y=杯前偏移 | MSLA B1 5.04-5.05: 壶嘴固定 |

**运动模式**: **奶缸嘴杯中心固定不动** — 这是心形与 Rosetta 的关键区别。白色圆形**不是靠移动奶缸形成，而是靠流速加速自然扩散**。

**倾角渐变原理** (模拟人类操作):
```
成形开始时倾角 60° (高流量 ~20ml/s) → 白色泡沫快速堆积，建立白圆基底
成形结束时倾角 45° (中流量 ~10ml/s) → 逐渐降低流量，防止溢出或过厚
```

**物理过程**:
1. 高流量奶泡从 5mm 低位注入，因距离近而直接浮在液面 (不穿透 crema)
2. 白色微泡沫以注入点为中心向外扩散，形成圆形白斑
3. 扩散速度由流速决定——倾角越大、流量越高、扩散越快
4. 成形阶段结束后，杯中应有一个占据约 60–70% 面积的白色圆形

**关键细节**:
- 如果奶缸移动，会产生条纹 (snail trail) 或不对称图案
- 如果流量不够，白圆太小或颜色太淡
- 如果流量太大，白圆可能溢出杯边缘
- 可以通过 `lwf_heart_roll_draw_dynamic=false` 回退到固定倾角 60°

#### [5c] 划穿收尾 (Cut-through / Finish) — 占总时长 20%

**目的**: 奶缸抬高，沿世界 Y 轴方向划穿白圆圆心，产生心形尖部。

| 参数 | 值 | MSLA 依据 |
|------|-----|-----------|
| 奶缸高度 (液面上方) | 80mm | 抬高避免破坏图案 |
| 奶缸倾角 | 50° | 中速细流, 精确划穿 |
| 推进距离 (沿 Y 轴) | 15mm | 穿过圆心产生尖部 |
| 轨迹总长 | ~15mm | 仅推进距离 |

**运动模式**: 在成形高度(5mm)→重新抬高到 80mm, 奶缸从 sway_y 位置沿世界 Y 轴直线推进 15mm。奶流从高处落下，以细流穿过白圆圆心——奶泡流切断圆形的一侧边缘，产生心形的尖部。

**物理过程 (MSLA "The Off Switch")**:
1. 抬高奶缸增加高度 (降低流速的自然制动)
2. 沿 Y 轴推进，奶流穿过白圆圆心
3. 细流在圆心处切断白色圆形 → 圆形变成心形
4. 最后降低肘部 (或移开奶缸) 干净收尾，避免拖尾

**关键细节**:
- 划穿前必须抬高奶缸，低位划穿会破坏整个图案
- 推进速度与流速配合——太快或太慢都会导致尖部不对称
- "The Off Switch" — 收尾时降低肘部 (而非提高奶缸)，切断奶流干净利落

---

## 3. 机器人实现

### 3.1 坐标系约定

```
世界 X 轴 (前)  → 倾倒倾角轴: 绕此轴旋转使奶缸前倾(+)/后仰(-)
世界 Y 轴 (左)  → 划穿方向轴: 奶缸嘴沿此方向倒奶, cut-through 沿此方向
世界 Z 轴 (上)  → 高度轴: 液面上方距离
```

**关键**: 代码中所有 `roll` 指**绕世界坐标系 X 轴的倾角** (非 TCP body-fixed roll)。

### 3.2 倾角基准

step4 提供 45° 基准倾角 (绕世界 X 轴)。step5 各阶段在基准上叠加增量：

```
实际绝对倾角 = step4基准(45°) + rel_roll(roll_deg - 45°) = roll_deg

融合: roll=45°  → rel=0°    (基准位置)
成形: roll=60°  → rel=+15°  (额外前倾)
收尾: roll=50°  → rel=+5°   (微前倾)
```

`makeStagePose()` 中 `rel_roll = roll_deg - 45.0` 实现此增量计算。

### 3.3 阶段路点生成公式

`generateHeartStageWaypoints()` 分三个阶段生成离散路点（世界坐标系）：

#### 阶段1 — 融合画圈 (N = total_points × 25%)

```
progress = i / (N - 1)                              // [0, 1]
angle = 2π × mix_circles × progress                 // 2圈 × 2π
x = origin_x + mix_circle_r × cos(angle)            // 圆圈 X
y = sway_y + mix_circle_r × sin(angle)              // 圆圈 Y
z = origin_z + mix_height                           // 固定高度 80mm
roll = roll_mix (固定 45°)
```

#### 阶段2 — 成形注入 (N = total_points × 55%)

```
x = origin_x                                         // 固定不动
y = sway_y                                           // 固定不动
z = origin_z + draw_height                           // 固定高度 5mm
roll = roll_draw_start + progress × (roll_end − roll_start)  // 60°→45° 线性渐变
```

> **如果 `roll_draw_dynamic=false`**: roll 固定为 `roll_draw` (60°), 无渐变。

#### 阶段4 — 划穿收尾 (N = total_points × 20%)

```
x = origin_x                                         // 固定
y = sway_y + progress × push_y                      // sway_y → sway_y+15mm
z = origin_z + mix_height                           // 固定高度 80mm
roll = roll_finish (固定 50°)
```

> 阶段编号使用 1/2/4 (跳过 3)，预留 3 用于未来图案扩展。

### 3.4 执行流程

```
step5_executeLatte():
  1. 获取当前 TCP 位姿作为 origin
  2. 通知 trajectory_recorder 开始录制 (ROS service)
  3. [5a] makeStagePose(origin, mix_height, 45°, offset_y) → moveCartesianStraight 到位
          generateHeartStageWaypoints(origin, hp, z, 45°, stage=1) → executeCartesianPath 画圈
  4. [5b] makeStagePose(origin, draw_height, 60°, offset_y) → moveCartesianStraight (降高度+倾角)
          generateHeartStageWaypoints(origin, hp, z, 60°, stage=2) → executeCartesianPath 定点注入
  5. [5c] makeStagePose(origin, mix_height, 50°, offset_y) → moveCartesianStraight (升高度+倾角)
          generateHeartStageWaypoints(origin, hp, z, 50°, stage=4) → executeCartesianPath 划穿
  6. 通知 trajectory_recorder 停止录制
```

每段先用 `moveCartesianStraight()` (笛卡尔 slerp 过渡) 到达阶段起始位姿，再用 `executeCartesianPath()` 执行阶段内离散路点。

### 3.5 参数表 (lwf_heart_ 前缀)

| 参数 | 类型 | 默认值 | 说明 | MSLA 依据 |
|------|------|--------|------|-----------|
| `lwf_heart_mix_height` | double | 0.08 | 融合高度 (m) | MSLA 5.02: 7–10cm |
| `lwf_heart_draw_height` | double | 0.005 | 成形高度 (m) | MSLA 4.02: <1cm |
| `lwf_heart_mix_circle_r` | double | 0.010 | 画圈半径 (m) | 硬币大小 ~1cm |
| `lwf_heart_mix_circles` | double | 2.0 | 画圈圈数 | 2 圈建立清晰圆形 |
| `lwf_heart_push_y` | double | 0.015 | 划穿推进距离 (m) | 15mm 穿过圆心 |
| `lwf_heart_sway_offset_y` | double | 0.01 | Y 轴杯前偏移 (m) | 留划穿空间 |
| `lwf_heart_total_points` | int | 200 | 总路点数 | 25%+55%+20% 分配 |
| `lwf_heart_velocity` | double | 0.5 | 速度缩放因子 | 约 4–5s 类人节奏 |
| `lwf_heart_roll_mix` | double | 45.0 | 融合绝对倾角 (°) | 细流 ~10ml/s |
| `lwf_heart_roll_draw` | double | 60.0 | 成形绝对倾角 (°) | ~20ml/s 高流量 |
| `lwf_heart_roll_draw_start` | double | 60.0 | 成形起始倾角 (°) | 高流量建立白圆底 |
| `lwf_heart_roll_draw_end` | double | 45.0 | 成形结束倾角 (°) | 降低流量防溢出 |
| `lwf_heart_roll_finish` | double | 50.0 | 收尾绝对倾角 (°) | 中速细流 |
| `lwf_heart_roll_draw_dynamic` | bool | true | 成形是否渐变 | 模拟人类回正 |
| `lwf_heart_verbose` | bool | true | 打印阶段详情 | 调试用 |

---

## 4. 常见图案缺陷与原因分析

| 缺陷 | 症状 | 可能原因 | 修复方向 |
|------|------|---------|---------|
| **Piddles (飞溅)** | 图案表面有气泡、白色飞溅点 | 倾倒高度 >10cm，流线断裂 | 降低 `mix_height` |
| **Snail trails (蜗牛痕迹)** | 白色条纹褪色、对比度低 | 倾倒高度 <5cm，泡沫未穿透 crema | 提高 `mix_height` |
| **Mushroom cloud (蘑菇云)** | 图案底部出现模糊蘑菇形状 | 融合画圈不充分或流量波动 | 增加 `mix_circles` 或 `total_points` |
| **不对称心形** | 尖部偏左或偏右 | 划穿方向偏移或 origin 不在杯中心 | 检查 `sway_offset_y` 和 `push_y` |
| **尖部不明显** | 心形底部无尖，呈圆形 | 划穿收尾不足 | 增加 `push_y` 或调低 `roll_finish` |
| **白圆过小** | 白色面积不足 | 成形流量不够或时间太短 | 增加 `roll_draw` 或成形点数比例 |
| **溢出** | 白圆超出杯边缘 | 成形流量太大或注入时间太长 | 降低 `roll_draw` 或成形点数比例 |
| **条纹过多** | 图案中有不期望的线条 | 成形阶段奶缸移动 | 检查 stage=2 路点是否 Y 固定 |
| **拖尾** | 收尾后有长白线 | "The Off Switch" 失败 | 提高收尾时 `roll_finish` 让重力帮助切断 |
| **IK 求解失败** | 笛卡尔直线失败 | 动态 roll 导致每点位姿不同 | 设置 `roll_draw_dynamic:=false` |

---

## 5. 扩展: 进阶心形变体

### 5.1 Heart-in-Heart (MSLA 5.04)

在心形内部嵌套第二个心形。需要 **turbo 流量 (30–40 ml/s)** 产生涡流驱动的嵌套扩散。

实现要点:
- 第二次 push 推进距离更长 (嵌套白圆需要更多空间)
- 需要更高的倾角 (~70°) 来产生 turbo 流量
- 当前代码可通过增加第三个 stage (stage=3) 扩展支持

### 5.2 Tulip (郁金香) (MSLA 5.05)

多层心形堆叠。每个"花瓣"是一个独立的成形注入 → 划穿收尾子序列。

实现要点:
- 需要多轮 (成形+划穿) 循环
- 每次划穿后 sway_offset_y 递增 (下一层花瓣向前推进)
- 当前代码架构的 `generateHeartStageWaypoints` 可复用于每层花瓣

### 5.3 Rosetta (叶形) (MSLA 5.06)

需要侧向摆动 (wiggle) 产生叶脉效果。

实现要点:
- 需要 `wiggle_amp` 和 `wiggle_cycles` 参数 (已从 HeartParams 中移除)
- 融合画圈改为侧向蛇形路径
- 摆动频率与推进速度同步以避免图案扭曲

> 当前代码专注于心形。Rosetta 需要额外的 `RosettaParams` 结构和 `generateRosettaWaypoints()` 函数喵~

---

## 6. 参考资料

- [Barista Hustle MSLA — 牛奶科学与拉花艺术 (付费课程)](https://www.baristahustle.com/education-products/single-course-sales/course-milk-science-and-latte-art/)
  - MSLA 2.04: 微泡沫质量评估
  - MSLA 3.06: 蒸汽打奶参数
  - MSLA 4.02: Pin-Drop 倾倒技术
  - MSLA 4.04: 流速、线宽、高度关系
  - MSLA 5.02: 僧帽 (Monk's Head) 与心形规格
  - MSLA 5.04: 心形嵌套 (Heart-in-a-Heart)
  - MSLA 7.02: 拉花常见缺陷诊断
- SCA WLAC (World Latte Art Championship) — 竞赛评分标准与图案规范
- 项目源码: `latte_backend/src/latte_workflow_node.cpp`、`include/latte_backend/latte_workflow_node.h`

---

> 本文档基于 Barista Hustle MSLA 课程 (v7.x) 的理论框架，结合本项目机器人实现的具体参数和公式。课程内容为付费资源，此处仅保留与代码实现相关的理论摘要和参数映射关系喵~
