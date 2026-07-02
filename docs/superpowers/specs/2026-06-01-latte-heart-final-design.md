# 心形拉花轨迹 — 最终设计文档

> 基于 Barista Hustle (MSLA全系列)、La Marzocco、Artisti、Fish River Roasters、Systema Coffee、Starbucks、FluidLab (ICLR 2023) 等权威来源
> 替代所有旧版设计文档。仅心形，其他图案（Rosetta/Tulip/Swan）删除。

---

## 1. 心形拉花动作分解

### 1.1 三阶段模型

```
阶段1 融合(35%): 高位7-10cm, 倾角45°, 画圈r≈5mm, 流速10ml/s
                 奶泡沉入crema下方 → 均匀棕色画布
                 填至杯 ½-⅔ 满
                 ↓ 不允许停顿(>4s奶泡分离)
阶段2 成形(35%): 低位<1cm贴杯沿, 倾角30°, 中心定点+微摆
                 流速20ml/s(3×加速), 白色奶泡浮面→圆形
                 填至杯 90% 满
                 ↓ 
阶段3 收尾(30%): 抬起→5cm, 倾角30°→60°, 细流拉线
                 快速(<1s)穿过圆心 → 心形尖部
```

### 1.2 动作来源对照

| 动作 | Barista Hustle | La Marzocco | Artisti | Fish River |
|------|---------------|-------------|---------|------------|
| 融合画圈 | — | 打圈倒入 | 圆形搅动 | 10分硬币大小圈 |
| 成形中心注入 | 定点中心 | 杯中心注入 | 保持正中 | 杯中心 |
| 抬起划穿 | 拉细流穿过 | 快速果断 | 直线穿过 | 穿过中心 |

---

## 2. 物理参数（多源交叉验证）

### 2.1 高度

| 阶段 | 人类标准 | 本项目值 | 来源 |
|------|---------|---------|------|
| 融合 | 7-10cm | **80mm** | Barista Hustle LA 2.02 / Starbucks |
| 成形 | <1cm 贴杯沿 | **5mm** | Barista Hustle MSLA 4.04 / latteartguide |
| 收尾 | 5-7cm | **80mm** (复用mix) | La Marzocco |

### 2.2 倾角（X轴 Roll）

| 阶段 | 倾角 | 流速 | 原理 |
|------|------|------|------|
| 融合 | **45°** | 10ml/s | 中等倾角+高位=奶泡穿透沉底 |
| 成形 | **30°** | 20ml/s | 低位+流量增大=白色奶泡浮面 |
| 收尾 | **60°** | 细流 | 抬高时加大倾角补偿重力拉散 |

> 线宽公式 (Barista Hustle MSLA 4.04): 流量 +100% → 线宽 +42%

### 2.3 纸杯规格

| 容量 | 上口径 | 备注 |
|------|--------|------|
| 8oz (240ml) | **80mm** | 标准拿铁杯 |
| 12oz (360ml) | **90mm** | 大杯 |

轨迹空间范围需在杯口内：X±35mm, Y±40mm（8oz杯）

---

## 3. 实现：5 阶段代码模型

```
阶段1 融合(25%): Z=+80mm  roll=45°  画圈r=5mm × 2.5圈
阶段T 下降(5%):  Z↓ 80→5mm  roll→30°  无摆动
阶段2 成形(30%): Z=+5mm  roll=30°  三角包络正弦微摆2mm
阶段3 抬起(10%): Z↑ 5→80mm  roll→60°  摆动衰减
阶段4 收尾(25%): Z=+80mm  roll=60°  中轴Y前推15mm划穿
```

### 3.1 所有参数（lwf_heart_* 前缀）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mix_height` | double | 0.08 | 融合高度 80mm |
| `draw_height` | double | 0.005 | 成形高度 5mm |
| `mix_circle_r` | double | 0.005 | 融合画圈半径 5mm |
| `mix_circles` | double | 2.5 | 融合画圈圈数 |
| `wiggle_amp` | double | 0.002 | 成形微摆振幅 2mm |
| `wiggle_cycles` | double | 3.0 | 成形微摆周期数 |
| `push_y` | double | 0.015 | 划穿Y轴距离 15mm |
| `sway_offset_y` | double | 0.01 | 杯前偏移 10mm |
| `total_points` | int | 200 | 总路点数 |
| `velocity` | double | 0.05 | 拉花专用速度缩放 |
| `verbose` | bool | true | 阶段详情日志 |
| `roll_mix` | double | 45.0 | 融合倾角(绝对) |
| `roll_draw` | double | 30.0 | 成形倾角(绝对) |
| `roll_finish` | double | 60.0 | 收尾倾角(绝对) |

### 3.2 Roll 计算规则

step4 已提供 X轴45° 基准。`roll_*` 参数为**总绝对倾角**。代码自动减去45°计算相对增量：

```cpp
double rel_roll = roll_deg - 45.0;  // step4 基准
auto q_roll = quat_from_x_rotation(rel_roll);
p.orientation = quat_mul(q_roll, origin.orientation);
```

| 阶段 | roll参数 | rel_roll | 总倾角 |
|------|---------|----------|--------|
| 融合 | 45° | 0° | 45° |
| 成形 | 30° | -15° | 30° |
| 收尾 | 60° | +15° | 60° |

---

## 4. 已修复的 Bug

| Bug | 根因 | 修复 |
|-----|------|------|
| fraction=0.004 | roll双叠（45°+45°=90°超出关节限位） | `rel_roll = roll-45°` |
| 笛卡尔绕行1.2m | eef_step=0.015跳过密集waypoints | `eef_step=0.002` |
| 融合阶段静止 | 缺少画圈动作 | Phase1 加画圈 r=5mm |

---

## 5. 参考文献

1. **Barista Hustle MSLA 4.04** — Flow Rate, Width & Height: 10ml/s填充, 20ml/s成形, 线宽+42%/流量+100%
2. **Barista Hustle LA 2.02** — Pin-Drop Technique: 高位>5cm, 最大15cm
3. **La Marzocco Home** — Pouring Latte Art Heart: 高位打圈→低位中心→抬起划穿
4. **Artisti** — The Essential Latte Art Heart: 三阶段模型（Mix/Drop/Cut）
5. **Fish River Roasters** — 3 Simple Steps: 融合画圈（10分硬币大小）
6. **Starbucks At Home** — How to Make a Latte Art Heart: 高位10-15cm
7. **FluidLab (Xian et al., ICLR 2023)** — 三角振幅包络 `A(i)=A_max*(1-|2i/N-1|)`
8. **Systema Coffee (May 2025)** — 90%问题来自距离太远
9. **Flair Espresso** — World Champion Emilee Bryant: 心形=定点注入+抬起划穿
10. **latte-pour-demos (ridxm)** — RM65双臂倒咖啡数据集: 倒奶段2.3s均值
