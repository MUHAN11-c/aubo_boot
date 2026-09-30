// 心形轨迹参数（纯数据，零 ROS 图依赖）。
// 源自 aubo_boot latte_backend（2026-09-30 移植），参数权威 = Barista Hustle
// MSLA (Milk Science & Latte Art) Ch.4-5，逐项对照表见注释。
//
// 图案结构（MSLA Ch.5 — Heart = 双元素设计）：
//   融合画圈（僧帽白圆，r=1cm×2圈，z=80mm 高位注入）
//   + 划穿收尾（沿 Y 轴穿过圆心 15mm 直线推进） = ♥
//
// 坐标系约定：代码中所有 "roll" 指绕**世界坐标系 X 轴**的倾角（非 TCP
// body-fixed roll）。世界 X（前）= 倾倒倾角轴；Y（左）= 划穿方向轴；Z=高度轴。
// step4 已禁用，roll 从水平（0°）直接起算，无需叠加基准。
//
// 倾角-流量映射（MSLA 4.04：倾角越大→流速越高，线宽∝√流量）：
//   45° ~10ml/s 融合 mix 细流穿透 crema 沉底
//   50° ~15ml/s 收尾 finish 中速精准切割
//   60° ~20ml/s 成形 draw 高流量泡沫浮面扩散
#ifndef LATTE_BACKEND__LATTE_HEART_HPP_
#define LATTE_BACKEND__LATTE_HEART_HPP_

namespace latte_backend
{

struct HeartParams
{
  // ── 高度（MSLA 5.02: 融合 7-10cm, 4.02: 成形 <1cm；收尾复用 mix_height 简化）──
  double mix_height = 0.08;   // 融合高度（m），液面上方 8cm
  double draw_height = 0.005;  // 成形高度（m），液面上方 5mm，奶缸嘴紧贴液面

  // ── 融合画圈（心形 = 僧帽白圆 + 中轴划穿；心形不摆动，wiggle 仅 Rosetta 用）──
  double mix_circle_r = 0.010;  // 画圈半径（m），MSLA 推荐 ~1cm 硬币大小
  double mix_circles = 2.0;     // 画圈圈数，总路径 ~12.6cm

  // ── XY 运动（世界系）；sway 前移白圆留划穿空间 ──
  double push_y = 0.015;       // 划穿收尾 Y 轴推进距离（m），穿过圆心产生尖部
  double sway_offset_y = 0.01;  // Y 轴杯前偏移（m）

  // ── 时序（融合 25% + 成形 55% + 收尾 20%，MSLA 类人节奏 ~4-5s）──
  int total_points = 200;
  double velocity = 0.5;       // 拉花专用速度缩放（独立于 lwf_velocity）
  bool verbose = true;

  // ── Roll 剖面（绕世界 X 轴绝对倾角，从水平 0° 起算）──
  double roll_mix = 45.0;     // 融合倾角，~10ml/s 细流穿透
  double roll_draw = 60.0;    // 成形倾角 — 兼容别名，仅 roll_draw_dynamic=false 时使用
  double roll_finish = 50.0;  // 收尾倾角，~15ml/s 中速划穿

  // ── 成形阶段动态 Roll 渐变（模拟手腕 60°→45° 回正；roll_draw 不自动同步）──
  double roll_draw_start = 60.0;
  double roll_draw_end = 45.0;
  bool roll_draw_dynamic = true;
};

}  // namespace latte_backend

#endif  // LATTE_BACKEND__LATTE_HEART_HPP_
