# P0 重构追踪表（2026-09-23 起，依据 FINAL_PLAN + gap 分析 + 两会话结论）

| 批次 | 内容 | 状态 | 门 | 提交 |
|------|------|------|-----|------|
| 0 | baseline-inventory 快照+schema 守卫；harvester 209 重封 | ✅ | system_tests/r0_gate 绿 | 本笔 |
| 1 | F1 掩膜门松 intake（容差 0.08s+yaml 键+校验+5 单测）；D3 节流单位×4；D2 destroy 守卫；D1 随批次6 | ✅ | harvester pytest 绿+r0_gate | 本笔 |
| 2 | 轴向预算拆维度/常数迁 yaml/axial_structural/假门清理 | ⬜ | 纯核 pytest+回放塔 analytic 只升 | |
| 3 | ToolState 三轴/DI 接线/保守收口/grip-hold-release（fake-tool） | ⬜ | mock FULL 到 LEVEL_RETREAT_CONFIRMED | |
| 4 | 分级许可 CONTACT/TOOL；unrefined 袋径门 | ⬜ | 网格 ≥18/20 + deny 臂侧可验 | |
| 5 | 套入实测行程判据替换计时 | ⬜ | I6 过+停滞超时收口 | |
| 6 | 变体A 轨迹内嵌重建；先导 bag 实验 | ⬜ | e2e 墙钟缩短+质量不降 | |
| 7 | bond 开/join 有界化/ID-1+plan_id/文档收口 | ⬜ | 全门复绿 | |

## 前提更新（相对 FINAL_PLAN，2026-09-23 实况）

- 取消不等终态：**已修**（execution_guard 三通道有界执行，781971b）；残留=lifecycle 收口无限 join（批次 7）+ RetireBucket 滞留致服务降级（P3 记录）。
- v4d 轨迹已定版（PTP+垂直入冠+沿轴+sequence blend），FINAL_PLAN §6.4 描述与其一致，继承不重做。
- `peach_stereo` confidence@20/point_step=24 为 09-20/21 落仓事实，非本重构改动面。
- 回放塔基线 2026-09-23 已重封（43be561），后续批次以其为 analytic 下界。
