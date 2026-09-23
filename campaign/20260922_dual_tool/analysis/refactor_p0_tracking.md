# P0 重构追踪表（2026-09-23 起，依据 FINAL_PLAN + gap 分析 + 两会话结论）

| 批次 | 内容 | 状态 | 门 | 提交 |
|------|------|------|-----|------|
| 0 | baseline-inventory 快照+schema 守卫；harvester 209 重封 | ✅ | system_tests/r0_gate 绿 | 本笔 |
| 1 | F1 掩膜门松 intake（容差 0.08s+yaml 键+校验+5 单测）；D3 节流单位×4；D2 destroy 守卫；D1 随批次6 | ✅ | harvester pytest 绿+r0_gate | 本笔 |
| 2 | 轴向预算：sig_p 双喂拆开（横向 6mm 下限只归径向，轴向独立 MAD+3mm 下限）；七常数迁 yaml 带 provenance；axial_safety 与 fruit_safety 去重（0 可恢复对拍）；axial_structural 旗+专用 reason；假门清理移批次4 | ✅ | 预算/跨域 15 测全绿+r0_gate | 本笔 |
| 3 | ToolState.msg 三轴+manifest；ToolActuator 三轴投影+命令域新沿门（§12.2）+双门幂等；io_states 订阅→confirmFeedback 接线；VerifyCut 有界等沿（2.0s）+CUT_UNCONFIRMED 保守收口（不撤退不重发）；ReleasePayload@stow+commandToolOpen；6 gtest | ✅代码+单测 | 单测全绿；**mock 负路径 e2e（无 DI→不撤退）待批次4 网格轮顺带验证** | 本笔 |
| 4 | 分级许可 CONTACT/TOOL；unrefined 袋径门 | ⬜ | 网格 ≥18/20 + deny 臂侧可验 | |
| 5 | waitImuFollowTravel 重写：主判据=FK 沿轴实测行程（容差 5mm），回退=/imu_follow/insert_progress（目标积分，imu_follow 同轮暴露）；时间只作截止（名义+2s）；停滞窗 3s/<1mm 收口 UNKNOWN；撤退=反向投影同判据；insert_progress 纯核 5 测 | ✅代码+单测 | 纯核 5 测绿；I6 现场步需操作员 IMU enable（人工参考），待现场窗口 | 本笔 |
| 6 | 变体A 轨迹内嵌重建；先导 bag 实验 | ⬜ | e2e 墙钟缩短+质量不降 | |
| 7 | bond 开/join 有界化/ID-1+plan_id/文档收口 | ⬜ | 全门复绿 | |

## 前提更新（相对 FINAL_PLAN，2026-09-23 实况）

- 取消不等终态：**已修**（execution_guard 三通道有界执行，781971b）；残留=lifecycle 收口无限 join（批次 7）+ RetireBucket 滞留致服务降级（P3 记录）。
- v4d 轨迹已定版（PTP+垂直入冠+沿轴+sequence blend），FINAL_PLAN §6.4 描述与其一致，继承不重做。
- `peach_stereo` confidence@20/point_step=24 为 09-20/21 落仓事实，非本重构改动面。
- 回放塔基线 2026-09-23 已重封（43be561），后续批次以其为 analytic 下界。
