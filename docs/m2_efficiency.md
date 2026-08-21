# M2 效率方案（量化依据，待 M1 真机数据校准）

对标田间公开数据（苹果，非套袋桃，仅作吞吐上限参考）：

| 系统 | 单臂周期 | 成功率 |
|------|----------|--------|
| 负压吸附笛卡尔（IJABE 2025） | 4.83 s/果 | 83.65% |
| 模块化双臂 2025 收获季 | 7.53 s/臂周期 | 80.0%（1738 次） |
| 四臂系统单臂区 | 9.59 s/果 | 可见果 82.00% |

当前栈每果强制 4 视角 + `scan.time_budget_s=45`，真机相机约 2.4 fps。
下界粗算：4 次到位 + 每视点最多 2×`frame_wait_s`（新鲜观测+新重建帧）≈ 数十秒，
加上 Survey/Build finalize，实测上一轮观察段已到 19–37 s 且未出 TSDF。
目标吞吐 150 果/小时 ≈ 24 s/果（含移动），与「每果 4 视角重建」冲突。

## M2 改动（M1 验收通过后再做）

1. **min_views 自适应**：首颗果 4 视角建模型；同簇后续果在 `skip_observation`
   且精化 RMSE/inlier 仍过门时降到 1–2 视角或 0（只用感知锚点+上一簇 TSDF 先验）。
2. **簇级观察复用**：同一 `scene_epoch` 内共享扫描轨迹，不再每果重新 4 点环绕。
3. **相机帧率**：2.4 fps 使每视点等待偏长；M2 把 Percipio 拉到 ≥10 fps 或缩短
   `frame_wait_s` 上限（须有 M1 的 `stage_durations` 证明瓶颈在等帧而非规划）。
4. **ScanBudget**：`maximum_moves` 与 `min_effective_views` 随质量门提前收口，
   已 CONVERGED 立即停，不再用满 45 s。

M1 必须先给出：单果 `prepare/observe/finalize` 实测秒数、`skip_reasons` 分布、
`observe_build_view_race` 是否消失。没有这些数，不得下调 `min_views`。

2026-08-21 `field_full_20260821_1840_grasp_opt3`：观察段约 15–17 s 且已出 TSDF
（4/5 视角），尚未通过接触验收，**仍不得下调 `min_views`**。见 [field_test.md](field_test.md) §10。
