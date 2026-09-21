# peach_stereo(hh4) vs percipio_camera（原驱动）端到端实测报告（终版）

**数据版本** 2026-09-21 重录终版（`test/` 精简归档；本轮录制链含 composite 全尺寸窗 + 自检门，视频**检测框/分割/标注清晰可见**）· **对照分析** [../analysis/percipio_vs_peach_stereo.md](../analysis/percipio_vs_peach_stereo.md) · **影响评估** [../analysis/peach_project_impact.md](../analysis/peach_project_impact.md)
**录制协议** 同场景同感知链（相机 → peach YOLO/SAM/圆柱拟合 → 全尺寸 composite 窗 + rviz 3D），两前端各 **75s 采集（hh4 145 帧 / percipio 146 帧）+ 30s×2 双窗视频**（composite=叠加图|彩色|深度JET 三联全尺寸 1920×480；rvizwin=3D 点云+圆柱标记）。场景=近距纸袋@0.55m+枝叶背景，两档每帧 3 检 1 确。录制前置自检门（check_frame.py：绿/橙框+红掩膜轮廓像素计数）不过门不录——本轮两档均一次过门。

## 0. 先答：target_1 的深度

**tid 是会话内轨迹号，不是稳定物理 ID**。09-21 重录轮 hh4 档曾同时确认 2 目标：`target_0`=近袋（掩膜中位 552.0mm），**`target_1`=远距小目标（半径 14.7mm）：掩膜深度中位 758.2±0.43mm、entry z 761.4±3.23mm**（103 帧；percipio 该轮只确认 1 个）。其余各轮两档各确认 1 个近袋但 tid 编号不同——跨轮/跨前端比较按物理目标对齐，勿按 tid。

## 1. 感知结果对比（本轮 145/146 帧）

| 指标（相机系，确认目标=近袋） | **peach_stereo hh4** | **percipio 原驱动** | 解读 |
|---|---|---|---|
| 检测/确认（每帧） | 3 / 1 | 3 / 1 | 一致 |
| mask_depth_ratio | **0.9977** | 0.9743 | hh4 掩膜内深度更满 |
| **entry std [mm] (x,y,z)** | **0.39 / 0.53 / 1.92** | 2.39 / 2.88 / 4.37 | **hh4 稳（x 6×、z 2.3×）** |
| 帧间跳变 \|Δ\|中位 [mm] | **0.30 / 0.52 / 1.53** | 1.08 / 1.64 / 2.31 | hh4 全面更平滑 |
| 袋半径 [mm] | 33.0 ±0.12 | 34.7 ±0.29 | hh4 一致性优 |
| 掩膜深度中位 [mm] | 551.0 ±0.00 | 550.0 ±0.12 | 两链目标深度差 1mm |
| entry 均值 [m] | (0.022, 0.148, 0.541) | (0.022, 0.154, 0.542) | 逐轴差 0.6/6.0/0.7mm，在 +8px≈10mm@0.6m 配准系统差内 |
| 窗内有效占比 | **0.5102** | 0.4646 | hh4 略优 |
| 场景深度中位 std [mm] | **0.44** | 0.83 | hh4 优 |
| det/seg/geom [ms] | 25/42/235 | 8/27/131 | 同卡同模型，≈ |
| 相机源帧率 | **13.6 gps** | ~1–2 fps | hh4 5.7×+（停走节拍决定项） |

## 2. 点云质量对比（目标掩膜内 + 空间覆盖与细节，frame_0141 量化）

| 指标 | hh4 | percipio | 解读 |
|---|---|---|---|
| 掩膜内点数 | 4556 | 4671 | 同量级 |
| 点密度 | **1.000** | 0.971 | hh4 目标掩膜内更满 |
| 局部平面粗糙度 | **0.33 mm** | 0.44 mm | hh4 目标表面更平滑（利于拟合） |
| 掩膜无窗内深度占比（全图） | 67.5% | **60.2%** | percipio 目标级覆盖更好 |
| **左缘 15% 列覆盖** | **0.000**（结构性全盲带） | 0.035 | **percipio 覆盖更好**：hh4 左缘视差搜索结构性零深 |
| **远端 1–1.5m 覆盖** | 0.017 | **0.024**（+40%） | **percipio 远端覆盖更好** |
| **细节密度（全有效 3×3 局部 std）** | 12.8 mm | **20.7 mm（+62%）** | **percipio 细节保留更好**（含真实细结构与少量噪声纹理；hh4 半分辨率+med3+tk3 链把细纹理抹平——换取拟合稳定性） |
| 全图有效占比 | **0.510** | 0.462 | hh4 中心区+时域中值补出更多有效像素 |
| 点云字段 | x/y/z@0/4/8、rgb@16、**confidence@20**、step24 | x/y/z、rgb@16、step20（无 confidence） |

## 2a. 避障链路（枝细结构，peach_vegetation × 深度四档）

`peach_vegetation` GPU 枝掩膜 × 各深度档（工具 `../scripts/branch_analysis.py`，图 `fig_branch_{depth,overlay}.png` + `branch/` 八件）：**hh4 现行档枝上覆盖最优（0.604/细枝 0.652，tk3 时域中值补细枝闪烁）**；关 med3+tk1 仅 +2% 细节 −3% 覆盖（负收益）；全分辨率 +18% 细节但覆盖 −18%、4.3gps、z_min≈0.53m 侵入工作区（坏交易）；percipio 细节密度最高（16.0 vs 12.9mm）但覆盖较低、~1–2fps。结论：**避障用 hh4 现行档不必改；细枝几何精度敏感的单帧场合用 percipio**。详见影响评估 §4a。

## 3. 图与视频（本目录，全部为本轮产物）

- `fig_f_compare.mp4`（30s，每前端=上 composite 三联全尺寸 + 下 rviz 3D，左右并排）+ 海报帧——**检测框/掩膜轮廓/ID 置信度/袋轴箭头全尺寸清晰可见**
- `{hh4,percipio2}_rvizwin.mp4`——rviz 3D 原始 30s 源视频（composite 源视频未随精简归档保留，`fig_f_compare.mp4` 即其合成终版；重录经 `../scripts/record_session.sh` 再生成）
- `fig_cyl.png` 圆柱拟合 · `fig_pcq.png` 点云质量 · `fig_a_debug.png` 检测标注（重建） · `fig_b_traces.png` 轨迹 · `fig_d_metrics.png` 指标 · `fig_e_rviz.png` composite 帧并排
- 数据 `../data/{hh4,percipio2}_frames.jsonl`

## 4. 结论

1. **hh4 的优势在"拟合稳定性+帧率"**：entry std、帧间跳变、密度、粗糙度全面占优且三轮方向稳定（z 3.6×/5.7×/2.3×），帧源 5.7×+——对停走节拍的圆柱拟合/抓取点是决定性的。门限口径见影响评估 §4：真实门=重建精配准体素 3mm（单帧 P95 4.5/9.0mm 均靠多视均值收敛）与 pregrasp 偏置 30mm（近袋 MAX 5.5/9.0mm，裕度 3.3–5.5×；**远目标 0.95m 上 hh4 P95 19.5/MAX 29mm 逼近 30mm 预算**）。
2. **percipio 的优势在"覆盖与细节"（用户图上观察，数据证实）**：左缘 15% 列 hh4 全盲（0.000 vs 0.035）、远端 1–1.5m 覆盖 +40%、细节密度 +62%（hh4 的半分辨率+med3+tk3 平滑链以细节损失换拟合稳定）。对依赖**单帧大范围覆盖、边缘细节、目标位于画面左缘**的场景，percipio 是更好的前端。
3. 两口径的"覆盖"要分开说：目标掩膜内密度 hh4 满（1.000），但空间分布上 percipio 更连片（无结构性左盲带、远端更多）。
4. 切前端采果前重做手眼标定（+8px≈10mm@0.6m 系统差）；用 hh4 时视点规划必须使目标居中（左盲带+细节损失是结构性代价）。

## 5. 录制链修复存档（本轮解决的"视频不对"问题）

上轮视频 rviz 内嵌 Image 面板被缩到 ~300px 宽且渲染偏暗，2px 框线/掩膜轮廓在视频里不可见（用户判"录制视频等都不对"——正确）。本轮修复：① collect.py 增 **E2E_GUI=1 全尺寸 composite 窗**（感知叠加图|彩色|深度JET 各 640×480 横排，ffmpeg x11grab 直录）；② **record_session.sh 一条链**（采集→后起 rviz→`check_frame.py` 截图自检门（绿/橙框+红掩膜轮廓像素计数，不过门拒绝录制）→30s 双窗并行录制→收尾 pgrep 复核）；③ make_fig_f 四源拼接（composite+rviz 每前端竖叠、两前端横排）。同日早前事件（confidence 字段重叠修复、挂死 recorder 堵流、XML 因果重测 0.053/0.466、叠加图修复）见 docs/testing-log.md 09-21 各条。

## 6. 复现

`../scripts/record_session.sh <tag> <秒>`（前端在跑前提下一条链：自检门+双窗录制）；`run_collect.sh`/`run_rviz_fixed.sh`/`run_percipio.sh depth_resolution:=640x480`（**必带分辨率**）；`run_analysis.sh`（统计+全部图件）。运行时中转 /tmp/e2e_live；bag 如需须 `timeout -s INT -k 10` 包裹。枝上深度质量（避障口径）另走 `../scripts/branch_analysis.py <profile> [n_frames]`（需 stereo 前端与 peach_vegetation 分割同跑；逐帧 JSON+SUMMARY 打 stdout——**跑前 tee 落盘**，产物图仅进 /tmp/e2e_live/branch/<profile>/）。
