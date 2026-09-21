# 学习式立体匹配实测——GitHub SOTA 零样本验证（RAFT-Stereo / IGEV × 9 档全灭）

**日期** 2026-09-20 · **问题** better-algorithms.md §2 遗留的未验证假设："学习式（Selective-IGEV / RAFT-Stereo 等）对本机散斑 IR 的泛化是未验证假设，按三门协议先测再定"
**结论先行**：**零样本学习式立体匹配在散斑 IR 域被实测证伪**。9 档配置（2 模型族 × 2 权重域 × 2 分辨率 + realtime 变体）全部以 4.9~9.9 倍精度差撞毁谷底门；覆盖增益（+11~27pp）确系幻觉主导，且部分幻觉**落在抓取窗内**（比窗外鬼影更危险）。现行 SGBM（uniq=6 半分辨率）保持当日全部候选中的帕累托最优。

## 1. 仪器与验证（为什么可信）

- **评分仪器与 SGBM 扫描轮完全同一套**：`sweep_metrics.py` 三门（谷底 |ΔZ| ≤ 基线×1.10 / 鬼影带 1.5–2.5m 不升 / 孤立边轮廓位移 ≤2px），数据同源（/tmp/simul 3 组同帧组 + /tmp/sweep/base 基线）。
- **A 链 Python 复现先行验证**（`/tmp/lstereo/repro_a.py`）：D 链（calibD→彩色配准）**位级一致**（diffD=0，3/3 组）；A 链与 C++ gridprobe4 输出在评估窗内中位差 **0.000mm**、覆盖率差 ≤0.05pp、谷底 8.25/8.00/8.50 vs 基线 8.50mm——指标级等价（差异集中在深度边缘歧义像素，窗外）。
- **SDK 配准零重实现**：ctypes 直调 `libtyimgproc.so` 的 `TYMapDepthImageToColorCoordinate`（结构体 156 字节与 /tmp/calib_cache.bin 624=4×156 对账），与 gridprobe 位级同语义。
- 学习式视差 → Z=f_h·B/d → 0.25mm 量化 → SDK 配准，与 SGBM 候选走**逐字相同的尾部**（`run_model.py`）。

## 2. 候选与配置

| 模型 | 权重 | 理由 |
|------|------|------|
| [RAFT-Stereo](https://github.com/princeton-vl/RAFT-Stereo) | eth3d / middlebury / realtime | GitHub 零样本泛化口碑最好；realtime 变体是唯一可能上生产帧率的 |
| [IGEV](https://github.com/gangweix/IGEV) | eth3d / middlebury | better-algorithms.md 首选候选族的基座（Selective-IGEV 同源） |

- 输入：节点同几何校正对（前 8 畸变、stereoRectify alpha=0），灰度平铺 3 通道（两仓库训练同约定），半分辨率 640×480（与现行 SGBM 可比）+ 全分辨率 1280×960（IGEV max_disp=256）。
- 环境：yolo_env（torch 2.12.1+cu130，3090）；IGEV 需 timm==0.5.4（两仓库 env.sh 同钉），以符号链接法隔离进 `/tmp/lstereo/igev_env`，不动 yolo_env。

## 3. 结果（三门）

| 候选 | 覆盖% | 谷底 |ΔZ| mm | 鬼影% | 边缘 px | 推理 ms@3090 | 门 |
|------|-------|------------|-------|--------|------------|-----|
| **SGBM u6（现行）** | 47.28 | 9.00 | 1.63 | -1.0 | 14.2（CPU） | a·G⁻·e（当日赢家） |
| SGBM u10（base） | 44.55 | 8.50 | 1.45 | 0.0 | — | PASS |
| raft_eth3d_s50 | **71.37** | **62.75 (7.4×)** | 0.00 | -3.0 | 277 | ❌acc+edge |
| raft_midd_s50 | 71.17 | 84.25 (9.9×) | 0.00 | -3.0 | 283 | ❌acc+edge |
| raft_rt_s50 (realtime) | 70.81 | 63.25 (7.4×) | 0.00 | -2.5 | **28** | ❌acc+edge |
| raft_eth3d_s100 | 60.81 | 42.00 (4.9×) | 1.77 | -4.0 | 935 | ❌全灭 |
| raft_midd_s100 | 70.86 | 46.50 (5.5×) | 0.00 | -4.0 | 946 | ❌acc+edge |
| igev_eth3d_s50 | 56.01 | 80.00 (9.4×) | 2.52 | -4.0 | 259 | ❌全灭 |
| igev_midd_s50 | 60.17 | 53.00 (6.2×) | 2.51 | -5.0 | 255 | ❌全灭 |
| igev_eth3d_s100 | 55.19 | 30.00 (3.5×) | 3.82 | -3.0 | 891 | ❌全灭 |
| igev_midd_s100 | 57.65 | 29.00 (3.4×) | 2.98 | -3.0 | 895 | ❌全灭 |

要点：

1. **覆盖增益是幻觉**：+11~27pp 覆盖里，与设备链共同有效域的深度本身就对不上——不是"多测了"，是"编了"。
2. **幻觉进窗比鬼影更危险**：半分辨率 RAFT 鬼影门 0.00% 全过，因为编造面大多落在 0.3–1.5m **抓取窗内**（SGBM 的失败模式是窗外鬼影，感知窗天然过滤）。
3. **错误形态 = ~3% 深度尺度偏差 + 严重局部形变**：A/D 中位比值 1.026~1.036（SGBM 1.006）；共同域 |ΔZ| P25/P50/P75 = 19/66/172mm（raft_eth3d_s50）——即使最好的四分之一像素也差 19mm（基线全图中位 8.5mm 的 2.2 倍），P75 到 172mm 说明大面积表面扭曲，非刚体平移可解释。
4. **realtime 变体 28ms/帧（35fps、272MB）** 是唯一够生产帧率的学习式档——但精度同样全灭。速度优势在精度不成立时无意义。
5. 全分辨率让 IGEV 谷底从 80→30mm（改善 2.7×）但仍差基线 3.4 倍；方向上分辨率有帮助，域差距是主矛盾。

## 4. 结论与可行路径

1. **"GitHub 上有更优算法"在本机散斑 IR 域被零样本实测否定**（RAFT-Stereo/IGEV 两族、eth3d/middlebury 两域、半/全分辨率）。这与 [left-blind-band-theory.md] 的业界对照一致：散斑结构光 IR 不是自然图像域，零样本权重没有学过"散斑洞该判无效"。
2. 若仍要学习式前端，剩余路径（均非 drop-in，按代价排序）：
   - **域内微调**：以设备链 18-pattern 深度为伪 GT（同帧组现成），在散斑对上微调 RAFT-Stereo/IGEV；数据量需求与收益未评估。
   - **更大零样本基座**（FoundationStereo 级）：模型重、未证对散斑有效，期望值低。
   - **混合置信**：学习式只用于 SGBM 高置信区的亚像素细化（不做填充）——收益上限低。
3. **本轮再次自证三门仪器的价值**：任何单指标（覆盖率/视觉观感）都会把 71% 的 RAFT 判为"碾压"；只有设备链同帧组裁决揭穿。这是继 WLS、minDisp>0 之后第三个（也是最大规模的）"覆盖幻觉"实例。
4. 现行生产链不动：SGBM uniq=6 半分辨率 + avg_k 时域融合仍是全部已测候选的帕累托最优（覆盖/精度/帧率/算力零占用）。

## 5. 复现

| 件 | 路径 |
|----|------|
| A 链复现+验证脚本 | `/tmp/lstereo/repro_a.py`（D 链位级、A 窗内中位 0.000mm） |
| 模型运行器（env 驱动） | `/tmp/lstereo/run_model.py`（FAMILY/CKPT/TAG/SCALE/VARIANT/MAXDISP） |
| 评分 | `python3 /tmp/sweep_metrics.py /tmp/sweep/base /tmp/sweep/<tag>…` |
| 候选输出 | `/tmp/sweep/{raft_*,igev_*}/`（A/D PGM ×3 组） |
| 权重 | `/tmp/lstereo/weights/`（Google Drive 官方，gdown 经代理下载） |
| 仓库快照 | `/tmp/lstereo/RAFT-Stereo`、`/tmp/lstereo/IGEV`（depth-1） |
| IGEV 环境 | `/tmp/lstereo/igev_env`（yolo_env 符号链接 + timm 0.5.4，隔离不动原环境） |

注意 /tmp 易失；本报告表格数字已固化，脚本可随 gridprobe 系列重建。

## 6. 引用

- [RAFT-Stereo (CVPR 2022 X)](https://github.com/princeton-vl/RAFT-Stereo) · Lahner et al.
- [IGEV (CVPR 2023)](https://github.com/gangweix/IGEV) · Xu & Wang
- 前置：[sgbm-sweep.md](sgbm-sweep.md)（仪器定义）、[better-algorithms.md](better-algorithms.md)（本轮假设来源）
