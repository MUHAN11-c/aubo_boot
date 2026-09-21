# 更优算法评估——本地数据实测 + GitHub/学术 SOTA 对照

**日期** 2026-09-20 · **问题** "参考 GitHub 等是否还有更优算法" · **前置** [sgbm-sweep.md](sgbm-sweep.md)（uniq 10→6 已落地）、[left-blind-band-theory.md](left-blind-band-theory.md)
**仪器** 同 sgbm-sweep（gridprobe4 真 SDK A/D 链，/tmp/simul 3 组同帧组，覆盖率主指标 + 精度/鬼影/轮廓三门）；gridprobe4 本轮扩展 `SWEEP_SCALE`（匹配分辨率）与 `SWEEP_WLS`（左右一致性+WLS 后滤波）与逐组计时

## 0. 结论速览

1. **经典域内已无"免费"的覆盖提升**：WLS 后滤波 +10~24pp 覆盖全是插值（精度 6.6× 劣化，被门拦截）——这是继 minDisp>0 之后第二个"覆盖幻觉"实例，三门仪器的价值再次自证。
2. **精度端存在真实更优档：全分辨率匹配**（`processing_scale: 1.0` + `num_disparities: 256`，均为已暴露参数，无需改代码）：Z 噪声 **−26%**（8.5→6.25mm，吻合 f 翻倍 → δZ 减半理论）、鬼影 **−33%**；代价覆盖 −5.8pp、纯匹配 82ms（≈12fps < 13.6 组率）。适合作"精度档"按需切换，不作默认。
3. **学习式立体匹配（GitHub SOTA）已被零样本实测证伪**（RAFT-Stereo/IGEV × eth3d/middlebury × 半/全分辨率共 9 档：谷底精度劣化 3.4~9.9 倍，覆盖增益是幻觉且部分落在抓取窗内）——散斑 IR 域差距是主矛盾，零样本权重不判散斑洞无效。剩余路径只有域内微调（设备链伪 GT）或更大零样本基座，均非 drop-in。详见 [learned-stereo-live-test.md](learned-stereo-live-test.md)。
4. 对照系不变：percipio 设备链（18 图案，2.43fps，8.3% 有效）在覆盖与帧率上全面劣于宿主 SGBM——"更优算法"的正确比较基线是宿主前端自身。

## 1. 本地实测（本轮新数据）

| 候选 | 注册网格窗内覆盖% | 谷底精度 mm | 鬼影% | 纯匹配 ms | 裁决 |
|------|------------------|-------------|-------|-----------|------|
| **u6 = uniq6@半分辨率（现行）** | **44.55** | 8.50 | 1.45 | 14.2 | 现行（覆盖端帕累托） |
| u6 + WLS(λ8000,σ1.5) | 54.89 (+10.3pp；z 网格 +24pp) | **56.50 (6.6×)** | 2.99 | 30.4 | ❌ 插值幻觉，精度/鬼影/轮廓三门全灭 |
| **full = 1.0 分辨率 + nd256** | 38.80 (−5.8pp) | **6.25 (−26%)** | **0.97 (−33%)** | 82.1（≈12fps） | ⚠️ 精度/鬼影最优档 |
| full + WLS | 51.60 | 56.00 | 3.92 | 155.9 | ❌ 同 WLS |

要点：
- **WLS 增益的定性**：WLS 用左右一致性把置信区视差沿彩色边缘外插到无纹理/遮挡区——填出来的"深度"在设备链有数据处对不上（谷底 56mm）。对**显示/补洞**有效，对**抓取量测**是幻觉。轮廓位移 −5px 也破门（外推改变深度边缘位置）。
- **全分辨率的得与失**：δZ=Z²/(fB)，f 翻倍 ⇒ 噪声减半，实测 −26% 吻合；鬼影降因为高分辨率下墙面错配更少。失在弱散斑纹理撑不起高分辨率匹配（覆盖降），且 82ms×（加 remap/量化/注册）撑不住 13.6gps 组率（≈12fps 纯匹配上限）。感知只吃 2Hz，**若走 avg_k 或按需单帧高清模式，12fps 不构成障碍**。
- 试法（零代码）：`stereo_camera.yaml` 改 `sgbm.processing_scale: 1.0` + `sgbm.num_disparities: 256` 即为精度档（参数已暴露；注意组率下降与内存）。

## 2. GitHub / 学术 SOTA 对照（学习式立体匹配）

| 方法 | 出处 | 特点 | 对本仓适用性 |
|------|------|------|--------------|
| **Selective-IGEV / Selective-RAFT**（CVPR 2024 Highlight） | [gangweix/Selective-IGEV](https://github.com/gangweix/Selective-IGEV)、[论文](https://openaccess.thecvf.com) | KITTI 2015 D1-all 上 Selective-RAFT 超 RAFT-Stereo 10.44%、Selective-IGEV 领先；有实时变体与预训练权重 | **首选候选**：精度领先 + 推理效率平衡；3090 上 640×480 预计 20~60ms |
| **RAFT-Stereo / 3D-RAFT** | [princeton-vl/RAFT-Stereo](https://github.com/princeton-vl/RAFT-Stereo)、[ETH3D 榜](https://www.eth3d.net) | 多层递归场变换；改进版实时推理；Middlebury 榜首（引用 915+） | 次选；跨域泛化口碑好 |
| **IGEV++**（TPAMI 2024 扩展） | [gangweix/IGEV-plusplus](https://github.com/gangweix/IGEV-plusplus) | 多距离几何编码体 + 稀疏补全，含实时变体 | 候选；稀疏补全思路与 avg_k 融合互补 |
| **HITNet** | [google-research/hitnet](https://github.com/google-research/hitnet) | 层次瓦片细化，实时（2025 年仍作实时基线，如 [GIP-Stereo](https://www.sciencedirect.com)） | 边缘细节好；权重生态较旧 |
| **FoundationStereo**（CVPR 2025, NVIDIA） | [Awesome-Deep-Stereo-Matching 收录](https://github.com) | 零样本 SOTA 但模型重 | 不适合 13.6fps 生产；可作离线裁判 |
| 散斑/IR 立体专题 | [IR Stereo Kinect](https://www.cs.cornell.edu)（结构光散斑 IR 双目）、[端到端散斑立体网络](https://ieeexplore.ieee.org)（IEEE 2024）、[深度立体综述](https://www.sciencedirect.com)、[双目+单目结构光组合](https://arxiv.org) | 证明散斑 IR 图可喂学习式匹配，但多为**域内训练/微调** | 风险点所在：零样本泛化到本机散斑未证 |

**风险与验证协议**（对任何学习式候选一致）：
- **零样本假设已实测**（同日续轮，见 [learned-stereo-live-test.md](learned-stereo-live-test.md)）：RAFT-Stereo 与 IGEV 的 eth3d/middlebury 权重在 3 组同帧对上 9 档全灭精度门（谷底 29~84mm vs 基线 8.5mm；~3% 深度尺度偏差 + 大面积局部形变；幻觉面多落在抓取窗内）。零样本路线就此关闭，后续只剩域内微调（设备链 18-pattern 伪 GT）与大基座两条重路线。
- 学习式模型在无纹理区倾向输出平滑"合理"面——正是 WLS/md16 式覆盖幻觉的高发区。**准入只认同帧组三门**：谷底 |ΔZ|（不劣化 10%）、鬼影带不升、轮廓位移 ≤2px；覆盖增益必须拆分"测得"与"补全"（对设备链 D 共同有效域内评估）。
- 仪器已就绪：/tmp/simul、/tmp/sdk3 帧组 + gridprobe4 + sweep_metrics.py + /tmp/lstereo/run_model.py 可对任意离线/在线视差图评分。
- 落地形态（若域内微调成立）：`camera_frontend` 缝位第二前端（与 percipio 并列，接口同构），torch 已在 venv（graspnet 同栈）、3090 空闲；RAFT-Stereo realtime 变体 28ms/帧@640×480 已证帧率可行。

## 3. 最终回答

**"是否还有更优算法"——有，但分层：**
- **今天就能用的更优档**：全分辨率精度档（参数切换，Z 噪声 −26%、鬼影 −33%），按场景需要启用；现行 uniq6 半分辨率仍是覆盖/帧率最优默认。
- **算法级的更优（学习式）已在同日实测中证伪零样本路线**：RAFT-Stereo/IGEV 9 档全部撞毁精度门（[learned-stereo-live-test.md](learned-stereo-live-test.md)）；域内微调是剩余的唯一严肃路线，代价与收益未评估。
- **三类"看起来更优"已被实测证伪**：视差窗平移（md16）、WLS 补洞、学习式零样本——它们的覆盖增益都是幻觉，这本身就是本轮最有复用价值的结论。

## 引用

- [Selective-IGEV (CVPR 2024)](https://github.com/gangweix/Selective-IGEV) · [arXiv 2403.07535](https://arxiv.org)
- [RAFT-Stereo](https://github.com/princeton-vl/RAFT-Stereo) · [ETH3D Leaderboard](https://www.eth3d.net)
- [IGEV++ (TPAMI 2024)](https://github.com/gangweix/IGEV-plusplus)
- [HITNet](https://github.com/google-research/hitnet) · [GIP-Stereo 2025](https://www.sciencedirect.com)
- [IR Stereo Kinect（散斑 IR 双目先例）](https://www.cs.cornell.edu) · [端到端散斑立体匹配网络（IEEE 2024）](https://ieeexplore.ieee.org) · [深度立体匹配综述（2022）](https://www.sciencedirect.com)
- [OpenCV disparity filtering (WLS) 文档](https://docs.opencv.org/4.x/d3/d14/classcv_1_1ximgproc_1_1DisparityWLSFilter.html)
