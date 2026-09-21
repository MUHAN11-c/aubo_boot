# 点云质量轮（round4）——广域调研（相机原理/算法/大模型）+ 可落地质量提升实测

**日期** 2026-09-20 · **目标** 参考范围拉满（厂商实现、算法开源库、基础模型）找点云质量/精度提升手段，实测过三门后落地
**前置** sgbm-sweep（仪器与 uniq=6）、better-algorithms（WLS/全分辨率档）、learned-stereo-live-test（零样本学习式证伪）

## 0. 结论速览

1. **落地一项**：`median_ksize: 3`（深度有效值 3×3 中值，默认开）——三门全过（谷底 8.75 优于 9.00、鬼影 +0.04 门内、覆盖不变）、点云局部平面粗糙度 **−12%（1.17→1.03mm，已低于设备链 1.08mm）**、13.5 gps 无回归。Percipio 官方 Viewer 的后处理四件套里就有 Median Filtering——这是厂商认可的官方手段。
2. **新指标**：局部平面粗糙度（3×3 平面拟合残差中位，mm）进入仪器，与 librealsense DQT 的 Plane Fit RMS、Orbbec Spatial Precision、ISO 10360-13 平面拟合形状误差同族——厂商/标准/我们三方收敛。
3. **两个平滑类候选被精度门证伪**：联合双边（谷底 11.25，跨深度边缘平滑伤量测——WLS 教训第三次重现）；引导滤波（纹理拷入深度，粗糙度反而 4.5× 劣化——调研文献预警过的 texture-copy 陷阱实测命中）。
4. **大模型探针负结果**：DA-V2 Metric（Indoor/Outdoor Small-hf）在 0.3–1.5m 工作窗内对微距植物内容**无信号**（近桶 26m/中桶 20m 倒挂、ρ≈0.01）——"mono 基础深度当裁判"路径对本项目判负；与零样本立体证伪互证：**基础模型对"近距+植被"双重 OOD**。
5. **不改行为的重要认知**：librealsense 官方结论"平滑滤波应在视差域做"（Z 对 d 非线性）。我们的 avg_k（均值）在 Z 域是域偏差（存量债，默认 k=1 未生效）；而**中值是序统计量、与单调变换可交换，域无关**——本次选中值恰好绕开该坑。

## 1. 实测：滤波候选（3 组同帧对，三门 + 粗糙度）

| 变体 | 覆盖% | 谷底 mm | 鬼影% | 边缘 px | 粗糙度 mm（D=1.08） | 裁决 |
|------|-------|---------|-------|---------|---------------------|------|
| qnone（对照，uniq6） | 47.08 | 9.00 | 1.61 | -2.0 | 1.17 | 基线 |
| **med3（3×3 有效中值）** | 47.04 | **8.75** | 1.65 | +0.0 | **1.03** | ✅ 全过+改善 |
| med5（5×5） | 47.08 | 8.50 | 1.69 | -2.0 | 0.94 | ⚠️ 鬼影 +0.08 超门 |
| jb（5×5 联合双边，IR 引导） | 47.16 | 11.25 | 1.71 | -4.0 | 0.89 | ❌ 精度门 |
| gf（引导滤波 r=4, eps=1e-3 米域） | 47.83 | 79.50 | 2.10 | +1.0 | 5.33 | ❌ 灾难（纹理拷入） |

机理：中值保边（输出窗口内的真实观测值，不在深度边缘处造中间值），线性/核平滑类（双边、引导）在边缘两侧取加权平均——正是"跨边缘平滑伤量测"。med5 的谷底更好（8.50）但鬼影带 +0.08 超门，取 med3 的门内安全档。

## 2. 实测：大模型探针（mono 基础深度当"裁判"）

按 Stereo Anywhere / MonSter（CVPR 2025）的"mono 只当裁判不改量测"范式探针：DA-V2 Metric Indoor/Outdoor Small-hf（Apache-2.0，transformers 一行调用，33ms/帧@3090）→ 对设备链 D 中位 ratio 对齐 → 相关性与外点检出。

| 域 | 0.3–0.6m 桶 mono 中位 | 0.6–0.9m 桶 | 0.9–1.5m 桶 | ρ_near |
|----|----------------------|-------------|-------------|--------|
| Indoor（max 20m） | **26m** | **20m（倒挂）** | 21m | 0.010 |
| Outdoor（max 80m） | 同量级饱和 | — | — | 0.005 |

工作窗内连"近/远粗排序"都没有——0.3–0.9m 全部饱和到量程顶且非单调。**判负原因不是分辨率或对齐**，是基础模型训练域（室内房间尺度、非植被微距）双重 OOD。MoGe-2（MIT，点图范式）为唯一未测的第二意见，DA-V2 的倒挂桶表已足够定性；置信度链路（WLS confidenceMap / 时域 MAD → PointField）仍是低成本后续项。

## 3. 广域调研要点（三路，细节含 URL 由各子调研归档）

**相机原理/厂商栈**（librealsense / Orbbec / Percipio / Luxonis）：
- 滤波次序即语义：Decimation →(视差域) Spatial → Temporal →(回到深度域) HoleFilling；深度滤波在视差域做（[librealsense post-processing](https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md)、[BKMs 白皮书](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/RealSense_DepthPostProcessing.pdf)）。
- 两家厂商官方明言补洞=假数据：Orbbec HoleFilling 文档"填所有洞会引入假数据，一般不推荐"；librealsense hole filling 只是 4 邻域规则——与本项目 WLS 证伪完全一致。
- 质量指标收敛：fill rate / 平面拟合 RMS / Z accuracy / 时域噪声（[DQT](https://github.com/IntelRealSense/librealsense/blob/master/tools/depth-quality/readme.md)、[Orbbec 指标](https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation/Depth%20Quality%20Metrics)、ISO 10360-13:2021 / VDI 2634-3）；亚像素平面拟合 RMS <0.1px 为佳、>0.2px 应重标定。
- 物理层优先（零算法风险的最大杠杆）：曝光/增益（保最低）/激光功率双向扫描，过曝欠曝都劣化散斑对比度；δZ=Z²δd/(fB) 是总纲（[RealSense tuning](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance)、[Keselman CVPRW 2017](https://arxiv.org/abs/1705.05548)）。本仓 ir_exposure/laser_power 已是节点参数，标定轮可用"亚像素 RMS<0.1px"当验收。
- Percipio 官方（[doc.percipio.xyz](https://doc.percipio.xyz/cam/latest/index.html)）：Viewer 自带 Fill Hole / Remove Outlier / Time Domain / **Median Filtering** 四件套 + TY_INT_ANALOG_GAIN 等物理旋钮。

**算法开源**（详见子调研归档）：OpenCV ximgproc 全家（guided/JBF/FBS/WLS+confidenceMap）里适合"只去噪"的只有掩膜回写类；点云兜底 Open3D `remove_statistical_outlier(20,2.0)`/`remove_radius_outlier(16,0.05)` + voxel 2–4mm；多帧同视角：中值>均值（野值稳健且无效位保持无效），TSDF 交叉验证用（零交叉会补面）；学习式"纯去噪"无工业现成轮子（DeepDepthDenoising ICCV19 11ms 可作备选）。飞点机理与 RGB 引导校正（[IEEE MMSP 2024](https://arxiv.org/abs/2410.08084)）。

**基础模型**：DA-V2 metric 六变体 + DepthPro（Apple AMLR 非商用）/ MoGe-2（MIT，`Ruicheng/moge-2-vitl`，3090 fp16 ~60ms）/ Metric3D v2 / UniDepth v2 / Marigold-Lotus（相对深度）/ DUSt3R-MASt3R-VGGT（NC 许可）；"mono 裁判"文献支撑充分（[Stereo Anywhere](https://openaccess.thecvf.com/content/CVPR2025/papers/Bartolomei_Stereo_Anywhere_Robust_Zero-Shot_Deep_Stereo_Matching_Even_Where_CVPR_2025_paper.pdf)、[MonSter](https://github.com/Junda24/MonSter)、Guided Stereo Matching CVPR 2019）——文献成立，但本项目内容域判负（§2）。

## 4. 落地与验证

- `stereo_camera_node.cpp`：`median_ksize` 参数（0/3/5，非法拒启；默认 3）+ computeDepth 有效值中值（nth_element，无分配）；rectify 日志加 `med=%d`。
- `stereo_camera.yaml`：`median_ksize: 3` 注释指向本报告。
- 构建 ✅；lint **新增行零违规**（存量 4 项：copyright/include_order/2×行宽，经 stash 对照法确认为本轮之前已有）；uncrustify 0 diff。
- **live 冒烟** ✅：`med=3` 生效、13.5–13.6 gps（与 13.5 基线无回归）、点云/彩色话题在发（域 77，演示栈恢复运行）。
- 验证脚本（/tmp 易失，数字已固化）：`/tmp/lstereo/{quality_exp,mono_probe}.py`、`/tmp/sweep/q*_u6`。

## 5. 后续路线（按性价比）

1. **物理层标定轮**（田间/台架）：ir_exposure×analog_gain×laser_power 扫描，验收=亚像素平面拟合 RMS<0.1px（DQT 口径）；需连续采集帧组（当前仪器只存同帧组快照）。
2. **avg_k 视差域化**（存量域偏差，默认 k=1 未生效）：均值搬进视差域或改中值帧融合。
3. **置信度 PointField**：SGBM uniqueness + WLS confidenceMap（离线）或时域 MAD → 每点置信度，下游按阈裁点。
4. 点云 SOR/ROR 兜底（感知消费端，离线可接受）。
5. 域内微调学习式前端：维持 learned-stereo-live-test 结论（仅此一条严肃学习式路线）。
