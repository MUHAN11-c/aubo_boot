# reconstruction 验收结果（追加式执行记录）

每轮建模/条件改动后重跑三道门，在此追加一节。检测与分割仅是辅助证据，
不是照片级真实性证明；口径见 `../README.md` 验收节。

## 轮次 2026-09-28 M0 — 红项收口 + GPU 化 + 基线冻结

改动：`add_bag` 生成期 BVH 间隙校验（袋内果球按需收缩至 ≥0.5 mm，
`fruit_clearance_m` 逐袋入 `scene_manifest.json`）；参考袋维持"不虚构内果"
（旧盘上 `.blend` 为带参考果球的陈旧产物，其 `bag_4` -0.139 mm 穿袋报错
随重建消除）；Cycles 渲染切 OptiX（RTX 3090），后端记入 manifest。

- 几何门（`validate_geometry.py`）：**0 errors**。153 袋全部闭合（非流形边 0）；
  144 个推断树袋果球全部包容，最小间隙 0.84 mm；参考袋 9 个无内果。
  2,951 mesh 对象 / 270 万顶点；2438 条枝连接 error<1e-6。
- 感知门（`validate_perception.py`，conf 0.25、IoU.5 一对一）：

  | 机位 | 可见 GT | 检出 | 匹配 | recall | precision | SAM 框提示数 |
  |------|--------|------|------|--------|-----------|--------------|
  | reference | 23 | 9 | 9 | 0.391 | 1.00 | 16 |
  | detail | 23 | 12 | 9 | 0.391 | 0.75 | 16 |
  | orchard | 7 | 2 | 1 | 0.143 | 0.50 | 7 |

  远景小目标（<20 px 框）按口径剔除出近端验证但仍计入 GT 分母，
  reference/detail 的 0.39 recall 是当前防回归地板。
- 参考深度对照（reference 机位 Position 通道 vs PeachDataSet 1200 实深）：
  9 袋逐袋 MAE 均值 **13.5 mm**、最大 24.4 mm；source_mask_iou 0.41–0.86。
  内参为 FOV 近似（fx=fy=640），该 MAE 含建模近似而非纯渲染误差。
- 耗时：全量重建 + 三机位 48 samples 渲染 **40.7 s**（OptiX，此前 CPU 需十余分钟级）；
  感知评测 7 s。产物：`output/{geometry,perception}_validation.json`、
  `reference_depth_mm.png`（uint16 毫米）均落盘。

基线冻结：`baselines/perception_matrix_baseline.json`（M3 首轮矩阵后写入，
分层地板以本节三机位数字为初值）。

## 轮次 2026-09-28 M1 — blender_orchard 并入（观感贡献吸收，检测无回退）

改动：`measure_priors.py`/`make_textures.py` 迁入（产物与原目录逐字节
一致：5 张贴图 sha256 相同、priors.json 相同）；`crease()`（袋厚折痕，
warp 0.005）并入 `geometry.py`，推断树袋启用、参考袋保持无折痕（深度对
照锚纯净）；paper/bark/soil/grass 贴图细节并入 `materials.py`（打包进
.blend，保持无外链依赖）；`git rm blender_orchard/`。

- 材质裁定：paper.png **只作 bump**。颜色乘法（factor .5）把红分量相对
  提高、绿蓝压半，偏离数据集中位 (101,60,55) 的低饱和分布——reference
  机位实测匹配 9→7（丢 GT 1、5 两参考袋）；回退 bump-only 后 **9/9 恢复**
  （detail 9、orchard 1 与前一致）。
- 几何门维持 0 errors；重建 40–44 s（OptiX）。

## 轮次 2026-09-28 M2 — 室外条件受控变量 + 先验分布锚定

改动：`lighting.py` 四光照预设（noon 基线锚 / morning / late_afternoon /
overcast，Nishita Air/Dust 映射浑浊度）；`occlusion.py` 名义档
none/light/heavy（0/2/4 前景叶挂结果枝）+ heavy 袋前横枝 + ~15% 走廊枝
（对齐 L_insert 0.090）；袋宽/高/高宽比按 priors 分位逆采样、倾角按现场
实测分位（逐袋 `source_percentile` 入 manifest）；`--scale
validation|field`（field=3 行×8 株 393 袋锚定场）；crease 幅度按袋厚缩放
（先验 p10 窄袋厚 0.027 m，原幅度会把腔体压至 0.28 mm——生成期校验当场
拦截后修复）。

- 几何门 0 errors；153 袋（none/light/heavy 各 48 + 参考 9）、袋前横枝
  48、走廊枝 16、遮挡叶 288、连接枝 2438→2790。
- 三锚定机位（现行混合遮挡实现）：reference 8/25、detail 9/21、orchard
  0/18——数字变化是**刻意加难**的结果，防回归职责移交矩阵分层门。

## 轮次 2026-09-28 M3 — 多轮视角感知验证矩阵首轮（基线冻结）

场景：validation 档（153 袋，none/light/heavy 名义档各 48 + 参考 9）。
矩阵：光照 4 档 × 分层子集（每档 20 树袋 + 全部 9 参考袋 = 69 目标 × 3 视
序列 + 6 停靠）= **852 帧**，24 samples，OptiX 57.0 min（均 4.0 s/帧，
独占 GPU；双 Blender 并发会 OOM，须串行）。评测 4m54s（GT 按 IndexOB
跨光照缓存、SAM 仅 primary 视）。摘要入库 `output/matrix_{summary,report,
trajectory,render_times}.*`；帧本体 8.1 GB 在 `output/matrix/`（gitignore，
可重现）。

**分层 recall@0.35（近距视，覆盖率桶为主分层）**：

| 光照 | cov=low(<0.2) | cov=mid | cov=high(>0.5) | primary | supplemental | alley_stop |
|------|------|------|------|------|------|------|
| noon | 0.592 | 0.285 | 0.034 | 0.271 | 0.273 | 0.296 |
| morning | 0.569 | 0.275 | 0.035 | 0.265 | 0.263 | 0.259 |
| late_afternoon | 0.556 | 0.275 | 0.034 | 0.255 | 0.262 | 0.235 |
| overcast | 0.539 | 0.239 | 0.012 | 0.238 | 0.231 | 0.136 |

**袋级多视聚合（conf .35，单光照内 3 视序列，产线口径）**：
any_view none 0.825 / light 0.775 / heavy 0.537 / reference 0.944；
two_views 0.787 / 0.662 / 0.487 / 0.917。多视序列把单视 ~0.27 的
primary recall 拉到 0.78–0.83（无/轻遮挡）——多轮视角的价值被直接量化。

**结论与口径裁定**：
1. 遮挡是主导因素且随实测覆盖率单调（low→mid→high：.59→.28→.03）；
   名义档持续倒挂（none 实测 0.416 ≈ light 0.399 < heavy 0.516），
   自然冠层本底淹没受控布叶——评测主分层用深度反测覆盖率桶。
2. 光照为次要因素：overcast 全面低 8–15%（漫射下袋面对比度降），
   noon 最优；morning 顺光化后与 late_afternoon 相当。
3. SAM 框提示掩膜 IoU：reference 0.834 vs 树袋 0.49–0.65（褶皱+
   遮挡降低掩膜质量）。
4. 基线已冻结 `baselines/perception_matrix_baseline.json`（分层
   recall@0.35 + 多视聚合，容差 0.05），`--gate` 复验 PASS——后续每轮
   建模改动重跑矩阵对基线，回退超容差即 exit≠0。

field 锚定：`--scale field`（3 行×8 株 393 袋、6933 连接）整园图在
`output/field_anchor/`，为导航轮铺路。外观对照板
`output/appearance_board.jpg`（数据集实拍 vs 渲染上下两行）。
