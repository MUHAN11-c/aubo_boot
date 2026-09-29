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

## 轮次 2026-09-29 树随机化 + 日光外观轮 — 矩阵复测与基线重冻结（121 袋场景）

场景变更（modeling_revision 2026-09-29-foliage-daylight-v1）：树结构去
克隆（干高 0.46–0.68 m、3 主枝为主偶 4、极角 42–58°、冠梢 20–32 根贴父
轴）、叶几何重做、材质/曝光重调（exposure -0.55→0）、光照预设体系重做
（turbidity→air/dust 密度 + cloud_cover + exposure_ev；**新增第 5 档
backlit 逆光困难组**，overcast 改真漫射关太阳）。袋 153→121（none 39 /
light 41 / heavy 32 / reference 9），连接 2760，几何门 0 errors；新增
`check_modeling_blender.py`（叶尖收口/冠根贴轴/overcast 无直射）3/3 过。

矩阵：5 光照 × 213 视（分层子集 20/档 + 9 参考，3 视序列 + 6 停靠）=
**1065 帧**，24 samples，OptiX 74.2 min（均 4.18 s/帧，独占 GPU 串行）；
评测近距行 1035 / 远眺 30。上午 10:15 的中断残帧（旧 blend）已清重跑。

**对 09-28 旧基线 --gate：FAIL（刻意，随即重冻结）**。失败格全部落在
名义遮挡档 level=light 与多视聚合；主分层（深度反测覆盖率桶）与
primary/supplemental 单视**全面改善**，reference 档 any_view 升满：

| 分层 recall@0.35 | 旧基线（noon） | 新（noon） | 旧→新（overcast） |
|------|------|------|------|
| cov=low / mid / high | .592/.285/.034 | **.817/.465/.106** | .539/.239/.012 → .797/.395/.081 |
| primary 单视 | .271 | **.360** | .238 → .316 |
| any_view none/light/heavy/ref | .825/.775/.537/.944 | .720/.480/.450/**1.0** | — |

口径裁定：名义档/多视聚合下降=**场景刻意加难**（标定：none 档实测覆盖
率均值 0.54、heavy 0.59，新随机树冠更密、袋位更深；与 M2"刻意加难"先例
同类）；同覆盖率桶下检测器看得更清（曝光/材质重调贡献）。旧基线对
新光照语义本不可比（turbidity→air/dust），按协议重冻结：
`baselines/perception_matrix_baseline.json` 50 格（含 backlit 首轮：
primary .346、cov low .822/mid .420/high .085——逆光未崩，袋面暗但有
对比），容差 0.05，与当轮 summary 逐格核对一致。

backlit 首轮结论：cov 桶与 noon 几乎持平（low .822 vs .817），逆光
困难组主要压低 mid/high 桶与 alley_stop；保留为长期压力档。

SAM 框提示掩膜 IoU：reference .776（旧 .834）、树袋 .44–.48（旧 .49–
.65）——新叶形/更密冠层下掩膜质量略降，不进门，跟踪观察。

产物：`output/matrix_{summary,report,trajectory,render_times}.*` 已拷贝
入库位；评审板重生成（lighting 板改 5 档、光照列表从 trajectory 单源
读取）`matrix_{lighting,occlusion}_board.jpg`。帧本体 ~10 GB 在
`output/matrix/`（gitignore，可重现）。全程离线，结束 pgrep 无残留。

## 轮次 2026-09-29 下午：地表细化与整园版本统一（groundcover-v2）

本轮优先 Blender 建模；未进行 ROS/Gazebo 导出或导航采摘验收。

- 草地由单片三角形改为弯曲的成簇草叶，使用独立固定种子，保留低矮通行带；
  加入贴地卷曲枯叶及低矮土块。均为推断环境；基础地面仍平坦。
- validation 保留实拍对齐局部：121 袋、2760 连接、4,386,201 顶点。
  field 改为纯 3 行×8 株推断桃树，移除不属于行列布局的局部参考树干：
  316 袋、6140 连接、11,397,093 顶点。整园默认相机改为能检查行列的高位总览。
- 两份 `.blend` 及清单源文件哈希一致，模型文件哈希核对通过。两场景几何门
  `errors=[]`；该门覆盖有限值、袋闭合及内部代理果包容，不覆盖全场枝叶碰撞，
  内部代理果尺寸也不是实测果径。
- 增加逐视角渲染上下文与 RGB/Depth/IndexOB/Position 文件哈希。感知验证先
  拒绝混光照、缺视角、旧图；记录 YOLO 与 MobileSAM 权重哈希。
  `render_saved.py` 即使未传 `--lighting` 也重新应用清单预设，避免清单被
  重渲染覆盖后与原 `.blend` 保存光照不一致。真实 Blender 重开回归：阴天
  显式指定与随后省略参数的 RGB 最大差异为 1/255（4 samples）。

同一 validation 模型：1280×720、48 samples、5 光照×3 视角，YOLO conf=0.25、
匹配 box IoU≥0.5。以下分母含背景小目标，不是只算前景 9 个参考袋：

| 光照 | reference 匹配/可见 GT | detail | orchard | reference 框提示 SAM 平均 IoU |
|---|---:|---:|---:|---:|
| noon | 8/18 | 9/18 | 0/14 | 0.583 |
| morning | 8/18 | 10/18 | 0/14 | 0.609 |
| late_afternoon | 8/18 | 10/18 | 1/14 | 0.604 |
| backlit | 8/18 | 9/18 | 1/14 | 0.595 |
| overcast | 8/18 | 10/18 | 0/14 | 0.601 |

SAM 使用 GT 框提示，不是检测到分割的端到端成绩。三锚定视角也不能替代完整
多视矩阵。本轮未重跑 1065 帧矩阵，未修改既有冻结基线；此前 matrix 报告属于
foliage-daylight-v1，不能作为 groundcover-v2 的完整矩阵验收。

验证：reconstruction pytest 20/20；Blender 叶尖/冠根/阴天回归 3/3；新增
来源校验、重渲染和对照板四个文件通过 ament_flake8。全包 pytest 的 flake8、
pep257 门仍失败，不能宣称全包测试通过。

人工复核产物：`output/modeling_20260929/before_after.jpg`、
`output/modeling_20260929/lighting_comparison.jpg`、`output/appearance_board.jpg`。
实拍水印仅出现在对照源图，未作为模型纹理。纸袋微褶皱、叶面细节仍有合成感，
外围环境较空、无地形测量和风致形变；这些是后续建模工作，不以检出率掩盖。

## 轮次 2026-09-29 傍晚：canopy-v4

产物 `output/modeling_20260929/canopy_v4/{validation,field}`（48 samples，
1280×720，noon）。本轮未重跑矩阵、未改冻结基线。

- 叶色：叶像素色度/明度 v3 0.67 → v4 0.50（validation 近景）/0.41（field
  作业道），实拍 0.36；中位亮度仍偏暗（G 79 vs 98），继续跟踪。
- 几何门：validation 121 袋、4,386,201 顶点、errors=[]（与 groundcover-v2
  顶点数一致，crown=1.0 树形未变）；field 395 袋、21,936,950 顶点、errors=[]。
- 感知（YOLO conf .25，IoU.5）：validation reference 8/19、detail 10/19、
  orchard 1/14；field aisle 9 可见 GT 中近距 3 袋全检出（IoU .89–.92），
  漏检均为 <1100 px 远袋；orchard 高位总览 0 GT（袋过小被过滤）。
- 坑：集合实例继承 pass_index，远景块复制袋会污染 IndexOB（一袋框横跨
  600 px）——远景块排除 pass_index>0 对象后修复。
- 回归：pytest 20/20；check_modeling_blender 3/3。

## 轮次 2026-09-29 傍晚：fruit-bag-v5

产物 `output/modeling_20260929/fruit_v5/{validation,field}`（48 samples，
1280×720，noon）。未重跑矩阵，未改冻结基线。

- 果：validation 112 颗、field 395 颗。横径 p10/p50/p90 = 6.7/7.3/8.1 cm
  （v4 代理果中位 3.9 cm）。质量 p50 约 0.19 kg（假设 970 kg/m³）。
  纸面间隙最小 1.7 mm。9 个实拍参考袋仍无内果。
- 袋宽 p50 14.3 cm：宽度分布在「装得下成熟果」处截断后的中位，高于套袋框
  p50 10.9 cm（后者含遮挡截断框）。
- 几何门 errors=[]。顶点 validation 1,014,519（v4 为 4,386,201），
  field 2,645,307（v4 为 21,936,950）。`.blend` 93 MB / 283 MB。
- 叶色：detail 叶像素中位 (69,88,62)、色度比 0.28；reference (63,84,56)、
  0.32。实拍 (72,98,62)、0.36。绿色通道仍低约 10。
- 感知（YOLO conf .25，IoU.5，分母含小目标）：validation reference 9/24
  （精度 1.0）、detail 11/32（精度 1.0）、orchard 0/25；field 作业道
  5/30，命中的都是 ≥1026 px 的近袋，漏检多为 ≤220 px。
- 回归：reconstruction pytest 22/22；check_modeling_blender 6/6。

## 轮次 2026-09-29 晚：fruit-bag-v5 矩阵验收

`run_matrix.py` 改为读取 blend 同目录的 `scene_manifest.json`，避免清单和几何不是同一场景。
矩阵用 `fruit_v5/validation/bagged_peach_orchard.blend`（revision
`2026-09-29-fruit-bag-v5`），`--samples 24 --subset-per-level 20`，与上一轮
冻结时的规模相同：5 光照 × 213 视 = 1065 帧，81.3 分钟，退出码 0。
帧在 `output/modeling_20260929/fruit_v5/matrix/`（gitignore，9.1 GB）。
摘要已拷到 `output/matrix_{summary,report,trajectory,render_times}.*`。
`output/` 验证场契约（blend、清单、三锚定图、几何/感知门产物）已换成这一版。

- 几何门：validation 121 袋、field 395 袋，`errors=[]`。
- 三锚定机位（YOLO conf .25，IoU.5）：reference 9/24、detail 11/32、orchard 0/25。
- 矩阵（conf .35）：noon primary 0.311，低覆盖 0.638 / 高覆盖 0.042；
  backlit primary 0.298。袋级多视（单光照 3 视）：reference 至少一视 1.00、
  至少两视 1.00；none 0.77/0.66，light 0.81/0.72，heavy 0.64/0.55。
- 旧基线属于 foliage-daylight，袋形和果径已变，不拿来比回归。按协议重冻结
  `baselines/perception_matrix_baseline.json`（50 格，容差 0.05），
  `--gate` 对当轮 summary 逐格通过。
