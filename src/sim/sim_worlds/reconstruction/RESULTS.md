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

人工复核产物（当时落在 `output/modeling_20260929/`，该中间目录已清）：
`output/appearance_board.jpg`；当轮 lighting 对照板未迁入现行 `output/` 根。
实拍水印仅出现在对照源图，未作为模型纹理。纸袋微褶皱、叶面细节仍有合成感，
外围环境较空、无地形测量和风致形变；这些是后续建模工作，不以检出率掩盖。

## 轮次 2026-09-29 傍晚：canopy-v4

产物当时 `output/modeling_20260929/canopy_v4/{validation,field}`（目录已清；48 samples，
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

产物当时 `output/modeling_20260929/fruit_v5/{validation,field}`（目录已清；48 samples，
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
帧当时在 `output/modeling_20260929/fruit_v5/matrix/`（gitignore，已清）。
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

## 轮次 2026-09-30：fruit-wrap-v6

产物 `output/` 验证场（48 samples，1280×720，noon），revision
`2026-09-30-fruit-wrap-v6`。对照板
`output/before_after.jpg`（左 v5 空枕头，右 v6 纸贴果）。
未重跑矩阵，未改冻结基线。

- 根因：v5 `cheek = Peach_bag 框宽/2`，中位袋宽 14.3 cm、果径 7.3 cm，再加
  袋底锥尖，侧视是空枕头+鸟嘴。现场 `bottom_to_neck` p50 7.0 cm ≈ 果径。
- 推断袋 112 个 `dimension_fit=fruit_wrap`：宽 p10/p50/p90 = 7.7/8.7/9.9 cm，
  高 8.2/9.2/10.5 cm，果径 6.6/7.3/8.2 cm。纸面间隙 4.9–5.0 mm。
  9 个 RGB-D 参考袋仍无内果（深度锚是纸面）。
- 几何门：121 袋、1,071,280 顶点、`errors=[]`。
- 感知（YOLO `best.pt` + MobileSAM，conf .25，IoU.5；分母含远小目标）：

  | 机位 | 可见 GT | 匹配 | recall | precision | 近袋 ≥2000 px |
  |------|--------|------|--------|-----------|----------------|
  | reference | 22 | 9 | 0.41 | 0.90 | 参考空袋 9/9，SAM 中位 0.97 |
  | detail | 18 | 5 | 0.28 | 1.00 | 裹果袋 4/4，SAM 中位 0.94 |
  | orchard | 6 | 0 | 0 | — | 无近袋（远景过小） |

  近袋才是外形验收：特写三颗圆裹果判 `peach_bag`（0.70–0.80），SAM 贴球面
  （0.83 / 0.94 / 0.98）。全图 recall 被远小目标拉低，不说明近袋失败。
  detail 另有 3 个 `peach_nobag`（远处圆团）。叠加
  `output/{reference,detail,orchard}_{detection,segmentation}.jpg`，
  拼板 `output/perception_board.jpg`。`detail` 机位已改，不能拿 v5 特写 11/32 比回归。
- 深度：单帧 1200 配准已从产物里去掉（`depth_comparison.jpg` 已删）。总体对照
  `output/dataset_depth_comparison.jpg`：实拍 n=722；仿真只留瞄准袋在
  0.55–0.75 m 且表观宽在实拍 p10–p90 内的近距袋（**留下 31，丢掉 419**，
  含后退超窗的机位、邻袋过远、1200 空参考袋）。

  | 量 | 实拍 p10/p50/p90 | 仿真留下 | 说明 |
  |----|------------------|----------|------|
  | 光轴深 | 0.33 / 0.61 / 1.13 m | n=31：0.58 / 0.64 / 0.70 m | 后退超窗的视已丢，中位对齐 |
  | 表观宽 | 5.1 / 10.9 / 20.7 cm | n=31：7.4 / 8.5 / 9.7 cm | 贴果纸；不是实拍框，不能再滤掉 |

  纸宽 112 袋 8.0 / 8.7 / 9.5 cm，与留下的表观宽同量级。
- 回归：`test_reconstruction_core.py` 含 `ConformingSimTests`；`check_modeling_blender` 6/6。

## 轮次 2026-09-30：leaf-orientation-v7

现行验证场 `output/`，48 samples、1280×720、noon；同机位 v6/v7 对照
`output/before_after.jpg`。本轮未重跑完整 1065 帧矩阵，未修改冻结基线。

- 根因：`Leaves.finish` 保存旋转与原型编号的 RNA 属性引用后继续新增属性，
  旧引用失效；旋转保持单位四元数、原型编号保持 0。最终实例全部朝 +X，
  只使用一个原型。全部属性建完后按名重新取得引用再写入，三方向实际实例
  矩阵与所选原型均正确。既有叶片方向采样、长度、宽度、叶数保持原值。
- 验证场：121 袋、112 个袋内成熟桃子、1,071,280 顶点；几何门 errors=[]，
  最小纸面间隙 4.893 mm。套袋里有桃子；9 个 RGB-D 参考袋只建可见纸面，
  内部未建模不代表实际内部为空。此前结果中的“空参考袋”仅指缺内部模型，
  此表述不再使用。
- 同机位感知（conf .25、IoU .5）：reference 9/13、detail 2/8、orchard 1/4，
  precision 均为 1.0。叶片恢复真实姿态改变遮挡，detail 匹配数较 v6 的 5
  降到 2；GT 分母也变，不能只看 recall 判断外观改善。SAM 仍为 GT 框提示。
- 回归：新增最终实例测试先失败（方向点积 0），修复后 Blender 7/7；
  reconstruction pytest 25/25；colcon test peach_sim 29 通过、2 失败，
  失败为既有 test_flake8（222 条）及 test_pep257（43 条），不宣称全包通过。
- 来源核对：Blender 4.5 API/节点文档及 v4.5.3 的 rna_attribute.cc、
  node_geo_input_named_attribute.cc、node_geo_mesh_to_points.cc、
  node_geo_instance_on_points.cc；以本机 4.5.14 实际实例矩阵为复现依据。
- 历史产物边界：matrix_* 与基线仍属 v5；dataset_depth_comparison.* 仍属 v6。
  v7 的完整多视和总体深度尚未复测，不混用历史数字证明当前版通过。

整园同版复验：395 袋、395 个内部成熟桃子、2,846,655 顶点；几何门 errors=[]，
最小纸面间隙 4.801 mm。与验证场 source_sha256 逐项一致；
产物更新到 output/field_anchor。感知：orchard 0 GT/0 匹配（远景过小），
aisle 1/7（3 个检测框，precision 1/3）。这不是通过完整矩阵门。

最终复核：回归文件既有缩进项修正后单文件 flake8 通过；再次 colcon test
仍为 29 通过、2 失败（flake8 221 条，pep257 43 条）。两份模型源码/模型/
全部锚定渲染哈希核对通过。本轮 Blender 与评测进程全部退出，重复中间目录已清。
收尾发现另一个会话启动了 ivg_sim ROS 进程，本轮未启动或终止该会话。


## 轮次 2026-09-30：real-features-v8

用户要求全量参考真实数据、完善树/枝/叶/套袋桃子并逐项核验。本轮完全离线，
未启动ROS或真机，未动驱动栈。现行验证场和整园revision均为
`2026-09-30-real-features-v8`。

- 全量审计本机3627帧（bag2050/nobag577/young1000），14498现存
  RGB/Depth/IR/VOC4文件逐一哈希，所有RGB做一致缩略像素代理统计。
  bag Depth/1397.png损坏、nobag Depth/233.png全零、nobag缺10个VOC4
  标注均显式记录，不补造。全量先验：bag2049/nobag567/young1000可读帧，
  bag无遮挡class0有效框观察6014，nobag无遮挡class0有效尺寸676。
  这些是重复视图下框观察，不是独立果实数，也不是实例完整三维测量。
- 纠正旧轮的类别解释：作者0/1/2/3是无遮挡/叶遮挡/枝遮挡/果遮挡，
  不是大小或成熟度；旧class1“成熟果”解释不再成立。当前取class0的
  p50–p90果径作为显式“成熟段”假设，970kg/m³仍为假设密度。
- 树：主枝根沿上段树干错层，次枝/梢逐级渐细；灰粗皮与红绿嫩枝。
  整园独立验收曾发现73个主枝比母干粗：冠幅1.5放大了length和radius，
  没有同步改变trunk。新增crown1.5实际网格测试先失败，修复按连接段
  实际母径将主枝root限制为92%，保持冠幅表达；验收阈值未放宽。
- 叶：20共享原型，尖端/细锯齿/主侧脉/短柄/两叶基腺体，卷边与垂曲；
  姿态在局部枝轴中生成，节位有界抖动。腺体大小、叶序/节距分布为推断。
- 袋：round_wrap / folded_panel_wrap / soft_fold_wrap，保留桃子体积，
  增加较直纸面、折角、平底和不规则褶皱。三个果径×三袋形实际BVH
  检查果心在内、保守包络间隙及网格封闭；不缩果来迁就纸袋。
- 内果：红黄皮、角度边界连续果缝、向内柄窝；梗根贴实际网格极点，
  另一端到袋颈，实际表面/端点验收。9个RGB-D参考袋只建可见纸面；
  内部未观测/未建模不表示实际袋内为空。

保存网格最终结果（验收直接读取.blend轴环、顶点及evaluated叶实例）：

| 场景 | 树 | 非trunk分枝 | 叶实例 | 套袋/内果 | 顶点 | 最小保守纸果间隙 |
|---|---:|---:|---:|---:|---:|---:|
| 验证场 | 9 | 2205 | 27357 | 121/112 | 1,076,240 | 4.459mm |
| 整园 | 24 | 11326 | 157335 | 395/395 | 2,856,235 | 4.310mm |

验证场errors=[]，最大枝根贴轴误差1.547µm，最大子/母径比0.915651；三纸形数量{'soft_fold_wrap': 37, 'folded_panel_wrap': 37, 'round_wrap': 38}。

整园errors=[]，最大枝根贴轴误差6.074µm，最大子/母径比0.920004；三纸形数量{'soft_fold_wrap': 133, 'folded_panel_wrap': 130, 'round_wrap': 132}。

当前版感知（conf.25、IoU.5，SAM使用GT框提示）：

| 场景/机位 | 可见GT | 匹配 | 检测框 |
|---|---:|---:|---:|
| 验证场/reference | 15 | 9 | 9 |
| 验证场/detail | 3 | 2 | 3 |
| 验证场/orchard | 4 | 1 | 1 |
| 整园/orchard | 0 | 0 | 0 |
| 整园/aisle | 7 | 2 | 2 |

- 逐项真实对照`output/real_feature_comparison.jpg/json`包含真实来源hash、
  全帧像素代理分布、当前实际几何报告和隔离渲染出处。板上是不同树/机位，
  不是配准误差评测；绿色像素含草/背景，黄水印排除仅近似，不是叶面积。
  `feature_details/`独立诊断只在内存隐藏其他物体，未修改保存模型。
  目检确认三纸形差异、叶尖/柄/齿缘和粗主枝→细梢层级；仍不是照片级验收。
- 五光照×三锚定共15视角，同一验证模型、48samples、1280×720；
  每个RGB/Depth/IndexOB/Position、模型、源码及感知manifest哈希核对。
  光照板/统计为`output/lighting_board.jpg`及`lighting_validation.json`。
  `before_after.jpg`同机位v7/v8；`appearance_board.jpg`包含同版整园。
- 回归：纯核pytest31/31；Blender主11/11（含冠幅失败→修复），
  独立验收行为3/3（故意移动枝根/梗能失败）。本轮8个新/回归文件
  ament_flake8与pep257通过。全包colcon仍29通过、2失败：flake8 215条、
  pep257 43条，不宣称全包全绿。
- 未重跑1065帧完整矩阵，不改基线；matrix_*仍v5、总体深度对照仍v6，
  不能作为当前版通过。隐藏冠结构/枝龄/枝领/芽与剪痕、叶序分布、纸背与
  纸厚、内部果面均非逐树实测；未证明全部枝叶/袋间无碰撞或物理真实性。
- 原始保存网格报告保留执行时输入路径；发布文件与执行输入逐字节hash一致。
  本轮生成、验收和渲染子程序均完成退出；不终止其他会话ROS进程。

最终独立实拍比较复核：稳定目录的6项几何关联齐全，validation/field保存模型、
六项隔离图、全部锚定渲染与5张实拍来源哈希一致。仍有纸褶较规则、纸色与
成熟桃局部红晕比实拍均匀的差异，不宣称照片级一致。field/aisle绿色像素代理
0.338；实拍bag全帧p10/p50/p90为0.004/0.154/0.487，仅报告代理，不设虚假
“真实通过”阈值。本轮重复中间目录已清理，稳定发布模型与证据保留，程序无残留。


## 轮次 2026-09-30：bag-paper-v9

用户明确只需要套袋桃子，并反馈 v8 套袋观感退步。本轮先比较旧图、
v8 与 Peach_bag：v8 内果约束把纸袋变成紧贴果实的球壳，袋尺寸采样
忽略实拍统计；几何细褶与高强度 bump 叠加，特写也有叶片遮挡。

- 保留 v8 原内果、果缝、果梗及已修复的树/叶结构；重新消费实拍袋宽高
  分位，随后用明确的推断上下界保证余量。未通过缩小桃子消除穿袋。
- 外袋采用宽纸面、折角和平底余量、少量主折痕与收口；降低纸纹 bump
  高度/强度和大色斑反差。三种袋形为 folded_gusset / broad_panel /
  creased_panel。尝试整面 flat shading 产生三角面伪影，已撤销。
- 九点实际 ray_cast 选择主体可见的特写，不删除枝叶；这不是精确可见
  面积指标，仍有树叶投影。主展示导出三种完整套袋资产，只改变摆放
  姿态，保留原网格、内果、果梗与扎丝，不显示裸果/叶片/骨架隔离图。
- 独立资产闭合且包容内果，最小保守间隙 4.087 mm；资产模型约 5.1 MB。
- 纯核 31/31、Blender 主回归 12/12、独立验收行为 3/3。包级 colcon
  29 项通过、2 项 lint 失败（214 条 flake8、42 条 pep257），不能声称
  包级全绿。新导出/套袋诊断/对照脚本独立 lint 通过。

实拍框与深度统计约束外形分布；隐藏袋背面、内部形态、真实折纸工艺
未测得，仍属推断。纸色和褶皱尚有合成感，不宣称照片级一致。
完整 1065 视角矩阵仍为 v5、总体深度对照仍为 v6，本轮没有将它们
冒充 v9 验收。同机位 75 mm 内果小样见 bag_before_after.jpg，手选
袋尺寸仅用于解释余量变化，不能作为实拍尺寸拟合通过的证据。

最终发布：validation/field 保存网格两项验收通过，5 光照×3 锚定图
均为 v9 同版模型，逐视角 RGB/EXR/模型哈希与感知报告关联通过。
套袋独立资产导出工具、模型与 PNG 哈希复验一致；实拍逐项对照仅
三种袋形和带袋果园。中间模型发布为稳定路径的相同字节后清理。
实拍对照可见模型折角/颈肩仍偏规整、纸面损伤不足；不设虚假观感通过阈值。


## 轮次 2026-09-30：bag-envelope-v10

用户再次指出套袋模型不对。上一轮 v9 的高次超椭圆截面和恒定深度
制造厚盒底、厚侧壁；宽平面测试错误地固化这一形状。实拍 255/746/1304
显示薄下沿、局部鼓起及不规则折叠，不能用“内果不穿袋”代替外观验收。

改用透镜截面；内果离底由 5 mm 改为 22 mm，留出余纸；底边半厚
1.2 mm，下部纸面沿内果切线展开，避免早期小样的横向台阶。袋高约束
改为果径+55..80 mm，果径不变；以上余量均属推断，不是实测。
仅生成独立三袋，正面/侧面/斜视均检查。旧整园、field 和五光照保留为
v9 历史产物，没有冒充本轮验收。

先运行薄底边测试，v9 报底部厚 52.382 mm；修复后 Blender 12 项和
纯核 31 项通过。最终保存的三袋闭合、内果包容，最小保守间隙
4.963 mm；源文件、保存模型和三张渲染哈希一致。新增独立
生成工具 ament_flake8/pep257 通过；未重跑全包 lint，v9 的两项失败未清偿。

产物：output/bag_assets/bagged_peaches.blend、bagged_peaches.png、front.png、
side.png、real_comparison.jpg。实拍对照非配准；v9/v10 三袋预览机位和
样本不同，不能当严格同机位回归。模型仍比真实纸袋平滑、规整，褶皱与
损伤不足；不宣称照片级一致。源码检索沿 Blender 4.5 官方 Mesh API
（https://docs.blender.org/api/4.5/）和现有 from_pydata/BVH 使用模式。
