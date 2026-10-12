# peach_sim — Blender 数据驱动果园重建 + 多视角感知验证矩阵

当前套袋专项修正版为 **v10 独立资产**：
[三只套袋桃子](reconstruction/output/bag_assets/bagged_peaches.png)、
[正面](reconstruction/output/bag_assets/front.png)、
[侧面](reconstruction/output/bag_assets/side.png)、
[Blender 模型](reconstruction/output/bag_assets/bagged_peaches.blend)。
袋底改为薄封边，果实上移以保留余纸；袋面仅在内果附近鼓起，侧边压扁。
`build_bag_assets.py` 直接复用果园的建袋函数和实拍统计，确定性生成三袋。
根目录整园、field、五光照和旧对照仍是 **v9 历史产物**，未用作 v10 验收。
工作区根目录重现：
```bash
CUDA_VISIBLE_DEVICES=0 _tools/blender-4.5.14-linux-x64/blender -b -t 6 \
  --python-exit-code 1 -P src/peach_sim/reconstruction/build_bag_assets.py
```

外观建模与感知验证的唯一入口是 **`reconstruction/`**（2026-09-24 重建，
2026-09-28 合并根目录 `blender_orchard/` 观感链并升级为受控条件矩阵）。
旧 Gazebo 入口仍在仓库中，但不是本次重建场景，也不代表通过真实感验收。

```
peach_sim/
  reconstruction/          # 现行 Blender 外观 + 感知门（离线，不进 harvest_system）
    evidence/              # 先验 JSON + 1200 深度/掩膜锚 + 对照 contact sheet
    output/                # 现行契约：三锚定图、门 JSON、对照板、matrix_* 摘要
    baselines/             # 冻结感知矩阵基线
  peach_sim/ config/ launch/ urdf/ worlds/ meshes/ maps/ blender/
                           # Gazebo Harmonic 场景包：锚定建模 + 自主导航轮
                           # （与 Blender 感知验收不是同一份 GT）
```

`output/modeling_*/`、`matrix*` 帧、EXR、`.blend` 本机可重渲，不入库。

## 工具链（全部离线前台，完成即退出，不跑 ROS、不动真机）

| 脚本 | 职责 | 主要产物 |
|------|------|----------|
| `measure_priors.py --full` | 全部可用 VOC4 框与有效深度分位统计 + 现场袋轴实测 | `evidence/priors.json` |
| `make_textures.py` | 程序化贴图（纸/叶/皮/土/草，确定性 seed） | `textures/*.png` |
| `audit_real_features.py` | 全部 3627 帧 RGB/Depth/IR/VOC4 文件完整性、哈希和像素代理统计；坏数据显式记录 | `evidence/real_feature_audit.json`、contact sheet |
| `validate_real_features.py` | 保存网格的母子枝径、根连通、实际叶实例、袋形、果缝与果梗验收 | `output/real_feature_validation.json` |
| `render_feature_details.py` / `compare_real_features.py` | 仅三种套袋外观与真实套袋逐项对照 | `output/feature_details/`、`real_feature_comparison.*` |
| `audit_sources.py` / `analyze_reference.py` | 数据集审计 + 6 帧逐袋测量 | `evidence/*` |
| `build_scene.py` | 建场（参考重建 + 推断果园 + 受控遮挡），渲三锚定机位 | `output/*.png`、EXR、`scene_manifest.json`、`.blend` |
| `export_bag_assets.py` | 从保存场景导出三种完整套袋资产，保留网格与内果 | `output/bag_assets/` |
| `validate_geometry.py` | 几何门：袋闭合 + 果包容（errors 非空即 raise） | `output/geometry_validation.json` |
| `validate_perception.py` | 三锚定机位感知门：YOLO/SAM 对实例 GT；1200 配准 MAE 只作单帧诊断 | `output/perception_validation.json` |
| `compare_dataset_depth.py` | Peach_bag 总体 vs 符合实拍 p10–p90 的仿真近距袋；不符合的视/袋丢掉 | `output/dataset_depth_comparison.jpg` |
| `viewpoints.py` | 停走轨迹纯核：survey 停靠 + 每目标近距拍照位 + 补视链 | 被 `run_matrix.py` 消费 |
| `run_matrix.py` | 矩阵渲染：已建 `.blend` × 光照预设 × 轨迹视角（含 ray_cast 冠外取景修正） | `output/matrix/<光照>/<视>/` |
| `evaluate_matrix.py` | 矩阵评测：分层聚合 + 深度合成 `depth_mm.png` + 基线回归门 | `output/matrix/{summary.json,report.md}` |
| `make_appearance_board.py` | 真实数据 vs 渲染对照板（人工 QA） | `output/appearance_board.jpg` |
| `render_evidence.py` | 每视角记录模型/光照/采样上下文及 RGB、三种 EXR 哈希，拒绝混用旧图 | `scene_manifest.json` 的 `rendered_views` |
| `make_review_boards.py` | 矩阵帧人工复核板：同停靠视角全光照对比（2 列网格，档单源读 trajectory）+ 四遮挡档各一 primary 近距（1×N，noon） | `output/matrix_{lighting,occlusion}_board.jpg` |
| `check_modeling_blender.py` | Blender 无渲染几何回归（`blender -b --python-exit-code 1 -P`）：叶实例方向/原型、整园冠幅母子枝径、闭袋包果、果缝/果梗、叶尖与 overcast 无直射太阳 | stdout（失败非零退出） |
| `lighting.py` / `occlusion.py` / `distributions.py` / `depth_io.py` | 纯核：光照预设 / 受控遮挡 / 分位采样 / 深度量化 | — |

纯核单测：`test_reconstruction_core.py`（含与
`src/peach/peach_harvester/peach_harvester/cycle_core/view_policy.py` 的补视几何对拍）、`test_measurement.py`、
`test_render_evidence.py`、`test_full_priors.py`、`test_real_features.py`、
`test_observed_foliage.py`（当前33项）。
Blender主回归13项，独立验收行为3项；两种规模保存网格均另外完整复验。

## 重现（工作区根目录，独立 Blender 4.5.14，venv `aubo_py3.12`）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

aubo_py3.12/bin/python src/peach_sim/reconstruction/measure_priors.py --full # 数据变了才需要
aubo_py3.12/bin/python src/peach_sim/reconstruction/make_textures.py      # 贴图确定性重建
aubo_py3.12/bin/python -m pytest src/peach_sim/reconstruction/ -q         # 纯核单测

_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/build_scene.py -- --view all --samples 48 --width 1280
_tools/blender-4.5.14-linux-x64/blender -b \
  src/peach_sim/reconstruction/output/bagged_peach_orchard.blend -t 4 \
  --python-exit-code 1 -P src/peach_sim/reconstruction/validate_geometry.py
aubo_py3.12/bin/python src/peach_sim/reconstruction/validate_perception.py
_tools/blender-4.5.14-linux-x64/blender -b -t 4 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/validate_real_features.py -- \
  src/peach_sim/reconstruction/output/bagged_peach_orchard.blend
# 全量数据审计，坏文件保留哈希与错误，不填造深度或标注
aubo_py3.12/bin/python src/peach_sim/reconstruction/audit_real_features.py

# 总体深度/尺度（不是单帧 1200）：noon wrap primary ×40，再直方图
# 双卡 OptiX BVH 会挂死；_enable_gpu 已改为一卡，仍建议 CUDA_VISIBLE_DEVICES=0
CUDA_VISIBLE_DEVICES=0 _tools/blender-4.5.14-linux-x64/blender -b -t 8 \
  --python-exit-code 1 -P src/peach_sim/reconstruction/run_matrix.py -- \
  --samples 8 --lightings noon --view-kinds primary --skip-reference \
  --max-views 40 --out src/peach_sim/reconstruction/output/depth_pop
aubo_py3.12/bin/python src/peach_sim/reconstruction/compare_dataset_depth.py

# 多视角感知验证矩阵（约 1 小时，RTX 3090 OptiX）
_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/run_matrix.py -- --samples 24 --subset-per-level 20
aubo_py3.12/bin/python src/peach_sim/reconstruction/evaluate_matrix.py \
  --matrix src/peach_sim/reconstruction/output/matrix --gate     # 或 --write-baseline 冻结

# 导航轮铺路：field 规模锚定（3 行×8 株成熟冠 + 远景行列；整园 + 作业道两图）
_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/build_scene.py -- --scale field \
  --samples 48 --width 1280 --out src/peach_sim/reconstruction/output/field_anchor
aubo_py3.12/bin/python src/peach_sim/reconstruction/validate_perception.py \
  --out src/peach_sim/reconstruction/output/field_anchor --views orchard,aisle
```

`output/` 根目录是现行验证场契约（三锚定 PNG + 检测/分割叠加 + manifest +
双门 JSON + `matrix_*` 摘要 + `appearance_board.jpg` /
`dataset_depth_comparison.jpg` + `before_after.jpg`）。矩阵帧、EXR、`.blend`、
`modeling_*/` 中间轮次、`field_anchor/`、`depth_pop/` 不入库，本机也不堆积
（重渲即可）。field 规模场景须显式传 `--out .../field_anchor`（`--out`
默认恒为 `output/`，不随 `--scale` 切换，上文示例即显式传参）。只改相机与
采样可用 `render_saved.py`。
`field` 场景只含 3 行×8 株推断桃树，不插入实拍对齐的局部树干；该参考局部
仅保留在 `validation` 场景。field 树用成熟冠幅（`SCALES['field']['crown']=1.5`，
株距 3 m 下行内冠层交接成树篱，树高仍 ≤2.5 m），并以无袋集合实例把行列
延到地平线（远景不含 IndexOB 目标）；validation 冠幅 1.0 与矩阵基线同构。
推断树上的袋内果取裸桃框宽的成熟段（p50–p90，约 6.7–7.9 cm），
套袋内有桃子；v9 三种推断纸形为折角袋、宽面袋和带主折痕纸袋，袋口收拢扎丝。
袋宽/高重新消费 Peach_bag 实拍分位采样，再按内果约束限制：宽为果径的
1.25–1.65 倍，高比果径多 5.5–8 cm。这些上下界是建模假设，不是现场测量。
宽面、底部余量与稀疏折痕替代 v8 贴果球壳，纸面 bump 强度和高度降低。
旧场景导出工具会导出该场景自己的版本，不能用来覆盖当前 v10 独立资产。
保留内部果实、果梗和扎丝的旧场景导出命令（另存目录）：
```bash
_tools/blender-4.5.14-linux-x64/blender -b \
  src/peach_sim/reconstruction/output/bagged_peach_orchard.blend \
  --python-exit-code 1 -P src/peach_sim/reconstruction/export_bag_assets.py -- \
  --out src/peach_sim/reconstruction/output/orchard_bag_export
```
同机位 v8/v9 小样对照为
`output/bag_before_after.jpg`；整树 `before_after.jpg` 的 detail 已改取景，不能当同机位回归。
特写用实际 ray_cast 的九点可见度选择完整袋面，仍可能有真实枝叶投影，
九点代理不等于可见面积比例。
果径来自无遮挡 class 0 的上半分位；“成熟段”是建模假设，不是数据集标签。
9 个实拍参考袋只建可观测纸面，未建内部桃子；这不表示实际袋内为空。
叶片是 20 个共享网格的几何节点实例。2026-09-30 v7 修复新增属性使旧 RNA
引用失效的问题，旋转与原型编号现在按记录生效，消除全体叶片同向平铺。当前实际结构、叶实例和袋形计数见 `real_feature_validation.json`。
地表新增成簇弯曲草叶、枯叶与低矮土块，基础地面仍平坦，不能作为实测地形
或已验证的物理碰撞模型。
`render_saved.py --view all --lighting backlit --out <独立目录>` 可比较相同模型
的光照；每次都重新应用声明的光照预设。旧图没有逐视角哈希时须重新渲染，
`validate_perception.py --out <目录>` 会拒绝缺失或混版图像。

现行验证场与整园采用 `2026-09-30-real-features-v8`。全量真实数据审计、
实际网格验收与三锚定感知均单独留证；五光照三锚定只作当前版有限对照。
`matrix_*` 摘要与冻结基线仍属 v5，`dataset_depth_comparison.*` 仍属 v6，
这些历史结果不作为 v8 验收。`before_after.jpg` 为同机位 v7/v8 对照。

| 真实可见特征 / 依据 | 当前表达 | 核验与边界 |
|---|---|---|
| 短干、开心形主枝、粗木轴与细侧枝 | 主枝根沿上段树干错开；次枝、细梢渐细；灰粗皮与红绿嫩枝 | 保存网格重建轴线、母子枝径与根连通图；隐藏布局和枝龄推断 |
| 披针尖叶、锯齿、中脉、侧脉、叶基腺体 | 20 个共享叶型；不同朝向、卷边与垂曲；沿局部枝轴互生 | evaluated 实例核验方向、原型、根位置；腺体尺寸与节距分布未实测 |
| 红褐纸袋、宽纸面、折角、不规则袋底、收拢袋口 | 三种纸形与褶皱、倾斜、扎丝 | 各袋闭合且包住内部桃子；折痕尺寸、背面与纸厚推断 |
| 红黄果皮、桃缝、柄窝、果梗 | 每个推断套袋建内部桃子，果缝跨角度边界正确生成，果梗连实际柄窝与袋颈 | 网格包容和果梗两端测距；袋内表面并非实拍透视测量 |
| 叶遮挡、枝遮挡、不同光照、冠层空隙 | 自然枝叶 + 受控叶/横枝遮挡，五种光照预设 | 真实逐项定性对照 + 三锚定 RGB/深度/IndexOB；不是照片级真实性门 |

实拍/模型逐项板：`output/real_feature_comparison.jpg`；出处、原文件哈希、
全帧像素代理分布和模型几何报告关联见同名 JSON。绿色像素含草和背景，
黄水印排除仅是保守代理，不能当叶面积或实例数。对照不是同树同机位配准，
不以“颜色相近”宣称完整真实。当前目检仍有纸褶较规则、纸色与桃红晕较均匀
的差异；这些差异不由几何通过或检测成功消除。

## 数据依据与可信边界

`/home/mu/Downloads/PeachDataSet`：2,050 组套袋 / 577 裸桃 / 1,000 幼桃。
`evidence/priors.json` 为袋宽/高/高宽比/深度/最近距的 p10–p90 分位摘要
（套袋无遮挡 class 0 有效框观察 n=6014；重复视图，非独立果实）与现场 14 袋轴实测（倾角 p50 23.8°、袋底世界高 ~1.34 m）。推断树
的袋尺寸/倾角按分位逆采样，逐袋记录 `source_percentile`；纸色中位
(99.25,58.25,54)。原始图只做统计与对照，**不作为贴图**。
全量现存文件 14498 个逐一哈希，RGB 3627 帧；bag/Depth/1397.png 损坏，
nobag/Depth/233.png 全零，nobag 缺 10 个 VOC4 标注，均留证不补造。
作者官方类别 0/1/2/3 分别为无遮挡/叶遮挡/枝遮挡/果遮挡，
并非成熟度或大小类别。公开作者仓库为 bag+nobag；本机另有 young 集。

局部 1200 帧：9 个实测参考袋（SAM 轮廓 + 行带有效深度约束可见表面，零深
度不当表面）；参考袋不虚构内果、不加折痕（保持深度对照锚纯净）。相机为
作者公开的 RGB 90° 水平视场近似（fx=fy=640，1280×720），非逐机标定；
枝条遮挡部分、袋背厚度、整园树形与树距是标注清楚的推断。

外部依据：[数据集作者](https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset)、
[eOrganic 套袋流程](https://eorganic.org/node/25727)、[UGA 修剪](https://extension.uga.edu/publications/detail.html?number=C1087)、
[NC State 桃树形态](https://plants.ces.ncsu.edu/plants/prunus-persica/common-name/common-peach/)、
湖南省林业局/DB41/T 1317-2016 树形规范、
[Blender 4.5 渲染通道](https://docs.blender.org/manual/en/4.5/render/layers/passes.html)。

## 受控条件与分层口径

- **光照**（`lighting.py`，Nishita 天空 + 单个 SUN，不动几何）：
  `noon` / `morning` / `late_afternoon` / `backlit`（保留逆光困难组）/
  `overcast`（均匀漫射天空近似，关闭直射太阳）。空气与尘埃密度分开配置，
  不用高浑浊度代替阴天；曝光由预设指定。参数落 manifest，尚非现场辐照度标定。
- **遮挡与枝干扰**（`occlusion.py`，逐袋 manifest 记录）：名义档
  none/light/heavy（0/2/4 片受控前景叶，heavy 附袋前横枝；走廊枝 ~15%
  独立概率，口径对齐 adaptive_shear L_insert 0.090）。**名义档会被自然
  冠层本底淹没**（实测 none≈0.38、light≈0.14 倒挂），评测主分层用渲染深
  度反测的连续覆盖率桶（low<0.2 / mid / high>0.5）。
- **视角轨迹**（`viewpoints.py`，复刻产线停走节拍）：每目标 = 近距拍照位
  （0.55–0.75 m，对齐数据集深度分布 p50 0.61 m）+ 补视链（0.15 m 直线
  截距 / 绕袋轴 ±30° 两候行程短者，总视数 ≤3，常数与 `view_policy.py`
  原值对拍防漂移）；`run_matrix.py` 用 ray_cast 做"冠外取景"修正（被挡
  则沿作业道后退）。作业道停靠（6 站远眺）仅作 survey 分层，不进每目标
  聚合门。内参 fx=fy=640 @1280×720 = Blender 36 mm 传感器 + 18 mm 镜头。

## 验收口径（三道门 + 矩阵基线门）

1. **几何门**：袋网格闭合、果球包容（生成期间隙 ≥0.5 mm 入 manifest）；
   errors 非空即 raise。不代表全部枝叶/袋间无碰撞。
2. **三锚定机位感知门**：YOLO 一对一 IoU.5 对 IndexOB 实例 GT；SAM 是
   GT 框提示，不冒充端到端分割召回；reference 机位另有对 1200 实深的
   逐袋 MAE。检出与分割是辅助证据，不是照片级真实性证明。
3. **矩阵基线门**（`baselines/perception_matrix_baseline.json`）：分层
   recall（光照 × 视角类型 × 遮档/覆盖率桶）+ 袋级多视聚合（单光照内
   3 视序列 ≥1/≥2 检出）不低于冻结基线 −0.05，防建模改动造成静默回
   归；深度为理想渲染值（无噪声模型）。逐轮结论随提交说明；冻结矩阵基线继续保留。

## 无草整园 Gazebo 查看（重建 GT 桥接，2026-10-09/10）

重建整园去草版（`build_scene.py --no-grass`）导出单 GLB 供 Gazebo 人工查看：
`blender/export_field_to_gz.py` 删相机/灯与 backdrop 远景行 → GN 叶实例插
Realize 节点求值物化（**两阶段**：先全部物化再删原对象——边物化边删会使
depsgraph 失效、23/24 棵树静默丢叶）→ 逐对象 `Mesh.transform` 烘焙世界
变换（多对象不 join）→ `export_yup=False` 导出（本机 gz-common
5.9/Assimp 链按 Z-up 直读 GLB 顶点，yup=True 会被竖成 90° 墙；与
build_assets 树 GLB 的表现差异未深究，以实测为准）。全叶版 ≈1.7 GB /
3650 万顶点（24 树 × ~170 万顶点叶幕），首次加载需数分钟；嫌慢用
`--decimate 0.25` 出轻量版。`--keep-backdrop` 留远景行。

```bash
# 导出（工作区根目录）
_tools/blender-4.5.14-linux-x64/blender -b \
  src/peach_sim/reconstruction/output/field_no_grass_20261009/bagged_peach_orchard.blend \
  -P src/peach_sim/blender/export_field_to_gz.py -- \
  --out src/peach_sim/meshes/field_no_grass.glb

# 仅看世界（纯视觉静态模型，查看完关窗即清）
GZ_SIM_RESOURCE_PATH=src:src/peach_sim gz sim -r \
  src/peach_sim/worlds/field_no_grass.sdf

# 带完整采摘机器人（固定工位：BUNKER 彩色底盘 + 臂链 + 球形末端）
# 本世界只有视觉地面、无地面碰撞；physics type="ignored" 不关闭物理。
# 禁用 mobile 模式以免底盘自由落体；驾驶体验用默认 peach_orchard.sdf。
ros2 launch peach_sim orchard_sim.launch.py \
  world:=install/peach_sim/share/peach_sim/worlds/field_no_grass.sdf mobile:=false
```

仅为人工查看效果；导航/感知 GT 仍以 `worlds/peach_orchard.sdf`（程序化
锚定世界）为准，两者不混用。

固定工位默认使用 `tool_profile:=bite_shear_v1`（peach 球形环抱咬合式剪切手
的真实 CAD 网格和 TCP 档案）与下载适配的 BUNKER Pro 2 外观；深灰车体、
黑色履带、橙色面板和金属灰
轮毂通过 GLB 材质显示。资产来源与重导出见 [BUNKER_SOURCE](meshes/BUNKER_SOURCE.md)。
其他 peach 真实末端可用 `tool_profile:=shear_v1` 或 `tool_profile:=adaptive_shear_v1`
显式选择。移动导航模式仍使用原来的低模底盘。

模型看不见、动态底盘下落、工具选型和渲染颜色的定位顺序与解决方法见
[包级架构文档：Gazebo 模型不可见](doc/architecture.md#gazebo-模型不可见定位与解决方法)。

## 自主导航（Gazebo Harmonic + Nav2，2026-10 轮）

仿真采摘工位可驾驶化 + 官方 Nav2 全栈，全部落在本包内（不进 harvest_system、
不动驱动九包）。三入口：

```bash
# 1) 自主导航：gz 履带驱动 + 桥接 + AMCL/NavFn/MPPI + RViz（2D Goal Point 发目标）
ros2 launch peach_sim orchard_nav.launch.py                 # rviz 默认开、gz gui 默认关
ros2 launch peach_sim orchard_nav.launch.py rviz:=False gui:=false  # 无头

# 2) 巡航采摘：逐树停靠位 NavigateToPose（蛇形穿行树行，每站停留模拟节拍）
ros2 run peach_sim orchard_tour --rows 2,3                  # 或省略 --rows 全园

# 3) SLAM 校验：slam_toolbox 在线建图，与确定性生成图人工对照
ros2 launch peach_sim orchard_nav.launch.py slam:=True rviz:=False
ros2 run nav2_map_server map_saver_cli -f /tmp/slam_check
```

（布尔 launch 参数注意：`slam`/`rviz` 走 IfCondition/PythonExpression 求值，
须用 Python 大写字面量 `True`/`False`；`gui` 由本包 Python 解析、大小写皆可。）

要点：

- **帧树（REP-105）**：`map → odom → platform_base_link → {left,right}_track/lidar_mount`。
  `odom→platform_base_link` 由 gz DiffDrive 发布（关节积分里程计），`map→odom` 由
  AMCL/SLAM 发布；Nav2 基座帧 = `platform_base_link`。
- **mobile URDF 是纯平台**（不含臂链）：`aubo_e5.urdf` 的 `world_joint` 在 URDF→SDF
  后把整机焊死在世界系（固定座语义），只读面外无法解除；本轮导航不动臂，故
  mobile 模式根 = `platform_base_link`、spawn 落地 z=0。锚定建模模式（默认）
  行为与旧版完全一致（含臂链与世界锚定）。
- **驱动 = gz DiffDrive + 履带滚筒轮**（官方 TB3/TB4 仿真同款）：mobile 下履带为
  continuous 关节 + 圆柱碰撞体（盒体仅视觉，等效履带接地印迹），各向同性 μ 1/1
  显式摩擦。冒烟实测（2026-10-08）：指令 1.2 m 实走 ~1.05 m（滚筒滑移 ~12%），
  AMCL 兜底。官方 TrackedVehicle+TrackController 表面速度方案在 gz-sim 8.15 的
  URDF→SDF 路径上不出牵引（指令链通、整车不动；疑 dartsim 未实现接触面速度
  API），留待原生 SDF 模型轮再试。
- **传感**：前桅杆 2D 激光（z=0.45 穿树干区，±95°——车体自遮挡扇区不发射），
  桥接 `clock/joint_states/odom/tf/scan` 入、`cmd_vel` 出（普通 Twist）。世界已加
  `gz::sim::systems::Sensors`（ogre2）系统渲染传感器，否则 /scan 无发布者。
- **地图**：`generate_orchard --map` 确定性生成 `maps/orchard_map.{pgm,yaml}`
  （树干保守上界 + 棚架立柱 + 地面边环），与世界同参数重生成；AMCL 初始位姿
  取场景 spawn，免手发 2D Pose Estimate。
- **冒烟结论（2026-10-08）**：AMCL/NavFn/MPPI 全 active，`map→odom→platform` TF
  链解析出生位姿；巡航 `orchard_tour --rows 2` 多轮到站（含出生点后方掉头
  航段）。**已知欠账**：窄过道（行距 4 m、作业位 x=0.95）掉头处 MPPI 仍会
  卡死被 progress checker 中止（2/7 站），`gz DiffDrive 保持最后一条速度指令
  无超时`——真实任务里由 Nav2 velocity_smoother 持续发布兜底；两项调参
  （RotationShim/行为树恢复策略）留下轮。
- **测完即停**：Ctrl+C 停栈后 `pgrep -af 'gz sim|ros2 launch|rviz2'` 清残留
  （MUST；无头冒烟建议 `timeout` 包裹）。

## 旧 Gazebo 静态场景

`peach_sim.params/scene/cli`、`config/orchard.yaml`、`worlds/peach_orchard.sdf`
既是旧静态管线（`orchard_sim.launch.py` 默认锚定模式），也是自主导航轮的
世界底座（`mobile:=true` 由 `orchard_nav.launch.py` Include）；新建模不消费
它们做感知验收，Blender 与 Gazebo 仍不是同一份 GT。

## 许可

包源码 BSD-3-Clause，见 LICENSE。真实数据集与模型权重保持各自原许可，
未重新分发。
