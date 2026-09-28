# peach_sim — Blender 数据驱动果园重建 + 多视角感知验证矩阵

外观建模与感知验证的唯一入口是 **`reconstruction/`**（2026-09-24 重建，
2026-09-28 合并根目录 `blender_orchard/` 观感链并升级为受控条件矩阵）。
旧 Gazebo 入口仍在仓库中，但不是本次重建场景，也不代表通过真实感验收。

## 工具链（全部离线前台，完成即退出，不跑 ROS、不动真机）

| 脚本 | 职责 | 主要产物 |
|------|------|----------|
| `measure_priors.py` | PeachDataSet 分位统计 + 现场袋轴实测 | `evidence/priors.json` |
| `make_textures.py` | 程序化贴图（纸/叶/皮/土/草，确定性 seed） | `textures/*.png` |
| `audit_sources.py` / `analyze_reference.py` | 数据集审计 + 6 帧逐袋测量 | `evidence/*` |
| `build_scene.py` | 建场（参考重建 + 推断果园 + 受控遮挡），渲三锚定机位 | `output/*.png`、EXR、`scene_manifest.json`、`.blend` |
| `validate_geometry.py` | 几何门：袋闭合 + 果包容（errors 非空即 raise） | `output/geometry_validation.json` |
| `validate_perception.py` | 三锚定机位感知门：YOLO/SAM 对实例 GT + 1200 帧深度对照 | `output/perception_validation.json` |
| `viewpoints.py` | 停走轨迹纯核：survey 停靠 + 每目标近距拍照位 + 补视链 | 被 `run_matrix.py` 消费 |
| `run_matrix.py` | 矩阵渲染：已建 `.blend` × 光照预设 × 轨迹视角（含 ray_cast 冠外取景修正） | `output/matrix/<光照>/<视>/` |
| `evaluate_matrix.py` | 矩阵评测：分层聚合 + 深度合成 `depth_mm.png` + 基线回归门 | `output/matrix/{summary.json,report.md}` |
| `make_appearance_board.py` | 真实数据 vs 渲染对照板（人工 QA） | `output/appearance_board.jpg` |
| `lighting.py` / `occlusion.py` / `distributions.py` / `depth_io.py` | 纯核：光照预设 / 受控遮挡 / 分位采样 / 深度量化 | — |

纯核单测：`test_reconstruction_core.py`（含与
`peach_harvester/cycle_core/view_policy.py` 的补视几何对拍）、`test_measurement.py`。

## 重现（工作区根目录，独立 Blender 4.5.14，venv `aubo_py3.12`）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

aubo_py3.12/bin/python src/peach_sim/reconstruction/measure_priors.py     # 数据变了才需要
aubo_py3.12/bin/python src/peach_sim/reconstruction/make_textures.py      # 贴图确定性重建
aubo_py3.12/bin/python -m pytest src/peach_sim/reconstruction/ -q         # 纯核单测

_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/build_scene.py -- --view all --samples 48 --width 1280
_tools/blender-4.5.14-linux-x64/blender -b \
  src/peach_sim/reconstruction/output/bagged_peach_orchard.blend -t 4 \
  --python-exit-code 1 -P src/peach_sim/reconstruction/validate_geometry.py
aubo_py3.12/bin/python src/peach_sim/reconstruction/validate_perception.py

# 多视角感知验证矩阵（约 1 小时，RTX 3090 OptiX）
_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/run_matrix.py -- --samples 24 --subset-per-level 20
aubo_py3.12/bin/python src/peach_sim/reconstruction/evaluate_matrix.py \
  --matrix src/peach_sim/reconstruction/output/matrix --gate     # 或 --write-baseline 冻结

# 导航轮铺路：field 规模锚定图（3 行×8 株，仅整园一图 + 完整 manifest）
_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/build_scene.py -- --scale field --view orchard \
  --samples 48 --width 1280 --out src/peach_sim/reconstruction/output/field_anchor
```

`output/` 是验证场契约（三锚定机位 + manifest + 双门产物）；矩阵帧在
`output/matrix/`（gitignore，可重现，`summary.json`/`report.md`/
`trajectory.json` 拷贝为 `output/matrix_*` 入库）；field 锚定在
`output/field_anchor/`。只改相机与采样可用 `render_saved.py`，无需重建几何。

## 数据依据与可信边界

`/home/mu/Downloads/PeachDataSet`：2,050 组套袋 / 577 裸桃 / 1,000 幼桃。
`evidence/priors.json` 为袋宽/高/高宽比/深度/最近距的 p10–p90 分位摘要
（n=722）与现场 14 袋轴实测（倾角 p50 23.8°、袋底世界高 ~1.34 m）。推断树
的袋尺寸/倾角按分位逆采样，逐袋记录 `source_percentile`；纸色中位
(101,60,55)。原始图只做统计与对照，**不作为贴图**。

局部 1200 帧：9 个实测参考袋（SAM 轮廓 + 行带有效深度约束可见表面，零深
度不当表面）；参考袋不虚构内果、不加折痕（保持深度对照锚纯净）。相机为
作者公开的 RGB 90° 水平视场近似（fx=fy=640，1280×720），非逐机标定；
枝条遮挡部分、袋背厚度、整园树形与树距是标注清楚的推断。

外部依据：[数据集作者](https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset)、
[eOrganic 套袋流程](https://eorganic.org/node/25727)、[UGA 修剪](https://extension.uga.edu/publications/detail.html?number=C1087)、
湖南省林业局/DB41/T 1317-2016 树形规范、
[Blender 4.5 渲染通道](https://docs.blender.org/manual/en/4.5/render/layers/passes.html)。

## 受控条件与分层口径

- **光照**（`lighting.py`，Nishita 天空，只换 world 不动几何）：
  `noon`（合并轮基线锚）/ `morning`（低角暖光，顺光侧——逆光方位会把袋
  拍成剪影，冒烟轮实测剔除）/ `late_afternoon`（西向侧光）/
  `overcast`（高浑浊度漫射软影）。参数落 manifest。
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
   归；深度为理想渲染值（无噪声模型）。逐轮结论追加 `reconstruction/RESULTS.md`。

## 旧 ROS 场景

`peach_sim.params/scene/cli`、`config/orchard.yaml`、`worlds/peach_orchard.sdf`
仍是旧 Gazebo 管线；新建模不消费它们，二者不是同一份 GT。保留既有包接口
避免本轮改变 ROS 运行栈；切换仿真资产需另做坐标与碰撞对账。

## 许可

包源码 BSD-3-Clause，见 LICENSE。真实数据集与模型权重保持各自原许可，
未重新分发。
