# 数据驱动小果园资产

来源是本项目已有 v11 `reconstruction/output/field_no_grass_20261009/bagged_peach_orchard.blend` 的 Tree00、Tree01，不使用历史 `sim_description/meshes/bagged_peach.glb`。数据参考为 [Multi-class peach RGB-D 数据集作者仓库](https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset)；袋果尺寸、颜色/遮挡统计和参考观测沿用 `reconstruction/evidence/priors.json`、`real_feature_audit.json`。

作者公开 README 覆盖 2050 帧套袋和 577 帧裸果；本机另有幼果子集，不能自动视为同一公开来源。本批袋果先验来自套袋与裸果，不把幼果用于补造来源。四类 VOC 标签是遮挡类别，不是成熟度。尺寸来自 FOV 近似内参和 RGB-D 估计，不是标定测量；树形、隐藏结构、行列和质量（假设密度）仍是推断。

## 复现

在 peach_sim 根目录，使用已有 Blender；目录外依赖只读。为了保留现有证据，复现应写到新目录：

```bash
scripts/sim check /home/mu/Desktop/aubo_e5_jazzy_ws/_tools/blender-4.5.14-linux-x64/blender \
  -b sim_worlds/reconstruction/output/field_no_grass_20261009/bagged_peach_orchard.blend \
  -t 6 --python-exit-code 1 -P sim_worlds/blender/export_data_orchard.py -- \
  --out .runtime/data_orchard_rebuild
```

`assets.json` 是本批次单一事实源，记录原 blend、场景清单、先验、数据审计、导出器和每个 GLB 的 SHA256。运行时拒绝哈希不一致的网格。

## 几何与运行边界

- 两株树、35 颗独立袋果；保留所有原袋果，不因大小、倾角或遮挡删选。整体只做 `(5.25, -2.3, 0)` 米平移。
- 树体和袋果导出为 Z-up GLB，袋果以原袋底为原点。纸袋、内果、梗和扎口一起移动。
- 叶片减面比例 0.20；枝干与袋果不减面。3615 段碰撞圆柱直接来自实际枝干网格环，半径取该段端环较大值。
- 静态/动态袋果共用视觉模型与实际几何 AABB，惯量使用该均匀刚性盒、质量来自源清单。不是柔性纸袋、剪切力或摩擦抓取模型。
- 默认地面平坦；道路/树位仍为推断，不代表真实果园测绘。GPU 图像、实际碰撞、导航和视觉采收需要独立运行验收。

2026-11-02 用户要求先完成代码与离线核对，暂不运行 Gazebo。离线检测成功不等于 Gazebo 视觉采摘成功。
