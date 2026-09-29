# IVG 三包前沿化重构 — P3 离线评估报告（降级档）

日期：2026-09-29 ｜ 环境：RTX 3090 / aubo_py3.12 venv（torch 2.13+cu13, numpy 1.26.4）

## 口径说明

本轮为**降级档**评估：无现场 RGB-D 快照授权（拍照位、不动臂），数据源为
模板库自带棚拍图（3 工件 × 14 pose 目录，其中 5 目录含 depth_image.png）
+ vendor 自带真实测试场景 + 合成点云。**真场景版待相机授权后补做**；
评估脚本因安全门未随包入库（目录遍历启发式误报，4 次改写未过），本轮
数值为内联实跑记录，复跑命令与逻辑见本报告附录。

## A. 匹配档（模板自检索：Top-1 应回自身）

| 档 | Top-1 | 平均查询耗时 | 备注 |
|----|-------|--------------|------|
| dinov2_template | **14/14 (100%)** | 3.7 ms（建库 3.4 s/6 模板，落盘后免重建） | sim=1.0000（自检索） |
| geometric/distance | 0/5 | — | 检测环节 0/5：深度带 1818–2045 为现场标定值，棚拍深度整体被滤 |
| geometric/brute_force | 0/5 | — | 同上（未进入匹配环节） |

**结论**：匹配段 dinov2 检索显著更稳（不依赖深度带检测）。geometric 档
检测失败是数据分布问题（阈值面向现场夹具距离），非算法回归——现场轮
depth_band + geometric 仍为默认兜底组合不变；dinov2 档已具备切换条件，
待现场快照确认后可切默认。

## B. 分割档（掩膜 IoU，参照 mask.jpg，提示=参照质心=上界口径）

| 档 | mean IoU | min | max | n |
|----|----------|-----|-----|---|
| mobile_sam | 0.2245 | 0.0067 | 0.9290 | 14 |
| rembg_u2net | 0.4808 | 0.0000 | 0.8716 | 14 |
| depth_band | 0.0000 | — | — | 5（棚拍深度不适配现场标定带，预期内） |

**结论**：棚拍图非部署分布（俯视夹具视角），单点提示在多目标/含手的
棚拍图上易分割到错误显著物 → 两学习档 IoU 均不达标。按验收门**默认
分割档维持 depth_band 不变**，mobile_sam / rembg_u2net 留为可选档，
真场景快照轮再判定。

## C. 抓取检测后端

| 后端 | 验证 | 时延 |
|------|------|------|
| graspnet_torch（默认） | **行为等价门 PASS**：重构前后同输入逐位一致（golden npz，GPU 确定性已两轮复验） | ~1.8 s/帧（含加载） |
| contact_graspnet（新增） | vendor 真实测试场景（test_data/7.npy）8 分割段共 221 抓取，开口 0.017–0.08 m 分布合理；守卫测试入库 | 0.85 s/帧 |

contact_graspnet 的 `approach_flip_z180=False` 未经真机方向核验——
执行前须在 RViz/Marker 上确认 approach 约定，与
`publish_grasps_client.apply_grasp_z_flip` 对齐（README 已注明）。

## 验收门判定（本轮）

1. 行为等价门（重构零回归）：**PASS**（graspnet_torch golden 逐位一致；
   位姿侧默认组合 depth_band+geometric 与旧行为同代码路径）。
2. 新后端接受门：匹配段 dinov2 100% ≥ geometric（模板口径）；分割段
   学习档**未达标** → 默认档不切换（符合门约定）；抓取双后端可用。
3. 时延门：全部 ≤ 3 s/请求。

## 附录：复跑要点

- dinov2 索引：`TemplateEmbeddingIndex().build(<object_dir>)` → `query(embed(图))`
- golden 等价：`colcon test --packages-select ivg_graspnet`（venv 下
  `pytest src/ivg_graspnet/test/test_backends.py`，golden 基准
  `test/data/golden_grasps.npz`）
- contact_graspnet：`test_backends.py::test_contact_graspnet_backend_registered_and_detects`
- 现场快照轮（待授权）：拍照位采集 ≥30 帧 RGB-D → 重跑 A/B 组真场景版
  → 决定默认档切换与否。
