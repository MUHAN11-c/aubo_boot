# 数据驱动 Blender 重建执行记录

依据：2026-09-24 用户批准的从真实数据重建方案；旧外观、统计与布局不作为输入。

目标：可编辑 Blender 场景，真实 RGB-D 支撑的局部枝果几何、完整树行环境、渲染及检测分割核验。
约束：离线；驱动只读；numpy 1.26.4；不启动真机；运行结束清进程。
技术：独立 Blender 4.5.14 Python/bpy；项目 venv 的 Pillow/numpy/Ultralytics YOLO+MobileSAM。

- [ ] 采样 RGB/Depth/IR/VOC，记录源、深度有效性、相机近似；以原模型分割恢复局部尺度。
- [ ] 从零建纸袋、独立叶片、连通枝条，保存局部及整园 Blender 场景。
- [ ] 渲染多机位 RGB/深度/实例 ID；检测分割对照；检查连接、穿插和轮廓。
- [ ] 保存证据与命令，更新包入口说明，清理进程。

关键校验：深度零洞不填为背景；公开 FOV 不是逐机标定；遮挡实例不使用整框背景深度；纸袋闭合、果实在内；叶片和挂果枝必须连接父枝；轴向深度与射线距离不可混用。

裁定：当前有大量用户未提交修改，新增 reconstruction 作为全新 Blender 入口，不覆盖旧生成物；包 README 改为新入口并明确旧入口未验收。无需重新请求已经批准的离线建模权限。
来源：https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset （RGB 对齐、90°×59°）；https://eorganic.org/node/25727 （袋口绕枝、纸袋收拢固定）；https://extension.uga.edu/publications/detail.html?number=C1087 （开心形骨架）；Blender 4.5 bpy/Passes 文档；项目 inference.py 的 Ultralytics SAM 用法。

---

## 2026-09-28 管线合并 + 室外条件矩阵轮（用户批准方案）

依据：用户批准的「Blender 建模升级 + 多轮视角离线感知验证矩阵」计划
（合并 blender_orchard、仅离线矩阵评测、规模参数化可伸缩三项裁定）。

- [x] M0 红项收口：bag_4 果穿袋为陈旧 .blend 残留（现行代码参考袋无内
      果）；add_bag 增生成期 BVH 间隙校验（≥0.5 mm，clearance 入
      manifest）；validate_perception 补跑落盘；Cycles 切 OptiX（全量
      重建 40 s）；RESULTS.md 建档。
- [x] M1 合并：measure_priors/make_textures/crease/贴图迁入（产物与
      blender_orchard 逐字节一致）；纸袋贴图仅作 bump——颜色乘法把红色
      推离数据集中位 (101,60,55)，reference 机位实测 9→7 匹配，回退为
      bump-only 后 9/9；git rm blender_orchard，三活文档头部注记同步。
- [x] M2 受控条件：lighting.py 四预设（morning 逆光方位冒烟实测剪影
      化，改顺光侧 170°）；occlusion.py 名义档 none/light/heavy + 枝干
      扰（袋前横枝/走廊枝 ~15%）；袋尺寸倾角 priors 分位逆采样入
      manifest；--scale validation/field（field=3 行×8 株锚定，393 袋）。
      坑：先验 p10 窄袋厚 0.027 m 遇原幅度 crease 腔体近闭合（果心距面
      0.28 mm）——crease 幅度按袋厚缩放修复。
- [x] M3 矩阵：viewpoints.py 停走轨迹（近距拍照位 0.55–0.75 m 对齐数据
      集深度分布 + 补视链常数与 view_policy 对拍 1e-9）；run_matrix.py
      ray_cast 冠外取景修正（冒烟 5/18 视位后退）；evaluate_matrix.py
      分层聚合 + depth_mm.png + 基线门。坑①：作业道远眺（>2 m）超 YOLO
      训练分布（数据集 p90 1.13 m），改为独立 survey 分层。坑②：名义
      遮挡档被自然冠层本底淹没（实测 none 0.38 vs light 0.14 倒挂），
      主分层改连续覆盖率桶 low/mid/high。坑③：双 Blender 并发抢 3090
      GPU OOM（矩阵 45 帧处死）——GPU 渲染作业必须串行。
- [ ] 全量矩阵跑通 → 冻结基线 → RESULTS.md 轮次落档（进行中）。

关键校验增补：受控遮挡叶必须挂结果枝（连接图 error<1e-6）；矩阵帧不入
git（output/matrix/ gitignore，summary/report/trajectory 拷贝入库）；
光照只改 world 不动几何（一场景多光照重渲）。

---

## 2026-09-29 树结构去克隆轮（用户在 GUI 目检发现）

- 用户目检发现整园树为同构克隆——核实：tree() 树干控制点/4 主枝等角/
  主枝长度比例全部硬编码，seed 只抖方位角与侧枝。重写为逐树随机：干高
  0.46–0.68 m（含倾斜）、主枝数 3 为主偶 4（DB41/T 1317 三主枝开心形）、
  主枝长 0.80–1.15 m/极角 42–58°/枝径随长缩放、冠梢 20–32 根补心，
  tip 夹 ≤2.4 m。袋位随 3 主枝减为 12/树（总袋 153→121），分层子集
  采样不受影响。
- 同轮捆绑外观/日光重调（modeling_revision 2026-09-29-foliage-daylight-v1）：
  叶片几何重做（披针形锯齿+叶柄+下垂）、材质调亮（纸/皮/叶/土/草）、
  exposure -0.55→0 交预设、光照预设体系重做（turbidity→air/dust 密度
  +cloud_cover+exposure_ev，新增第 5 档 backlit 逆光困难组）——光照
  语义变更使 09-28 冻结基线不可比，矩阵轮须重冻结。
- 新坑（GUI 探索联动）：用户把 Blender 界面切中文且勾了"翻译新建数据"，
  新建节点起中文名（原理化BSDF），materials.py 按英文名 get() 直接
  NoneType 崩——全部改为按节点类型查找（BSDF_PRINCIPLED/BACKGROUND/
  OUTPUT_MATERIAL），任何语言偏好下稳健。GUI 语言属用户偏好，代码侧
  适配而非改回语言。
- 验证：几何门 0 errors（121 袋/2760 连接）；前后对比板
  output/tree_variety_before_after.jpg（克隆感消除、无破面）；矩阵
  全量重跑（5 光照档含 backlit）+ 基线重冻结见 RESULTS.md 当轮。

### 2026-09-29 下午追加：groundcover-v2

- 已完成弯曲草丛、枯叶、土块；参考局部仅留 validation，field 为纯 24 株行列。
- 已统一两份模型源版本，完成 5 光照×3 锚定视角与有限几何验证，逐视角哈希拒绝混版。
- 已生成修改前后、实拍与五光照对照板；完整矩阵和冻结基线仍为上一轮，未宣称覆盖新版。
- 后续建模优先：纸袋微褶皱和纸感、叶面细节、外围地表与实测地形；再验证导出材质、
  碰撞简化及 RGB-D/ROS 同管线。当前模型尚不构成自主导航或采摘成功证明。

## 2026-09-29 傍晚：canopy-v4（冠层密度 + 叶色 + 远景行列）

- [x] 叶色按 PeachDataSet RGB 实测校正（60 帧叶像素中位 sRGB (72,98,62)，
      色度/明度 0.36；v3 渲染 0.67 → v4 0.50/作业道 0.41），加天光高光。
- [x] field 成熟冠幅 crown=1.5（主枝/侧枝伸展、侧枝与新梢数随冠幅增），
      侧枝末端夹 2.2 m 守树高 ≤2.5 m；validation crown=1.0 顶点数不变。
- [x] field 远景：无袋集合实例 14 块续行列到地平线；新增作业道停靠机位
      aisle（90° HFOV）；validate_perception 增 `--views`。
- [ ] 下一步：纸袋外形（袋口收束绕枝、下垂软袋、褶皱）；叶片实例化降
      .blend 体积；validation 矩阵按 v4 重跑重冻结；导出前碰撞简化。

## 2026-09-29 傍晚：fruit-bag-v5（成熟果 + 袋形 + 叶片实例）

- [x] 果径取 Peach_nobag class 1 的 u≥0.5（p50–p90）；袋宽重抽到装得下，
      高度不够只抬高度。质量按假设密度 970 kg/m³ 写入清单。
- [x] 纸袋改为圆截面水滴形：果颊最宽、袋口收到约 1.6 cm 再小幅外翻，
      扎丝收成细圈。果不缩小；间隙不够直接失败。
- [x] 叶片改为 20 个共享原型的几何节点实例（5 材质 × 4 外形）。
- [x] 叶色再提一档，对准实拍叶像素 (72,98,62)。
- [x] 按 v5 重跑感知矩阵（subset 20，1065 帧）并重冻结基线，基线门通过。
- [ ] 下一步：导出（USD/SDF、碰撞代理）。
