# 左视差盲带与近目标不可测性——双目理论 + 业界实现对照分析

**日期** 2026-09-20 · **对象** peach_stereo（PS800-E1 双 IR，左参考 SGBM）· **姊妹篇** [report.md](report.md)（参数与三方裁决）
**触发问题** 演示中 YOLO 检出 2 个套袋、仅 1 个确认——第二袋位于图像左下角 x∈[26,165]，掩膜∩有效深度 = **0 px**。

## 0. 结论速览

1. 左参考双目在图像左侧产生**宽度 ≈ numDisparities 的结构性无效带**——Intel RealSense 官方文档记载的同构现象（左参考匹配 → 边界 non-overlap → 零深度），不是本仓缺陷。
2. 本仓盲带边界实测 166 px（彩色坐标），理论折算 164 px，偏差 2 px；全局最近可测深度实测 268 mm，理论 f·B/numDisp = 266.8 mm，偏差 1.2 mm。
3. "贴左 + 近"目标是**绝对盲区**：每个像素的视差 d 大于自身列号 u，匹配点落在右传感器之外——**与 numDisparities 取值无关**，任何参数都救不回。
4. 管线对该目标的处置（`mask_unavailable` → REOBSERVE，不造假位姿）与业界诚实约定一致（RealSense 官方建议裁掉/忽略这些像素，插值补洞白皮书只适用于小遮挡洞、不适用于结构性盲带）。
5. 业界标准缓解 = **视点规划**（移动相机让目标落画面中部）——本仓拍照位/视点规划策略与之同构。

## 1. 理论推导（多视几何）

### 1.1 平行校正下的对应几何与可测域

校正后极线水平，左图像素 u_l 与右图 u_r = u_l − d 对应，视差 d = f·B / Z
（Hartley & Zisserman《Multiple View Geometry》第 2 版 §11 校正几何；RealSense 白皮书式 (1) 同形）。

左参考匹配要求 u_r ≥ 0，故逐像素约束：

```
d ≤ u_l   ⇔   Z ≥ Z_min(u_l) = f·B / u_l
```

即**左边缘像素"看得近"的能力最差**：第 0 列永远测不了任何东西，第 u 列最近只能测 f·B/u。搜索窗 [minDisparity, minDisparity+numDisparities) 再与 [0, u_l] 取交集，得到有效搜索域。u_l < numDisparities 的列对近目标（大 d）被结构性截断——这就是盲带的来源。

### 1.2 本仓数值代入（f_half=548.9 px，B=62.22 mm，numDisp=128@半分辨率）

| 量 | 公式 | 理论 | 实测 | 偏差 |
|----|------|------|------|------|
| 全局最近可测深度 | f·B/numDisp | 266.8 mm | **268 mm** | +1.2 mm |
| 盲带宽度（彩色坐标） | FOV 边距 51 + 128×0.88 | 164 px | **166 px** | +2 px |
| 带边列的可测下限 | f·B/128 | 267 mm（带边只能测 ≥267mm 的内容） | 与实测带内 0 有效一致 | ✓ |
| 0.55m 目标的视差 | f·B/Z | 62 px | 中央袋中位 548mm ↔ d≈62 | ✓ |

### 1.3 第二袋的绝对盲区判定

bbox 彩色 x∈[26,165] → 折算半分辨率校正左系列号 u ≈ 0–65；若袋距 ≈0.5–0.55 m，则 d ≈ 62–68 px。

```
对每个像素：d ≈ 62–68  >  u ≤ 65（且仅右缘个别列 u≥d）
→ 匹配点 u_r = u − d < 0，落在右传感器之外
```

**与 numDisparities 无关**：把 numDisp 调到 256 也改变不了 d ≤ u 的物理约束，只是把"截断线"从 128 挪到 256。实测掩膜∩深度 = 0 px 严格吻合。

### 1.4 半遮挡（half-occlusion）——洞的第二来源

左参考图看不见物体左侧被遮挡区（另一目看到的像素在参考图中无对应）。Scharstein & Szeliski 的经典分类综述（IJCV 2002）把半遮挡列为 dense 双目的固有难题，Middlebury 评估协议显式屏蔽遮挡区不参与评分。这解释了覆盖矩形内 ~17% 的散洞（SGBM 弱纹理失败为其余部分）。

## 2. 业界实现对照（官方文档 / GitHub）

| 实现 | 官方记载 | 与本仓对照 |
|------|----------|-----------|
| **Intel RealSense D400**（[librealsense #6311](https://github.com/IntelRealSense/librealsense/issues/6311) 官方回复） | "深度用左目作匹配参考，图像边界产生 non-overlap 区（零深度）"——左带是**预期行为**；官方建议应用层裁掉该边 | 同构：本仓 166px 左带、感知层等效裁剪 |
| **RealSense 白皮书**（[arXiv 1705.05548](https://arxiv.org/abs/1705.05548)） | Z = f·B/d；视差搜索范围 ↔ min-Z 权衡；投射/遮挡对深度图的影响专章 | 同式：本仓 Z_min = 548.9×62.22/128 = 267mm |
| **RealSense disparityShift**（[Intel 社区](https://community.intel.com)、[librealsense issue](https://github.com/)） | 平移视差窗可压 min-Z（如 30–110cm 窗），**代价是 max-Z 等量缩小**——一扇窗两头不能同时要 | 本仓未启用：0.3m 下限对台架 0.43–0.97m 场景有富余 |
| **OpenCV**（[StereoSGBM 文档](https://docs.opencv.org/4.x/d2/d85/classcv_1_1StereoSGBM.html)、[issue #9879](https://github.com/opencv/opencv/issues/9879)） | 视差搜索区间显式约束左带；minDisparities>0 时边缘截断是已知行为 | 本仓 minDisp=0，带边 ≈ numDisp 折算，实测吻合 |
| **RealSense 后处理白皮书**（[Depth Post-Processing, Mouser/Intel](https://www.mouser.com/)） | hole-filling 只补小遮挡洞；结构性无效带建议裁剪而非插值 | 本仓不做 hole filling 补盲带（会造假深度），一致 |
| **Middlebury 双目基准**（[vision.middlebury.edu/stereo](https://vision.middlebury.edu/stereo/)） | 评估协议显式忽略遮挡区 | 同精神：mask_unavailable 不计为可测 |
| 结构光（Kinect / PS800-E1 设备链） | 投射器-接收器几何产生**投射侧**阴影遮挡；无窗口搜索左带，但点稀疏 | percipio 链无左带、8.3% 有效——"窄密 vs 全稀"互补 |

## 3. 缓解手段矩阵（业界做法 vs 本仓选择）

| 手段 | 原理 | 代价 | 本仓 |
|------|------|------|------|
| **视点规划 / 拍照位移** | 移动相机使目标落画面中部（u ≫ d） | 需臂可动、节拍变长 | ✅ **采用**（拍照位/视点规划；RealSense 官方同建议：移动相机或裁边） |
| 裁剪输出到公共视域 | 发布时裁掉盲带 | 丢 FOV 观感 | 部分（感知窗等效） |
| disparityShift / 负 minDisparity | 平移视差窗换 min-Z | max-Z 等量损失 | ❌ 不需要 |
| 右参考 / 双参考融合 | 盲带换侧或消除 | 实现复杂 ×2、算力 | ❌ |
| WLS/hole-filling 后处理 | 补小洞 | **不能补盲带**，插值即造假 | ❌（感知宁 REOBSERVE 不造假） |
| 结构光前端 | 无左带 | 稀疏 + 停走节拍 | 备选（percipio 前端已在库，+8px 系统差见 report.md §5） |

## 4. 判定

- 检测层 2/2 正确（YOLO 两个套袋、conf 0.76/0.83、IoS=0）；几何层 1/2 可测是**物理事实**而非缺陷——理论、实测、三家业界记载三方互证。
- 正确的产品行为就是本仓现行行为：目标贴左近距 → `mask_unavailable` → REOBSERVE → 等待下一个视点。真机轮次中臂移动拍照位后该目标自然可测。
- 若田间工况频繁出现"多目标贴左近距"，优先级排序：调整拍照位使工作目标群居画面中部 > 换 percipio 前端 > 其它。

## 引用

- [IntelRealSense/librealsense #6311：左边界无效深度（官方）](https://github.com/IntelRealSense/librealsense/issues/6311)
- [Intel RealSense Stereoscopic Depth Cameras（白皮书, arXiv 1705.05548）](https://arxiv.org/abs/1705.05548)
- [Intel 社区：Disparity Shift 与 Min-Z/Max-Z 权衡](https://community.intel.com/t5/Items/Intel-RealSense-Touchless-Sensing/filtering-out-invalid-depth-values/td-p/637591)
- [OpenCV cv::StereoSGBM 官方文档](https://docs.opencv.org/4.x/d2/d85/classcv_1_1StereoSGBM.html) · [opencv #9879：视差边缘截断](https://github.com/opencv/opencv/issues/9879)
- [Intel Depth Post-Processing 白皮书（Mouser）](https://www.mouser.com/)
- [Middlebury Stereo Evaluation](https://vision.middlebury.edu/stereo/)
- Hartley & Zisserman, *Multiple View Geometry*, 2nd ed., §11（校正与对应几何）
- Scharstein & Szeliski, "A Taxonomy and Evaluation of Dense Two-Frame Stereo Correspondence Algorithms", IJCV 2002（半遮挡）
