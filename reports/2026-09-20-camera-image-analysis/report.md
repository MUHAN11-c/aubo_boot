# 相机图像分析参数结果报告

| 项 | 内容 |
|----|------|
| 日期 | 2026-09-20 |
| 对象 | `src/peach_stereo`（宿主 SGBM 立体前端）对照 Percipio PS800-E1 设备链 |
| 相机 | PS800-E1，SN 207000152740，GigE 169.254.10.110，散斑结构光双 IR，官方基线 62.2mm |
| 仪器 | 真 SDK 同帧组（彩色+双 IR+设备深度同微秒抓取，`TIME_SYNC=HOST`），gridprobe 系列自写探针 |
| 数据 | `/tmp/sdk3/`（三方裁决帧组×3）、`/tmp/simul/`、`/tmp/final_cmp_*.png`（三张对比图） |
| 状态 | 节点终态=原配准几何回退 + 3 项真修复；4 文件改动在工作区**未提交**（node/launch/README/testing-log） |
| 记录 | 过程全录 `docs/testing-log.md` 09-20 条 |

---

## 1. 设备与标定参数

### 1.1 官方规格（Percipio P-Series / PS800-E1 页）

| 参数 | 官方值 | 本仓实测 | 一致性 |
|------|--------|----------|--------|
| 基线 | 62.2 mm | stereoRectify P1 提取 **62.22 mm** | ✅ 差 0.03mm |
| Z 精度 | 0.51 mm @ 0.5 m；2.01 mm @ 1.0 m | 见 §4.3 | 参照系 |
| XY 精度 | 1.14 mm @ 0.5 m；2.29 mm @ 1.0 m | 见 §4.2 | 参照系 |
| 工作范围 | 0.3–1.0 m（短距结构光） | 台架 ~0.6 m | ✅ 在规格内 |
| FOV | 61°×48° | — | — |
| 深度分辨率 | 最高 1280×960 | IR 对 1280×960，匹配在 640×480 | ✅ |
| 高精度帧率 | 0.87–1.54 fps | 设备深度 2.43 fps；宿主 SGBM **13.6 gps** | ✅ 官方亚帧率=宿主前端存在理由 |

### 1.2 设备标定直读（TY_STRUCT_CAM_CALIB_DATA，非 yaml、非猜测）

| 标定 | 参数值 | 备注 |
|------|--------|------|
| 左 IR（深度参考系）1280×960 | fx=1103.57，fy=1104.22，cx=647.40，cy=490.49；有理畸变 k1=0.1397… | 全模型 |
| 右 IR 外参（相对左） | R 含 **4.2° 绕 Y 会聚**；T=(-62.19, 0.19, 1.72) mm | 平移 x 分量≈基线 |
| **DEPTH_CAM 自有标定** | fx=fy=1044.93，cx=605.31，cy=491.63，**畸变全零，外参单位阵** | 零畸变校正网格，与左 IR 标定完全不同——本轮关键发现 |
| 彩色（2560×1920） | fx=1859.78，fy=1860.05，cx=1304.69，cy=962.722 | 折算 640×480 后 f≈465 |
| 彩色 vs 共用 yaml | 折算后 dcy 差 ≈ **-4.1 px** | 两前端共有小差异，独立债，不单独修 |

### 1.3 校正与配准几何参数（stereoRectify，CALIB_ZERO_DISPARITY，alpha=0）

| 参数 | 值 |
|------|----|
| R1 | **2.64° 绕 Y** |
| P1 | f=1097.74 px，cx=639.05（半分辨率 f=548.9 px） |
| 深度量化 | 0.25 mm/LSB（scale unit），链内 Z=f·B/d 直接算米，REP-103 合规 |
| 设备主点折叠证据 | DEPTH_CAM cx 605.31 vs 左 IR 647.40 → Δ=-42.1 px → -42.1/1044.93=2.31°，与 R1=2.64° 同量级（标定噪声内自洽）：设备把校正旋转折进了 DEPTH_CAM 标定 |

## 2. 算法参数（现行节点）

| 参数 | 值 | 来源 |
|------|----|------|
| SGBM | minDisp=0，numDisp=128（半分辨率≈全分辨率 256），blockSize=5，P1=200，P2=3200，disp12MaxDiff=5，preFilterCap=31，uniquenessRatio=10，speckleWindowSize=100，speckleRange=2，MODE_SGBM_3WAY | `stereo_camera.yaml` + 源码 |
| 匹配分辨率 | processing_scale=0.5（640×480，~13ms/帧） | yaml |
| 时域融合 | avg_k（1/2/4/8；本轮修复为**有效值均值**：逐像素有效计数求商，无效 0 不再进分母） | yaml+修复 |
| 配准 | `TYMapDepthImageToColorCoordinate(z_rect, calibL)`，z 保持校正左系网格**直接**喂入（勿加任何 warp，见 §5） | 修复后终态 |
| SDK 配准语义 | libtyimgproc 1.1.0 反汇编证实：纯针孔、只读内参区、**忽略畸变**、内参按 imageW/intrinsicWidth 折算（半分辨率输入合法）；失败→丢帧+节流告警 | 反汇编 |

## 3. 测量方法参数（为什么可信）

- **同帧组**：彩色+双 IR+设备深度在同一 `TY_FRAME_DATA`（`TIME_SYNC=HOST` 同微秒）——消除场景漂移伪影。纯 live 背靠背不可靠：本台架枝叶数分钟漂 **3~16 px**、54% 特征外点；percipio 自一致噪声底 **±4 px**。
- **孤立边判据**：Sobel>350（彩色强边）、|ΔZ|/2>35mm（深度边）、±22 px 内恰一条彩色边、每 2 行采样；彩色边裁判在本台架稠密纹理下失效（重合平台 ~56%），仅同帧组裁决采信。
- **稀疏深度比对**：全局位移扫描 + 中位 |ΔZ| 谷（tile 匹配在设备深度 10~16% 有效时全灭，弃用）。
- 探针：camport4 预编译 `stereo_grab` 无 tty 必崩（selectDevice 交互），照节点 openDevice 流程自写 gridprobe/2/3。

## 4. 结果

### 4.1 三方配准裁决（相对设备链 D=dev∘calibD，三组同帧组一致）

| 变体 | 位移（彩色 640×480 坐标） | 裁决 |
|------|--------------------------|------|
| **A = z_rect 直接喂 calibL（原实现=终态）** | **+7~+9 px** | ✅ 保留 |
| B = z 反校正回原始左 IR 网格喂 calibL | +22~+26 px | ❌ 已回退 |
| C = z 重投影到 calibD 网格喂 calibD | +25~+26 px | ❌ 否决 |

交叉验证：B 方案 base-vs-fixed 实测 +19.1 px vs 解析预测（R1=2.64°→+19.6 px）吻合——warp 实现正确但**方向错**；C 的 +26 px 证明 calibD 针孔 ≠ 设备真实内部网格（后者不可从标定推导）。结论：校正网格∘calibL 与设备网格∘calibD 本就近似同构，两个"更纯"方案都反而偏离。

### 4.2 位移换算（f_c≈465 px；ΔX = Z·Δu/f_c）

| 位移 | 角度 | ΔX @0.5m | @0.6m | @1.0m | 对照官方 XY 精度 1.14mm@0.5m |
|------|------|----------|-------|-------|------------------------------|
| +8 px（A） | 0.99° | 8.6 mm | **10.3 mm** | 17.2 mm | 系统差 ≈7.5 倍 XY 精度 → 手眼重标定吸收 |
| +23 px（B） | 2.83° | 24.7 mm | 29.7 mm | 49.5 mm | ≈22 倍，不可接受 |
| +26 px（C） | 3.19° | 27.9 mm | 33.5 mm | 55.9 mm | ≈24 倍，不可接受 |

### 4.3 深度一致性（A−D）

| 量 | 值 | 判读 |
|----|----|------|
| \|ΔZ\| 中位 | **4.8 mm**（n=14.6 万像素） | ≈ 0.9 px 有效视差噪声 |
| 理论 1px 视差 @0.6m | δZ=Z²/(f·B)=0.36/(1097.74×0.06222)=**5.27 mm** | 实测落点与单帧 SGBM 匹配噪声理论值相符——**深度面本身一致，差异集中在配准平移** |
| 图像形态 | A−D 残差图暗底+深度不连续处亮边 | 纯针孔+最近邻重采样链在该误差形态下的必然产物，无未解释残差 |

### 4.4 修复验证与性能

| 项 | 结果 |
|----|------|
| 回退逐位校验 | 与 09-20 上午基线 dx=0、dy=0（配准输出逐位一致） |
| avg_k k=4 冒烟 | 近距 (200,450)mm 占比 9.09% vs k=1 的 8.97%——无新增幻影（修复前无效 0 进分母会产生偏近幻影面） |
| color_mode 失配 | 由 WARN+fallback 改为 FATAL 拒启（原静默 fallback 有 2560×1920 全链错位隐患） |
| 配准失败处理 | 丢帧+节流告警（原静默发全零深度） |
| 性能 | 13.6 gps 无回归；新增 `rectify: R1=…deg` 诊断日志 |
| lint | 本轮新增代码零违规（uncrustify 193 行 diff 与 cpplint 行宽为提交前存量债） |

## 5. 结论与处置

1. **A 为唯一正确基线**：图像证据（轮廓阶梯：绿 D→黄 A 差 8px，红 B/品红 C 再偏 3 倍）、SDK 反汇编、同帧组实测三方互证。
2. **禁止再加配准 warp**：B/C 已被真 SDK 实测证伪；节点头注释载明缘由。
3. **+8px 遗留定性**：前端间（单帧 SGBM vs 设备 18-pattern）**系统平移**，非 bug；三连拍稳定 +7~+9px。处置：换 `camera_frontend` 采果前重做一次手眼标定吸收（旋转样系统差可被手眼吸收），或接受 ~1cm 偏差；随距离线性放大（§4.2）。
4. **彩色 dcy≈-4.1px（设备标定 vs 共用 yaml）**：两前端共有，独立记录，不单独修。
5. 深度（Z）维度无几何错误；时域噪声用 avg_k 压制（均值 bug 已修）。

## 6. 证据与复现

| 证据 | 路径（本目录已存 PNG 副本；帧组数据在 /tmp 易失） |
|------|------|
| 轮廓叠加图（裁决主图） | `final_cmp_edges.png`（绿=D 参考、黄=A 现行、红=B 回退、品红=C 否决 + 2.5× 放大） |
| 六格总览 | `final_cmp_overview.png`（彩/D/A/B/C/A−D 残差） |
| 活栈实拍对照 | `final_cmp_live.png`（peach_stereo vs percipio） |
| 延伸分析 | `left-blind-band-theory.md`（左视差盲带理论推导 + RealSense/OpenCV 业界对照 + 缓解矩阵） |
| 算法评估 | `sgbm-sweep.md`（参数扫描→uniq 10→6 落地）、`better-algorithms.md`（WLS 证伪 + 全分辨率精度档 + 学习式 SOTA 路线）、`learned-stereo-live-test.md`（学习式零样本 9 档实测全灭，假设闭环）、`pointcloud-quality.md`（广域调研 + 有效值中值 3×3 落地 + mono 基础模型探针判负） |
| 三方帧组数据 | `/tmp/sdk3/`（A/B/C/D ×3 组）；`/tmp/simul/`（gridprobe2 组） |
| 生成/分析脚本 | `/tmp/make_final_images.py`、`/tmp/final_verdict.py`、`/tmp/grid_verify.py`、探针源 gridprobe*.cpp |
| 过程全录 | `docs/testing-log.md` 09-20 条 |

## 7. 权威资料

- [Percipio P-Series 官方规格页](http://en.percipio.xyz)（Z/XY 精度、范围、帧率、基线）
- [Percipio.XYZ 工业级 3D 相机](https://percipio.xyz)、[Birdwave Atlas: PS800-E1](https://atlas.birdwave.io)（第三方规格交叉）
- camport4 SDK：`TYApi.h`、`TYCoordinateMapper.h`、`TYDefs.h` + `libtyimgproc.so.1.1.0` 反汇编（配准链纯针孔语义）
- [OpenCV 相机标定教程](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)（重投影误差判读）、`cv::stereoRectify`/`StereoSGBM` 官方文档（×16 定点亚像素）
- Hartley & Zisserman《Multiple View Geometry》（Z=fB/d 及视差误差传播 δZ=Z²δd/(fB)）
- [REP-103](https://www.ros.org/reps/rep-0103.html)（单位 SI）
