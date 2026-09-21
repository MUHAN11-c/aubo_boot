# peach_stereo

PS800-E1 **主机侧单图案立体深度**相机节点：`percipio_camera` 的替代前端，
输出**话题同构**的 RGB-D，感知/重建零改动接入。依据与数据见
`docs/testing-log.md` 09-16/09-17/09-18/09-20/09-21 各条目（根因 → 解锁 → 单图案验证 → 实时演示 → 需求评估 → 感知 A/B → 配准几何/SGBM/时域参数定档）。

## PS800-E1 规格档案（本机实测，2026-09-16/17）

| 项 | 值 | 来源 |
|----|-----|------|
| 型号 / SN | Percipio PS800-E1 / 207000152740 | 设备直读 |
| 深度原理 | 散斑结构光双目（每帧 18 幅图案，image number=18 且 (18+1)/2×match5=47.5 顶满算力约束 <48） | SDK 直读 + 官方参数文档 |
| 固件 / SDK | tycam R3.6.49 / camport4 4.2.10，legacy GigE2.0 协议 | 日志 |
| 官方深度帧率 | **0.8 fps @ 全部分辨率**（1280×960/640×480/320×240 同值）；渲染延迟 1781 ms | en.percipio.xyz 产品页 |
| 实测深度帧率（设备端） | 2.43 fps（411.5 ms/帧，设备日志 `got one frame` 间隔证实；官方口径=IR 采集率÷图案数） | 台架 |
| 官方深度量程/精度 | 400–800 mm；Z 0.51 mm@500 mm、XY 1.14 mm@500 mm | 产品页 |
| 彩色 | yuyv 可选 640×480–2560×1920；**单流 15.00 fps**；不设档位默认 2560×1920（会拖死帧组到 2.5 gps） | 台架 |
| 双目 IR | 1280×960 mono8；独立模式采集率随曝光 14.8–25 fps；**L+R 同帧硬件同步** | 台架 |
| 立体几何 | 基线 **62.2 mm**，f≈1098 px @1280 宽；右相机外参=相对左 IR（头文件注释明示） | 设备标定直读 |
| 本链路深度 | 主机 SGBM：**13.5–13.7 fps**；时域噪声 1.49 mm@0.8 m（k=1）/0.96 mm（k=5），设备端对照 1.13 mm；与设备深度同目标互证差 **5 mm**；09-20 hh4 档：谷底一致性 8.75→7.50 mm、粗糙度 1.03→0.94 mm | 台架 |
| 本链路近距下限 | SGBM 理论 ≈0.27 m（f·B/numDisp），低于设备额定 0.4 m；近距光学质量未标定 | 计算 |
| 激光投射器 | auto ctrl=1/power=50 默认；**独立 IR 模式须手动解锁**（auto=0+power=100，否则输出非光信号底噪）；laser 设置跨连接自动复位、**曝光值跨连接残留** | 台架 |
| 帧组时间戳 | TIME_SYNC=HOST 后彩色/L/R 同微秒（纪元 µs），RGB-D 对齐天然成立 | 台架 |
| 深度口径 | uint16 × 0.25 mm（与感知 `depth_scale_unit: 0.25` 对齐，零配置） | 设计 |
| 配准几何 | z(校正网格)直喂左 IR 标定；与设备配准链同帧组实测差 **+8px**（≈10mm@0.6m）；反校正/换 DEPTH_CAM 标定更差（+23/+26px），勿改 | 09-20 真 SDK 三方实测 |
| DEPTH_CAM 标定 | fx=fy=1044.93、cx=605.31、畸变全零（零畸变校正网格，区别于左 IR 标定）；设备内部网格不可从标定推导 | 设备直读 |
| SGBM 匹配参数 | `num_disparities=128` 已证最优（0.3m 感知下限 ⇒ d≥113.8 ⇒ 下一档 112 会切 305mm>300mm）；`uniqueness_ratio=6`（10→6：同帧组扫描覆盖 +2.7pp、精度门内、帧率 13.5gps 不变）；`mode=hh4` 端到端 12 变体矩阵帕累托最优（精度 8.75→7.50mm、鬼影 −0.17pp、粗糙度 −9%、13.7gps 无回归；HH 同质量慢 2×；时域 k3（视差域批式）/全分辨率档记档未落地）；**minDisparity>0 已证伪**（远背景强制错配成中距，谷底精度 25×劣化） | 09-20 参数扫描 + 端到端矩阵（reports/2026-09-20-camera-image-analysis/sgbm-sweep.md、reports/2026-09-20-e2e-stereo-optimal/） |
| 滑窗时域中值 | `temporal_k`（1=关/3/5，非法拒启；部署 3）：配准后彩色网格 k 帧逐像素有效中值，每帧照常发布不除率；entry std z 0.41→0.21mm、袋半径 std 0.27→0.14mm（均 −48%）、覆盖 +0.4pp、13.7gps 无回归（~3ms 开销在采集线程内）；tk=5 半径更优（±0.06mm）未取 | 09-21 live A/B（testing-log 09-21 续） |
| 避障（枝细结构） | peach_vegetation 枝掩膜×深度四档实测：**现行档（scale0.5+med3+tk3）枝上覆盖全档最优**（0.604/细枝 0.652，tk3 补细枝闪烁）——为避障关滤波/上全分辨率皆负收益（全分辨率：细节 +18% 但覆盖 −18%、4.3gps、z_min≈0.53m 侵入工作区）；细节密度 percipio 16.0 vs 本链 12.9mm，细枝精度敏感的单帧场合用 percipio | 09-21 避障轮（test/report/branch/ + 影响评估 §4a） |
| 已知限制 | jpeg 插件编不了 mono16（`/compressed` 空载荷），深度压缩通道用 `compressedDepth`（12 字节容器头）或 `zstd`；raw RELIABLE 大图本机 FastDDS 投递坍塌 | 台架 |
| 无效参数（本机实测勿再调） | image number / SGPM 相位数 / 深度分辨率 / frame_rate 请求 / 深度模式 IR 曝光——均不改深度采集帧率 | 09-16 全实测 |

## 工作原理

```
解锁激光(关自动控制+拉功率) → 彩色640x480 + 双目IR 1280x960 同帧组(~13.7 组/s)
→ stereoRectify 校正 → SGBM(HH4, 半分辨率 ~54ms, 相机节拍限速) → 深度(校正左系, mm)
→ TYMapDepthImageToColorCoordinate 配准到彩色几何（temporal_k>1 时滑窗逐像素有效中值，每帧照常发布）
→ 与 percipio 同构：color/image_raw + depth/image_raw(uint16×0.25mm)
   + {color,depth}/camera_info + depth_registered/points（stereo 多 confidence 字段，见话题节）
```

- **配准几何（09-20 真机复核，勿加反校正 warp）**：SDK 按所传标定针孔解释深度输入网格（忽略畸变、按分辨率折算）。同帧组真 SDK 三方实测：z(校正网格)直接喂左 IR 标定与设备深度配准链最贴合（+8px）；反校正回原始左 IR 网格(+23px)或重投影到 DEPTH_CAM 标定网格(+26px)都更差——设备已把校正旋转折进 DEPTH_CAM 标定主点，两套网格近似同构。源码注释有完整论证。
- **帧组硬件同步**：彩色/左 IR/右 IR 同时间戳（纪元微秒，TIME_SYNC=HOST），RGB-D 时间对齐天然满足感知 `sync_slop 0.05s`。
- **深度口径与 percipio 一致**：uint16 × `depth_scale_unit 0.25`(mm)，感知端默认参数直接可用。
- **质量-帧率旋钮**：`avg_k`（1→~13.7fps/1.49mm；4→~3.4fps；实测 k=5≈0.96mm@0.8m，优于设备 18 图案单帧 1.13mm）。k 帧均值为**有效值均值**（无效 0 不进分母，逐像素按有效计数求商）。
- **空间去噪**：`median_ksize`（0=关/3/5，默认 3）深度有效值中值——只去噪不补洞（中心无效保持无效、无效邻居不进中值）；3×3 过同帧组三门且点云局部平面粗糙度 −12%（1.17→1.03mm，低于设备链 1.08mm）；联合双边/引导滤波被精度门证伪勿再引入（reports/2026-09-20-camera-image-analysis/pointcloud-quality.md）。
- **路径聚合模式**：`sgbm.mode`（3way/sgbm/hh/hh4，默认 hh4）。hh4 相比 3way 全指标占优（09-20 端到端矩阵，同帧组 vs 设备 18 图案链）：谷底 8.75→7.50mm、鬼影 1.65→1.48%、粗糙度 1.03→0.94mm、覆盖 −0.34pp 门内，匹配 10.9→54ms 仍装进相机 73ms 帧间隔（13.7gps 无回归）。hh 同质量但慢 2×；视差域 k3 时域融合与全分辨率档实测有增益但未落地（率÷3 / 粗糙度劣化，数字见 reports/2026-09-20-e2e-stereo-optimal/）。
- **滑窗时域中值**：`temporal_k`（1=关/3/5，部署 3）在配准后彩色网格深度上做 k 帧逐像素**有效值中值**（无效 0 不进窗），每帧照常发布不除率——区别于 avg_k 批式（率÷k）；续四否决的是视差域批式 k3（率÷3），与本逐帧滑窗是两个设计点。live A/B（09-21）：entry std z 0.41→0.21mm（−48%）、袋半径 std 0.27→0.14mm（−48%）、场景中位 std −48%、覆盖 +0.4pp、13.7gps 无回归。中值是序统计量、域无关，顺带稳定配准闪烁（参考 librealsense temporal filter 多帧融合）。
- 相比设备端深度（2.43fps）：**~5.6 倍帧率**，代价是单图案噪声 +30%（对 3–8mm 尺度的感知管线裕度充足，见评估条目）。
- 与 percipio 前端的已知系统差：配准输出 +8px（≈10mm@0.6m）；**切前端采果前重做一次手眼标定**可吸收。

## 话题（namespace=camera，与 percipio_camera 对齐）

| 话题 | 类型 | 说明 |
|------|------|------|
| `color/image_raw` | bgr8 | 彩色，与深度同帧组时间戳 |
| `depth/image_raw` | 16UC1 | **已配准到彩色几何**，单位 0.25mm |
| `color/camera_info` | CameraInfo | 按彩色流分辨率折算的 K |
| `depth/camera_info` | CameraInfo | 与彩色同 K（配准后），frame=`camera_depth_optical_frame` |
| `depth_registered/points` | PointCloud2 | 配准彩色点云（与 percipio `color_point_cloud_enable` 同名；stereo 多一个 `confidence` 字段，见下） |

静态 TF：`camera_link → {color,depth}_frame → 各 optical`（与 percipio 同名链；depth 帧 identity 等价）。

**`confidence` 字段布局**：每点 FLOAT32，`x/y/z@0/4/8`、`rgb@16`、`confidence@20`、`point_step=24`。语义：`temporal_k>1` 时=窗内采样占比×取值一致性（`1/(1+(hi−lo)·0.25/med_mm)`），与发布深度逐像素对齐；`temporal_k=1` 时恒 1.0。**勿把 confidence 放 offset 16**——`setPointCloud2FieldsByString("xyz","rgb")` 的 rgb 槽就在 16，重叠会互相覆写（09-21 线上实测颜色被置信度损坏后修正）。消费方按字段名读（move_group octomap / RViz / GraspNet `read_points_numpy` 均兼容）。

## 使用

```bash
colcon build --packages-select peach_stereo
# 与 percipio_camera 互斥（相机独占）：先停 percipio 相机栈
ros2 launch peach_stereo stereo_camera.launch.py
ros2 topic hz /camera/depth/image_raw   # 预期 ~13.7 Hz（本机 topic hz CLI 有恒 0 帧怪癖，健康验证用 30 帧探针，见 docs/testing.md 冒烟节）
```

参数见 `config/stereo_camera.yaml`（nav2 式：部署值即事实源，键名冻结）。顶层键必须用 `/**:` 通配——节点全名是 `/camera/peach_stereo_camera_node`，用节点名做顶层键**从不匹配**且不报错（所有部署值静默不生效，09-21 才修复的存量坑，此前默认值碰巧一致未暴露）。

## 边界与注意

- **与 percipio_camera 的端到端对比档案**（2026-09-20/21：函数/流程级分析、A/B 实测、窗口录像、降级态排查与切换运维规则）见 [test/](test/README.md)。

- **激光满功率持续点亮**（`laser_power: 100`）：长时间运行的热管理未验证；停栈自动复位（auto=1/50）。台架连续观察为宜，田间长时间挂机前先做温升确认。
- 依赖 `percipio_camera` 源码树 vendored 的 camport4 SDK（头文件未随包安装，CMake 直接引用源码路径）；运行时 `.so` 由 `install/percipio_camera/lib` 解析。
- 大消息投递：raw RELIABLE 大图在本机 FastDDS 下有坍塌现象（testing-log 09-16）。Jazzy `image_transport` 无法按话题关插件；整栈 catch-all 会订 jpeg/compressedDepth，16UC1 与 bgr8 交叉编码每帧 ERROR。本节点只发 raw Image（感知订 raw）。
- 默认不进 harvest_system（`camera_frontend` 默认 percipio）。整栈切 stereo：`camera_enabled:=true camera_frontend:=stereo`。
- 深度工作范围：设备端额定 0.4–0.8m；本链路 SGBM 理论下限 ~0.27m（近距光学质量未标定）。
