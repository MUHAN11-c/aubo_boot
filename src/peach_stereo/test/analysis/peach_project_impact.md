# peach_stereo 对 peach 项目的影响评估（深度版）

**日期** 2026-09-21 · **证据等级**：代码 file:line + 三轮 A/B 实测（145/146 帧终版轮为主）+ testing-log 09-17 E2E 复核 · **上游材料** [../report/report.md](../report/report.md)、[percipio_vs_peach_stereo.md](percipio_vs_peach_stereo.md)
**问题** 若 peach_stereo 替换 percipio_camera 成为生产相机前端，优势在哪一层真实兑现、哪些已知缺陷未解、真实门限与预算是多少、切前端的硬门槛是什么。

---

## 1. 执行摘要

1. **帧率优势真实但逐层衰减**：相机源 5.6× → 感知层 **3.1×**（实测 7.5fps vs 2.43fps，BoundedWorker 丢旧保新）→ 身份锁定 ~3×（同 18 帧口径）→ **重建收口层：stereo 有一个代码在册、尚未修复的阻断缺陷**（掩膜纳秒精确 stamp 配对，`capture.py:724`）。
2. **精度：近距目标 hh4 全面占优，远距目标 hh4 退化、percipio 直接不可见**（额定 0.4–0.8m）。hh4 在 0.95m 目标上 P95 偏差 19.5mm/MAX 29mm，逼近 pregrasp 30mm 偏置预算。
3. **"pregrasp 3mm 门"是讹传**（沿自早期对比报告）：仓库真实门限=重建精配准体素 **fine_voxel 3mm**、pregrasp 偏置 **30mm**、采集漂移门 **40mm**。本文按真门重算裕度。
4. **切换是部署参数级的**（`camera_frontend:=stereo`，感知/重建/调度/臂零改动），但 **stereo 前端从未在真机闭环过 observe→build 链**（percipio 已过真机 pick1 全链）；头号未解项是 stamp 配对缺陷。
5. **双前端按场景并存是合理终态**：停走节拍+近距拟合用 hh4；覆盖/细节/左缘/远端确认用 percipio。

## 2. 消费者地图（代码级）

| 消费者 | 订阅 | 处理模型 | 证据 |
|---|---|---|---|
| `peach_scene_perception_node` | `/camera/{color,depth}/image_raw`（message_filters ApproximateTime，slop 0.05s，RELIABLE depth10） | **`BoundedWorker(capacity=1, drop_oldest=True)`**——单工有界队列丢旧保新：高帧源不会堆积，永远处理最新帧 | scene_perception_node.py:196-200 |
| move_group（octomap） | `/camera/depth_registered/points` | MoveIt 感知插件，场景碰撞 | sensors_3d.yaml |
| 手眼标定链 | 彩色内参共用 `color_camera_info.yaml`（两前端同 K，逐位一致已验） | — | 标定唯一性整理轮 |
| peach_arm/调度/观测 | **不直接订相机话题**（只经 IDL：`target_observations`/`GraspDecision`/`SceneSnapshot`） | 相机影响全部经感知/重建间接传导 | io.md 消费者列 |

**推论**：相机前端的任何差异（帧率/深度质量/字段）只通过两条链路影响 peach：①感知→重建→`GraspDecision`（几何质量链）；②octomap（碰撞场景链）。臂与调度对前端无感。

## 3. 帧率优势的逐层兑现分解（核心深度项）

| 层 | stereo(hh4) | percipio | 兑现比 | 依据 |
|---|---|---|---|---|
| 相机源 | 13.6gps | 2.43fps | 5.6× | 三轮实测 |
| 感知吞吐 | **~7.5fps**（BoundedWorker 丢旧保新，源>处理上限→满速） | 2.43fps（源<上限→被源限） | **3.1×** | 09-17 A/B 实测（target_reconstruction.yaml:18 注释载明） |
| 身份锁定（registry lock） | 同 18 帧 ≈2.4s | 同 18 帧 ≈7.4s | ~3× | testing-log:278「纯帧数口径 18 帧也差 3 倍」 |
| **重建收口（observe→build）** | **mock E2E FAIL**：`views=1 < min_views=2` | mock E2E FAIL（另一机理） | **0×（未兑现）** | testing-log:315-316 |

**2.8s vs 48s 口径澄清**：48s 含"迟确认的第 3 目标反复重置 settle"的病理性放大；对等口径是 18 帧×帧率差 ≈3×。宣传口径应从"5.7×"修正为**"感知锁定 ~3×；整栈节拍增益待真机复测"**。

**stereo 收口层三连缺陷（09-17 遗留，代码仍在）**：
- ① `missing_mask`：重建采帧要求与深度帧**同纳秒时间戳**的掩膜——`mask_ctx.masks.get(mask_ctx.stamp_ns)`（**capture.py:724，精确查找无容差**）；13.6fps 深度 ≫ 7.5fps 掩膜节奏，配对近乎必失配。遗留修法（testing-log:316）：最近邻 stamp 容差配对或感知掩膜流提频——**未实施**。
- ② `near_duplicate`：相机静止时平移 0.0mm/旋转 0.00° 被去重拒收 → VIEW_FAST 单视策略下静态只能积 1 视，`min_views=2` 依赖 4.4s race 窗内完成补视移动（规划+执行难达成）。
- ③ 13.6fps 下重建 worker 队列持续打满拒帧。

percipio 侧对称缺陷（帧慢+漂移门 63–200mm）在真机已消失（真机 pick1 observe→refit→READY 全链 8.56s 过门，testing-log:362）；mock 期的 0.25 漂移放宽是伪影、已回调 0.04（yaml:31）。

## 4. 精度链路与真实门限（修正讹传）

**仓库真实门限/预算**（非"3mm pregrasp 门"）：

| 门限 | 值 | 含义 | 配置 |
|---|---|---|---|
| 重建精配准体素 | **3mm** | 单帧抖动低于它，多视精配准才不被噪声主导 | target_reconstruction.yaml:44 `fine_voxel: 0.003` |
| pregrasp 轴向偏置 | **30mm** | 预抓取停点距入口的绝对预算（抖动吃这份预算） | grasp_standoffs.yaml `pregrasp_standoff_m: 0.03` |
| 采集漂移门 | **40mm** | 单帧目标漂移超限跳帧（风摆 5-6cm 吸收窗） | target_reconstruction.yaml:31 |
| 近距探测窗 | 0.3m | 掩膜深度统计与采集的有效窗下限 | 重建/感知采集口径 |

**终版轮尾部统计（按稳定目标分组，|Δ| 相对中位）**：

| 目标 | 前端 | z 中位 | P95 | P99 | MAX | 对 3mm 体素 | 对 30mm 偏置 |
|---|---|---|---|---|---|---|---|
| 近袋（主目标） | hh4 (n=141) | 540mm | **4.50mm** | 5.30 | 5.50 | 单帧 P95 略超体素；冻结/多视均值按 std/√N 收敛（std 1.92→8 帧均值 ~0.7mm）✓ | 裕度 5.5× ✓ |
| 近袋（主目标） | percipio (n=142) | 540mm | 9.00mm | 9.00 | 9.00 | 单帧超体素更多；多视同样收敛 ✓ | 裕度 3.3× ✓ |
| **远目标** | hh4 (n=81) | **954mm** | **19.50mm** | 24.20 | **29.00** | 单帧大幅超体素 | **MAX 29mm 逼近 30mm 预算——远距目标上 hh4 精度退化是真实风险** |
| 远目标 | percipio | **不可见**（额定 0.4–0.8m，从不确认） | — | — | — | — | 两种"范围"语义：hh4 量程更远但精度退化；percipio 空间覆盖连片但量程额定 0.8m 截止 |

半径：近袋 hh4 ±0.30mm vs percipio ±0.80mm（MAX）；远目标 hh4 半径 ±8.1mm（退化）。两链 entry 均值逐轴差 0.6/6.0/0.7mm（+8px≈10mm@0.6m 配准系统差内，手眼重标吸收）。

### 4a. 避障链路（细结构/枝）：peach_vegetation 枝掩膜 × 深度四档实测

用 `peach_vegetation`（GPU Frangi 枝/叶分割）的 branch_mask 评深度流对细结构的可用性（工具 `test/scripts/branch_analysis.py`，图 `test/report/branch/`、`fig_branch_{depth,overlay}.png`；掩膜为彩色系、两前端同质，差异全在深度侧）：

| 档位 | 枝上覆盖 | 粗枝覆盖 | **细枝覆盖** | 细节密度(3×3 std) | 帧率 |
|---|---|---|---|---|---|
| **hh4 现行（med3+tk3，部署档）** | **0.604** | **0.569** | **0.652** | 12.9mm | 13.5gps |
| hh4 med0+tk1 | 0.585 | 0.553 | 0.629 | 13.1mm | 13.5gps |
| hh4 全分辨率 med0+tk1 | 0.493 | 0.454 | 0.544 | 15.1mm | **4.3gps** |
| percipio（n=7 部分） | 0.540 | 0.482 | 0.622 | **16.0mm** | ~1–2fps |

**结论（回答"hh4 能否更细化"）**：
1. **避障口径下 hh4 现行档已是覆盖最优**（0.604，细枝 0.652）——tk3 时域中值在补细枝深度闪烁，**为避障关滤波/改档是负收益**（med0+tk1：细节仅 +2%、覆盖 −3%）。此前"percipio 覆盖更好"是全图连片性口径（无左盲带）；枝掩膜×有效深度口径下 hh4 反超。
2. **细节上限结构性受限**：全分辨率换 +18% 细节，但覆盖 −18%、帧率 −68%、且 numDisp=128 在全分辨率下 z_min≈0.53m 挺进 0.3–0.8m 工作区（近枝失深）——坏交易，不采用。细节密度 percipio 仍最高（16.0），细枝几何精度敏感场合（单帧、无停走约束）用 percipio。
3. 运维发现（本轮新增）：**peach_vegetation 换相机节点后订阅楔死（收流不吐掩膜），每次前端/档位切换后必须重启 veg**；且其 launch 的 autostart 事件与 main 自激活确定性互杀（`ros2 run` 直起可用、勿发 lifecycle 命令）——两缺陷已记档待修。

## 5. 资源与故障面

| 项 | 事实 | 影响 |
|---|---|---|
| CPU | stereo 节点常驻 ~241%（2.4 核 / 全机 20 核）+ 感知 GPU | 12% 机器预算，可接受；与其他 CPU 密集组件并存无争用风险记录 |
| 掉线恢复 | **无重连**（percipio 有 offline 线程 + `/camera/device_event`） | stereo 掉线=人工重启节点；活度靠 observability ingest_liveness/话题年龄（不在 lifecycle bond 名单） |
| 投递层怪癖 | raw RELIABLE 大图：`ros2 topic hz` CLI 恒 0 帧（echo/rclpy 正常；09-17 实测投递抖动 0.07–1.49s） | **不影响感知**（BoundedWorker 丢旧保新 + 7.5fps 消费实测正常）；影响诊断工具与低容忍订户（如 bag record）——死订户还会反堵发布（09-21 终轮 4 recorder 事故，已沉淀清理规则） |
| 停栈 | 析构复位激光+关设备（干净） | 少一类 SHM 自伤；SHM 仍例行清 |
| 依赖 | 头文件/运行时 .so 依赖 percipio_camera 源码树 vendored SDK | percipio 包不可移除 |

## 6. 跨前端耦合参数（切前端=改这些的墙钟语义）

| 参数 | 现值 | 前端耦合点 |
|---|---|---|
| `tentative_ttl_frames` | 20 帧 | 按帧计 TTL：2.43fps≈8.2s vs 7.5fps≈2.7s 墙钟——09-17 已按双前端调平（原 8 在 stereo 下弃置过快） |
| `max_views` | 24 | 帧栈上限；注释已载明双前端有效视角口径（percipio 4-6 / stereo 密度 3×靠 view_filter 去重） |
| 重建 race 窗 | 按实测帧率自适应 | 2.0fps→15.1s vs 13.4fps→**4.4s**——stereo 窗内完成补视移动更紧（§3 缺陷②的放大器） |
| `sync_slop` | 0.05s | stereo TIME_SYNC=HOST 同微秒天然满足；percipio 设备时戳近似满足 |
| 手眼外参 | active.yaml | 按前端链标定——切换必须重标（+8px 系统差） |

## 7. 真机验证状态矩阵

| 链路段 | percipio | stereo(hh4) |
|---|---|---|
| 台架感知 A/B（本轮） | ✓ 146 帧 | ✓ 145 帧 |
| mock 整栈 E2E（09-17） | observe→build **FAIL**（漂移门，真机已证伪为 mock 伪影） | observe→build **FAIL**（stamp 配对+去重互斥，**真机未复测**） |
| 真机 observe→refit→READY（pick1） | ✓ 8.56s 全链（testing-log:362） | **从未执行** |
| 真机采果 | 真机验收挂账中 | 未开始 |

## 8. 切换决策与硬门槛

**建议维持**：停走节拍生产用 stereo（hh4+uniq6+med3+tk3）的**方向不变**，但把"可直接切"修正为带硬门槛的路线：

1. **【阻断】修复掩膜 stamp 精确配对**（capture.py:724 → 最近邻容差配对或感知掩膜流提频）+ 复审 near_duplicate/min_views/race 窗联动——修完先 mock E2E 过 observe→build，再真机闭环一次（对齐 percipio pick1 口径）。
2. **【阻断】手眼标定按 stereo 链重做**（吸收 +8px≈10mm@0.6m 系统差）。
3. **【硬约束】视点规划目标居中**：左缘 15% 列结构性全盲（0.000 vs 0.035）；远目标（>0.8m）要么不派给 hh4 要么接受 19–29mm 级散布（逼近 30mm 偏置预算）。
4. 激光温升确认；覆盖/细节敏感场景（远端枝条、左缘目标、单帧大范围）留在 percipio 档——双前端并存。
5. 宣传口径同步：整栈速度增益以真机复测为准（感知锁定层 ~3× 是当前有据上限，2.8s vs 48s 含病理个例）。

**对 peach 代码库的本轮改动清单**（相机侧）：confidence 偏移修复（@20/step24）、yaml `/**:` 键匹配修复、io.md 同步、test/ 归档与录制链（composite 窗+自检门）。感知/重建/调度/臂零改动。
