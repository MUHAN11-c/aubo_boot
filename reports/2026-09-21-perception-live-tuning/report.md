# 感知管线真实数据迭代优化轮报告（2026-09-21，stereo 活流）

**范围**：用户目标「相机实连 + RViz2 可视化持续迭代；成熟方案简洁优化；性能优秀；结果稳定；抗干扰强」。本轮 = 几何核优化收口（bebfdb4）+ stereo 活流 E2E 取证 + 成熟库对比评估。分支 `test/20260909-field-traj`。

**环境**：stereo hh4 档活流（13.7gps，域 77，昨晚 e2e 起的相机栈全程保留未动）；感知单节点调参模式（`/tmp/perception_tune.launch.py`：output_frame 置空→T=I、tf_status=ok、身份链照跑；gravity_mode=fixed；publish_debug_image 由 TUNE_DEBUG 环境开关）。台架场景：室内+真枝+2 套袋果。

## 1. 性能（live 实测，2 目标，N≈2.4k 点/目标）

| 段 | 本轮 live（stereo 2 目标） | W0 基线（percipio 1 目标） | 昨晚争用态 |
|---|---|---|---|
| detect | 4.9–5.6 ms | 4.0–5.4 | 24–38 |
| segment | 25.9–28.5 ms | 17.2–26.5 | 44–60 |
| geometry | 85.3–99.3 ms | 34.6–61.6 | 350–600 |
| **total** | **127.3–144.7 ms** | 84.3–105.4 | 450–700 |
| 有效帧率 | **7.5–8.0 fps** | 2.43–2.54 | 1.4–2.2 |

- 有效 8fps = 容量 1 worker 丢弃后稳态（源 13.7gps，处理 ~8gps）。收齐窗 10 帧约 1.3 s（原 percipio 设计 25 s 窗），节拍余量充足。
- 昨晚 350–600ms 确证为**争用放大**（同机 SGBM/观测栈把 CPU 打到 96%），非代码退化；本轮空载+优化后 geometry ≈ 42–50ms/目标（微基准单目标 35.9ms）。
- **debug 开销 ≈ 噪声级**：开/关两轮 total 差在场景方差内（127↔145 互有高低）。PF-3 零拷贝路线已兑现——**订户门控（get_subscription_count）按数据裁定不做**，避免无收益代码。

## 2. 稳定性与抗干扰（60s 取证，485 帧 @8fps）

- **身份零抖动**：`idset_changes=0`，双目标 485/485 帧全程在册、无 stale/out_of_view。
- 视觉质量（r0 debug 图，runs/tune_20260921/r0/debug.png）：双目标框贴合、掩膜边界贴边、轴/剪切线合理、无灰框误检；target_1（小目标，YOLO conf 0.51）掩膜略粗糙但稳定。

## 3. 发现（记录在案，未改行为）

1. **单帧门 ACCEPT 从不触发**：调参模式与昨日生产栈（bag 175240，56/56）均为 100% REOBSERVE。下游 `refine.py:84` 偏好 ACCEPT 但有 non-REJECT 兜底→良性但偏好分支实际死码，观测指标面上长期 REOBSERVE。
2. **`low_valid_depth` 语义疑点**：valid_ratio 按**检测框 ROI**均值计（pose_pipelines._prepare_estimate_inputs），背景超窗即拉低——袋外背景多的框恒 <0.40 命中该 flag（生产 56/56）。若按掩膜∩valid/掩膜计，语义更贴「目标有效深度」。
3. **`travel_too_short` 恒命中**（生产 56/56）：entry/行程公式与实测袋长组合下 travel<0.05m 常态化，同属「良性但指标失真」。
4. 调参模式 gravity=fixed 提示，极性类 flags（taper_polarity_swapped 等）受重力方向影响，本轮台架值不代表生产行为——生产口径已用昨日 bag 单独取证（见发现 1-3）。

## 4. 成熟方案评估（先评估后换）

| 项 | 结论 | 证据 |
|---|---|---|
| 匈牙利指派 | **已在用 scipy** `linear_sum_assignment`（identity.py:20,92） | 零工作 |
| pyransac3d 0.7.0 圆柱拟合 | **否决**：轴误差 15.9°（自研 0.051°，同合成真值）、320ms vs 38.5ms（8×）、无种子确定性（重跑轴差 27°，违本仓种子可复现约定） | /tmp/run_pr3d.sh，隔离 --target 安装未动主 venv |
| Open3D voxel 降采样替代预筛 | **不采纳**：voxel 折叠按哈希序选点非种子化，破坏确定性；预筛打分已覆盖同动机（O(iter×512+16×N)） | 分析性结论 |
| estimate_normals 换 o3d | **保留自研**：深度跳变门（30mm）是领域逻辑，o3d 无等价 | — |

自研拟合栈（scipy+numpy，点+法线构造、Eberly 抛光、种子化）在精度/速度/确定性三轴均优于候选成熟库——本轮「成熟方案简洁化」的正确动作是**保留+已完成的复杂度优化**，而非替换。

## 5. 产物与复现

- 调参闭环工具（/tmp，本机）：`perception_tune.launch.py`（相机系模式）、`run_perception_tune.sh`（域 77 启动器）、`run_stab.sh`（60s 稳定性取证）、`run_pr3d.sh`（成熟库对比）。
- 采样数据：`runs/tune_20260921/r0/`（debug.png + state.json）。
- 用户侧 RViz：`rviz2 -d src/aubo_e5_moveit_config/rviz/moveit.rviz`（调参模式 markers/点云在相机系，Fixed Frame 改 `camera_color_optical_frame` 即可看；生产栈直接用原配置）。
- perf_baseline.json 新增 `live_tuning_stereo_2targets_ms` 段（附 provenance）。

## 6. 遗留（按优先序）

1. 发现 2/3 的 flag 语义修正（low_valid_depth 改掩膜分母；travel 公式复核）——改的是门控指标口径，须真机验收轮一起做。
2. 若要吃满 stereo 13.7gps（total 需 ≤73ms）：候选=锁定目标几何降频重估（轴/直径缓存在注册表）或逐目标并行估计——均为行为契约级改动，单独立轮。
3. ACCEPT 死分支：要么收紧 flags 让 ACCEPT 可达，要么 refine 偏好逻辑改口——与 1 同轮。

## 清理

调参感知节点已停（pgrep 复核 0 残留）；stereo 相机栈为用户活数据源保留未动。
