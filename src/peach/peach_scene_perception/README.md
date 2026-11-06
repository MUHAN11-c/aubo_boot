# peach_scene_perception

RGB-D 场景感知 Lifecycle 节点：输出检测、掩膜、单帧几何与带稳定身份的目标观测；不发运动，也不把单帧 ACCEPT 当作接触许可。

源码分工、处理链和生命周期见 [架构](doc/architecture.md)，话题、BeginScene、参数来源和 QoS 见 [接口](doc/io.md)。以下保留各项实现边界与历史复核口径。

深度曝光时间为零、深度不是单通道、焦距非正或内参非有限时，节点跳过该帧的三维推理。
运行时修改深度窗的一端会与当前另一端一起校验。非法深度窗、重力向量和整数下限在参数监听器更新前拒绝。

`sync_slop_s` 必须大于 0；Jazzy 近似同步器使用严格小于容差，零容差即使三路 stamp 相同也不会配对。同步器在构造期读取此值，因此参数只允许启动时覆盖，运行期写入会被拒绝，修改后须重启节点。

`target_memory.enable=false` 时按当帧检测序号发布已确认的 `target_N`，不提供跨帧身份关联。诊断 Updater 在进程内复用，cleanup 暂停、configure 恢复，避免重复声明诊断参数。
直接 lifecycle shutdown 同样停止采帧、等待 worker 并释放 ROS 实体；未 configure 或已 cleanup 时重复收尾安全。

收集窗口仍要求最少帧数及连续无新增确认 ID。缺掩膜/深度而没有三维锚点的
`untracked_*` 只保留拟合诊断，不进入注册表，也不属于正在积累确认的目标。
当本帧已有确认目标且注册表没有活跃待确认项时，这类占位不再阻止稳定关窗；
暂时丢失深度的待确认项须等待其确认或 TTL 淘汰，歧义及溢出身份仍阻止关窗。
只有占位或只有历史确认时继续等待原超时，避免提前锁空集。
内部 `confirmation_pending` 缺省为保守等待，不改变 ROS 接口、目标资格或执行许可。

`camera_frontend:=percipio`（由整栈透传）时装入 `config/percipio_frontend.yaml`：把 `max_frame_age_s` 放到 2.5 s，避免官方 18 图案一帧被默认 0.50 s 年龄门丢掉。前端只在 launch 期切换，节点没有热切参数。挂袋 PCA 主轴相对重力超过 45° 时按悬挂回退（`peach_vision_core.bag_landmarks`），不以 18 图案剪影宽度当长轴。

两种相机前端发布的候选均保留同一曝光的几何：`translation_direction` 为 `bag_bottom` 到 `bag_neck` 的单位方向，与 `entry_pose` 旋转的 Z 轴一致。身份表继续对位置和轴做 EMA，供跨帧身份关联及记忆锚点使用；EMA 不覆写当帧候选方向，也不作为时间稳定性或运动许可的证据。感知 `ACCEPT` 仍只表示单帧初值可用。

袋轴在各向异性 1.2 和重力角 45° 的边界附近，几何参考与资格分开计算。先保留
原同曝光的完整评估；若其轴退化，再以当前点云的条件轴重新构造截面、端点、
2D 长度补全及入口。新的资格只能维持或降低原资格，直径保留两种构造的较大值，
该标量不代表两种轴线和中心的完整空间包络并集；
退化轴的误差代理至少 20°。原来满足 PCA 条件的帧直接返回原结果。原始 mask、
深度、身份、曝光和时效不变，没有跨帧平均、驻留等待或旧几何续签。

`axis_conditioned_from_current_cloud` 表示采用上述几何参考；原重力先验资格标记
仍保留。若完整旧评估要求更严格状态，追加 `axis_conditioning_qualification_preserved`。
内部诊断还保留 `axis_conditioning_weight` 和 `axis_qualification_reference_status`；
既有 ROS fitting 的 `theta_err_deg`/`axis_confidence` 反映保守误差代理。
连续性和点到轴残差不等于物理轴精度；纸袋的圆柱径向残差也可能增大。

此变化同时进入身份匹配的中心/轴、`target_observations`、`initial_pose` 和
Web/RViz。重建可从 REOBSERVE 候选刷新已绑定目标的 hint 和掩膜中心，所以弱轴
目标的重建输入会改变；独立点云 refit、融合质量、轴冲突、GraspDecision 身份与
时效门仍须通过。不能把共享 `axis_hint` 算法未改解释为下游模型逐值不变。

检测用 `model/best.pt`，实例分割默认 `model/FastSAM-s.pt`（Ultralytics 8.3 FastSAM，
bbox prompt）。`sam_model_path` 指向 `mobile_sam.pt` 时仍走 MobileSAM box prompt。
`mobile_sam.pt` 保留在包内作回退，不是启动必检资产。

FastSAM 的框与掩膜匹配按当前帧缓存每个掩膜的总面积，避免多框比较重复扫描同一张掩膜；框内交集、IoU 门、最小面积严格大于门与匹配顺序不变，缓存不跨帧保留。

袋地标成功且给出正 `d95_m` 时，直接使用其端点和直径，不再计算随后会被覆盖的局部轴分位带。地标失败，或 `d95_m` 为零、负值、NaN 时仍执行原回退路径；最终端点延伸、极性、同帧轴/入口姿态和全部门控保持不变。共用几何核只缓存确定性 RANSAC 采样索引，不缓存点云、拟合结果或候选资格。

2026-11-05 前一轮离线优化复核使用同一原始 bag 的 401 个完整曝光、1604 个目标，基线、仅索引缓存与完整优化的全量结果逐值一致；相同曝光的四目标 CPU 几何均值为 29.224 / 25.094 / 23.626 ms。此计时包含掩膜前处理和完整袋估计，不含检测、分割、相机、ROS 发布；不能直接作为实景 FPS 或宿主总耗时。该轮仅减少重复计算；本轮条件轴和资格下限的独立证据保存在 `runs/peach_followup_20261105/reports/perception/`，不沿用旧数值等价结论。

加载时打官方 `model.info()`。`harvest_state.timing` 的检测/分割周期用
`Results.speed`（preprocess + inference + postprocess，毫秒/图）；
`detect_ms` / `segment_ms` 是这三项之和。decode / geometry / publish / total
仍是节点墙钟。离线口径见 `peach_system_tests` 的 `benchmark_perception.py`。

`InferenceEngine.reset()` 可在模型未加载或已加载时重复调用。YOLO 与分割器
分别在各自的预热/推理锁内清除模型引用，引擎随后清除预热标志；下一次
预热或推理按原配置重新加载。SAM 预测器图像特征缓存随旧模型对象一并丢弃；
FastSAM 无 `set_image` 缓存，批量失败时重跑 `segment()`。检测/分割参数与
候选估计行为保持不变。
