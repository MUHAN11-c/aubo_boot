# peach_scene_perception

Peach RGB-D scene perception lifecycle node.

深度曝光时间为零、深度不是单通道、焦距非正或内参非有限时，节点跳过该帧的三维推理。
运行时修改深度窗的一端会与当前另一端一起校验。非法深度窗、重力向量和整数下限在参数监听器更新前拒绝。

`target_memory.enable=false` 时按当帧检测序号发布已确认的 `target_N`，不提供跨帧身份关联。诊断 Updater 在进程内复用，cleanup 暂停、configure 恢复，避免重复声明诊断参数。
直接 lifecycle shutdown 同样停止采帧、等待 worker 并释放 ROS 实体；未 configure 或已 cleanup 时重复收尾安全。

`camera_frontend:=percipio`（由整栈透传）时装入 `config/percipio_frontend.yaml`：把 `max_frame_age_s` 放到 2.5 s，避免官方 18 图案一帧被默认 0.50 s 年龄门丢掉。发布袋轴使用身份 EMA（`axis_held`），不跟单帧圆柱 RANSAC 翻转。前端只在 launch 期切换，节点没有热切参数。挂袋 PCA 主轴相对重力超过 45° 时按悬挂回退（`peach_vision_core.bag_landmarks`），不以 18 图案剪影宽度当长轴。

检测用 `model/best.pt`，实例分割默认 `model/FastSAM-s.pt`（Ultralytics 8.3 FastSAM，
bbox prompt）。`sam_model_path` 指向 `mobile_sam.pt` 时仍走 MobileSAM box prompt。
`mobile_sam.pt` 保留在包内作回退，不是启动必检资产。

加载时打官方 `model.info()`。`harvest_state.timing` 的检测/分割周期用
`Results.speed`（preprocess + inference + postprocess，毫秒/图）；
`detect_ms` / `segment_ms` 是这三项之和。decode / geometry / publish / total
仍是节点墙钟。离线口径见 `peach_system_tests` 的 `benchmark_perception.py`。

`InferenceEngine.reset()` 可在模型未加载或已加载时重复调用。YOLO 与分割器
分别在各自的预热/推理锁内清除模型引用，引擎随后清除预热标志；下一次
预热或推理按原配置重新加载。SAM 预测器图像特征缓存随旧模型对象一并丢弃；
FastSAM 无 `set_image` 缓存，批量失败时重跑 `segment()`。检测/分割参数与
候选估计行为保持不变。
