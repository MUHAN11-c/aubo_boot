# peach_target_reconstruction 接口

跨包名字与 QoS 对照 [interface_manifest.yaml](../../peach_interfaces/config/interface_manifest.yaml)。节点默认名 `peach_target_reconstruction_node`；业务 topic 为绝对名，`~/...` 的 action/service 随节点 namespace 和名称解析。

## 输入

- RGB、深度、CameraInfo：`camera.color_topic`、`camera.depth_topic`、`camera.camera_info_topic`，部署为 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`，三路 RELIABLE / VOLATILE / KEEP_LAST(10)，近似同步队列 10。深度须彩色配准；uint16 按 depth_scale_unit 换成毫米，浮点按米换算。零深度 stamp、布局/尺寸/内参异常拒收。
- `/peach/perception/initial_pose`：BagGraspCandidateArray，RELIABLE / TRANSIENT_LOCAL / depth 1；绑定初值和当前轴 hint。
- `/peach/perception/target_observations`：PeachTargetObservationArray；`/peach/perception/diagnostics`：BagFittingArray；`/joint_states`：JointState。均 RELIABLE / VOLATILE / depth 10；分别提供目标/掩膜、类型记忆和曝光关节证据，关节历史缓存最多 8192 条。
- `/peach_supervisor/state`：HarvestState，RELIABLE / TRANSIENT_LOCAL / depth 1；执行目标与批次目录绑定权威。
- TF：查询深度曝光时刻的 `frames.base_frame ← depth.header.frame_id`，不回退 latest；工具当前 TF 另用于预抓取残差观测。

## 输出

以下后缀均位于 `/peach/reconstruction/`。除 preview_cloud 外均 RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)：

- `status`：String，内容为 collector 状态 token；`diagnostics`：ReconstructionStatus；`diagnostics_debug`：String JSON。前两者分别是轻量状态和类型化健康/采集摘要，debug 保存逐帧、配准、融合和耗时明细。diagnostics offered deadline 为 1.5 秒，超期事件只记录日志。
- `grasp_decision`：GraspDecision，模型身份、冻结有效窗、能力与诊断预算；`pregrasp_verification`：PregraspVerification，当前工具相对模型的残差观测。许可、残差与感知 ACCEPT 不能互相替代。
- `refined_pose`：BagGraspCandidateArray；`refined_axis`：Vector3Stamped；`refined_diagnostics`：BagFittingArray；`shape_hypothesis`：ShapeHypothesis。几何在 base_frame；场景障碍只从精化候选读取目标胶囊。
- `local_cloud`、`tsdf_cloud`：PointCloud2；`markers`：MarkerArray。内容变更与最小发布间隔可节流；空结果用于清屏。
- `preview_cloud`：PointCloud2，BEST_EFFORT / TRANSIENT_LOCAL / depth 1，供显示；有 TSDF 时优先显示 TSDF，否则显示采集云。

独立系统健康发布到 `/diagnostics`，与上面的类型化 `reconstruction/diagnostics` 分开。1 Hz 心跳尝试重发 status、diagnostics、diagnostics_debug、grasp_decision 和 pregrasp_verification；状态锁忙时跳过并在健康诊断 heartbeat_skipped_busy 计数。心跳不重新 refit，也不延长同模型 valid_until。

## Action 与调试服务

`~/build_target_model` 使用 `peach_interfaces/action/BuildTargetModel`。Active 才受理，同一时间只允许一份 Build。显式绑定后等待最少独立机位及配置的角基线/平均深度门，反馈实际 view_count；完成时返回 TargetModel、质量和失败信息。TargetModel 没有单独的发布 topic。整次 Build 使用一个期限，包含等待、重试、finalize 与提交；取消/超时会作废迟到产物。

独立入口在 Build、采集或数值阶段活跃时以 1 ms 预算等待回调；其余 IDLE/READY 时预算为 50 ms，消息/事件就绪可提前唤醒。每轮仍让出 GIL，避免空 wait-set 唤醒持续挤占重建线程。活动检查不取重建状态锁。该预算不构成 Action 的硬实时响应保证，数值库与正在执行的回调仍须自行返回；原 Build 总期限、质量门、话题频率与许可有效期语义不变。

曝光关节历史容量仍为 8192 条，解码速度后按整数纳秒二分查两侧证据；时间回退清空、重复时间戳边界及 `capture.joint_state_tolerance_s` 拒绝规则均保留。该缓存优化不放宽静止门或使用最新 TF/关节状态替代曝光证据。

四个 Trigger 服务为 `~/reset_reconstruction`、`~/finalize_reconstruction`、`~/save_session`、`~/query_reconstruction_state`。前三者经过 Active 门；query 只读。它们不提供手动“开始采集”服务，auto_mode=false 时开新会话应使用 BuildTargetModel。

## 配置来源与覆盖

GPL 声明/基础约束来自 `config/target_reconstruction_parameters.yaml`，部署值来自 `target_reconstruction.yaml`；`params.py` 补充整数下限与 min_views<=max_views。同步容差及 min/recommended/max_views 启动只读，recommended 只作建议。采集/ICP/refitter 等对象有构造期快照；并非所有被 GPL 接受的参数更新都会重建算法对象。

独立 launch 覆盖顺序：params_file → grasp_standoffs → 工具档案 → use_sim_time → params_overlay → task_params_file → capture_auto_mode。部署 YAML 中 auto_mode=true，但独立 launch 的 capture_auto_mode 默认 false，最后覆盖；autostart 默认 true、tool_profile 默认 adaptive_shear_v1。整栈有独立策略透传，不能从这些默认值推断整栈当前运行状态。无硬件维护时显式 `autostart:=false`。

`capture.auto_mode` 是本节点可热改的采集开关，false 不禁用显式 Build。bringup 的 reconstruction_enabled 控制进程/节点装配，supervisor 的 skip_reconstruction 控制作业策略，两者不在本包伪装为同一个开关。模型执行身份必须包含真实 calibration_version；未知时可查看几何，但不得伪造版本。
