# peach_scene_perception 接口

跨包图名与业务 QoS 对照 [interface_manifest.yaml](../../peach_interfaces/config/interface_manifest.yaml)；本页补充节点 `_wire_ros` 中的图像与可视化端口。节点名默认 `peach_scene_perception_node`，业务发布话题为绝对名；namespace 只改变节点私有服务名等相对图名。

## 输入与服务

- `color_topic`、`depth_topic`、`camera_info_topic` 分别订阅 Image、Image、CameraInfo，部署值为 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`。三路均 RELIABLE / VOLATILE / KEEP_LAST(1)，同步队列另由 `sync_queue_size` 控制。输入深度必须已与彩色像素配准。
- `/joint_states`：JointState，默认 RELIABLE / VOLATILE / depth 10；用于推理停稳门。当前帧几何不据此改用其它曝光。
- `/peach_supervisor/state`：HarvestState，RELIABLE / TRANSIENT_LOCAL / depth 1；镜像执行目标和阶段，覆盖显示 selected 并组织落盘目录。
- `~/begin_scene`：`peach_interfaces/srv/BeginScene`，默认 `/peach_scene_perception_node/begin_scene`。Active 才接受；返回 scene_epoch 与身份保留/清除信息。未提供运动 service 或 action。
- TF：以深度 stamp 查询 `output_frame ← camera_optical_frame`；相机帧参数为空时依次使用消息 frame_id。TF 回退只能作为带 stale 标记的几何显示，不更新稳定身份。

## 输出

以下相对后缀都位于 `/peach/perception/`：

- `initial_pose`：BagGraspCandidateArray，RELIABLE / TRANSIENT_LOCAL / depth 1；已确认目标的同曝光几何初值，供重建绑定。ACCEPT 不表示接触许可。
- `target_observations`：PeachTargetObservationArray，RELIABLE / VOLATILE / depth 10；锁定集、身份、priority、跟踪状态及目标掩膜，供调度和重建消费。
- `harvest_state`：String JSON，RELIABLE / TRANSIENT_LOCAL / depth 1；锁窗、优先级、发现进度、耗时与数据目录镜像。
- `axis`：Vector3Stamped；`diagnostics`：BagFittingArray。均 RELIABLE / VOLATILE / depth 10。后者为算法拟合诊断，区别于系统健康 `/diagnostics`。
- `single_cloud`：PointCloud2；`detections`：Detection2DArray；`masks`：Image；`markers`：MarkerArray；`debug_image` 和 `debug_image_raw`：Image。均 RELIABLE / VOLATILE / depth 1。掩膜为 uint16 实例序号（检测索引+1），与稳定 target_id 分开；逐目标关联以 target_observations 为准。

几何 header 使用深度曝光 stamp 和输出坐标系；检测与彩色叠加图使用彩色 header；掩膜使用深度 stamp 和相机系。`debug_image_raw` 仅在开启 `publish_debug_image` 且有订阅者时转发/转换原图，不等待推理，但仍经过 Active、同步与入口年龄门。健康由 diagnostic_updater 发布到 `/diagnostics`。

## 配置来源

`scene_perception_parameters.yaml` 生成声明和基础约束，`params.py` 补充整数下限、有限重力向量与有效深度窗检查。不要手改生成模块当作参数事实源。

独立 launch 的覆盖顺序为：部署 `params_file` → 共享 grasp_standoffs → `tool_profile` 档案 → use_sim_time → 可选 `params_overlay` → `task_params_file` → percipio 专用年龄门。独立 launch 的 `autostart=true`、`tool_profile=adaptive_shear_v1`；整栈可另行覆盖，不能与整栈默认混写。无硬件维护时显式使用 `autostart:=false`。

`camera_frontend` 仅是 launch 选择 overlay 的参数，不是节点热切相机的参数。`yolo_*`/`sam_*`、`pipeline.*`、`target_memory.*`、`harvest.*`、工具几何及重力分组的完整值以部署 YAML 为准；构造期配置修改后须重启，逐帧读取键才热生效。

`tool.L_insert` 只参与最短有效行程门；视觉建议上限由袋颈距离减安全余量生成，执行夹紧在臂侧。`tool.L_blade` 保留档案参数，但当前 `_derive` 传给 ToolGeometry 的 blade_offset_m 为零，不能把该配置值解释成已在感知行程中扣除。
