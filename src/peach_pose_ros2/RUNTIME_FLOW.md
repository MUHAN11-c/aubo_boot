# PeachPose 完整运行流程文档

> 本文覆盖 `peach_pose_ros2` 从启动到输出的完整运行逻辑：运行形态与前置条件、
> 启动链路、参数加载、节点初始化、每帧处理、算法管线、融合决策段、输出话题、
> 生命周期与常见故障。逐行讲解见 `TUTORIAL.md`，参数权威值见
> `config/peach_pose.yaml`。

## 1. 程序定位与总体架构

`peach_pose_ros2` 是纯感知节点：订阅 RGB-D + CameraInfo，输出桃袋/裸果的
**抓取参考位姿**（入口点、套入方向、行程、三态安全状态），**不发送运动指令**。
节点内部按五层组织：

| 层 | 模块 | 职责 |
|---|---|---|
| 参数层 | `params.py` | 参数声明/加载/热调（DEFAULTS/DESCRIPTIONS/DYNAMIC） |
| 转换层 | `conversions.py` | TF/四元数、几何变换、内部结果→ROS 消息 |
| 可视化层 | `visualization.py` | Marker、debug 叠加图、概率热图、检测点云 |
| 编排层 | `peach_pose_node.py` | 节点构造、TF 查询、逐帧回调、发布 |
| 算法包 | `peach_pose/` | 纯算法核心（不 import rclpy，可离线测试） |

数据流一图流：

```
相机/回放器 RGB-D → peach_pose_node
  → YOLO 检测 → MobileSAM 分割 → hybrid_dilated 前景
  → 袋线/果线几何拟合 + 三态安全门控
  → TF 变到 base_link → 基础话题发布
  →（可选）时序融合 + ROI 云重拟合 → 融合话题发布
```

## 2. 运行前置条件

1. **环境**：`source /opt/ros/jazzy/setup.bash`，构建并 source 工作区：

   ```bash
   colcon build --packages-select peach_pose_msgs peach_pose_ros2 \
     --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

2. **Python 环境**：依赖 `aubo_py3.12` venv（numpy==1.26.4 + torch +
   ultralytics，GPU 优先）。节点启动器 shebang 在构建期指向该 venv，
   运行时无需手动 source。
3. **数据源二选一**：真相机（`percipio_rgbd.launch.py`，RGB-D 配准）或
   数据集回放器。
4. **TF**：结果要落在机器人坐标系时，需 `hand_eye_extrinsics_publisher`
   发布 `wrist3_Link→camera_link` 静态 TF；缺失时节点告警并回退相机系。
5. **清理残留进程**（多实例同时加载 YOLO/SAM 可能段错误）：

   ```bash
   pgrep -af 'peach_pose_ros2|peach_dataset_replayer|ros2 launch'
   ```

## 3. 启动链路（三种运行形态）

### 3.1 真相机

```bash
# 终端 1：相机驱动（RGB + 深度 + 内参）
ros2 launch percipio_camera percipio_rgbd.launch.py
# 终端 2：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py
```

### 3.2 无相机冒烟（数据集回放）

```bash
# 终端 1：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py
# 终端 2：回放 rgb/ + depth/ 成对 PNG（默认 1Hz、3 帧）
aubo_py3.12/bin/python tools/peach_dataset_replayer.py \
  --dataset <数据集根> --limit 3 --loop --rate 0.5
```

回放器以与真相机相同的话题名（`/camera/color/image_raw`、
`/camera/depth/image_raw`、`/camera/color/camera_info`）发布消息，三路
同一时间戳，便于时间同步。回放深度为真毫米，节点参数
`depth_scale_unit` 需先改为 1.0（改 `config/peach_pose.yaml` 后重启）。

### 3.3 离线评估

```bash
aubo_py3.12/bin/python -m peach_pose_ros2.peach_pose.e2e_validate \
  --output e2e_validation.json
```

固定跑 `hybrid_dilated` 前景，袋/果两类都评估，输出 JSON + Markdown 报告。

### 3.4 launch 文件做了什么

`launch/peach_pose.launch.py` 只启动一个节点：

```python
Node(package='peach_pose_ros2', executable='peach_pose_node',
     name='peach_pose_node', parameters=[config], output='screen')
```

- `executable='peach_pose_node'`：`setup.py` 中 `console_scripts` 生成的
  启动器，shebang 经 `options.build_scripts.executable` 指向 venv；
- `name` 必须与 `peach_pose.yaml` 顶层键一致，否则参数静默不生效；
- `parameters=[config]` 把 yaml 全量灌入，覆盖代码内默认值；
- `output='screen'` 让日志打到终端。

## 4. 参数加载与分类

`params.py` 维护三张表，节点只调 `declare_params` / `load_params` /
`make_on_set_callback`：

| 类别 | 参数（默认值） | 作用 |
|---|---|---|
| 输入话题 | `color_topic` `/camera/color/image_raw`；`depth_topic` `/camera/depth/image_raw`；`camera_info_topic` `/camera/color/camera_info` | RGB-D 与内参来源 |
| 坐标系 | `camera_optical_frame` `camera_color_optical_frame`；`output_frame` `base_link`；`tf_timeout_sec` 0.5 | 手眼光学系、输出系、TF 超时 |
| 深度/同步 | `depth_scale_unit` 0.25；`sync_slop_s` 0.05 | 原始深度→毫米；三路同步允差 |
| 检测阈值 | `yolo_conf` 0.3（宽进）；`min_detection_conf` 0.5（严出）；`yolo_iou` 0.5；`sam_max_bboxes` 8；`sam_min_area` 100 | 两级检测过滤与 SAM 限制 |
| 几何门限 | `min_depth_m` 0.3；`max_depth_m` 2.5；`min_points` 100 | 有效深度范围与最小点数 |
| 可视化 | `publish_debug_image`/`publish_masks`/`publish_detection_cloud` true；`detection_cloud_stride` 2 | 输出开关 |
| 模型/版本 | `yolo_model_path`/`sam_model_path` 包内权重；`model_version`/`calibration_version` | 权重路径与结果追溯标识 |
| 融合段 | `refined_outputs_enabled` true；`recon_cloud_topic`；`recon_min_points` 3000；`recon_refined_period_sec` 1.0；`target_kind` bag | 融合/精化输出 |
| 工具几何 | `tool.D_inner` 0.104 等 8 项 | 套袋工具物理约束，直接参与安全门控 |

热调规则：`DYNAMIC` 集合内的参数（阈值、可视化开关等）可运行时修改，
其余声明为 `read_only`（话题、模型路径、工具几何等需重启生效）。
`config/peach_pose.yaml` 是权威默认值，改默认值必须与代码同步。

## 5. 节点初始化序列（`PeachPoseNode.__init__`）

1. 注册节点名 `peach_pose_node`，创建 `CvBridge`；
2. `declare_params` + `load_params` 读入全部参数，注册热调回调；
3. 解析模型路径（`_resolve_model_path`：相对包目录/绝对路径/目录，无效
   告警并回退包内 `model/best.pt`、`model/mobile_sam.pt`）；
4. 建 `InferenceEngine`（YOLO + SAM，懒加载，`threading.Lock` 保证 CUDA
   线程安全，设备优先 `cuda:0`）；
5. 建 `CandidateEstimator`：内部构造袋线 `RobustBagPosePipeline` 与果线
   `RobustFruitPosePipeline`（共享同一 `ToolGeometry`）；
6. 若 `refined_outputs_enabled`：建 `TemporalFuser`、订阅 ROI 点云、
   建 4 个融合发布者；
7. 建 8 个基础发布者，用 `ApproximateTimeSynchronizer` 同步订阅
   RGB/深度/内参（RELIABLE QoS，queue=10），注册 `tf2_ros` 监听。

## 6. 每帧处理流程（`_on_rgbd` 主回调）

三路消息时间戳差 ≤ `sync_slop_s` 时触发一次回调，分七段执行：

### 6.1 转换与校验

- RGB 经 cv_bridge 转 `bgr8`；深度 `passthrough` 保持 uint16；
- 深度 dtype 非 uint16、RGB/深度分辨率不一致、CameraInfo 尺寸与深度不符，
  任一失败 warn 并**丢整帧**；
- 深度原始值乘 `depth_scale_unit`（0.25）并钳位到 uint16 值域 → 毫米。

### 6.2 内参与坐标系

- 内参 K 只取 CameraInfo 的 `k` 矩阵（fx/fy/cx/cy），不 FOV 推导；
- 相机系回退链：参数 `camera_optical_frame` → 深度图头 → 彩图头 → 内参头；
- `_lookup_T_out_cam`：先按帧时间戳查 TF，失败退最新 TF（告警一次），
  再失败回退相机系发布（`out_frame=cam_frame`，避免静默用错坐标系）。

### 6.3 YOLO 检测

`engine.detect(rgb)` 跑 YOLO（conf≥0.3），输出按置信度降序的检测 dict；
再按 `min_detection_conf`（0.5）二级过滤，通过的进管线；检测框先发
`~/detections`，并按开关反投影检测框深度点云发 `~/detection_cloud`。

### 6.4 逐目标处理（核心循环）

对每个保留的检测框：

1. `engine.segment(rgb, [bbox])`：以 YOLO 框为 box prompt 跑 MobileSAM，
   概率 >0.5 二值化为掩膜；原始概率图逐像素取最大，累积成热图画布；
2. 打包 `BagObservation`（RGB、深度、K、坐标系、重力提示、版本元数据）；
3. `CandidateEstimator.estimate`：
   - `build_mask` 构造 `hybrid_dilated` 前景：
     `SAM ∩ 有效深度 ∩ 膨胀(深度连通域)`（膨胀 5px）；
   - 按 class_id 路由：0=袋装桃→袋线，1=裸果→果线；
   - 掩膜缺失或交集 <50px → 显式 REOBSERVE（`mask_unavailable`），
     禁止深度-only 静默回退；
   - 回填 `strategy_id`、模型/标定/工具版本与耗时统计；
4. TF 变换：`_apply_T_to_grasp3d` 把 3D 几何从相机系变到输出系
   （点 R·p+t，方向只乘 R 并归一化）；
5. `conversions._to_*` 装成 3D 候选/2D 关键点/诊断消息；
   `visualization._to_markers` / `_draw_debug` 生成可视化。

### 6.5 批量发布

- 每帧无条件发四类主消息：`~/grasp_candidates`、`~/grasp_candidates_2d`、
  `~/fitting`、`~/markers`（空数组也发，区分"没检出"与"节点死"）；
- `~/masks`、`~/debug_image`、`~/detection_cloud` 按参数开关发；
- 每 2s 一条候选计数日志作为心跳。

### 6.6 融合决策段

`refined_outputs_enabled` 时，每帧取最优候选（ACCEPT 优先，其余按置信度）
灌入 `TemporalFuser`，见第 8 节。

## 7. 算法管线细节（pipeline.py）

两条管线共享安全门控，**只用传感器实测深度**：

### 袋线 `RobustBagPosePipeline`（class 0，12 阶段）

1. bbox 裁到图内，<8px 直接 REJECT（`invalid_bbox`）；
2. 有效深度掩膜：0.3~2.5m 内才可用；
3. 前景掩膜：SAM∩有效深度 ≥50px；不可用时显式 `depth_fallback`
   （中心深度带 + 最大连通域），不静默；
4. pinhole 反投影成 3D 点，剔深度离群点，<100 点 REJECT
   （`insufficient_measured_points`）；
5. 重力向量：参数 `gravity_hint_xyz` 或默认相机系 +Y，归一化；
6. 法线估计（深度图邻域叉积，`fitting.estimate_normals`）；
7. 轴估计三级降级：法线 ≥200 点 → 圆柱 RANSAC（inlier≥0.35 采纳）→
   重力先验（置信度封顶 0.4）→ 符号统一为"底→颈=逆重力"；
8. 底/颈/袋径：沿轴投影 P10/P90 定底颈，横向 P95×2 为袋径上界；
9. 2D 交叉校验：掩膜 PCA 主轴与 3D 轴投影夹角 >45° 打
   `axis_2d_mismatch`；
10. entry/travel/抓取架：`entry_start = 袋底 − (d_tool+d_s)·轴`，
    行程 = 到颈部减刃长减余量，构造右手抓取架；
11. 误差预算门控：`(预停+行程)·sin(轴角误差) ≤ 径向净空
    (D_inner/2 − 袋半径 − clearance_min)`，否则
    `tool_clearance_failed`；
12. 状态规则：无 flag → ACCEPT；`tool_clearance_failed` → REJECT；
    其余 → REOBSERVE；2D 关键点投影回像素，填诊断指标。

### 果线 `RobustFruitPosePipeline`（class 1）

几何换成球拟合（半径夹紧 25~45mm）+ 梗洼检测定轴（Fibonacci 球面扫描找
径向下陷最深的 20° 方向帽），并做重力极性校正（梗朝下的洼实为萼洼，
翻轴并降置信）。entry/travel/门控公式与袋线相同。

## 8. 融合决策段详解

### 8.1 时序融合（`TemporalFuser`）

每帧灌入最优候选的入口位置、套入方向、袋径、置信度：

- 离群门控：位置距融合值 >50mm 或轴夹角 >20° 判离群剔除；
- 接受后按置信度加权 EMA 更新（α=0.3×conf）；
- 接受历史做迷你 DBSCAN（eps=30mm）统计一致簇大小
  （`support_frames`），并给出位置/轴向收敛指标；
- 轴向按无向处理（点积 <0 先翻转），避免 ±axis 二义性。

输出：

- `~/refined_candidates`：最优候选替换为融合值（位置/方向/袋径），
  打 `|fused` 与 `support_frames=N` 标记；
- `~/fusion_stats`：帧号、接受/拒绝数、簇大小、`pos_std_m`/`axis_std_deg`；
- `~/refinement_evolution`：融合位姿快照（PoseStamped）。

### 8.2 ROI 云重拟合（`refiner.py`，降频）

每 `recon_refined_period_sec`（1s）尝试一次：订阅的 ROI 点云
（`/recon_refiner_node/roi_cloud`）点数 ≥`recon_min_points`（3000）时，
open3d 法线估计 + 按 `target_kind` 重拟合（bag→圆柱、fruit→球），
发布 `~/recon_refined`（单候选：中心、方向、袋径=2×半径、置信度=内点率，
`n_points`/`rms` 写入诊断）。open3d 仅 venv 有，首次使用才 import。

## 9. 输出话题与消息字段

节点命名空间 `~/`，实际为 `/peach_pose_node/`，队列深 10：

| 话题 | 类型 | 内容 |
|---|---|---|
| `~/grasp_candidates` | BagGraspCandidateArray | **主输出**：3D 抓取参考 |
| `~/grasp_candidates_2d` | BagGrasp2DArray | 图像平面关键点（UI 叠加用） |
| `~/fitting` | BagFittingArray | 21 项拟合诊断标量（无效填 -1） |
| `~/detections` | Detection2DArray | YOLO 原始检测框 |
| `~/masks` | Image (mono8) | SAM 概率热图 |
| `~/markers` | MarkerArray | RViz 轴线/行程/刀具/状态 |
| `~/debug_image` | Image (bgr8) | 检测框+掩膜+箭头+状态叠加图 |
| `~/detection_cloud` | PointCloud2 | 检测框内反投影彩色点云（输出系） |
| `~/refined_candidates` | BagGraspCandidateArray | 时序融合最终候选 |
| `~/fusion_stats` | FusionStats | 融合收敛性统计 |
| `~/refinement_evolution` | PoseStamped | 融合位姿快照 |
| `~/recon_refined` | BagGraspCandidateArray | ROI 云重拟合精化结果 |

主输出 `~/grasp_candidates` 单个候选字段：

| 字段 | 含义 |
|---|---|
| `header` | frame_id=输出系（默认 base_link），stamp=RGB 帧时刻 |
| `target_id` | `target_0`…（帧内序号，2D/3D/诊断按它对齐） |
| `entry_pose` | 工具接近预停位姿（Z 轴=袋轴） |
| `bag_bottom` / `bag_neck` | 袋底/袋颈 3D 点 |
| `translation_direction` | 套袋平移方向（单位向量，底→颈） |
| `bag_diameter_upper_m` | 袋径上界 |
| `suggested_travel_m` | 建议行程 |
| `confidence` | 综合置信度 |
| `status` | 0=ACCEPT / 1=REOBSERVE / 2=REJECT |
| `diagnostic_flags` | 原因字符串数组 |
| `strategy_id` | 前景策略（`hybrid_dilated`/`|fused`/`recon_refined:...`） |
| `model_version` / `calibration_version` / `tool_version` | 版本追溯三件套 |

## 10. 生命周期、故障排查与关键契约

### 停止与清理

`Ctrl+C` 触发 `main()` 的 finally 分支：`destroy_node()` +
`rclpy.shutdown()`。换数据源/参数前确认无残留进程，相机设备与 SDK 通道
一样独占，重开前必须清干净。

### 常见故障

| 现象 | 排查 |
|---|---|
| 收不到消息 | 检查 QoS（本节点 RELIABLE，发布端也须 RELIABLE）；话题名是否匹配 |
| 首帧慢 | 模型懒加载 + CUDA warmup，属正常；看 `~/fitting` 与日志耗时 |
| 坐标不对 | 看 `header.frame_id`：若为相机系说明 TF 缺失（extrinsics_publisher 未起） |
| 全是 REOBSERVE/REJECT | 读 `diagnostic_flags` 对照门控规则；`~/fitting` 有量化指标 |
| SAM 失败刷屏 | 限流 2s 一条日志；下游按无掩膜显式 REOBSERVE，不炸节点 |

### 关键契约（易错点）

- **深度单位**：管线内部 uint16 毫米；Percipio 原始值 ×0.25 → 毫米，
  /1000 → 米；数据集回放设 `depth_scale_unit=1.0`。
- **坐标系**：几何在相机光学系求解，发布前变到 `output_frame`；TF 缺失
  回退相机系并告警，绝不静默用错坐标。
- **三态语义**：ACCEPT=可执行 / REOBSERVE=信息不足建议重采 /
  REJECT=禁止动作；状态规则"宁可拒，不出错结果"。
- **只发参考**：本节点不连接运动接口，机械臂执行需下游消费
  `~/grasp_candidates`（或 `~/refined_candidates`）自行规划。
