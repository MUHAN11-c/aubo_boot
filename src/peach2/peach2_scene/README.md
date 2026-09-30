# peach2_scene

Peach v2 分层场景快照（方案 §6.3）：按需取**一帧**与 TF / 关节同 stamp 对齐的深度，建出 L2 硬障碍（胶囊 / 盒，
**全臂受查**）与 L3 软体素统计，经 `/apply_planning_scene` 原子写入 MoveIt PlanningScene 的 `world.collision_objects`。
不写 ACM、不发任何运动或 IO。

纯逻辑零 ROS（pytest 覆盖）：`scene_core.py`（快照核）、`robot_model.py`（URDF 碰撞几何 → 表面采样）、
`timesync.py`（joint_states 插值）、`conversions.py`（深度单位 / TF 位姿 / TargetModel → 胶囊）、
`primitives.py`（HardObject → SolidPrimitive）、`params.py`（yaml 校验）。`scene_node.py` 只做接线。

## 分层语义

| 层 | 内容 | 谁负责 | 本包做什么 |
|----|------|--------|------------|
| L1 固定 | 臂、工具、相机、台面（URDF collision） | URDF / MoveIt | 自身滤除：点与体素离机器人表面 ≤ `self_margin_m`（体素再加半对角）即删 |
| L2 硬障碍 | 粗枝、主干、柱等 ≥ 12 mm 的结构 | **本包** | 聚类 → 胶囊 / 盒（覆盖每个体素立方体），写入 PlanningScene，**全臂**碰撞 |
| L3 软障碍 | 叶、细枝（< 12 mm）、叶状薄面 | 本包只统计 | **不进碰撞**；`n_soft_voxels` 与软体素中心（将来 costmap） |
| L4 目标 | 袋邻域（底→颈胶囊，R = d95/2 + `target_margin_m`，两端各延 `target_axial_margin_m`，颈端再延 `target_neck_overshoot_m`） | peach2_manipulation | 从快照挖空（点 + 体素半对角 + 拟合对象净空三道，三处同一段），留给臂侧 |

与 peach2_manipulation 的分工：

- 本包只写 `world.collision_objects`，id 前缀取 srv 常量 `BuildSceneSnapshot.HARD_OBJECT_PREFIX`
  （`peach_scene_hard_0000` …）。**不写 ACM**：接触阶段（套入 / 剪切）的局部放行由 manipulation 按阶段本地改
  ACM，本包的写入不会覆盖它。
- **所有**带有效底 / 颈 / d95 的 TargetModel 都被挖空（含颈端越出段）。主审分工（接口变更 01）：manipulation
  在每次规划前把除当前目标外的所有袋写成 `peach_bag_` 胶囊，当前目标只在接近段写入、INSERT 起移除；挖空只保证
  硬障碍不会吞掉套袋走廊。
- 硬障碍对全臂生效（旧 `peach_scene_obstacles` 只让相机 × 障碍受查）。

## 快照流程

1. 服务到达 → 记录请求时刻，只收 **stamp 晚于请求**的深度帧（不用缓存旧帧）；`frame_timeout_s` 内无可用帧即失败。
2. 逐帧对齐（任一步失败换下一帧）：`camera_info` 尺寸与深度一致（深度须已配准到彩色）；编码 `16UC1/mono16`
   × `depth_unit_m`（0 与 65535 无效）或 `32FC1` 米；`/joint_states` 在图像 stamp **插值**（需前后样本夹住、间隔
   ≤ `joint_max_gap_s`），任一关节速度 > `max_joint_speed_rad_s` 视为臂在动拒帧；TF `base←camera` **精确 stamp**；
   `robot_description` 中每个有碰撞几何的连杆逐个查 TF `base←link`（同 stamp，robot_state_publisher 是唯一 FK 源），
   把连杆表面采样换到 base；TCP 同 stamp 查（失败则截断按到 base 原点距离排序）；`/camera/depth/confidence`
   与深度 stamp 差 ≤ 5 ms 才用。
3. `scene_core.prepare_frame`：深度窗 `[min_depth_m, max_depth_m]` → `peach2_core.depth.mask_points` 反投影（有
   2D 掩膜时每类一次）→ base → 工作半径裁剪 → 点级自身滤除。
4. `scene_core.build_scene`（同批多帧合并）：点级目标挖空 → 3 cm 体素，点数 < `min_points_per_voxel` 的体素是飞点
   → 体素级自身 / 目标滤除（加半对角）→ 硬 / 软分类 → 26 邻域聚类 → 拟合 → 按到 TCP 距离截断 `max_objects`。
5. 一次 `ApplyPlanningScene`（`is_diff=true`，`robot_state.is_diff=true`）：REMOVE 上次写入的全部 id + ADD 新对象。
   成功才提交批次缓存；失败保留旧缓存与旧 id，诊断计数。

响应：`n_hard_objects`、`n_soft_voxels`、`truncated`（达 `max_objects`、丢了离 TCP 最远者）、`n_frames`（本批合并
帧数）、`frame_stamp`（本次所取帧的图像 stamp；取到帧后即填，失败也带）、`frame_age_s`、`message`（统计摘要或失败原因）。
失败时 `truncated=false`、`n_frames=0`。

`clear_previous=true`：新批次，丢弃缓存帧并替换上一批全部对象。`false`：把新帧追加进本批（上限
`max_frames_per_batch`）再整体重建。

**目标模型更新**（`/peach/target_model/models`，按 `(target_id, model_revision)` 判变化）：用**本批缓存帧**重建并
重写（新精化目标的邻域随之挖空），不取新帧。

### 硬 / 软分类

- 有 2D 掩膜（只统计来自有掩膜帧的点）：枝占比 ≥ `branch_vote_min` → 硬，除非邻域几何可测地细（线状且宽
  < `hard_min_thickness_m`：细枝在枝掩膜里也是 L3）；否则叶占比 ≥ `leaf_vote_min` → 软，除非线状且宽
  ≥ `leaf_mask_max_width_m`（防 ExG 把树皮标成叶）。
- 无掩膜或票数不决：3×3×3 体素邻域 PCA，线性度 (λ1−λ2)/λ1 ≥ `linearity_min` 时宽 = 2√(3λ2) ≥
  `hard_min_thickness_m` 为硬；非线状且厚 2√(3λ3) ≤ `flat_thickness_max_m` 为叶状薄面（软）；其余为硬。邻域点
  < 10 不可信，按硬。
- 叶状薄面簇包围尺寸 > `soft_max_extent_m` 回升为硬（大平面是主干 / 柱 / 地面）。

### 拟合

每个硬体素簇取 {AABB 盒, PCA 盒, PCA 胶囊} 中体积最小且覆盖所有体素立方体者；填充率 < `fit_min_fill_ratio`
或与机器人 / 目标邻域净空不足时沿主轴递归二分，单体素立方体总是合法。胶囊在 MoveIt 里写成同一
CollisionObject 内的 CYLINDER + 两端 SPHERE（`shape_msgs` 无胶囊）。

## 接口

| 名称 | 类型 | 方向 | QoS / 说明 |
|------|------|------|-----------|
| `/peach/scene/build_snapshot` | `peach2_interfaces/BuildSceneSnapshot` | 服务端 | 同步；阻塞 ≤ `frame_timeout_s` + `apply_timeout_s` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 订 | `image_qos_reliability`（默认 best_effort），depth 2 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 订 | 同上；K 用彩色内参（深度已配准） |
| `/camera/depth/confidence` | `sensor_msgs/Image` | 订（可选） | 同上；mono8 / float |
| `/joint_states` | `sensor_msgs/JointState` | 订 | best_effort, depth 100 |
| `/robot_description` | `std_msgs/String` | 订 | reliable, transient_local |
| `/peach/target_model/models` | `peach2_interfaces/TargetModelArray` | 订 | reliable, transient_local, depth 1 |
| `/tf` `/tf_static` | tf2 | 订 | `tf2_ros.TransformListener`（自带 Reentrant 组） |
| `/apply_planning_scene` | `moveit_msgs/ApplyPlanningScene` | 客户端 | 只写 world.collision_objects |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发 | 机器人模型、深度帧龄、目标挖空数、批次帧数、写入数、失败计数、最近统计 |
| `/bond` | `bond/Status` | 发 | activate 起、deactivate 停（缺 bondpy 降级 WARN） |

话题 / 服务名固定在代码里，不是参数（改名用 launch remap）。回调组：服务、图像、关节、目标模型、robot_description
各一个 MutuallyExclusive，ApplyPlanningScene 客户端 Reentrant；`MultiThreadedExecutor(6)`。等帧在场景锁外，
建快照与写场景在 `_scene_lock` 内串行（服务与模型触发的重建互斥）。

`robot_description` 缺失或有碰撞网格读不到时服务直接失败：没有完整自身几何，臂点会变成假障碍，起点碰撞。

## 参数

`config/scene.yaml`（ROS 参数只有 `config_file`）。直读 + 显式校验：每键必填、未知键拒绝、类型 / 范围越界或
交叉约束不满足拒绝 configure。

| 键 | 默认 | 含义 |
|----|------|------|
| `base_frame` / `tcp_frame` | base_link / tcp | 输出系 / 截断排序参考 |
| `camera_frame` | camera_color_optical_frame | 配准深度的光学系；空 = 用深度 header.frame_id |
| `depth_unit_m` | 0.00025 | uint16 深度 LSB（stereo 0.25 mm；Percipio 改 0.001） |
| `image_qos_reliability` | best_effort | 图像订阅可靠性 |
| `frame_timeout_s` / `tf_timeout_s` | 2.0 / 0.3 | 等帧上限 / 单次 TF 等待（须 < 前者） |
| `max_joint_speed_rad_s` / `joint_buffer_s` / `joint_max_gap_s` | 0.02 / 5.0 / 0.2 | 静止门 / 关节缓存 / 插值最大间隔 |
| `self_sample_spacing_m` | 0.01 | 碰撞网格表面采样步长（≤ `self_margin_m`） |
| `max_frames_per_batch` / `apply_timeout_s` | 8 / 5.0 | 批次帧上限 / 写场景等待 |
| `voxel_size_m` / `min_points_per_voxel` | 0.03 / 5 | 体素边长 / 飞点门 |
| `workspace_radius_m` | 1.5 | 距 base 原点裁剪半径 |
| `pixel_stride` / `min_confidence` | 2 / 0.5 | 反投影步长 / 置信度门（有置信度图时） |
| `min_depth_m` / `max_depth_m` | 0.3 / 1.5 | 光轴深度窗 |
| `self_margin_m` | 0.03 | 自身滤除余量 |
| `target_margin_m` / `target_axial_margin_m` | 0.05 / 0.05 | 目标邻域径向 / 轴向余量 |
| `target_neck_overshoot_m` | 0.09 | 颈端额外挖空：开口越过袋颈 L_blade（三把刀最大 0.079）+ 0.01，挂枝段是套入必经区 |
| `hard_min_thickness_m` | 0.012 | 硬障碍最小粗细（§6.3） |
| `linearity_min` / `flat_thickness_max_m` / `soft_max_extent_m` | 0.5 / 0.010 / 0.25 | 几何分类 |
| `branch_vote_min` / `leaf_vote_min` / `leaf_mask_max_width_m` | 0.2 / 0.6 / 0.05 | 掩膜投票 |
| `fit_min_fill_ratio` | 0.25 | 拟合填充率下限（不足则二分） |
| `max_objects` | 400 | 硬对象上限（保留离 TCP 最近者） |

## 运行

```bash
ros2 launch peach2_scene scene.launch.py                  # 未配置，交给 lifecycle manager
ros2 launch peach2_scene scene.launch.py autostart:=true  # 自己 configure + activate
ros2 service call /peach/scene/build_snapshot peach2_interfaces/srv/BuildSceneSnapshot \
  "{request_id: manual_1, clear_previous: true}"
```

需要 move_group（`/apply_planning_scene`）、robot_state_publisher、相机与 `/joint_states` 在线。`autostart` 只驱动
本节点 lifecycle，不触发任何运动 / IO。

## 怎么测

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
timeout 1200 colcon build --base-paths src/peach2 --packages-select peach2_scene \
  --packages-skip peach2_interfaces peach2_core --build-base build/v2/peach2_scene --install-base build/v2/peach2_scene_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
timeout 1200 colcon test --base-paths src/peach2 --packages-select peach2_scene \
  --build-base build/v2/peach2_scene --install-base build/v2/peach2_scene_install
colcon test-result --test-result-base build/v2/peach2_scene --verbose
```

pytest 不调用 `rclpy.init()`：合成深度场景（粗枝、细枝、方叶 / 长叶、目标袋、自身工具盒、飞点）验证粗枝为硬、叶为软、
目标邻域挖空（含颈端越出段）、自身点滤除、飞点不成体素、上限按 TCP 距离截断、多帧合并、深度窗；另测拟合覆盖 /
二分 / 目标净空、URDF 解析（拒 DTD、二进制 "solid" 头 STL、真 `aubo_e5.urdf`）、关节插值、参数校验、
CollisionObject 组装（REMOVE→ADD、id 前缀 = srv 常量）。

## 接口需求

- ~~响应缺 `truncated` / `frame_stamp` / `n_frames`~~、~~`peach_scene_hard_` 前缀进契约~~：接口变更 01 已落地并接线。
- 枝 / 叶 2D 掩膜无 v2 契约：需要与深度**同尺寸、同 header.stamp（精确）**、已配准到彩色的 mono8
  `branch_mask` / `leaf_mask`（或一张 label 图）写进 `interfaces.yaml`。
- L3 软体素无输出话题：建议 `sensor_msgs/PointCloud2 /peach/scene/soft_voxels`（transient_local）供 costmap / 可视化。
- 方案 §11 接口表写 `/peach/scene/snapshot` + `n_objects`，`interfaces.yaml` 与 srv 为 `/peach/scene/build_snapshot` +
  `n_hard_objects`；本包按后者实现，方案文字待统一。

## 已知限制与 TODO(M0)

- TODO(M0)：接枝 / 叶掩膜话题（契约落定后在 `_align` 里按精确 stamp 取掩膜填 `FrameInput.branch_mask/leaf_mask`；
  核与测试已支持）。v1 `peach_vegetation` 的 ExG 叶掩膜可能把树皮标成叶，已由 `leaf_mask_max_width_m` 兜底。
- 几何回退偏保守：叶片边缘体素邻域被截断时可能读成线状窄带而判硬（合成场景方叶 < 40% 点落入硬对象）；长条叶
  （≥ 12 mm 宽的线状）按规则就是硬。上掩膜后改善。
- TODO(M0)：`hard_min_thickness_m`、`linearity_min`、`flat_thickness_max_m`、`soft_max_extent_m`、投票阈值、
  `max_joint_speed_rad_s` 为设计值，需真机 bag 回放标定。
- 自身采样以台面网格为主（约 34 万点 / 1 cm），`robot_description` 到达时一次性采样约 2 s；每次快照重建一次 KD 树。
- move_group 重启后旧 id 不存在导致 REMOVE 失败时，会不带 REMOVE 重发一次（ADD 同 id 覆盖）；超过新数量的旧 id 若仍在
  场景里不会被删（写 WARN）。
- 帧获取失败时（`clear_previous=true` 也一样）上一批对象保留在场景里并返回失败，由 task 决定是否继续。
- TODO：launch_testing（isolated domain，mock 深度 / TF / joint_states + 假 ApplyPlanningScene 服务）覆盖 lifecycle、
  取帧超时、stamp 对齐、REMOVE/ADD 往返、目标更新重写。
