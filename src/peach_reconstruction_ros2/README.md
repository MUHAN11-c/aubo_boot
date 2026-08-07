# peach_reconstruction_ros2 — 桃子多视角局部重建包（Phase 2）

## 简介

多视角局部重建：订阅 RGB-D，**默认全自动**（零服务）——感知候选到位即
自动绑定目标开始重建，低速连续拖动机械臂（迁就相机 0.8 FPS 深度帧率）
时随视角变化自动采帧，采满 `max_views` 自动完成并输出重叠度指标。
每帧按 `depth.header.stamp` 查 `base←camera` TF，反投影建云并变到
`base_frame` 多帧累加。6 个 Trigger 服务全部保留作手动备用
（`capture.auto_mode:=false` 回 Phase 2 纯手动服务流）。

**Phase 边界：TSDF 融合（Phase 4）已启用**——finalize 时对全部已采帧批量
积分（Open3D ScalableTSDFVolume），产出带颜色的 tsdf_cloud；几何 refit
（轴/半径精化）属 Phase 5。本阶段不做在线配准/网格化。不发送运动指令。

## 使用方法

### 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_reconstruction_ros2
source install/setup.bash
```

### 运行

```bash
# 终端 1：感知节点（提供 /peach/perception/initial_pose，可选但建议）
ros2 launch peach_pose_ros2 peach_pose.launch.py

# 终端 2：重建节点
ros2 run peach_reconstruction_ros2 peach_reconstruction_node --ros-args \
  --params-file install/peach_reconstruction_ros2/share/peach_reconstruction_ros2/config/reconstruction.yaml

# 无相机冒烟：数据集回放（真毫米深度需覆盖 depth_scale_unit:=1.0）
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --loop --limit 40
# 回放没有 base_link TF 链时需补一个静态 TF（示意）：
ros2 run tf2_ros static_transform_publisher --x 0.4 --y 0.0 --z 0.6 \
  --roll 0 --pitch 0 --yaw 0 --frame-id base_link \
  --child-frame-id camera_color_optical_frame
```

无相机冒烟时建议同时覆盖 `-p view_filter.allow_duplicate_views:=true
-p capture.min_views:=2`（静态 TF 下视角不动，默认过滤会拒重复视角）。

### 自动模式（默认工作流，零服务）

`capture.auto_mode:=true`（默认）时全程不需要敲任何服务：

1. **自动开始**：IDLE 且 `/peach/perception/initial_pose` 有候选 → 自动绑定
   最优候选（第一个 ACCEPT，否则第一个）进入 COLLECTING；无候选静默等待；
2. **自动采帧**：COLLECTING 时每个新同步帧检查——首帧直采；后续帧与上一
   已采位姿比较，平移 ≥ `view_filter.min_translation` **或** 旋转 ≥
   `min_rotation_deg` 即采（连续运动超上限只告警不拒帧；距上次采帧
   < `capture.auto_min_interval_s` 跳过；`require_robot_static=true` 时
   关节速度超阈跳过；TF 失败跳过本帧记 `tf_failures`）；
3. **自动完成**：`capture.auto_finalize_at_max` 且采满 `max_views` → 自动
   执行与 finalize 服务相同的装配 + overlap + 置 READY。

READY 后停采；调一次 `reset_reconstruction`（或 `start_reconstruction`）
即进入下一轮。低速连续拖动机械臂（如 RViz 0.01 速度缩放）即是完整操作。

### 手动服务（备用；自动模式下照常可用，语义不变）

```bash
N=/peach_reconstruction_node
ros2 service call $N/start_reconstruction std_srvs/srv/Trigger      # IDLE→COLLECTING，绑定最优候选
ros2 service call $N/capture_frame std_srvs/srv/Trigger             # 手动补拍一帧（严格视角过滤）
ros2 service call $N/remove_last_frame std_srvs/srv/Trigger         # 弹掉最后一帧
ros2 service call $N/finalize_reconstruction std_srvs/srv/Trigger   # 拼云发 local_cloud，→READY
ros2 service call $N/save_session std_srvs/srv/Trigger              # 落盘 session_<时间戳>/
ros2 service call $N/reset_reconstruction std_srvs/srv/Trigger      # 清空回 IDLE（开下一轮）
```

### 参数

31 个参数全部带中文 `ParameterDescriptor`，权威值在
`config/reconstruction.yaml`（与 `declare_parameter` 逐项对齐）。常用：

| 参数 | 默认 | 说明 |
|---|---|---|
| `frames.base_frame` | base_link | 重建输出坐标系 |
| `camera.*_topic` | /camera/* 三件套 | RGB-D 输入（深度 uint16 或 32FC1） |
| `sync_slop_s` / `tf_timeout_sec` | 0.05 / 0.5 | 同步允差 (s) / TF 查询超时 (s) |
| `depth_scale_unit` | 0.25 | uint16 深度 × 本值 = 毫米；32FC1 米制下不生效 |
| `capture.min_views` / `recommended_views` / `max_views` | 4 / 5 / 8 | 视角数门槛/推荐/上限 |
| `capture.auto_mode` | true | 自动开始/采帧/完成总开关；false=纯手动服务流 |
| `capture.auto_finalize_at_max` | true | 采满 max_views 自动 finalize |
| `capture.auto_min_interval_s` | 2.0 | 两次自动采帧最小间隔 (s) |
| `capture.require_robot_static` / `static_joint_vel_thresh` | false / 0.01 | 静止门禁与关节速度阈值 [rad/s] |
| `capture.max_frame_age_s` | 2.0 | 缓存帧龄期上限 (s)，超过拒采 |
| `view_filter.min/max_translation` | 0.020 / 0.080 | 与上一帧平移上下限 [m] |
| `view_filter.min/max_rotation_deg` | 5.0 / 25.0 | 与上一帧旋转上下限 [deg] |
| `view_filter.allow_duplicate_views` | false | true 时重复视角仅告警仍采帧 |
| `local_volume.size_x/y/z` | 0.30/0.30/0.40 | TSDF 云 ROI 裁剪盒 [m]（中心=绑定候选/首帧云质心） |
| `tsdf.enable` | true | finalize 时批量积分 TSDF；false=只做原始累加 |
| `tsdf.voxel_length` / `sdf_trunc` / `depth_trunc` | 0.003/0.012/1.5 | TSDF 体素/截断/深度截断 [m] |
| `cloud_filter.voxel_size` / `enable_statistical_filter` | 0.003 / true | 提取云体素降采样 [m] / 统计离群剔除（20 邻域 2σ） |
| `session.root_dir` | "" | 空 = `<工作区>/peach_sessions`（按包 share 路径反推） |

### 单测

```bash
cd src/peach_reconstruction_ros2
PYTHONPATH=. $HOME/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python -m pytest test/ -q
```

（须先 source ROS 与工作区 install；42 例业务测试 + flake8/pep257 lint；
TSDF 用例依赖 open3d，系统 python3 下经 conftest 自动跳过。）

## 执行逻辑

### 自动模式主循环（默认，`_on_rgbd` 回调末尾驱动）

```text
每个新同步帧（深度归一化 + 缓存后）：
  IDLE     → initial_pose 有候选 → 绑定最优候选自动开始（无候选静默等待）
  COLLECTING → 满 max_views → 自动 finalize（装配 + overlap → READY）
             → 否则尝试自动采帧：
                 本帧已采过 / 陈帧（> max_frame_age_s）→ 跳过
                 require_robot_static 且关节速度超阈 → 跳过
                 按 depth.header.stamp 查 TF（失败跳过本帧记 tf_failures）
                 首帧直采；否则平移 ≥ min_translation 或旋转 ≥ min_rotation_deg
                 才采；超上限只告警照采；间隔 < auto_min_interval_s 跳过
  READY/FAILED → 停采，等 reset/start 进入下一轮
```

决策纯逻辑在 `frame_collector.py`（`should_auto_start` /
`auto_capture_decision` / `should_auto_finalize`），节点只做 TF/订阅接线。
手动服务在自动模式下照常可用：`capture_frame` 走严格视角过滤（重复/跳变
拒帧），适合补拍；`remove_last` 回滚；`reset` 开下一轮。

### 一次手动采帧的完整门禁链（`~/capture_frame`）

```text
状态须 COLLECTING（否则先 start）
  → 帧栈未满（< max_views）
  → 有同步缓存帧，且时间戳新于上次采帧（不重复采同一帧）
  → 帧龄期 ≤ max_frame_age_s（陈帧拒采）
  → require_robot_static 时：已收 /joint_states 且最大关节速度 ≤ 阈值
  → 按 depth.header.stamp 查 T_base_camera（超时 tf_timeout_sec）
      按 stamp 失败 → 回退一次最新 TF（命中打 tf_stale 标记）
      彻底失败 → 拒帧并计 tf_failures
  → 视角过滤：与上一已采帧比相对平移/旋转
      同时低于下限 = 重复视角（allow_duplicate_views 时仅告警放行）
      任一高于上限 = 跳变，恒拒
  → 深度归一化 uint16[mm] → pinhole 反投影 [m] → T_base_camera 变到
    base_frame → CapturedFrame 入栈 → 重发累加云/状态/诊断/Marker
```

TF 策略（本包选定，README 明示）：**按 stamp 查询失败时允许一次 latest
回退**，命中帧打 `tf_stale` 诊断标记（写入 CapturedFrame.diagnostic_flags
与 session 元数据），与 peach_pose_node 的语义一致；彻底失败才拒帧。

`~/finalize_reconstruction`：视角数 ≥ min_views 时把全部已采帧点云拼接发到
`/peach/reconstruction/local_cloud`（frame_id=base_frame），状态 → READY；
不足 min_views 时 success=false 并保持 COLLECTING。READY 语义不变。

### finalize 的 TSDF 后处理链（Phase 4，`tsdf.enable:=true` 默认开）

```text
collector.finalize() 成功
  → overlap 指标（相邻帧 NN 统计）
  → TSDF 批量积分（逐帧 BGR→RGB、uint16[mm]→float[m]，
      外参取逆 T_camera_base = inv(T_base_camera) 传给 Open3D——方向最易错，
      由 test_tsdf_volume.py 的方向守门用例固防）
  → extract_point_cloud → ROI 裁剪（local_volume，中心=绑定候选中心，
      无候选时用首帧云质心）→ 体素降采样（cloud_filter.voxel_size）
      → 统计离群剔除（20 邻域 2σ）
  → 发布 /peach/reconstruction/tsdf_cloud（xyz+rgb 彩色云，transient_local）
  → message 追加「；TSDF N 点（积分 X.XXs）」，diagnostics 加 tsdf 键
      （points/integrate_time_s/voxel_length/sdf_trunc/roi_center）
  → save_session 时 result/tsdf_cloud.ply 随之落盘
```

raw `local_cloud`（原始累加云）保持原样继续发布，与 tsdf_cloud 并存对照。
TSDF 云缓存在节点内随 `_publish_all` 重发；帧栈变动（start/capture/
remove_last/reset）后缓存失效，需重新 finalize 生成。

### finalize 重叠度指标（overlap，Phase 3 新增）

finalize 成功时对**相邻已采帧**（i-1 与 i）的 base 系点云算最近邻距离统计
（先固定种子随机抽稀到 ≤20000 点再用 cKDTree，结果可复现）：

- message 末尾追加聚合值，形如
  `局部重建完成：5 视角，2 345 678 点；重叠 mean=1.2mm p95=3.4mm`
  （mean 取各对平均，p95 取最差对；不足 2 帧时注明不可用）；
- 完整结果并入 `/peach/reconstruction/diagnostics` JSON 的 `overlap` 键：
  `pairs:[{i,mean_mm,median_mm,p95_mm},...]`（i 为对中较后帧下标）、
  `frame_centroids_base`（每帧质心 [m]）、`centroid_base`（装配总质心 [m]）；
- 帧栈变动（start/capture/remove_last/reset）后缓存失效，diagnostics
  中 `overlap` 为 null，需重新 finalize。

**Phase 3 真机刚性对齐判据（建议）**：同一静态目标、4+ 视角，overlap
**mean < 5mm 且 p95 < 15mm** 视为刚性对齐合格；超标时按以下顺序排查：
depth scale（depth_scale_unit）→ 内参 → RGB-D 对齐（registration）→
手眼外参 → TF 方向（base←camera 别写反）→ 时间戳（tf_stale 帧多说明
HW 时间戳超前严重）→ 运动中曝光（停稳再采）。判据的灵敏度由
`test/test_multiview_alignment.py` 的合成圆柱反例守门：外参错 ±20mm 时
聚合 mean 从 ≈0.6mm 恶化到 ≈16.5mm（约 25 倍）。

## 软件框架

### 节点与入口

单节点 `peach_reconstruction_node`，标准 console_scripts 入口；setup.py 经
`options.build_scripts.executable` 把启动器 shebang 指到 venv 解释器
（约定同 `AGENTS.md` 第 8 节）。无 launch 文件，`ros2 run` + `--params-file`
启动。

### 模块划分

| 模块 | 职责 | 依赖 |
|---|---|---|
| `reconstruction_node.py` | ROS 面：参数/订阅/服务/发布/TF/诊断 | rclpy 等 |
| `captured_frame.py` | CapturedFrame 数据合约（一帧的全部内容） | numpy |
| `frame_collector.py` | 状态机 + 视角过滤 + 帧栈（纯逻辑） | numpy |
| `cloud_builder.py` | 深度反投影 + 点云变 base 系 + BGR 打包 | numpy |
| `overlap.py` | 相邻帧重叠度指标（NN 统计/质心，固定种子抽稀） | numpy/scipy |
| `tsdf_volume.py` | LocalTsdf：Open3D TSDF 批量积分 + ROI/降采样/离群剔除 | open3d |
| `tf_utils.py` | Transform→4×4 / 刚体逆 / 相对运动量 | tf_transformations |
| `session_io.py` | session 落盘（png/npy/yaml + metadata） | cv2/yaml |
| `visualization.py` | 相机轨迹 MarkerArray | visualization_msgs |

深度归一化复用
`peach_pose_ros2.peach_pose.depth_geometry.normalize_depth_to_uint16_mm`
（uint16 毫米约定全链路一致）。

### 话题与服务

| 方向 | 名称 | 类型 |
|------|------|------|
| sub | `/camera/color/image_raw` 等三件套 | `sensor_msgs/Image` / `CameraInfo` |
| sub | `/peach/perception/initial_pose` | `peach_pose_msgs/BagGraspCandidateArray` |
| sub | `/joint_states` | `sensor_msgs/JointState` |
| pub | `/peach/reconstruction/local_cloud` | `sensor_msgs/PointCloud2`（xyz+rgb 彩色云，frame_id=base_frame；RViz 选 RGB8 上色） |
| pub | `/peach/reconstruction/tsdf_cloud` | `sensor_msgs/PointCloud2`（TSDF 融合云，xyz+rgb，finalize 后非空） |
| pub | `/peach/reconstruction/status` | `std_msgs/String`（IDLE/COLLECTING/READY/FAILED） |
| pub | `/peach/reconstruction/diagnostics` | `std_msgs/String`（JSON：views/拒帧/TF/点数/overlap/tsdf 等） |
| pub | `/peach/reconstruction/markers` | `visualization_msgs/MarkerArray`（相机轨迹） |
| srv | `~/start_reconstruction` 等 6 个 | `std_srvs/Trigger` |

四个 `/peach/reconstruction/*` 发布者均为 **transient_local 闩锁**（depth=1）：
启动时先发一次 IDLE + 空云，之后每次服务调用重发；后启动的订阅者
（`tools/peach_validation_recorder.py`、RViz）也能拿到最后一次发布。

### session 落盘格式

`session.root_dir/session_YYYYMMDD_HHMMSS/`：每帧 `frame_XX_rgb.png`（BGR）、
`frame_XX_depth.npy`（uint16 mm）、`frame_XX_camera_info.yaml`、
`frame_XX_T_base_camera.yaml`，外加 `metadata.yaml`（参数快照 + 帧级摘要 +
TSDF 摘要）；若 finalize 已生成 TSDF 云，另存 `result/tsdf_cloud.ply`
（xyz+rgb ASCII）。
