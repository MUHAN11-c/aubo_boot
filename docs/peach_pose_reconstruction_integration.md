# 桃子首帧感知与连续局部重建联动说明

本文定义 `peach_pose_ros2` 与 `peach_reconstruction_ros2` 的跨包接口和现场流程。两包共同完成“全局拍照建档 → 选择目标 → 接近过程中连续重建 → 输出精化抓取位姿 → 完成后切换下一目标”，但都不直接发送机械臂运动命令。

> **当前范围（2026-08）**：本联动方案以套袋桃为开发、调参和真机验收对象。
> 主路径为袋类检测 → 袋体掩膜 → 圆柱/袋轴几何 → 局部 TSDF → 圆柱 refit →
> 沿袋轴接近。裸桃球拟合及果梗方向能力仅保留为研究/兼容分支，不属于当前
> 工具接触和抓取验收范围。

- `peach_pose_ros2`：首帧发现、数量锁定、稳定 ID、分割掩膜、初始三维候选、质量、优先级与采摘状态。
- `peach_reconstruction_ros2`：选中目标的逐帧门控、精确时刻 FK/TF、受限 ICP、在线 TSDF、最终几何拟合与抓取许可。
- MoveIt 或任务管理器：读取两包输出，规划观察、接近和抓取动作。

两个包的参数权威源分别是 `src/peach_pose_ros2/config/peach_pose.yaml` 和 `src/peach_reconstruction_ros2/config/reconstruction.yaml`。

## 1. 职责边界

| 阶段 | `peach_pose_ros2` | `peach_reconstruction_ros2` | 上层任务管理器 |
|---|---|---|---|
| 全局观察 | 检测全部桃子并建立世界系目标表 | 保持 `IDLE` | 移动到全局观察位 |
| 数量锁定 | 锁定已确认目标、数量与固定优先级 | 读取选中目标 | 记录本轮 run ID |
| 接近目标 | 持续发布状态、掩膜和初始位姿 | 同步深度、掩膜、TF 并融合合格帧 | 低速规划观察轨迹 |
| 精化结果 | 保留初始结果，不被重建覆盖 | 发布 refined 位姿、轴线、诊断和许可 | 通过全部安全门后抓取 |
| 完成切换 | 标记当前目标完成并推进下一个 ID | 结束旧会话后绑定下一目标 | 抓取与撤离确认后调用完成服务 |

二维检测负责“发现和身份”，三维重建负责“靠近后的几何精化”。重建不会增加首帧目标数量，也不会改写 `/peach/perception/initial_pose`，最终结果通过 `/peach/reconstruction/refined_pose` 独立发布。

## 2. 总体链路

```text
Percipio RGB + Depth + CameraInfo
              │
              ▼
peach_pose_ros2
YOLO → SAM → 深度反投影 → 几何拟合 → base_link 变换
              │
              ├─ target_id / priority / harvest_status
              ├─ selected_target_id / 逐目标 mask
              └─ /peach/perception/initial_pose
                              │
                              ▼
peach_reconstruction_ros2
同时间戳 mask + depth + 精确 TF/FK
              │
              ├─ 质量、视角与漂移门控
              ├─ FK 绝对位姿 + 受限 ICP 小修正
              ├─ 合格帧即时积分 TSDF
              └─ finalize 后重新拟合球或圆柱
                              │
                              ▼
refined_pose / refined_axis / grasp_decision / diagnostics
                              │
                              ▼
MoveIt 或采摘任务管理器
```

## 3. 跨包接口

### 3.1 感知包提供给重建包

| 话题 | 类型 | 用途 |
|---|---|---|
| `/peach/perception/initial_pose` | `BagGraspCandidateArray` | 世界系初始候选、目标类型与质量状态 |
| `/peach/perception/target_observations` | `PeachTargetObservationArray` | 选中 ID、优先级、跟踪状态、逐目标掩膜和诊断 |
| `/peach/perception/diagnostics` | `BagFittingArray` | 初始拟合与 ACCEPT、REOBSERVE、REJECT 判定 |
| 相机 RGB、深度、CameraInfo | 标准 `sensor_msgs` | 彩色局部 TSDF 原始观测 |
| `/joint_states` 与 TF | 标准 ROS 接口 | 曝光时刻 FK 与手眼位姿 |

`PeachTargetObservation` 是主契约，包含稳定 `target_id`、固定 `priority`、`confirmed`、`selected`、采摘状态、跟踪状态、初始三维与二维候选、拟合结果、该目标独立掩膜及异常标志。重建使用独立掩膜，不使用整幅多目标掩膜画布。

### 3.2 重建包提供给下游

| 话题 | 含义 |
|---|---|
| `/peach/reconstruction/status` | 状态和帧数摘要 |
| `/peach/reconstruction/diagnostics` | 目标绑定、门控、FK/ICP 和 TSDF 诊断 |
| `/peach/reconstruction/grasp_decision` | 是否允许抓取及原因 |
| `/peach/reconstruction/local_cloud` | 当前掩膜裁剪后的单帧局部点云 |
| `/peach/reconstruction/tsdf_cloud` | 在线融合点云 |
| `/peach/reconstruction/refined_pose` | finalize 后精化抓取位姿 |
| `/peach/reconstruction/refined_axis` | 精化主轴 |
| `/peach/reconstruction/refined_diagnostics` | 最终拟合质量 |
| `/peach/reconstruction/markers` | RViz 几何标记 |

`grasp_decision.allowed=true` 只表示视觉门通过，不代表 MoveIt 可达性、碰撞、安全区和末端执行器条件已通过。

## 4. 时间戳、坐标系与掩膜

1. 感知三维计算和重建 TF 查询都以 `depth.header.stamp` 为基准。
2. `PeachTargetObservation.mask.header.stamp` 使用对应深度时间戳；重建只接受同时间戳掩膜。
3. 图像平面调试结果可以保留 RGB 时间戳，但不能替代重建的深度时间戳契约。
4. 世界系结果默认在 `base_link`，原始点位于相机光学坐标系。
5. 重建禁止退回 latest TF。图像时刻 TF 不可用时必须丢弃该帧。
6. FK 与手眼外参给绝对相机位姿；ICP 只能在配置边界内做小修正。

只有当前选中目标处于 `OBSERVED`、掩膜非空且时间戳匹配时才可融合。默认还要通过掩膜像素数、有效深度比例、目标中心漂移和帧龄门限。

## 5. 全局目标计划

首次出现已确认且可选择的目标时，感知包锁定本轮目标集合：

- `target_set_locked=true`，`target_count` 固定。
- 按可用状态、距离、置信度和 ID 形成固定顺序。
- 创建本轮 `harvest_run_id`，两包的数据都关联到该运行目录。
- 锁定后新出现目标不自动加入，避免接近过程中数量和优先级跳变。

状态含义：

| 状态 | 含义 |
|---|---|
| `PLANNED` | 在固定计划中，尚未轮到 |
| `SELECTED` | 当前接近和重建目标 |
| `WAITING_QUALITY` | 跟踪、TF 或质量暂不合格 |
| `HARVESTED` | 已由上层确认完成 |

`TargetRegistry` 使用 `base_link` 空间匹配维护身份，短时遮挡后应尽量复用原 ID。若全局数量或观察质量不正确，应调整全局观察位后调用 reset 开始新一轮。

抓取和撤离确认完成后再调用：

```bash
ros2 service call /peach_pose_node/complete_selected_target std_srvs/srv/Trigger "{}"
```

感知包会推进到下一个当前可用且未完成的固定 ID。若剩余目标暂不可用，selected 可暂时为空，恢复后继续。

## 6. 重建状态机与逐帧流程

| 状态 | 行为 |
|---|---|
| `IDLE` | 等待候选和选中目标 |
| `COLLECTING` | 接收合格帧并在线积分 TSDF |
| `READY` | finalize 完成，发布最终产物并停止采集 |
| `FAILED` | 预留状态；当前异常主要由诊断和服务失败返回 |

默认 `capture.auto_mode=true`，收到可用候选后从 `IDLE` 自动开始。默认
`capture.auto_finalize_at_max=false`，达到足够视角后仍需显式 finalize；设为 true
时最大视角数可触发自动 finalize。

每帧按以下顺序处理：

1. 检查状态和帧龄。
2. 取得绑定 ID 的同时间戳逐目标掩膜。
3. 检查跟踪、掩膜像素、有效深度和中心漂移。
4. 生成局部深度与点云，执行 ROI、距离、体素和离群点处理。
5. 查询图像精确时刻 `base_link → camera_link`。
6. 检查采集间隔、平移与旋转基线。
7. 用当前局部点云对已有模型执行受限点到平面 ICP。
8. ICP 合格则采用小修正；ICP 不合格但 FK 初始重叠合格时用 FK fallback；两者都不合格则拒绝。
9. 使用最终相机位姿即时积分 TSDF。
10. 发布点云、状态和诊断，并记录运行数据。

自动模式会跳过近重复视角；超过期望最大步长时会警告但仍可能采集。手动 `capture_frame` 的视角检查更严格，过大移动会拒绝。自动接近仍应连续、低速、平滑。

至少达到 `min_views` 后调用：

```bash
ros2 service call /peach_reconstruction_node/finalize_reconstruction std_srvs/srv/Trigger "{}"
```

finalize 检查覆盖与重叠，提取点云和网格，并按目标类型重新拟合：袋体用圆柱，裸果用球。成功后进入 `READY` 并发布 refined 结果。它不会自动运动，也不会自动保存完整 session。

## 7. FK、ICP 与最终结果

| 名称 | 来源 | 作用 |
|---|---|---|
| 初始候选 | YOLO、SAM、单帧深度拟合 | 首帧发现、排序和接近初值 |
| FK 相机位姿 | 关节状态、运动学、手眼外参 | 决定每帧点云在世界系的绝对位置 |
| ICP 修正 | 当前点云与已有模型配准 | 补偿小量位姿、同步和深度误差 |
| refined 位姿 | 多帧 TSDF 后的球或圆柱重拟合 | 最终抓取几何 |

模型点不足时使用 `model_warmup`。只有适应度、RMSE、平移和旋转边界全部通过才采用 `mode=icp`。ICP 超界通常意味着串掩膜、时间戳、外参或重叠问题，应回退 FK 或拒绝，不能放宽限制强行对齐。

## 8. 绑定和切换规则

运行时必须保持：

```text
harvest_state.selected_target_id
    == target_observations 中 selected 的 ID
    == reconstruction diagnostics.target_id
    == 当前 session.target_id
```

重建在 `READY` 时收到新首选 ID，会清空旧帧、TSDF、最终产物和掩膜缓存，回到 `IDLE` 后绑定下一目标。若仍在 `COLLECTING`，实现会拒绝中途切换，防止两目标点云混合；应先 finalize/save，或 reset 后切换。

候选选择器对非空 `selected_target_id` 实施严格硬过滤。首选目标为 `REOBSERVE` 时可以继续使用该目标的安全初值；首选目标缺失、`REJECT` 或带阻断 TF 标记时返回无候选，不会改绑其他目标。采集前仍须核对上述 ID 不变量；不相等通常表示运行了旧进程或不同版本节点，应停止融合、reset 并重启。

## 9. 真机完整流程

启动前先查残留进程：

```bash
pgrep -af "ros2 launch|component_container|extrinsics_publisher|ros2 run"
```

机械臂上电、安全恢复只能由现场用户通过示教器或控制柜完成。首次运动速度和加速度缩放不高于 0.1。

若相机单独启动，bringup 必须关闭重复相机：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_e5_bringup bringup.launch.py \
  hardware_mode:=real camera_enabled:=false
```

其余终端分别启动：

```bash
ros2 launch percipio_camera percipio_rgbd.launch.py
ros2 launch peach_pose_ros2 peach_pose.launch.py
ros2 launch peach_reconstruction_ros2 reconstruction.launch.py
```

重建 launch 默认加载包内 `config/reconstruction.yaml`。

全局拍照后查询计划：

```bash
ros2 service call /peach_pose_node/query_harvest_state std_srvs/srv/Trigger "{}"
```

核对锁定状态、数量、选中 ID 和优先级。然后上层以 initial pose 作为低速接近初值，接近期间观察重建状态、诊断、局部点云和 TSDF 点云；确认绑定 ID 一致，视角达到要求后 finalize。仅在 `grasp_decision.allowed=true` 且规划、安全和末端条件也通过时抓取。抓取与撤离成功后调用完成服务，进入下一目标。

## 10. 异常处理

| 场景 | 处理 |
|---|---|
| 目标短时遮挡 | 保留计划和 ID，等待恢复或换观察角度 |
| 掩膜串目标、TSDF 双影 | 拒绝帧，检查 ID、掩膜和视角，不放宽 ICP 掩盖问题 |
| TF 缺失或过期 | 检查 joint_states、手眼 TF 和同步，禁止 latest TF 回退 |
| 深度空洞 | 调整距离、曝光或视角，不融合该帧 |
| 机械臂过快 | 降速并增加连续视角 |
| ICP 失败、FK 合格 | 可 fallback，同时检查重叠；频繁发生则复查标定和同步 |
| ICP 与 FK 都失败 | 拒绝帧并回到重叠更好的观察位 |
| 采集中 selected 改变 | finalize/save 当前会话，或 reset 后重新绑定 |
| finalize 视角不足 | 继续采集不同基线视角再重试 |

## 11. 数据管理

本轮采摘默认写入工作区 `harvest_runs/`，也可用 `AUBO_HARVEST_DATA_DIR` 覆盖：

```text
harvest_runs/<harvest_run_id>/
├── manifest.yaml
├── events.jsonl
├── latest_perception.json
├── latest_reconstruction.json
└── masks/<stamp_ns>_<target_id>.png
```

完整重建 session 默认写入 `peach_sessions/session_<timestamp>/`，包含逐帧
`frame_XX_rgb.png`、`frame_XX_depth.npy`、相机内参与 FK/ICP 位姿 YAML、
`metadata.yaml`，以及可选的 `result/tsdf_cloud.ply` 和 `result/tsdf_mesh.ply`。

```bash
ros2 service call /peach_reconstruction_node/save_session std_srvs/srv/Trigger "{}"
ros2 service call /peach_reconstruction_node/query_reconstruction_state std_srvs/srv/Trigger "{}"
```

`events.jsonl` 用于时间序列回放，`latest_*.json` 用于 UI 快速查询，session 用于离线复现几何问题。

## 12. RViz 与验收

RViz2 建议设置 Fixed Frame 为 `base_link`，添加：

- PointCloud2 `/peach/reconstruction/local_cloud`：检查单帧掩膜是否干净。
- PointCloud2 `/peach/reconstruction/tsdf_cloud`：检查模型是否变完整且无重影。
- MarkerArray `/peach/perception/markers`：检查初始目标和 ID。
- MarkerArray `/peach/reconstruction/markers`：检查精化中心、轴线和拟合几何。
- TF：检查 `base_link → wrist3_Link → camera_link`。

完整验收应满足：数量和优先级锁定稳定；遮挡恢复 ID 不变；感知、重建、session 的目标 ID 一致；积分帧都有同时间戳掩膜和精确 TF；ICP 修正有界；TSDF 无明显重影或邻近目标混入；抓取端使用 refined 而非 initial；完成服务只在实际抓取和撤离成功后调用。

## 13. Web 集中查看

`peach_perception_web` 可把本文件涉及的目标计划、初始/refined 位姿、ID 绑定、
FK/ICP 诊断、TSDF 数值摘要和抓取许可集中显示在浏览器中：

```bash
ros2 launch peach_perception_web peach_perception_web.launch.py
```

本机访问 `http://127.0.0.1:8090`。现场平板访问时显式传
`host:=0.0.0.0`，并使用机器人电脑的局域网 IP。Web 包只订阅和显示，不调用本文件
列出的 reset、complete、finalize、save 或任何机械臂运动接口；算法验收仍以 ROS
原始话题、RViz 和落盘 session 为准。图像、Marker、局部点云和 TSDF 点云不经过
Web 包，统一在 RViz2 中查看。
