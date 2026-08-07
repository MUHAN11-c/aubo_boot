# PeachPose 三功能包开发进度台账

> 最后更新：2026-08-07（Phase 4 完成）。每完成一个 Phase 或中途重要变更时更新本文。
> 背景与需求见用户提供的《PeachPose ROS2 三功能包最终开发方案》；Phase 0 审计报告在会话中，
> 本文是持续维护的进度快照。

## 一、总体状态

| Phase | 内容 | 状态 |
|---|---|---|
| Phase 0 | 现有代码审计（《PeachPose ROS2 当前架构审计报告》） | ✅ 完成 |
| Phase 1 | 整理 peach_pose_ros2（功能不退化） | ✅ 完成并验证 |
| Phase 2 | 新建 peach_reconstruction_ros2（手动采帧 + 原始累加，无 TSDF） | ✅ 完成并验证 |
| Phase 3 | 多帧刚性对齐验证手段（overlap 指标 + 合成圆柱测试） | ✅ 软件侧完成；⏳ **真机验证待用户执行** |
| — | 过程数据记录器（validation_runs 落盘） | ✅ 完成（方案外追加需求） |
| — | 重建改全自动（自动开始/采帧/完成，零服务） | ✅ 完成（方案外追加需求） |
| — | 累加云 RGB 彩色、debug_image 掩膜改轮廓 | ✅ 完成（方案外追加需求） |
| — | 末端 TCP 定义（aubo_e5_moveit_config tcp.yaml） | ✅ 完成（方案外追加需求） |
| Phase 4 | Open3D TSDF 融合 | ✅ 完成并验证 |
| Phase 5 | TSDF 云几何二次拟合（圆柱/球 refit） | ⬜ 未做 |
| Phase 6 | 新建 peach_pose_fusion_ros2（single vs refined 一致性） | ⬜ 未做 |
| Phase 7 | Session 离线回放复算（参数扫描不需真机） | ⬜ 未做（save_session 落盘已具备） |

## 二、已完成明细

### Phase 0：审计（2026-08-07）

- 输出 22 项审计报告：目录树/节点/话题/参数/TF 链/调用链/fitting 路径/axis 定义/底颈 entry 计算/安全门控/replay 机制/Python 环境/相机耦合点/技术债清单/两新包目录设计/三包数据流/分阶段迁移方案。
- 关键结论：算法核心（pipeline/fitting/contracts）零相机依赖；`depth_scale_unit=0.25` 语义=raw×0.25mm；重力为固定相机系向量（非 TF）；`suggested_travel_end` 漏做 TF 变换（真 bug）；32FC1 深度静默丢帧；aubo_scene_recon 的 TSDF 后端与 pc_utils 可复用。

### Phase 1：peach_pose_ros2 整理（2026-08-07）

- 修 `suggested_travel_end`/`position` 漏 TF 变换；detections/masks/debug_image 的 frame_id 改为图像自身坐标系。
- 新增 `peach_pose/depth_geometry.py`：深度归一化唯一入口（uint16×scale / 32FC1 米制×1000），32FC1 不再丢帧。
- 新增 `gravity_mode`（fixed 默认=旧行为 / tf=由本帧 TF 旋转反推相机系重力，只乘旋转）。
- TF 回退打标 `tf_stale`/`tf_unavailable`；RGB-D 时间戳偏差日志。
- 新增 7 个 `/peach/perception/*` 规范化话题（与旧 `~/` 话题并行，旧接口全保留）。
- 新增测试 test_depth_geometry.py + test_transform_contract.py（方向向量不受平移影响、travel_end 回归）。
- 验证：lint 绿；pytest 42 例过；数据集回放冒烟有候选且 tf_unavailable 打标正确。

### Phase 2：peach_reconstruction_ros2 新建（2026-08-07）

- ament_python 新包（venv console_scripts 约定，接入 lint）：6 个 Trigger 服务、CapturedFrame 内存缓存、TF 按 depth.header.stamp 查询（stamp 失败回退 latest 打 tf_stale，彻底失败拒帧）、视角有效性过滤、base_link 多帧原始累加。
- 话题：/peach/reconstruction/{local_cloud,status,diagnostics,markers}（相机轨迹）。
- 测试 23 例（反投影解析对齐、R@p+t、inv(T)@T=I、视角过滤 11 例）。

### Phase 3：对齐验证手段（2026-08-07）

- overlap.py：finalize 时输出相邻帧最近邻距离 mean/median/p95 [mm] + 逐帧质心。
- test_multiview_alignment.py：合成圆柱 5 视角，正例 NN mean 0.59mm/质心误差 0.71mm；反例（外参错 ±20mm）mean 16.5mm——指标灵敏度 ~25×。
- 真机判据：mean < 5mm 且 p95 < 15mm；不合格按 depth scale→内参→registration→手眼→TF 方向→时间戳→运动中曝光排查。
- **真机验证待做**：用户按手册跑 4~5 视角手动采帧，回报 overlap 实测分布。

### 方案外追加（用户驱动，2026-08-07）

- **aubo_e5_moveit_config**：新增 `config/tcp.yaml`（TCP 相对 wrist3_Link：x=0.00 y=47.90 z=151.07 mm，姿态未标定取恒等），launch 经 static_transform_publisher 发 wrist3_Link→tcp 静态 TF；package.xml 补 tf2_ros。
- **过程数据记录器** `tools/peach_validation_recorder.py`：每步 16 件产物（相机 RGB/深度 npy+伪彩、感知话题 YAML、点云 PLY、cloud_render.png 双视图、参数 dump、TF 快照、manifest.yaml + run 级 index.md）；`validation_runs/` 已 gitignore。
- **重建全自动**：`capture.auto_mode=true` 默认——有候选自动开始、视角平移 ≥2cm 或旋转 ≥5° 自动采帧（间隔门 2s）、满 8 视角自动 finalize；手动服务全保留备用。
- **local_cloud 彩色化**：xyz+rgb 打包（RViz 选 RGB8），逐点颜色比对验证。
- **debug_image**：SAM 掩膜半透明填色改 2px 轮廓线。
- 记录器/重建包 QoS：/peach/reconstruction/* 改 transient_local 闩锁（RViz 需把该 Display 的 Durability 设 Transient Local 才能看到历史帧）。
- peach_gantry_description / peach_moveit_config 标记为【新结构模型，当前不参与】（AGENTS.md 已记）。

### Phase 4：TSDF 融合（2026-08-07）

- tsdf_volume.py（LocalTsdf 封装 open3d 0.19 ScalableTSDFVolume，懒加载）；finalize 时批量积分（保留 remove_last 回滚），8 帧 1280×720 积分约 2.0s。
- 外参方向：T_camera_base = 刚体逆(T_base_camera)；守门测试实测正确 4.15mm vs 反接 853.4mm（200× 量级差）。
- 后处理链：ROI 裁剪（local_volume 0.30×0.30×0.40m，中心=候选 bottom/neck 中点）→ voxel 0.003m → 统计离群剔除。
- 新话题 /peach/reconstruction/tsdf_cloud（xyz+rgb、闩锁）；diagnostics 加 tsdf 键（points/integrate_time_s/…）；session 加 result/tsdf_cloud.ply。
- 新参数 tsdf.enable=true、cloud_filter.voxel_size=0.003、cloud_filter.enable_statistical_filter=true（共 31 参数）。
- 合成测试：质心误差 4.15mm、径向中位 0.0352m（GT 0.035）、TSDF vs raw 质心距 3.23mm；pytest 44 例过。

## 三、未做事项

### Phase 5：几何二次拟合（下一步）

- 对 TSDF 融合云复用 peach_pose_ros2/fitting.py 做圆柱（袋桃）/球（裸桃）refit；方向二次消歧（bottom→neck）；输出 refined center/axis/bottom/neck/diameter/rmse/inlier。
- 发布 /peach/reconstruction/{refined_pose,refined_axis} 等；diagnostics 补 refined 字段。

### Phase 6：peach_pose_fusion_ros2 新建

- 三包之三：订阅 /peach/perception/* 与 /peach/reconstruction/refined_*；Δp/Δθ（不加 abs）/ΔD 一致性判断；状态机 NO_TARGET/INITIAL_ONLY/RECONSTRUCTING/REFINED/ACCEPT/INCONSISTENT；发布 /peach/final_*。
- 阈值参数化（max_position_error=0.015 / max_axis_error_deg=8.0 / max_diameter_error=0.010 起步）。
- 纯 ROS+numpy，不依赖 torch/open3d。

### Phase 7：Session 离线回放复算

- save_session 落盘已具备（rgb.png/depth.npy/camera_info.yaml/T_base_camera.yaml + result/*.ply + metadata.yaml）。
- 待做：离线重放入口（读 session 目录重跑 TSDF/refit/fusion 参数扫描，不起真机）。

### 真机验证（用户侧待做/进行中）

- Phase 3 手册真机跑：4~5 视角，finalize 读 overlap mean/p95；合格线 mean<5mm、p95<15mm。
- 相机帧率排查：`ros2 topic hz /camera/depth/image_raw`；确认 /camera/depth_registered/points 无订阅者（RViz Display 会触发驱动端组包开销）。
- 记录器在关键步骤 record 落盘，最终综合报告由 validation_runs 汇编（报告未出）。

### 已知技术债（不阻塞，按优先级）

- peach_moveit_config/README.md 的"新机械臂不参与"备注已保留（用户确认）。
- TSDF 质心公差余量不大（4.15/5mm）；无候选时 ROI 中心用首帧质心可能偏。
- 手眼外参更新后必须重启 extrinsics_publisher（tf2 静态 TF 不覆盖）。
- 相机 HW 时间戳超前导致 tf_stale；真机若占比高，加大 tf_timeout_sec 或查时间源。
- peach_pose_ros2 隐式依赖 scipy/torch/ultralytics 未入 package.xml；算法阈值（深度窗/RANSAC 等）仍硬编码。
