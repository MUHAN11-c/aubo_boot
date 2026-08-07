# peach_recon

## 简介

桃果场景的**纯三维重建包**：订阅对齐 RGB-D + CameraInfo 与 TF，用
Open3D 官方 TSDF 管线（`ScalableTSDFVolume`，RGB8 真彩）逐帧融合，
定时提取点云并按 ROI 盒裁剪后发布 `~/roi_cloud`；`~/reset`(Trigger)
重建 volume。

架构定位（翻转后）：**只产点云，不知道抓取语义**——候选时序融合、
ROI 云重拟合、最终方向/抓取定位全部在 peach_pose_ros2（感知决策层）。
唯一残留的耦合是对 grasp_candidates 的订阅，仅取"当前最优候选位置"
作 ROI 盒中心。

节点经 console_scripts + venv shebang 运行（setup.py 构建期把启动器
解释器解析到工作区 `aubo_py3.12` venv，open3d 在 venv；约定同
peach_pose_ros2 / aubo_scene_recon）。

## 使用方法

```bash
colcon build --packages-select peach_recon
source install/setup.bash
ros2 launch peach_recon recon.launch.py        # 纯 TSDF 重建节点
```

话题与服务：

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| sub | `/camera/color/image_raw` 等三路 | `Image/CameraInfo` | RGB-D（TSDF 融合输入，近似同步） |
| sub | `/peach_pose_node/grasp_candidates` | `BagGraspCandidateArray` | 仅取最优候选位置作 ROI 中心 |
| pub | `~/roi_cloud` | `sensor_msgs/PointCloud2` | TSDF 提取云按 ROI 盒裁剪（1 Hz，xyz+rgb） |
| srv | `~/reset` | `std_srvs/Trigger` | 重建 TSDF volume |

参数见 `config/recon.yaml`（订阅话题/坐标系/TSDF/ROI 共 13 项，全部中文
行内注释；`tsdf_voxel_length`/`sdf_trunc` 为重建内核参数）。

## 执行逻辑

1. RGB-D + CameraInfo 三路近似同步（QoS/slop 写法同 peach_pose_ros2）。
2. 每帧：cv_bridge 转换 → TF 查相机位姿（输出系←相机系，T_map_cam）→
   `TsdfRecon.integrate_rgbd()`：`RGBDImage.create_from_color_and_depth`
   （depth_scale=1000，毫米）→ `volume.integrate(rgbd, intrinsic,
   inv(T_map_cam))`（Open3D integrate 需要 camera→world，故取逆，
   参照 aubo_scene_recon 的 TSDF 后端写法）。
3. 定时（roi_cloud_period_sec）：`extract_point_cloud()` → 若有 ROI 中心
   （最新最优候选位置）按 roi_size_m 盒裁剪 → 颜色 [0,1]→uint8→位打包
   → `~/roi_cloud` 发布（工具函数复用 peach_pose_ros2.visualization）。
4. `~/reset`：新建 ScalableTSDFVolume（清空全部融合）。

## 软件框架

```
peach_recon/
├── peach_recon/
│   ├── roi_accumulator.py     # TsdfRecon：Open3D ScalableTSDFVolume 封装
│   ├── eval_refinement.py     # 三路对比评估（A 单帧/B 融合/C 重建精化，
│   │                          #   B/C/stats 订阅 peach_pose_node 决策输出）
│   └── recon_refiner_node.py  # 纯重建节点 + main（console_scripts 入口）
├── config/recon.yaml          # 13 项参数（与代码 DEFAULTS 对齐）
├── launch/recon.launch.py     # 标准 Node() 启动
├── setup.py                   # options.build_scripts.executable → aubo_py3.12 venv
└── test/                      # lint + TSDF smoke（open3d 缺失经 conftest 跳过）
```

依赖：rclpy、peach_pose_msgs、geometry_msgs、std_msgs、std_srvs、
peach_pose_ros2（TF/点云工具函数）；解释器为工作区 venv
（aubo_py3.12，numpy 1.26 + open3d 0.19）。
