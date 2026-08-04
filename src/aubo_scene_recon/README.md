# aubo_scene_recon — Eye-in-hand 点云场景重建

## 简介

订阅 Percipio 相机数据，按 TF 把每帧拼到 `map_frame`（默认 `base_link`）
累积成场景点云，实时发布并可保存 PLY。两个后端：**open3d**（默认，彩色点云
累加 + 体素 + 统计滤波）与 **tsdf**（RGB-D TSDF 融合）。只建图，不驱动机械臂。

## 使用方法

### 依赖

```bash
aubo_py3.12/bin/pip install -r requirements.txt   # 含 open3d==0.19.0
# 或只装本包所需: aubo_py3.12/bin/pip install 'open3d==0.19.0'
```

### 运行

前置条件（缺一不可）：

1. **TF 链**：`base_link → wrist3_Link`（bringup，real/sim 均可）+
   `wrist3_Link → camera_link`（extrinsics_publisher，bringup 默认自带，
   `extrinsics_enabled:=true`）。标定没做时发布的是标称外参，重建会有
   系统性偏移。
2. **相机数据**：`ros2 launch percipio_camera percipio_rgbd.launch.py`
   —— 在默认 `percipio_camera.launch.py` 之上强制打开深度、深度到 color
   配准与点云（`depth_enable` / `depth_registration_enable` /
   `point_cloud_enable` / `color_point_cloud_enable`），open3d 与 tsdf
   两个后端的数据都由它提供。

启动（先 source 环境）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws && source install/setup.bash

ros2 launch aubo_scene_recon recon.launch.py                  # 默认 open3d 后端
ros2 launch aubo_scene_recon recon.launch.py backend:=tsdf    # TSDF（订阅 color+depth）
```

launch 参数：`params_file`（默认包内 `config/recon.yaml`）、`backend`
（open3d|tsdf）、`pointcloud_topic`（默认 `/camera/depth_registered/points`）、
`save_dir`（默认空串，见"输出物"节）。

然后让相机扫过场景（RViz 拖动手腕 / MoveIt 规划，慢速），累积地图以
`publish_period_sec` 周期发到 `/recon/map_cloud`。可选 RViz 配置：

```bash
rviz2 -d "$(ros2 pkg prefix aubo_scene_recon)/share/aubo_scene_recon/rviz/recon.rviz"
```

### 话题与服务

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| sub | `pointcloud_topic` | `sensor_msgs/PointCloud2` | 彩色点云（open3d 后端） |
| sub | `color_topic` + `depth_topic` + `color_info_topic` | Image / Image / CameraInfo | RGB-D（tsdf 后端，ApproximateTime 同步，slop 0.1 s；深度须 16UC1 且与彩色同分辨率） |
| pub | `/recon/map_cloud` | `sensor_msgs/PointCloud2` | 累积地图（frame=`map_frame`） |
| pub | `/recon/status` | `std_msgs/String` | `frames_ok/frames_dropped/map_points/backend` 计数 |
| srv | `~/reset` | `std_srvs/Trigger` | 清空地图与帧计数 |
| srv | `~/save` | `std_srvs/Trigger` | 保存 PLY（空图返回 `success=false, "empty map"`） |

```bash
ros2 service call /recon_fusion_node/save std_srvs/srv/Trigger {}
ros2 service call /recon_fusion_node/reset std_srvs/srv/Trigger {}
ros2 topic echo /recon/status --once
```

启动后超过 `no_cloud_warn_sec`（默认 5 s）仍无输入会周期告警——通常是
percipio_rgbd 没起或话题名不匹配。

### 参数（权威源 `config/recon.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `backend` | open3d | 融合后端：open3d（点云累加）/ tsdf（RGB-D TSDF） |
| `map_frame` | base_link | 地图坐标系（融合与发布的参考系） |
| `pointcloud_topic` | /camera/depth_registered/points | 输入彩色点云（open3d） |
| `color_topic` / `depth_topic` / `color_info_topic` | /camera/color/image_raw 等 | RGB-D 输入（tsdf） |
| `voxel_size` | 0.005 | 体素下采样边长 (m)，0 关闭 |
| `sdf_trunc` | 0.04 | TSDF 截断距离 (m) |
| `depth_scale` | 4000.0 | 深度(m)=raw/depth_scale；Percipio DepthScaleUnit 0.25 → 4000 |
| `min_range` / `max_range` | 0.2 / 3.0 | 有效深度裁剪 (m)；`max_range` 兼作 TSDF depth_trunc |
| `tf_timeout_sec` | 0.5 | TF 查询超时 (s) |
| `publish_period_sec` / `status_period_sec` | 0.5 / 1.0 | 地图 / 状态发布周期 (s) |
| `max_map_points` | 2000000 | 地图点数上限，超限自动加大体素再压 |
| `outlier_every_n` | 5 | 每 N 帧做一次统计离群滤波，0 关闭（open3d） |
| `save_dir` | ""（空串） | 地图保存目录；空串回退 `<进程CWD>/recon_maps` |
| `no_cloud_warn_sec` | 5.0 | 启动后无输入的告警时限 (s) |

### 两个后端怎么选

- **open3d（默认，首选）**：输入厂商已配准的彩色点云，逐帧 TF 变换后累加，
  每帧 `voxel_down_sample(voxel_size)`，每 `outlier_every_n` 帧做一次统计
  滤波（nb_neighbors=20，std_ratio=2.0），点数超 `max_map_points` 自动把
  体素 ×1.5 迭代压缩。真彩稳定、链路最短；深度比例、配准误差都由相机
  驱动兜底，不涉及 `depth_scale`。
- **tsdf**：输入 color+depth+CameraInfo，`ScalableTSDFVolume`
  （`voxel_length=voxel_size`、`sdf_trunc`、RGB8）多帧加权融合后提取点云，
  几何更干净（多帧加权去噪）。两个注意点：① 深度比例必须配对——
  Percipio DepthScaleUnit 常见 0.25，即 `depth_scale=4000`；若重建整体
  偏近/偏远，改 `config/recon.yaml` 的 `depth_scale` 后重启节点（参数仅
  启动时读取），或回退 open3d 后端（厂商点云不涉及该参数）；
  ② ScalableTSDF 偶发几何正常但顶点色全 0，此时自动改用高度伪彩并
  提示一次——要相机真彩请用 open3d。

### 输出物

`~/save` 把当前地图写成 **二进制 PLY**（xyz + rgb）：

```text
<save_dir>/scene_YYYYMMDD_HHMMSS.ply
```

`save_dir` 默认空串 → 回退 `<进程CWD>/recon_maps`（share 目录无法可靠反推
工作区根，故不硬编码绝对路径）。从工作区根起 launch 时即落在仓库的
`recon_maps/`；节点 ready 日志会打印实际 `save_dir=`，以服务返回值为准。

### 测试

```bash
source /opt/ros/jazzy/setup.bash
cd src/aubo_scene_recon && ../../aubo_py3.12/bin/python -m pytest test/ -q
```

`test_pc_utils.py` 4 例（点云工具与融合后端，需 venv 内 open3d）+
flake8/pep257 lint。`colcon test` 走系统 python3，缺 open3d 时
`test/conftest.py` 自动跳过 `test_pc_utils.py`，属预期。

## 执行逻辑

### 数据流

```text
percipio_rgbd（点云 / color+depth+CameraInfo）
  → recon_fusion_node（单节点）
      ├─ TF 查 map←camera：先按图像 stamp，失败回退最新 TF
      │    （相机 HW 时间戳常超前机器人 TF，严格按 stamp 会
      │    extrapolation into the future 丢帧，重建缺块/错位）
      ├─ backend.integrate()：open3d 点云累加 或 TSDF 体融合
      ├─ 定时发布 /recon/map_cloud（PointCloud2）与 /recon/status（String）
      └─ 服务 ~/save（写 PLY）、~/reset（清空地图）
```

## 软件框架

### 节点入口（venv 包装脚本）

节点**不走 console_scripts**（系统 python3 没装 open3d）：launch 以标准
`Node()` 调 `scripts/recon_fusion_node` 包装脚本，脚本按优先级选解释器——
`AUBO_PYTHON` 显式指定 > 已激活的 venv（`VIRTUAL_ENV` 非空）> 自动
`source` 工作区 `aubo_py3.12/bin/activate`——最终以
`python3 -m aubo_scene_recon.fusion_node` 运行。18 个参数全部带中文
`ParameterDescriptor`（`ros2 param describe /recon_fusion_node <参数>` 可查）。
