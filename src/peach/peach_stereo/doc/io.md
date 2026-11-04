# `peach_stereo` io（活文档）

> 当前接口快照。节点 `peach_stereo_camera_node`，launch 默认 namespace `camera`；
> 改话题/QoS/参数须同一轮改本文与 [architecture.md](architecture.md)。

## 话题

| 名字（namespace 内） | 类型 | QoS | 含义 |
|---|---|---|---|
| `color/image_raw` | `sensor_msgs/msg/Image` | depth 1，Reliable / Volatile（参数化） | BGR8 彩色图，frame `camera_color_optical_frame`；YUYV 解码后厂参 `TYUndistortImage`（与官方驱动同调用，对齐棋盘格所用官方彩图）；深度计算前先发 |
| `depth/image_raw` | `sensor_msgs/msg/Image` | 同上 | `16UC1` 深度，**单位 0.25 mm**，0=无效；frame `camera_depth_optical_frame`；配准后再按彩色厂参最近邻 remap 到与彩图同一网格 |
| `color/camera_info` | `sensor_msgs/msg/CameraInfo` | 同上 | 彩色内参；可用 `color_camera_info_file` 覆盖 |
| `depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 同上 | 深度内参（配准后彩色系） |
| `depth_registered/points` | `sensor_msgs/msg/PointCloud2` | 同上 | 配准点云，字段 `xyz` / `rgb` / `confidence`，`point_step=24`；**无订阅者时不构建、不发** |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | — | `stereo_health` 1 s：`rate_gps`、`publish_fps`、`last_publish_age_s`、drop 计数、laser 配置 |

订阅 `/joint_states`（`sensor_msgs/msg/JointState`，SensorDataQoS / KeepLast 100，Best Effort / Volatile）；默认绝对名可经 ROS remap 改接线。无 service、无 action。点云只有在有订阅者时才生成，这是性能保护；color/depth 的 header stamp 取各自 component timestamp（不是单一 group stamp），color 在 SGBM 前发布、depth 在 SGBM 后发布；
`depth` 的 0.25 mm 必须在消费端边界换算成米（驱动不做米制转换）。

## TF

静态 TF：`camera_link` → `camera_color_frame` → `camera_color_optical_frame`；
`camera_link` → `camera_depth_frame` → `camera_depth_optical_frame`。
`camera_name` 参数改变前缀；默认 `camera`。

## 参数（`config/stereo_camera.yaml`）

| 参数 | 默认 | 作用 |
|---|---|---|
| `device_ip` | `169.254.10.110` | PS800-E1 IP |
| `camera_name` | `camera` | 话题/TF 前缀 |
| `qos_depth` | `1` | image/info/pointcloud 发布队列深度；1=只保最新 |
| `qos_reliability` | `reliable` | `reliable`/`best_effort`；本地 raw 大图默认 reliable |
| `color_mode` | `640x480` | 彩色档位；默认 2560x1920 会拖死组率 |
| `color_camera_info_file` | launch 注入 `percipio_camera/config/color_camera_info.yaml` | 覆盖 `camera_info` 的棋盘格内参（对着官方去畸变彩图）；像素去畸变仍用设备厂参，与官方节点一致 |
| `ir_exposure` | 990 | 左右 IR 曝光（深度质量）。百兆会把组率打到 ~0.7 gps，不要为此压到 100 |
| `laser_power` | 100 | 投射器功率；停栈复位 |
| `sgbm.num_disparities` | 128 | 半分辨率视差范围；匹配前左缘补同宽再裁回，避免 OpenCV 左盲区切掉配准后左袋 |
| `sgbm.block_size` | 5 | SGBM 块大小 |
| `sgbm.uniqueness_ratio` | 6 | 匹配唯一性比；提到 10 抠不掉玻璃门且袋面打洞，官方轮廓清晰来自 18 图案 |
| `sgbm.processing_scale` | 0.5 | 匹配分辨率相对 1280x960 的缩放 |
| `sgbm.mode` | hh4 | SGBM 路径聚合模式 |
| `avg_k` | 1 | k 帧时域融合（质量换帧率） |
| `median_ksize` | 3 | 深度有效值中值窗 |
| `temporal_k` | 3 | 配准后彩色网格深度滑窗中值；仅完整曝光窗有静止关节证据时融合；1=关 |

launch 参数：`device_ip`、`color_camera_info_file`；`respawn=True`、
`respawn_delay=3.0`。相机连接独占，不能与 `percipio_camera` 同时开。

## 诊断与失败

- 设备未打开 / 激光或曝光配置失败：ERROR；
- 未发布首帧：WARN；`publish_fps < 10`：WARN；
- 正常：`streaming <fps> fps`；多帧未启用或刚清窗时 WARN `streaming single-frame; temporal motion/unknown/warmup`，原图和单帧深度继续发布；
- 计数：`groups` / `published` / `drop_stream_status` / `drop_incomplete` /
  `drop_no_stamp` / `drop_invalid_stamp` / `drop_depth` / `laser_config_ok` /
  `time_sync_ready`。
- `rate_gps` 与 `publish_fps` 是本节点发布侧观测，不证明订阅端或感知达到 10 FPS。
- `color_age_ms` / `depth_age_ms` / `depth_compute_ms`：发布时 header age 与 SGBM+配准耗时；用于区分相机侧延迟和下游队列；age 保留符号。
- `temporal_fusion_active`：本帧窗至少 2 帧；`temporal_window_frames`：当前静止窗样本数；`temporal_bypass_frames`：运动/未知而退回单帧的累计帧数。K3 首帧的有效点 confidence 沿原公式为 1/3；窗外单帧 confidence=1 仅表示当前深度有效，不是静止许可，须结合融合诊断和消费端静止门。
- HOST 同步写失败拒启，不等待 `TIME_SYNC_READY`。连续触发 GET-modify-SET 为 `TRIGGER_MODE_OFF`；读回失败或 `STREAM_ASYNC_OFF` 未粘住则拒启。触发保留 EX 其余字段。彩色 AE 每次打开都重新打开（清除上次误钉的 INT=100），与 `ir_exposure=990` 同为产品曝光；组率掉到 ~0.7 gps 时先查网口是否千兆。stamp 缺失不借另一流。纪元年龄不在 0–10 s、左右 IR 相差超过 1 ms、重复/回跳计 `drop_invalid_stamp` 并清图像窗。HOST 同步后若固件写入 Unix 纳秒（≥1e17）按纳秒解释，否则按 SDK 微秒。原始曝光 stamp 保持不变。取帧后先拷出再归还设备缓冲。
- 六关节证据及整个 `avg_k`/K3 曝光窗检查规则见 [architecture.md](architecture.md#多帧滤波的曝光时域)。缺关节时相机单帧可用，多帧融合关闭。
- 采集端核对 SDK 的帧尺寸和缓冲区长度后才向 OpenCV 传递指针。彩色 YUYV 至少每像素 2 字节，左右 IR 至少每像素 1 字节。尺寸与配置或标定不符、缓冲区过短的分量计入 `drop_stream_status`，缺少该分量的整组计入 `drop_incomplete`。

## 不负责

不选果、不重建、不规划；不做米制深度转换；不替代 `percipio_camera` 的设备端
深度。点云/彩色原图的质量与长期稳定性见 [architecture.md](architecture.md)。
