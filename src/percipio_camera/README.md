# percipio_camera — Percipio（图漾）工业相机 ROS 2 驱动

## 简介

图漾 Percipio 工业相机的厂商 ROS 2 驱动（基于 camport4/TY SDK，GigE 取流），
以 component 形式运行，向本工作区提供 eye-in-hand 相机的彩色图、深度图、
CameraInfo、红外图与（彩色）点云。下游消费者：手眼标定
（aubo_hand_eye_calibration，用 `color/image_raw` + `color/camera_info`）、
点云场景重建（aubo_scene_recon，用 `depth_registered/points`）、桃子位姿估计
（peach_pose_ros2，用 RGB-D 三路话题）。bringup 经 `camera_enabled`
（默认 true）include 本包 `percipio_camera.launch.py`；外参
`wrist3_Link → camera_link` 由 extrinsics_publisher 发布，`camera_link`
正是本驱动发布的相机基座 frame。

本包为**厂商代码，不改动源码、不参与 lint**；本项目仅做两处项目化改动：

1. **launch 默认值**（`launch/percipio_camera.launch.py`）：`device_ip` 默认
   `169.254.10.110`（厂商为空串，本工作单元相机地址）；`depth_enable=true`、
   `point_cloud_enable=false`、`color_point_cloud_enable=true`（默认出彩图 +
   已配准彩色点云，不出灰度点云）。
2. **新增 `color_camera_info_file` 参数**：非空时用 yaml 内参覆盖设备 Flash
   标称内参，默认指向包内 `config/color_camera_info.yaml`（棋盘自标内参，
   640x480 / plumb_bob）；传 `''` 回退设备内参。实现见"执行逻辑"。

## 使用方法

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select percipio_camera
source install/setup.bash

# 单独启动相机（默认 IP 169.254.10.110，话题在 /camera 命名空间下）
ros2 launch percipio_camera percipio_camera.launch.py

# RGB-D 封装：在主 launch 上强制打开深度、配准与点云（scene_recon / peach_pose 用）
ros2 launch percipio_camera percipio_rgbd.launch.py

# 随整机 bringup 启动（camera_enabled 默认 true，无需显式操作）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real
```

常用检查与工具：

```bash
ros2 run percipio_camera list_devices                    # 枚举在线设备（SN/IP）
ros2 run percipio_camera network_ip_config               # 配置设备 IP
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once         # 应看到 yaml 覆盖后的内参
ros2 launch percipio_camera percipio_camera.launch.py --show-args   # 全部参数说明
```

安全约定：相机驱动本身不驱动机械臂；与 bringup/MoveIt 联调涉及运动时，
速度/加速度缩放先压 0.1，确认后再放宽（见根 AGENTS.md 第 10 节）。

关键 launch 参数（权威源 `launch/percipio_camera.launch.py`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `camera_name` | camera | 节点名、话题命名空间与 frame 前缀 |
| `device_ip` | 169.254.10.110 | 按 IP 选设备；或用 `serial_number` 按 SN 选 |
| `color_enable` / `color_resolution` | true / 640x480 | 彩色流（BGR8 发布） |
| `color_camera_info_file` | share 内 `config/color_camera_info.yaml` | 内参覆盖文件，`''` 回退设备 Flash 内参 |
| `depth_enable` / `depth_resolution` | true / 640x400 | 深度流（16UC1，单位 mm） |
| `depth_registration_enable` | true | 深度配准到彩色坐标系 |
| `point_cloud_enable` | false | 灰度点云 `depth/points` |
| `color_point_cloud_enable` | true | 彩色点云 `depth_registered/points` |
| `device_workmode` | trigger_off | trigger_off / trigger_soft / trigger_hard |
| `frame_rate_control` / `frame_rate` | false / 5.0 | 限帧（软触发周期）；开启后 workmode 失效 |
| `device_auto_reconnect` | true | 掉线自动重连 |
| `depth_speckle_filter` / `depth_time_domain_filter` | false / false | 散斑 / 时域滤波 |
| `ir_undistortion` / `left_ir_enable` | true / false | IR 去畸变 / 左 IR 流 |
| `*_qos`（color/depth/ir/point_cloud 等） | default | 可选 sensor_data 等 rmw 档案名 |

## 执行逻辑

容器与节点：`percipio_camera.launch.py` 把 ComposableNode
`percipio_camera::PercipioCameraNodeDriver` 装进 `component_container`
（容器名 `camera_container`），并 `PushRosNamespace(camera_name)`——
节点最终为 `/camera/camera`，全部相对话题落在 `/camera/` 下。

启动序列（`src/percipio_camera_node_driver.cpp`）：

1. `PercipioCameraNodeDriver::init()`（:33）：`TYInitLib` 初始化 SDK，声明
   `serial_number`/`device_ip`/`device_workmode`/`camera_parameter`/日志参数；
2. `startDevice()`（:62）**阻塞重试** `selectDevice` 直到找到设备（找不到会
   循环刷 "Not found any device!"，检查网线/IP/是否被旧进程独占——
   `Open device fail -1014` 即独占表现）；
3. `initializeDevice()`（:93）：建 `PercipioDevice`，按 `device_workmode` 设
   连续/软触发/硬触发，应用 `camera_parameter`（launch/parameters.xml 内容），
   注册事件回调，构造 `PercipioCameraNode`。

发布面（`src/percipio_camera_node.cpp`）：`setupTopics()`（:347）依次
getParameters → setupDevices → setupPublishers → setupSubscribers。
`PercipioDevice` 内部收帧线程把帧组好后回调 `onNewFrame()`（:720），按流开关
分发到 depth/color/IR/点云发布；每路仅在**有订阅者**时才真正组包发布
（`SUBSCRIVER_CHECK`）。首帧时发一次静态 TF：`camera_link` →
`camera_<stream>_frame` → `camera_<stream>_optical_frame`（:698）。图像时间戳
为相机硬件 μs 直接换算的 ROS 时间（`HWTimeUsToROSTime`，:27）——常超前机器人
TF，下游按 stamp 查 TF 需容错（scene_recon 已处理）。

`color_camera_info_file` 机制：getParameters 用
`camera_calibration_parsers::readCalibration` 解析 yaml（:96-114），成功置
`color_info_file_override_`；`publishColorFrame()`（:390-397）保留设备上报的
header/width/height，仅替换 `distortion_model/d/k/r/p`。解析失败报错并回退设备
内参。

开关联动（getParameters :191-198、setupDevices :235-258）：
`color_point_cloud_enable=true` 强制 `depth_registration_enable=true` 且
`point_cloud_enable=false`；彩色流关闭则配准与彩色点云一同关闭；
`depth_enable=false` 但只要任一云开关打开，深度流仍会在设备层打开
（只是不发布 `depth/image_raw`）。

事件与控制面：SDK 掉线/重连/超时事件 → `device_event` 话题（transient_local）；
`device_auto_reconnect=true` 时设备层重连线程自动恢复取流。三个订阅话题提供
运行时控制：`soft_trigger`（软触发，需 `device_workmode:=trigger_soft`）、
`dynamic_config`（JSON 字符串改设备参数）、`reset`（设备复位）。

## 软件框架

```text
camport4/                    # TY SDK（find_package(TYCam) + 预编译库，按架构选 lib）
include/ src/                # 驱动源码（类清单见下）
launch/percipio_camera.launch.py   # 主入口（单机，默认参数见上表）
launch/percipio_rgbd.launch.py     # RGB-D 封装：深度/配准/点云开关强制全开
                                   #   （经上述联动，实际只出 depth_registered/points）
launch/cam_base_cfg.launch.py      # 基础配置入口（multi_cam 用，默认 1280x960）
launch/multi_cam.launch.py         # 双相机示例（按 SN/IP 各起一份）
launch/parameters.xml              # 经 camera_parameter 透传给设备的特性配置
launch/*.py                        # 厂商示例脚本（send_trigger/offline_detect 等）
config/color_camera_info.yaml      # 棋盘自标彩色内参（项目化改动②的默认文件）
scripts/depth_accuracy_eval.py     # 深度精度评估（YOLO26s-depth，自带 venv 约定）
```

关键类（`percipio_camera` 命名空间）：

- `PercipioCameraNodeDriver`（percipio_camera_node_driver.cpp）——component 入口：
  SDK init、设备选择、事件回调；插件名 `percipio_camera::PercipioCameraNodeDriver`，
  宿主可执行 `percipio_camera_node`。
- `PercipioCameraNode`（percipio_camera_node.cpp）——参数解析、流开关、
  发布/订阅、静态 TF、`color_camera_info_file` 覆盖。
- `PercipioDevice` + `GigEBase`（gige_2_0/2_1）——设备句柄与 GigE 协议层，
  收帧/限帧/重连线程。
- 独立工具可执行：`list_devices`、`network_ip_config`（装到 `lib/percipio_camera`）。

对外接口（均在 `/<camera_name>/` 命名空间下，默认 `/camera/`）：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| pub | `color/image_raw` | sensor_msgs/Image | 彩色 BGR8 |
| pub | `color/camera_info` | sensor_msgs/CameraInfo | 内参（可被 yaml 覆盖） |
| pub | `depth/image_raw` + `depth/camera_info` | Image / CameraInfo | 16UC1 mm（或 16SC3 xyz） |
| pub | `depth_registered/points` | PointCloud2 | 彩色点云 xyz+rgb，frame=`camera_depth_optical_frame` |
| pub | `depth/points` | PointCloud2 | 灰度点云（`point_cloud_enable`） |
| pub | `left_ir/image_raw` 等 | Image / CameraInfo | 左/右 IR（默认关） |
| pub | `device_event` | std_msgs/String | 掉线/重连/超时，transient_local |
| sub | `soft_trigger` | std_msgs/String | 软触发（trigger_soft 模式） |
| sub | `dynamic_config` | std_msgs/String | JSON 设备配置 |
| sub | `reset` | std_msgs/Empty | 设备复位 |
| TF | `camera_link`→`camera_*_frame`→`camera_*_optical_frame` | 静态 TF | 首帧发布一次；`camera_link` 由外参接到 `wrist3_Link` |

注意：peach_pose_ros2 的 `inspector/config.py` 持有彩色内参的同值拷贝并注明
"权威源为本包 `config/color_camera_info.yaml`"——改标定文件后需同步该处。
