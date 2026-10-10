# percipio_camera（原驱动）vs peach_stereo（本项目）函数级·流程级对比分析

**日期** 2026-09-20 · **对象** `src/percipio_camera`（厂商驱动包，~7.7k 行）vs `src/peach_stereo`（本项目，652 行单文件节点）
**实测数据** 见 `../report/report.md`（同场景同感知链 A/B，~120 帧/档）；离线仪器数据见 `reports/2026-09-20-e2e-stereo-optimal/`

---

## 0. 一句话总结

两包是同一台 PS800-E1 的两条深度路径：**percipio_camera 把立体匹配交给相机固件**（18 幅散斑图案在设备端融合，主机只做解码/配准/发布），**peach_stereo 把双 IR 原图搬到主机自己算**（单图案 SGBM）。深度来源不同导致帧率 2.43 vs 13.7fps、单帧时域稳定 1.5 vs 9.0mm 的镜像差异；配准、话题、TF、深度口径两边同构（都走 libtyimgproc 的 `TYMapDepthImageToColorCoordinate`，感知零改动可切换）。实测（report.md §2）：健康态两链感知输出一致性 <0.8mm；门限口径见 peach_project_impact.md §4——真实门是重建精配准体素 3mm 与 pregrasp 偏置 30mm（早期"3mm 门"为讹传，已修正）。

---

## 1. 架构与包拓扑

| 维度 | percipio_camera | peach_stereo |
|---|---|---|
| 进程形态 | **Composable 容器**（`percipio_camera.launch.py` 起 `component_container` + `ComposableNode`，`RCLCPP_COMPONENTS_REGISTER_NODE`） | 单进程普通节点（`ros2 run/launch` 直起） |
| 代码体量 | ~7.7k 行/13 文件（含 vendored SDK 传输层 gige_2_0/2_1、huffman、XML 解析、TCP 日志服务器） | 652 行单文件 |
| 分层 | Driver(入口)→Node(发布)→Device(SDK+处理线程) 三层 | 一个类全包 |
| 使能组件 | depth+color(+IR L/R 可选)；深度可关闭仅 RGB（手眼标定路径） | 固定 RGB+IR L/R（不用设备深度组件） |
| 发布通道 | `image_transport`（raw+compressed 全插件，16UC1 交叉编码问题） | 只发 raw Image（规避 Jazzy 插件问题） |
| 帧率控制 | `frame_rate_control` 软触发线程（默认 2.5fps） | 无（设备连发 ~13.7 组/s） |
| 生命周期 | 非 Lifecycle；有离线重连线程/事件 | 非 Lifecycle；构造即采集 |

---

## 2. percipio_camera 逐文件职责

| 文件 | 行数 | 职责 |
|---|---|---|
| `percipio_camera_node_driver.cpp` | 169 | composable 入口：SDK init、设备枚举选择、workmode、相机事件回调、TCP 日志服务器 |
| `percipio_camera_node.cpp` | 751 | ROS 发布层：参数、流开关、QoS、四路图像+两种点云发布、静态 TF、软触发/动态配置/复位订阅 |
| `percipio_device.cpp` | 1350 | SDK 封装+采集/处理线程：开闭、标定读取、流模式、深度滤波、配准、点云映射、软触发、离线重连 |
| `percipio_depth_algorithm.cpp` | — | 时域滤波管理器（`depth_time_domain_num` 帧） |
| `percipio_xml.cpp` / `percipio_video_mode.cpp` | 2264/604 | 设备 XML 参数解析、流模式枚举/匹配 |
| `gige_2_0.cpp` / `gige_2_1.cpp` | 494/623 | GigE 传输协议两代实现（vendored 传输层） |
| `huffman.cpp` | 464 | 图像解码 |
| `network_ip_config.cpp` / `list_devices.cpp` / `percipio_log_server.cpp` | 385/257/240 | 网络配置、设备枚举、TCP 日志服务器 |

---

## 3. percipio_camera 关键函数逐个分析

### 3.1 入口层 `PercipioCameraNodeDriver`

- **`init()`**：`TYInitLib` → `TYImageProcesAcceEnable(false)`（关硬加速）→ 声明设备定位参数（serial_number/device_ip/device_workmode/camera_parameter XML/日志参数）→ `tycam_log_server_init` → `startDevice()`。**无参数校验拒绝启动**——非法 `depth_resolution` 直到设备打开后才报 `Unsupported stream mode`（本轮实测踩中：默认 640x400 该机型不支持）。
- **`startDevice()`**：死循环 `selectDevice(TY_INTERFACE_ALL,…)` 找不到设备就一直 ERROR 重试（阻塞构造线程）。
- **`initializeDevice()`**：建 `PercipioDevice` → 读 SN/型号/固件/配置版本 → 设 workmode（continuous/trigger_soft/trigger_hard）→ 注册相机事件回调 → 可选 `setDeviceConfig(xml)`（设备端参数整包下发）→ 建 `PercipioCameraNode`。
- **`onCameraEventCallback`**：offline/connect/timeout 三事件 → `/camera/device_event`（transient_local，运维可 latch 订阅）——本项目没有的观测通道。
- **`~PercipioCameraNodeDriver`**：`TYDeinitLib`。**注意**：容器 SIGINT→SIGKILL 超时路径上 SDK 设备句柄不保证干净关闭（本轮降级态事故的帮凶之一）。

### 3.2 发布层 `PercipioCameraNode`

- **`getParameters()`**：四流 ×（enable/resolution/format/qos/camera_info_qos）+ 全局（auto_reconnect、frame_rate_control、frame_rate、**laser_power 默认 -1**、depth_registration_enable、speckle/time-domain 滤波、点云开关/QoS、ir_undistortion、ir_enhancement）。耦合规则：`color_point_cloud_enable→registration=true`；无 color→两者关。
- **`setupDevices()`**：滤波初始化下发给 device → 能力检查（hasColor/hasDepth/…不支持则关流）→ `frame_rate_init` → **`laser_power` 仅 ≥0 时 `TYSetInt(POWER)`，从不碰 `TY_BOOL_LASER_AUTO_CTRL`**（本轮发现的关键缺陷①：激光器模式完全继承前任客户端）→ 逐流 `stream_open`（深度即使 disable，若要点云也隐式打开）→ `getDepthValueScale` → `startStreams`。
- **`startStreams()`**：`setFrameCallback(onNewFrame)` + `stream_start`——回调在**设备采集线程**里直接做解码/配准/发布（无独立 worker、无界队列概念；重处理会拖慢整帧循环）。
- **`setupPublishers()`**：`depth_registered/points`（彩色云）与 `depth/points`（原始云）互斥；每流 `image_transport` 发布器（Jazzy 下全插件加载，compressed 编不了 mono16 → harvest catch-all 订阅时每帧 ERROR，本项目 README 已记载）。
- **`publishColorFrame/publishDepthFrame/publishLeftIRFrame/publishRightIRFrame`**：订户计数门控；`camera_info` 取设备标定（彩色支持文件覆盖，与本项目共用同一 yaml）；时间戳 `HWTimeUsToROSTime(硬件微秒)`——与本项目 stamp 语义一致（设备纪元µs）；深度 16UC1 原样、ABC16 支持两格式。
- **`publishColorPointCloud`**：设备 SDK 已产出的 float XYZ（mm）→ NaN 门控 → `/1000` 转米 + 缩放后的彩色 → xyz+rgb 紧凑点云。**`publishPointCloud`**：`PUBLISH_INVALID_POINT_CLOUD_DATA` 宏下无效点填 0 并保持 width×height 布局（非 dense 语义却标 `is_dense=true`，下游须自滤零点——与本项目"仅有效点、is_dense=true"不同）。
- **`publishStaticTransforms`**：与本项目同构 link→frame→optical（RPY(-π/2,0,-π/2)），但**按启用流惰性发布**（首帧时）。
- **`onNewFrame`**：首帧发 TF → 逐流发布 → 云。**单回调串行**：深度滤波+配准+双云投影都在采集线程。
- 三个订阅：`soft_trigger`（手动触发）、`dynamic_config`（XML 热改）、`reset`（Empty → `device->reset()`，设备级复位通道，运维可用）。

### 3.3 设备层 `PercipioDevice`

- **`device_open`**：接口/设备打开 → 读四份标定（depth/color/IR L/R）→ 解析能力 XML。**标定只读一次**；IR/深度内外参无文件覆盖入口。
- **`GigEBase::video_mode_init`**：枚举各流模式（本轮日志所见 depth 640x480/1280x960/320x240 等）。
- **`stream_open(idx, resolution, format)`**：`resolveStreamResolution` 字符串→宽高 → `TYSetEnum(IMAGE_MODE)`；**不支持的模式在此时才失败**（缺陷②：默认值 640x400 对本机型非法，错误发生在设备已打开之后）。
- **`frame_rate_init(en,fps)`**：en→软触发模式；`stream_start` 里起 **`softTriggerSend` 线程**（1000/fps 周期发软触发，最小间隔 60ms 兜底）——2.5fps 的来源；禁用则 continuous 自由跑。
- **`set_laser_power(power)`**：仅 `TYSetInt(LASER, POWER)`。**没有 AUTO_CTRL 开关**（缺陷①的根）；peach_stereo 则显式 `auto=false+power=100`。
- **`getDepthValueScale`**：`TY_FLOAT_DEPTH_SCALE_UNIT`（本机 0.25mm）——深度口径与本项目硬编码 0.25 一致。
- **`frameDataReceive`（采集线程主循环）**：`TYFetchFrame`（continuous 2000ms / trigger 200ms 超时）→ 逐分量：
  - **DEPTH**：`0xFFFF→0` 清洗 → 可选 `TYDepthSpeckleFilter`（设备侧散斑滤波）→ 可选时域滤波（N 帧管理，失败**丢整帧**）→ `depthStreamReceive` + `p3dStreamReceive`；
  - **RGB**：`TYDecodeImage`（yuyv→BGR 等 huffman 解码）；
  - **IR L/R**：解码 → `IREnhancement`（线性/直方图增强，默认 off）→ `IRUndistortion`（默认 on，`TYUndistortImage`）。
  末尾统一 `_callback(VideoStream)` → 归还缓冲。
- **`depthStreamReceive`（配准核心）**：C16 深度 → 可选 `TYUndistortImage(cam_depth_calib)`（默认仅当未开配准时做）→ **开配准则 `TYMapDepthImageToColorCoordinate(cam_depth_calib→cam_color_calib)`** 并换彩色内参 → ABC16 路径走 3D 点外参逆变换再投彩色网格。**与本项目是同一个 libtyimgproc 调用、同一对标定**——两链配准几何同源（实测 +8px 系统差来自本项目"校正网格直喂 calibL"的网格语义差，reports/2026-09-20-camera-image-analysis 已裁定）。
- **`p3dStreamReceive`**：`TYMapDepthImageToPoint3d`（彩色或深度标定）产 float XYZ 云（发布层用）。
- **`device_offline_reconnect` 线程**：事件唤醒 → `Release+Reconnect` 循环 → 成功后 `_node->setupDevices()` 重启流（本项目无此机制，断了就没了）。
- **`reset()`**：设备复位特性（订阅 `reset` 触发）。

---

## 4. peach_stereo 对应函数（对照表）

| 流程职责 | percipio_camera | peach_stereo | 差异要点 |
|---|---|---|---|
| 设备发现/打开 | `startDevice`/`device_open`（selectDevice 死循环） | `openDevice()`（按 IP 枚举，一次失败即 FATAL 拒启） | 本项目 fail-fast；驱动 fail-retry |
| 时间同步 | 设备默认时戳 | `TY_ENUM_TIME_SYNC_TYPE=HOST`（帧组同微秒） | 本项目显式锁定，RGB-D 同步天然成立 |
| 流模式 | `stream_open`（字符串解析，非法才报错） | `setupColorMode()`（枚举匹配 yuyv 640x480，失配 FATAL） | 本项目防 fallback 档位错位 |
| 激光器 | `set_laser_power`（仅功率，默认不碰） | 显式 `auto=false + power=100`，停栈复位 | **关键差**：驱动继承状态（缺陷①） |
| 深度来源 | 设备固件 18 图案（`frameDataReceive` 直取） | `computeDepth()`：remap→半分辨率→**SGBM(HH4)**→z=fB/d→med3→avg_k | 主机算 vs 设备算 |
| 深度后处理 | `TYDepthSpeckleFilter`+时域滤波（设备侧，可选） | med3 空间中值（默认开）+avg_k（默认 1） | 各自实现，语义都是"只去噪不补洞" |
| 配准 | `depthStreamReceive`→`TYMapDepthImageToColorCoordinate` | `publishGroup`→同一 SDK 调用（z 校正网格直喂 calibL） | 同库同调用；网格语义差 +8px（已裁定勿改） |
| 点云 | `publishColorPointCloud`（设备 SDK float XYZ）/`publishPointCloud`（含 0 点占位） | `publishRegisteredCloud`（订户门控，针孔反投影，仅有效点） | 无效点表达不同 |
| camera_info | 设备标定+彩色文件覆盖 | 同（共用同一 yaml） | 一致 |
| TF | 按启用流惰性发布 link→frame→optical | 启动即发全链 | 帧名同构 |
| 帧率 | 软触发线程 2.5fps | 无控制（13.7gps 相机节拍限速） | 5.6× |
| 断线恢复 | offline 重连线程+device_event | 无 | 驱动胜 |
| 停止路径 | 容器优雅停可能超时→SIGKILL→SDK 不净 | 析构礼貌复位激光+关设备 | 本项目停得干净（但复位动作本身是切换毒源之一，见 §6） |

---

## 5. 数据流逐步对照（一帧的生命周期）

| # | percipio_camera（设备深度路径） | peach_stereo（主机 SGBM 路径） |
|---|---|---|
| 1 | 设备端逐幅投影 **18 幅散斑图案**，固件内立体匹配+融合，产出深度帧（~411ms/帧） | 设备连发**单图案** IR L/R + color 同帧组（73ms/组，TIME_SYNC=HOST 同微秒） |
| 2 | `TYFetchFrame` 取深度分量（软触发节拍 2.5fps） | `TYFetchFrame` 取 RGB+IR L/R 三分量 |
| 3 | `0xFFFF→0` 清洗；可选设备散斑滤波、时域 N 帧滤波 | remap 校正（1280×960）→ resize 半分辨率 |
| 4 | （可选）`TYUndistortImage` 深度去畸变 | `cv::StereoSGBM(HH4)` 视差 → z=f_proc·B/d |
| 5 | `TYMapDepthImageToColorCoordinate`（depth calib→color calib） | med3 有效值中值 →（avg_k）→ 0.25mm 量化 |
| 6 | `VideoStreamPtr->DepthInit`（换彩色内参） | `TYMapDepthImageToColorCoordinate`（calibL→calibC，校正网格直喂——+8px 系统差来源） |
| 7 | 发布 `depth/image_raw`(16UC1×0.25) + `depth/camera_info` | 同左（话题/口径同构） |
| 8 | （可选）`TYMapDepthImageToPoint3d` → `depth_registered/points`（设备 SDK 3D 点+彩色） | `publishRegisteredCloud`：针孔反投影（用彩色 K）→ xyz+rgb |
| 9 | color yuyv→`TYDecodeImage`→bgr 发布 | `cvtColor(YUY2→BGR)` 发布 |
| 10 | 全部在设备采集线程串行 | 全部在采集线程串行（云发布有订户门控） |

**逐帧成本**：驱动路径 ≈ 411ms（设备图案序列主导）；本项目 ≈ 73ms（其中 SGBM(HH4) 54ms）。**这就是 2.43 vs 13.7fps 的全部来源**。

---

## 6. 实测结论对照（详见 ../report/report.md）

| 指标 | percipio（健康档） | peach_stereo hh4 | 裁定 |
|---|---|---|---|
| 相机源帧率 | 2.43 fps | **13.5–13.7 gps** | 本项目 5.6×（决定停走节拍整栈速度：历史锁定 2.8s vs 48s） |
| 单帧逐像素时域稳定 | **1.5 mm** | 9.0 mm（3WAY 10.25） | 驱动 6×（18 图案融合），被下游统计拟合吸收（两链 entry std 都 <0.6mm） |
| 感知输出 entry std (z) | **0.250 mm** | 0.486 mm | 驱动略优；真实门限=重建体素 3mm/pregrasp 偏置 30mm（"3mm 门"为讹传，见影响评估 §4） |
| 两链 entry 均值差 | — | — | **<0.8mm**（配准同源互证） |
| 覆盖/检出/mdr | 49.3% / 2检1确 / 0.999 | 49.5% / 2检1确 / 0.999 | 一致 |
| 点云表面粗糙度（场景级） | 1.08–1.14 mm | **0.94–1.00 mm** | 本项目更平滑（半分辨率 resize 降噪） |
| 切换健壮性 | 修复后健康（XML 清空，valid 0.477/mdr 0.986）；此前崩塌系 parameters.xml 残留值，已修复 | 同左免疫面（不用设备深度管线） | 修复后两档均稳定 |

### 降级态悬案复盘（根因已定：parameters.xml 调参残留值）

2026-09-20 下午起 percipio 深度反复崩塌（valid 5–12%），先后排除：配准环节、laser_power、软触发/自由跑、IR 组件、闲置冷却、SIGKILL/优雅停组合、激光预置、SDK 固件复位、乃至整机断电。**最终判别实验（install 副本改名→官方 launch 无 XML 下发）一击恢复健康（0.474/0.977）**——真凶为 `parameters.xml` 的 `DepthSgbmImageNumber=2` 调参残留：launch 无条件读取并下发该文件（percipio_camera.launch.py:13/99），把设备 18 图案 SGBM 砍成 2 幅，深度大面积无效。**修复**：源码清空该值（=设备默认 18）+ rebuild，官方配置复测健康（0.477/0.986）。**因果终裁（09-21 ros2 bag A/B，/home/mu/Pictures/video/）**：同日同会话只翻转 XML 值——值 2：valid 0.096（78 帧/20s，帧率反常高=2 幅图案佐证）；空值：valid 0.473（39 帧，稳定）——重启假说排除，因果坐实。前期"切换毒害/激光状态/热衰减"假设均系排查路径（证伪矩阵完整保留于 testing-log）。**教训：该 XML 是无条件下发通道，任何实验值残留即生产事故。**

## 7. 驱动缺陷清单（本轮实测暴露，上游/本仓改进建议）

1. **`parameters.xml` 无条件下发且覆盖用户参数**（launch:99 在用户参数之后写入）：调参残留值直接改变设备行为且极难排查（本事故耗时一下午+一次整机断电）。建议：默认不下发或改为可选；文件内所有非空值需要显式白名单。
2. **`depth_resolution` 默认 640x400 本机型不支持**：错误发生在设备已打开后（半初始化状态）。建议默认改 640x480 或启动前校验。
3. **`laser_power` 不带 `LASER_AUTO_CTRL` 开关**（node:149/232，默认 -1 不碰）：切换场景下激光器状态继承不可控（非本事故根因，但为真实缺陷）。
4. **复位/降级态崩溃**：SDK 复位后实例以 `Percipio::NullPointerException` SIGABRT 崩溃（实测两次）；无深度健康自检。
5. `publishPointCloud` 无效点填 0 且 `is_dense=true`（语义陷阱）；处理全在采集线程。
6. 运维项：反复强杀容器会积累 FastDDS SHM 死锁文件（`/dev/shm/fastrtps_*`，166 个），导致新订户无法连接——定期清理或避免 SIGKILL。

## 8. 选型结论

- **生产采果（停走节拍）**：peach_stereo——帧源 5.7× 决定整栈速度；实测（09-21 双健康档）拟合抓取点稳定性更优（entry z std 0.58 vs 3.31mm）；感知输出一致性两链等价。
- **单帧即稳/低算力场景**：percipio 原驱动（已修复）——ROI 级深度统计更稳（中位 std=0）；2.43fps 对非停走场景够用。
- **切换纪律**：percipio 前端必须带 `depth_resolution:=640x480`；`parameters.xml` 保持全空值（官方默认）；切换前端采果前重做手眼标定吸收 +8px 系统差。
