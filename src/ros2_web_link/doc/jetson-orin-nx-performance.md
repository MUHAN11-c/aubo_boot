# 压力测试与 Jetson Orin NX 16GB 部署参数

当前版本为 **ros2_web_link 0.8.3**，2026-11-03重新构建并安装到主工作区。本轮面向用户指定的 **Orin NX 16GB，ROS 2与浏览器同机运行**；目标机未连接，以下均为x86桌面实测，不能换算为NX性能或作为NX生产放行。

本轮修复后主安装完成两种缓存模式各17项功能检查，以及下列8档独立压力验收。浏览器全程headless，不显示平台窗口、不争抢桌面焦点。原有有界面材质检查、被手动关闭及中断的轮次保留原判定，不并入本轮通过数。

## 修复与适用范围

- Service预算与缓存开关独立：默认cache0，所有WebSocket连接在创建工作线程之前共享128项准入；满额原生result=false，不排队或重试。创建等待默认5秒；超时但未消费的owner任务继续占名额，迟到任务跳过创建。工厂可能注册后抛错、销毁未确认时保留隔离名额，正常重建Node才恢复。非零缓存仍保留原生兼容门，有效槽位为min(cache_size,max_in_flight)。预算不涵盖Action、所有原生分配或整机内存。
- 工作台保留URDF/关节/Marker/点云/Path/图像/曲线模板的QoS、compression与throttleMs，显示仍默认关闭。此前BEST_EFFORT选项被转换成RELIABLE的问题已用实际公有转换入口复现并修复。
- DDS可靠恢复：自有rosapi/demo的标准service writer和观测播放器默认writer采用100ms心跳，仅限这些进程。保持录制QoS；已有显式环境XML或cwd的DEFAULT_FASTRTPS_PROFILES.xml优先，不切RMW、不设置RMW_FASTRTPS_USE_QOS_FROM_XML，外部业务节点不受本包默认配置影响。100ms是恢复间隔，不是总延迟上限。
- URDF显式visual材质按RViz2优先规则覆盖嵌入材质；没有声明时保留资产材质。每轮浏览器实际核对7个DAE机械臂link的opacity=1且transparent=false，并检查暂停/重启/断线释放资源。

依据：[Jazzy rclpy源码](https://github.com/ros2/rclpy/tree/jazzy/rclpy/rclpy)、[rosbridge 2.7.1 Service](https://github.com/RobotWebTools/rosbridge_suite/blob/2.7.1/rosbridge_library/src/rosbridge_library/capabilities/call_service.py)、[rmw_fastrtps 8.4.4 profile规则](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/README.md#creating-services-with-different-profiles)、[Fast DDS可靠writer设置](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/dds_layer/core/policy/eprosimaExtensions.html#rtpsreliablewriterqos)、[RViz2材质规则](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/robot/robot_link.cpp#L660)。实现核对本机Jazzy实际apt源码；未修改系统包。

## 0.8.3环境与取证

Intel Core Ultra 7 265K（20逻辑CPU）、约31.46GiB可见RAM、RTX3090，Ubuntu24.04/Jazzy；rosbridge/rosapi2.7.1、rclpy7.1.12、FastDDS2.14.7 / rmw_fastrtps8.4.4、Node22.22.3。浏览器实际版本、NVIDIA渲染器与1440×1000/DPR1视口见各轮device.json。只测回环地址，未测远程局域网或Android。

录制59.471871442秒，两个MCAP合计1,207,874,729字节；116帧完整点云（最大约4.15MB）、453帧深度、78帧相机原始彩色、612帧Peach彩色结果。传输档用相机原始彩色，浏览器用Peach结果图，因此两档CPU/接收率不能按相同彩色负载比较。录制为640×480，深度单位0.00025m；录制源有丢帧，不是无损标定数据。

回放1倍速、/clock30Hz、read-ahead100；云100ms、RGB/深度500ms。浏览器11路显示，其中8路空间显示、4个同Topic云显示（400000总预算，实际约34.5万点）、单一WebGL上下文。四云共享网络订阅，不代表四台相机。传输档多连接在一个Node进程，不等同多个浏览器。

## 0.8.3压力结果

| 场景 | 实际负载/s | RPC失败/完成 | 最慢会话成功RPC P95/ms | 整组峰值RSS/MiB | CPU均值/P95（逻辑核） | 结果 |
|---|---:|---:|---:|---:|---:|---|
| 3客户端+慢读者，cache0，100ms查询 | 180.1 | 0/2928 | 108.5 | 753.2 | 0.41/0.53 | PASS |
| 6客户端+慢读者，cache0 | 180.1 | 0/534 | 50.1 | 809.5 | 0.49/0.66 | PASS |
| 3客户端+慢读者，显式cache64，100ms查询 | 180.0 | 0/5157 | 7.7 | 773.8 | 0.40/0.52 | PASS |
| JSON，独立回放30秒 | 各约30 | 见编码对照 | 见编码对照 | 731.0 | 0.79/1.08 | PASS |
| CBOR，独立回放30秒 | 各约30 | 见编码对照 | 见编码对照 | 551.3 | 0.28/0.32 | PASS |
| 3客户端+慢读者，cache0，600秒稳态 | 600.2 | 0/894 | 24.2 | 777.0 | 0.36/0.48 | PASS |
| 11显示/4点云，无界面GPU，DDS元数据 | 600.5 | 图谱/IDL逐轮完整核对 | — | 2201.7 | 1.10/1.39 | PASS |
| 同负载无界面GPU复测，无DDS探针 | 600.3 | 图谱/IDL逐轮完整核对 | — | 2222.3 | 1.09/1.36 | PASS |

CPU为进程user+system增量，1.0表示一个逻辑核持续满载；统计排除前15秒及负载器退出阶段。RSS是进程树求和，共享页会重复计数；PSS另每10秒采样且仅使用完整进程树样本。GPU分配未被完整计入，不能直接当作Jetson统一内存占用。

RPC P95取各代会话成功请求P95的最大值，失败请求另计，不是全部请求总体P95/P99。保留3秒RPC截止和成功会话P95<500ms；普通流10秒健康门、慢读者恢复8秒、回放首云偏移3319.42ms+1000ms（4320ms）恢复门均未放宽。浏览器资源门为整组RSS<3072MiB、整机MemAvailable≥4096MiB、全流程<800秒；传输内层RSS门1024MiB、外层1200MiB。它们是采样停测门，不是已证明生效的cgroup硬上限。

| 传输档 | 健康/含慢读者吞吐 MiB/s | 时间回退次数 | 慢读恢复/暂停 |
|---|---:|---:|---:|
| 3客户端+慢读者，cache0，100ms查询 | 29.27/39.22 | 2 | 4/4 |
| 6客户端+慢读者，cache0 | 58.39/68.35 | 2 | 4/4 |
| 3客户端+慢读者，显式cache64，100ms查询 | 29.28/39.31 | 2 | 4/4 |
| 3客户端+慢读者，cache0，600秒稳态 | 29.06/39.05 | 10 | 14/14 |

吞吐仅为WebSocket应用payload，非物理局域网/TCP/TLS速率；100ms为查询调度间隔，不保证精确10Hz。

### 编码对照

| 编码 | payload MiB/s | 彩色/深度/点云解码P95 ms | Service P50/P95 ms | Node峰值RSS MiB |
|---|---:|---:|---:|---:|
| none | 12.72 | 47.09/31.84/29.74 | 4.45/212.54 | 199.6 |
| cbor | 9.41 | 2.05/2.19/5.87 | 3.31/83.71 | 136.9 |

两种编码各自启动独立回放、从相同录制起点运行，仍不是逐帧配对压缩率试验；解码时间不含网络/全桥序列化或GPU执行。原串行JSON→CBOR档在末尾瞬时TF检查失败，原始FAIL及TF检查点诊断均保留，不被独立单编码通过覆盖。独立回放改变的是对照的录制相位，末尾仍要求完整TF，没有延长或放宽门限。只读复现时检查落在新一轮0.900秒，录制首条动态TF在1.022秒；此时缓存无动态帧符合时序，属于检查点选择问题。见本轮encoding、encoding-tf-checkpoint-final及encoding-checkpoint-analysis.json，原判定不改写。

### 无界面浏览器交互与点云恢复

| 轮次 | 回退次数/操作数 | 唯一上报离散交互P95/最大 ms | 稳态LongTask数/最大 ms | 完整树PSS峰值 MiB | 首60s/末60s RSS中位 MiB |
|---|---:|---:|---:|---:|---:|
| 11显示/4点云，无界面GPU，DDS元数据 | 10/113 | 40/280 | 1/174 | 1180.3 | 2075.1/2137.1 |
| 同负载无界面GPU复测，无DDS探针 | 10/113 | 40/248 | 1/76 | 1195.6 | 2088.1/2156.1 |

元数据轮收到1176次云回调，无元数据丢弃。writer源时间→reader历史插入P95 111.38ms、最大159.25ms；新时间域首云在回退后最大3415.4ms到达。该间隔包含录制首帧空档；reader历史时间不是网卡时间，不能当相机到屏幕延迟。
Event Timing仅包含原生上报≥16ms、interactionId>0的离散事件，按id取最大值，duration为8ms量化；不是INP、连续滚轮/拖动延迟或GPU时间。缓冲丢弃计数必须为0才接受。长任务按事件起始时间归属阶段。headless验证了同一NVIDIA WebGL渲染/解码/交互代码，但不证明可见窗口呈现、物理输入手感或NX桌面合成性能。首末内存窗口不是泄漏证明，仍需目标机数小时验证。

## Orin NX 16GB建议起点与待验收

| 项目 | 当前默认/边界 | NX本机浏览器建议起点（待实测） |
|---|---|---|
| OS/依赖 | ROS2 Jazzy，numpy1.26.4；不改系统驱动 | 先核对JetPack/BSP、Ubuntu与Jazzy ARM64兼容性，记录浏览器/GPU驱动版本 |
| Service缓存 | 默认0；非零仍有本机原生指纹门 | 保持0；不复制x86指纹到ARM |
| Service共享准入 | 默认128；创建等待5秒 | 显式16起步；全图查询内部并发4，保留余量；满额处理为失败，不自动重试 |
| SDK等待 | 每客户端Service/Action共用256 | 应用可设maxPendingRequests=64；与桥端Service总额分别生效 |
| 显示 | 最多16路配置/8路空间/40万总云点 | 模型+TF+1路云；Marker/Path/RGB/深度按需，先不并开RViz2/Lichtblick |
| 单云 | 默认10万点、CBOR/100ms | 5万点、CBOR/500ms起步；点数只减绘制，不减原始传输字节 |
| RGB/深度 | 最多400万像素/1600万输入字节、500ms | 640×480、1000ms起步；上游已有CompressedImage才选择，保留真实深度单位 |
| 画布 | 一个renderer，DPR自动上限2 | 1280×800或相近窗口/DPR1，记录实际值；没有虚构targetFps参数 |
| 曲线/图谱 | 600样本/8字段；图谱按需 | 1–3字段/300点；手动刷新，避免高频全图扫描 |
| 出站/入站 | q64 / incoming100，按消息数 | 保留；过载先降源数据/频率，不调大排队掩盖积压 |
| 时间/QoS | 回放显式useSimTime=true | 实时传感按发布者匹配Sensor/BEST_EFFORT；真机不误开仿真时钟 |

以上是部署起点，不是NX实测调优结论。先读取现场nvpmodel/BSP、供电和散热配置，不自动切功耗档或锁频；使用NVIDIA tegrastats与整机PSS、RAM/EMC/GPU、温度/降频、swap和业务推理进程共同测量。RSS共享页及GPU统一内存不能简单相加；至少保留4GiB MemAvailable作为本项目初始停测门，不能依赖swap证明响应合格。Jazzy首选Ubuntu24.04 ARM64；不能从RTX3090结果推算Orin帧率。

## 复测与使用

先source Jazzy和工作区；`ros2 launch ros2_web_link self_test.launch.py`启动无硬件示例。需要局域网访问时显式设置address，测试结束在launch终端Ctrl+C并按PID核对自己的进程。通用包不依赖Peach业务；Peach录制样例另需同版本peach_interfaces与aubo_description。

NX只读回放示例（替换实际路径）：

```bash
ROS_DOMAIN_ID=220 ros2 launch ros2_web_link replay.launch.py \
  bag:=/实际录制目录/peach_rgbd_mock_60s project:=peach_replay_large \
  asset_packages:=ros2_web_link,aubo_description \
  bridge_service_client_cache_size:=0 service_client_max_in_flight:=16
```

同机浏览器打开http://127.0.0.1:8388/#workbench。点云属性中设置500ms/CBOR/maxPoints=50000；RGB与深度各1000ms，录制深度depthScale=0.00025不改。保存工作区JSON。真实NX验收仍需本机可见浏览器、现场功耗/散热模式、目标业务同时运行及至少30分钟稳态，记录零RPC失败、完整图谱/IDL、循环恢复、各流健康、内存/GPU余量与退出清理；有界面手测由操作员主动执行，不用自动化抢焦点。

本轮原始数据：[验收目录](../../../runs/ros2_web_v083_acceptance_20261103/)、[统计](../../../runs/ros2_web_v083_acceptance_20261103/analysis.json)、[核验/清理](../../../runs/ros2_web_v083_acceptance_20261103/verification.json)。`analyze.py`重算统计，不修改原始判定。每次新压测使用新输出目录，冻结实际运行代码，不覆盖失败记录；无界面脚本保留headless=true与硬件NVIDIA渲染器断言。所有用例结束检查原始功能报告与外层资源报告，任一失败都不算通过。

以下完整保留0.8.2失败基线，里面的默认64/缓存专属128项等描述仅适用于历史版本。当前部署参数以本页上半部分与launch.md/io.md为准。

<details>
<summary>展开历史0.8.2完整报告（归档参数，不作当前指南）</summary>

# 压力测试与 Jetson Orin NX 16GB 部署参数

目标由用户确认：**Jetson Orin NX 16GB，ROS 2 与浏览器同机运行**。目前未连接目标机，JetPack、载板、供电和散热型号待确认。**本轮已检出 cache0 路径的查询超时和可重复的工作台循环回放点云恢复超期，目标机暂不予生产性能放行。** 本轮测量设备是 x86 桌面；本文给出可复核的桌面压力结果、ARM64 部署前置条件和目标机验收门，不能据此宣布 NX 已验收。

本轮仅新增测量脚本与文档，未发行新运行版本；原0.8.2归档不覆盖。最新说明以工作区本文件为准。

## 环境、负载与统计口径

生产运行版本为 ros2_web_link 0.8.2。测试没有改动 SDK、桥接适配、页面或 DDS 设置；使用现有显式行为探针，外加本轮 `runs` 内的 CPU/RSS 采样与 600 秒浏览器操作脚本。每个用例独立启动，运行前后核对被测文件哈希、PID 创建时间和端口。原始数据与可重算脚本见[本轮目录](../../../runs/ros2_web_stress_20261102/)。

| 项目 | 本轮桌面条件 | 最终目标条件 |
|---|---|---|
| CPU / 架构 | Intel Core Ultra 7 265K，20 个逻辑 CPU，x86_64 | Orin NX 16GB，ARM64；实际在线核心数受功耗模式影响 |
| RAM | 系统可见 31457MiB | 标称 16GB；CPU、GPU、桌面与推理共享内存 |
| GPU / 浏览器 | RTX3090，驱动595.99.02，Chrome152.0.7977.82；1440×1000、DPR1、ANGLE/OpenGL | 目标机 Chromium 与实际 NVIDIA WebGL 渲染器待核对 |
| OS / ROS | Ubuntu 24.04.5，内核 6.8.0-147，Jazzy | 目标系统版本待核对，Jazzy 首选 Ubuntu 24.04 ARM64 |
| 通信 | rosbridge 2.7.1，FastDDS，回环地址，共享 WebSocket，无自定义 XML | 同一基线优先；不能把回环结果当局域网结果 |
| 数据 | 已结束录制的真实 RGB-D 相机 + mock 机械臂观测 | 首轮先使用同一数据，不接硬件 |

录制长 **59.471871442s**，两个 MCAP 共 **1,207,874,729B**。完整点云 116 帧，最大消息约 4.15MB；深度 453 帧、相机原始彩色 78 帧、Peach 彩色结果图 612 帧，图像为 640×480。传输探针选 `/camera/color/image_raw`，浏览器选 `/peach/perception/debug_image`，两档彩色源不同；不能将两档CPU或接收率视为同一负载直接比较。回放 1 倍速循环，`/clock` 30Hz、read-ahead queue 100；云订阅间隔 100ms、彩色/深度 500ms，不代表相机 10Hz/2Hz 保证，实际接收率受录制源、节流和循环空档影响。

- 传输压力：独立 SDK 连接共同接收完整云、RGB-D、关节、TF；额外一个慢读者每 45s 中暂停读 socket 15s，再恢复。中途重建两个健康客户端。多客户端在一个 Node 进程内，不等于多个 Chromium 标签页。
- 浏览器压力：一个有界面的独立 Chromium、11 路显示、8 路空间显示、4 个同 Topic 点云显示，总配置预算 400000 点，实际约 34.5 万点；重复操作视角、属性、图谱刷新/过滤、接口查询、开关、导航与滚动。四个显示复用同一 Topic 网络订阅，不等于四台相机。
- CPU：每秒采样各自进程的 `user+system` 增量，`1.0 core` 表示一个逻辑 CPU 持续满载。表内均值/P95 排除前 15s 与负载生成器退出后的样本；不能按主频或核心数换算为 NX 速度。
- RSS：整组进程 RSS 相加，含桥、rosapi、HTTP、回放、负载器/浏览器及 launch；共享映射会重复计数。浏览器另每 10s 采样 PSS，只有整组进程均成功采集的样本才计入汇总。本机 systemd-run 摘要仅报告KB级 MemoryPeak，与进程树采样不符，因此不用于内存结论或外层限制生效证明；停测门由脚本采样执行。GPU 独立/统一内存不能由 RSS 推算；NX 需结合整机 tegrastats。[psutil 内存定义](https://psutil.io/7.2/#psutil.Process.memory_full_info)
- 吞吐：应用收到的 WebSocket payload 字节，不含 TCP/TLS/DDS 帧头。多客户端统计按所有会话开始至关闭的时间跨度计算，健康客户端和慢读者分列。
- 不测相机到屏幕端到端延迟：录制header时间不是当前墙钟，不能拿两者相减。传感器到像素的延迟需目标机在线时间同步和专门取证。
- RPC：传输探针用 `/rosapi/nodes` 只读查询；保持 **3s 请求截止、每代会话 P95 <500ms**。多客户端探针保存每代成功请求的 P95，本文取其中最大值；失败请求另计，不会进入这些延迟分位数，不能把它误写成全部请求的总体 P95/P99。
- 浏览器沿用组件截止：普通 SDK Service 默认5s，图谱单次调用最多3s、整体快照默认10s/并行4；要求完整快照，部分结果也判失败。
- 浏览器事件：PerformanceObserver 原生 Event Timing/Long Task/LoAF；仅上报 ≥16ms 的离散事件，duration 以 8ms 量化。同一 interactionId 合并取最大 duration。原采样长测缓冲溢出的结果单独标注不完整；修正采样只保存参与同一指标的 interactionId>0 事件并计数忽略项，仍保持6000条上限。[W3C Event Timing 工作草案的分组示例](https://w3c.github.io/event-timing/)。它不是 INP、连续拖动/滚轮延迟或 GPU 时间；自动操作耗时还包含等待与截图。

循环边界会清空旧时间域数据，云第一帧偏移 3.319s；浏览器沿用冻结录制时序加 1s 的恢复预算。普通连续流保持 10s 健康门，慢读者恢复 8s 门；未放宽超时或内存上限来获得通过。

## 本轮实测

下表是本轮桌面固定负载的分档结果，不是系统极限、NX 换算值或长期无故障保证。`PASS` 仅表示该档完成了既定时长和探针门限。

| 场景 | 计划/实际负载时长 | 整组峰值 RSS / MiB | CPU 均值 / P95（核） | RPC 失败/完成数 | 最慢会话成功 RPC P95 / ms | 结果 |
|---|---:|---:|---:|---:|---:|---|
| 单客户端 JSON/CBOR | 各30s / 32.4s+30.3s | 693.3 | 0.51 / 0.99 | 见下表 | 见下表 | **PASS** |
| 3客户端+慢读者，cache0 | 600s / 459.2s | 776.1 | 0.37 / 0.49 | 2/681 | 26.8 | **FAIL** |
| 3客户端+慢读者，cache64 | 600s / 600.1s | 781.4 | 0.38 / 0.50 | 0/897 | 11.9 | **PASS** |
| 6客户端+慢读者，cache64 | 180s / 180.1s | 808.7 | 0.51 / 0.69 | 0/540 | 28.6 | **PASS** |
| 3客户端+慢读者，cache0，100ms查询 | 180s / 36.7s | 712.4 | 0.45 / 0.55 | 2/879 | 24.5 | **FAIL** |
| 3客户端+慢读者，cache64，100ms查询 | 180s / 180.1s | 761.2 | 0.42 / 0.52 | 0/5100 | 12.4 | **PASS** |
| 本机浏览器11显示，cache0（原采样） | 600s / 534.8s | 2470.3 | 1.58 / 2.00 | 图谱/IDL另验 | — | **FAIL** |
| 本机浏览器11显示，cache0（修正采样） | 600s / 534.6s | 2470.8 | 1.57 / 1.98 | 图谱/IDL另验 | — | **FAIL** |

时长按探针会话跨度记录，可能包含初始连接/收尾；失败档没有补足计划时长。CPU采样不包含采样器自身、Codex及其他未归属进程。

| 传输压力档 | 健康客户端 payload / MiB·s⁻¹ | 加慢读者总 payload / MiB·s⁻¹ | 实际RPC/s | 回退次数 | 慢读者恢复/暂停 |
|---|---:|---:|---:|---:|---:|
| 3客户端+慢读者，cache0，100ms查询 | 28.42 | 36.85 | 23.95 | 0 | 1/1 |
| 3客户端+慢读者，cache64，100ms查询 | 29.00 | 38.99 | 28.31 | 2 | 4/4 |
| 6客户端+慢读者，cache64 | 58.45 | 68.43 | 3.00 | 2 | 4/4 |
| 3客户端+慢读者，cache0 | 29.58 | 39.74 | 1.48 | 7 | 10/10 |
| 3客户端+慢读者，cache64 | 28.92 | 38.89 | 1.49 | 10 | 14/14 |

100ms 是探针请求调度间隔，不保证每客户端精确10Hz：同步等待、图像处理和循环调度都会降低实际调用率。

### 编码对照

| 编码 | payload / MiB·s⁻¹ | 彩色解码P95 / ms | 深度解码P95 / ms | 点云解码P95 / ms | 服务P50 / P95 / ms | Node峰值RSS / MiB |
|---|---:|---:|---:|---:|---:|---:|
| none | 12.78 | 58.35 | 39.04 | 32.43 | 3.60 / 327.63 | 179.9 |
| cbor | 9.14 | 3.83 | 5.15 | 3.30 | 2.46 / 5.82 | 132.7 |

两种编码顺序运行，录制位置与接收帧数不同，因此这些数不是逐帧配对的压缩率试验。解码只计适配器回调内处理，不含完整 rosbridge 序列化、网络和浏览器绘制。CBOR优先的建议基于本次观察及二进制输入路径，实际收益需在ARM上重测。

### 资源分布

| 场景 / 进程组 | 平均CPU / 核 | 活跃窗口RSS峰值 / MiB |
|---|---:|---:|
| 3客户端+慢读者，cache0 / bag-player | 0.050 | 152.5 |
| 3客户端+慢读者，cache0 / bridge | 0.204 | 299.2 |
| 3客户端+慢读者，cache0 / http | 0.000 | 23.8 |
| 3客户端+慢读者，cache0 / launch-and-harness | 0.023 | 62.4 |
| 3客户端+慢读者，cache0 / rosapi | 0.002 | 81.1 |
| 3客户端+慢读者，cache0 / sdk-load-generator | 0.092 | 171.7 |
| 3客户端+慢读者，cache64 / bag-player | 0.049 | 200.8 |
| 3客户端+慢读者，cache64 / bridge | 0.213 | 281.3 |
| 3客户端+慢读者，cache64 / http | 0.000 | 23.8 |
| 3客户端+慢读者，cache64 / launch-and-harness | 0.022 | 62.9 |
| 3客户端+慢读者，cache64 / rosapi | 0.001 | 81.2 |
| 3客户端+慢读者，cache64 / sdk-load-generator | 0.091 | 172.9 |
| 6客户端+慢读者，cache64 / bag-player | 0.049 | 149.2 |
| 6客户端+慢读者，cache64 / bridge | 0.278 | 325.1 |
| 6客户端+慢读者，cache64 / http | 0.000 | 23.8 |
| 6客户端+慢读者，cache64 / launch-and-harness | 0.021 | 61.4 |
| 6客户端+慢读者，cache64 / rosapi | 0.002 | 81.3 |
| 6客户端+慢读者，cache64 / sdk-load-generator | 0.160 | 174.1 |
| 3客户端+慢读者，cache0，100ms查询 / bag-player | 0.058 | 133.9 |
| 3客户端+慢读者，cache0，100ms查询 / bridge | 0.248 | 273.4 |
| 3客户端+慢读者，cache0，100ms查询 / http | 0.000 | 23.9 |
| 3客户端+慢读者，cache0，100ms查询 / launch-and-harness | 0.023 | 61.2 |
| 3客户端+慢读者，cache0，100ms查询 / rosapi | 0.012 | 81.0 |
| 3客户端+慢读者，cache0，100ms查询 / sdk-load-generator | 0.111 | 157.3 |
| 3客户端+慢读者，cache64，100ms查询 / bag-player | 0.051 | 155.3 |
| 3客户端+慢读者，cache64，100ms查询 / bridge | 0.230 | 303.7 |
| 3客户端+慢读者，cache64，100ms查询 / http | 0.000 | 23.8 |
| 3客户端+慢读者，cache64，100ms查询 / launch-and-harness | 0.022 | 61.4 |
| 3客户端+慢读者，cache64，100ms查询 / rosapi | 0.009 | 81.4 |
| 3客户端+慢读者，cache64，100ms查询 / sdk-load-generator | 0.104 | 165.9 |
| 本机浏览器11显示，cache0（原采样） / bag-player | 0.052 | 212.3 |
| 本机浏览器11显示，cache0（原采样） / bridge | 0.306 | 226.3 |
| 本机浏览器11显示，cache0（原采样） / browser-gpu-process | 0.693 | 316.8 |
| 本机浏览器11显示，cache0（原采样） / browser-other | 0.092 | 706.6 |
| 本机浏览器11显示，cache0（原采样） / browser-renderer | 0.344 | 696.5 |
| 本机浏览器11显示，cache0（原采样） / http | 0.000 | 24.6 |
| 本机浏览器11显示，cache0（原采样） / launch-and-harness | 0.000 | 41.8 |
| 本机浏览器11显示，cache0（原采样） / playwright-runner | 0.094 | 229.6 |
| 本机浏览器11显示，cache0（原采样） / rosapi | 0.001 | 82.9 |
| 本机浏览器11显示，cache0（修正采样） / bag-player | 0.053 | 216.6 |
| 本机浏览器11显示，cache0（修正采样） / bridge | 0.310 | 218.3 |
| 本机浏览器11显示，cache0（修正采样） / browser-gpu-process | 0.670 | 300.5 |
| 本机浏览器11显示，cache0（修正采样） / browser-other | 0.093 | 705.4 |
| 本机浏览器11显示，cache0（修正采样） / browser-renderer | 0.349 | 715.2 |
| 本机浏览器11显示，cache0（修正采样） / http | 0.000 | 24.9 |
| 本机浏览器11显示，cache0（修正采样） / launch-and-harness | 0.000 | 41.8 |
| 本机浏览器11显示，cache0（修正采样） / playwright-runner | 0.091 | 227.2 |
| 本机浏览器11显示，cache0（修正采样） / rosapi | 0.001 | 82.9 |

browser-gpu-process 的CPU数值是Chromium GPU进程的CPU消耗，不是GPU利用率；后者需目标机GR3D等观测。各进程组峰值发生时间可能不同，不能相加替代整组同时峰值。RSS首/末60s窗口中位数只用于观察，包含分配器缓存与GC，10分钟记录不能证明不存在泄漏。

| 场景 | 首60s窗口RSS中位数 / MiB | 末60s窗口RSS中位数 / MiB | 差值 / MiB |
|---|---:|---:|---:|
| 本机浏览器11显示，cache0（原采样） | 2155.7 | 2407.5 | +251.8 |
| 本机浏览器11显示，cache0（修正采样） | 2158.8 | 2413.2 | +254.4 |
| 3客户端+慢读者，cache0，100ms查询 | 645.5 | 645.5 | +0.0 |
| 3客户端+慢读者，cache64，100ms查询 | 637.5 | 686.9 | +49.4 |
| 6客户端+慢读者，cache64 | 707.5 | 741.4 | +33.9 |
| 3客户端+慢读者，cache0 | 659.5 | 706.1 | +46.6 |
| 3客户端+慢读者，cache64 | 645.4 | 729.0 | +83.6 |

短于120s的有效样本窗口重叠，尤其高频cache0只运行36.7s，差值0不代表稳定。窗口从外层采样15s后、且负载器存活的样本开始选取；与每次回放/GC的相位并不严格配对。存在增长的用例不能标记为长期内存稳定，需延长观察并区分播放器缓存、桥接分配器和浏览器资源。

### 本机浏览器11显示，cache0（原采样）

判定 **FAIL**；观察到 9 次回放时钟回退，完成 101 个带阶段标签的自动交互；样本最大点数 345472。

整组RSS峰值 2470.3MiB；完整PSS样本 52 个，峰值 1473.5MiB。
页面JS堆采样峰值 144.6MiB。原生上报的 338 个独立 interactionId，duration P95 72ms、最大 144ms。
稳定流阶段长任务 1 次，最大 88ms；观察器丢弃计数 {"longtask": 0, "long-animation-frame": 4, "event": 408}。

| 阶段 | >50ms长任务次数 | 最大时长 / ms |
|---|---:|---:|
| initial-eight-displays | 1 | 87.0 |
| navigation | 1 | 62.0 |
| steady-streams | 1 | 88.0 |

**本档事件证据不完整：缓冲到达6000条上限，以上分位数/最大值只覆盖已保留前缀，不能代表整段试验。** 最后保留事件的页面时间约 515.76s。

图谱最后样本规模 {'nodes': 3, 'topics': 21, 'edges': 36, 'services': 61, 'actions': 0}；这是录制回放节点图，不是完整Peach运行图。

最后一次采样的累计渲染提交率 57.1fps，最近120帧CPU提交P95 0.30ms；它们不是GPU完成帧率/耗时。

浏览器失败原因：`AssertionError [ERR_ASSERTION]: point stream empty outside bounded rewind recovery`。本次未完成600秒及末尾暂停/重启/断开断言；finally已退出，不能用此前短测覆盖此失败。

### 本机浏览器11显示，cache0（修正采样）

判定 **FAIL**；观察到 9 次回放时钟回退，完成 101 个带阶段标签的自动交互；样本最大点数 345552。

整组RSS峰值 2470.8MiB；完整PSS样本 52 个，峰值 1488.1MiB。
页面JS堆采样峰值 147.7MiB。原生上报的 358 个独立 interactionId，duration P95 80ms、最大 216ms。
稳定流阶段长任务 0 次（无最大时长样本）；观察器丢弃计数 {"longtask": 0, "long-animation-frame": 0, "event": 0}。

| 阶段 | >50ms长任务次数 | 最大时长 / ms |
|---|---:|---:|
| graph-refresh-filter-and-fit | 1 | 54.0 |
| initial-eight-displays | 1 | 189.0 |
| navigate-and-return | 1 | 50.0 |
| navigation | 1 | 61.0 |

采样修正只过滤不进入交互指标的 interactionId=0 事件，单独计数 {'nonInteractionEvents': 5557}；6000条上限、16ms阈值及所有功能门限不变。

图谱最后样本规模 {'nodes': 3, 'topics': 21, 'edges': 36, 'services': 61, 'actions': 0}；这是录制回放节点图，不是完整Peach运行图。

最后一次采样的累计渲染提交率 56.5fps，最近120帧CPU提交P95 0.60ms；它们不是GPU完成帧率/耗时。

浏览器失败原因：`AssertionError [ERR_ASSERTION]: point stream empty outside bounded rewind recovery`。本次未完成600秒及末尾暂停/重启/断开断言；finally已退出，不能用此前短测覆盖此失败。

### 发现与边界

cache0 三客户端负载在约459s会话跨度内出现两次 `/rosapi/nodes` 超时（681次完成请求中2次失败）。内存门未触发，进程仍存活并正常清理；成功请求P95低不能抵消超时。100ms调度下的cache0又在36.7s、879次请求中出现2次超时；cache64相同调度完成180s、5100次零失败。无诊断基线不能把故障唯一归到DDS、rclpy发现或排队，当前保留为服务客户端路径稳定性问题。
两次本机浏览器压力均在约535s、第9次循环后触发点云回填超期，复测的事件记录没有丢弃。复测失败样本距离最近云回调4.934s；从观察到时钟回退起计算的4.320s期限已超出52ms，两个计时起点不同。图谱快照完整、渲染/着色器错误为0。该证据表明约定恢复时间未满足，尚不能单凭它判定是DDS、桥队列、浏览器接收还是渲染层的唯一根因。两个600s计划与末尾暂停/重启/断开断言均未完成。
缓存64与缓存0的对照只用于本机条件比较。默认缓存所需原生指纹尚未审核ARM64，所以桌面缓存开启成功也不能消除NX阻断。首轮目标机仅允许受监测的cache0验证；生产放行仍需原生兼容性和长稳态补测。

本轮没有通过增加服务截止时间、出站队列或RSS门限掩盖失败。原始失败与清理结果分别保留；并发数、数据率、图谱规模都受本录制限制，不宣称“兼容一切ROS图规模”或“达到最大吞吐”。

逐项输入、主机信息、日志、资源采样、界面截图见[原始证据](../../../runs/ros2_web_stress_20261102/)，汇总见[analysis.json](../../../runs/ros2_web_stress_20261102/analysis.json)。


## NX 软件与架构前置条件

2026-11-02 查阅官方版本信息：JetPack **7.2.1 / Jetson Linux 39.2.1** 已列出 Orin 系列，基于 **Ubuntu 24.04 / kernel 6.8**；不能再笼统写“Orin 的 JetPack 只支持 22.04”。这只是候选基础系统，实际载板 BSP 与相机/机械臂驱动、Peach 的 CUDA/TensorRT/PyTorch 依赖须分别核对，不进行自动刷机或全栈迁移。[NVIDIA 版本说明](https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2.1)

若设备仍是 JetPack 6.2.3，其基础系统为 Ubuntu 22.04 / Jetson Linux 36.5.2。Jazzy 的 Ubuntu 24.04 ARM64 是 Tier 1，而 22.04 ARM64 不在该 Tier 1 矩阵。不要混装 Noble deb 到 Jammy；先选择受载板支持的 24.04 系统，或另行验证 24.04 ARM64 容器的 DDS、设备与 GPU 路径。本轮未验证容器方案。[JetPack 6.2.3](https://developer.nvidia.com/embedded/jetpack-sdk-623)、[REP-2000 Jazzy 平台矩阵](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst)

**当前代码存在必须显式处理的 ARM64 启动门：** `scripts/service_client_compat.py::_review_runtime()` 对默认缓存路径核对 rclpy 7.1.12、Python 方法源码和一个已审 x86 原生库 SHA256。ARM64 二进制不同，不能复制此指纹或绕过检查。**cache0 在本轮桌面三客户端长测出现 3s 查询超时（见上表），因此目前只用于受监测的目标机验证，不予生产性能放行。** 第一次 NX 验证启动必须设置 `service_client_cache_size:=0`；回放 launch 对应名称为 `bridge_service_client_cache_size:=0`。这选择上游逐调用客户端路径，仍核对 rosbridge 2.7.1 及有关方法，不代表支持任意最新版 rosbridge。缓存开启需要目标机源码审查与独立原生长测，当前未完成。

| 层 | 当前锁定/要求 | ARM64 处理 |
|---|---|---|
| ROS / Python | Jazzy / Python 3.12；ROS 包走 apt 或同版源码构建 | 在目标机确认实际 rosbridge-server/library 2.7.1 与依赖，未知布局拒启时不得去掉检查 |
| rclpy 服务缓存 | 默认 64，cache0 可部署 | 首次设 0；本轮桌面 cache0 结果只是参考，不是 ARM 原生证明 |
| Web 构建 | Node 22+；本轮 Node 22.22.3；lockfile 固定依赖 | 在 ARM64 上 `npm ci`；不要复制 x86 node_modules/esbuild 二进制 |
| 浏览器 SDK | roslib 2.1.0、Three 0.186.1、Cytoscape 3.34.4 等以 lockfile 为准 | JS/dist 可复用，但 GPU/浏览器表现须本机测；运行 HTTP 服务不依赖 Node 常驻 |
| 浏览器 | 本轮桌面 Chrome，目标 Chromium 待定 | 记录实际版本、viewport/DPR；确认 WebGL 用 NVIDIA，而非 SwiftShader/llvmpipe，不使用忽略 GPU 黑名单或关闭沙箱参数掩盖问题 |
| 模型和录制 | 模型资产通过 package 白名单读取，录制单独存储 | 本Peach样例还需同录制版本的 peach_interfaces（仅IDL，无节点），在ARM上构建/安装；通用SDK不依赖它；不要依赖 x86 厂商驱动库 |
| Peach 推理环境 | 本包自身不使用 CUDA 推理 | 并行推理另测；保持工作区 numpy==1.26.4 约束，不因 JetPack 更新自动升级 ABI |

## NX 16GB 本机浏览器起步参数

以下是**待目标机验证的保守起步值**，不是已测容量或新的默认值。先保存工作台配置，再逐项增加；同 Topic 的多个显示必须用相同类型、QoS 和订阅参数。

| 项目 | 当前值/硬上限 | NX 起步建议与理由 |
|---|---|---|
| 活跃浏览器/连接 | 未声明多浏览器容量保证 | 一个 Chromium 窗口、一个工作台、共享一个 SDK 连接；先不并开 RViz2、Lichtblick 和额外云页面 |
| 画布 | 共一个 WebGL renderer；DPR 当前自动上限 2 | 1280×800 或相近窗口、实际 DPR 1；记录缩放。当前没有 `targetFps`、`maxFps` 或 DPR launch 参数 |
| 显示数量 | 16 路配置、8 路同时空间显示 | 模型+TF+1 路云；Marker/Path 按需；RGB 与深度按需启用。四云是压力档，不是默认运行档 |
| 单云点预算 | `maxPoints` 默认/上限 100000；总云预算 400000 | 50000 起步；先减绘制点数。该值在浏览器采样，不减少原始传输字节 |
| 云订阅 | CBOR，100ms | CBOR，500ms 起步；仍过载用上游标准 PointCloud2 降采样话题，数据处理不搬到 HTTP 代理 |
| 原始 RGB/深度 | CBOR，500ms；图像最大 4000000 像素 | 640×480、1000ms 起步；每路独立开关。maxPixels 只拒绝超预算帧，不会缩图；尺寸由上游决定 |
| 压缩图像 | 支持普通 PNG/JPEG CompressedImage | 上游已有 image_transport 压缩话题才选择；CBOR 不是 JPEG/H.264，当前不支持 compressedDepth，不能承诺使用 Jetson 视频硬解 |
| 关节 | 项目模板可无节流 | 50ms 起步用于显示；不改控制器、运动命令频率，也不随意节流用于精确计算的 TF |
| 曲线 | 最多 8 字段、600 样本，100ms 绘图节流 | 1–3 字段、300 点起步；不保存原始图像/点云历史 |
| 图谱/接口 | 默认256节点/512Topic/2048边；用户触发快照 | 手动刷新；不要每 100ms 全图扫描。压测 RPC 频率不是产品刷新建议 |
| 服务缓存 | 默认 64；范围 0–1024 | ARM 首次 0；不要默认使用未审 ARM 原生缓存路径 |
| 出站队列 | `write_queue_size=64`，单位为消息 | 初始保留已测 64；不是 64MB，不是按字节的全局内存上限。过载优先减频/减源数据，不调大队列 |
| 入站/消息 | incoming queue100；单消息16000000B | 保留；检查 `transportStats()` 拒绝数，不扩大限制容纳未知负载 |
| 分片预算 | SDK 合计32000000B、8 组、每组4096片、30s | 保留；分片不是免内存传输，也不是限流替代品 |
| 在途请求 | SDK 默认256；缓存路径128 | 应用避免高并发；cache0 不采用缓存路径的128项保护，不能以此宣称全桥请求总量被限制为128 |
| QoS / 时间 | 实际项目显式设置 | 录制沿用已测 Reliable；实时传感按发布者匹配 Sensor/BEST_EFFORT。回放 useSimTime=true，真机不得误开 |

页面设置步骤：点击点云显示名称 → 展开“传输设置 · QoS / 编码”，订阅间隔填 `500`、传输选 `CBOR` → 展开“显示选项 · JSON”，保留其他有效项并将 `maxPoints` 设为 `50000` → 点“应用显示属性”。对RGB/深度各设1000ms；不要改掉录制深度的 `depthScale=0.00025`。确认正常后在“工作区设置与保存”选择“保存到本机”或“下载配置 JSON”。建议从一条点云开始；若保留同Topic多个显示，需一起保持传输参数兼容。

初始资源门建议：Web 整组 RSS ≤3GiB 作为压力探针停测条件，整机 `MemAvailable` 至少保留4GiB；同时记录 PSS、tegrastats RAM/EMC/GPU 和推理进程。RSS 中的共享页与 GPU 分配使这些数不能简单相加。若实际目标任务已没有余量，先关额外显示/降低数据率，不能靠 swap 证明实时响应合格。3GiB/4GiB 是本项目的验收起点，非 NVIDIA 官方限值，也不是“Peach 还一定可用多少 GB”的估算。

目标功耗模式先读 `sudo /usr/sbin/nvpmodel -q` 和 `/etc/nvpmodel.conf`。官方 NX 16GB 表中 15W 与25W 的在线核心/CPU/GPU上限不同；25W 不保证所有部件都比15W更快。40W/MAXN_SUPER 依赖刷机配置、载板供电与散热，不能把 `nvpmodel -m 0` 作为统一优化命令。先测现场部署模式，再由操作员确认是否比较另一受支持模式；不默认锁频或切 MAXN。[NVIDIA 功耗和时钟表](https://docs.nvidia.com/jetson/archives/r39.2.1/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html#supported-modes-and-power-efficiency)

官方 r39.2.1 表中的部分 NX 16GB 配置如下，仅用于理解部署差异；目标机必须读取自己的 nvpmodel/BSP，频率列是上限而非持续实测值。[NVIDIA NX 16GB 配置](https://docs.nvidia.com/jetson/archives/r39.2.1/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html#supported-modes-and-power-efficiency)

| 功耗档 | 在线 CPU | CPU 上限 / MHz | GPU 上限 / MHz | 条件 |
|---|---:|---:|---:|---|
| 15W | 4 | 1420.8 | 612 | 标准或相应 Super 配置 |
| 25W | 8 | 1497.6 | 408 | 标准或相应 Super 配置 |
| 40W | 8 | 1497.6 | 1173 | Super 刷机配置及兼容供电/散热 |

VDD_IN 是模块电源轨观测，不等于整机插座功耗；相机、USB与载板外设也需计入实际供电。使用当前动态调频模式建立基线，不用锁频结果代替日常运行条件。

## 可复测步骤

### 本轮桌面证据复核

运行[分析脚本](../../../runs/ros2_web_stress_20261102/analyze.py)重新计算 JSON 汇总，原始日志与探针判定不被改写：

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
python3 runs/ros2_web_stress_20261102/analyze.py
```

`run_case.py` 与 `run_browser_stress.py` 属于这台桌面、本录制与本次目录布局的外部采样封装，不是跨设备安装器。每次重新测试应选新的输出目录，保留失败记录；不得覆盖已有的 `inputs.json` 或在测试中重建/改动冻结源码。运行结束需同时检查用例原始报告与外层资源报告；“网页 PASS”不覆盖资源退出失败。

### NX 首轮只读回放

先核对 BSP/版本并在目标 ARM64 环境构建本包，复制已完成的录制与模型资产。该项目合同包含 `peach_interfaces/msg/PeachTargetObservationArray`，须同时构建/安装同版本 `peach_interfaces` 与 `aubo_description`；它们是可选回放样例依赖，不给通用包加入Peach业务依赖。可用 `ros2 interface show peach_interfaces/msg/PeachTargetObservationArray` 验证IDL可见。在目标机工作区中设置实际路径：

```bash
source /opt/ros/jazzy/setup.bash
cd /实际工作区
source install/setup.bash
pgrep -af 'ros2 launch|ros2 bag|component_container|extrinsics_publisher|ros2 run'

ROS_DOMAIN_ID=220 ros2 launch ros2_web_link replay.launch.py \
  bag:=/实际录制目录/peach_rgbd_mock_60s \
  project:=peach_replay_large \
  asset_packages:=ros2_web_link,aubo_description \
  bridge_service_client_cache_size:=0 \
  web_port:=8088 ws_port:=9090
```

本机浏览器打开 `http://127.0.0.1:8088/#workbench`，按上表先启用少量显示，保存工作区。先运行5分钟基本显示检查，随后30分钟单页重复交互；再分别跑多客户端和高频RPC，最后才增加 Peach 实际算法负载。录制重放本身不运行重建或推理，不验证 Peach 作业成功率。

另开终端采样；`tegrastats` 可用前景模式，结束按 Ctrl+C。记录频率为1000ms，保留 RAM/SWAP、CPU@频率、EMC、GR3D、温度和 VDD_IN；字段随 BSP 变化，缺字段标未采集，不写成0。不要对他人的采集进程执行 `tegrastats --stop`。[NVIDIA tegrastats 用法](https://docs.nvidia.com/jetson/archives/r39.2.1/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html)

```bash
tegrastats --interval 1000
```

复用现有只读多客户端探针时，先结束上一套回放并确认域/端口清空，再从已 source 的工作区运行。此命令内部启动回放并清理自己的子进程；输出必须是新目录，Node须在 PATH：

```bash
python3 src/ros2_web_link/scripts/run_transport_benchmark.py \
  --bag /实际录制目录/peach_rgbd_mock_60s \
  --evidence /实际证据目录/nx16-cache0-30min \
  --project peach_replay_large --domain 220 \
  --web-port 22688 --ws-port 22690 \
  --asset-packages ros2_web_link,aubo_description \
  --service-client-cache-size 0 \
  --multi-client-seconds 1800 --client-count 3
```

探针保留1024MiB ROS+Node组RSS门和4GiB整机可用内存门；这组不启动浏览器。目标机因此拒测或失败时保留报告，先确认实际负载/内存，不在命令中直接提高门限。默认请求周期2s，SDK一般调用默认超时5s，此压力探针显式采用3s；并发档改 `--client-count 6`，高频档改 `--rpc-period-ms 100`，逐档单独测试。

## 目标机验收表

| 项目 | 目标与取证 | 当前状态 |
|---|---|---|
| ARM64 启动 | 锁定 rosbridge与依赖；cache0启动/退出正常；普通通信回归完成 | 未连接设备 |
| 本机硬件渲染 | 记录 Chromium版本、NVIDIA WebGL renderer、DPR、viewport；无软件回退 | 未测 |
| 常用显示负载 | 30min：单云50k、RGB-D按需、图谱与接口查询；无崩溃/GL丢失/断流 | 未测 |
| 压力档 | 相同录制四云/11显示；另测3/6客户端、慢读者、重连、循环 | 桌面结果见上；NX未测 |
| 服务响应 | 3s截止零失败；最慢会话P95<500ms；保留失败次数 | NX未测 |
| 交互响应 | 原生离散交互目标P95≤200ms、单次≤500ms；首开/重启单列；不冒充INP | NX未测；属于本项目体验目标 |
| 渲染 | 稳态采样提交帧率目标≥30fps，另核对肉眼连续性/原生帧时序；不是GPU完成FPS | NX未测 |
| 统一内存 | 无OOM、无持续swap增长；MemAvailable≥4GiB；PSS/RSS/tegrastats趋势完整 | NX未测 |
| 热与功耗 | 按目标模式稳定30min，记录温度/频率/EMC/GPU/VDD_IN；无与卡顿对应的持续热/功率降频 | NX未测；阈值取实际BSP/载板规范 |
| 与Peach并行 | 同时跑实际感知/规划/可选重建，比较关闭/开启重建两档；默认关闭 | 未测；独立包回放不覆盖 |
| 释放 | 暂停/断线后订阅/在途/活跃GL归零；退出本次拥有进程、端口关闭 | 桌面逐例核对；NX待核对 |

本轮没有测 ARM 原生性能、真实局域网、Jetson 显示驱动、热稳态或 Peach 推理共存。没有以桌面 CPU 限额模拟 NX，也没有用 TOPS 推导 rosbridge/JavaScript 处理速度。结束手动测试后先暂停并断开页面，再 Ctrl+C 停止回放与采样，按 PID 身份核对清理；Gazebo 仍屏蔽。

## 证据与结束检查

八档均使用各自的新目录，失败没有覆盖。运行输入哈希、端口释放与拥有进程核对见[结束验证](../../../runs/ros2_web_stress_20261102/verification.json)；此验证只核对证据和收尾，不把失败用例改为通过。


</details>
