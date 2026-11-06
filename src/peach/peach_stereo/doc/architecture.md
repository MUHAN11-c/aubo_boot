# peach_stereo 架构

## 职责

PS800-E1 主机单图案立体 RGB-D 前端，与 percipio 设备端前端互斥。

## 源码地图

`src/stereo_camera_node.cpp`：SDK 打开/采集、关节曝光窗、校正、SGBM、配准、滤波、发布及诊断；`src/packet_delay_transaction.hpp`：SDK 包间延时配置、原值保存、读回及回滚/退出恢复事务；`config/stereo_camera.yaml`：部署参数；`launch/stereo_camera.launch.py`：namespace、内参与重启策略。CMake/package.xml 声明 C++17、Jazzy/OpenCV 与 percipio SDK 依赖；仅注册官方 ament lint。

## 入口、输入输出与参数

`ros2 launch peach_stereo stereo_camera.launch.py`；整栈由 camera_enabled/camera_frontend 选择。发布 `/camera/color/image_raw`、`depth/image_raw`、两路 camera_info、`depth_registered/points` 及 `/diagnostics`。深度 uint16×0.25mm；点云按字段名读取 xyz/rgb/confidence。彩色 YUYV 解码后走与官方驱动相同的厂参 `TYUndistortImage`（棋盘格标定对着官方去畸变彩图）；`camera_info` 仍覆盖 `color_camera_info.yaml`。配准深度再用同一套彩色厂参做最近邻 remap，落到与彩图像素同一网格，避免只去畸变彩色后 RGB-D 错位；不用 TY C16 双线性，以免把 0 掺进空洞。去畸变失败不发该帧畸变原图。

## 生命周期、并发与失败

普通 C++ 节点，不进托管名单。采集/匹配是重路径；launch 在进程失败后延迟3秒 respawn。彩色模式、激光/曝光/标定、连续触发（`TRIGGER_MODE_OFF`）或 TIME_SYNC_HOST 配置失败拒启；enable 之后、StartCapture 之前 GET-modify-SET 触发为 `TRIGGER_MODE_OFF`（只改 mode，不清零 EX 其余字段）；核对 `STREAM_ASYNC_OFF`（RGB+IR 同组）。IR `ir_exposure=990` 与彩色 AE（每次打开重新打开，清除误钉 INT=100）保持产品曝光：百兆链路才会把 RGB+IR 组率打到 ~0.7 gps，千兆隔离取帧 990+AE 实测 13.6 gps（与 10-16/10-21 live_mock 12.0–13.8 gps 同档）。采集线程只做 SDK 取帧、曝光校验、RGB/双目 IR 克隆并立即归还 SDK 缓冲；容量为 1 的最新帧槽交给独立处理线程做 SGBM/发布。处理落后时替换尚未处理的旧帧，计入 `drop_processing_backlog`，不阻塞采集、不扩大队列。两路图像保持同组各自曝光 stamp，不改变下游 500 ms 新鲜度门。关闭时先停止并唤醒帧槽，join 采集/处理线程后再 StopCapture、清 SDK 队列和关闭设备；不在采集运行态调用 TYClearBufferQueue。线程异常同样经此收尾退出，供 launch 重启。正常关闭及打开设备、启动采集线程的构造异常路径均释放已打开设备并复位激光。两前端不可同时连接。点云按有效深度压紧后由 `PointCloud2Modifier::resize` 同步维护 width、row_step 与 data 长度。参数由部署 YAML 与 C++ 声明共同核对，裸 ros2 run 不自动加载部署 YAML。部署 hh4+半分辨率隔离台架约 13.7 gps；`harvest_system` 真实相机+mock 在不钉 OpenMP 时 `stack.log` 记 12.0–13.1 gps、drop=0（2026-10-16 `live_mock_20261016_183842`）。钉 `OMP_NUM_THREADS=1` 或他程序抢核可落到约 6 gps，仍 drop=0。SGBM 在 `minDisparity=0` 时不计算左图前 `num_disparities` 列（OpenCV 代价体，不是 IR 视场）；半分辨率 128 列配准到彩色后约 154 px 无深度，官方设备 18 图案深度同一机位仍能看到左袋。匹配前左缘补同宽黑边再裁回，只打开这条算法盲区（实机 xmin 154→45，与官方 ~48 同档）。官方套袋轮廓清晰是因为 18 图案只在袋/竿上出深度、玻璃门为空；单图案门上仍有散斑，把 uniqueness 从 6 拧到 10 抠不掉门，袋面反而出现空洞，故保持 6。不改走设备 18 图案（~1 fps）。单图案边缘锐度仍低于 18 图案，这是帧率换质量。近距质量、温升与长期稳定性需独立验收，历史帧率不是产品保证。

`stereo_health` 的 `rate_gps` 为最近两次诊断之间的采集组率，`publish_fps` 为同一窗口成功发出 RGB-D 组的帧率；配准失败计入 `drop_depth`，不递增 `published`。`last_publish_age_s` 使用主机单调时钟记录最近成功发布的帧龄，尚未发布为 -1；未发布或窗口发布率低于 10 FPS 时告警。`capture_age_ms` 是有效帧取回并校验时的源曝光年龄，`processing_queue_wait_ms` 是从 fetch 返回到处理开始的单调时钟耗时（含克隆与最新槽等待）；二者用于区分设备/传输和处理排队。计数与 FPS 均为本节点内部观测，不证明订阅端原图或完整感知达到 10 FPS；断流继续取帧的策略未改变。

`pipeline_*` 在同一成功发布组上拆分 SDK fetch、克隆归还、单槽等待、彩色处理/发布、IR校正、SGBM、空间中值、深度变换/均值、配准、时域滤波、深度发布和点云。采集字段随帧传递，处理完成后连同成功序号及两路源戳一次性保存，诊断用短 mutex 复制；图像计算不持锁。`pipeline_fetch_to_publish_ms` 只比较主机单调时钟，独立于相机epoch偏差；`pipeline_capture_age_ms` 仍含未知同步偏差。诊断每秒采样最近成功组，不提供逐帧全集，原独立atomic最近值也不能拼加成同帧时延。详细字段边界见[io](io.md)。此仪表不修改曝光、时间戳、匹配、滤波、容量或发布QoS。

空间有效中值保持3/5核及有效邻居的上中值规则：正值掩码腐蚀后，完整正值内部邻域使用OpenCV浮点中值；洞边和图像边界仍逐像素选取有效邻居，不使用复制边界代替截断邻域。非正中心保留原位模式；NaN中心且没有正值邻居时明确保留源位模式，避免旧空邻域的未初始化读取。正常有限非负输入与冻结原循环逐像素bitwise对照；特殊值另验证NaN/Inf/正负零和非连续ROI。实现不改变匹配、深度滤波语义或图像尺寸，实际收益由同场景分段测量确认。

本机SDK为源stamp声明微秒与同组采集时刻，公开API未规定该时刻对应曝光开始、结束或中点。`TIME_SYNC_READY` 不提供独立offset误差界；因此源年龄与主机单调分段分别报告，源stamp原样传递。

`packet_delay_us` 为只读启动参数，C++声明默认-1，不读写设备该属性；部署YAML为50。非负值在StartCapture之前验证SDK属性可读写、整数范围/步进并保存原值，设置后要求读回一致。写入前标记恢复责任，SDK部分写入失败或读回不符时立即尝试回滚，仍拒启；回滚失败保留待恢复状态，由构造异常close在Stop/Clear后再次尝试。正常退出同样在Stop后、CloseDevice前恢复实际捕获的原值并读回，原值未知不猜工厂值。恢复失败输出ERROR和各返回码，不伪报成功；重复close不重复写已恢复属性。恢复字段与配置字段经短mutex提供诊断；关闭阶段主要依靠日志，不能保证关机后订阅端还能收到最后诊断。

部署50的依据限于2026-11-05本机千兆单相机PS800-E1、4buffer、RGB640×480/双IR1280×960、IR990/彩色AE不变的100/50/100us独占SDK对照：稳态源龄均值304.371/127.599/295.548ms，主机单调到达13.484/15.024/13.513Hz，三轮120组完整、packet_lost=0、image_dropped=1且恢复读回100。源龄仍含未知时钟offset；完整帧与checksum不单独证明深度精度，多相机、不同网卡或网络需重新验收。SDK缓冲仍4个，包大小/重传/曝光/源戳及500ms新鲜度门不变。

## 多帧滤波的曝光时域

滑窗保留每帧配准深度 `cv::Mat` 的引用。中值写入独立输出矩阵，窗口中的历史帧可继续按原始值参与计算；
640×480 的 uint16 图每个入窗帧省去一次 614400 字节的像素复制。

保留 9/20 的 med3/hh4 与 9/21 的 K3 逐像素有效中值/置信度公式。`/joint_states` 标准订阅提供六关节源时间证据；处理线程只在整个融合窗的最早至当前 IR 曝光均有连续静止证据时使用 `avg_k`/`temporal_k`。每端由前后状态夹住、距离最多 0.1 s，窗内相邻状态间隔也最多 0.1 s；六关节速度及相邻位置差/源 dt 均不超过 0.03 rad/s。缺字段、NaN、运动、缺后侧样本、断流、时间回跳均视为未知/不静止。关节证据保留最多 5 s/8192 条，不做插值，也不以消息收到时刻的速度覆盖历史曝光。相机侧约 311 ms 源延迟仍按曝光查窗。

运动/未知时清空多帧深度和置信度，只发布当前单帧，静止证据恢复后从空窗重新攒 K3；不关闭空间 med3、不改 SGBM、曝光或匹配分辨率。`avg_k>1` 的全部贡献曝光也纳入同一检查。处理线程独占 cv::Mat 历史；executor 回调只在 mutex 下更新小型关节证据窗，图像计算不持该锁。固定相机+mock 夹具里 mock 关节变化仍保守关闭多帧融合，这是夹具与腕载相机产品的差异。

每个 color/IR component 必须有自身非零 stamp；HOST 同步写失败拒启；不等待 `TIME_SYNC_READY`（10-21 13.8 gps 路径只 SetEnum）。取帧时把 SDK 时间戳按微秒或 Unix 纳秒换成纳秒，纪元年龄须为 0–10 s（保留本机历史 harvest+RViz 送达 2.7–7.3 s 时的上限；`TY_INT_EXPOSURE_TIME` 未在本机头文件注明单位，不能将整数990称为990 ms）；左右 IR 允许同一帧组内最多 1 ms 之差。流内重复/回跳、未来时间、异步 IR 对均丢弃并清图像历史，禁止借另一流时间或以收到时刻伪造曝光。时间异常递增采集时域编号；编号随最新帧传给处理线程，即使中间帧被覆盖，也会先清空旧融合窗，时钟恢复后按新证据重新攒窗。`color_age_ms`/`depth_age_ms` 保留符号，不把负值抹成零。这些是应用级时间一致性检查，不是时钟精度认证。

## 验证与限制

`colcon test --packages-select peach_stereo --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
