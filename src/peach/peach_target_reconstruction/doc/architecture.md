# peach_target_reconstruction 架构

## 职责与入口

`TargetReconstructionNode` 的图名和 console script 均为 `peach_target_reconstruction_node`。节点可由独立 launch 或 `peach_harvester` brain 进程装配。它绑定一个目标，按同一会话收集 RGB-D、构云、ICP/TSDF、精化袋几何并生成模型结果；本包不发运动，也不负责选择下一颗执行目标。

## 源码地图

- `target_reconstruction_node.py`：Lifecycle、RGB-D/观测/关节订阅、精确时刻 TF、采帧三段门禁、构云提交、Build action、取消和调试服务。
- `reconstruction_core.py`：collector/帧环/掩膜/产物共享状态、自动绑定与采集、finalize、心跳、消息发布和冻结有效窗。它是依赖宿主 ROS 实体的 mixin，不能把它当作零 ROS 算法库。
- `session.py`：已解码 RGB-D 的深度归一化、内参/尺寸门与柱/球 refitter 装配；`capture.py`：曝光关节速度证据、采帧/掩膜门、绑定防抖、FrameCollector 和耗时统计。
- `integrate.py`：Open3D 云、ROI/有效深度、有界 ICP、ICP target 增量缓存、TSDF、重叠与视角覆盖统计。
- `refine.py`：候选/轴 hint、目标类型记忆、柱/球 refit、袋关键点融合、走廊/剪切平面与预抓取误差；`refit_orchestrator.py` 串联 refit、关键点、视角支持与预算。实现名表仅在 refine.py。
- `publish.py`：发布节流、诊断、许可、TargetModel、PregraspVerification、点云与 Marker 消息投影；`session_recorder.py`：session/geometry、PLY、图像和元数据落盘。
- `params.py` 包装 GPL listener 并补充整数下限/视角预算校验；`config/target_reconstruction_parameters.yaml` 是生成 schema，`target_reconstruction.yaml` 是部署覆盖值。`setup.py` 生成参数模块、安装入口/launch/config；package.xml、resource 和 setup.cfg 提供依赖及 ament 元数据。

## 采集、融合与模型

1. 节点 Active 后同步 RGB、深度和 CameraInfo，worker 容量 3，满时拒绝新帧。归一化及校验通过后进入有界曝光帧环：最多 96 帧、192 MiB 图像数组，时间窗取 `capture.max_frame_age_s`。
2. 自动模式根据候选/调度 selected 绑定；显式 BuildTargetModel 可直接绑定目标，且在 auto_mode=false 时仍可请求采帧。绑定切换通过 holdoff，reset/换目标使采集世代失效。已绑定候选的有限轴 hint/中心更新不以 ACCEPT 为必要条件；这只是重建初值。
3. 对目标掩膜先精确同戳、再在容差内最近邻配对。StrictMaskGate 检查可用性、像素数、有效深度、目标漂移和可选邻锚间距；不能用相邻目标的掩膜补采。
4. 公共采帧门检查帧上限、缓存、掩膜、唯一曝光、帧龄、曝光前后关节静止证据和 frame_id。TF 一律查询该深度 stamp 的 `base ← camera`，失败跳帧，无 latest TF 降级。部署默认要求静止，并非默认连续运动积分。
5. 构云与有界 ICP 在状态锁外计算；提交时复核目标、采集世代、状态和曝光新旧，再积分 TSDF。旧会话计算结果不得更新新会话缓存、统计或产物。ICP target 在全量提取之间按修正量自适应复用增量云。
6. finalize 检查最少独立机位，再提取 TSDF/网格，做 refit 与袋关键点融合；Build 在等待阶段还检查配置的角基线和平均深度覆盖，直接 Trigger finalize 不额外执行这些 Build 等待门。积分帧数和独立机位数分开报告。Build 达标后可自行 finalize，`auto_finalize_at_max` 只控制自动采集满栈收口。
7. 发布精化几何、预抓取观测和 GraspDecision；完整 TargetModel 随 Build 结果返回。预算数值与能力布尔值的实际派生见 [共核接口](../../peach_vision_core/doc/io.md)，不能把诊断余量自行改述成执行门。

## 生命周期与并发

configure 创建订阅、服务、action、TF、worker 和心跳；Active 才积分或受理会改状态的请求，query 只读。RLock 保护 collector、TSDF、目标与产物；阻塞 TF 和构云/ICP 放在锁外，返回后复核同一世代。1 Hz 心跳尝试重发状态、类型化/JSON 诊断、许可及预抓取观测；状态锁忙时跳过本次并累计 heartbeat_skipped_busy，不能承诺持续 1 Hz 出口。同一 model_revision 的 generated_at/valid_until 在首次发布冻结，不随心跳续签。

大点云/Marker 按版本和最小间隔节流；预览优先 TSDF，无 TSDF 时才使用采集云。Build 在计算、消息组装、逐条发布及成功提交前复核取消/整次期限，迟到结果清空并清屏；这不抢占正在执行的数值库调用，DDS 多话题发布也不提供原子提交。

cleanup/shutdown 先停止采帧 worker、唤醒 Build 等待再拆 ROS 实体，诊断 Updater 复用。阶段耗时使用单调时钟；会话与许可使用节点注入时钟，两种用途不能混加。健康计时器与 HarvestState 订阅各用独立互斥回调组；这与 action/心跳的可重入组分开，但线程栈采样仍依赖 executor/GIL 能调度。

独立节点入口使用公开的 `spin_once`：Build 占槽、COLLECTING 或数值阶段活跃时以 1 ms 为等待预算；其余 IDLE/READY 时等待上限为 50 ms，消息/事件就绪可提前唤醒，并非固定睡眠。活动快照无锁读取，每轮后仍 `time.sleep(0)` 让出 GIL。锁等待占住线程池时，Jazzy/Fast DDS 可连续返回没有就绪实体的 wait-set；短轮次与显式让出、非阻塞心跳及互斥状态镜像共同限制数值线程被挤占的放大路径。预算不等于回调或重建的硬实时上限，不抢占数值库，不改几何、质量门或视觉预抓取。

曝光关节历史使用 `ExposureJointHistory`，最多保留 8192 条整数纳秒时间戳与已解码速度。查询以二分取曝光前最后一条、曝光后第一条（均含相等），避免在状态锁内线性扫描全部 ROS Time 对象。重复时间戳保留首/尾各自证据；时间回退或乱序仍清空旧域。速度缺失、非有限值、任一侧证据缺失或超出原容差仍拒绝，禁止回退最新关节状态。

## 验证范围

`test/` 只有 ament flake8/pep257 包装。离线回放、mock、模型身份和现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)。纯 lint 不证明融合正确、取消时延或真实剪切成功。
